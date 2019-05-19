
#if defined(LINKS_WITH_EXTERN_OPENCV)
#include <opencv2/opencv.hpp>
#else
#include "sl_core/opencv/cv_wrapper.hpp"
using namespace slutils;
#endif

#include "sl_core/ai/skeleton/tracking/tracker3d.h"

namespace zed_tracking {

    Tracker3D::Tracker3D(
            double gate_distance,
            bool detector_likelihood,
            std::vector<double> likelihood_weights,
            bool velocity_in_motion_term,
            double min_confidence,
            double min_confidence_detections,
            double sec_before_old,
            double sec_before_fake,
            double sec_remain_new,
            int detections_to_validate,
            double period,
            double position_variance,
            double acceleration_variance,
            std::string world_frame_id,
            bool debug_mode,
            bool vertical) :
    gate_distance_(gate_distance),
    detector_likelihood_(detector_likelihood),
    likelihood_weights_(likelihood_weights),
    velocity_in_motion_term_(velocity_in_motion_term),
    min_confidence_(min_confidence),
    min_confidence_detections_(min_confidence_detections),
    sec_before_old_(sec_before_old),
    sec_before_fake_(sec_before_fake),
    sec_remain_new_(sec_remain_new),
    detections_to_validate_(detections_to_validate),
    period_(period),
    position_variance_(position_variance),
    acceleration_variance_(acceleration_variance),
    world_frame_id_(world_frame_id),
    debug_mode_(debug_mode),
    vertical_(vertical) {
        tracks_counter_ = 0;
    }

    Tracker3D::~Tracker3D() {

    }

void
    Tracker3D::newFrame(const std::vector<zed_tracking::Detection>& detections) {
        detections_.clear();
        unassociated_detections_.clear();
        lost_tracks_.clear();
        new_tracks_.clear();
        detections_ = detections;

        struct timeval current_detections_time = detections_[0].getTime();

        for (std::list<zed_tracking::Track3D*>::iterator it = tracks_.begin(); it != tracks_.end();) {
            zed_tracking::Track3D* t = *it;
            bool deleted = false;

            if (((t->getVisibility() == zed_tracking::Track3D::NOT_VISIBLE && (t->getSecFromLastHighConfidenceDetection(current_detections_time)) >= sec_before_old_)
                    || (!t->isValidated() && t->getSecFromFirstDetection(current_detections_time) >= sec_before_fake_))) {
                if (debug_mode_) {
                    std::cout << "Track " << t->getId() << " DELETED" << std::endl;
                }
                delete t;
                it = tracks_.erase(it);
                deleted = true;
            } else if (!t->isValidated() && t->getUpdatesWithEnoughConfidence() == detections_to_validate_) {
                t->validate();
                if (debug_mode_) {
                    std::cout << "Track " << t->getId() << " VALIDATED" << std::endl;
                }
            } else if (t->getStatus() == zed_tracking::Track3D::NEW && t->getSecFromFirstDetection(current_detections_time) >= sec_remain_new_) {
                t->setStatus(zed_tracking::Track3D::NORMAL);
                if (debug_mode_) {
                    std::cout << "Track " << t->getId() << " set to NORMAL" << std::endl;
                }
            }

            if (!deleted) {
                if (t->getStatus() == zed_tracking::Track3D::NEW && t->getVisibility() == Track3D::VISIBLE)
                    new_tracks_.push_back(t);
                if (t->getVisibility() == zed_tracking::Track3D::NOT_VISIBLE)
                    lost_tracks_.push_back(t);
                it++;
            }

        }
    }

void
    Tracker3D::updateTracks() {
        createDistanceMatrix();
        createCostMatrix();

        // Solve Global Nearest Neighbor problem:
        zed_tracking::Munkres munkres;
        cost_matrix_ = munkres.solve(cost_matrix_, false); // rows: targets (tracks), cols: detections

        updateDetectedTracks();
        fillUnassociatedDetections();
        updateLostTracks();
        createNewTracks();
    }

    /************************ protected methods ************************/

    int
    Tracker3D::createNewTrack(zed_tracking::Detection& detection) {
        if (detection.getConfidence() < min_confidence_)
            return -1;

        zed_tracking::Track3D* t;
        t = new Track3D(
                ++tracks_counter_,
                world_frame_id_,
                position_variance_,
                acceleration_variance_,
                period_,
                velocity_in_motion_term_);

        t->init(detection.getWorldCentroid()(0), detection.getWorldCentroid()(1), detection.getWorldCentroid()(2), detection.getHeight(), detection.getDistance(), detection.getTime());

        bool first_update = true;
        t->update(detection.getWorldCentroid()(0), detection.getWorldCentroid()(1),
                detection.getWorldCentroid()(2), detection.getHeight(), detection.getDistance(), 0.0,
                detection.getConfidence(), min_confidence_, min_confidence_detections_,
                detection.getTime(), first_update);

        std::cout << "Created track with ID: " << t->getId() << std::endl;

        tracks_.push_back(t);
        return tracks_counter_;
    }

void
    Tracker3D::createDistanceMatrix() {
        distance_matrix_ = cv::Mat_<double>(tracks_.size(), detections_.size());
        int track = 0;
        for (std::list<zed_tracking::Track3D*>::const_iterator it = tracks_.begin(),
                end = tracks_.end(); it != end; it++) {
            zed_tracking::Track3D* t = *it;
            int measure = 0;
            for (std::vector<zed_tracking::Detection>::iterator dit = detections_.begin(); dit != detections_.end(); dit++) {
                double detector_likelihood;

                // Compute detector likelihood:
                if (detector_likelihood_) {
                    detector_likelihood = dit->getConfidence();
                } else {
                    detector_likelihood = 0;
                }

                // Compute motion likelihood:
                double motion_likelihood = t->getMahalanobisDistance(
                        dit->getWorldCentroid()(0),
                        dit->getWorldCentroid()(1),
                        dit->getWorldCentroid()(2),
                        dit->getTime());

                // Compute joint likelihood and put it in the distance matrix:

                distance_matrix_(track, measure++) = likelihood_weights_[0] * detector_likelihood + likelihood_weights_[1] * motion_likelihood;

                // Remove NaN and inf:
                if (std::isnan(distance_matrix_(track, measure - 1)) | (not std::isfinite(distance_matrix_(track, measure - 1))))
                    distance_matrix_(track, measure - 1) = 2 * gate_distance_;


            }
            track++;
        }

    }

void
    Tracker3D::createCostMatrix() {
        cost_matrix_ = distance_matrix_.clone();
        for (int i = 0; i < distance_matrix_.rows; i++) {
            for (int j = 0; j < distance_matrix_.cols; j++) {
                if (distance_matrix_(i, j) > gate_distance_)
                    cost_matrix_(i, j) = 1000000.0;
            }
        }

    }

void
    Tracker3D::updateDetectedTracks() {

        // Iterate over every track:
        int track = 0;
        for (std::list<zed_tracking::Track3D*>::iterator it = tracks_.begin(); it != tracks_.end(); it++) {
            bool updated = false;
            zed_tracking::Track3D* t = *it;

            for (int measure = 0; measure < cost_matrix_.cols; measure++) {
                // If a detection<->track association has been found:
                if (cost_matrix_(track, measure) == 0.0 && distance_matrix_(track, measure) <= gate_distance_) {
                    zed_tracking::Detection& d = detections_[measure];

                    if ((t->getLowConfidenceConsecutiveFrames() < 10) || (d.getConfidence() > ((min_confidence_ + min_confidence_detections_) / 2))) {
                        // Update track with the associated detection:
                        bool first_update = false;
                        t->update(d.getWorldCentroid()(0), d.getWorldCentroid()(1),
                                d.getWorldCentroid()(2), d.getHeight(),
                                d.getDistance(), distance_matrix_(track, measure),
                                d.getConfidence(), min_confidence_, min_confidence_detections_,
                                d.getTime(), first_update);

                        t->setVisibility(d.isOccluded() ? zed_tracking::Track3D::OCCLUDED : zed_tracking::Track3D::VISIBLE);
                        updated = true;
                        break;
                    } else {
                        std::cout << "Id: " << t->getId() << ", lowConfConsFrames: " << t->getLowConfidenceConsecutiveFrames() << ", newConf: " << d.getConfidence() << std::endl;
                    }
                }
            }
            if (!updated) {
                if (t->getVisibility() != zed_tracking::Track3D::NOT_VISIBLE) {
                    t->setVisibility(zed_tracking::Track3D::NOT_VISIBLE);
                }
            }
            track++;
        }
    }

void
    Tracker3D::fillUnassociatedDetections() {
        // Fill a list with detections not associated to any track:
        if (cost_matrix_.cols == 0 && detections_.size() > 0) {
            for (size_t measure = 0; measure < detections_.size(); measure++)
                unassociated_detections_.push_back(detections_[measure]);
        } else {
            for (int measure = 0; measure < cost_matrix_.cols; measure++) {
                bool associated = false;
                for (int track = 0; track < cost_matrix_.rows; track++) {
                    if (cost_matrix_(track, measure) == 0.0) {
                        if (distance_matrix_(track, measure) > gate_distance_)
                            break;
                        associated = true;
                    }
                }
                if (!associated && detections_[measure].getConfidence() > min_confidence_) {
                    unassociated_detections_.push_back(detections_[measure]);
                }
            }
        }
    }

void
    Tracker3D::updateLostTracks() {

    }

void
    Tracker3D::createNewTracks() {
        for (std::list<zed_tracking::Detection>::iterator dit = unassociated_detections_.begin();
                dit != unassociated_detections_.end(); dit++) {
            createNewTrack(*dit);
        }
    }

void
    Tracker3D::setMinConfidenceForTrackInitialization(double min_confidence) {
        min_confidence_ = min_confidence;
    }

void
    Tracker3D::setSecBeforeOld(double sec_before_old) {
        sec_before_old_ = sec_before_old;
    }

void
    Tracker3D::setSecBeforeFake(double sec_before_fake) {
        sec_before_fake_ = sec_before_fake;
    }

void
    Tracker3D::setSecRemainNew(double sec_remain_new) {
        sec_remain_new_ = sec_remain_new;
    }

void
    Tracker3D::setDetectionsToValidate(int detections_to_validate) {
        detections_to_validate_ = detections_to_validate;
    }

void
    Tracker3D::setDetectorLikelihood(bool detector_likelihood) {
        detector_likelihood_ = detector_likelihood;
    }

void
    Tracker3D::setLikelihoodWeights(double detector_weight, double motion_weight) {
        likelihood_weights_[0] = detector_weight;
        likelihood_weights_[1] = motion_weight;
    }

void
    Tracker3D::setVelocityInMotionTerm(bool velocity_in_motion_term, double acceleration_variance, double position_variance) {
        velocity_in_motion_term_ = velocity_in_motion_term;
        acceleration_variance_ = acceleration_variance;
        position_variance_ = position_variance;

        // Update all existing tracks:
        for (std::list<zed_tracking::Track3D*>::iterator it = tracks_.begin(); it != tracks_.end(); it++) {
            zed_tracking::Track3D* t = *it;
            t->setVelocityInMotionTerm(velocity_in_motion_term_, acceleration_variance_, position_variance_);
        }
    }

void
    Tracker3D::setAccelerationVariance(double acceleration_variance) {
        acceleration_variance_ = acceleration_variance;

        // Update all existing tracks:
        for (std::list<zed_tracking::Track3D*>::iterator it = tracks_.begin(); it != tracks_.end(); it++) {
            zed_tracking::Track3D* t = *it;
            t->setAccelerationVariance(acceleration_variance_);
        }
    }

void
    Tracker3D::setPositionVariance(double position_variance) {
        position_variance_ = position_variance;

        // Update all existing tracks:
        for (std::list<zed_tracking::Track3D*>::iterator it = tracks_.begin(); it != tracks_.end(); it++) {
            zed_tracking::Track3D* t = *it;
            t->setPositionVariance(position_variance_);
        }
    }

void
    Tracker3D::setGateDistance(double gate_distance) {
        gate_distance_ = gate_distance;
    }

} /*namespace zed_tracking*/
