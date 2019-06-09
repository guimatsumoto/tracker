#include <opencv2/opencv.hpp>
#include "tracking/tracker.h"

namespace tracker {

    Tracker::Tracker(double gate_distance, bool detector_likelihood,
            std::vector<double> likelihood_weights, bool velocity_in_motion_term, double min_confidence,
            double min_confidence_detections, double sec_before_old, double sec_before_fake,
            double sec_remain_new, int detections_to_validate, double period, double position_variance,
            double acceleration_variance, std::string world_frame_id, bool debug_mode) :

    gate_distance_(gate_distance), detector_likelihood_(detector_likelihood),
    likelihood_weights_(likelihood_weights), velocity_in_motion_term_(velocity_in_motion_term),
    min_confidence_(min_confidence), min_confidence_detections_(min_confidence_detections),
    sec_before_old_(sec_before_old), sec_before_fake_(sec_before_fake),
    sec_remain_new_(sec_remain_new), detections_to_validate_(detections_to_validate), period_(period),
    position_variance_(position_variance), acceleration_variance_(acceleration_variance),
    world_frame_id_(world_frame_id), debug_mode_(debug_mode) {
        tracks_counter_ = 0;
    }

    Tracker::~Tracker() {

    }

    /*Verify if the actual object to be passed is a Pose or if we need a Detection object*/
    void
    Tracker::newFrame(const std::vector<tracker::Detection>& detections) {
        detections_.clear();
        unassociated_detections_.clear();
        lost_tracks_.clear();
        new_tracks_.clear();
        detections_ = detections;

        struct timeval currentTime;
        gettimeofday(&currentTime, NULL);

        for (std::list<tracker::Track*>::iterator
            it = tracks_.begin(); it != tracks_.end();) {
            Track* t = *it;
            bool deleted = false;

            // If the track either became old or was never validated before it can be considered fake
            // the track is deleted
            if ((t->getVisibility() == tracker::Track::NOT_VISIBLE &&
                    (t->getSecFromLastHighConfidenceDetection(currentTime)) >= sec_before_old_)
                    || (!t->isValidated() && t->getSecFromFirstDetection(currentTime) >= sec_before_fake_)) {
                delete t;
                it = tracks_.erase(it);
                deleted = true;
                if (debug_mode_)
                    std::cout << "Track " << t->getId() << " has been deleted" << std::endl;
            }// If the track isn't yet validated, but it was updated, with confidence, a certain number
                // of times, the track is valdiated
            else if (!t->isValidated() && t->getUpdatesWithEnoughConfidence() == detections_to_validate_) {
                t->validate();
                if (debug_mode_)
                    std::cout << "Track " << t->getId() << " has been validated" << std::endl;
            }                // If the track is validated but still has status NEW, check if a certain amount of time
                // has passed since its first detection and if so set status to NORMAL
            else if (t->getStatus() == tracker::Track::NEW &&
                    t->getSecFromFirstDetection(currentTime) >= sec_remain_new_) {
                t->setStatus(Track::NORMAL);
                if (debug_mode_)
                    std::cout << "Track " << t->getId() << " set to normal status" << std::endl;

            }

            // If the track hasn't been deleted because it was too old or was considered fake
            if (!deleted) {
                // If it's NEW and VISIBLE
                if (t->getStatus() == tracker::Track::NEW && t->getVisibility() == Track::VISIBLE) {
                    new_tracks_.push_back(t);
                }
                // If it's NOT_VISIBLE
                if (t->getVisibility() == tracker::Track::NOT_VISIBLE) {
                    lost_tracks_.push_back(t);
                }
                it++;
            }

        }
    }

void
    Tracker::updateTracks() {
        createDistanceMatrix();
        createCostMatrix();

        tracker::Munkres munkres;
        cost_matrix_ = munkres.solve(cost_matrix_, false);

        updateDetectedTracks();
        fillUnassociatedDetections();
        updateLostTracks();
        createNewTracks();
    }

int
    Tracker::createNewTrack(tracker::Detection& detection) {
        if (detection.getConfidence() < min_confidence_)
            return -1;

        tracker::Track* t;
        t = new Track(++tracks_counter_, world_frame_id_, position_variance_,
                acceleration_variance_, period_, velocity_in_motion_term_);

        t->init(detection.getWorldCentroid()(0), detection.getWorldCentroid()(1),
                detection.getWorldCentroid()(2), detection.getHeight(), detection.getDistance(),
                detection.getTime());

        bool first_update = true;

        t->update(detection.getWorldCentroid()(0), detection.getWorldCentroid()(1),
                detection.getWorldCentroid()(2), detection.getHeight(), detection.getDistance(),
                0.0, detection.getConfidence(), min_confidence_, min_confidence_detections_,
                detection.getTime(), first_update);

        if (debug_mode_)
            std::cout << "Created Track with ID: " << t->getId() << std::endl;

        tracks_.push_back(t);
        return tracks_counter_;
    }

void
    Tracker::createDistanceMatrix() {
        // Creates a matrix representing the distance between active tracks 
        // and the current time detections
        distance_matrix_ = cv::Mat_<double>(tracks_.size(), detections_.size());

        int track = 0;
        for (std::list<tracker::Track*>::const_iterator it = tracks_.begin(),
                end = tracks_.end(); it != end; it++) {
            Track* t = *it;
            int measure = 0;
            for (std::vector<tracker::Detection>::iterator dit = detections_.begin(), dend = detections_.end();
                    dit != dend; dit++) {
                // Acount for detection confidence
                double detector_likelihood;
                if (detector_likelihood_) {
                    detector_likelihood = dit->getConfidence();
                } else {
                    detector_likelihood = 0;
                }

                // Account for motion likelihood
                double motion_likelihood = t->getMahalanobisDistance(
                        dit->getWorldCentroid()(0),
                        dit->getWorldCentroid()(1),
                        dit->getTime());

                // Compute joint likelihood
                distance_matrix_(track, measure++) = likelihood_weights_[0] * detector_likelihood +
                        likelihood_weights_[1] * motion_likelihood;
                // Clean NaN and infs
                if (std::isnan(distance_matrix_(track, measure - 1)) or
                        (not std::isfinite(distance_matrix_(track, measure - 1))))
                    distance_matrix_(track, measure - 1) = 2 * gate_distance_;
            }
            track++;
        }
    }

void
    Tracker::createCostMatrix() {
        cost_matrix_ = distance_matrix_.clone();
        for (int i = 0; i < distance_matrix_.rows; i++)
            for (int j = 0; j < distance_matrix_.cols; j++)
                if (distance_matrix_(i, j) > gate_distance_)
                    cost_matrix_(i, j) = 1000000.0;
    }

void
    Tracker::updateDetectedTracks() {
        int track = 0;
        for (std::list<tracker::Track*>::iterator it = tracks_.begin(); it != tracks_.end(); it++) {
            bool updated = false;
            Track* t = *it;

            for (int measure = 0; measure < cost_matrix_.cols; measure++) {
                if (cost_matrix_(track, measure) == 0.0 &&
                        distance_matrix_(track, measure) <= gate_distance_) {
                    tracker::Detection& d = detections_[measure];

                    if ((t->getLowConfidenceConsecutiveFrames() < 10) ||
                            (d.getConfidence() > ((min_confidence_ + min_confidence_detections_) / 2))) {
                        bool first_update = false;
                        t->update(d.getWorldCentroid()(0), d.getWorldCentroid()(1),
                                d.getWorldCentroid()(2), d.getHeight(), d.getDistance(),
                                distance_matrix_(track, measure), d.getConfidence(), min_confidence_,
                                min_confidence_detections_, d.getTime(), first_update);

                        t->setVisibility(d.isOccluded() ? Track::OCCLUDED : Track::VISIBLE);
                        updated = true;
                        break;
                    } else {
                        if (debug_mode_)
                            std::cout << "Track " << t->getId() << " has low confidence" << std::endl;
                    }
                }
            }
            if (!updated)
                if (t->getVisibility() != tracker::Track::NOT_VISIBLE)
                    t->setVisibility(tracker::Track::NOT_VISIBLE);
            track++;
        }

    }

void
    Tracker::fillUnassociatedDetections() {
        if (cost_matrix_.cols == 0 && detections_.size() > 0) {
            for (size_t measure = 0; measure < detections_.size(); measure++)
                unassociated_detections_.push_back(detections_[measure]);
        } else {
            for (int measure = 0; measure < cost_matrix_.cols; measure++) {
                bool associated = false;
                for (int track = 0; track < cost_matrix_.cols; track++) {
                    if (cost_matrix_(track, measure) == 0.0)
                        if (distance_matrix_(track, measure) > gate_distance_)
                            break;
                    associated = true;
                }
                if (!associated)
                    unassociated_detections_.push_back(detections_[measure]);
            }
        }
    }

void
    Tracker::updateLostTracks() {

    }

    void
    Tracker::createNewTracks() {
        for (std::list<tracker::Detection>::iterator dit = unassociated_detections_.begin();
                dit != unassociated_detections_.end(); dit++) {
            createNewTrack(*dit);
        }
    }

void
    Tracker::setMinConfidenceForTrackInitialization(double min_confidence) {
        min_confidence_ = min_confidence;
    }

void
    Tracker::setSecBeforeOld(double sec_before_old) {
        sec_before_old_ = sec_before_old;
    }

void
    Tracker::setSecBeforeFake(double sec_before_fake) {
        sec_before_fake_ = sec_before_fake;
    }

void
    Tracker::setSecRemainNew(double sec_remain_new) {
        sec_remain_new_ = sec_remain_new;
    }

void
    Tracker::setDetectionsToValidate(int detection_to_validate) {
        detections_to_validate_ = detection_to_validate;
    }

void
    Tracker::setDetectorLikelihood(bool detector_likelihood) {
        detector_likelihood_ = detector_likelihood;
    }

void
    Tracker::setLikelihoodWeights(double detector_weight, double motion_weight) {
        likelihood_weights_[0] = detector_weight;
        likelihood_weights_[1] = motion_weight;
    }

void
    Tracker::setVelocityInMotionTerm(bool velocity_in_motion_term, double acceleration_variance, double position_variance) {
        velocity_in_motion_term_ = velocity_in_motion_term;
        acceleration_variance_ = acceleration_variance;
        position_variance_ = position_variance;

        // Update all existing tracks:
        for (std::list<Track*>::iterator it = tracks_.begin();
                it != tracks_.end(); it++) {
            tracker::Track* t = *it;
            t->setVelocityInMotionTerm(velocity_in_motion_term_,
                    acceleration_variance_, position_variance_);
        }
    }

void
    Tracker::setAccelerationVariance(double acceleration_variance) {
        acceleration_variance_ = acceleration_variance;

        // Update each track as well
        for (std::list<tracker::Track*>::iterator it = tracks_.begin();
                it != tracks_.end(); it++) {
            tracker::Track* t = *it;
            t->setAccelerationVariance(acceleration_variance_);
        }
    }

void
    Tracker::setPositionVariance(double position_variance) {
        position_variance_ = position_variance;

        for (std::list<tracker::Track*>::iterator it = tracks_.begin();
                it != tracks_.end(); it++) {
            tracker::Track* t = *it;
            t->setPositionVariance(position_variance_);
        }
    }

    void
    Tracker::setGateDistance(double gate_distance) {
        gate_distance_ = gate_distance;
    }

} /*namespace tracker*/
