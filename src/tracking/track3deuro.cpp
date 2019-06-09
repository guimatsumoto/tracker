#include "tracking/track3deuro.h"

namespace tracker {

    Track3DEuro::Track3DEuro(
            int id,
            std::string frame_id,
            double period,
            double mincutoff,
            double beta,
            double dcutoff) :
    id_(id),
    frame_id_(frame_id),
    period_(period) {
        color_ = Eigen::Vector3f(
                float(rand() % 256) / 255,
                float(rand() % 256) / 255,
                float(rand() % 256) / 255);

        MAX_SIZE = 90; //XXX create a parameter!!!
        filter_ = new tracker::OneEuroFilter3D(1 / period, mincutoff, beta, dcutoff);
        tmp_filter_ = new tracker::OneEuroFilter3D(1 / period, mincutoff, beta, dcutoff);
    }

    Track3DEuro::~Track3DEuro() {
        delete filter_;
        delete tmp_filter_;
    }

void
    Track3DEuro::init(const tracker::Track3DEuro& old_track) {
        double x, y, z;
        old_track.filter_->getState(x, y, z);

        /* TODO: Implement the reference constructor
        filter_->init(x, y, z, 10, old_track.velocity_in_motion_term_);
         */

        *tmp_filter_ = *filter_;
        visibility_ = old_track.visibility_;

        distance_ = old_track.distance_;
        age_ = old_track.age_;

        validated_ = validated_ || old_track.validated_;
        low_confidence_consecutive_frames_ = old_track.low_confidence_consecutive_frames_;

        first_time_detected_ = old_track.first_time_detected_;
        last_time_detected_ = old_track.last_time_detected_;
        last_time_detected_with_high_confidence_ = old_track.last_time_detected_with_high_confidence_;
        last_time_predicted_ = old_track.last_time_predicted_;
        last_time_predicted_index_ = old_track.last_time_predicted_index_;

        data_association_score_ = old_track.data_association_score_;
    }

    void
    Track3DEuro::init(double x, double y, double z, double height, double distance,
            struct timeval detection_time) {

        distance_ = distance;
        status_ = NEW;
        visibility_ = VISIBLE;
        validated_ = false;
        updates_with_enough_confidence_ = low_confidence_consecutive_frames_ = 0;
        first_time_detected_ = detection_time;
        last_time_predicted_ = last_time_detected_ = last_time_detected_with_high_confidence_ = detection_time;
        last_time_predicted_index_ = 0;
        age_ = 0.0;
        last_detector_confidence_ = 0;
        double time_in_sec = ((detection_time.tv_sec * (double) 1000000) + (detection_time.tv_usec)) / 1000000;
        filter_->update(x, y, z, time_in_sec, false);
    }

    void
    Track3DEuro::update(
            double x,
            double y,
            double z,
            double height,
            double distance,
            double data_assocation_score,
            double confidence,
            double min_confidence,
            double min_confidence_detections,
            struct timeval detection_time,
            bool first_update) {

        double time_in_sec = ((detection_time.tv_sec * (double) 1000000) + (detection_time.tv_usec)) / 1000000;


        if (std::isnan(x) or std::isnan(y) or std::isnan(z)) {
            filter_->update(time_in_sec);
            *tmp_filter_ = *filter_;

            int difference = int(round(diff_s(detection_time, last_time_predicted_) / period_));
            last_time_predicted_index_ = (MAX_SIZE + last_time_predicted_index_ + difference) % MAX_SIZE;
            last_time_predicted_ = last_time_detected_ = detection_time;

            if (confidence > min_confidence) {
                updates_with_enough_confidence_++;
                last_time_detected_with_high_confidence_ = last_time_detected_;
            }

            if (!first_update)
            {
                if ((confidence < (min_confidence + min_confidence_detections) / 2) && (last_detector_confidence_ < (min_confidence + min_confidence_detections) / 2)) {
                    low_confidence_consecutive_frames_++;
                } else {
                    low_confidence_consecutive_frames_ = 0;
                }
            }
            else
            {
                if (confidence < (min_confidence + min_confidence_detections) / 2) {
                    low_confidence_consecutive_frames_++;
                } else {
                    low_confidence_consecutive_frames_ = 0;
                }
            }

            last_detector_confidence_ = confidence;

            data_association_score_ = data_assocation_score;

            // Compute track age:
            age_ = diff_s(detection_time, first_time_detected_);

            return;
        }
        visibility_ = VISIBLE;

        //Update One Euro Filter
        int difference;

        filter_->update(x, y, z, time_in_sec, false);

        *tmp_filter_ = *filter_;

        difference = int(round(diff_s(detection_time, last_time_predicted_) / period_));
        last_time_predicted_index_ = (MAX_SIZE + last_time_predicted_index_ + difference) % MAX_SIZE;
        last_time_predicted_ = last_time_detected_ = detection_time;

        // Update z_ and height_ with a weighted combination of current and new values:
        z_ = z_ * 0.9 + z * 0.1;
        height_ = height_ * 0.9 + height * 0.1;
        distance_ = distance;

        if (confidence > min_confidence) {
            updates_with_enough_confidence_++;
            last_time_detected_with_high_confidence_ = last_time_detected_;
        }

        if ((confidence < (min_confidence + min_confidence_detections) / 2) && (last_detector_confidence_ < (min_confidence + min_confidence_detections) / 2)) {
            low_confidence_consecutive_frames_++;
        } else {
            low_confidence_consecutive_frames_ = 0;
        }
        last_detector_confidence_ = confidence;

        data_association_score_ = data_assocation_score;

        // Compute track age:
        age_ = diff_s(detection_time, first_time_detected_);
    }

void
    Track3DEuro::validate() {
        validated_ = true;
    }

bool
    Track3DEuro::isValidated() {
        return validated_;
    }

int
    Track3DEuro::getId() {
        return id_;
    }

void
    Track3DEuro::setStatus(Track3DEuro::Status s) {
        status_ = s;
    }

    Track3DEuro::Status
    Track3DEuro::getStatus() {
        return status_;
    }

void
    Track3DEuro::setVisibility(Track3DEuro::Visibility v) {
        visibility_ = v;
    }

    Track3DEuro::Visibility
    Track3DEuro::getVisibility() {
        return visibility_;
    }

float
    Track3DEuro::getSecFromFirstDetection(struct timeval current_time) {
        return (float) diff_s(current_time, first_time_detected_);
    }

float
    Track3DEuro::getSecFromLastDetection(struct timeval current_time) {
        return (float) diff_s(current_time, last_time_detected_);
    }

float
    Track3DEuro::getSecFromLastHighConfidenceDetection(struct timeval current_time) {
        return (float) diff_s(current_time, last_time_detected_with_high_confidence_);
    }

float
    Track3DEuro::getLowConfidenceConsecutiveFrames() {
        return low_confidence_consecutive_frames_;
    }

int
    Track3DEuro::getUpdatesWithEnoughConfidence() {
        return updates_with_enough_confidence_;
    }

double
    Track3DEuro::getLastDetectorConfidence() {
        return last_detector_confidence_;
    }

void
    Track3DEuro::updateFilter() {
        // TODO: Verify that it should be implemented for the 1 euro filter
        // filter_->update();
    }

void
    Track3DEuro::getState(double& x, double& y, double& z) {
        filter_->getState(x, y, z);
    }

} /*namespace tracker*/
