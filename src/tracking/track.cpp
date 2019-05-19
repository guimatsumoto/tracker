#include "sl_core/ai/skeleton/tracking/track.h"

namespace zed_tracking {

    Track::Track(int id, std::string frame_id, double position_variance, double acceleration_variance,
            double period, bool velocity_in_motion_term) :
    id_(id), frame_id_(frame_id),
    period_(period), velocity_in_motion_term_(velocity_in_motion_term) {
        color_ = Eigen::Vector3f(
                float(rand() % 256) / 255,
                float(rand() % 256) / 255,
                float(rand() % 256) / 255);

        MAX_SIZE = 90;
        if (velocity_in_motion_term) {
            filter_ = new zed_tracking::KalmanFilter(period, position_variance, acceleration_variance, 4);
            tmp_filter_ = new zed_tracking::KalmanFilter(period, position_variance, acceleration_variance, 4);
            mahalanobis_map4d_.resize(MAX_SIZE, MahalanobisParameters4d());
        } else {
            filter_ = new zed_tracking::KalmanFilter(period, position_variance, acceleration_variance, 2);
            tmp_filter_ = new zed_tracking::KalmanFilter(period, position_variance, acceleration_variance, 2);
            mahalanobis_map2d_.resize(MAX_SIZE, MahalanobisParameters2d());
        }
    }

    Track::~Track() {
        delete filter_;
        delete tmp_filter_;
    }

void
    Track::init(const zed_tracking::Track& old_track) {
        double x, y;
        old_track.filter_->getState(x, y);

        filter_->init(x, y, 10, old_track.velocity_in_motion_term_);

        *tmp_filter_ = *filter_;
        visibility_ = old_track.visibility_;

        std::cout << "Old track id: " << old_track.id_ << std::endl;
        std::cout << "New track id: " << id_ << std::endl;

        z_ = old_track.z_;
        z_previous = 0.0/0.0;
        height_ = old_track.height_;
        distance_ = old_track.distance_;
        age_ = old_track.age_;

        velocity_in_motion_term_ = old_track.velocity_in_motion_term_;
        validated_ = old_track.validated_ || validated_;
        low_confidence_consecutive_frames_ = old_track.low_confidence_consecutive_frames_;

        first_time_detected_ = old_track.first_time_detected_;
        last_time_detected_ = old_track.first_time_detected_;
        last_time_detected_with_high_confidence_ = old_track.first_time_detected_;
        last_time_predicted_ = old_track.last_time_predicted_;
        last_time_predicted_index_ = old_track.last_time_predicted_index_;

        data_association_score_ = old_track.data_association_score_;
    }

    void
    Track::init(double x, double y, double z, double height, double distance,
            struct timeval detection_time) {
        filter_->init(x, y, distance, velocity_in_motion_term_);
        z_ = z;
        height_ = height;
        distance_ = distance;
        status_ = NEW;
        visibility_ = VISIBLE;
        validated_ = false;
        updates_with_enough_confidence_ = low_confidence_consecutive_frames_ = 0;
        first_time_detected_ = detection_time;
        last_time_predicted_ = last_time_detected_ = last_time_detected_with_high_confidence_ = detection_time;
        last_time_predicted_index_ = 0;
        last_detector_confidence_ = 0;
        age_ = 0.0;
    }

    void
    Track::update(double x, double y, double z, double height, double distance,
            double data_association_score, double confidence, double min_confidence,
            double min_confidence_detections, struct timeval detection_time,
            bool first_update) {
        int difference;
        double vx, vy;
        if (velocity_in_motion_term_) {
            // We will use durations as follows
            // d = 1 second
            // d2 = 2 seconds

            /*Now using milisecond precision with timeval*/
            struct timeval t2 = detection_time;
            t2.tv_sec -= 1;
            struct timeval t = diff_ms(first_time_detected_, t2) >= 0 ? first_time_detected_ : t2;

            t = diff_ms(t, last_time_detected_) >= 0 ? last_time_detected_ : t;

            t2 = detection_time;
            t2.tv_sec -= 2;
            t = diff_ms(t, t2) >= 0 ? t : t2;

            double dt = diff_s(t, last_time_predicted_);

            // Calculating how many frames we have skipped
            difference = int(round(dt / period_));

            int vIndex = (MAX_SIZE + last_time_predicted_index_ + difference) % MAX_SIZE;

            if (difference != 0) {
                vx = -(x - mahalanobis_map4d_[vIndex].x) / dt;
                vy = -(y - mahalanobis_map4d_[vIndex].y) / dt;
            } else {
                vx = mahalanobis_map4d_[vIndex].x;
                vy = mahalanobis_map4d_[vIndex].y;
            }
        }

        int framesLost = int( round(diff_s(detection_time,
                last_time_detected_) / period_)) - 1;

        for (int i = 0; i < framesLost; i++) {
            filter_->predict();
            filter_->update();
        }

        filter_->predict();

        if (velocity_in_motion_term_) {
            filter_->update(x, y, vx, vy, distance);
        } else {
            filter_->update(x, y, distance);
        }

        *tmp_filter_ = *filter_;

        difference = int( round(diff_s(detection_time, last_time_predicted_) / period_));
        last_time_predicted_index_ = (MAX_SIZE + last_time_predicted_index_ + difference) % MAX_SIZE;
        last_time_predicted_ = last_time_detected_ = detection_time;

        if (velocity_in_motion_term_)
            filter_->getMahalanobisParameters(mahalanobis_map4d_[last_time_predicted_index_]);
        else
            filter_->getMahalanobisParameters(mahalanobis_map2d_[last_time_predicted_index_]);

        // Updating z_ and height_ with a weighted combination of current and new values
        z_previous = z_;
        z_ = z_ * 0.9 + z * 0.1;
        height_ = height_ * 0.9 + height * 0.1;
        distance_ = distance;

        if (confidence > min_confidence) {
            updates_with_enough_confidence_++;
            last_time_detected_with_high_confidence_ = last_time_detected_;
        }

        if (first_update)
        {
            if (confidence < (min_confidence + min_confidence_detections) / 2) {
                low_confidence_consecutive_frames_++;
            } else {
                low_confidence_consecutive_frames_ = 0;
            }
        }
        else
        {
            if ((confidence < (min_confidence + min_confidence_detections) / 2) && (last_detector_confidence_ < (min_confidence + min_confidence_detections) / 2)) {
                low_confidence_consecutive_frames_++;
            } else {
                low_confidence_consecutive_frames_ = 0;
            }
        }

        last_detector_confidence_ = confidence;

        data_association_score_ = data_association_score;

        age_ = diff_s(detection_time, first_time_detected_);

    }

void
    Track::validate() {
        validated_ = true;
    }

bool
    Track::isValidated() {
        return validated_;
    }

int
    Track::getId() {
        return id_;
    }

void
    Track::setStatus(Track::Status s) {
        status_ = s;
    }

    Track::Status
    Track::getStatus() {
        return status_;
    }

void
    Track::setVisibility(Track::Visibility v) {
        visibility_ = v;
    }

    Track::Visibility
    Track::getVisibility() {
        return visibility_;
    }

float
    Track::getSecFromFirstDetection(struct timeval current_time) {
        return (float) (diff_s(current_time, first_time_detected_));
    }

float
    Track::getSecFromLastDetection(struct timeval current_time) {
        return (float) (diff_s(current_time, last_time_detected_));
    }

    float
    Track::getSecFromLastHighConfidenceDetection(struct timeval current_time) {
        return (float) (diff_s(current_time, last_time_detected_with_high_confidence_));
    }

float
    Track::getLowConfidenceConsecutiveFrames() {
        return low_confidence_consecutive_frames_;
    }

int
    Track::getUpdatesWithEnoughConfidence() {
        return updates_with_enough_confidence_;
    }

double
    Track::getMahalanobisDistance(double x, double y, const struct timeval& when) {
        int difference = int(round(diff_s(when, last_time_predicted_) / period_));

        int index;
        if (difference <= 0) {
            index = (MAX_SIZE + last_time_predicted_index_ + difference) % MAX_SIZE;
        } else {
            for (int i = 0; i < difference; i++) {
                tmp_filter_->predict();
                last_time_predicted_index_ = (last_time_predicted_index_ + 1) % MAX_SIZE;
                if (velocity_in_motion_term_)
                    tmp_filter_->getMahalanobisParameters(mahalanobis_map4d_[last_time_predicted_index_]);
                else
                    tmp_filter_->getMahalanobisParameters(mahalanobis_map2d_[last_time_predicted_index_]);
                tmp_filter_->update();
            }
            last_time_predicted_ = when;
            index = last_time_predicted_index_;
        }

        if (velocity_in_motion_term_) {
            // Using d = 1 second and d2 = 2 seconds
            timeval t2 = when;
            t2.tv_sec -= 1;
            struct timeval t = diff_s(first_time_detected_, t2) >= 0 ? first_time_detected_ : t2;

            t = diff_s(t, last_time_detected_) >= 0 ? last_time_detected_ : t;

            t2 = last_time_predicted_;
            t2.tv_sec -= 2;
            t = diff_s(t, t2) >= 0 ? t : t2;

            double dt = diff_s(t, last_time_predicted_);

            difference = int(round(dt / period_));
            int vIndex = (MAX_SIZE + last_time_predicted_index_ + difference) % MAX_SIZE;

            double vx, vy;
            if (difference != 0) {
                vx = -(x - mahalanobis_map4d_[vIndex].x) / dt;
                vy = -(y - mahalanobis_map4d_[vIndex].y) / dt;
            } else {
                vx = mahalanobis_map4d_[vIndex].x;
                vy = mahalanobis_map4d_[vIndex].y;
            }

            return zed_tracking::KalmanFilter::performMahalanobisDistance(x, y, vx, vy, mahalanobis_map4d_[index]);
        } else {
            return zed_tracking::KalmanFilter::performMahalanobisDistance(x, y, mahalanobis_map2d_[index]);
        }

    }

void
    Track::setVelocityInMotionTerm(bool velocity_in_motion_term, double acceleration_variance, double position_variance) {
        velocity_in_motion_term_ = velocity_in_motion_term;

        // Re-initialize Kalman filter
        filter_->setPredictModel(acceleration_variance);
        filter_->setObserveModel(position_variance);
        double x, y;
        filter_->getState(x, y);
        filter_->init(x, y, distance_, velocity_in_motion_term_);

        *tmp_filter_ = *filter_;
    }

void
    Track::setAccelerationVariance(double acceleration_variance) {
        filter_->setPredictModel(acceleration_variance);
        tmp_filter_->setPredictModel(acceleration_variance);
    }

void
    Track::setPositionVariance(double position_variance) {
        filter_->setObserveModel(position_variance);
        tmp_filter_->setObserveModel(position_variance);
    }

sl::float3
    Track::getSpeed(){
        double x, y, vx, vy;
        filter_->getState(x, y, vx, vy);
        float y_speed = std::isfinite(z_previous) ? (float)(z_ - z_previous)/period_ : NAN;
        return sl::float3((float)vx, (float)vy, y_speed);
    }

} /*namespace zed_tracking*/
