#include "tracking/track3d.h"

namespace tracker {

    Track3D::Track3D(
            int id,
            std::string frame_id,
            double position_variance,
            double acceleration_variance,
            double period,
            bool velocity_in_motion_term) :
    id_(id),
    frame_id_(frame_id),
    period_(period),
    velocity_in_motion_term_(velocity_in_motion_term) {
        color_ = Eigen::Vector3f(
                float(rand() % 256) / 255,
                float(rand() % 256) / 255,
                float(rand() % 256) / 255);

        MAX_SIZE = 90; //XXX create a parameter!!!
        if (velocity_in_motion_term_) {
            filter_ = new tracker::KalmanFilter3D(period, position_variance, acceleration_variance, 6);
            tmp_filter_ = new tracker::KalmanFilter3D(period, position_variance, acceleration_variance, 6);
            mahalanobis_map6d_.resize(MAX_SIZE, MahalanobisParameters6d());
        } else {
            filter_ = new tracker::KalmanFilter3D(period, position_variance, acceleration_variance, 3);
            tmp_filter_ = new tracker::KalmanFilter3D(period, position_variance, acceleration_variance, 3);
            mahalanobis_map3d_.resize(MAX_SIZE, MahalanobisParameters3d());
        }

    }

    Track3D::~Track3D() {
        delete filter_;
        delete tmp_filter_;
    }

void
    Track3D::init(const tracker::Track3D& old_track) {
        double x, y, z;
        old_track.filter_->getState(x, y, z);

        filter_->init(x, y, z, 10, old_track.velocity_in_motion_term_);

        *tmp_filter_ = *filter_;
        visibility_ = old_track.visibility_;

        distance_ = old_track.distance_;
        age_ = old_track.age_;

        velocity_in_motion_term_ = old_track.velocity_in_motion_term_;
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
    Track3D::init(double x, double y, double z, double height, double distance,
            struct timeval detection_time) {

        //Init Kalman filter
        filter_->init(x, y, z, distance, velocity_in_motion_term_);
        distance_ = distance;
        status_ = NEW;
        visibility_ = VISIBLE;
        validated_ = false;
        updates_with_enough_confidence_ = low_confidence_consecutive_frames_ = 0;
        first_time_detected_ = detection_time;
        last_time_predicted_ = last_time_detected_ = last_time_detected_with_high_confidence_ = detection_time;
        last_time_predicted_index_ = 0;
        age_ = 0.0;


    }

    void
    Track3D::update(
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
        if (std::isnan(x) or std::isnan(y) or std::isnan(z)) {
            if (first_update)
                visibility_ = NOT_VISIBLE;
            filter_->predict();
            filter_->update();

            *tmp_filter_ = *filter_;

            int difference = int(round(diff_s(detection_time, last_time_predicted_) / period_));
            last_time_predicted_index_ = (MAX_SIZE + last_time_predicted_index_ + difference) % MAX_SIZE;
            last_time_predicted_ = last_time_detected_ = detection_time;
            if (velocity_in_motion_term_)
                filter_->getMahalanobisParameters(mahalanobis_map6d_[last_time_predicted_index_]);
            else
                filter_->getMahalanobisParameters(mahalanobis_map3d_[last_time_predicted_index_]);

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


            return;
        }
        visibility_ = VISIBLE;

        //Update Kalman filter
        int difference;
        double vx, vy, vz;
        if (velocity_in_motion_term_) {
            // We will use durations as follows
            // d = 2 second (alternatively 1 second)
            // d2 = 3 seconds (alternatively 2 seconds)

            struct timeval t2 = detection_time;
            t2.tv_sec -= 2;
            struct timeval t = diff_ms(first_time_detected_, t2) >= 0 ? first_time_detected_ : t2;

            t = diff_ms(t, last_time_detected_) >= 0 ? last_time_detected_ : t;

            t2 = detection_time;
            t2.tv_sec -= 3;
            t = diff_ms(t, t2) >= 0 ? t : t2;

            double dt = diff_s(t, last_time_predicted_);

            difference = int(round(dt / period_));

            int vIndex = (MAX_SIZE + last_time_predicted_index_ + difference) % MAX_SIZE;

            if (difference != 0
                    and not mahalanobis_map6d_[vIndex].x == 0
                    and not mahalanobis_map6d_[vIndex].y == 0
                    and not mahalanobis_map6d_[vIndex].z == 0) // prevent initial drift
            {
                vx = -(x - mahalanobis_map6d_[vIndex].x) / dt;
                vy = -(y - mahalanobis_map6d_[vIndex].y) / dt;
                vz = -(z - mahalanobis_map6d_[vIndex].z) / dt;
            } else {
                vx = mahalanobis_map6d_[vIndex].x;
                vy = mahalanobis_map6d_[vIndex].y;
                vz = mahalanobis_map6d_[vIndex].z;
            }
        }

        // Update Kalman filter from the last time the track was visible:
        int framesLost = int(round(diff_s(detection_time, last_time_detected_) / period_)) - 1;

        for (int i = 0; i < framesLost; i++) {
            filter_->predict();
            filter_->update();
        }

        filter_->predict();
        if (velocity_in_motion_term_) {
            filter_->update(x, y, z, vx, vy, vz, distance);
        } else {
            filter_->update(x, y, z, distance);
        }

        *tmp_filter_ = *filter_;
        difference = int(round(diff_s(detection_time, last_time_predicted_) / period_));
        last_time_predicted_index_ = (MAX_SIZE + last_time_predicted_index_ + difference) % MAX_SIZE;
        last_time_predicted_ = last_time_detected_ = detection_time;
        if (velocity_in_motion_term_)
            filter_->getMahalanobisParameters(mahalanobis_map6d_[last_time_predicted_index_]);
        else
            filter_->getMahalanobisParameters(mahalanobis_map3d_[last_time_predicted_index_]);

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
    Track3D::validate() {
        validated_ = true;
    }

bool
    Track3D::isValidated() {
        return validated_;
    }

int
    Track3D::getId() {
        return id_;
    }

void
    Track3D::setStatus(Track3D::Status s) {
        status_ = s;
    }

    Track3D::Status
    Track3D::getStatus() {
        return status_;
    }

void
    Track3D::setVisibility(Track3D::Visibility v) {
        visibility_ = v;
    }

    Track3D::Visibility
    Track3D::getVisibility() {
        return visibility_;
    }

float
    Track3D::getSecFromFirstDetection(struct timeval current_time) {
        return (float) diff_s(current_time, first_time_detected_);
    }

float
    Track3D::getSecFromLastDetection(struct timeval current_time) {
        return (float) diff_s(current_time, last_time_detected_);
    }

float
    Track3D::getSecFromLastHighConfidenceDetection(struct timeval current_time) {
        return (float) diff_s(current_time, last_time_detected_with_high_confidence_);
    }

float
    Track3D::getLowConfidenceConsecutiveFrames() {
        return low_confidence_consecutive_frames_;
    }

int
    Track3D::getUpdatesWithEnoughConfidence() {
        return updates_with_enough_confidence_;
    }

double
    Track3D::getMahalanobisDistance(double x, double y, double z, const struct timeval& when) {
        int difference = int(round(diff_s(when, last_time_predicted_) / period_));

        int index;
        if (difference <= 0) {
            index = (MAX_SIZE + last_time_predicted_index_ + difference) % MAX_SIZE;
        } else {
            for (int i = 0; i < difference; i++) {
                tmp_filter_->predict();
                last_time_predicted_index_ = (last_time_predicted_index_ + 1) % MAX_SIZE;
                if (velocity_in_motion_term_)

                    tmp_filter_->getMahalanobisParameters(mahalanobis_map6d_[last_time_predicted_index_]);
                else
                    tmp_filter_->getMahalanobisParameters(mahalanobis_map3d_[last_time_predicted_index_]);
                tmp_filter_->update();
            }
            last_time_predicted_ = when;
            index = last_time_predicted_index_;
        }

        if (velocity_in_motion_term_) {
            // Using durations
            // d = 1 second
            // d2 = 2 seconds

            struct timeval t2 = when;
            t2.tv_sec -= 1;
            struct timeval t = diff_s(first_time_detected_, t2) >= 0 ? first_time_detected_ : t2;

            t = diff_s(t, last_time_detected_) >= 0 ? last_time_detected_ : t;

            t2 = last_time_predicted_;
            t2.tv_sec -= 2;
            t = diff_s(t, t2) >= 0 ? t : t2;

            double dt = diff_s(t, last_time_predicted_);

            difference = int(round(dt / period_));
            int vIndex = (MAX_SIZE + last_time_predicted_index_ + difference) % MAX_SIZE;

            double vx, vy, vz;
            if (difference != 0) {
                vx = -(x - mahalanobis_map6d_[vIndex].x) / dt;
                vy = -(y - mahalanobis_map6d_[vIndex].y) / dt;
                vz = -(z - mahalanobis_map6d_[vIndex].z) / dt;
            } else {
                vx = mahalanobis_map6d_[vIndex].x;
                vy = mahalanobis_map6d_[vIndex].y;
                vz = mahalanobis_map6d_[vIndex].z;
            }

            return tracker::KalmanFilter3D::performMahalanobisDistance(x, y, z, vx, vy, vz, mahalanobis_map6d_[index]);
        } else {
            return tracker::KalmanFilter3D::performMahalanobisDistance(x, y, z, mahalanobis_map3d_[index]);
        }

    }

void
    Track3D::setVelocityInMotionTerm(bool velocity_in_motion_term, double acceleration_variance, double position_variance) {
        velocity_in_motion_term_ = velocity_in_motion_term;

        // Re-initialize Kalman filter
        filter_->setPredictModel(acceleration_variance);
        filter_->setObserveModel(position_variance);
        double x, y, z;
        filter_->getState(x, y, z);
        filter_->init(x, y, z, distance_, velocity_in_motion_term_);

        *tmp_filter_ = *filter_;
    }

void
    Track3D::setAccelerationVariance(double acceleration_variance) {
        filter_->setPredictModel(acceleration_variance);
        tmp_filter_->setPredictModel(acceleration_variance);
    }

void
    Track3D::setPositionVariance(double position_variance) {
        filter_->setObserveModel(position_variance);
        tmp_filter_->setObserveModel(position_variance);
    }

void
    Track3D::getState(double &x, double &y, double &z) {
        filter_->getState(x, y, z);
    }

double
    Track3D::getLastDetectorConfidence() {
        return last_detector_confidence_;
    }

void
    Track3D::updateFilter() {
        filter_->update();
    }

} /*namespace tracker*/
