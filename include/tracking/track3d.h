#ifndef TRACKING_TRACK_3D_H_
#define TRACKING_TRACK_3D_H_
#include "sl_core/ai/ai_release.hpp"

#include <Eigen/Eigen>
#include <cmath>
#include "sl_core/ai/skeleton/tracking/kalman_filter3d.h"
#include "sl_core/ai/skeleton/bayes/bayesFlt.hpp"
#include "sl_core/ai/skeleton/utils.hpp"

namespace zed_tracking {

    /** \brief Track represents information about a track (or target) */
    class  Track3D {
    public:

        /** \brief A track has Status NEW if it has been recently created, otherwise it has NORMAL Status */
        enum Status {
            NEW,
            NORMAL
        };

        /** \brief Visibility states if the track is currently visible by the sensor or partially occluded or totally occluded */
        enum Visibility {
            VISIBLE = 0, // No occlusion
            OCCLUDED = 1, // Partially occlusion
            NOT_VISIBLE = 2 // Total occlusion
        };

    protected:

        /** \brief Dimension of a circular buffer which keep tracks of filter parameters along time */
        int MAX_SIZE;

        /** \brief Track ID */
        const int id_;

        /** \brief Track frame id (frame id of the last detection associated to the track */
        const std::string frame_id_;

        /** \brief Inverse of the frame rate */
        const double period_;

        /** \brief If true, the track is validated, meaning that it has been associated with a certain number of high confidence detections */
        bool validated_;

        /** \brief Kalman filter associated to the track */
        zed_tracking::KalmanFilter3D* filter_;

        /** \brief Temporary copy of the Kalman filter associated to the track (used for recovery filter information when a track is re-found) */
        zed_tracking::KalmanFilter3D* tmp_filter_;

        /** \brief First time a detection is associated to the track */
        struct timeval first_time_detected_;

        /** \brief Last time a detection is associated to the track */
        struct timeval last_time_detected_;

        /** \brief Last time a detection with high detection confidence is associated to the track */
        struct timeval last_time_detected_with_high_confidence_;

        /** \brief Last time a prediction has been performed for the track */
        struct timeval last_time_predicted_;

        /** \brief Index in the circular buffer corresponding to the last time a prediction has been performed */
        int last_time_predicted_index_;

        /** Variables used for computing the detection/track Mahalanobis distance */
        std::vector<MahalanobisParameters3d> mahalanobis_map3d_;

        /** Variables used for computing the detection/track Mahalanobis distance */
        std::vector<MahalanobisParameters6d> mahalanobis_map6d_;

        /** \brief Track Status*/
        Status status_;

        /** \brief Track Visibility */
        Visibility visibility_;

        /** \brief Number of high confidence detections associated to the track */
        int updates_with_enough_confidence_;

        /** \brief Track centroid z coordinate */
        double z_;

        /** \brief Track height */
        double height_;

        /** \brief Track distance from the camera */
        double distance_;

        /** \brief Track age (in seconds) */
        double age_;

        /** \brief Confidence of the last detection associated to the track */
        double last_detector_confidence_;

        /** \brief Last data association score obtained by this track */
        double data_association_score_;

        /** \brief Color associated to the track */
        Eigen::Vector3f color_;

        /** \brief If true, both position and velocity are considered in computing detection<->track Mahalanobis distance */
        bool velocity_in_motion_term_;

        /** \brief Count the number of consecutive updates with low confidence detections */
        int low_confidence_consecutive_frames_;

    public:

        /** \brief Constructor. */
        Track3D(
                int id,
                std::string frame_id,
                double position_variance,
                double acceleration_variance,
                double period,
                bool velocity_in_motion_term);

        /** \brief Destructor. */
        virtual ~Track3D();

        /** \brief Track initialization with an old track. */
        virtual void
        init(const zed_tracking::Track3D& old_track);

        /**
         * \brief Track initialization.
         *
         * \param[in] x Track centroid x coordinate
         * \param[in] y Track centroid y coordinate
         * \param[in] z Track centroid z coordinate
         * \param[in] height Track height
         * \param[in] distance Track distance from the sensor
         * \param[in] detection_source DetectionSource which provided the last detection associated to the track
         */
        virtual void
        init(
                double x,
                double y,
                double z,
                double height,
                double distance,
                struct timeval detection_time);

        /**
         * \brief Update track with new detection information.
         *
         * \param[in] x Detection centroid x coordinate
         * \param[in] y Detection centroid y coordinate
         * \param[in] z Detection centroid z coordinate
         * \param[in] height Detection height
         * \param[in] distance Detection distance from the sensor
         * \param[in] confidence Detection confidence
         * \param[in] min_confidence Minimum confidence for track initialization
         * \param[in] min_confidence_detections Minimum confidence for detection
         * \param[in] detection_source DetectionSource which provided the detection
         */
        virtual void
        update(
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
                bool first_update = false);

        /**
         * \brief Compute Mahalanobis distance between detection with position (x,y) and track.
         *
         * \param[in] x Detection centroid x coordinate.
         * \param[in] y Detection centroid y coordinate.
         * \param[in] when Time instant.
         *
         * \return the Mahalanobis distance.
         */
        virtual double
        getMahalanobisDistance(double x, double y, double z, const struct timeval& when);

        /* Validate a track */
        virtual void
        validate();

        /**
         * \brief Get track validation flag
         *
         * \return true if the track has been validated, false otherwise.
         */
        virtual bool
        isValidated();

        /**
         * \brief Get track ID
         *
         * \return track ID
         */
        virtual int
        getId();

        /**
         * \brief Set track status to s
         *
         * \param[in] s status
         */
        virtual void
        setStatus(Status s);

        /**
         * \brief Get track status
         *
         * \return track status
         */
        virtual Status
        getStatus();

        /**
         * \brief Set track Visibility.
         *
         * \param[in] v Visibility status.
         */
        virtual void
        setVisibility(Visibility v);

        /**
         * \brief Get track Visibility.
         *
         * \return track Visibility.
         */
        virtual Visibility
        getVisibility();

        /**
         * \brief Get time passed from first detection-track association.
         *
         * \return time passed from first detection-track association.
         */
        virtual float
        getSecFromFirstDetection(struct timeval current_time);

        /**
         * \brief Get time passed from last detection-track association.
         *
         * \return time passed from last detection-track association.
         */
        virtual float
        getSecFromLastDetection(struct timeval current_time);

        /**
         * \brief Get time passed from last detection-track association with a high confidence detection.
         *
         * \return time passed from last detection-track association with a high confidence detection.
         */
        virtual float
        getSecFromLastHighConfidenceDetection(struct timeval current_time);

        /**
         * \brief Get the number of consecutive updates with low confidence detections.
         *
         * \return the number of consecutive updates with low confidence detections.
         */
        virtual float
        getLowConfidenceConsecutiveFrames();

        /**
         * \brief Get the number of updates with enough confidence detections.
         *
         * \return the number of updates with enough confidence detections.
         */
        virtual int
        getUpdatesWithEnoughConfidence();

        /**
         * \brief Set flag stating if people velocity should be used in motion term for data association
         *
         * \param[in] velocity_in_motion_term If true, people velocity is also used in motion term for data association
         * \param[in] acceleration_variance Acceleration variance (for Kalman Filter)
         * \param[in] position_variance Position variance (for Kalman Filter)
         */
        virtual void
        setVelocityInMotionTerm(bool velocity_in_motion_term, double acceleration_variance, double position_variance);

        /**
         * \brief Set acceleration variance (for Kalman Filter)
         *
         * \param[in] acceleration_variance Acceleration variance (for Kalman Filter)
         */
        virtual void
        setAccelerationVariance(double acceleration_variance);

        /**
         * \brief Set position variance (for Kalman Filter)
         *
         * \param[in] position_variance Position variance (for Kalman Filter)
         */
        virtual void
        setPositionVariance(double position_variance);

        virtual void
        getState(double& x, double& y, double& z);

        virtual double
        getLastDetectorConfidence();

        virtual void
        updateFilter();

    };

} /*namespace zed_tracking*/

#endif /* TRACKING_TRACK_H_ */

