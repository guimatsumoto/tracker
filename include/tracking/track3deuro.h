#ifndef TRACKER_TRACK_3D_EURO_H_
#define TRACKER_TRACK_3D_EURO_H_

#include <Eigen/Eigen>
#include <cmath>
#include "tracking/one_euro_filter.h"
#include "utils.hpp"

namespace tracker {

    /** \brief Track represents information about a track (or target) */
    class  Track3DEuro {
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
        tracker::OneEuroFilter3D* filter_;

        /** \brief Temporary copy of the Kalman filter associated to the track (used for recovery filter information when a track is re-found) */
        tracker::OneEuroFilter3D* tmp_filter_;

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

        /** \brief Count the number of consecutive updates with low confidence detections */
        int low_confidence_consecutive_frames_;

    public:

        /** \brief Constructor. */
        Track3DEuro(
                int id,
                std::string frame_id,
                double period,
                double mincutoff = 1.0,
                double beta = 0.0,
                double dcutoff = 1.0);

        /** \brief Destructor. */
        virtual ~Track3DEuro();

        /** \brief Track initialization with an old track. */
        virtual void
        init(const tracker::Track3DEuro& old_track);

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

        virtual void
        getState(double& x, double& y, double& z);

        virtual double
        getLastDetectorConfidence();

        virtual void
        updateFilter();

    };

} /*namespace tracker*/

#endif /* TRACKER_TRACK_H_ */

