#ifndef TRACKING_TRACK_H_
#define TRACKING_TRACK_H_

/*TODO: Check include dependencies*/
#include "sl_core/ai/ai_release.hpp"

#include <Eigen/Eigen>
#include <cmath>
#include "sl_core/ai/skeleton/tracking/kalman_filter.h"
#include "sl_core/ai/skeleton/bayes/bayesFlt.hpp"
#include "sl_core/ai/skeleton/utils.hpp"

namespace zed_tracking {

    class  Track {
    public:

        enum Status {
            NEW,
            NORMAL
        };

        enum Visibility {
            VISIBLE = 0,
            OCCLUDED = 1,
            NOT_VISIBLE = 2
        };

    protected:
        // Dimension of circular buffers that may keep filter information over time
        int MAX_SIZE;

        // Track ID
        const int id_;

        // Frame ID of last detection
        const std::string frame_id_;

        // Minimum period for new detection set
        const double period_;

        // FLAG indicating whether the Track has been validated or not
        bool validated_;

        // Track's KalmanFilter
        zed_tracking::KalmanFilter* filter_;

        // Temporary KalmanFilter used for when a track is re-found
        zed_tracking::KalmanFilter* tmp_filter_;

        // First time a detections is associated to the track
        struct timeval first_time_detected_;

        // Last time a detection was associated to the track
        struct timeval last_time_detected_;


        // Last time a high confidence detection was associated to the track
        struct timeval last_time_detected_with_high_confidence_;

        // Last time a prediction was performed
        struct timeval last_time_predicted_;

        // Last index in the circular buffer corresponding to the last prediction instant
        int last_time_predicted_index_;

        // Variables for the 2d Mahalanobis distance
        std::vector<MahalanobisParameters2d> mahalanobis_map2d_;

        // Variables for the 4d Mahalanobis distance
        std::vector<MahalanobisParameters4d> mahalanobis_map4d_;

        // Track status
        Status status_;

        // Track visibility
        Visibility visibility_;

        // Number of high confidence detections
        int updates_with_enough_confidence_;

        // Track centroid coordinate
        double z_;

        // Track last centroid coordinate for speed computation
        double z_previous;

        // Track height
        double height_;

        // Track distance
        double distance_;

        // Track's age (seconds)
        double age_;

        // Last detection confidence
        double last_detector_confidence_;

        // Last data association score
        double data_association_score_;

        // Last data association score TODO: Probably no data association necessary for one camera
        double last_association_confidence_;

        // Track's color
        Eigen::Vector3f color_;

        bool velocity_in_motion_term_;

        // Number of consecutive low confidence detection
        int low_confidence_consecutive_frames_;

    public:

        // Constructor
        Track(int id, std::string frame_id, double position_variance,
                double acceleration_variance, double period, bool velocity_in_motion_term);

        // Destructor
        virtual ~Track();

        // Initialize track from old track
        virtual void
        init(const zed_tracking::Track& old_track);

        // Initialize track
        virtual void
        init(double x, double y, double z, double height,
                double distance, struct timeval detection_time);

        // Update track with a new detection
        virtual void
        update(double x, double y, double z, double height,
                double distance, double data_association_score,
                double confidence, double min_confidence,
                double min_confidence_detections,
                struct timeval detection_time, bool first_update = false);

        // Compute 2D Mahalanobis distance
        virtual double
        getMahalanobisDistance(double x, double y, const struct timeval& when);

        // Validate the track
        virtual void
        validate();

        // Get validation FLAG
        virtual bool
        isValidated();

        // Get ID
        virtual int
        getId();

        // Set status
        virtual void
        setStatus(Status s);

        // Get status
        virtual Status
        getStatus();

        // Set visibility
        virtual void
        setVisibility(Visibility v);

        // Get visibility
        virtual Visibility
        getVisibility();

        // Get time since first detection association
        virtual float
        getSecFromFirstDetection(struct timeval current_time);

        // Get time since last detection association
        virtual float
        getSecFromLastDetection(struct timeval current_time);

        // Get time since last high confidence detection
        virtual float
        getSecFromLastHighConfidenceDetection(struct timeval current_time);

        // Get the number of consecutive low confidence detection updates
        virtual float
        getLowConfidenceConsecutiveFrames();

        // Get the number of updates with enough confidence
        virtual int
        getUpdatesWithEnoughConfidence();

        // Set people velocity in motion term FLAG
        virtual void
        setVelocityInMotionTerm(bool velocity_in_motion_term,
                double acceleration_variance, double position_variance);

        // Set acceleration variance
        virtual void
        setAccelerationVariance(double acceleration_variance);

        // Set position variance
        virtual void
        setPositionVariance(double position_variance);

        virtual sl::float3
        getSpeed();

    };

} /*namespace zed_tracking*/

#endif /* TRACKING_TRACK_H_ */
