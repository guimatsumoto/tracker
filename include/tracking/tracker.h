#ifndef TRACKING_TRACKER_H_
#define TRACKING_TRACKER_H_
#include "sl_core/ai/ai_release.hpp"

#include "sl_core/ai/skeleton/tracking/detection.h"
#include "sl_core/ai/skeleton/tracking/track.h"
#include "sl_core/ai/skeleton/tracking/munkres.h"
#include <chrono>

/*TODO: Verify if data association is necessary*/

namespace zed_tracking {

    class  Tracker {
    protected:
        // Active tracks
        std::list<zed_tracking::Track*> tracks_;

        // Lost tracks
        std::list<zed_tracking::Track*> lost_tracks_;

        // Tracks with NEW status
        std::list<zed_tracking::Track*> new_tracks_;

        // Current frame's detections
        std::vector<zed_tracking::Detection> detections_;

        // Current frame's unassociated detections
        std::list<zed_tracking::Detection> unassociated_detections_;

        // Unique tracks counter
        int tracks_counter_;

        // World reference frame TODO: Probably remove it
        std::string world_frame_id_;

        // Threshold for track initialization
        double min_confidence_;

        // Threshold for a detection to be sent to the tracker
        const double min_confidence_detections_;

        // Nulber of detections needed to validate a track
        int detections_to_validate_;

        // Seconds before a track is considered old
        double sec_before_old_;

        // Seconds to so a NEW track becomes NORMAL
        double sec_remain_new_;

        // Seconds within a track should be validated, otherwise its ignored
        double sec_before_fake_;

        // Gate distance for joint likelihood association
        double gate_distance_;

        // Flag indicating whether detection confidence should be considered for data association
        bool detector_likelihood_;

        // Weights for the single terms of the joint likelihood
        std::vector<double> likelihood_weights_;

        //Flag indicating whether velocity should be considered for data association
        bool velocity_in_motion_term_;

        // Minimum time period between two detections
        const double period_;

        // Position variance (UKF)
        double position_variance_;

        // Acceleration variance (UKF)
        double acceleration_variance_;

        // Flag indicating whether to show debug messages or not
        const bool debug_mode_;

        // Distance matrix for data association
        cv::Mat_<double> distance_matrix_;

        // Cost matrix for global nearest neighbor
        cv::Mat_<double> cost_matrix_;

        // Create the distance matrix
        virtual void
        createDistanceMatrix();

        // Create the cost matrix
        virtual void
        createCostMatrix();

        // Update tracks that have a detection association
        virtual void
        updateDetectedTracks();

        // Fill unassociated detections list
        virtual void
        fillUnassociatedDetections();

        // Create new tracks from unassociated detections
        virtual void
        createNewTracks();

        // Create a new track from a Pose object
        virtual int
        createNewTrack(Detection& detection);

        // Update lost tracks
        virtual void
        updateLostTracks();

    public:
        // Constructor
        Tracker(double gate_distance, bool detector_likelihood,
                std::vector<double> likelihood_weights, bool velocity_in_motion_term,
                double min_confidence, double min_confidence_detections, double sec_before_old,
                double sex_before_fake, double sec_remain_new, int detections_to_validate,
                double period, double position_variance, double acceleration_variance,
                std::string world_frame_id, bool debug_mode);

        // Destructor
        virtual ~Tracker();

        // Initialize a new set of detections
        virtual void
        newFrame(const std::vector<zed_tracking::Detection>& detections);

        // Update the set of tracks
        virtual void
        updateTracks();

        // Set min_confidence
        virtual void
        setMinConfidenceForTrackInitialization(double min_confidence);

        // Set time after a not visible track becomes old
        virtual void
        setSecBeforeOld(double sec_before_old);

        // Set time within a track should be validated
        virtual void
        setSecBeforeFake(double sec_before_fake);

        // Set the time a track should remain NEW before it becomes NORMAL
        virtual void
        setSecRemainNew(double sec_remain_new);

        // Set minimum number of detections before a track can be validated
        virtual void
        setDetectionsToValidate(int detections_to_validate);

        // Set the use detection confidence on data association FLAG
        virtual void
        setDetectorLikelihood(bool detector_likelihood);

        // Set likelihood weights
        virtual void
        setLikelihoodWeights(double detector_weight, double motion);

        // Set velocity in lotion term FLAG
        virtual void
        setVelocityInMotionTerm(bool velocity_in_motion_term, double acceleration_variance,
                double position_variance);

        // Set acceleration variance
        virtual void
        setAccelerationVariance(double acceleration_variance);

        // Set position variance
        virtual void
        setPositionVariance(double position_variance);

        // Set gate distance for data association
        virtual void
        setGateDistance(double gate_distance);

    };

} /*namespace zed_tracking*/

#endif /* TRACKING_TRACKER_H_ */
