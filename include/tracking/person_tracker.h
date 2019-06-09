#ifndef TRACKER_PERSON_TRACKER_H_
#define TRACKER_PERSON_TRACKER_H_

#include "tracking/person_track.h"
#include "tracking/pose.h"
#include "tracking/track.h"
#include "tracking/tracker.h"
#include "tracking/track3d.h"
#include "tracking/track3deuro.h"
#include <chrono>
#include <sys/time.h>

namespace tracker {
    // Inherits from Tracker (2D) -> project the barycenter on the floor

    class  PersonTracker : public Tracker {
    public:

        PersonTracker(double gate_distance, bool detector_likelihood,
                std::vector<double> likelihood_weights,
                bool velocity_in_motion_term,
                double min_confidence,
                double min_confidence_detections,
                double sec_before_old, double sec_before_fake,
                double sec_remain_new, int detections_to_validate,
                double period, double position_variance,
                double acceleration_variance,
                std::string world_frame_id, bool debug_mode,
                bool vertical) :
        Tracker(gate_distance, detector_likelihood, likelihood_weights,
        velocity_in_motion_term, min_confidence,
        min_confidence_detections, sec_before_old, sec_before_fake,
        sec_remain_new, detections_to_validate, period,
        position_variance, acceleration_variance,
        world_frame_id, debug_mode) {
        }

        virtual
        ~PersonTracker();

        void
        newFrame(const std::vector<Eigen::Vector4f>& depth_joints, std::vector<int> shape, struct timeval time);

        void
        updateTracks();

        //std::vector<std::tuple<int, Eigen::Vector3d, std::vector<Eigen::Vector4d>, std::vector<zed_tracking::Track3DEuro::Visibility>>>
        std::vector<tracker::PeoplePose>
        getTrackedPeople();

        tracker::float3
        getKP(int person_idx, int body_part_idx);

        tracker::float3
        getGazeDirection(int person_id);

        tracker::float3
        getBodyDirection(int person_id);

        tracker::float3
        getSpeed(int person_id);

    protected:
        // Doesn't need to be a static variable, as we are no longer working with a camera network
        static int count;

        // All active tracks
        std::list<tracker::PersonTrack*> tracks_;

        // List of active lost tracks
        std::list<tracker::PersonTrack*> lost_tracks_;

        // List of all track with NEW status
        std::list<tracker::PersonTrack*> new_tracks_;

        // Detections of current frame
        std::vector<tracker::Detection> detections_;

        // List of unassociated detections;
        std::list<tracker::Detection> unassociated_detections_;

        void
        createDistanceMatrix();

        void
        updateDetectedTracks();

        void
        createNewTracks();

        void
        createCostMatrix();

        void
        fillUnassociatedDetections();

        int
        createNewTrack(tracker::Detection& detection);

    private:


    };

} /*namespace tracker*/
#endif /* TRACKER_PERSON_TRACKER_H */
