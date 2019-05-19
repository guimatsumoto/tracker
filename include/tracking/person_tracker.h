#ifndef TRACKING_PERSON_TRACKER_H_
#define TRACKING_PERSON_TRACKER_H_

#include "sl_core/ai/ai_release.hpp"

#include "sl_core/ai/skeleton/tracking/person_track.h"
#include "sl_core/ai/skeleton/tracking/pose.h"
#include "sl_core/ai/skeleton/tracking/track.h"
#include "sl_core/ai/skeleton/tracking/tracker.h"
#include "sl_core/ai/skeleton/tracking/track3d.h"
#include "sl_core/ai/skeleton/tracking/track3deuro.h"
#include <chrono>

namespace zed_tracking {

    struct PeopleSkeletonOutput {
        unsigned int id;

        sl::float3 barycenter;

        std::vector<sl::float4> keypoints;
        int keypoint_number;

        std::vector<int> keypoints_links;

        std::vector<sl::float3> keypoints_2d;
        int keypoints_2d_number = 0;

        sl::float3 speed;

        std::vector<int> img_position;
        std::vector<int> img_bbox;

        std::vector<sl::float3> world_bbox;
        std::vector<int> world_bbox_links;

        // Joint angles as sl::Transforms for UNITY
        std::vector<sl::Transform> joint_angles;
        std::vector<int> joint_angle_trios; //This defines the three joints forming each joint angle

        // Gaze direction vector, not normalized
        sl::float3 gaze_direction = sl::float3(NAN, NAN, NAN);
        // Body orientaton vector, not normalized
        sl::float3 body_orientation = sl::float3(NAN, NAN, NAN);
        
        uint64_t ts;
    };

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
        newFrame(const std::vector<sl::float4>& depth_joints, std::vector<int> shape, struct timeval time);

        void
        updateTracks();

        //std::vector<std::tuple<int, Eigen::Vector3d, std::vector<Eigen::Vector4d>, std::vector<zed_tracking::Track3DEuro::Visibility>>>
        std::vector<struct zed_tracking::PeopleSkeletonOutput>
        getTrackedPeople();

        sl::float3
        getKP(int person_idx, int body_part_idx);

        sl::float3
        getGazeDirection(int person_id);

        sl::float3
        getBodyDirection(int person_id);

        sl::float3
        getSpeed(int person_id);

    protected:
        // Doesn't need to be a static variable, as we are no longer working with a camera network
        static int count;

        // All active tracks
        std::list<zed_tracking::PersonTrack*> tracks_;

        // List of active lost tracks
        std::list<zed_tracking::PersonTrack*> lost_tracks_;

        // List of all track with NEW status
        std::list<zed_tracking::PersonTrack*> new_tracks_;

        // Detections of current frame
        std::vector<zed_tracking::Detection> detections_;

        // List of unassociated detections;
        std::list<zed_tracking::Detection> unassociated_detections_;

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
        createNewTrack(zed_tracking::Detection& detection);

    private:


    };

} /*namespace zed_tracking*/
#endif /* TRACKING_PERSON_TRACKER_H */
