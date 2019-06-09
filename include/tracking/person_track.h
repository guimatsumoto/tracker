#ifndef TRACKER_PERSON_TRACK_H_
#define TRACKER_PERSON_TRACK_H_

#include <Eigen/Eigen>
#include <cmath>
#include "tracking/kalman_filter3d.h"
#include "bayes/bayesFlt.hpp"
#include "tracking/detection.h"
#include "tracking/track.h"
#include "tracking/track3d.h"
#include "tracking/track3deuro.h"
#include "tracking/tracker3d.h"
#include <memory>
#include "tracking/pose.h"

namespace tracker {

    /** \brief PersonTrack represents information about a track (or target) */
    class  PersonTrack : public Track {
    private:
        bool
        isValid(const Eigen::Vector4d& joint);

        bool
        isValid(const std::vector<tracker::float4>& joint);

    protected:
        std::vector<tracker::Track3DEuro*> joint_tracks_;
        std::vector<Eigen::Vector4d> raw_joints_tmp_;
        std::vector<tracker::OneEuroFilter3D*> bbox_vertices_;
        bool all_joint_tracks_initialized_;

        static int count;
        int debug_count_;

        // Unfitted joints
        std::vector<tracker::float4>
        getJointsPosition();

    public:

        /** \brief Constructor. */
        PersonTrack(int id,
                std::string frame_id, double position_variance,
                double acceleration_variance, double period,
                bool velocity_in_motion_term,
                const std::vector<Eigen::Vector4d>& joints);

        /** \brief Destructor. */
        virtual ~PersonTrack();

        bool
        anyNaNs(const std::vector<Eigen::Vector4d>& joints);

        void
        update(double x,
                double y,
                double z,
                double height,
                double distance,
                double data_assocation_score,
                double confidence,
                double min_confidence,
                double min_confidence_detections,
                struct timeval detection_time,
                const std::vector<Eigen::Vector4d>& joints,
                bool first_update = false);

        void
        init(double x, double y, double z, double height, double distance,
                struct timeval detection_time,
                const std::vector<Eigen::Vector4d>& joints);

        bool
        areJointsInitialized();

        std::vector<tracker::Track3DEuro*>
        getKeypoints();

        /*
        std::vector<tracker::float4>
        getJointsFitted();
        */

        Eigen::Vector3d
        getBarycenter();

        Eigen::Vector3d
        getKP(int body_part_idx);

        Eigen::Vector3d
        getGazeDirection();

        Eigen::Vector3d
        getBodyDirection();

        std::vector<Eigen::Vector3d>
        getBoundingBox();

        std::vector<int>
        getBoundingBoxLinks();

        std::vector<Eigen::Affine3f>
        getJointAngles();

        //friend std::ostream&
        //operator<< (std::ostream& ss, const PersonTrack& s);
    };

} /*namespace tracker*/

#endif /* TRACKER_PERSON_TRACK_H_ */
