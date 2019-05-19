#ifndef TRACKING_PERSON_TRACK_H_
#define TRACKING_PERSON_TRACK_H_

#include "sl_core/ai/ai_release.hpp"

#include <Eigen/Eigen>
#include <cmath>
#include "sl_core/ai/skeleton/tracking/kalman_filter3d.h"
#include "sl_core/ai/skeleton/bayes/bayesFlt.hpp"
#include "sl_core/ai/skeleton/tracking/detection.h"
#include "sl_core/ai/skeleton/tracking/track.h"
#include "sl_core/ai/skeleton/tracking/track3d.h"
#include "sl_core/ai/skeleton/tracking/track3deuro.h"
#include "sl_core/ai/skeleton/tracking/tracker3d.h"
#include <memory>
#include "sl_core/ai/skeleton/tracking/pose.h"

#include "sl_core/ai/skeleton/fitting/pso.h"
#include "sl_core/ai/skeleton/fitting/tree.hpp"

namespace zed_tracking {

    /** \brief PersonTrack represents information about a track (or target) */
    class  PersonTrack : public Track {
    private:
        bool
        isValid(const Eigen::Vector4d& joint);

        bool
        isValid(const std::vector<sl::float4>& joint);

    protected:
        std::vector<zed_tracking::Track3DEuro*> joint_tracks_;
        std::vector<Eigen::Vector4d> raw_joints_tmp_;
        std::vector<zed_tracking::OneEuroFilter3D*> bbox_vertices_;
        bool all_joint_tracks_initialized_;

        static int count;
        int debug_count_;

        // Unfitted joints
        std::vector<sl::float4>
        getJointsPosition();

        // Skeleton Fitting
        std::vector<sl::float4> fitted_keypoints_; // Last fitted global positions
        float last_optim_state_[NUM_OF_DIMENSIONS]; // Last optimisation state
        slBody25KinChain *body;

        void
        fitToSkeleton();

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

        std::vector<zed_tracking::Track3DEuro*>
        getKeypoints();

        std::vector<sl::float4>
        getJointsFitted();

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

        std::vector<sl::Transform>
        getJointAngles();

        //friend std::ostream&
        //operator<< (std::ostream& ss, const PersonTrack& s);
    };

} /*namespace zed_tracking*/

#endif /* TRACKING_PERSON_TRACK_H_ */
