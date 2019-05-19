#ifndef TRACKING_POSE_H_
#define TRACKING_POSE_H_
#include "sl_core/ai/ai_release.hpp"

#include <vector>
#include <array>
#include <Eigen/Eigen>

namespace zed_tracking {

    // Pose Keypoints ordering

    enum PoseJoints {
        NOSE = 0,
        NECK,
        RSHOULDER,
        RELBOW,
        RWRIST,
        LSHOULDER,
        LELBOW,
        LWRIST,
        MIDHIP,
        RHIP,
        RKNEE,
        RANKLE,
        LHIP,
        LKNEE,
        LANKLE,
        REYE,
        LEYE,
        REAR,
        LEAR,
        LBIGTOE,
        LSMALLTOE,
        LHEEL,
        RBIGTOE,
        RSMALLTOE,
        RHEEL,
        SIZE
    };

    class  PoseLinks {
    public:
        static const size_t SIZE = 26;
        static const std::array<std::pair<PoseJoints, PoseJoints>, PoseLinks::SIZE> LINKS;
    };

    class  PoseMeta {
    public:
        // Array to keep 3D position of all joints of a pose
        Eigen::Matrix<double, 3, PoseJoints::SIZE> joints;
        PoseMeta();
    };

    class  Pose {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        Eigen::Matrix<double, 3, PoseJoints::SIZE> m_joints;
        bool m_is_mutable;

    public:
        Pose(PoseMeta& m);
        Pose();

        static const std::array<std::string, PoseJoints::SIZE> JOINT_NAMES;

        inline Eigen::Matrix<double, 3, PoseJoints::SIZE> getJoints() const {
            return m_joints;
        }

        inline Eigen::Vector3d getJoint(size_t i) const {
            return m_joints.col(i);
        }

        inline void setJoint(size_t i, const Eigen::Vector3d& j) {
            if (!m_is_mutable)
                throw std::runtime_error("This pose cannot be changed.");
            m_joints.col(i) << j;
        }

        inline void setJoints(const Eigen::Matrix<double, 3, PoseJoints::SIZE>& m) {
            if (!m_is_mutable)
                throw std::runtime_error("This pose cannot be changed.");
            m_joints = m;
        }

        inline bool isValid(size_t i) const {
            return !m_joints.col(i).hasNaN();
        }

        inline bool isValid() const {
            return !m_joints.hasNaN();
        }
    };

} /*namespace zed_tracking*/

#endif /* TRACKING_POSE_H_ */
