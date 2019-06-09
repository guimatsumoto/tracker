#include "tracking/pose.h"

namespace tracker {
	const std::array<std::string, PoseJoints::SIZE>
    Pose::JOINT_NAMES{
        {
            "NOSE",
            "NECK",
            "RIGHT_SHOULDER",
            "RIGHT_ELBOW",
            "RIGHT_WRIST",
            "LEFT_SHOULDER",
            "LEFT_ELBOW",
            "LEFT_WRIST",
            "MID_HIP",
            "RIGHT_HIP",
            "RIGHT_KNEE",
            "RIGHT_ANKLE",
            "LEFT_HIP",
            "LEFT_KNEE",
            "LEFT_ANKLE",
            "RIGHT_EYE",
            "LEFT_EYE",
            "RIGHT_EAR",
            "LEFT_EAR",
            "LEFT_BIG_TOE",
            "LEFT_SMALL_TOE",
            "LEFT_HEEL",
            "RIGHT_BIG_TOE",
            "RIGHT_SMALL_TOE",
            "RIGHT_HEEL"
        }};

    const std::array<std::pair<PoseJoints, PoseJoints>, PoseLinks::SIZE>
            PoseLinks::LINKS = {
        /*
        std::pair<PoseJoints, PoseJoints> (NECK, RSHOULDER),
        std::pair<PoseJoints, PoseJoints> (NECK, LSHOULDER),
        std::pair<PoseJoints, PoseJoints> (RSHOULDER, RELBOW),
        std::pair<PoseJoints, PoseJoints> (RELBOW, RWRIST),
        std::pair<PoseJoints, PoseJoints> (LSHOULDER, LELBOW),
        std::pair<PoseJoints, PoseJoints> (LELBOW, LWRIST),
        std::pair<PoseJoints, PoseJoints> (NECK, MIDHIP),
        std::pair<PoseJoints, PoseJoints> (RHIP, RKNEE),
        std::pair<PoseJoints, PoseJoints> (RKNEE, RANKLE),
        std::pair<PoseJoints, PoseJoints> (LHIP, LKNEE),
        std::pair<PoseJoints, PoseJoints> (LKNEE, LANKLE),
        std::pair<PoseJoints, PoseJoints> (NECK, NOSE),
        std::pair<PoseJoints, PoseJoints> (NOSE, REYE),
        std::pair<PoseJoints, PoseJoints> (REYE, REAR),
        std::pair<PoseJoints, PoseJoints> (NOSE, LEYE),
        std::pair<PoseJoints, PoseJoints> (LEYE, LEAR),
        */

        std::pair<PoseJoints, PoseJoints> (NECK, MIDHIP),
        std::pair<PoseJoints, PoseJoints> (NECK, RSHOULDER),
        std::pair<PoseJoints, PoseJoints> (NECK, LSHOULDER),
        std::pair<PoseJoints, PoseJoints> (RSHOULDER, RELBOW),
        std::pair<PoseJoints, PoseJoints> (RELBOW, RWRIST),
        std::pair<PoseJoints, PoseJoints> (LSHOULDER, LELBOW),
        std::pair<PoseJoints, PoseJoints> (LELBOW, LWRIST),
        std::pair<PoseJoints, PoseJoints> (MIDHIP, RHIP),
        std::pair<PoseJoints, PoseJoints> (RHIP, RKNEE),
        std::pair<PoseJoints, PoseJoints> (RKNEE, RANKLE),
        std::pair<PoseJoints, PoseJoints> (MIDHIP, LHIP),
        std::pair<PoseJoints, PoseJoints> (LHIP, LKNEE),
        std::pair<PoseJoints, PoseJoints> (LKNEE, LANKLE),
        std::pair<PoseJoints, PoseJoints> (NECK, NOSE),
        std::pair<PoseJoints, PoseJoints> (NOSE, REYE),
        std::pair<PoseJoints, PoseJoints> (REYE, REAR),
        std::pair<PoseJoints, PoseJoints> (NOSE, LEYE),
        std::pair<PoseJoints, PoseJoints> (LEYE, LEAR),
        std::pair<PoseJoints, PoseJoints> (RSHOULDER, REAR),
        std::pair<PoseJoints, PoseJoints> (LSHOULDER, LEAR),
        std::pair<PoseJoints, PoseJoints> (LANKLE, LBIGTOE),
        std::pair<PoseJoints, PoseJoints> (LANKLE, LSMALLTOE),
        std::pair<PoseJoints, PoseJoints> (LANKLE, LHEEL),
        std::pair<PoseJoints, PoseJoints> (RANKLE, RBIGTOE),
        std::pair<PoseJoints, PoseJoints> (RANKLE, RSMALLTOE),
        std::pair<PoseJoints, PoseJoints> (RANKLE, RHEEL)
    };

    PoseMeta::PoseMeta() {
        // Start all meta data with NaN values
        joints.fill(std::numeric_limits<double>::quiet_NaN());
    }

    Pose::Pose(PoseMeta& m) : m_is_mutable(false) {
        m_joints = m.joints;
    }

    Pose::Pose() : m_is_mutable(true) {
        m_joints.fill(std::numeric_limits<double>::quiet_NaN());
    }
} /*namespace tracker*/
