#include "tracking/detection.h"

namespace tracker {

    Detection::Detection(std::vector<Eigen::Vector4d> keypoints, Eigen::Vector3d gravity_center, struct timeval detection_time, double distance) :
    keypoints_(keypoints), world_centroid_(gravity_center), detection_time_(detection_time), distance_(distance) {
        is_occluded_ = false;
        // Compute the mean confidence
        int count = 0, count_conf = 0;
        double confidence = 0;
        for (int keypoint = 0; keypoint < PoseJoints::SIZE; keypoint++)
        {
            if (keypoints[keypoint](3) <= 0)
            {
                is_occluded_ = false;
                if (keypoint >= 0 && keypoint <= 14)
                {
                    confidence += 2*keypoints[keypoint](3);
                    count_conf++;
                }
                else
                    confidence += keypoints[keypoint](3);
                count_conf++;
            }
            else
            {
                if (keypoint >= 0 && keypoint <= 14)
                {
                    confidence += 2*keypoints[keypoint](3);
                    count_conf++;
                }
                else
                    confidence += keypoints[keypoint](3);
                count_conf++;
                count++;
            }
        }
        confidence_ = confidence / count_conf;
        if (count <= 6)
            is_occluded_ = true;
    }

    Detection::~Detection() {

    }

    struct timeval
    Detection::getTime() {
        return detection_time_;
    }

    Eigen::Vector3d
    Detection::getWorldCentroid() const {
        return world_centroid_;
    }

    double
    Detection::getConfidence() {
        return confidence_;
    }

    bool
    Detection::isOccluded() {
        return is_occluded_;
    }

    std::string
    Detection::getObjectName() {
        return "Person";
    }

    void
    Detection::setConfidence(double confidence) {
        confidence_ = confidence;
    }

    void
    Detection::setWorldCentroid(const Eigen::Vector3d& centroid) {
        world_centroid_ = centroid;
    }

    std::vector<Eigen::Vector4d>
    Detection::getJoints() {
        return keypoints_;
    }

    double
    Detection::getHeight() {
        // Here we are assuming that a person's height is equivalent to the height of 
        // their nose in the world reference plane COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP
        return keypoints_[0](1);
    }

    double
    Detection::getDistance() {
        return distance_;
    }

} /*namespace tracker*/
