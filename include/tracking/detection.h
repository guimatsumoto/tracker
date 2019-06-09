#ifndef TRACKER_DETECTION_H_
#define TRACKER_DETECTION_H_

#include <Eigen/Eigen>
#include <tracking/pose.h>

namespace tracker {

    /** \brief Detection represents information about a people detection */
    class Detection {
    protected:
        /** \brief ROS message containing detection information */
        std::vector<Eigen::Vector4d> keypoints_;

        /** \brief Detection source which produced the detection */
        struct timeval detection_time_;

        /** \brief Detection centroid in world reference frame */
        Eigen::Vector3d world_centroid_;

        bool is_occluded_;

        double confidence_;

        double distance_;

    public:

        /** \brief Constructor.
            \param[in] keypoints vector composed of a keypoint concatenation of (x, y, z) + score
         */
        Detection(std::vector<Eigen::Vector4d> keypoints, Eigen::Vector3d gravity_center, struct timeval detection_time, double distance);

        /** \brief Destructor. */
        virtual ~Detection();

        /**
         * \brief Returns the detection time.
         *
         * \return a timeval struct with the time of detection.
         */
        struct timeval
        getTime();

        /**
         * \brief Returns the detection centroid in world reference frame.
         *
         * \return the detection centroid in world reference frame.
         */
        Eigen::Vector3d
        getWorldCentroid() const;

        /**
         * \brief Returns the confidence of the people detector associated to the detection.
         *
         * \return the confidence of the people detector associated to the detection.
         */
        double
        getConfidence();

        /**
         * \brief Returns if the detection corresponds to an occluded person or not.
         *
         * \return true if the detection corresponds to an occluded person, false otherwise.
         */
        bool
        isOccluded();

        /**
         * \brief The object name
         *
         * \return the object name
         */
        std::string
        getObjectName();

        /**
         * \brief Set the confidence of the people detector associated to the detection.
         *
         * \param[in] confidence Confidence of the people detector associated to the detection.
         */
        void
        setConfidence(double confidence);

        /**
         * \brief Set the detection centroid in world reference frame.
         *
         * \param[in] centroid The detection centroid in world reference frame.
         */
        void
        setWorldCentroid(const Eigen::Vector3d& centroid);

        std::vector<Eigen::Vector4d>
        getJoints();


        /**
         * \brief Returns the detection height from the ground plane.
         *
         * \return the detection height from the ground plane.
         */
        double
        getHeight();

        /**
         * \brief Returns the distance of the detection from the sensor.
         *
         * \return the distance of the detection from the sensor.
         */
        double
        getDistance();

    };

} /*namespace tracker*/

#endif /* TRACKER_DETECTION_H_ */
