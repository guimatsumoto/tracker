#ifndef TRACKER_TRACKER_3D_H_
#define TRACKER_TRACKER_3D_H_

#include "tracking/detection.h"
#include "tracking/track3d.h"
#include "tracking/munkres.h"

/** \brief Tracker performs tracking-by-detection */
/*TODO: Verify the use of a pose or detection class*/

namespace tracker {

    class  Tracker3D {
    protected:
        /** \brief List of all active tracks */
        std::list<tracker::Track3D*> tracks_;

        /** \brief List of lost tracks */
        std::list<tracker::Track3D*> lost_tracks_;

        /** \brief List of tracks with Status = NEW */
        std::list<tracker::Track3D*> new_tracks_;

        /** \brief List of current detections */
        std::vector<tracker::Detection> detections_;

        /** \brief List of current detections not associated to any track */
        std::list<tracker::Detection> unassociated_detections_;

        /** \brief Track ID counter */
        int tracks_counter_;

        /** \brief World reference frame used for tracking */
        std::string world_frame_id_;

        /** \brief Minimum confidence for track initialization */
        double min_confidence_;

        /** \brief Minimum confidence of detections sent to tracking */
        const double min_confidence_detections_;

        /** \brief Minimum number of detection<->track associations needed for validating a track */
        int detections_to_validate_;

        /** \brief Time after which a not visible track becomes old */
        double sec_before_old_;

        /** \brief Time after which a visible track obtain NORMAL status */
        double sec_remain_new_;

        /** \brief Time within which a track should be validated (otherwise it is discarded) */
        double sec_before_fake_;

        /** \brief Gate distance for joint likelihood in data association */
        double gate_distance_;

        /** \brief Flag stating if people detection confidence should be used in data association (true) or not (false) */
        bool detector_likelihood_;

        /** \brief Weights for the single terms of the joint likelihood */
        std::vector<double> likelihood_weights_;

        /** \brief If true, people velocity is also used in motion term for data association */
        bool velocity_in_motion_term_;

        /** \brief Minimum time period between two detections messages */
        const double period_;

        /** \brief Position variance (for Kalman Filter) */
        double position_variance_;

        /** \brief Acceleration variance (for Kalman Filter) */
        double acceleration_variance_;

        /** \brief Flag enabling debug mode */
        const bool debug_mode_;

        /** \brief Detections<->tracks distance matrix for data association */
        cv::Mat_<double> distance_matrix_;

        /** \brief Detections<->tracks cost matrix to be used to solve the Global Nearest Neighbor problem */
        cv::Mat_<double> cost_matrix_;

        /** \brief if true, the sensor is considered to be vertically placed (portrait mode) */
        bool vertical_;

        /** \brief Create detections<->tracks distance matrix for data association */
        virtual void
        createDistanceMatrix();

        /** \brief Create detections<->tracks cost matrix to be used to solve the Global Nearest Neighbor problem */
        virtual void
        createCostMatrix();

        /** \brief Update tracks associated to a detection in the current frame */
        virtual void
        updateDetectedTracks();

        /** \brief Fill list containing unassociated detections */
        virtual void
        fillUnassociatedDetections();

        /** \brief Create new tracks with high confidence unassociated detections */
        virtual void
        createNewTracks();

        /** \brief Create a new track with detection information */
        virtual int
        createNewTrack(Detection& detection);

        /** \brief Update lost tracks */
        virtual void
        updateLostTracks();

    public:
        /** \brief Constructor */
        Tracker3D(double gate_distance, bool detector_likelihood, std::vector<double> likelihood_weights, bool velocity_in_motion_term,
                double min_confidence, double min_confidence_detections, double sec_before_old, double sec_before_fake,
                double sec_remain_new, int detections_to_validate, double period, double position_variance,
                double acceleration_variance, std::string world_frame_id, bool debug_mode, bool vertical);

        /** \brief Destructor */
        virtual ~Tracker3D();

        /**
         * \brief Initialization when a new set of detections arrive.
         *
         * \param[in] detections Vector of current detections.
         *
         */
        virtual void
        newFrame(const std::vector<tracker::Detection>& detections);

        /**
         * \brief Update the list of tracks according to the current set of detections.
         */
        virtual void
        updateTracks();

        /**
         * \brief Set minimum confidence for track initialization
         *
         * \param[in] min_confidence Minimum confidence for track initialization
         */
        virtual void
        setMinConfidenceForTrackInitialization(double min_confidence);

        /**
         * \brief Set time after which a not visible track becomes old
         *
         * \param[in] sec_before_old Time after which a not visible track becomes old
         */
        virtual void
        setSecBeforeOld(double sec_before_old);

        /**
         * \brief Set time within which a track should be validated (otherwise it is discarded)
         *
         * \param[in] sec_before_fake Time within which a track should be validated (otherwise it is discarded)
         */
        virtual void
        setSecBeforeFake(double sec_before_fake);

        /**
         * \brief Set time after which a visible track obtain NORMAL status
         *
         * \param[in] sec_remain_new Time after which a visible track obtain NORMAL status
         */
        virtual void
        setSecRemainNew(double sec_remain_new);

        /**
         * \brief Set minimum number of detection<->track associations needed for validating a track
         *
         * \param[in] detections_to_validate Minimum number of detection<->track associations needed for validating a track
         */
        virtual void
        setDetectionsToValidate(int detections_to_validate);

        /**
         * \brief Set flag stating if people detection confidence should be used in data association (true) or not (false)
         *
         * \param[in] detector_likelihood Flag stating if people detection confidence should be used in data association (true) or not (false)
         */
        virtual void
        setDetectorLikelihood(bool detector_likelihood);

        /**
         * \brief Set likelihood weights for data association
         *
         * \param[in] detector_weight Weight for detector likelihood
         * \param[in] motion_weight Weight for motion likelihood
         */
        virtual void
        setLikelihoodWeights(double detector_weight, double motion_weight);

        /**
         * \brief Set flag stating if people velocity should be used in motion term for data association
         *
         * \param[in] velocity_in_motion_term If true, people velocity is also used in motion term for data association
         * \param[in] acceleration_variance Acceleration variance (for Kalman Filter)
         * \param[in] position_variance Position variance (for Kalman Filter)
         */
        virtual void
        setVelocityInMotionTerm(bool velocity_in_motion_term, double acceleration_variance, double position_variance);

        /**
         * \brief Set acceleration variance (for Kalman Filter)
         *
         * \param[in] acceleration_variance Acceleration variance (for Kalman Filter)
         */
        virtual void
        setAccelerationVariance(double acceleration_variance);

        /**
         * \brief Set position variance (for Kalman Filter)
         *
         * \param[in] position_variance Position variance (for Kalman Filter)
         */
        virtual void
        setPositionVariance(double position_variance);

        /**
         * \brief Set gate distance for joint likelihood in data association
         *
         * \param[in] gate_distance Gate distance for joint likelihood in data association.
         */
        virtual void
        setGateDistance(double gate_distance);
    };

} /*namespace tracker*/

#endif /* TRACKER_TRACKER_3D_H_ */
