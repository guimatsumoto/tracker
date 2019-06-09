#ifndef TRACKER_KALMAN_FILTER_H_
#define TRACKER_KALMAN_FILTER_H_

#include <cmath>
#include <Eigen/Eigen>
#include "bayes/allFilters.hpp"

namespace tracker {

    /** \brief PredictModel Prediction model (linear state predict model) */
    class  PredictModel : public Bayesian_filter::Linear_predict_model {
    protected:
        /* \brief time step */
        const double dt_;

    public:
        /** \brief Constructor. */
        PredictModel(double dt, double acceleration_variance);

        /** \brief Destructor. */
        virtual ~PredictModel();
    };

    /** \brief ObserveModel Observation model (linear observation is additive uncorrelated model) */
    class  ObserveModel : public Bayesian_filter::Linear_uncorrelated_observe_model {
    protected:
        /** \brief Position variance. */
        double position_variance_;

    public:
        /** \brief Constructor. */
        ObserveModel(double position_variance, int ouput_dimension);

        /** \brief Destructor. */
        virtual ~ObserveModel();
    };

    /** \brief MahalanobisParameters2d Contains variables for bayesian estimation with state dimension = 2. */
    class  MahalanobisParameters2d {
    public:

        MahalanobisParameters2d() : SI(Bayesian_filter_matrix::Empty), x(0), y(0) {
            SI.resize(2, 2, false);
        }

        /** \brief Innovation covariance matrix. */
        Bayesian_filter::FM::SymMatrix SI;

        /** \brief Position x component. */
        double x;

        /** \brief Position y component. */
        double y;
    };

    /** \brief MahalanobisParameters4d Contains variables for bayesian estimation with state dimension = 4. */
    class  MahalanobisParameters4d {
    public:

        MahalanobisParameters4d() : SI(Bayesian_filter_matrix::Empty), x(0), y(0), vx(0), vy(0) {
            SI.resize(4, 4, false);
        }

        /** \brief Innovation covariance matrix. */
        Bayesian_filter::FM::SymMatrix SI;

        /** \brief Position x component. */
        double x;

        /** \brief Position y component. */
        double y;

        /** \brief Velocity x component. */
        double vx;

        /** \brief Velocity y component. */
        double vy;
    };

    /** \brief KalmanFilter provides methods for bayesian estimation with Kalman Filter. */
    class  KalmanFilter {
    protected:

        /** \brief Time interval.*/
        double dt_;

        /** \brief Scale factor for computing depth noise variance.*/
        double depth_multiplier_;

        /** \brief Position variance. */
        double position_variance_;

        /** \brief Acceleration variance.*/
        double acceleration_variance_;

        /** \brief State/output dimension.*/
        int output_dimension_;

        Bayesian_filter::Unscented_scheme* filter_;

        /** \brief Prediction model. */
        PredictModel* predict_model_;

        /** \brief Observation model. */
        ObserveModel* observe_model_;

    public:

        /** \brief Constructor. */
        KalmanFilter(double dt, double position_variance, double acceleration_variance, int output_dimension);

        /** \brief Constructor initializing a new KalmanFilter with another one. */
        KalmanFilter(const KalmanFilter& orig);

        /** \brief Overload of = operator for copying KalmanFilter objects. */
        KalmanFilter& operator=(const KalmanFilter& orig);

        /** \brief Destructor. */
        virtual ~KalmanFilter();

        /**
         * \brief Filter initialization procedure.
         *
         * \param[in] x Position x component.
         * \param[in] y Position y component.
         * \param[in] distance Distance from the sensor.
         * \param[in] velocity_in_motion_term If true, both target position and velocity constitute the output vector.
         */
        virtual void
        init(double x, double y, double distance, bool velocity_in_motion_term);

        /**
         * \brief Prediction step.
         */
        virtual void
        predict();

        /**
         * \brief Prediction step.
         *
         * \param[out] x Position x component.
         * \param[out] y Position y component.
         * \param[out] vx Velocity x component.
         * \param[out] vy Velocity y component.
         */
        virtual void
        predict(double& x, double& y, double& vx, double& vy);

        /**
         * \brief Update step.
         */
        virtual void
        update();

        /**
         * \brief Update step.
         *
         * \param[in] x Position x component.
         * \param[in] y Position y component.
         * \param[in] distance Distance from the sensor.
         */
        virtual void
        update(double x, double y, double distance);

        /**
         * \brief Update step.
         *
         * \param[in] x Position x component.
         * \param[in] y Position y component.
         * \param[in] vx Velocity x component.
         * \param[in] vy Velocity y component.
         * \param[in] distance Distance from the sensor.
         */
        virtual void
        update(double x, double y, double vx, double vy, double distance);

        /**
         * \brief Get filter state.
         *
         * \param[out] x Position x component.
         * \param[out] y Position y component.
         * \param[out] vx Velocity x component.
         * \param[out] vy Velocity y component.
         */
        virtual void
        getState(double& x, double& y, double& vx, double& vy);

        /**
         * \brief Get filter state.
         *
         * \param[out] x Position x component.
         * \param[out] y Position y component.
         */
        virtual void
        getState(double& x, double& y);

        /**
         * \brief Obtain variables for bayesian estimation with output dimension = 2.
         *
         * \param[out] mp Object of class MahalanobisParameters2d.
         */
        virtual void
        getMahalanobisParameters(MahalanobisParameters2d& mp);

        /**
         * \brief Obtain variables for bayesian estimation with output dimension = 4.
         *
         * \param[out] mp Object of class MahalanobisParameters4d.
         */
        virtual void
        getMahalanobisParameters(MahalanobisParameters4d& mp);

        /**
         * \brief Compute Mahalanobis distance between measurement and target predicted state.
         *
         * \param[in] x Input x position.
         * \param[in] y Input y position.
         * \param[in] mp Object of class MahalanobisParameters2d.
         *
         * \return Mahalanobis distance between measurement (x,y) and target predicted state.
         */
        static double
        performMahalanobisDistance(double x, double y, const MahalanobisParameters2d& mp);

        /**
         * \brief Compute Mahalanobis distance between measurement and target predicted state.
         *
         * \param[in] x Input x position.
         * \param[in] y Input y position.
         * \param[in] vx Input x velocity.
         * \param[in] vy Input y velocity.
         * \param[in] mp Object of class MahalanobisParameters4d.
         *
         * \return Mahalanobis distance between measurement (x,y,vx,vy) and target predicted state.
         */
        static double
        performMahalanobisDistance(double x, double y, double vx, double vy, const MahalanobisParameters4d& mp);

        /**
         * \brief Get filter innovation covariance.
         *
         * \return innovation covariance matrix.
         */
        virtual Bayesian_filter::FM::SymMatrix
        getInnovationCovariance();

        /**
         * \brief Set prediction model.
         *
         * \param[in] acceleration_variance Acceleration variance.
         */
        virtual void
        setPredictModel(double acceleration_variance);

        /**
         * \brief Set observation model.
         *
         * \param[in] position_variance Position variance.
         */
        virtual void
        setObserveModel(double position_variance);

    };

} /*namespace tracker*/

#endif /* TRACKER_KALMAN_FILTER_H_ */
