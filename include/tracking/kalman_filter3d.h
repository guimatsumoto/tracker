#ifndef TRACKING_KALMAN_FILTER_3D_H_
#define TRACKING_KALMAN_FILTER_3D_H_
#include "sl_core/ai/ai_release.hpp"

#include <cmath>
#include <Eigen/Eigen>
#include "sl_core/ai/skeleton/bayes/allFilters.hpp"

namespace zed_tracking {

    // Linear state predict model

    class  PredictModel3D : public Bayesian_filter::Linear_predict_model {
    protected:
        // Update time step
        const double dt_;

    public:
        PredictModel3D(double dt, double acceleration_variance);

        virtual ~PredictModel3D();
    };

    // Additive uncorrelated linear observation model

    class  ObserveModel3D : public Bayesian_filter::Linear_uncorrelated_observe_model {
    protected:
        // Position variance
        double position_variance_;

    public:
        ObserveModel3D(double position_variance, int ouput_dimension);

        virtual ~ObserveModel3D();
    };


    // Parameters for bayesian estimation with state dimension = 2. */

    class  MahalanobisParameters3d {
    public:

        MahalanobisParameters3d() : SI(Bayesian_filter_matrix::Empty), x(0), y(0), z(0) {
            SI.resize(3, 3, false);
        }

        // Innovation covariance matrix.
        Bayesian_filter::FM::SymMatrix SI;

        // Position x component.
        double x;

        // Position y component.
        double y;

        // Position z component.
        double z;
    };

    // Parameters for bayesian estimation with state dimension = 4.

    class  MahalanobisParameters6d {
    public:

        MahalanobisParameters6d() : SI(Bayesian_filter_matrix::Empty), x(0), y(0), z(0), vx(0), vy(0), vz(0) {
            SI.resize(6, 6, false);
        }

        // Innovation covariance matrix.
        Bayesian_filter::FM::SymMatrix SI;

        // Position x component.
        double x;

        // Position y component.
        double y;

        // Position y component.
        double z;

        // Velocity x component.
        double vx;

        // Velocity y component.
        double vy;

        // Velocity y component.
        double vz;
    };

    // KalmanFilter3D provides methods for bayesian estimation with Kalman Filter.

    class  KalmanFilter3D {
    protected:

        // Update time step
        double dt_;

        // Scale factor for computing depth noise variance.
        double depth_multiplier_;

        // Position variance.
        double position_variance_;

        // Acceleration variance.
        double acceleration_variance_;

        // State/output dimension.
        int output_dimension_;

        // The actual filter class, predicting and updating upon the Prediction and Obervation models
        Bayesian_filter::Unscented_scheme* filter_;

        // Prediction model.
        PredictModel3D* predict_model_;

        // Observation model.
        ObserveModel3D* observe_model_;

    public:

        // Constructor.
        KalmanFilter3D(double dt, double position_variance, double acceleration_variance, int output_dimension);

        // Constructor initializing a new KalmanFilter with another one.
        KalmanFilter3D(const KalmanFilter3D& orig);

        // Overload of = operator for copying KalmanFilter objects.
        KalmanFilter3D& operator=(const KalmanFilter3D& orig);

        // Destructor.
        virtual ~KalmanFilter3D();

        /**
         * \brief Filter initialization procedure.
         *
         * \param[in] x Position x component.
         * \param[in] y Position y component.
         * \param[in] distance Distance from the sensor.
         * \param[in] velocity_in_motion_term If true, both target position and velocity constitute the output vector.
         */
        virtual void
        init(double x, double y, double z, double distance, bool velocity_in_motion_term);

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
        predict(double& x, double& y, double& z, double& vx, double& vy, double& vz);

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
        update(double x, double y, double z, double distance);

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
        update(double x, double y, double z, double vx, double vy, double vz, double distance);

        /**
         * \brief Get filter state.
         *
         * \param[out] x Position x component.
         * \param[out] y Position y component.
         * \param[out] vx Velocity x component.
         * \param[out] vy Velocity y component.
         */
        virtual void
        getState(double& x, double& y, double& z, double& vx, double& vy, double& vz);

        /**
         * \brief Get filter state.
         *
         * \param[out] x Position x component.
         * \param[out] y Position y component.
         */
        virtual void
        getState(double& x, double& y, double& z);

        /**
         * \brief Obtain variables for bayesian estimation with output dimension = 2.
         *
         * \param[out] mp Object of class MahalanobisParameters2d.
         */
        virtual void
        getMahalanobisParameters(MahalanobisParameters3d& mp);

        /**
         * \brief Obtain variables for bayesian estimation with output dimension = 4.
         *
         * \param[out] mp Object of class MahalanobisParameters4d.
         */
        virtual void
        getMahalanobisParameters(MahalanobisParameters6d& mp);

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
        performMahalanobisDistance(double x, double y, double z, const MahalanobisParameters3d& mp);

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
        performMahalanobisDistance(double x, double y, double z, double vx, double vy, double vz, const MahalanobisParameters6d& mp);

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

} /*namespace zed_tracking*/

#endif /* TRACKING_KALMAN_FILTER_3D_H_ */
