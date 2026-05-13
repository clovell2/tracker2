#pragma once
#include <Eigen/Dense>
// The base class for track motion models.
class MotionModel {
    public:
        /*
        * Meaurement update of the motion model.
        *
        * @param measurement the new state measurement to appy to the motion model
        * @param prev_state the previous motion model state vector.
        * 
        * @return the new state from the motion model
        * 
        */
        virtual Eigen::Matrix<float,6,1> state_correction(Eigen::Matrix<float,6,1>& measurement, Eigen::Matrix<float,6,1>& prev_state) = 0;
        virtual Eigen::Matrix<float,6,1> state_update(Eigen::Matrix<float,6,1>& state) = 0;
        virtual Eigen::Matrix<float,6,6> covariance_update() = 0;
};

// Alpha-Beta filter. Derived from the MotionModel base class
class AlphaBetaFilter: public MotionModel {
    public:
        /*
        * Constructor for a basic Alpha-Beta motion model
        * 
        * @param dt timestep of the motion model.
        * @param Px0 initial X coordinate covariance matrix for position and velocity
        * @param Py0 initial y coordinate covariance matrix for position and velocity
        * @param Pz0 initial z coordinate covariance matrix for position and velocity
        */
        AlphaBetaFilter(float dt, Eigen::Matrix2f Px0, Eigen::Matrix2f Py0, Eigen::Matrix2f Pz0);
        /*
        * Computes the state correction portion of a Kalman filter.
        * Implemented with 6 states to cover the position and velocity in a cartesian coordinate system.
        * 
        * @param state the measurement state vector containing new information of the track position and velocity
        * @param prev_state the previous estimate of the track state
        * 
        * @return The new track state estimate.
        * 
        */
        Eigen::Matrix<float,6,1> state_correction(Eigen::Matrix<float,6,1>& measurement, Eigen::Matrix<float,6,1>& prev_state) override;
        /*
        * Computes the state propogation portion of a Kalman filter.
        * Implemented with 6 states to cover the position and velocity in a cartesian coordinate system.
        * 
        * @param state the current track position and velocity.
        * 
        * @return The new track state estimate.
        * 
        */
        Eigen::Matrix<float,6,1> state_update(Eigen::Matrix<float,6,1>& state) override;
        /*
        * Propogates the covariance of a Kalman filter.
        * 
        * @return The new covariance matrix
        */
        Eigen::Matrix<float,6,6> covariance_update() override;
        /*
        * Returns the Covariance Matrix of the system
        *
        * @return the 6 by 6 matrix of covariance terms of the states.
        */
        Eigen::Matrix<float,6,6> P();

    private:
        /*
        * Helper function to compute the Kalman Gain of a covariance matrix
        *
        * @param the 2 by 2 covariance matrix of a single axis 
        * 
        * @return the Kalman gain matrix
        */
        Eigen::Matrix2f compute_kalman_gain(Eigen::Matrix2f& P);

        // State Transition Matrix
        Eigen::Matrix<float,2,2> A;
        // Process Noise Matrix
        Eigen::Matrix<float,2,2> Q;
        //Measurement Matrix
        Eigen::Matrix<float,2,2> H;
        // Measurement Noise Matrix
        Eigen::Matrix<float,2,2> R;
        // Kalman Gain Matrix (x)
        Eigen::Matrix<float,2,2> Kx;
        // Kalman Gain Matrix (y)
        Eigen::Matrix<float,2,2> Ky;
        // Kalman Gain Matrix (z)
        Eigen::Matrix<float,2,2> Kz;
        // Covariance Matrix (x)
        Eigen::Matrix<float,2,2> Px;
        // Covariance Matrix (y)
        Eigen::Matrix<float,2,2> Py;
        // Covariance Matrix (z)
        Eigen::Matrix<float,2,2> Pz;
};
