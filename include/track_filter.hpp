#pragma once
#include <Eigen/Dense>
#include <ctime>

// State data of the track filter
struct Track {
    public:
        // Unique ID of Track
        std::string uuid;
        // Unix Timestamp of last measurement update
        time_t last_update_time;
        // State Data of Track
        Eigen::Matrix<float,6,1> state;
};
// IDEA: Make this an abstract class to allow different motion models to be implemented??
class MotionModel {
    public:
        MotionModel(float dt, Eigen::Matrix2f Px0, Eigen::Matrix2f Py0, Eigen::Matrix2f Pz0);
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
        Eigen::Matrix<float,6,1> state_correction(Eigen::Matrix<float,6,1>& measurement, Eigen::Matrix<float,6,1>& prev_state);
        /*
        * Computes the state propogation portion of a Kalman filter.
        * Implemented with 6 states to cover the position and velocity in a cartesian coordinate system.
        * 
        * @param state the current track position and velocity.
        * 
        * @return The new track state estimate.
        * 
        */
        Eigen::Matrix<float,6,1> state_update(Eigen::Matrix<float,6,1>& state);
        /*
        * Propogates the covariance of a Kalman filter.
        */
        void covariance_update();
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

class TrackFilter {
    TrackFilter();
    void measurement_update(Eigen::Matrix<float,6,1>& measurement);
    void time_update();
    // Return track information
    Track get();
    // Return track id.
    std::string id();
    private:
        // Structure containing track information
        Track track;
        // Underlying motion model describing the track dynamics
        MotionModel motion_model;
};
