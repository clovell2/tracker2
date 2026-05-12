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
// TODO: Make this an abstract class to allow different motion models to be implemented
class MotionModel {
    public:
        MotionModel();
        /*
        * Computes the state update portion of a Kalman filter.
        * Implemented with 6 states to cover the position and velocity in a cartesian coordinate system.
        * 
        * @param state the measurement state vector containing new information of the track position and velocity
        * @param prev_state the previous estimate of the track state
        * 
        * @return The new track state estimate.
        * 
        */
        Eigen::Matrix<float,6,1> state_update(Eigen::Matrix<float,6,1> state, Eigen::Matrix<float,6,1> prev_state);
        /*
        * Updates the covariance and kalman gain update of a Kalman filter.
        */
        void covariance_update();
    private:
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
    void measurement_update(Eigen::Matrix<float,6,1> measurement);
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
