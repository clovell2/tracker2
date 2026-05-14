#pragma once
#include <Eigen/Dense>
#include <ctime>
#include <memory>
#include "motion_model.hpp"

// State data of the track filter
struct Track {
    public:
        // Unique ID of Track
        std::string uuid;
        // Unix Timestamp of last measurement update
        time_t last_update_time;
        // State Data of Track
        Eigen::Matrix<float,6,1> state;
        // Covariance Data of Track
        Eigen::Matrix<float,6,6> covar;
};

// Interface for the motion model to interact with track information
class TrackFilter {
    public:
        /*
        * @param id the unique identification string for the track
        * @param track_motion_model the type of motion model the track will use
        * @param initial_states the initial states of the track nominally: 
        *       - x position [0]
        *       - x velocity [1] 
        *       - y position [2]
        *       - y velocity [3]
        *       - z position [4]
        *       - z velocity [5]
        */
        TrackFilter(std::string id, std::unique_ptr<MotionModel> track_motion_model, Eigen::Matrix<float,6,1> initial_states);
        /*
        * Apply a measurement to a track
        *
        * @param measurement 6 by 1 matrix representing the x,u,y,v,z,w states of a vehicle
        */
        void measurement_update(Eigen::Matrix<float,6,1>& measurement);
        // Update propogate track states and covariance.
        void time_update();
        /*
        * Switch motion model of a track to a different type.
        *
        * @param new_model smart pointer to a new MotionModel
        */
        void switch_motion_model(std::unique_ptr<MotionModel> new_model);
        // Return track information
        Track get();
        // Return track id.
        std::string id();
    private:
        // Structure containing track information
        Track track;
        // Underlying motion model describing the track dynamics
        std::unique_ptr<MotionModel> motion_model;
};
