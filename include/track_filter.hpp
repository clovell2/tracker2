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
};

class TrackFilter {
    TrackFilter(std::string id, std::unique_ptr<MotionModel> motion_model);
    void measurement_update(Eigen::Matrix<float,6,1>& measurement);
    void time_update();
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
