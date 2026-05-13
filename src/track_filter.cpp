#include "track_filter.hpp"

TrackFilter::TrackFilter(std::string id, std::unique_ptr<MotionModel> motion_model) {
    id = id;
    motion_model = std::move(motion_model);
};

void TrackFilter::measurement_update(Eigen::Matrix<float,6,1>& measurement) {
    track.state = motion_model->state_correction(measurement, track.state);
    track.last_update_time = std::time(nullptr);
};

void TrackFilter::time_update() {
    track.state = motion_model->state_update(track.state);
    motion_model->covariance_update();
};

void TrackFilter::switch_motion_model(std::unique_ptr<MotionModel> new_model) {
    motion_model.release();
    motion_model = std::move(new_model);
};

Track TrackFilter::get() {
    return track;
};

std::string TrackFilter::id() {
    return track.uuid;
};