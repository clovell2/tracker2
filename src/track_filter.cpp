#include "track_filter.hpp"
// Next Steps:
// 1. Initialize motion model. One of the things I'm concerned about is the value of T in the A matrix from 3.27 in Blackman, which seems to assume a fixed time update rate.
// May be fine for now, but it means I'll have to force it to poll at that sample rate... maybe??
// 2. 
MotionModel::MotionModel() {
    float T = 0.01; //dt
    float sigma_m2 = 0.1;

    A << 1.0, T,
        0.0, 1.0;

    H << 1.0, 0.0,
         0.0, 1.0;

    Q << sigma_m2*(0.3333)*T*T*T, sigma_m2*(0.5)*T*T,
         sigma_m2*(0.5)*T*T, sigma_m2*T;
    //This R value is a guess... and probably wrong. need to tune it.
    R << 1.0, 0.0,
         0.0, 1.0;

};

Eigen::Matrix<float,6,1> MotionModel::state_update(Eigen::Matrix<float,6,1> state, Eigen::Matrix<float,6,1> prev_state) {
    Eigen::Matrix<float,6,1> states;
    states << A*prev_state.segment<2>(0) + Kx*(state.segment<2>(0) - H*prev_state.segment<2>(0)), 
              A*prev_state.segment<2>(2) + Ky*(state.segment<2>(2) - H*prev_state.segment<2>(2)), 
              A*prev_state.segment<2>(4) + Kz*(state.segment<2>(4) - H*prev_state.segment<2>(4));
    return states;

};

void MotionModel::covariance_update() {
    Eigen::Matrix<float,2,2> denominator = H*Px*H.transpose() + R;
    Kx = A*Px*H.transpose()*denominator.inverse();
    Px = (A - Kx*H)*Px*A.transpose() + Q;

    denominator = H*Py*H.transpose() + R;
    Ky = A*Py*H.transpose()*denominator.inverse();
    Py = (A - Ky*H)*Py*A.transpose() + Q;

    denominator = H*Pz*H.transpose() + R;
    Kz = A*Pz*H.transpose()*denominator.inverse();
    Pz = (A - Kz*H)*Pz*A.transpose() + Q;
};

TrackFilter::TrackFilter() {
    
};

void TrackFilter::measurement_update(Eigen::Matrix<float,6,1> measurement) {
    track.state = motion_model.state_update(measurement, track.state);
    track.last_update_time = std::time(nullptr);
};

void TrackFilter::time_update() {
    motion_model.covariance_update();
};

Track TrackFilter::get() {
    return track;
};

std::string TrackFilter::id() {
    return track.uuid;
};