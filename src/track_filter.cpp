#include "track_filter.hpp"
// Next Steps:
// 1. Initialize motion model. One of the things I'm concerned about is the value of T in the A matrix from 3.27 in Blackman, which seems to assume a fixed time update rate.
// May be fine for now, but it means I'll have to force it to poll at that sample rate... maybe??
// 2. 
MotionModel::MotionModel(float dt=0.0, Eigen::Matrix2f Px0 = Eigen::Matrix2f::Identity(), 
                         Eigen::Matrix2f Py0 = Eigen::Matrix2f::Identity(), 
                         Eigen::Matrix2f Pz0 = Eigen::Matrix2f::Identity()) {
    float sigma_m2 = 0.1;

    A << 1.0, dt,
        0.0, 1.0;

    H << 1.0, 0.0,
         0.0, 1.0;

    Q << sigma_m2*(0.3333)*dt*dt*dt, sigma_m2*(0.5)*dt*dt,
         sigma_m2*(0.5)*dt*dt, sigma_m2*dt;

    //This R value is a guess... and probably wrong. need to tune it.
    R << 1.0, 0.0,
         0.0, 1.0;

    Kx = compute_kalman_gain(Px);
    Ky = compute_kalman_gain(Py);
    Kz = compute_kalman_gain(Pz);
};

Eigen::Matrix<float,6,1> MotionModel::state_correction(Eigen::Matrix<float,6,1>& measurement, Eigen::Matrix<float,6,1>& prev_state) {
    Eigen::Matrix<float,6,1> states;
    states << prev_state.segment<2>(0) + Kx*(measurement.segment<2>(0) - H*prev_state.segment<2>(0)), 
              prev_state.segment<2>(2) + Ky*(measurement.segment<2>(2) - H*prev_state.segment<2>(2)), 
              prev_state.segment<2>(4) + Kz*(measurement.segment<2>(4) - H*prev_state.segment<2>(4));

    Kx = compute_kalman_gain(Px);
    Px = (Eigen::Matrix2f::Identity() - Kx*H)*Px;

    Ky = compute_kalman_gain(Py);
    Py = (Eigen::Matrix2f::Identity() - Ky*H)*Py;

    Kz = compute_kalman_gain(Pz);
    Pz = (Eigen::Matrix2f::Identity() - Kz*H)*Pz;

    return states;
};

Eigen::Matrix<float,6,1>  MotionModel::state_update(Eigen::Matrix<float,6,1>& state) {
    Eigen::Matrix<float,6,1> states;
    states << A*state.segment<2>(0),
              A*state.segment<2>(2),
              A*state.segment<2>(4);
    return states;
};

void MotionModel::covariance_update() {
    Px = A*Px*A.transpose() + Q;
    Py = A*Py*A.transpose() + Q;
    Pz = A*Pz*A.transpose() + Q;
};

Eigen::Matrix<float,6,6> MotionModel::P() {
    Eigen::Matrix<float,6,6> output;
    output.block<2,2>(0,0) = Px;
    output.block<2,2>(2,2) = Py;
    output.block<2,2>(4,4) = Pz;
    return output;
}

Eigen::Matrix2f MotionModel::compute_kalman_gain(Eigen::Matrix2f& P) {
 Eigen::Matrix2f denominator = H*P*H.transpose() + R;
   return P*H.transpose()*denominator.inverse();
};

TrackFilter::TrackFilter() {

};

void TrackFilter::measurement_update(Eigen::Matrix<float,6,1>& measurement) {
    track.state = motion_model.state_correction(measurement, track.state);
    track.last_update_time = std::time(nullptr);
};

void TrackFilter::time_update() {
    track.state = motion_model.state_update(track.state);
    motion_model.covariance_update();
};

Track TrackFilter::get() {
    return track;
};

std::string TrackFilter::id() {
    return track.uuid;
};