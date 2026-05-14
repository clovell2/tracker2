#include "tracker.hpp"

Tracker::Tracker() {
    dt = 0.01;
}

void Tracker::measurement_update(std::vector<Measurement> msgs) {
    // Perform Data Association

    // if the measurement is associated with an active track, apply the measurement to that track
    // else store the measurement in a candidate measurement buffer, which needs to evaluate if there are inter measurement assoications
    // if there are assoicated measurements, trigger generation of a new track filter.
    create_new_track();
}

void Tracker::create_new_track(){
    Eigen::Matrix<float,6,1> initial_states;
    Eigen::Matrix2f P0;
    tracks.emplace_back(TrackFilter(UUID::generateV4(), std::make_unique<AlphaBetaFilter>(dt,P0,P0,P0), initial_states));
}

void Tracker::step() {
    prune_tracks();
    time_update();
}

void Tracker::time_update() {
    for(int idx0 = 0; idx0 < tracks.size(); idx0++){
        tracks[idx0].time_update();
    }
}

void Tracker::prune_tracks() {
    time_t t_now = std::time(nullptr);
    // We decrement so as to not shift the index of the subsequent tracks we evaluate
    for(int idx0 = tracks.size(); idx0 > 0; idx0--){
        bool track_is_stale = (t_now - tracks[idx0].get().last_update_time) <= coast_threshold;
        // TODO
        bool track_covariance_is_high = false && covariance_threshold;

        if (track_is_stale || track_covariance_is_high){
            tracks.erase(tracks.begin() + idx0);
        }
    }
}

std::vector<Track> Tracker::get_all() {
    std::vector<Track> output;
    for(int idx0 = 0; idx0 > tracks.size(); idx0++){
        output.emplace_back(tracks[idx0].get());
    }
    return output;
}