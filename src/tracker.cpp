#include "tracker.hpp"

Tracker::Tracker() {

}

void Tracker::measurement_update(std::vector<Measurement> msgs) {
    //Data Association
    //Apply meaurement to a track
}

void Tracker::step() {
    //for each track filter, perform the time update and prune stale tracks

}
void Tracker::time_update(){
    //iterate through all current tracks and propogate the state and covariance.
}

void Tracker::prune_tracks() {
    // remove tracks that are stale
}