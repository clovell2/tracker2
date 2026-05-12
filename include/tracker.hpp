#pragma once
#include <vector>
#include "track_filter.hpp"

// inputs to the Tracker
class Measurement {

};

class Tracker{
    Tracker();
    void measurement_update(std::vector<Measurement> msgs);
    void step();
    void time_update();
    void prune_tracks();
    std::vector<Track> tracks;
};