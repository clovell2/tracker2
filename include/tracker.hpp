#pragma once
#include <vector>
#include "track_filter.hpp"
#include <ctime>
#include "uuid.hpp"

// inputs to the Tracker
class Measurement {

};

class Tracker{
    public:
        Tracker();
        void measurement_update(std::vector<Measurement> msgs);
        // Prune track vector of stale tracks and propogate tracks in time
        void step();
        /*
        * Get all active tracks.
        * 
        * @return vector of Track elements
        */
        std::vector<Track> get_all();
    private:
        // Create new track
        void create_new_track();
        // Propogates all tracks state and covariance.
        void time_update();
        // Remove tracks that are stale or have too uncertain states.
        void prune_tracks();
    
        std::vector<TrackFilter> tracks;
        time_t coast_threshold;
        float covariance_threshold;
        float dt;
};