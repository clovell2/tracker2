#include "main.hpp"

std::atomic<bool> is_running(true);
void handle_sigint(int) {
    is_running = false;
}

int main(){
    // Create interrupt handler
    std::signal(SIGINT, handle_sigint);

    //Create ADSB hardware interface
    ADSBInterface adsb; 

    // Initialize hardware
    spdlog::info("Hardware Started.");
    adsb.init_interface();

    // Initialize tracker
    spdlog::info("Tracker Started.");

    while(is_running){
        // poll the hardware buffer for new ADS-B messages
        adsb.write_buffer();

        // measurement update
        //tracker.measurement_update(adsb_to_measurement(adsb.read_buffer()));

        // prune tracks
        //tracker.prune_tracks();

        // time update
        //tracker.time_update();
        
        // Wait until the next time step
    }
    spdlog::info("Stopping Tracker.");
    return 0;
}