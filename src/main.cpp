#include "main.hpp"

std::atomic<bool> is_running(true);
void handle_sigint(int) {
    is_running = false;
}

int main(){
    // Create interrupt handler
    std::signal(SIGINT, handle_sigint);

    // Initialize hardware
    spdlog::info("Hardware Started.");

    // Initialize tracker
    spdlog::info("Tracker Started.");

    Eigen::Matrix2d m;

    m << 1, 2,
         3, 4;

    std::cout << m << std::endl;
    
    //while(is_running){
    //      poll the hardware buffer for new ADS-B messages
    //      measurement update
    //      prune tracks
    //      time update
    //}
    spdlog::info("Stopping Tracker.");
    return 0;
}