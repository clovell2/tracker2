#include <catch2/catch_test_macros.hpp>
#include "track_filter.hpp"
#include <iostream>
/*
* The purpose of the track filter is to provide an interface between the 
* motion model for a specific vehicle and external systems. This can be 
* verified to be correctly implemented by:
* 1. checking a track can be initialized with a valid motion model at an initial state
* 2. checking a track can be updated with a measurement
* 3. checking a track can be stepped in time
* 4. checking the motion model can be changed without modifying the underlying track states
*/
class MockMotionModel : public MotionModel {
    public:
    Eigen::Matrix<float,6,1> state_correction(Eigen::Matrix<float,6,1>& measurement, Eigen::Matrix<float,6,1>& prev_state) override {
        return measurement + prev_state;
    }
    Eigen::Matrix<float,6,1> state_update(Eigen::Matrix<float,6,1>& state) override {
        return 2.0*state;
    }
    Eigen::Matrix<float,6,6> covariance_update() override{
        Eigen::Matrix<float,6,6> ones = Eigen::Matrix<float,6,6>::Identity();
        P += ones;
        return P;
    }
    Eigen::Matrix<float,6,6> P = Eigen::Matrix<float,6,6>::Zero();
};

TEST_CASE( "Track Filter Initialization", "[track_filter]" ) {
    std::string id = "867-5309";
    Eigen::Matrix<float,6,1> initial_states;
    initial_states << 1.0, 2.0, -1.0, 3.14, 2.7, -6;
    TrackFilter test_filter = TrackFilter(id, std::make_unique<MockMotionModel>(), initial_states);
    Track init_track = test_filter.get();

    // Check the name is correct
    CHECK(test_filter.id() == id);
    CHECK(init_track.uuid == id);
    // Check the initial states were registered
    CHECK(init_track.state == initial_states);
    // Check the last update time is not at epoch zero
    CHECK(init_track.last_update_time != 0);
}

TEST_CASE( "Track Filter Measurement Update", "[track_filter]" ) {
    std::string id = "867-5309";
    Eigen::Matrix<float,6,1> initial_states, measurement;
    initial_states << 1.0, 2.0, -1.0, 3.14, 2.7, -6;
    measurement << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    TrackFilter test_filter = TrackFilter(id, std::make_unique<MockMotionModel>(), initial_states);
    Track init_track = test_filter.get();

    // Apply measurement update
    test_filter.measurement_update(measurement);
    Track test_track = test_filter.get();

    // Check that the state correction function was applied
    Eigen::Matrix<float,6,1> expected_output =  measurement + init_track.state;
    CHECK(test_track.state == expected_output);
    // Check that the update time was changed
    CHECK(test_track.last_update_time >= init_track.last_update_time);
}

TEST_CASE( "Track Filter Time Update", "[track_filter]" ) {
    Eigen::Matrix<float,6,1> initial_states;
    initial_states << 1.0, 2.0, -1.0, 3.14, 2.7, -6;
    TrackFilter test_filter = TrackFilter("867-5309", std::make_unique<MockMotionModel>(), initial_states);
    Track init_track = test_filter.get();

    // Apply Time Update
    test_filter.time_update();
    Track test_track = test_filter.get();

    // Check the states were updated
    Eigen::Matrix<float,6,1> expected_output = 2.0*init_track.state;
    CHECK(test_track.state == expected_output);
    // Check the covariance was updated
    Eigen::Matrix<float,6,6> expected_output2 = Eigen::Matrix<float,6,6>::Identity();
    CHECK(test_track.covar == expected_output2);
}

TEST_CASE( "Track Filter Motion Model Switching", "[track_filter]" ) {
    std::unique_ptr<MockMotionModel> model1 = std::make_unique<MockMotionModel>();
    std::unique_ptr<MockMotionModel> model2 = std::make_unique<MockMotionModel>();
    Eigen::Matrix<float,6,1> initial_states;
    initial_states << 1.0, 2.0, -1.0, 3.14, 2.7, -6;
    TrackFilter test_filter = TrackFilter("id", std::move(model1), initial_states);
    test_filter.switch_motion_model(std::move(model2));

    CHECK(test_filter.get().state == initial_states);
    
    // Not sure how to check the pointer without exposing the data, which I don't really want to do
    SKIP();
    CHECK(false);
}