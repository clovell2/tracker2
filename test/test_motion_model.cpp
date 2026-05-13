#include <catch2/catch_test_macros.hpp>
#include "motion_model.hpp"
#include <iostream>
// The purpose of the motion model is to provide a representation of a vehicles
// dynamics in space over time. That this is implemented correctly can be verified by:
// 1. checking that the states of the model can be updated in time, and they maintain kinematic consistency
// 2. checking that the uncertainty of the states increases with time without new measuremeents
// 3. checking that the addition of new measurements does not violate kinematic consistency and
// 4. checking that the addition of new measurements decreases the uncertainty of state values

TEST_CASE( "Motion Model State Update", "[motion_model][alpha_beta_filter]" ) {
    Eigen::Matrix<float,6,1> output1, output2;
    // x=0, u=1, y=0, v=-1, z=0, w=0.5
    Eigen::Matrix<float,6,1> state = {0.0,1.0,0.0,-1.0,0.0,0.5};
    Eigen::Matrix<float,2,2> Px, Py, Pz; 
    Px << 1.0, 0.0, 0.0, 1.0;
    Py << 1.0, 0.0, 0.0, 1.0;
    Pz << 1.0, 0.0, 0.0, 1.0;
    AlphaBetaFilter test_model = AlphaBetaFilter(0.01, Px, Py, Pz);
    output1 = test_model.state_update(state);
    output2 = test_model.state_update(output1);

    // Check we're not adding/removing states
    REQUIRE(output1.size()==state.size());
    REQUIRE(output2.size()==state.size());
    // First Time Step
    CHECK(output1[0]>=state[0]);
    CHECK(output1[1]==state[1]);
    CHECK(output1[2]<=state[2]);
    CHECK(output1[3]==state[3]);
    CHECK(output1[4]>=state[4]);
    CHECK(output1[5]==state[5]);
    //Second Time Step
    CHECK(output2[0]>=output1[0]);
    CHECK(output2[1]==output1[1]);
    CHECK(output2[2]<=output1[2]);
    CHECK(output2[3]==output1[3]);
    CHECK(output2[4]>=output1[4]);
    CHECK(output2[5]==output1[5]);
}

TEST_CASE( "Motion Model Uncertainty Update", "[motion_model][alpha_beta_filter]" ) {
    Eigen::Matrix<float,6,6> output;
    Eigen::Matrix<float,2,2> Px, Py, Pz; 
    Px << 1.0, 0.0, 0.0, 1.0;
    Py << 1.0, 0.0, 0.0, 1.0;
    Pz << 1.0, 0.0, 0.0, 1.0;
    AlphaBetaFilter test_model = AlphaBetaFilter(0.01, Px, Py, Pz);
    test_model.covariance_update();
    output = test_model.P();
    //Let's see if the covariance terms have increased
    CHECK((output.block<2,2>(0,0).array() >= Px.array()).all());
    CHECK((output.block<2,2>(2,2).array() >= Py.array()).all());
    CHECK((output.block<2,2>(4,4).array() >= Pz.array()).all());
}

TEST_CASE( "Motion Model New Measurement", "[motion_model][alpha_beta_filter]" ) {
    Eigen::Matrix<float,6,1> output1;
    Eigen::Matrix<float,6,6> output2;
    Eigen::Matrix<float,6,1> initial_state = {0.0,1.0,0.0,-1.0,0.0,0.5};
    Eigen::Matrix<float,6,1> measurement = {1.0,1.0,1.0,0.0,2.0,4.0};
    Eigen::Matrix<float,2,2> Px, Py, Pz; 
    Px << 1.0, 0.0, 0.0, 1.0;
    Py << 1.0, 0.0, 0.0, 1.0;
    Pz << 1.0, 0.0, 0.0, 1.0;
    AlphaBetaFilter test_model = AlphaBetaFilter(0.01, Px, Py, Pz);
    output1 = test_model.state_correction(measurement, initial_state);
    output2 = test_model.P();
    // Check we're not adding/removing states
    REQUIRE(output1.size()==initial_state.size());
    // First Time Step
    CHECK(output1[0]>=initial_state[0]);
    CHECK(output1[1]==initial_state[1]);
    CHECK(output1[2]>=initial_state[2]);
    CHECK(output1[3]>=initial_state[3]);
    CHECK(output1[4]>=initial_state[4]);
    CHECK(output1[5]>=initial_state[5]);

    //Let's see if the covariance terms have decreased
    CHECK((output2.block<2,2>(0,0).array() <= Px.array()).all());
    CHECK((output2.block<2,2>(2,2).array() <= Py.array()).all());
    CHECK((output2.block<2,2>(4,4).array() <= Pz.array()).all());
}