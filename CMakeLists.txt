cmake_minimum_required(VERSION 3.16)

project(Tracker VERSION 1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(CTest)

set(spdlog_DIR "./external/spdlog/install/lib/cmake/spdlog")
find_package(spdlog REQUIRED)

set(SRC
    src/tracker.cpp
    src/track_filter.cpp
    src/alpha_beta_filter.cpp
    src/adsb_interface.cpp
    )

add_library("lib_${PROJECT_NAME}" SHARED ${SRC})

add_executable(${PROJECT_NAME} 
                src/main.cpp)

target_include_directories("lib_${PROJECT_NAME}" PUBLIC 
    ./include
    ./external/eigen
    ./external/spdlog/include
    )

target_link_libraries(${PROJECT_NAME} PRIVATE 
            "lib_${PROJECT_NAME}")

if(BUILD_TESTING)
find_package(Catch2 REQUIRED)

set(TEST_FILES 
        ./test/test_motion_model.cpp
        ./test/test_track_filter.cpp)

add_executable(Tests ${TEST_FILES})

target_link_libraries(Tests PRIVATE 
            Catch2::Catch2WithMain
            "lib_${PROJECT_NAME}")
endif()