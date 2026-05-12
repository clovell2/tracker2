cmake_minimum_required(VERSION 3.16)

project(Tracker VERSION 1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SRC
    src/main.cpp
    src/tracker.cpp
    src/track_filter.cpp
    src/adsb_interface.cpp
    )

set(spdlog_DIR "./external/spdlog/install/lib/cmake/spdlog")
find_package(spdlog REQUIRED)

add_executable(${PROJECT_NAME} ${SRC})

target_include_directories(${PROJECT_NAME} PRIVATE 
    ./include
    ./external/eigen
    ./external/spdlog/include
    )

set(BUILD_TESTING OFF)
# this is a hacky way of disabling testing. Since Catch2 is a submodule I need to manually disable building the executable
if(BUILD_TESTING==ON)
    find_package(Catch2 REQUIRED)
    file(GLOB_RECURSE TEST_FILES 
        ./test/test_*.cpp)
    add_executable(Tests ${TEST_FILES})
    target_link_libraries(Tests PRIVATE Catch2::Catch2WithMain)
endif()