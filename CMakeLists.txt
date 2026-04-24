cmake_minimum_required(VERSION 3.16)

project(Tracker VERSION 1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SRC
    src/main.cpp
    )

set(spdlog_DIR "./external/spdlog/install/lib/cmake/spdlog")
find_package(spdlog REQUIRED)

add_executable(${PROJECT_NAME} ${SRC})

target_include_directories(${PROJECT_NAME} PRIVATE 
    ./include
    ./external/eigen
    ./external/spdlog/include
    )
