cmake_minimum_required(VERSION 3.10)
project(MyKalmanFilterProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_executable(filter 
    kalman.cpp 
    extendedKalman.cpp 
    extendedKalman.hpp
)

target_link_libraries(filter PRIVATE Eigen3::Eigen)

