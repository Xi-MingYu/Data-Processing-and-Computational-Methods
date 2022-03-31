cmake_minimum_required(VERSION 3.21)
project(Filter)

set(CMAKE_CXX_STANDARD 14)

include_directories(
        ./Filter/Kalman_Filter
        ./eigen
)

add_executable(Filter main.cpp Filter/Kalman_Filter/Kalman_Filter.cpp)
