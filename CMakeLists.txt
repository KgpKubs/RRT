cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories("include")

set(SRCS src/Random_sampling.cpp
  src/RT_RRT_star.cpp)

set(PLANNERS planners)
add_library(${PLANNERS} SHARED ${SRCS})
