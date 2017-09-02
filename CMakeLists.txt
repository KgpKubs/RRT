cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories("include")

set(SRCS src/Random_sampling.cpp)

set(PLANNERS planners)
add_library(${PLANNERS} SHARED ${SRCS})

add_executable(target test/main.cpp)
target_link_libraries(target ${PLANNERS})
