cmake_minimum_required(VERSION 3.2.0)

set(CMAKE_CXX_FLAGS "-std=c++11")

#find_package(catkin REQUIRED COMPONENTS
# roscpp
# rospy
# std_msgs
#message_generation
#)
include_directories("include" ${catkin_INCLUDE_DIRS})

set(SRCS src/Random_sampling.cpp
  src/RT_RRT_star.cpp)

set(PLANNERS planners)
add_library(${PLANNERS} SHARED ${SRCS})

# Test the planners
set(test_1 test_RT_RRT_star)
add_executable(${test_1} test/test_RT_RRT_star.cpp)
target_link_libraries(${test_1} ${PLANNERS} ${catkin_LIBRARIES})
