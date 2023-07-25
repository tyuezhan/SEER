cmake_minimum_required(VERSION 2.8.3)
project(kino_searching)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  plan_env
  cv_bridge
  traj_utils
  active_perception
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES kino_searching
 CATKIN_DEPENDS plan_env traj_utils active_perception
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library(kino_searching 
    src/kino_acc_astar.cpp
    src/kino_yaw.cpp 
    )
target_link_libraries(kino_searching
    ${catkin_LIBRARIES} 
    )  
