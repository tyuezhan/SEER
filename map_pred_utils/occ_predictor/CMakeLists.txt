cmake_minimum_required(VERSION 3.10)
project(occ_predictor)

ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
link_directories(/usr/local/lib/)
## equivalent to -L /usr/local/lib of gcc/g++
## Then libcnpy.so will be used as cnpy

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
  occ_predictor_msgs
)


# add_service_files(
#   FILES
#   ResetMap.srv
#   SaveScene.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES map_generator
 CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  geometry_msgs
  message_runtime
  occ_predictor_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)
