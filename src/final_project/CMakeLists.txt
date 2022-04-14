cmake_minimum_required(VERSION 2.8.3)
project(final_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  ugv_course_libs
  dynamic_reconfigure
  sensor_msgs
  std_msgs
  geometry_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)





catkin_package(DEPENDS OpenCV)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#################
## Drive Scripts ##
#################
add_executable(DriveFP src/DriveFP.cpp)

target_link_libraries(DriveFP ${catkin_LIBRARIES})

add_executable(GpsCodeFP src/GpsCodeFP.cpp)

target_link_libraries(GpsCodeFP ${catkin_LIBRARIES})



 