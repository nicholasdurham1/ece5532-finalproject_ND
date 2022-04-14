cmake_minimum_required(VERSION 2.8.3)
project(gps_sim_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  ugv_course_libs
  dynamic_reconfigure
  sensor_msgs
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#################
## Test Script ##
#################
add_executable(test_gps_sim_project test/test_gps_project.cpp)
target_link_libraries(test_gps_sim_project ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
add_executable(Drive src/Drive.cpp)
target_link_libraries(Drive ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
add_executable(Markers src/Markers.cpp)
target_link_libraries(Markers ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
add_executable(GpsCode src/GpsCode.cpp)
target_link_libraries(GpsCode ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
