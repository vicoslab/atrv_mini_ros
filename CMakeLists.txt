cmake_minimum_required(VERSION 2.8.3)
project(atrv_mini_ros)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs nav_msgs tf geometry_msgs sensor_msgs angles)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# atrv_mini
add_executable(atrv_mini_node src/ATRVmini_node.cc)
add_library(atrv_mini_driver src/ATRVmini_driver.cc)
add_library(rflex_driver src/rflex_driver.cc)
target_link_libraries(atrv_mini_node ${catkin_LIBRARIES} atrv_mini_driver rflex_driver)

install(DIRECTORY scripts/
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



