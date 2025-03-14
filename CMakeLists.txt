cmake_minimum_required(VERSION 3.10)
project(atrv_mini_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(angles REQUIRED)

# Include the include/ directory for headers
include_directories(include)

# Add libraries
add_library(atrv_mini_driver src/ATRVmini_driver.cc)
add_library(rflex_driver src/rflex_driver.cc)

# Add the main executable
add_executable(atrv_mini_node src/ATRVmini_node.cc)

# Link libraries
target_link_libraries(atrv_mini_node atrv_mini_driver rflex_driver)

ament_target_dependencies(atrv_mini_driver sensor_msgs)
ament_target_dependencies(atrv_mini_node rclcpp std_msgs tf2_msgs nav_msgs sensor_msgs geometry_msgs angles tf2_ros)

# Install targets
install(TARGETS
  atrv_mini_node
  atrv_mini_driver
  rflex_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/teleop_atrv_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
