cmake_minimum_required(VERSION 3.8)
project(robot_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs)



add_executable(plant_node src/plant_node.cpp)
add_executable(sensor_node src/sensor_node.cpp)

ament_target_dependencies(plant_node rclcpp geometry_msgs)
ament_target_dependencies(sensor_node rclcpp)

install(TARGETS
  plant_node
  sensor_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
