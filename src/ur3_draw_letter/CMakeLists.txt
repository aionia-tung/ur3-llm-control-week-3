cmake_minimum_required(VERSION 3.8)

project(ur3_draw_letter)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(draw_letter_node
  src/draw_letter_node.cpp
)

ament_target_dependencies(draw_letter_node
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
)

install(
  TARGETS draw_letter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf rviz srdf
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
