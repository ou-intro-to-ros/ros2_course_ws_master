cmake_minimum_required(VERSION 3.8)
project(final_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)  # Added for action client support
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  OpenCV
  ament_index_cpp
)

# Maze Explorer Node (new)
add_executable(maze_explorer_node src/maze_explorer.cpp)
ament_target_dependencies(maze_explorer_node ${dependencies})

# Maze Explorer Node (new)
add_executable(map_saver_node src/map_saver_node.cpp)
ament_target_dependencies(map_saver_node ${dependencies} ament_index_cpp)

add_executable(aruco_goal_node src/aruco_goal_node.cpp)
ament_target_dependencies(aruco_goal_node ${dependencies})
target_link_libraries(aruco_goal_node yaml-cpp)

# Install targets
install(TARGETS
  maze_explorer_node
  map_saver_node
  aruco_goal_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch/
  config
  worlds
  urdf
  materials
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install maps directory (will be empty initially, populated during runtime)
install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

ament_package()