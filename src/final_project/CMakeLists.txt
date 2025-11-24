cmake_minimum_required(VERSION 3.8)
project(final_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
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

include_directories(include)

set(dependencies
  rclcpp
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
)

# ArUco Detector Node
add_executable(aruco_detector_node src/aruco_detector_node.cpp)
ament_target_dependencies(aruco_detector_node ${dependencies})

# Shape Detector Node
add_executable(shape_detector_node src/shape_detector_node.cpp)
ament_target_dependencies(shape_detector_node ${dependencies})

# Color Detector Node
add_executable(color_detector_node src/color_detector_node.cpp)
ament_target_dependencies(color_detector_node ${dependencies})

# Exploration Manager Node
add_executable(exploration_manager_node src/exploration_manager_node.cpp)
ament_target_dependencies(exploration_manager_node ${dependencies})

# Navigation Controller Node
add_executable(navigation_controller_node src/navigation_controller_node.cpp)
ament_target_dependencies(navigation_controller_node ${dependencies})

# Install targets
install(TARGETS
  aruco_detector_node
  shape_detector_node
  color_detector_node
  exploration_manager_node
  navigation_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch/
  config
  worlds
  urdf
  materials
  DESTINATION share/${PROJECT_NAME}
)

# Install maps directory (will be empty initially, populated during runtime)
install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

ament_package()