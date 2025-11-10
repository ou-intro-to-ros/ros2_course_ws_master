cmake_minimum_required(VERSION 3.8)
project(opencv_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Include directories
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# Executable 1: Read and Display
add_executable(read_display src/read_display.cpp)
target_link_libraries(read_display ${OpenCV_LIBS})
ament_target_dependencies(read_display ament_index_cpp)

# Executable 2: Color Conversion
add_executable(color_convert src/color_convert.cpp)
target_link_libraries(color_convert ${OpenCV_LIBS})
ament_target_dependencies(color_convert ament_index_cpp)

# Executable 3: Resize, Crop, Rotate
add_executable(resize_crop_rotate src/resize_crop_rotate.cpp)
target_link_libraries(resize_crop_rotate ${OpenCV_LIBS})
ament_target_dependencies(resize_crop_rotate ament_index_cpp)

# Executable 4: Draw Shapes
add_executable(draw_shapes src/draw_shapes.cpp)
target_link_libraries(draw_shapes ${OpenCV_LIBS})
ament_target_dependencies(draw_shapes ament_index_cpp)

# Executable 5: Blur and Smoothing
add_executable(blur_smooth src/blur_smooth.cpp)
target_link_libraries(blur_smooth ${OpenCV_LIBS})
ament_target_dependencies(blur_smooth ament_index_cpp)

# Install executables
install(TARGETS
  read_display
  color_convert
  resize_crop_rotate
  draw_shapes
  blur_smooth
  DESTINATION lib/${PROJECT_NAME}
)

# Install images directory
install(DIRECTORY
  images/
  DESTINATION share/${PROJECT_NAME}/images
)

ament_package()