cmake_minimum_required(VERSION 3.8)
project(grid_visualizer_pkg)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Add executable
add_executable(grid_visualizer_node src/grid_visualizer_node.cpp)
target_compile_features(grid_visualizer_node PUBLIC cxx_std_17)

# Include directories
target_include_directories(grid_visualizer_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# Dependencies linking (ROS 2 style helper)
ament_target_dependencies(grid_visualizer_node
  "rclcpp"
  "geometry_msgs"
)

# Link non-ROS dependencies (like OpenCV) using standard target link libraries
target_link_libraries(grid_visualizer_node
  ${OpenCV_LIBRARIES}
)

# Install executable
install(TARGETS
  grid_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
