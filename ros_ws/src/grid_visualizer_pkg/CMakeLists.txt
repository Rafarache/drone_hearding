cmake_minimum_required(VERSION 3.8)
project(grid_visualizer_pkg)

# The global search wavefront and local field solver are CPU intensive. Respect
# an explicit Debug/Release choice, but optimize ordinary colcon builds too.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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
  "ament_index_cpp"
  "geometry_msgs"
  "std_msgs"
)

# Link non-ROS dependencies (like OpenCV) using standard target link libraries
target_link_libraries(grid_visualizer_node
  ${OpenCV_LIBRARIES}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_goal_quiet_zone test/test_goal_quiet_zone.cpp)
  target_compile_features(test_goal_quiet_zone PUBLIC cxx_std_17)
  target_include_directories(test_goal_quiet_zone PRIVATE include)
  add_test(NAME test_goal_quiet_zone COMMAND test_goal_quiet_zone)
  add_executable(test_search_coverage test/test_search_coverage.cpp)
  target_compile_features(test_search_coverage PUBLIC cxx_std_17)
  target_include_directories(test_search_coverage PRIVATE include)
  add_test(NAME test_search_coverage COMMAND test_search_coverage)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # These standalone regression tests use assert(), including Release builds.
    target_compile_options(test_goal_quiet_zone PRIVATE -UNDEBUG)
    target_compile_options(test_search_coverage PRIVATE -UNDEBUG)
  endif()
endif()

# Install executable
install(TARGETS
  grid_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
