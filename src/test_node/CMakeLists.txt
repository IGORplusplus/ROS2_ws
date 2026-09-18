cmake_minimum_required(VERSION 3.8)
project(test_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

# Create executable
add_executable(px4_ros2_communication
  src/px4_ros2_communication.cpp
)

# Tell the compiler where to find the header
target_include_directories(px4_ros2_communication PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link ROS 2/PX4 dependencies
ament_target_dependencies(px4_ros2_communication
  rclcpp
  px4_msgs
)

# Install executable
install(
  TARGETS px4_ros2_communication
  DESTINATION lib/${PROJECT_NAME}
)

# Install header
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
