cmake_minimum_required(VERSION 3.8)

project(custom_flight)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(include)

add_executable(
    flight_mode_executor
    src/flight_mode_executor.cpp
)

ament_target_dependencies(
    flight_mode_executor
    rclcpp
    px4_msgs
)

install(
    TARGETS
        flight_mode_executor
    DESTINATION
        lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include/
)

install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
