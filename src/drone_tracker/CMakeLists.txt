cmake_minimum_required(VERSION 3.5)
project(drone_tracker)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(/usr/local/lib)

add_executable(drone_tracker drone_tracker.cpp)



ament_target_dependencies(drone_tracker Eigen3 rclcpp px4_msgs geometry_msgs tf2 tf2_ros)

target_compile_features(drone_tracker PUBLIC c_std_99 cxx_std_17)

install(TARGETS drone_tracker DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
