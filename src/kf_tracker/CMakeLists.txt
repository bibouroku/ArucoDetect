cmake_minimum_required(VERSION 3.8)
project(kf_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(kf_tracker_lib SHARED
  include/kf_tracker/kf_tracker_lib.hpp
  src/kf_tracker_lib.cpp
)


target_include_directories(kf_tracker_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(kf_tracker_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(kf_tracker_lib
  PUBLIC
  "rclcpp"
  "geometry_msgs"
  "px4_msgs"
  "tf2"
  "tf2_ros"
)

ament_export_targets(kf_tracker_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rclcpp
)
install(
  DIRECTORY include/
  DESTINATION include
)

include_directories(${Eigen3_INCLUDE_DIRS})

install(TARGETS kf_tracker_lib
  EXPORT kf_tracker_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
