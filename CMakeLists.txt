cmake_minimum_required(VERSION 3.8)
project(sentry_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_localization REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Pure logic (no rclcpp), unit-tested.
add_library(${PROJECT_NAME}_core src/map_pose_core.cpp src/map_image.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(passthrough_odom_publisher src/passthrough_odom_publisher.cpp)
ament_target_dependencies(passthrough_odom_publisher nav_msgs rclcpp)
add_executable(ekf_input_publisher src/ekf_input_publisher.cpp)
target_include_directories(ekf_input_publisher PRIVATE include)
ament_target_dependencies(ekf_input_publisher nav_msgs rclcpp)
add_executable(lockstep_ekf src/lockstep_ekf.cpp)
target_link_libraries(lockstep_ekf Eigen3::Eigen ${robot_localization_LIBRARIES})
ament_target_dependencies(lockstep_ekf diagnostic_msgs diagnostic_updater nav_msgs rclcpp
  robot_localization sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros)

add_executable(map_pose_publisher src/map_pose_publisher.cpp)
target_link_libraries(map_pose_publisher ${PROJECT_NAME}_core)
ament_target_dependencies(map_pose_publisher geometry_msgs nav_msgs rclcpp tf2 tf2_ros)

add_executable(map_autosaver src/map_autosaver.cpp)
target_link_libraries(map_autosaver ${PROJECT_NAME}_core)
ament_target_dependencies(map_autosaver nav_msgs rclcpp slam_toolbox)

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS passthrough_odom_publisher ekf_input_publisher lockstep_ekf map_pose_publisher map_autosaver
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config map DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  ament_copyright(launch src include test)
  ament_flake8(launch)
  ament_pep257(launch)
  ament_add_gtest(test_map_pose_core test/test_map_pose_core.cpp)
  target_link_libraries(test_map_pose_core ${PROJECT_NAME}_core)
  ament_add_gtest(test_map_image test/test_map_image.cpp)
  target_link_libraries(test_map_image ${PROJECT_NAME}_core)
  ament_add_gtest(test_ekf_input test/test_ekf_input.cpp)
  target_include_directories(test_ekf_input PRIVATE include)
  ament_target_dependencies(test_ekf_input nav_msgs)
endif()

ament_package()
