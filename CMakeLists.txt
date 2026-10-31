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
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "srv/SaveMap.srv"
  DEPENDENCIES builtin_interfaces)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(TBB REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Pure logic (no rclcpp), unit-tested.
add_library(${PROJECT_NAME}_core src/map_pose_core.cpp src/map_image.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(passthrough_odom_publisher src/passthrough_odom_publisher.cpp)
add_executable(localization_backend src/localization_backend.cpp)
ament_target_dependencies(localization_backend
  rclcpp rclcpp_components nav2_amcl nav2_map_server nav2_lifecycle_manager slam_toolbox
  sensor_msgs std_srvs tf2 tf2_ros)
# slam_toolbox exports its backend libraries separately from its rosidl targets.
target_link_libraries(localization_backend ${slam_toolbox_LIBRARIES} Boost::serialization TBB::tbb)
ament_target_dependencies(passthrough_odom_publisher nav_msgs rclcpp)
add_executable(ekf_input_publisher src/ekf_input_publisher.cpp)
target_include_directories(ekf_input_publisher PRIVATE include)
ament_target_dependencies(ekf_input_publisher nav_msgs rclcpp)

add_executable(map_pose_publisher src/map_pose_publisher.cpp)
target_link_libraries(map_pose_publisher ${PROJECT_NAME}_core)
ament_target_dependencies(map_pose_publisher geometry_msgs nav_msgs rclcpp tf2 tf2_ros)

add_executable(map_autosaver src/map_autosaver.cpp)
target_link_libraries(map_autosaver ${PROJECT_NAME}_core)
ament_target_dependencies(map_autosaver nav_msgs rclcpp slam_toolbox)
rosidl_get_typesupport_target(save_map_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(map_autosaver "${save_map_typesupport}")
ament_export_dependencies(rosidl_default_runtime)

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS localization_backend passthrough_odom_publisher ekf_input_publisher map_pose_publisher map_autosaver
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/seeded_ekf DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config map DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_save_now test/test_save_now.py TIMEOUT 90
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV "PYTEST_ADDOPTS=-p no:anyio")
  ament_add_pytest_test(test_seeded_ekf test/test_seeded_ekf.py TIMEOUT 30
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV "PYTEST_ADDOPTS=-p no:anyio")
  ament_add_pytest_test(test_field_map test/test_field_map.py TIMEOUT 120
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV "PYTEST_ADDOPTS=-p no:anyio")
  set(SENTRY_FIELD_MAP_WORLD "" CACHE FILEPATH "Optional corrected-field SDF for map reproduction tests")
  if(SENTRY_FIELD_MAP_WORLD)
    if(NOT EXISTS "${SENTRY_FIELD_MAP_WORLD}")
      message(FATAL_ERROR "SENTRY_FIELD_MAP_WORLD must name an existing SDF fixture")
    endif()
    ament_add_pytest_test(test_field_map_fixture test/test_field_map_fixture.py TIMEOUT 120
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      ENV "PYTEST_ADDOPTS=-p no:anyio" "FIELD_MAP_WORLD=${SENTRY_FIELD_MAP_WORLD}")
  endif()
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
