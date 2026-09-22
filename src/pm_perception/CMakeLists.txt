cmake_minimum_required(VERSION 3.5)
project(pm_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(rolling_elevation_grid src/rolling_elevation_grid.cpp)
target_include_directories(rolling_elevation_grid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(depth_elevation_mapper_node src/depth_elevation_mapper_node.cpp)
target_include_directories(depth_elevation_mapper_node PRIVATE include)
target_link_libraries(depth_elevation_mapper_node rolling_elevation_grid)
ament_target_dependencies(depth_elevation_mapper_node
  geometry_msgs nav_msgs rclcpp sensor_msgs tf2 tf2_ros)

install(TARGETS rolling_elevation_grid depth_elevation_mapper_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rolling_elevation_grid test/test_rolling_elevation_grid.cpp)
  target_include_directories(test_rolling_elevation_grid PRIVATE include)
  target_link_libraries(test_rolling_elevation_grid rolling_elevation_grid)
endif()

ament_export_include_directories(include)
ament_export_libraries(rolling_elevation_grid)
ament_package()
