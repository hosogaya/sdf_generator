cmake_minimum_required(VERSION 3.8)
project(sdf_generator_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sdf_generator REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sdf_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
# find_package(PCL REQUIRED COMPONENTS common io)

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

set (dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  sdf_msgs
  tf2_ros
  pcl_conversions
  sdf_generator
)

set(node_plugins "")

# tsdf server
add_library(tsdf_server SHARED src/tsdf_server.cpp)
ament_target_dependencies(tsdf_server ${dependencies})
target_include_directories(tsdf_server
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
rclcpp_components_register_nodes(tsdf_server "sdf_generator::TsdfServer")
set(node_plugins "${node_plugins}sdf_generator::TsdfServer;$<TARGET_FILE:tsdf_server>\n")
ament_export_targets(tsdf_server HAS_LIBRARY_TARGET)

install(
  TARGETS tsdf_server
  EXPORT tsdf_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(tsdf_server_node 
  src/tsdf_server.cpp
  src/tsdf_server_node.cpp
)

ament_target_dependencies(tsdf_server_node ${dependencies})
target_include_directories(tsdf_server_node
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

install(
  TARGETS tsdf_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_package()
