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
  tf2_ros
  pcl_conversions
  sdf_generator
)

set(node_plugins "")

function(add_components base_name class_name)
	add_library(${base_name} SHARED src/${base_name}.cpp)
  ament_target_dependencies(${base_name} ${dependencies})
  # target_link_libraries(${base_name} ${PCL_LIBRARIES})
  target_include_directories(${base_name}
    PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>"
      # ${EIGEN3_INCLUDE_DIR} 
      # ${PCL_INCLUDE_DIR} 
      # ${OpenCV_INCLUDE_DIRS}
  )
  rclcpp_components_register_nodes(${base_name} "sdf_generator::${class_name}")
  set(node_plugins "${node_plugins}sdf_generator::${class_name};$<TARGET_FILE:${base_name}>\n")
  ament_export_targets(${base_name} HAS_LIBRARY_TARGET)

  install(
    TARGETS ${base_name}
    EXPORT ${base_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endfunction()

add_components(tsdf_server TsdfServer)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
