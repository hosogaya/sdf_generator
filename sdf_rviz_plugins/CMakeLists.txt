cmake_minimum_required(VERSION 3.8)
project(sdf_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# # find dependencies
find_package(ament_cmake REQUIRED)
find_package(sdf_msgs REQUIRED)
find_package(sdf_generator REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(pluginlib REQUIRED)

include_directories(include ${OGRE_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}/Paging)

set(CMAKE_AUTOMOC ON)

set(sdf_rviz_plugins_headers_to_moc
  include/sdf_rviz_plugins/material_loader.h
  include/sdf_rviz_plugins/mesh_visual.h
  include/sdf_rviz_plugins/mesh_display.h
)

foreach(header "${sdf_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(sdf_rviz_plugins_moc_files "${header}")
endforeach()


set (sources 
  src/material_loader.cpp
  src/mesh_visual.cpp
  src/mesh_display.cpp
)

set (dependencies
  rclcpp
  sdf_msgs
  sdf_generator
  rviz_common
  rviz_rendering
)

add_library(${PROJECT_NAME} SHARED
  ${sdf_rviz_plugins_moc_files}
  ${sources}
)

ament_target_dependencies(${PROJECT_NAME} 
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY content
  DESTINATION share/${PROJECT_NAME}
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
