cmake_minimum_required(VERSION 3.8)
project(sdf_generator)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

set (dependencies
    rclcpp
    rclcpp_components
    sensor_msgs
    geometry_msgs
    tf2_ros
    pcl_conversions
)

set (sources_wo_ros
    src/core/tsdf_map.cpp
    src/integrator/ray_caster.cpp
    src/integrator/simple_tsdf_integrator.cpp
    src/integrator/thread_safe_index.cpp
    src/integrator/tsdf_integrator.cpp
    src/interpolator/interpolator.cpp
    src/point_cloud/point_cloud_processor.cpp
    src/point_cloud/lidar_processor.cpp
)

add_library(${PROJECT_NAME} SHARED ${sources_wo_ros})
# ament_target_dependencies(${PROJECT_NAME} ${dependencies})

target_include_directories(${PROJECT_NAME}
    PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${EIGEN3_INCLUDE_DIR} 
    ${PCL_INCLUDE_DIR} 
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    ${EIGEN3_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

# set(node_plugins "")
# rclcpp_components_register_nodes(${PROJECT_NAME} "sdf_generator::TsdfServer")
# set(node_plugins "${node_plugins}sdf_generator::TsdfServer;$<TARGET_FILE:${PROJECT_NAME}>\n")
# ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib
    INCLUDES DESTINATION include
)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}/
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(Eigen3 PCL OpenCV)
ament_export_targets(${PROJECT_NAME}_export)

ament_package()

# function(add_sdf_test base_name)
# 	add_executable(${base_name} test/${base_name}.cpp)
#     target_link_libraries(${base_name} PUBLIC
#         ${PROJECT_NAME}
#         ${EIGEN3_LIBRARIES}
#         ${PCL_LIBRARIES}
#         ${OpenCV_LIBS}
#     )
#     target_include_directories(${base_name}
#         PUBLIC
#         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#         "$<INSTALL_INTERFACE:include>"
#         ${EIGEN3_INCLUDE_DIR} 
#         ${PCL_INCLUDE_DIR} 
#         ${OpenCV_INCLUDE_DIRS}
#     )
#     install(
#         TARGETS ${base_name}
#         DESTINATION lib/test/${PROJECT_NAME}
#     )
# endfunction()

# add_sdf_test(tsdf_map)