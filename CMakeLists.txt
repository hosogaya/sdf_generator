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

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${EIGEN3_INCLUDE_DIR} 
    ${PCL_INCLUDE_DIR} 
    ${OpenCV_INCLUDE_DIRS}
)

# target_link_libraries(${PROJECT_NAME}
#     ${EIGEN3_LIBRARIES}
#     ${PCL_LIBRARIES}
#     ${OpenCV_LIBS}
# )

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