cmake_minimum_required(VERSION 3.8)
project(module_6_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

include_directories(include)

add_library(ekf_lib STATIC src/ekf_lib.cpp)
target_include_directories(ekf_lib PRIVATE ${EIGEN_INCLUDE_DIR} include)
target_link_libraries(ekf_lib Eigen3::Eigen)

# Define los archivos fuente
set(SOURCES
  src/gps_utils.cpp
)

add_executable(ekf_node src/ekf_node.cpp ${SOURCES})
ament_target_dependencies(ekf_node rclcpp nav_msgs visualization_msgs sensor_msgs geometry_msgs)
target_include_directories(ekf_node PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(ekf_node Eigen3::Eigen ekf_lib)

add_executable(gps_sub src/gps_sub.cpp ${SOURCES})
ament_target_dependencies(gps_sub rclcpp nav_msgs sensor_msgs geometry_msgs)

add_executable(marker_pub src/marker.cpp ${SOURCES})
ament_target_dependencies(marker_pub rclcpp nav_msgs sensor_msgs visualization_msgs geometry_msgs tf2_ros tf2)

install(TARGETS
  ekf_node
  gps_sub
  marker_pub
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  worlds
DESTINATION share/${PROJECT_NAME})

ament_package()