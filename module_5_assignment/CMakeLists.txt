cmake_minimum_required(VERSION 3.8)
project(module_5_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)
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

add_library(lqr_lib STATIC src/lqr_lib.cpp)
target_include_directories(lqr_lib PRIVATE ${EIGEN_INCLUDE_DIR} include)
target_link_libraries(lqr_lib Eigen3::Eigen)

add_executable(piController  src/line_following.cpp)
ament_target_dependencies(piController rclcpp sensor_msgs geometry_msgs cv_bridge)
target_link_libraries(piController ${OpenCV_LIBS})

# Añade el ejecutable y vincula la biblioteca
add_executable(lqr_node src/lqr_node.cpp)
target_link_libraries(lqr_node Eigen3::Eigen lqr_lib)
ament_target_dependencies(lqr_node rclcpp tf2_ros geometry_msgs nav_msgs angles)
target_include_directories(lqr_node PRIVATE ${EIGEN_INCLUDE_DIR} include)


add_executable(marker_pub  src/marker.cpp)
ament_target_dependencies(marker_pub rclcpp nav_msgs visualization_msgs)

install(TARGETS
  lqr_node
  piController
  marker_pub
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  urdf
  config
  worlds
  bag_files
  rviz
DESTINATION share/${PROJECT_NAME})

ament_package()