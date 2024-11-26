cmake_minimum_required(VERSION 3.8)
project(module_4_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(NOT Eigen3_FOUND)
  message(FATAL_ERROR "Eigen3 no encontrado. Asegúrate de que esté instalado correctamente.")
endif()


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

add_executable(wallFollowing  src/wall_follower.cpp)
ament_target_dependencies(wallFollowing rclcpp sensor_msgs geometry_msgs)
target_include_directories(wallFollowing PRIVATE ${EIGEN3_INCLUDE_DIR})

add_executable(calculateVelAcc  src/imu.cpp)
ament_target_dependencies(calculateVelAcc rclcpp sensor_msgs Eigen3)
target_include_directories(calculateVelAcc PRIVATE ${EIGEN3_INCLUDE_DIR})


install(TARGETS
  wallFollowing
  calculateVelAcc
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  worlds
  models
DESTINATION share/${PROJECT_NAME})


ament_package()
