cmake_minimum_required(VERSION 3.8)
project(omni_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(omni_control src/omni_robot_controller.cpp)
rclcpp_components_register_nodes(omni_control "OmniRobotController::OmniControl")

ament_auto_add_library(odom_tf_converter src/odom_tf_converter.cpp)
rclcpp_components_register_nodes(odom_tf_converter "OmniRobotController::OdomToTf")

install(DIRECTORY 
  launch
  config
  xacro
  world
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

ament_auto_package()
