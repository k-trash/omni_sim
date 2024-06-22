cmake_minimum_required(VERSION 3.8)
project(omni_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

generate_parameter_library(omni_controllers_param
  config/omni_controllers_param.yaml
)

add_library(omni_controllers
  src/omni_controllers.cpp
  src/odom_solver.cpp
)

target_include_directories(omni_controllers PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/omni_controllers_param>
)
target_link_libraries(omni_controllers PUBLIC omni_controllers_param)
ament_target_dependencies(omni_controllers PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(omni_controllers PRIVATE "OMNI_CONTROLLERS_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface omni_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/omni_sim
)

install(TARGETS omni_controllers omni_controllers_param
  EXPORT export_omni_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY xacro
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

ament_export_include_directories(include)
ament_auto_package()
