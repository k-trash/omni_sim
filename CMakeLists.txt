cmake_minimum_required(VERSION 3.8)
project(omni_sim)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=return-type -Werror=shadow -Werror=format)
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
  class_loader
  tf2
  tf2_msgs
  yaml-cpp
  yaml_cpp_vendor
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(parameter_traits REQUIRED)
find_package(backward_ros REQUIRED)
find_package(controller_manager REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#ament_auto_find_build_dependencies()

add_compile_definitions(RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR})
add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})

generate_parameter_library(omni_controllers_param
  config/omni_controllers_param.yaml
)

add_library(omni_controllers SHARED
  src/omni_controllers.cpp
  src/odom_solver.cpp
)

#target_compile_features(omni_controllers PUBLIC cxx_std_17)
target_include_directories(omni_controllers PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/omni_controllers>
)
target_link_libraries(omni_controllers PUBLIC omni_controllers_param)
ament_target_dependencies(omni_controllers PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(omni_controllers PRIVATE "OMNI_CONTROLLERS_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface omni_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS omni_controllers omni_controllers_param
  EXPORT export_omni_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY
  launch
  config
  xacro
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

ament_export_targets(export_omni_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()