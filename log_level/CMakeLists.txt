cmake_minimum_required(VERSION 3.8)
project(log_level)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)

# Make the msg files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Float64Stamped.msg"
  DEPENDENCIES std_msgs
 )
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

 # Make the signal_generator component
add_library(signal_generator_component SHARED
  src/signal_generator.cpp
)
add_dependencies(signal_generator_component ${PROJECT_NAME})
target_link_libraries(signal_generator_component "${cpp_typesupport_target}")
target_compile_definitions(signal_generator_component
  PRIVATE "LOG_LEVEL_BUILDING_DLL"
)
ament_target_dependencies(signal_generator_component
  rclcpp
  std_msgs
  rclcpp_components
)
rclcpp_components_register_nodes(signal_generator_component "log_level::SignalGenerator")

install(TARGETS
  signal_generator_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Make the pid_controller_good component
add_library(pid_controller_good_component SHARED
  src/pid_controller_good.cpp
)
add_dependencies(pid_controller_good_component ${PROJECT_NAME})
target_link_libraries(pid_controller_good_component "${cpp_typesupport_target}")
target_compile_definitions(pid_controller_good_component
  PRIVATE "LOG_LEVEL_BUILDING_DLL"
)
ament_target_dependencies(pid_controller_good_component
  rclcpp
  std_msgs
  rclcpp_components
)
rclcpp_components_register_nodes(pid_controller_good_component "log_level::good::PIDController")
install(TARGETS
  pid_controller_good_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Make the pid_controller_bad component
add_library(pid_controller_bad_component SHARED
  src/pid_controller_bad.cpp
)
add_dependencies(pid_controller_bad_component ${PROJECT_NAME})
target_link_libraries(pid_controller_bad_component "${cpp_typesupport_target}")
target_compile_definitions(pid_controller_bad_component
  PRIVATE "LOG_LEVEL_BUILDING_DLL"
)
ament_target_dependencies(pid_controller_bad_component
  rclcpp
  std_msgs
  rclcpp_components
)
rclcpp_components_register_nodes(pid_controller_bad_component "log_level::bad::PIDController")
install(TARGETS
  pid_controller_bad_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
