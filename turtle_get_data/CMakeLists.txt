cmake_minimum_required(VERSION 3.8)
project(turtle_get_data)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(dependencies 
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  std_msgs
  geometry_msgs
  builtin_interfaces
  nav2_msgs
)

# find dependencies
foreach(dependency ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

set(action_files
  "action/Mission.action"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES ${dependencies}
)

add_executable(get_motor_effort src/get_motor_effort.cpp)
ament_target_dependencies(get_motor_effort ${dependencies})

add_executable(get_motor_distance src/get_motor_distance.cpp)
ament_target_dependencies(get_motor_distance ${dependencies})

ament_export_dependencies(rosidl_default_runtime)

add_library(action_server SHARED
  src/get_turtle_data.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_server ${dependencies})
rclcpp_components_register_node(action_server PLUGIN "action_get_data::GetTurtleDataServer" EXECUTABLE turtle_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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
