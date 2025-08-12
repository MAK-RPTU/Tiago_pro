cmake_minimum_required(VERSION 3.8)
project(cartesian_mover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Boost REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)  
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

find_package(gazebo_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(attach_gazebo_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"action/ExecuteTask.action"
DEPENDENCIES builtin_interfaces action_msgs
)

# Add executable
# add_executable(cartesian_path_planner src/cartesian_path_planner.cpp)
# add_executable(cartesian_planner src/cartesian_planner.cpp)
# add_executable(joint_space_simple src/joint_space_simple.cpp)
# add_executable(joint_space_scalable src/joint_space_scalable.cpp)
# add_executable(task_executor_server src/task_executor_server.cpp)
add_executable(pick_cube_node src/pick_cube_node.cpp)

# target_link_libraries(joint_space_scalable yaml-cpp)
# target_link_libraries(task_executor_server yaml-cpp)


# Link libraries
ament_target_dependencies(pick_cube_node
  rclcpp
  geometry_msgs
  gazebo_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
)

# Make sure this line is present!
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
# target_link_libraries(task_executor_server ${cpp_typesupport_target})

# Install

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  )


install(TARGETS
  pick_cube_node
  DESTINATION lib/${PROJECT_NAME})

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


ament_export_dependencies(rosidl_default_runtime)
ament_package()
