cmake_minimum_required(VERSION 3.8)
project(agent_ros_bridge_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate ROS interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Message definitions
  msg/Intent.msg
  msg/Entity.msg
  msg/Constraint.msg
  msg/ContextQuery.msg
  msg/ContextResponse.msg
  
  # Service definitions
  srv/ParseIntent.srv
  srv/ResolveContext.srv
  
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
