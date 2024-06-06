cmake_minimum_required(VERSION 3.5)
project(rf_sensor)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Tag.msg"
  "msg/TagArray.msg"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(include)

# Rename the library target to avoid conflicts with the project name
add_library(rf_sensor_lib SHARED
  src/RfReceiver.cpp
  src/RfTag.cpp
)

ament_target_dependencies(rf_sensor_lib
  rclcpp
  std_msgs
  gz-sim8
  gz-transport13
)
ament_export_dependencies(rosidl_default_runtime)

# Link against generated message types
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(rf_sensor_lib ${cpp_typesupport_target})

# Install
install(TARGETS rf_sensor_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
