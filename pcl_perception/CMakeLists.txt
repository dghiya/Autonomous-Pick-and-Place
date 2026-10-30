cmake_minimum_required(VERSION 3.8)
project(pcl_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom interfaces first so the targets exist
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(pcl_perception_node
  src/main.cpp
  src/pcl_perception_node.cpp
)

target_include_directories(
  pcl_perception_node PRIVATE 
  include 
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(
  pcl_perception_node
  ${PCL_LIBRARIES}
)

# Link the generated message library target to your node executable
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(pcl_perception_node "${cpp_typesupport_target}")

ament_target_dependencies(pcl_perception_node
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  message_filters
  tf2_sensor_msgs
)

# --- Install ---
install(TARGETS
  pcl_perception_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  config
  worlds
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()