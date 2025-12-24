cmake_minimum_required(VERSION 3.10)
project(h1_platform)

# ------------------------------------------------------------------
# Build settings
# ------------------------------------------------------------------
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fPIC")

# ------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ROS 2 / msgs
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Unitree SDK deps (as in tree)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)

# Optional extras used by other sample nodes in this pkg
find_package(h1_interface REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# GStreamer (for video publisher sample)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0 gstreamer-app-1.0)

# ------------------------------------------------------------------
# Include / Link (paths from prior setup)
# ------------------------------------------------------------------
include_directories(
  include
  /usr/include/eigen3
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  include/unitree
)

link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

# Define dependency list for hardware interface
set(DEPENDENCY_LIST
  hardware_interface
  pluginlib
  rclcpp
  unitree_hg
  unitree_api
)

# ------------------------------------------------------------------
# Executables
# ------------------------------------------------------------------
# H1-2 Dual Arm Trajectory Following
add_executable(h1_2_dual_arm_controller src/h1_2_arm_server.cpp)
ament_target_dependencies(h1_2_dual_arm_controller rclcpp rclcpp_action control_msgs sensor_msgs std_srvs trajectory_msgs)
target_link_libraries(h1_2_dual_arm_controller unitree_sdk2)

# H1-2 Dual Arm Recorder
add_executable(h1_2_dual_arm_recorder src/h1_2_arm_record.cpp)
ament_target_dependencies(h1_2_dual_arm_recorder rclcpp std_srvs h1_interface unitree_hg unitree_api)
target_link_libraries(h1_2_dual_arm_recorder unitree_sdk2)

# H1 Highroscontrol
add_executable(h1_highroscontrol src/h1_highroscontrol.cpp)
ament_target_dependencies(h1_highroscontrol
  unitree_hg unitree_api rclcpp std_msgs std_srvs h1_interface tf2 tf2_ros
  sensor_msgs trajectory_msgs rosbag2_cpp hardware_interface pluginlib
)

# H1 Video Publisher
add_executable(h1_video_publisher src/h1_video_stream.cpp)
ament_target_dependencies(h1_video_publisher rclcpp sensor_msgs)
target_include_directories(h1_video_publisher PRIVATE ${GST_INCLUDE_DIRS})
target_link_directories(h1_video_publisher PRIVATE ${GST_LIBRARY_DIRS})
target_compile_options(h1_video_publisher PRIVATE ${GST_CFLAGS_OTHER})
target_link_libraries(h1_video_publisher ${GST_LIBRARIES} ${OpenCV_LIBS})

# ------------------------------------------------------------------ ROS2 HW
## ROS2 HW COMPILE
add_library(
  h1_platform
  SHARED
  src/arm_hardware/h1_2_arm_hw.cpp
)

target_include_directories(
  h1_platform
  PRIVATE
  include
)

# Add ROS 2 dependencies to the hardware interface library
ament_target_dependencies(h1_platform ${DEPENDENCY_LIST})

## ROS2 HW INSTALL
install(
  TARGETS h1_platform
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

## ROS2 HW EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  h1_platform
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  unitree_hg
  unitree_api
)

pluginlib_export_plugin_description_file(hardware_interface h1_platform.xml)

# ------------------------------------------------------------------
# Install
# ------------------------------------------------------------------
install(TARGETS
  h1_2_dual_arm_controller
  h1_2_dual_arm_recorder
  h1_highroscontrol
  h1_video_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Helper scripts/binaries you already had
install(PROGRAMS h1_platform/h1_2_statepublisher h1_platform/h1_statepublisher
  DESTINATION lib/${PROJECT_NAME}
)

# Configs & launch files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
# ------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})
ament_package()
