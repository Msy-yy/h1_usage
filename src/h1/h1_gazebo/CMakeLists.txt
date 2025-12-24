cmake_minimum_required(VERSION 3.10)
project(h1_gazebo)

find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  h1_gazebo/example.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS config/kill_gz.sh
  DESTINATION lib/${PROJECT_NAME} 
)

# ------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})
ament_package()