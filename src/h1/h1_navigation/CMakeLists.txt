cmake_minimum_required(VERSION 3.10)
project(h1_navigation)
find_package(ament_cmake REQUIRED)
install(
  DIRECTORY launch maps param behavior_trees
  DESTINATION share/${PROJECT_NAME}
)
ament_package()