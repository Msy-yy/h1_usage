cmake_minimum_required(VERSION 3.10)
project(h1_interface)
add_compile_options(-std=c++17)

set(AMENT_DEPENDENCIES std_srvs 
                       std_msgs
                       )

set(srv_files 
                srv/H1Modes.srv
)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
                        "srv/H1Modes.srv"
)

ament_package()