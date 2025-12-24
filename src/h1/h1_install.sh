#!/usr/bin/env bash

function color_echo () {
    echo "$(tput setaf 2; tput bold)$1$(tput sgr0)"
}

function install_binary_packages () {
    color_echo "Installing build pacakges."
    sudo apt-get install libpcl-dev build-essential\
                         libunwind-dev\
                         neofetch\
                         cmake\
                         libglfw3-dev\
                         libglew-dev\
                         libeigen3-dev\
                         libjsoncpp-dev\
                         libtclap-dev\
                         libeigen3-dev\
                         libboost-all-dev\
                         gstreamer1.0-plugins-base\
                         gstreamer1.0-plugins-good\
                         gstreamer1.0-plugins-bad\
                         gstreamer1.0-plugins-ugly\
                         gstreamer1.0-libav\
                         gstreamer1.0-tools\
                         gstreamer1.0-x\
                         gstreamer1.0-alsa\
                         gstreamer1.0-gl\
                         gstreamer1.0-gtk3\
                         gstreamer1.0-qt5\
                         gstreamer1.0-pulseaudio\
                         libgstreamer-plugins-base1.0-dev\
                         gstreamer1.0-tools\
                         libavcodec-dev\
                         libavformat-dev\
                         libavutil-dev\
                         libswscale-dev\
                         libgtk-3-dev\
                         git\
                         libbullet-dev \
                         python3-colcon-common-extensions \
                         python3-flake8 \
                         python3-pip \
                         python3-pytest-cov \
                         python3-setuptools \
                         wget \
                         nano -y

    pip3 install flask

    color_echo "Installing build pacakges."
    sudo apt install ros-dev-tools\
                         ros-humble-ros2-control \
                         ros-humble-hardware-interface \
                         ros-humble-controller-interface \
                         ros-humble-pluginlib\
                         ros-humble-robot-upstart\
                         ros-humble-xacro\
                         ros-humble-slam-toolbox\
                         ros-humble-teleop-twist-keyboard\
                         ros-humble-teleop-twist-joy\
                         ros-humble-geodesy\
                         ros-humble-pcl-ros\
                         ros-humble-nmea-msgs\
                         ros-humble-robot-localization\
                         ros-humble-interactive-marker-twist-server\
                         ros-humble-pointcloud-to-laserscan\
                         ros-humble-twist-mux\
                         ros-humble-rmw-cyclonedds-cpp\
                         ros-humble-rosidl-generator-dds-idl\
                         ros-humble-navigation2\
                         ros-humble-moveit\
                         ros-humble-joint-state-publisher-gui\
                         ros-humble-ros2-control\
                         ros-humble-ros2-controllers\
                         ros-humble-gripper-controllers\
                         ros-humble-test-msgs\
                         ros-humble-moveit\
                         ros-humble-moveit-servo\
                         ros-humble-navigation2\
                         ros-humble-hardware-interface\
                         ros-humble-realsense2-*\
                         ros-humble-rqt-tf-tree\
                         ros-humble-librealsense2*\
                         ros-humble-rosbridge-server\
                         ros-humble-domain-bridge\
                         ros-humble-ros-gz\
                         ros-humble-joy-linux\
                         ros-humble-nav2-* -y
}

function install_debian () {
    color_echo "Copying over debian packages"
    sudo cp h1_bringup/debian/45-mbs.rules /etc/udev/rules.d/
    sudo service udev restart && sudo udevadm trigger
    sudo usermod -aG input $USER
}

function install_motd () {
    color_echo "Copying over motd"
    sudo chmod -x /etc/update-motd.d/*
    sudo cp h1_bringup/config/10-qre-h1-motd /etc/update-motd.d/
    sudo chmod +x /etc/update-motd.d/10-qre-h1-motd
}


function install_livox () {
    echo -e "${CYAN}Installing Livox${NC}"
    rm -rf ../third_party/sensors/8May2025_livox_sdk/build
    mkdir ../third_party/sensors/8May2025_livox_sdk/build
    cd ../third_party/sensors/8May2025_livox_sdk/build
    cmake ..
    make -j4
    sudo make install
}


RED='\033[0;31m'
DGREEN='\033[0;32m'
GREEN='\033[1;32m'
WHITE='\033[0;37m'
BLUE='\033[1;34m'
CYAN='\033[1;36m'
NC='\033[0m' 

# Binary packages installation
install_binary_packages
install_motd
install_debian
# install_livox