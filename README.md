# H1/H1-2_carry_box demo

> [!WARNING]
> Unitree are constantly updating their firmware and at times changing their naming convention which **may** cause breakage in some of the newer firmwares. In that case either submit and issue and/or create a pull request with the correct feature.

# Table of Contents

<!-- TOC -->

- [H1 + H1-2 + H1-M v2.0](#h1--h1-2--h1-m-v20)
- [Table of Contents](#table-of-contents)
- [Documentation](#documentation)
- [Current Available Features](#current-available-features)
- [H1/H1-2/H1-M Installation](#h1h1-2h1-m-installation)
  - [Robot](#robot)
  - [Host](#host)
- [Information](#information)
  - [H1 Network Table](#h1-network-table)
- [Quick Start](#quick-start)
  - [Ethernet Connection](#ethernet-connection)
  - [Static Network Connection](#static-network-connection)
  - [Teleop](#teleop)
  - [Visualiation](#visualiation)
- [ROS2 Modules](#ros2-modules)
  - [H1 Over View](#h1-over-view)
  - [Tele-Operation](#tele-operation)
  - [Core Launch Files](#core-launch-files)
    - [Arms H1 Core Launch Files](#arms-h1-core-launch-files)
    - [Arms H1-2 Core Launch Files](#arms-h1-2-core-launch-files)
    - [SDK Examples](#sdk-examples)
  - [Modes Selection](#modes-selection)
    - [Posture \& State Commands](#posture--state-commands)
    - [Setter Commands (Change Robot State)](#setter-commands-change-robot-state)
    - [Getter Commands (Query Current Robot State)](#getter-commands-query-current-robot-state)
    - [Toggle Commands](#toggle-commands)
    - [Examples](#examples)
  - [Dual Arm Record H1-2](#dual-arm-record-h1-2)
    - [Launch](#launch)
    - [Configuration](#configuration)
    - [Start recording](#start-recording)
    - [Stop recording and save](#stop-recording-and-save)
    - [Play last recorded trajectory](#play-last-recorded-trajectory)
    - [Play specific file](#play-specific-file)
  - [Dual Arm Control H1-2](#dual-arm-control-h1-2)
    - [Launch](#launch-1)
    - [Configuration](#configuration-1)
    - [Poses](#poses)
  - [Sensors](#sensors)
    - [Lidar Ouster](#lidar-ouster)
    - [Depth Camera Realsense D435i](#depth-camera-realsense-d435i)
    - [Depth Camera ZED2i](#depth-camera-zed2i)
      - [Emlid M2](#emlid-m2)
  - [Simulation](#simulation)
    - [Gazebo Fortress (Work In Progress)](#gazebo-fortress-work-in-progress)
    - [Unitree RL Gym](#unitree-rl-gym)
    - [Isaac Lab](#isaac-lab)
  - [Navigation](#navigation)
    - [Odometric Navigation](#odometric-navigation)
    - [Map Navigation](#map-navigation)
  - [SLAM (Simultaneous Localization and Mapping)](#slam-simultaneous-localization-and-mapping)
- [Debugging](#debugging)
  - [RQT](#rqt)
  - [Dynamic Reqconfigure](#dynamic-reqconfigure)
  - [TF](#tf)
  - [Video Stream](#video-stream)
- [Miscellanious](#miscellanious)
  - [Update System Kernel Cyclone](#update-system-kernel-cyclone)
  - [New Livox Mid 360 Units Setup (First Time)](#new-livox-mid-360-units-setup-first-time)
  - [Sync host computer and unitree computer](#sync-host-computer-and-unitree-computer)
  - [Xacro to URDF](#xacro-to-urdf)
  - [Enable Internet](#enable-internet)
    - [LAN](#lan)
    - [ZED2i build](#zed2i-build)

<!-- /TOC -->

# Documentation

The documentation websites usually lag behind and this README.md should have the latest changes pertaining to the MBS H1 ROS Docs. 

1. [Unitree H1 Docs](https://support.unitree.com/home/en/H1_developer)


# Current Available Features

- [X] H1 Viz
- [X] H1 Base High Level Driver
    - [X] H1 Joint State Publisher
    - [X] H1 IMU Publisher
    - [X] H1 cmd Control
    - [X] H1 Lidar Odom (Kiss) (Completely untuned)
- [X] H1 Base Low Level Driver (Provided by Unitree)
- [X] H1 Arm
  - [X] H1 Arm Control
  - [X] H1 Arm Joint Control
  - [ ] H1 Arm Moveit2 (Under consideration)
- [X] H1 Description
- [X] H1 Lidar | Ouster
- [X] H1 Depth Camera | Intel Realsense D435 (Utilizing Inbuilt no depth - Requires external plug)
- [X] H1 Bringup
- [X] H1 Multiplexer
- [ ] H1 SLAM
- [ ] H1 Odom Navigation
- [ ] H1 Map Navigation
- [X] H1 Webserver
- [X] H1 Gazebo Fortress (Needs Tuning)
  

# H1/H1-2/H1-M Installation

## Robot

- Enable Internet by connecting it to a switch with internet

```bash
sudo ip link set eth0 down && sudo ip link set eth0 up
sudo dhclient eth0
```

- Switch H1 id with ctrl+h

```bash
h1_unit_238
```

- Switch username

```bash
sudo hostnamectl set-hostname h1-unit-284-pc-4
```

- Disable redundant B2 slam

```bash
sudo systemctl stop unitree_slam.service
sudo systemctl disable unitree_slam.service
sudo rm /lib/systemd/system/unitree_slam.service
```

- Update date and time, ensure you have internet in the robot. The quickest way is using the USB WiFi stick
```bash
sudo timedatectl set-timezone Europe/Berlin
sudo date -s "$(wget --method=HEAD -qSO- --max-redirect=0 google.com 2>&1 | sed -n 's/^ *Date: *//p')"
```

- Install ROS2 Humble dev tools

```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt install ros-dev-tools
```

- Create directory for h1 Workspace

```bash
sudo mkdir /opt/mybotshop
```
```bash
sudo chown -R unitree:unitree /opt/mybotshop
```

- Clone the repository, copy over to the h1's PC and run the installer script
  - Press Y when installing Livox Mid360
  - Select gdm3 when prompted in webserver installation

```bash
cd /opt/mybotshop/src/mybotshop/ && sudo chmod +x h1_install.sh && ./h1_install.sh
```


```bash
cd /opt/mybotshop/src/mybotshop/h1_webserver && sudo chmod +x webserver_installer.sh && ./webserver_installer.sh
```

- Build unitree sdk

```bash
cp -r /opt/mybotshop/src/third_party/unitree/04Aug2025_unitree_sdk2/ /opt/mybotshop
cd /opt/mybotshop/04Aug2025_unitree_sdk2/build
cmake .. && make && sudo make install
```

- Build Kiss ICP

```bash
python3 -m pip install --user --upgrade "cmake>=3.24"
export PATH="$HOME/.local/bin:$PATH"
cd /opt/mybotshop && colcon build --symlink-install --packages-select kiss_icp --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.24
```

- Build ros2 workspace

```bash
cd /opt/mybotshop && colcon build --symlink-install && source install/setup.bash
```

- Add the following to `.bashrc`

```bash
source /opt/mybotshop/install/setup.bash
alias h1_build='cd /opt/mybotshop && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash'
```

- Increase cyclone kernel

```bash
printf "net.core.rmem_max=33554432\nnet.core.rmem_default=33554432\nnet.core.wmem_max=33554432\nnet.core.wmem_default=33554432\nnet.ipv4.udp_rmem_min=16384\nnet.ipv4.udp_wmem_min=16384\n" | sudo tee /etc/sysctl.d/90-ros2-dds.conf
```

```bash
sudo sysctl --system 
```


- Run the startup installer
  - Ensure H1/H1-2 for **description** in the `/opt/mybotshopt/src/mybotshop/h1_bringup/scripts/startup_installer.py`

```bash
ros2 run h1_bringup startup_installer.py
```

## Host

- Clone the repository into a ros2_ws and build essential items only via:

```bash
colcon build --symlink-install --packages-select h1_description h1_viz && source install/setup.bash 
```

- Source and run rviz2

```bash
export H1_NS="h1_unit_238"
ros2 launch h1_viz view_robot.launch.py
```

# Information

## H1 Network Table

| IP Address           | Device            | Username       | Password     |
| -------------------- | ----------------- | --------       | ---------    |
| 192.168.123.161      | H1 MCU            | x              | x            |
| 192.168.123.164      | H1 Auxiliary PC 2 | unitree        | Unitree0408  |
| 192.168.123.164:9000 | H1 Webserver MBS  | admin          | mybotshop    |
| 192.168.123.165      | H1 Nvidia BagPack | administrator  | mybotshop    |
| 192.168.123.120      | Mid360 Lidar      | x              | x            |
| 192.168.123.20       | Ouster Lidar      | x              | x            |


>[!NOTE]
> Do not set your ip as these H1 IPs

# Quick Start

Applicable only if qre_h1 repository has been installed. If not please follow the installation instructions.

## Ethernet Connection

- Power on the robot
- Connect `LAN` cable to the ethernet port (Top Most for the available demo unit)

## Static Network Connection
For the first time, one needs to connect through a LAN cable to configure the robot's network.

To create a static connection in your PC (not the robots), in Ubuntu go to Settings → Network then click on + and create a new connection.

1. Change the connection to `Manual` in the IPv4 settings.
2. Set the Address IP as `192.168.123.51` and the Netmask as `24`.
3. Click save and restart your network.
   
   
After a successful connection, check the host's local IP by typing in the Host PC's terminal:

```bash
ifconfig
```
Now, ping the robot:

```bash
ping 192.168.123.164
```

Access the robot via SSH:
```bash
ssh -X unitree@192.168.123.164
```

The default password is:
```bash
123
```

## Teleop 

This requires the installation of the ROS2 Modules on H1. If not done please follow installation on [H1 Auxiliary PC Installation](#h1-auxiliary-pc)

- Teleop H1

```bash
ROS_DOMAIN_ID=10 ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/h1_unit_238/cmd_vel
```

## Visualiation 

- Launch H1 Viz

```bash
ros2 launch h1_viz view_robot.launch.py
```

# ROS2 Modules

## H1 Over View

>[!NOTE]
> Please ensure that the arms are straight down when you power on the robot, if the orientation is different, it will effect the ros2 control operation.  

- Starts driver for operating H1 in high-level mode for the legs and arms:

>[!IMPORTANT]
> The arms will initialize and move such that the forearms will be facing forward.

- Launch the h1 ros2 driver which includes
  - Domain Bridge
  - Joint States
  - Robot Description
  - Arm Control
  - Leg Control (High-level)
  - Inertial Measurement Unit Publisher 
  - Sensor Fusion
  - Twist Mux
  - D435i depth camera driver
  - Livox Mid360 lidar driver

These files can be viewed in the webserver and activated and deactivated from there.

- Launch the h1 rviz

```bash
ros2 launch h1_viz view_robot.launch.py
```

## Tele-Operation

- Teleop h1

```bash
ROS_DOMAIN_ID=10 ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/h1_unit_238/hardware/cmd_vel
```

## Core Launch Files

These files can be launched/activated from the webserver as well. The webserver should be on by default. The installation procedure should be followed first.

```bash
ros2 launch h1_platform highlevel_ros.launch.py 
```
```bash
ros2 launch h1_platform domain_bridge.launch.py 
```
```bash
ros2 launch h1_platform state_publisher.launch.py 
```
```bash
ros2 launch h1_webserver webserver.launch.py
```
```bash
ros2 launch h1_control twistmux.launch.py 
```
```bash
ros2 launch h1_lidar livox_mid360.launch.py 
```

### Arms H1 Core Launch Files

```bash
ros2 launch h1_description h1_description.launch.py
```

### Arms H1-2 Core Launch Files

```bash
ros2 launch h1_description h1_2_description.launch.py
ros2 launch h1_platform h1_2_arm_server.launch.py
```

### SDK Examples

```bash
cd /opt/mybotshop/04Aug2025_unitree_sdk2/build/
cmake .. && make && sudo make install
```

```bash
cd /opt/mybotshop/04Aug2025_unitree_sdk2/build/bin
./h1_loco_client --network_interface=eth0 --shake_hand
```

## Modes Selection

>[!IMPORTANT]
> Most functionalities are disable for Unitree H1. Startup requires remote.

### Posture & State Commands

| Command               | Description                                       |
| --------------------- | ------------------------------------------------- |
| `damp`                | Set all motors to damping mode.                   |
| `start`               | Start locomotion control.                         |
| `stand_up`            | Stand up from sitting/squatting.                  |
| `zero_torque`         | Disable torque on all motors.                     |
| `stop_move`           | Stop all motion immediately.                      |
| `high_stand`          | Stand at high position.                           |
| `low_stand`           | Stand at low position.                            |
| `balance_stand`       | Stand with balance mode active.                   |

### Setter Commands (Change Robot State)

| Command                             | Parameters                                        |
| ----------------------------------- | ------------------------------------------------- |
| `set_fsm_id=<id>`                   | Integer FSM ID to switch to.                      |
| `set_balance_mode=<0/1>`            | Enable (1) or disable (0) balance mode.           |
| `set_swing_height=<value>`          | Set swing height in meters.                       |
| `set_stand_height=<value>`          | Set stand height in meters.                       |
| `set_velocity="vx vy ω [duration]"` | Set velocity (m/s, m/s, rad/s, \[duration in s]). |
| `move="vx vy ω"`                    | Command motion without duration.                  |
| `set_task_id=<id>`                  | Set active task ID.                               |
| `set_speed_mode=<mode>`             | Change locomotion speed mode.                     |


### Getter Commands (Query Current Robot State)

| Command            | Returns                          |
| ------------------ | -------------------------------- |
| `get_fsm_id`       | Current finite state machine ID. |
| `get_fsm_mode`     | Current FSM mode.                |
| `get_balance_mode` | Current balance mode (0/1).      |
| `get_swing_height` | Current swing foot height.       |
| `get_stand_height` | Current standing height.         |
| `get_phase`        | Current gait phase vector.       |

### Toggle Commands

| Command                         | Parameters | Description                                     |
| ------------------------------- | ---------- | ----------------------------------------------- |
| `continous_gait=<true/false>`   | bool       | Enable or disable continuous gait mode.         |
| `switch_move_mode=<true/false>` | bool       | Switch between movement mode and standing mode. |

### Examples

- Damp State:

```bash
ROS_DOMAIN_ID=10 ros2 service call /h1_unit_238/hardware_modes h1_interface/srv/H1Modes \
'{"request_data": "damp"}'
```

## Dual Arm Record H1-2

>[!WARNING]
> When launch the arm driver the arm violently snaps to the H1-2 initial position. Ensure the arms are near to the start position to 
> prevent damage to the robot and keep surrounding area clear.

### Launch

```bash
ros2 launch h1_platform h1_2_arm_record.launch.py
```

### Configuration

Update to requirements the `/opt/mybotshop/src/mybotshop/h1_platform/config/h1_2_dual_arm_controller.yaml` file

### Start recording

>[!IMPORTANT]
> Keep in mind the arms will start moving from this area so try to keep away from the body to avoid scratches. In the end of the recording be 
> sure to bring the arms in a location close to the initial position for the next recording session.

```bash
ROS_DOMAIN_ID=10 ros2 service call /h1_unit_238/dual_arm/record std_srvs/srv/SetBool "data: true"
```

### Stop recording and save
```bash
ROS_DOMAIN_ID=10 ros2 service call /h1_unit_238/dual_arm/record std_srvs/srv/SetBool "data: false"
```

### Play last recorded trajectory  
```bash
ROS_DOMAIN_ID=10 ros2 service call /h1_unit_238/dual_arm/playback h1_interface/srv/H1Modes "request_data: ''"
```

### Play specific file
```bash
ROS_DOMAIN_ID=10 ros2 service call /h1_unit_238/dual_arm/playback h1_interface/srv/H1Modes "request_data: '/opt/mybotshop/h1_2_trajectories/trajectory_20250925_160052.csv'"
```

## Dual Arm Control H1-2

### Launch

```bash
ros2 launch h1_platform h1_2_arm_server.launch.py
```

### Configuration

Update to requirements the `/opt/mybotshop/src/mybotshop/h1_platform/config/h1_2_dual_arm_controller.yaml` file

### Poses

- T pose
```bash
ROS_DOMAIN_ID=10 ros2 action send_goal /h1_unit_1789/dual_arm/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory:
      {joint_names: [
        left_shoulder_pitch, left_shoulder_roll, left_shoulder_yaw, left_elbow,
        left_wrist_roll, left_wrist_pitch, left_wrist_yaw,
        right_shoulder_pitch, right_shoulder_roll, right_shoulder_yaw, right_elbow,
        right_wrist_roll, right_wrist_pitch, right_wrist_yaw],
       points: [
         {positions: [0.0, 0.3, 0.0, 1.57, 0.0, 0.0, 0.0,   0.0,-0.3, 0.0, 1.57, 0.0, 0.0, 0.0],
          time_from_start: {sec: 2, nanosec: 0}},
         {positions: [0.0, 1.57, 0.0, 1.57, 0.0, 0.0, 0.0,  0.0,-1.57,0.0, 1.57, 0.0, 0.0, 0.0],
          time_from_start: {sec: 5, nanosec: 0}}
       ]}}"
```

- Ready pose
```bash
ROS_DOMAIN_ID=10 ros2 action send_goal /h1_unit_1789/dual_arm/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory:
      {joint_names: [
        left_shoulder_pitch, left_shoulder_roll, left_shoulder_yaw, left_elbow,
        left_wrist_roll, left_wrist_pitch, left_wrist_yaw,
        right_shoulder_pitch, right_shoulder_roll, right_shoulder_yaw, right_elbow,
        right_wrist_roll, right_wrist_pitch, right_wrist_yaw],
       points: [
         {positions: [0.0, 0.3, 0.0, 0.7, 0.0, 0.0, 0.0,   
                      0.0,-0.3, 0.0, 0.7, 0.0, 0.0, 0.0],
          time_from_start: {sec: 2, nanosec: 0}},
       ]}}"
```

## Sensors

### Lidar Ouster

This should be running via services and can be controlled via the webserver or systemctl to activate or deactivate it. When turned off, you can test via:

```bash
ros2 launch h1_lidar ouster.launch.py
```

### Depth Camera Realsense D435i

Test the native driver via:

```bash
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
```

This should be running via services and can be controlled via the webserver or systemctl to activate or deactivate it. When turned off, you can test via:

```bash
ros2 launch h1_depth_camera realsense.launch.py
```

### Depth Camera ZED2i

This should be running via services in the NVIDIA bagpack and can be controlled via systemctl to activate or deactivate it. When turned off, you can test via:

```bash
ros2 launch h1_depth_camera zed.launch.py
```

#### Emlid M2

This should be running via services and can be controlled via the webserver or systemctl to activate or deactivate it. When turned off you can test via:

```bash
ros2 launch h1_gps emlid_m2.launch.py
```

For configuration, the webserver's VNC can be utilized. Open the browser and navigate to http://192.168.2.15 to configure the emlid M2's output, RTK settings, Base station connection, etc.

## Simulation

### Gazebo Fortress (Work In Progress)

```bash
ros2 launch h1_gazebo h1_2_fortress_simulation.launch.py
```

### Unitree RL Gym

For sim to real deployment, please follow instructions at [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) 

### Isaac Lab

Please refer to [Isaac lab](https://isaac-sim.github.io/IsaacLab/main/index.html) for training and deployment.

## Navigation

### Odometric Navigation

```bash
ros2 launch h1_navigation odom_navi.launch.py 
```

### Map Navigation

- Ensure the a map is generated and available in the ros package (i.e. after the map is saved, you have placed and performed ``colcon build``)

```bash
ros2 launch h1_navigation map_navi.launch.py 
```

## SLAM (Simultaneous Localization and Mapping)

- Ensure the **h1_bringup** is running and then also launch

```bash
ros2 launch h1_navigation slam.launch.py 
```

- You can begin mapping using the `teleop` at **0.2m/s** with the keyboard. Once you are satisfied with your map you can export it by running the following command:

```bash
ROS_DOMAIN_ID=10 ros2 run nav2_map_server map_saver_cli -f /opt/mybotshop/src/mybotshop/h1_navigation/maps/custom_map --ros-args --remap map:=/h1_unit_001/map
ROS_DOMAIN_ID=10 ros2 run nav2_map_server map_saver_cli -f /opt/mybotshop/src/mybotshop/h1_navigation/maps/custom_map_2 --ros-args --remap map:=/h1_unit_001/map
```

- Rebuild so that the maps can be found (This is required if the map name is not **map_** otherwise it will directly work)

```bash
colcon build --symlink-install
```

- Then source the environment

```bash
source /opt/mybotshop/install/setup.bash
```

# Debugging

## RQT

```bash
ROS_DOMAIN_ID=10 ros2 run rqt_gui rqt_gui --ros-args --remap tf:=/h1_unit_238/tf --ros-args --remap tf_static:=/h1_unit_238/tf_static
```

## Dynamic Reqconfigure

- From configure select Dynamic Reconfigure

```bash
ROS_DOMAIN_ID=10 ros2 run rqt_gui rqt_gui --ros-args --remap tf:=/h1_unit_238/tf --ros-args --remap tf_static:=/h1_unit_238/tf_static
```

## TF

```bash
ROS_DOMAIN_ID=10 ros2 run rqt_tf_tree rqt_tf_tree --force-discover --ros-args --remap tf:=/h1_unit_238/tf --ros-args --remap tf_static:=/h1_unit_238/tf_static
```

```bash
ROS_DOMAIN_ID=10 ros2 run tf2_tools view_frames.py --force-discover --ros-args --remap tf:=/h1_unit_238/tf --ros-args --remap tf_static:=/h1_unit_238/tf_static
```

## Video Stream
- Faster version

```bash
gst-launch-1.0 udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 ! \
    application/x-rtp, media=video, encoding-name=H264 ! \
    rtph264depay ! queue max-size-buffers=1 ! h264parse ! queue ! avdec_h264 ! \
    videoconvert ! autovideosink
```

# Miscellanious

## Update System Kernel Cyclone

```bash
printf "net.core.rmem_max=33554432\nnet.core.rmem_default=33554432\nnet.core.wmem_max=33554432\nnet.core.wmem_default=33554432\nnet.ipv4.udp_rmem_min=16384\nnet.ipv4.udp_wmem_min=16384\n" | sudo tee /etc/sysctl.d/90-ros2-dds.conf
```

```bash
sudo sysctl --system 
```

## New Livox Mid 360 Units Setup (First Time)

- Temporary ip setup for livox when reset

```bash
sudo ip addr flush dev eth0
sudo ip addr add 192.168.1.5/24 dev eth0
sudo ip link set eth0 up
```

## Sync host computer and unitree computer

```bash
rsync -avP -t --delete -e ssh src unitree@192.168.123.164://opt/mybotshop
```

- Steamdeck

```bash
rsync -avP -t --delete -e ssh src deck@192.168.68.162://home/deck/ros2_ws
```

## Xacro to URDF


```bash
ros2 run xacro xacro src/mybotshop/h1_description/xacro/h1_2.xacro > src/mybotshop/h1_description/urdf/h1_2.urdf
```

## Enable Internet 

### LAN

- This enables LAN via the ethernet cable. This has to be connected to a router with internet access

```bash 
sudo ip link set eth0 down && sudo ip link set eth0 up
sudo dhclient eth0
```

### ZED2i build

```bash
colcon build --symlink-install --packages-select zed_components zed_ros2 zed_wrapper h1_bringup h1_depth_camera h1_gps reach_rs h1_viz h1_description && source install/setup.bash
```
