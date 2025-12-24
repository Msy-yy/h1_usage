# Dev Test

## Dual Arm Record H1-2

### Launch

```bash
ros2 launch h1_platform h1_2_arm_record.launch.py
```

### Configuration

Update to requirements the `/opt/mybotshop/src/mybotshop/h1_platform/config/h1_2_dual_arm_controller.yaml` file

### Start recording
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
ROS_DOMAIN_ID=10 ros2 service call /h1_unit_238/dual_arm/playback h1_interface/srv/H1Modes "request_data: '/opt/mybotshop/h1_trajectories/trajectory_20250925_152349.csv'"
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
ROS_DOMAIN_ID=10 ros2 action send_goal /h1_unit_238/dual_arm/follow_joint_trajectory \
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
ROS_DOMAIN_ID=10 ros2 action send_goal /h1_unit_238/dual_arm/follow_joint_trajectory \
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