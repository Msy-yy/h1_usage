- Install ROS2 Humble -> Gazebo Fortress via:

```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-gz-ros2-control
```

- Usage example

```bash
ros2 action send_goal /joint_effort_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory -f "{
  trajectory: {                                
    joint_names: [                                     
      'torso_joint', 
    ],
    points: [
      {
        positions: [1.57,],
        time_from_start: {sec: 2, nanosec: 0}
      }                                      
    ]
  }
}"
```