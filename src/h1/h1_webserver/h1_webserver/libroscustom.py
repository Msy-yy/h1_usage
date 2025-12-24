#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.

from std_msgs.msg import Bool
from rclpy.action import ActionClient
# -------------------------------------------- Import Custom Interfaces
from h1_interface.srv import H1Modes
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# -------------------------------------------- Advanced Features
# from h1_webserver.libllm import ROS2LLM
# from h1_webserver.libgpsconverter import GpsConverter


# -------------------------------------------- Open Source Web Interface
class ROS2CustomWebInterface():

    def __init__(self, web_node):

        self.web_node = web_node
        self.web_node.create_timer(
            1, self.ros2_webserver_control_buttons_callback)

        # -------------------------------------------- Advanced Features
        # self.llm = ROS2LLM(self.web_node)
        # self.gps_converter = GpsConverter(self.web_node)
        
        # -------------------------------------------- G1 Control Client
        self.h1_hardware_client = self.web_node.create_client(H1Modes, 'hardware_modes')
        
        self.web_node.get_logger().info(self.web_node.colorize(
            "ROS2 Custom Web Interface Loaded", "green"))

    # -------------------------------------------- Webserver Control Buttons
    def ros2_webserver_control_buttons_callback(self):

        try:
            # -------------------------------------------- Robot Computer Block 1
            if self.web_node.control_block_11:
                self.web_node.get_logger().info("Control Block 11 is active")
                self.h1_hardware_client.call_async(H1Modes.Request(request_data='damp'))
                self.web_node.control_block_11 = False
            elif self.web_node.control_block_12:
                self.web_node.get_logger().info("Control Block 12 is active")
                self.send_follow_trajectory()
                self.web_node.control_block_12 = False
            elif self.web_node.control_block_13:
                self.web_node.get_logger().info("Control Block 13 is active")
                self.send_follow_trajectory_cross()
                self.web_node.control_block_13 = False
            elif self.web_node.control_block_14:
                self.web_node.get_logger().info("Control Block 14 is active")
                self.web_node.control_block_14 = False
            elif self.web_node.control_block_15:
                self.web_node.get_logger().info("Control Block 15 is active")
                self.cancel_follow_trajectory()
                self.web_node.control_block_15 = False
            elif self.web_node.control_block_16:
                self.web_node.get_logger().info("Control Block 16 is active")
                self.send_follow_trajectory_down()
                self.web_node.control_block_16 = False
            elif self.web_node.control_block_17:
                self.web_node.get_logger().info("Control Block 17 is active")
                self.send_follow_trajectory_uncross()
                self.web_node.control_block_17 = False
            elif self.web_node.control_block_18:
                self.web_node.get_logger().info("Control Block 18 is active")
                self.web_node.control_block_18 = False

            # -------------------------------------------- Robot Computer Block 2
            elif self.web_node.control_block_21:
                self.web_node.get_logger().info("Control Block 21 is active")
                self.web_node.control_block_21 = False
            elif self.web_node.control_block_22:
                self.web_node.get_logger().info("Control Block 22 is active")
                self.web_node.control_block_22 = False
            elif self.web_node.control_block_23:
                self.web_node.get_logger().info("Control Block 23 is active")
                self.web_node.control_block_23 = False
            elif self.web_node.control_block_24:
                self.web_node.get_logger().info("Control Block 24 is active")
                self.web_node.control_block_24 = False
            elif self.web_node.control_block_25:
                self.web_node.get_logger().info("Control Block 25 is active")
                self.web_node.control_block_25 = False
            elif self.web_node.control_block_26:
                self.web_node.get_logger().info("Control Block 26 is active")
                self.web_node.control_block_26 = False
            elif self.web_node.control_block_27:
                self.web_node.get_logger().info("Control Block 27 is active")
                self.web_node.control_block_27 = False
            elif self.web_node.control_block_28:
                self.web_node.get_logger().info("Control Block 28 is active")
                self.web_node.control_block_28 = False

            # -------------------------------------------- Robot Computer Block 3
            elif self.web_node.control_block_31:
                self.web_node.get_logger().info("Control Block 31 is active")
                self.web_node.control_block_31 = False
            elif self.web_node.control_block_32:
                self.web_node.get_logger().info("Control Block 32 is active")
                self.web_node.control_block_32 = False
            elif self.web_node.control_block_33:
                self.web_node.get_logger().info("Control Block 33 is active")
                self.web_node.control_block_33 = False
            elif self.web_node.control_block_34:
                self.web_node.get_logger().info("Control Block 34 is active")
                self.web_node.control_block_34 = False
            elif self.web_node.control_block_35:
                self.web_node.get_logger().info("Control Block 35 is active")
                self.web_node.control_block_35 = False
            elif self.web_node.control_block_36:
                self.web_node.get_logger().info("Control Block 36 is active")
                self.web_node.control_block_36 = False
            elif self.web_node.control_block_37:
                self.web_node.get_logger().info("Control Block 37 is active")
                self.web_node.control_block_37 = False
            elif self.web_node.control_block_38:
                self.web_node.get_logger().info("Control Block 38 is active")
                self.web_node.control_block_38 = False

            # -------------------------------------------- Robot Computer Block 4
            elif self.web_node.control_block_41:  # Reboot
                self.web_node.get_logger().info("Control Block 41 is active")
                self.web_node.sudo_command_line("sudo -S shutdown -r now")
                self.web_node.control_block_41 = False
            elif self.web_node.control_block_42:  # Update system
                self.web_node.get_logger().info("Control Block 42 is active")
                # self.web_node.sudo_command_line("sudo -S apt-get update")
                self.web_node.control_block_42 = False
            elif self.web_node.control_block_43:  # Clear Journal and History
                self.web_node.get_logger().info("Control Block 43 is active")
                # self.web_node.sudo_command_line("sudo -S journalctl --vacuum-time=7d; history -c")
                self.web_node.control_block_43 = False
            elif self.web_node.control_block_44:  # Reload Udev Rules
                self.web_node.get_logger().info("Control Block 44 is active")
                self.web_node.sudo_command_line(
                    "sudo -S udevadm control --reload && sudo -S udevadm trigger")
                self.web_node.control_block_44 = False
            elif self.web_node.control_block_45:  # Shutdown
                self.web_node.get_logger().info("Control Block 45 is active")
                self.web_node.sudo_command_line("sudo -S shutdown -h now")
                self.web_node.control_block_45 = False
            elif self.web_node.control_block_46:  # Upgrade System
                self.web_node.get_logger().info("Control Block 46 is active")
                # self.web_node.sudo_command_line("sudo -S apt upgrade -y")
                self.web_node.control_block_46 = False
            elif self.web_node.control_block_47:  # Restart NetworkManager
                self.web_node.get_logger().info("Control Block 47 is active")
                self.web_node.sudo_command_line(
                    "sudo -S systemctl restart NetworkManager")
                self.web_node.control_block_47 = False
            elif self.web_node.control_block_48:  # Clear Memory Cache
                self.web_node.get_logger().info("Control Block 48 is active")
                self.web_node.sudo_command_line(
                    "sudo -S sync; sudo -S sysctl -w vm.drop_caches=3")
                self.web_node.control_block_48 = False

            # -------------------------------------------- GPS Navi
            elif self.web_node.gps_navi:
                self.web_node.get_logger().info("GPS Waypoint is active")
                self.gps_converter.convert_waypoints()
                self.web_node.gps_navi = False
                
            # -------------------------------------------- LLM Action
            elif self.web_node.control_block_gpt:
                self.web_node.get_logger().info("Control Block GPT is active")
                self.web_node.control_block_gpt = False
                # self.llm.driver()
                
        except Exception as e:
            self.web_node.get_logger().error(
                f"Error in ros2_webserver_control_buttons_callback: {e}")

        # -------------------------------------------- Slider Rig
        try:
            if self.web_node.rig1 or self.web_node.rig2 or self.web_node.rig3:
                self.web_node.get_logger().info("Rig 1: " + str(self.web_node.rig1)
                                                + " Rig 2: " +
                                                str(self.web_node.rig2)
                                                + " Rig 3: " + str(self.web_node.rig3))

                # self.rig_control.base_plate = self.web_node.rig1
                # self.rig_control.left_plate = self.web_node.rig2
                # self.rig_control.right_plate = self.web_node.rig3
                # self.srv_rig_control.call_async(self.rig_control)

                # Reset Mechanism
                self.web_node.rig1 = None
                self.web_node.rig2 = None
                self.web_node.rig3 = None
        except Exception as e:
            self.web_node.get_logger().error(f"Rig Slider Error: {e}")

        try:
            if self.web_node.e_stop == 200:
                self.web_node.robot_estop.publish(Bool(data=True))
            if self.web_node.e_stop == 250:
                self.web_node.robot_estop.publish(Bool(data=False))
                self.web_node.e_stop = None
        except Exception as e:
            self.web_node.get_logger().error(f"Robot E-Stop Error: {e}")    