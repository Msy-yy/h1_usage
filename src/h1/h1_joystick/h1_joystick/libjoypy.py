#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2024, MYBOTSHOP GmbH, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of MYBOTSHOP GmbH nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of MYBOTSHOP GmbH.

import os
import time
import rclpy
import subprocess

from sensor_msgs.msg import Joy


class CustomJoyControls:

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('h1_joystick_commands')

        # ---- Configuration ----
        self.DEFAULT_TRAJ_PATH = '/opt/mybotshop/h1_2_trajectories/trajectory_20251002_110614.csv'
        self.ROS_DOMAIN_ID = '10'
        # -----------------------

        self.subscription = self.node.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

    def execute_program(self):
        rclpy.spin(self.node)

    def joy_callback(self, msg):

        self.node.get_logger().info(
            f'{self.colorize("GO2 Logitech F710 Joystick Activated","green")}', once=True)

        # ------------- H1 Dual-Arm controls -------------
        # LT + B : start recording (SetBool true)
        if msg.axes[2] == -1.0 and msg.buttons[1] == 1:  # LT + B
            self.node.get_logger().info(
                f'{self.colorize("H1 Dual-Arm: RECORD START","orange")}')
            svc = "/h1_unit_238/dual_arm/record"
            command = f"ros2 service call {svc} std_srvs/srv/SetBool \"data: true\""
            self.execute_ros2_command(command)

        # LT + X : stop recording (SetBool false)
        if msg.axes[2] == -1.0 and msg.buttons[2] == 1:  # LT + X
            self.node.get_logger().info(
                f'{self.colorize("H1 Dual-Arm: RECORD STOP","orange")}')
            svc = "/h1_unit_238/dual_arm/record"
            command = f"ros2 service call {svc} std_srvs/srv/SetBool \"data: false\""
            self.execute_ros2_command(command)

        # LT + Down : playback with empty request_data
        if msg.axes[2] == -1.0 and msg.axes[7] == -1.0:  # LT + D-Pad Down
            self.node.get_logger().info(
                f'{self.colorize("H1 Dual-Arm: PLAYBACK (last/empty)","orange")}')
            svc = "/h1_unit_238/dual_arm/playback"
            command = (
                f"ros2 service call {svc} h1_interface/srv/H1Modes "
                "\"request_data: ''\""
            )
            self.execute_ros2_command(command)

        # LT + Right : playback with file path
        if msg.axes[2] == -1.0 and msg.axes[6] == -1.0:  # LT + D-Pad Right
            self.node.get_logger().info(
                f'{self.colorize("H1 Dual-Arm: PLAYBACK (file path)","orange")}')
            svc = "/h1_unit_238/dual_arm/playback"
            command = (
                f"ros2 service call {svc} h1_interface/srv/H1Modes "
                f"\"request_data: '{self.DEFAULT_TRAJ_PATH}'\""
            )
            self.execute_ros2_command(command)

        # LT + Up : also playback with file path (second shortcut)
        if msg.axes[2] == -1.0 and msg.axes[7] == 1.0:  # LT + D-Pad Up
            self.node.get_logger().info(
                f'{self.colorize("H1 Dual-Arm: PLAYBACK (file path)","orange")}')
            svc = "/h1_unit_238/dual_arm/playback"
            command = (
                f"ros2 service call {svc} h1_interface/srv/H1Modes "
                f"\"request_data: '{self.DEFAULT_TRAJ_PATH}'\""
            )
            self.execute_ros2_command(command)
            
        # RT + B : HARDWARE MODE -> DAMP
        if msg.axes[5] <= -0.9 and msg.buttons[1] == 1:  # RT + B
                self.node.get_logger().info(
                    f'{self.colorize("H1: HARDWARE MODE -> DAMP","orange")}')
                svc = "/h1_unit_238/hardware_modes"
                command = (
                    f"ros2 service call {svc} h1_interface/srv/H1Modes "
                    "\"request_data: 'damp'\""
                )
                self.execute_ros2_command(command)

    def execute_ros2_command(self, command: str):
        """Execute a ros2 CLI command with ROS_DOMAIN_ID enforced."""
        try:
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = self.ROS_DOMAIN_ID

            process = subprocess.Popen(
                command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env
            )
            stdout, stderr = process.communicate()
            time.sleep(2)

            if process.returncode == 0:
                print("Command executed successfully:")
                print(stdout.decode('utf-8'))
            else:
                print("Error executing command:")
                print(stderr.decode('utf-8'))

        except Exception as e:
            print("Exception occurred:", e)

    def colorize(self, text, color):
        color_codes = {
            'green': '\033[92m',
            'yellow': '\033[93m',
            'orange': '\033[38;5;208m',
            'blue': '\033[94m',
            'red': '\033[91m'
        }
        return color_codes[color] + text + '\033[0m'
