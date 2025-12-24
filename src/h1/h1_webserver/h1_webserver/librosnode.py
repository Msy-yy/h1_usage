#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.

import os
import cv2
import time
import math
import psutil
import tf2_ros
import platform
import threading
import subprocess
import numpy as np

from flask import Flask, request
from waitress import serve
from playsound import playsound
from datetime import datetime, timedelta

from .libwebserver import setup_routes

from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import NavSatFix, BatteryState
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class ROS2WebServer(Node):

    def __init__(self):
        super().__init__('h1_webserver')
        self.get_logger().info(self.colorize("Starting ROS2 Web Server", "orange"))

        # Parameters Declaration
        self.declare_parameters(namespace='', parameters=[
            ('robot_map_topic', 'map'),
            ('robot_base_link', 'base_link'),
            ('robot_cmd_vel', 'cmd_vel'),
            ('robot_e_stop', 'e_stop'),
            ('robot_gps_topic', '/gps_unconfigured'),
            ('robot_battery_topic', 'battery'),
            ('robot_gps_lat', 0.0),
            ('robot_gps_lon', 0.0),
            ('robot_services', ['']),
            ('robot_webserver', 'robot-webserver'),
            ('robot_password', 'password'),
            ('robot_max_linear_velocity', 0.1),
            ('robot_max_angular_velocity', 0.0),
            ('robot_home_dir', '/home/administrator'),
            ('robot_rosbag_dir', '/opt/ros/bags/'),
            ('robot_rosbag_storage', 1000),
            ('robot_rosbag_duration', 600),
            ('web_security', True),
            ('web_user', 'administrator'),
            ('web_password', 'password'),
            ('web_server_ip', '0.0.0.0'),
            ('web_server_port', 9900),
        ])

        # Parameters Initialization
        self.param_robot_base_link = self.get_parameter(
            'robot_base_link').value
        self.param_robot_cmd_vel = self.get_parameter(
            'robot_cmd_vel').value
        self.param_robot_e_stop = self.get_parameter(
            'robot_e_stop').value
        self.param_robot_map_topic = self.get_parameter(
            'robot_map_topic').value
        self.param_robot_gps_topic = self.get_parameter(
            'robot_gps_topic').value
        self.param_robot_battery_topic = self.get_parameter(
            'robot_battery_topic').value
        self.param_robot_gps_lat = self.get_parameter('robot_gps_lat').value
        self.param_robot_gps_lon = self.get_parameter('robot_gps_lon').value
        self.param_robot_services = self.get_parameter('robot_services').value
        self.param_robot_home_dir = self.get_parameter('robot_home_dir').value
        self.param_robot_rosbag_dir = self.get_parameter(
            'robot_rosbag_dir').value
        self.param_robot_rosbag_storage = self.get_parameter(
            'robot_rosbag_storage').value
        self.param_robot_rosbag_duration = self.get_parameter(
            'robot_rosbag_duration').value
        self.param_robot_password = self.get_parameter('robot_password').value
        self.param_robot_webserver = self.get_parameter(
            'robot_webserver').value
        self.security_enabled = self.get_parameter('web_security').value
        self.param_web_user = self.get_parameter('web_user').value
        self.param_web_password = self.get_parameter('web_password').value
        self.param_web_server_ip = self.get_parameter('web_server_ip').value
        self.param_web_server_port = self.get_parameter(
            'web_server_port').value
        self.param_max_linear_velocity = self.get_parameter(
            'robot_max_linear_velocity').value
        self.param_max_angular_velocity = self.get_parameter(
            'robot_max_angular_velocity').value

        # Print Parameters
        self.get_logger().info(
            f'{self.colorize(f"security_enabled: {self.security_enabled}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"web_user: {self.param_web_user}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"web_password: {self.param_web_password}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"web_server_ip: {self.param_web_server_ip}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"web_server_port: {self.param_web_server_port}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"robot_webserver: {self.param_robot_webserver}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"robot_password: {self.param_robot_password}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"robot_services: {self.param_robot_services}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"robot_map_topic: {self.param_robot_map_topic}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"robot_battery_topic: {self.param_robot_battery_topic}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"robot_cmd_vel: {self.param_robot_cmd_vel}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"robot_e_stop: {self.param_robot_e_stop}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"robot_gps_topic: {self.param_robot_gps_topic}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"robot_gps_lat: {self.param_robot_gps_lat}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"robot_gps_lon: {self.param_robot_gps_lon}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"robot_base_link: {self.param_robot_base_link}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"robot_max_linear_velocity: {self.param_max_linear_velocity}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"robot_max_angular_velocity: {self.param_max_angular_velocity}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"robot_home_dir: {self.param_robot_home_dir}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"robot_rosbag_dir: {self.param_robot_rosbag_dir}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"robot_rosbag_storage: {self.param_robot_rosbag_storage}","blue")}')
        self.get_logger().info(
            f'{self.colorize(f"robot_rosbag_duration: {self.param_robot_rosbag_duration}","blue")}')

        # Audio for Webserver
        self.audio_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "audio")

        # Timer callback
        # self.create_timer(4, self.custom_timer_callback)

        # Publishers
        self.robot_cmdvel = self.create_publisher(
            Twist, self.param_robot_cmd_vel, 10)
        self.robot_estop = self.create_publisher(
            Bool, self.param_robot_e_stop, 10)

        # Subscribers
        qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        qos_d = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.create_subscription(
            OccupancyGrid, self.param_robot_map_topic, self.map_callback, qos)

        self.battery_sub = self.create_subscription(
            BatteryState, self.param_robot_battery_topic, self.battery_callback, qos_d)

        self.gps_subscriber = self.create_subscription(
            NavSatFix, self.param_robot_gps_topic, self.gps_callback, qos_d)

        # Activate TFs
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Declare Server Variables
        self.map_data = None
        self.map_data_raw = None
        self.robot_position = None
        self.slam_process = None
        self.map_navi_process = None
        self.bag_process = None
        self.battery_percentage = 0
        self.battery_voltage = 0.0
        self.charging_status = "Unknown"
        self.missions_ = []
        self.robot_gps_position = [
            self.param_robot_gps_lat, self.param_robot_gps_lon]
        self.declare_parameter('default_speed', 50)
        self.h1_webserver_dir = get_package_share_directory(
            'h1_webserver')
        self.audio_dir = os.path.join(
            self.h1_webserver_dir, 'h1_webserver/audio')
        self.waypoints_dir = os.path.join(
            self.h1_webserver_dir, 'navi_outdoor_waypoints/gps_coordinates.json')
        self.nogozones_dir = os.path.join(
            self.h1_webserver_dir, 'navi_outdoor_nogozones/gps_coordinates.json')
        self._last_net_io_counters = psutil.net_io_counters()

        # ROS2 Control Blocks
        self.control_block_11 = False
        self.control_block_12 = False
        self.control_block_13 = False
        self.control_block_14 = False
        self.control_block_15 = False
        self.control_block_16 = False
        self.control_block_17 = False
        self.control_block_18 = False

        self.control_block_21 = False
        self.control_block_22 = False
        self.control_block_23 = False
        self.control_block_24 = False
        self.control_block_25 = False
        self.control_block_26 = False
        self.control_block_27 = False
        self.control_block_28 = False

        self.control_block_31 = False
        self.control_block_32 = False
        self.control_block_33 = False
        self.control_block_34 = False
        self.control_block_35 = False
        self.control_block_36 = False
        self.control_block_37 = False
        self.control_block_38 = False

        self.control_block_41 = False
        self.control_block_42 = False
        self.control_block_43 = False
        self.control_block_44 = False
        self.control_block_45 = False
        self.control_block_46 = False
        self.control_block_47 = False
        self.control_block_48 = False
        
        ## LLM
        self.control_block_gpt = False
        
        # GPS Navi
        self.gps_navi = False
        
        # Rig
        self.rig1 = None
        self.rig2 = None
        self.rig3 = None
        
        # Emergency stop
        self.e_stop = None

        # Webserver
        self.app = Flask(__name__)
        self.app.secret_key = 'mybotshop2025!'
        self.setup_routes = setup_routes(self)
        self.server_thread = threading.Thread(
            target=self.run_flask_server, daemon=True)
        self.server_thread.start()

        self.get_logger().info(
            f'{self.colorize("ROS2 Web Server Started Successfully","green")}')

    def run_flask_server(self):
        # Production mode
        serve(
            self.app,
            host=self.param_web_server_ip,
            port=self.param_web_server_port,
            threads=4
        )
        # Development mode
        # self.app.run(
        #     host=self.param_web_server_ip,
        #     port=self.param_web_server_port,
        #     debug=False,
        #     use_reloader=False
        # )

    def get_system_specs(self):
        system_specs = []

        # Uptime info
        uptime_seconds = time.time() - psutil.boot_time()
        uptime = timedelta(seconds=uptime_seconds)
        days = uptime.days
        hours, remainder = divmod(uptime.seconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        uptime_str = f"{days} days, {hours:02}:{minutes:02}:{seconds:02}"
        system_specs.append(f"System Uptime:    {uptime_str}")

        # General system information
        system_specs.append(
            f"Operating System: {platform.system()} {platform.release()}")
        system_specs.append(f"Architecture:     {platform.architecture()[0]}")
        system_specs.append(f"Processor:        {platform.processor()}")

        # Memory info
        memory = psutil.virtual_memory()
        system_specs.append(
            f"Total Memory:     {memory.total / (1024 ** 3):.2f} GB")
        system_specs.append(
            f"Used Memory:      {memory.used / (1024 ** 3):.2f} GB")
        system_specs.append(
            f"Free Memory:      {memory.available / (1024 ** 3):.2f} GB")

        # CPU info
        system_specs.append(
            f"CPU Count:        {psutil.cpu_count(logical=True)}")
        system_specs.append(
            f"CPU Usage:        {psutil.cpu_percent(interval=1)}%")

        # Disk usage info
        disk = psutil.disk_usage('/')
        system_specs.append(
            f"Total Disk:       {disk.total / (1024 ** 3):.2f} GB")
        system_specs.append(
            f"Used Disk:        {disk.used / (1024 ** 3):.2f} GB")
        system_specs.append(
            f"Free Disk:        {disk.free / (1024 ** 3):.2f} GB")

        # Network I/O info (new addition)
        current_net_io_counters = psutil.net_io_counters()

        # Calculate bytes sent/received since the last check
        if hasattr(self, '_last_net_io_counters') and self._last_net_io_counters is not None:
            bytes_sent = current_net_io_counters.bytes_sent - \
                self._last_net_io_counters.bytes_sent
            bytes_recv = current_net_io_counters.bytes_recv - \
                self._last_net_io_counters.bytes_recv
        else:
            bytes_sent = 0
            bytes_recv = 0
            self._last_net_io_counters = current_net_io_counters

        # Update last counters for the next calculation
        self._last_net_io_counters = current_net_io_counters

        network_rate_mb_s = (bytes_sent + bytes_recv) / (1024 * 1024)
        system_specs.append(f"Network Rate:     {network_rate_mb_s:.2f} MB/s")

        # Temperature info
        try:
            temps = psutil.sensors_temperatures()
            if temps:
                # Try to get CPU temperature from common sensor names
                temp_value = None
                for name, entries in temps.items():
                    if any(keyword in name.lower() for keyword in ['cpu', 'core', 'processor']):
                        if entries:
                            temp_value = entries[0].current
                            break

                # If no CPU-specific sensor found, use the first available temperature
                if temp_value is None:
                    for name, entries in temps.items():
                        if entries:
                            temp_value = entries[0].current
                            break

                if temp_value is not None:
                    system_specs.append(
                        f"Temperature:      {temp_value:.1f}°C")
                else:
                    system_specs.append("Temperature:      N/A")
            else:
                system_specs.append("Temperature:      N/A")
        except (AttributeError, OSError):
            system_specs.append("Temperature:      N/A")

        return system_specs

    def map_callback(self, msg):
        """Process and store the map data."""
        self.map_data_raw = msg

        # Process map data to a grayscale image
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Normalize and convert OccupancyGrid data to image
        self.map_data = cv2.normalize(
            data, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

    def gps_callback(self, msg):
        self.robot_gps_position[0] = msg.latitude
        self.robot_gps_position[1] = msg.longitude

    def command_line_start(self, command):
        """
        Start the ROS2 launch process with the provided command list.

        Args:
        - command (list): A list of strings representing the command to run, e.g., ["ros2", "launch", "robot_nav2", "slam.launch.py"]

        Returns:
        - process: The subprocess.Popen instance representing the running command.
        """
        self.get_logger().error(self.colorize(
            f"Executing commands: {command}", "yellow"))
        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            return process
        except:
            self.get_logger().error(self.colorize(
                f"Command error", "red"))
            return None

    def command_line_stop(self, process):
        """
        Gracefully terminate the ROS2 launch process. If it does not terminate in time, force kill it.

        Args:
        - process (subprocess.Popen): The subprocess instance to terminate.
        """
        self.get_logger().info(self.colorize(
            f"Terminating Process: {process}", "yellow"))
        try:
            if process is None:
                self.get_logger().error(self.colorize(
                    f"Process is None", "red"))
                return
            pid = process.pid
            try:
                parent = psutil.Process(pid)
                children = parent.children(recursive=True)
                if children:
                    self.get_logger().info(self.colorize(
                        f"Killing Children of PID: {pid}", "yellow"))
                    for child in children:
                        child.terminate()
                        process.terminate()  # Gracefully terminate the main process
                        process.wait(timeout=5)
            except psutil.NoSuchProcess:
                self.get_logger().error(self.colorize(
                    f"No Such Process: {pid}", "red"))
            except psutil.AccessDenied:
                self.get_logger().error(self.colorize(
                    f"Permission Denied: {pid}", "red"))
            except Exception as e:
                self.get_logger().error(self.colorize(
                    f"Error in Termination: {pid}", "red"))
        except:
            self.get_logger().error(self.colorize(
                f"Process kill error", "red"))

    def sudo_command_line(self, command):
        try:
            cmd = f"echo {self.param_robot_password} | {command}"
            try:
                subprocess.run(cmd, check=True, shell=True, timeout=3)
            except:
                self.get_logger().error(f"Error : {cmd}")
        except subprocess.CalledProcessError as e:
            self.web_node.get_logger().error(
                f"Error reactivating services: {e}")

    def battery_callback(self, msg):
        try:
            percentage = msg.percentage
            voltage = msg.voltage

            if not isinstance(percentage, (int, float)) or math.isnan(percentage):
                self.get_logger().debug(
                    "Received non-numeric or NaN battery percentage, defaulting to 0.")
                self.battery_percentage = 0
            else:
                self.battery_percentage = percentage * 100 if percentage is not None else 0

            if not isinstance(voltage, (int, float)) or math.isnan(voltage):
                self.battery_voltage = 0.0
            else:
                self.battery_voltage = voltage if voltage is not None else 0.0

            status_map = {
                0: 'Unknown',
                1: 'Charging',
                2: 'Discharging',
                3: 'Not Charging',
                4: 'Full'
            }
            self.charging_status = status_map.get(
                msg.power_supply_status, 'Unknown')

        except Exception as e:
            self.get_logger().error(f"Error processing battery message: {e}")
            self.battery_percentage = 0
            self.battery_voltage = 0.0
            self.charging_status = 'Error'

    def ros2_bag_create(self):
        if self.bag_process is not None:
            self.get_logger().info(
                f'{self.colorize(f"Recording Already in Progress with PID {self.bag_process.pid}","red")}')
            return f"Recording Already in Progress with PID {self.bag_process.pid}"
        else:
            self.get_logger().info(
                f'{self.colorize(f"Starting Recording ROS2 bag","yellow")}')
            current_datetime = datetime.now()
            current_datetime_str = current_datetime.strftime(
                "%Y-%m-%d_%H-%M-%S")
            self.ros2bag_output = self.param_robot_rosbag_dir + current_datetime_str + '.bag'

            command = ['ros2', 'bag', 'record'] + [
                '-a',
                '-o', self.ros2bag_output,
                # '-d', str(self.param_robot_rosbag_duration), # Not in ROS2 Foxy
                '-b', str(self.param_robot_rosbag_storage),
                # '--repeat-latched'
            ]
            try:
                self.bag_process = subprocess.Popen(command)
                self.get_logger().info(
                    f'{self.colorize(f"Recording ROS2 bag with PID {self.bag_process.pid}","green")}')
                return f"Recording ROS2 bag with PID {self.bag_process.pid}"
            except subprocess.CalledProcessError as e:
                self.get_logger().info(
                    f'{self.colorize(f"Recording Failed {e}!","red")}')
                return f"Recording Failed {e}!"

    def ros2_bag_save(self):
        if self.bag_process:
            self.bag_process.terminate()
            self.bag_process.wait()
            self.get_logger().info(
                f'{self.colorize(f"Recording Stopped and is being saved! Please wait +1 min/s for every 10mins of recording","green")}')
            self.bag_process = None
            return 'Recording Stopped and is being saved! Please wait +1 min/s for every 10mins of recording'
        else:
            self.get_logger().info(
                f'{self.colorize(f"No Recording Process Found.","red")}')
            return 'No Recording Process Found.'

    def play_audio(self, audio_file="voice_test.mp3"):
        filepath = os.path.join(self.audio_dir, audio_file)
        self.get_logger().info(
            f'{self.colorize(f"Playing Audio (playsound - UACDemo): {filepath}","green")}')
        try:
            # From aplay -l -> Card digit, Device digit
            os.environ['AUDIODEV'] = 'hw:1,0'
            os.environ['ALSA_CARD'] = '1'
            os.environ['ALSA_PCM'] = 'plughw:1,0'

            playsound(filepath)
            return f"Audio successfully played {audio_file}."
        except Exception as e:
            self.get_logger().error(
                f'{self.colorize(f"Playsound Error: {e}","red")}')
            return f"Failed to play audio: {e}"

    def create_logs(self):
        log_file_name = f"robot_status_{datetime.now().strftime('%Y-%m-%d_H%H-M%M')}.txt"
        log_file_path = os.path.join(f"/opt/mybotshop/", log_file_name)

        try:
            command = f"touch {log_file_path}"
            subprocess.run(command, shell=True, check=True)
            self.get_logger().info(
                f'{self.colorize(f"Log File Created: {log_file_path}","green")}')
            with open(log_file_path, 'w') as log_file:
                for service in self.param_robot_services:
                    command = f"echo {self.param_robot_password} | sudo -S journalctl -u {service} -n 50"
                    self.get_logger().info(
                        f'{self.colorize(f"Executing Command: {command}","yellow")}')
                    result = subprocess.run(
                        command, shell=True, capture_output=True, text=True)
                    if result.returncode == 0:
                        log_file.write(f"Service: {service}\n")
                        log_file.write(result.stdout + "\n")
                    else:
                        log_file.write(
                            f"Service: {service} - Error: {result.stderr}\n")
            return f"Log file created successfully: {log_file_path}"
        except Exception as e:
            self.get_logger().error(
                f'{self.colorize(f"Error creating log file: {e}","red")}')
            return f"Failed to create log file: {e}"

    def colorize(self, text, color):
        color_codes = {
            'green': '\033[92m',
            'purple': '\033[95m',
            'cyan': '\033[96m',
            'yellow': '\033[93m',
            'orange': '\033[38;5;208m',
            'blue': '\033[94m',
            'red': '\033[91m'
        }
        return color_codes[color] + text + '\033[0m'

    def destroy_node(self):
        self.get_logger().info(self.colorize("Shutting down ROS2 Web Server", "red"))

        # Stop any running subprocesses first
        if self.bag_process:
            self.ros2_bag_save()

        if self.slam_process:
            self.command_line_stop(self.slam_process)

        if self.map_navi_process:
            self.command_line_stop(self.map_navi_process)

        # Shutdown Flask using request context
        try:
            if hasattr(self, 'app'):
                self.app.config['SERVER_NAME'] = None
                ctx = self.app.test_request_context()
                ctx.push()
                func = request.environ.get('werkzeug.server.shutdown')
                if func:
                    func()
                ctx.pop()
        except Exception as e:
            self.get_logger().error(f"Error shutting down Flask: {e}")

        # Wait for the thread to finish with timeout
        if hasattr(self, 'server_thread') and self.server_thread.is_alive():
            self.server_thread.join(timeout=1.0)
            if self.server_thread.is_alive():
                self.get_logger().warning("Forcing Flask thread termination")

        super().destroy_node()
