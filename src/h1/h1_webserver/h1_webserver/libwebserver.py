#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.

import os
import cv2
import math
import json
import rclpy
import subprocess
import numpy as np

from werkzeug.utils import secure_filename
from ament_index_python.packages import get_package_share_directory
from flask import request, jsonify, render_template, redirect, url_for, session

from geometry_msgs.msg import Twist
from tf2_ros import TransformException


def setup_routes(node):

    @node.app.route('/')
    def home_page():
        if node.security_enabled:
            if 'logged_in' in session and session['logged_in']:
                return render_template('index.html')
            else:
                return redirect(url_for('login'))
        else:
            return render_template('index.html')

    @node.app.route('/login', methods=['GET', 'POST'])
    def login():
        if request.method == 'POST':
            entered_username = request.form['username']
            entered_password = request.form['password']
            if entered_username == node.param_web_user and entered_password == node.param_web_password:
                session['logged_in'] = True  # Set logged-in session
                return redirect(url_for('home_page'))
            else:
                return "Incorrect password! Please try again.", 403  # Forbidden error
        return render_template('login.html')

    @node.app.route('/logout')
    def logout():
        session.pop('logged_in', None)  # Clear the session to log out
        return redirect(url_for('login'))

    @node.app.route('/navi_general_page')
    def navi_general_page():
        if node.security_enabled:
            if 'logged_in' in session and session['logged_in']:
                return render_template('navi_general.html')
            else:
                return redirect(url_for('login'))
        else:
            return render_template('navi_general.html')
        
    @node.app.route('/navi_indoor_page')
    def navi_indoor_page():
        if node.security_enabled:
            if 'logged_in' in session and session['logged_in']:
                return render_template('navi_indoor.html')
            else:
                return redirect(url_for('login'))
        else:
            return render_template('navi_indoor.html')
    
    @node.app.route('/navi_outdoor_page')
    def navi_outdoor_page():
        if node.security_enabled:
            if 'logged_in' in session and session['logged_in']:
                return render_template('navi_outdoor.html')
            else:
                return redirect(url_for('login'))
        else:
            return render_template('navi_outdoor.html')

    @node.app.route('/console_page')
    def console_page():
        if node.security_enabled:
            if 'logged_in' in session and session['logged_in']:
                return render_template('console.html')
            else:
                return redirect(url_for('login'))
        else:
            return render_template('console.html')
    
    @node.app.route('/vnc_page')
    def vnc_page():
        if node.security_enabled:
            if 'logged_in' in session and session['logged_in']:
                return render_template('vnc.html')
            else:
                return redirect(url_for('login'))
        else:
            return render_template('vnc.html')
    
    @node.app.route('/safety_page')
    def safety_page():
        if node.security_enabled:
            if 'logged_in' in session and session['logged_in']:
                return render_template('safety.html')
            else:
                return redirect(url_for('login'))
        else:
            return render_template('safety.html')
        
    @node.app.route('/restart_vnc', methods=['POST'])
    def restart_vnc():
        try:        
            home_env = os.environ.copy()
            home_env['HOME'] = node.param_robot_home_dir
            try:
                subprocess.run(['vncserver', '-kill', ':1'], check=True)
            except:
                pass
            subprocess.run([
                'vncserver', ':1',
                '-geometry', '1920x1080',
                '-depth', '24',
                '-localhost', 'no'
            ], check=True, env=home_env)
            return jsonify({'status': 'success'})
        except subprocess.CalledProcessError as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500
        
    @node.app.route('/save_audio', methods=['POST'])
    def save_audio():
        try:
            if 'audio' not in request.files:
                node.get_logger().error(node.colorize(
                    "No file provided", "red"
                ))
                return jsonify({"status": "error", "message": "No file provided"}), 400

            file = request.files['audio']
            if file.filename == '':
                node.get_logger().error(node.colorize(
                    "No filename", "red"
                ))
                return jsonify({"status": "error", "message": "No filename"}), 400

            filename = secure_filename(file.filename)
            save_path = os.path.join(node.audio_dir, filename)
            os.makedirs(node.audio_dir, exist_ok=True)
            node.get_logger().info(node.colorize(
                f"Saving Audio to: {save_path}", "blue"
            ))
            file.save(save_path)

            return jsonify({"status": "success", "message": "AUDIO SAVED", "file": filename}), 200
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 500

    @node.app.route('/slider_control', methods=['POST'])
    def slider_control():
        data = request.json

        linear_velocity = float(data.get('linear_velocity'))
        angular_velocity = float(data.get('angular_velocity'))

        node.param_max_linear_velocity = linear_velocity / 100.0
        node.param_max_angular_velocity = angular_velocity / 100.0

        node.get_logger().info(node.colorize(
            f"Received Max Linear Velocity: {node.param_max_linear_velocity}", "yellow"))
        node.get_logger().info(node.colorize(
            f"Received Max Angular Velocity: {node.param_max_angular_velocity}", "yellow"))

        return jsonify({"message": "Slider Data Received Successfully"})
    
    @node.app.route('/rig_control', methods=['POST'])
    def rig_control():
        data = request.json

        node.rig1 = float(data.get('rig_base'))
        node.rig2 = float(data.get('rig_left'))
        node.rig3 = float(data.get('rig_right'))

        return jsonify({"message": "Rig Slider Data Received Successfully"})

    @node.app.route('/web_joystick_control', methods=['POST'])
    def web_joystick_control():
        twist = Twist()
        data = request.json

        joystick_id = data.get('joystickId', 'joystick1')  # Default to joystick1
        angle = data.get('angle', 0)
        distance = data.get('distance', 0)

        node.get_logger().info(node.colorize(
            f"Joystick ({joystick_id}): angle={angle}°, distance={distance}", "yellow"))

        # Convert distance from cm to meters and scale
        distance_m = distance * 2.0 / 200.0

        x_velocity = y_velocity = z_velocity = yaw_velocity = 0.0

        if distance != 0:
            angle_rad = math.radians(angle)

            if joystick_id == 'joystick1':
                x_velocity = distance_m * math.sin(angle_rad)
                yaw_velocity = distance_m * math.cos(angle_rad) * -1.0
            elif joystick_id == 'joystick2':
                z_velocity = distance_m * math.sin(angle_rad) 
                y_velocity = distance_m * math.cos(angle_rad) * -1.0

        node.get_logger().info(node.colorize(
            f"Computed velocities - x: {x_velocity:.2f}, y: {y_velocity:.2f}, z: {z_velocity:.2f} ,yaw: {yaw_velocity:.2f}", "yellow"))

        # Clamp velocities to max limits
        twist.linear.x =  float(max(min(x_velocity, node.param_max_linear_velocity), -node.param_max_linear_velocity))
        twist.linear.y =  float(max(min(y_velocity, node.param_max_linear_velocity), -node.param_max_linear_velocity))
        twist.linear.z =  float(max(min(z_velocity, node.param_max_linear_velocity), -node.param_max_linear_velocity))
        twist.angular.z = float(max(min(yaw_velocity, node.param_max_angular_velocity), -node.param_max_angular_velocity))

        node.get_logger().info(node.colorize(
            f"Twist command - linear.x={twist.linear.x:.2f}, linear.y={twist.linear.y:.2f}, linear.z={twist.linear.z:.2f}, angular.z={twist.angular.z:.2f}", "green"))

        node.robot_cmdvel.publish(twist)

        return jsonify({"message": f'Joystick: {joystick_id}, Velocity: linear.x={twist.linear.x:.2f}, linear.y={twist.linear.y:.2f}, angular.z={twist.angular.z:.2f}'})


    @node.app.route('/web_button', methods=['POST'])
    def web_button():
        data = request.json 
        action = data.get('action')  
        node.get_logger().info(node.colorize(
            f"Received action: {action}", "yellow"))        
        # ------------------------------------------------------- Control Commands -------------------------------------------------------
        if action == "e_stop":
            node.get_logger().info(node.colorize(
                f"Emergency Stop Activated", "red"))            
            node.e_stop = 200
            return jsonify({"message": "Emergency Stop Activated"})

        elif action == "e_stop_release":
            node.get_logger().info(node.colorize(
                f"Emergency Stop Deactivated", "green"))
            node.e_stop = 250               
            return jsonify({"message": "Emergency Stop Deactivated"})    
        # ------------------------------------------------------- Log Commands -------------------------------------------------------
        elif action == "create_logs":
            response = node.create_logs()
            return jsonify({"message": f"{response}"})
        # ------------------------------------------------------- Audio -------------------------------------------------------
        elif action == "audio_voice_test":
            response = node.play_audio("voice_test.mp3")
            return jsonify({"message": response})

        elif action == "audio_obstruction":
            response = node.play_audio("obstruction.mp3")
            return jsonify({"message": response})

        elif action == "audio_car_horn":
            response = node.play_audio("car_horn.mp3")
            return jsonify({"message": response})

        elif action == "audio_boat_horn":
            response = node.play_audio("boat_horn.mp3")
            return jsonify({"message": response})
        # ------------------------------------------------------- ROS2 Bag Commands -------------------------------------------------------
        elif action == "record_bag":
            response = node.ros2_bag_create()
            return jsonify({"message": response})

        elif action == "stop_record_bag":
            response = node.ros2_bag_save()
            return jsonify({"message": response})
       
        # ------------------------------------------------------- SLAM Commands -------------------------------------------------------
        elif action == "start_slam":
            if node.slam_process == None:
                node.get_logger().info(node.colorize(
                    f"Starting SLAM", "green"))
                node.slam_process = node.command_line_start(
                    ["ros2", "launch", "mmp_nav2", "slam.launch.py"])
                return jsonify({"message": "SLAM Activated"})
            elif node.slam_process != None:
                node.get_logger().info(node.colorize(
                    f"SLAM already running", "orange"))
                return jsonify({"message": "SLAM already running"})

        elif action == "stop_slam":
            if node.slam_process != None:
                node.command_line_stop(node.slam_process)
                node.slam_process = None
                node.get_logger().info(node.colorize(
                    f"SLAM Deactivated", "red"))
                return jsonify({"message": "SLAM Deactivated"})
            else:
                node.get_logger().info(node.colorize(
                    f"SLAM already deactivated", "orange"))
                return jsonify({"message": "SLAM already deactivated"})
            
        elif action == "save_map":
            node.command_line_start(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f",
                    "/opt/mybotshop/src/mybotshop/mmp_nav2/maps/custom_map",
                    "--ros-args", "--remap",
                    "map:=/mmp_0110/map"])

            node.command_line_start(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f",
                    "/opt/mybotshop/src/mybotshop/mmp_nav2/maps/custom_map_2",
                    "--ros-args", "--remap",
                    "map:=/mmp_0110/map"])
            return jsonify({"message": "Saving 2D Map"})
        # ------------------------------------------------------- Map Navigation Commands -------------------------------------------------------
        elif action == "start_map_navi":
            if node.map_navi_process == None:
                node.get_logger().info(node.colorize(
                    f"Starting Map Navigation", "green"))
                node.map_navi_process = node.command_line_start(
                    ["ros2", "launch", "mmp_nav2", "map_navi.launch.py"])
                return jsonify({"message": "Map Navigation Activated"})
            elif node.map_navi_process != None:
                node.get_logger().info(node.colorize(
                    f"Map Navigation already running", "orange"))
                return jsonify({"message": "Map Navigation already running"})

        elif action == "stop_map_navi":
            if node.map_navi_process != None:
                node.command_line_stop(node.map_navi_process)
                node.map_navi_process = None
                node.get_logger().info(node.colorize(
                    f"Map Navigation Deactivated", "red"))
                return jsonify({"message": "Map Navigation Deactivated"})
            else:
                node.get_logger().info(node.colorize(
                    f"Map Navigation already deactivated", "orange"))
                return jsonify({"message": "Map Navigation already deactivated"})
            
        # ------------------------------------------------------- GPS Navigation Commands -------------------------------------------------------
        elif action == "gps_navi_start":
            node.gps_navi = True
            return jsonify({"message": "GPS Navigation Activated"})
        elif action == "gps_navi_stop":
            node.gps_navi = False
            return jsonify({"message": "GPS Navigation Deactivated"})
        # ------------------------------------------------------- ROS2 Control Commands -------------------------------------------------------
        elif action == "control_block_11":
            node.control_block_11 = True
            return jsonify({"message": "Control Block 11 Activated"})
        elif action == "control_block_12":
            node.control_block_12 = True
            return jsonify({"message": "Control Block 12 Activated"})
        elif action == "control_block_13":
            node.control_block_13 = True
            return jsonify({"message": "Control Block 13 Activated"})
        elif action == "control_block_14":
            node.control_block_14 = True
            return jsonify({"message": "Control Block 14 Activated"})
        elif action == "control_block_15":
            node.control_block_15 = True
            return jsonify({"message": "Control Block 15 Activated"})
        elif action == "control_block_16":
            node.control_block_16 = True
            return jsonify({"message": "Control Block 16 Activated"})
        elif action == "control_block_17":
            node.control_block_17 = True
            return jsonify({"message": "Control Block 17 Activated"})
        elif action == "control_block_18":
            node.control_block_18 = True
            return jsonify({"message": "Control Block 18 Activated"})
            
        elif action == "control_block_21":
            node.control_block_21 = True
            return jsonify({"message": "Control Block 21 Activated"})
        elif action == "control_block_22":
            node.control_block_22 = True
            return jsonify({"message": "Control Block 22 Activated"})
        elif action == "control_block_23":
            node.control_block_23 = True
            return jsonify({"message": "Control Block 23 Activated"})
        elif action == "control_block_24":
            node.control_block_24 = True
            return jsonify({"message": "Control Block 24 Activated"})
        elif action == "control_block_25":
            node.control_block_25 = True
            return jsonify({"message": "Control Block 25 Activated"})
        elif action == "control_block_26":
            node.control_block_26 = True
            return jsonify({"message": "Control Block 26 Activated"})
        elif action == "control_block_27":
            node.control_block_27 = True
            return jsonify({"message": "Control Block 27 Activated"})
        elif action == "control_block_28":
            node.control_block_28 = True
            return jsonify({"message": "Control Block 28 Activated"})
            
        elif action == "control_block_31":
            node.control_block_31 = True
            return jsonify({"message": "Control Block 31 Activated"})
        elif action == "control_block_32":
            node.control_block_32 = True
            return jsonify({"message": "Control Block 32 Activated"})
        elif action == "control_block_33":
            node.control_block_33 = True
            return jsonify({"message": "Control Block 33 Activated"})
        elif action == "control_block_34":
            node.control_block_34 = True
            return jsonify({"message": "Control Block 34 Activated"})
        elif action == "control_block_35":
            node.control_block_35 = True
            return jsonify({"message": "Control Block 35 Activated"})
        elif action == "control_block_36":
            node.control_block_36 = True
            return jsonify({"message": "Control Block 36 Activated"})
        elif action == "control_block_37":
            node.control_block_37 = True
            return jsonify({"message": "Control Block 37 Activated"})
        elif action == "control_block_38":
            node.control_block_38 = True
            return jsonify({"message": "Control Block 38 Activated"})
            
        elif action == "control_block_41":
            node.control_block_41 = True
            return jsonify({"message": "Control Block 41 Activated"})
        elif action == "control_block_42":
            node.control_block_42 = True
            return jsonify({"message": "Control Block 42 Activated"})
        elif action == "control_block_43":
            node.control_block_43 = True
            return jsonify({"message": "Control Block 43 Activated"})
        elif action == "control_block_44":
            node.control_block_44 = True
            return jsonify({"message": "Control Block 44 Activated"})
        elif action == "control_block_45":
            node.control_block_45 = True
            return jsonify({"message": "Control Block 45 Activated"})
        elif action == "control_block_46":
            node.control_block_46 = True
            return jsonify({"message": "Control Block 46 Activated"})
        elif action == "control_block_47":
            node.control_block_47 = True
            return jsonify({"message": "Control Block 47 Activated"})
        elif action == "control_block_48":
            node.control_block_48 = True
            return jsonify({"message": "Control Block 48 Activated"})
        elif action == "control_block_gpt":
            node.control_block_gpt = True
            return jsonify({"message": "Control Block GPT Activated"})
        # ------------------------------------------------------- Unknown Commands -------------------------------------------------------
        else:
            node.get_logger().warning(node.colorize(
                f"Unknown action: {action}", "orange"))
            return jsonify({"message": "Unknown action"}), 400

    @node.app.route('/update_gps_position', methods=['POST'])
    def update_gps_position():
        """Update robot's position."""
        if node.robot_gps_position[0] is not None and node.robot_gps_position[1] is not None:
            gps_position = {
                "lat": node.robot_gps_position[0], "lon": node.robot_gps_position[1]}
            node.get_logger().info(node.colorize(
                f"Position updated to: {gps_position}", "cyan"))
            return jsonify({"lat": node.robot_gps_position[0], "lon": node.robot_gps_position[1], "message": "Position updated successfully"}), 200

        return jsonify({"message": "Invalid position data"}), 400
    
    @node.app.route('/save_waypoints', methods=['POST'])
    def save_waypoints():
        data = request.json
        with open(node.waypoints_dir, 'w') as f:
            json.dump(data, f)
        return 'OK', 200
    
    @node.app.route('/save_no_go_zones', methods=['POST'])
    def save_no_go_zones():
        data = request.json
        with open(node.nogozones_dir, 'w') as f:
            json.dump(data, f)
        return 'OK', 200

    @node.app.route('/load_no_go_zones', methods=['GET'])
    def load_no_go_zones():
        try:
            with open(node.nogozones_dir, 'r') as f:
                data = json.load(f)
            return jsonify(data)
        except Exception:
            return jsonify([]), 200
        
    @node.app.route('/load_waypoints', methods=['GET'])
    def load_waypoints():
        try:
            with open(node.waypoints_dir, 'r') as f:
                data = json.load(f)
            return jsonify(data)
        except Exception:
            return jsonify([]), 200

    @node.app.route('/terminal_submission', methods=['POST'])
    def terminal_submission():
        data = request.json
        commandTerminal = data.get('terminal_input')
        node.get_logger().info(node.colorize(
            f"Received Command: {commandTerminal}", "green"))
        try:
            process = subprocess.Popen(
                commandTerminal,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            stdout, stderr = process.communicate(timeout=10)
            if stdout:
                print(f"Terminal Output:\n{stdout}")
            if stderr:
                print(f"Terminal Error:\n{stderr}")
            rp = f"\n{stdout}"
            return jsonify({"message": rp})
        except subprocess.TimeoutExpired:
            error_message = f"Command '{commandTerminal}' timed out."
            node.get_logger().error(node.colorize(error_message, "red"))
            rp = f"\n{error_message}"
            return jsonify({"message": rp})
        except FileNotFoundError:
            error_message = f"Command '{commandTerminal}' not found."
            node.get_logger().error(node.colorize(error_message, "red"))
            rp = f"\n{error_message}"
            return jsonify({"message": rp})
        except Exception as e:
            error_message = f"Error executing command '{commandTerminal}': {e}"
            node.get_logger().error(node.colorize(error_message, "red"))
            rp = f"\n{error_message}"
            return jsonify({"message": rp})

    @node.app.route('/status')
    def status():
        statuses = []
        for srv in node.param_robot_services:

            try:
                # Check the service status using systemctl
                result = subprocess.run(
                    ['systemctl', 'is-active', srv], capture_output=True, text=True)

                # If the service is active, the result will be "active"
                if result.stdout.strip() == 'active':
                    chk_status = "Active"
                else:
                    chk_status = "Inactive"

            except subprocess.CalledProcessError as e:
                # In case of an error running the subprocess, set as error
                chk_status = "Error"
                node.get_logger().error(
                    f"Error checking service {srv}: {e}")

            # Append the service name and its status
            statuses.append(f"{srv} : {chk_status}")

        return jsonify(statuses=statuses)

    @node.app.route('/reactivate_services', methods=['POST'])
    def reactivate_services():
        try:
            services_to_activate = node.param_robot_services
            for service in services_to_activate:
                if service == node.param_robot_webserver:
                    continue
                command = f"echo {node.param_robot_password} | sudo -S service {service} restart"
                try:
                    subprocess.run(command, check=True, shell=True)
                except:
                    node.get_logger().error(
                        f"Error reactivating service: {service}")

            return jsonify({"message": "Services reactivated successfully."}), 200
        except subprocess.CalledProcessError as e:
            node.get_logger().error(f"Error reactivating services: {e}")
            return jsonify({"message": "Error reactivating services.", "error": str(e)}), 500
        
    @node.app.route('/webserver_connection_status')
    def webserver_connection_status():
        return jsonify(status="ok"), 200, {
            'Cache-Control': 'no-store, no-cache, must-revalidate, max-age=0',
            'Pragma': 'no-cache',
            'Expires': '0'
        }
        
    @node.app.route('/deactivate_services', methods=['POST'])
    def deactivate_services():
        try:
            services_to_deactivate = node.param_robot_services
            for service in services_to_deactivate:
                if service == node.param_robot_webserver:
                    continue
                command = f"echo {node.param_robot_password} | sudo -S service {service} stop"
                try:
                    subprocess.run(command, check=True, shell=True)
                except:
                    node.get_logger().error(
                        f"Error deactivating service: {service}")

            return jsonify({"message": "Services deactivated successfully."}), 200
        except subprocess.CalledProcessError as e:
            node.get_logger().error(f"Error deactivating services: {e}")
            return jsonify({"message": "Error deactivating services.", "error": str(e)}), 500

    @node.app.route('/reset_component', methods=['POST'])
    def reset_component():
        try:
            data = request.json
            component_name = data.get('component')
            command = f"echo {node.param_robot_password} | sudo -S service {component_name} restart"
            subprocess.run(command, check=True, shell=True)
            node.get_logger().info(f"Resetting component: {component_name}")
            return jsonify({"message": f"Component {component_name} reset successfully"})
        except subprocess.CalledProcessError as e:
            node.get_logger().error(f"Error resetting component: {e}")
            return jsonify({"message": "Error resetting component service.", "error": str(e)}), 500

    @node.app.route('/deactivate_component', methods=['POST'])
    def deactivate_component():
        try:
            data = request.json
            component_name = data.get('component')
            command = f"echo {node.param_robot_password} | sudo -S service {component_name} stop"
            subprocess.run(command, check=True, shell=True)
            node.get_logger().info(f"Stopping component: {component_name}")
            return jsonify({"message": f"Component {component_name} stopped successfully"})
        except subprocess.CalledProcessError as e:
            node.get_logger().error(f"Error stopping component: {e}")
            return jsonify({"message": "Error stopping component service.", "error": str(e)}), 500

    @node.app.route('/battery_status')
    def battery_status():
        try:
            percentage = node.battery_percentage
            voltage = node.battery_voltage
            status = node.charging_status

            if percentage is None or math.isnan(percentage):
                percentage = 0
            if voltage is None or math.isnan(voltage):
                voltage = 0.0

            return jsonify({
                'percentage': percentage,
                'voltage': voltage,
                'status': status
            })

        except Exception as e:
            node.get_logger().error(f"Error generating battery status: {e}")
            return jsonify({
                'percentage': 0,
                'voltage': 0.0,
                'status': 'Error'
            }), 500

    @node.app.route('/system_info')
    def system_info():
        full_info = node.get_system_specs()
        return jsonify({'system_info': full_info})

    @node.app.route('/mission')
    def mission():
        missions_ = []
        for mis_ in node.missions_:
            missions_.append(f"{mis_[0]} : {mis_[1]}")
        return jsonify(missions=missions_)

    @node.app.route('/get_map', methods=['POST'])
    def get_map():
        """Serve an enhanced map as an image with correctly placed origin marker."""
        if hasattr(node, 'map_data_raw') and node.map_data_raw is not None:
            width = node.map_data_raw.info.width
            height = node.map_data_raw.info.height
            resolution = node.map_data_raw.info.resolution
            origin_x = node.map_data_raw.info.origin.position.x
            origin_y = node.map_data_raw.info.origin.position.y
            data = np.array(node.map_data_raw.data,
                            dtype=np.int8).reshape((height, width))

            # Convert OccupancyGrid data to an image (grayscale)
            map_img = cv2.normalize(
                data, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

            # Invert the grayscale image
            map_img = cv2.bitwise_not(map_img)

            # Convert map_img to 3 channels if it is single-channel (grayscale)
            if len(map_img.shape) == 2:
                map_img = cv2.cvtColor(map_img, cv2.COLOR_GRAY2RGB)

            # Create a grid image (same size as the map) and draw the grid on it
            num_tiles = 6
            tile_width = width // num_tiles
            tile_height = height // num_tiles
            grid_color = (220, 220, 220)  # Black grid lines for contrast

            # Draw vertical grid lines on the grid image
            for x in range(tile_width, map_img.shape[1], tile_width):
                cv2.line(map_img, (x, 0), (x, map_img.shape[0]), grid_color, 1)

            # Draw horizontal grid lines on the grid image
            for y in range(tile_height, map_img.shape[0], tile_height):
                cv2.line(map_img, (0, y), (map_img.shape[1], y), grid_color, 1)

            # Add white padding around the map
            padding_size = 20
            map_img = cv2.copyMakeBorder(
                map_img, padding_size, padding_size, padding_size, padding_size,
                cv2.BORDER_CONSTANT, value=[255, 255, 255]  # White padding
            )

            # Calculate the origin position in pixels
            origin_px = int(abs(origin_x / resolution)) + padding_size
            origin_py = int(
                height - abs(origin_y / resolution)) + padding_size

            # Flip Image
            map_img = cv2.flip(map_img, 0)

            # Draw a stylish crosshair at the origin
            axis_length = 12  # Length of the cross arms
            thickness = 1     # Thickness of the cross lines

            # Draw the X-axis (red, to the right)
            cv2.line(map_img, (origin_px, origin_py), (origin_px +
                                                       axis_length, origin_py), (0, 0, 255), thickness)

            # Draw the Y-axis (green, upwards)
            cv2.line(map_img, (origin_px, origin_py), (origin_px,
                                                       origin_py - axis_length), (0, 255, 0), thickness)

            # Add "Origin" text below the marker
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.25
            text_thickness = 1
            text_size = cv2.getTextSize(
                "Origin", font, font_scale, text_thickness)[0]
            text_x = origin_px - 5  # Center text horizontally
            # Place text slightly below
            text_y = origin_py + text_size[1] + 5

            # Draw the text
            cv2.putText(map_img, "Origin", (text_x, text_y), font,
                        font_scale, (52, 167, 4), text_thickness, cv2.LINE_AA)

            # Robot Overlay
            try:
                # Load the robot image
                file_path = os.path.join(get_package_share_directory('go2_webserver'),
                                         'go2_webserver',
                                         'static/media/robots/robot.png')
                # Load with transparency (if available)
                robot_img = cv2.imread(file_path, cv2.IMREAD_UNCHANGED)

                # Get the transform from map to base_link
                transform = node.tf_buffer.lookup_transform(
                    "map",
                    node.param_robot_base_link,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0))

                robot_x = transform.transform.translation.x
                robot_y = transform.transform.translation.y

                # Convert the robot position to pixel coordinates
                robot_px = int((robot_x - origin_x) /
                               resolution) + padding_size - 10
                robot_py = int(height - (robot_y - origin_y) /
                               resolution) + padding_size - 10

                # Debug logs
                node.get_logger().info(f"Resolution: {resolution}")
                node.get_logger().info(
                    f"Robot X in pixels (before padding): {(robot_x - origin_x) / resolution}")
                node.get_logger().info(
                    f"Robot Y in pixels (before padding): {(height - (robot_y - origin_y) / resolution)}")

                node.get_logger().info(
                    f"Origin position: x={origin_x}, y={origin_y}")
                node.get_logger().info(
                    f"Robot position: x={robot_x}, y={robot_y}")
                node.get_logger().info(
                    f"Origin pixel coordinates: px={origin_px}, py={origin_py}")
                node.get_logger().info(
                    f"Robot pixel coordinates: px={robot_px}, py={robot_py}")

                # Resize and overlay the robot image
                robot_img_resized = cv2.resize(
                    robot_img, (20, 20), interpolation=cv2.INTER_NEAREST)

                # If the image has an alpha channel
                if robot_img_resized.shape[2] == 4:
                    alpha_channel = robot_img_resized[:, :, 3]
                    rgb_channels = robot_img_resized[:, :, :3]

                    for c in range(3):
                        map_img[robot_py:robot_py + robot_img_resized.shape[0],
                                robot_px:robot_px + robot_img_resized.shape[1], c] = \
                            map_img[robot_py:robot_py + robot_img_resized.shape[0],
                                    robot_px:robot_px + robot_img_resized.shape[1], c] * (1 - alpha_channel / 255.0) + \
                            rgb_channels[:, :, c] * (alpha_channel / 255.0)
                else:
                    map_img[robot_py:robot_py + robot_img_resized.shape[0],
                            robot_px:robot_px + robot_img_resized.shape[1]] = robot_img_resized
            except TransformException as e:
                node.get_logger().error(f"Failed Robot Overlay: {e}")

            # Encode the image as PNG
            _, buffer = cv2.imencode('.png', map_img)
            return buffer.tobytes(), 200, {'Content-Type': 'image/png'}

        return "Map data not available", 404
