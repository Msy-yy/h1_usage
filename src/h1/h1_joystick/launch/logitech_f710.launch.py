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
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    go2_control_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '10')
    
    nsp = os.environ.get('H1_NS', 'h1_unit_001')
    rmp = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]
    
    filepath_config_teleop = PathJoinSubstitution(
        [FindPackageShare('h1_control'), 'config', ('teleop.yaml')]
    )
    
    
    node_joy = Node(
        namespace=nsp,
        remappings=rmp,
        name='joy_node',
        package='joy_linux',
        executable='joy_linux_node',
        output='screen',
        parameters=[filepath_config_teleop]
    )
    
    # node_joy = Node(
    #     namespace=nsp,
    #     package='joy',
    #     executable='joy_node',
    #     output='screen',
    #     name='joy_node',
    #     parameters=[filepath_config_teleop]
    # )

    node_teleop_twist_joy = Node(
        namespace=nsp,
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        name='teleop_twist_joy_node',
        remappings={('cmd_vel', 'joy_teleop/cmd_vel')},
        parameters=[filepath_config_teleop]
    )
    
    node_teleop_twist_joy_custom_controls = Node(
        namespace=nsp,
        remappings=rmp,
        name='control_logitech_f710',
        package='h1_joystick',
        executable='logitech_f710',
        output='screen',
    )

    ld = LaunchDescription()
    
    ld.add_action(go2_control_domain_id)
    
    ld.add_action(node_joy)
    ld.add_action(node_teleop_twist_joy)
    ld.add_action(node_teleop_twist_joy_custom_controls)

    return ld

