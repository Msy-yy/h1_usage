#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.
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
import launch
import launch_ros.actions
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    h1_control_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '10')
    
    nsp = os.environ.get('H1_NS', 'h1_unit_001')
    rmp = [('/tf', 'tf'),
           ('/tf_static', 'tf_static'),
           ]
    
    h1_ros_params = PathJoinSubstitution(
        [FindPackageShare('h1_platform'), 'config', 'h1_2_dual_arm.yaml'])

    h1_arm_record = launch_ros.actions.Node(
        namespace=nsp,
        remappings=rmp,
        package='h1_platform',
        executable='h1_2_dual_arm_recorder',
        name='h1_2_dual_arm_recorder',
        output='screen',
        parameters=[h1_ros_params]
    )
    ld = launch.LaunchDescription()

    ld.add_action(h1_control_domain_id)
    ld.add_action(h1_arm_record)
    
    return ld
