#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.
#

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
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable


def generate_launch_description():

    h1_control_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '10')

    nsp = os.environ.get('H1_NS', 'h1_unit_238')
    rmp = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/joint_states', 'joint_states'),
        ('/robot_description', 'robot_description'),
        ('/goal_pose', 'goal_pose'),
        ('/clicked_point', 'clicked_point'),
        ('/initialpose', 'initialpose'),
        ('/global_costmap/costmap', 'global_costmap/costmap'),
        ('/global_costmap/costmap_updates', 'global_costmap/costmap_updates'),
        ('/global_costmap/published_footprint', 'global_costmap/published_footprint'),
        ('/goal_pose', 'goal_pose'),
        ('/local_costmap/costmap', 'local_costmap/costmap'),
        ('/local_costmap/costmap_updates', 'local_costmap/costmap_updates'),
        ('/map', 'map'),
        ('/map_updates', 'map_updates'),
        ('/marker', 'marker'),
        ('/local_costmap/published_footprint', 'local_costmap/published_footprint'),
        ('/local_plan', 'local_plan'),
        ('/waypoints', 'waypoints'),
        ('/plan', 'plan'),
    ]
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("h1_viz"), "rviz", "robot.rviz"]
    )

    node_rviz = Node(
        namespace=nsp,
        remappings=rmp,
        name='rviz',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(h1_control_domain_id)
    ld.add_action(node_rviz)

    return ld
