#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2024, MYBOTSHOP GmbH, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of MYBOTSHOP GmbH.
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

import rclpy
from rclpy.node import Node

from .include.reach_rs_driver import ReachRsDriver

class ROS2Node(Node):

    def __init__(self):
        # Inherit ROS2 Node
        super().__init__("reach_rs")

        # Initialize ROS2 Params
        self.declare_parameters(
            namespace='',
            parameters=[
                ('reach_rs_host_or_ip',   rclpy.Parameter.Type.STRING),
                ('reach_rs_port',         rclpy.Parameter.Type.STRING),
                ('reach_rs_frame_id',     rclpy.Parameter.Type.STRING),
                ('fix_timeout',           rclpy.Parameter.Type.STRING),
                ('time_ref_source',       rclpy.Parameter.Type.STRING),
                ('useRMC',                rclpy.Parameter.Type.BOOL),
                ('covariance_matrix',     rclpy.Parameter.Type.STRING),
            ])

def main(args=None):

    # Intialize Node
    rclpy.init(args=args)
    node = ROS2Node() 

    # Auxiliary Code
    node.get_logger().info("Launching Emlid Reach RS2")
    
    reachRsDriver = ReachRsDriver(node)
    reachRsDriver.run()

    # Keep Node Alive 
    rclpy.spin(node)

    # Shutdown Node 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()