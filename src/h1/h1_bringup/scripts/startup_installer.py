#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.

import os
import robot_upstart
from ament_index_python.packages import get_package_share_directory


def install_job(job_name, package_name, launch_filename, domain_id=10,
                e_user=None, rmw_type='rmw_fastrtps_cpp', disable_srvs=False):

    # 1. Uninstall existing service
    print(color_string(33, "Uninstalling existing service: {}".format(job_name)))
    os.system("sudo service {} stop".format(job_name))
    uninstall_job = robot_upstart.Job(
        name=job_name, rosdistro=os.environ['ROS_DISTRO'])
    uninstall_job.uninstall()

    # 2. Configure new service
    print(color_string(32, "Installing new service: {}".format(job_name)))
    linux_service = robot_upstart.Job(name=job_name,
                                      user=e_user,
                                      ros_domain_id=domain_id,
                                      rmw=rmw_type,
                                      workspace_setup=os.path.join(get_package_share_directory(
                                          'h1_bringup'), 'config/setup.bash')
                                      )

    linux_service.add(package=package_name, filename=launch_filename)
    linux_service.install()

    # 3. Set service state
    if disable_srvs:
        os.system("sudo systemctl disable {}".format(job_name))
        return

    # 4. Refresh for activation
    os.system(
        "sudo systemctl daemon-reload && sudo systemctl start {}".format(job_name))


def color_string(color, string):
    return "\033[{}m{}\033[0m".format(color, string)


if __name__ == "__main__":

    jobs = [

        # -------------------------------------------------------- Core
        {"name": "h1-webserver",
         "package": "h1_webserver",
         "launch_filename": "launch/webserver.launch.py",
         "rmw_type": "rmw_cyclonedds_cpp",
         },

        {"name": "h1-highlevel-ros-controller",
         "package": "h1_platform",
         "launch_filename": "launch/highlevel_ros.launch.py",
         "rmw_type": "rmw_cyclonedds_cpp",
         "disable": True
         },

        {"name": "h1-domain-bridge",
         "package": "h1_platform",
         "launch_filename": "launch/domain_bridge.launch.py",
         "disable": True
         },

        {"name": "h1-twist-mux",
         "package": "h1_control",
         "launch_filename": "launch/twistmux.launch.py",
         "disable": True
         },

        # -------------------------------------------------------- Switchable

        # h1_2_description.launch.py | h1_description.launch.py
        {"name": "h1-description",
         "package": "h1_description",
         "launch_filename": "launch/h1_2_description.launch.py",
         "disable": True
         },

        # h1_2_state_publisher.launch.py | h1_state_publisher.launch.py
        {"name": "h1-state-publisher",
         "package": "h1_platform",
         "launch_filename": "launch/h1_2_state_publisher.launch.py",
         "disable": True
         },

        # {"name": "h1-arm-record",
        #  "package": "h1_platform",
        #  "launch_filename": "launch/h1_2_arm_record.launch.py",
        #  "rmw_type": "rmw_cyclonedds_cpp",
        #  "disable": True
        #  },

        # -------------------------------------------------------- Sensors
        {"name": "h1-d435i",
         "package": "h1_depth_camera",
         "launch_filename": "launch/realsense.launch.py",
         "disable": True,
         },

        {"name": "h1-emlid",
         "package": "h1_gps",
         "launch_filename": "launch/emlid_m2.launch.py",
         "disable": True
         },

        {"name": "h1-ouster",
         "package": "h1_lidar",
         "launch_filename": "launch/ouster.launch.py",
         "disable": True
         },
        
        # {"name": "h1-logitechf710",
        #  "package": "h1_joystick",
        #  "launch_filename": "launch/logitech_f710.launch.py",
        #  "disable": True
        #  },
        
        # {"name": "h1-zed2i",
        #  "package": "h1_depth_camera",
        #  "launch_filename": "launch/zed.launch.py",
        #  "user": "root",
        #  "disable": False
        #  },

        # {"name": "h1-livox-mid360",
        #  "package": "h1_lidar",
        #  "launch_filename": "launch/livox_mid360.launch.py",
        #  "disable": True
        #  },

        # -------------------------------------------------------- Tools
        {"name": "h1-kiss-odom",
         "package": "h1_lidar",
         "launch_filename": "launch/kiss_lio.launch.py",
         "disable": True
         },
    ]

    for job in jobs:
        install_job(job_name=job["name"],
                    package_name=job["package"],
                    e_user=job.get("user", None),
                    launch_filename=job["launch_filename"],
                    disable_srvs=job.get("disable", False),
                    rmw_type=job.get("rmw_type", "rmw_fastrtps_cpp"),
                    )
