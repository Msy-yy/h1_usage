#!/usr/bin/env python3

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
import os
from glob import glob
from setuptools import setup

package_name = 'reach_rs'
submodules = "reach_rs/include"

setup(
    name=package_name,
    version='0.6.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')), 
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),       
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Salman Omar Sohail',
    maintainer_email='salman.sohail@mybotshop.de',
    description='ROS2 Wrapper for ReachRS',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'launch_reach_rs = reach_rs.reach_rs_node:main'
        ],
    },
)
