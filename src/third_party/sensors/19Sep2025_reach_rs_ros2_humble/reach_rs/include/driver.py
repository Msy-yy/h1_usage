# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# Copyright (c) 2018, Enway GmbH
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math

import rclpy

from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped

from reach_rs.include.checksum_utils import check_nmea_checksum
import reach_rs.include.parser


class RosNMEADriver(object):

    def __init__(self, node):

        self.node = node

        self.fix_pub = self.node.create_publisher(NavSatFix, 'emlid/fix', 1)
        self.vel_pub = self.node.create_publisher(TwistStamped, 'emlid/vel', 1)
        self.time_ref_pub = self.node.create_publisher(TimeReference, 'emlid/time_reference', 1)

        self.time_ref_source = self.node.get_parameter('time_ref_source') 
        self.use_RMC = self.node.get_parameter('useRMC') 
        self.covariance_matrix = self.node.get_parameter('covariance_matrix') 

    # Returns the fix if we successfully did something with the passed in
    # nmea_string
    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        if not check_nmea_checksum(nmea_string):
            self.node.get_logger().error(f'Received a sentence with an invalid checksum. Sentence was: {repr(nmea_string)}')
            return None

        parsed_sentence = reach_rs.include.parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            self.node.get_logger().error(f'Failed to parse NMEA sentence. Sentence was: {nmea_string}')
            return None

        if timestamp:
            current_time = timestamp
        else:
            current_time = self.node.get_clock().now().to_msg()
        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id
        current_time_ref = TimeReference()
        current_time_ref.header.stamp = current_time
        current_time_ref.header.frame_id = frame_id
        if self.time_ref_source.value:
            current_time_ref.source = self.time_ref_source.value
        else:
            current_time_ref.source = frame_id

        if not self.use_RMC.value and 'GGA' in parsed_sentence:
            data = parsed_sentence['GGA']
            gps_qual = data['fix_type']
            if gps_qual == 0:
                current_fix.status.status = NavSatStatus.STATUS_NO_FIX
            elif gps_qual == 1:
                current_fix.status.status = NavSatStatus.STATUS_FIX
            elif gps_qual == 2:
                current_fix.status.status = NavSatStatus.STATUS_SBAS_FIX
            elif gps_qual in (4, 5):
                current_fix.status.status = NavSatStatus.STATUS_GBAS_FIX
            else:
                current_fix.status.status = NavSatStatus.STATUS_NO_FIX

            current_fix.status.service = NavSatStatus.SERVICE_GPS

            current_fix.header.stamp = current_time

            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude

            if self.covariance_matrix.value and isinstance(self.covariance_matrix.value, list) and len(self.covariance_matrix.value) == 9:
                for i in range(9):
                    current_fix.position_covariance[i] = self.covariance_matrix.value[i]
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_KNOWN
            else:
                hdop = data['hdop']
                current_fix.position_covariance[0] = hdop ** 2
                current_fix.position_covariance[4] = hdop ** 2
                current_fix.position_covariance[8] = (2 * hdop) ** 2  # FIXME
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude

            self.fix_pub.publish(current_fix)

            if not math.isnan(data['utc_time']):
                current_time_ref.time_ref = rclpy.time.Time(seconds=data['utc_time']).to_msg()
                self.last_valid_fix_time = current_time_ref
                self.time_ref_pub.publish(current_time_ref)

                
            return current_fix

        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']

            # Only publish a fix from RMC if the use_RMC flag is set.
            if self.use_RMC.value:
                if data['fix_valid']:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                current_fix.status.service = NavSatStatus.SERVICE_GPS

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                current_fix.altitude = float('NaN')
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.fix_pub.publish(current_fix)

                if not math.isnan(data['utc_time']):
                    current_time_ref.time_ref = rclpy.time.Time(seconds=data['utc_time']).to_msg()
                    self.last_valid_fix_time = current_time_ref
                    self.time_ref_pub.publish(current_time_ref)

            # Publish velocity from RMC regardless, since GGA doesn't provide it.
            if data['fix_valid']:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * \
                    math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * \
                    math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)
                
            return current_fix
        else:
            return None

    """Helper method for getting the frame_id with the correct TF prefix"""
    def get_frame_id(self):
        frame_id = self.declare_parameter('frame_id', 'gps').value
        prefix = self.declare_parameter('tf_prefix', '').value
        if len(prefix):
            return '%s/%s' % (prefix, frame_id)
        return frame_id