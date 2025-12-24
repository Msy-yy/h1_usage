#! /usr/bin/env python3

import rclpy

import socket
import diagnostic_updater
import diagnostic_msgs
from reach_rs.include.driver import RosNMEADriver

from sensor_msgs.msg import NavSatFix, NavSatStatus
    
FixStatus = {-1 : 'No Fix',
              0 : 'Single Fix',
              1 : 'SBAS Fix',
              2 : 'GBAS Fix'}

class ReachRsDriver(object):

    def __init__(self, node):

        self.node = node
        self.node.get_logger().info(f"{self.colorize('Initializing Driver', 'yellow')}")
        self.host = self.node.get_parameter('reach_rs_host_or_ip')
        self.port = self.node.get_parameter('reach_rs_port')
        self.frame_id = self.node.get_parameter('reach_rs_frame_id')
        self.fix_timeout = self.node.get_parameter('fix_timeout')

        self.address = (self.host.value, int(self.port.value))
        self.socket = None
        self.driver = RosNMEADriver(self.node)
        
        self.diagnostics = diagnostic_updater.Updater(self.node)
        self.diagnostics.setHardwareID('Emlid Reach RS')
        self.diagnostics.add('Receiver Status', self.add_diagnotics)
        
        self.connected = False
        self.connection_status = 'Not Connected'
        self.last_fix = None
        self.node.get_logger().info(f"{self.colorize('Driver initialization complete', 'orange')}")
    
    def colorize(self, text, color):
        color_codes = {
            'green': '92',
            'yellow': '93',
            'orange': '38;5;208',  
            'blue': '94',
            'red': '91'
        }
        return f'\033[{color_codes[color]}m{text}\033[0m'

    def __del__(self):
        if self.socket:
            self.socket.close()
        
    def update(self):
        self.diagnostics.update()
    
    def receives_fixes(self):
        if not self.last_fix:
            return False
        
        duration = (self.node.get_clock().now() - self.last_fix.header.stamp)
        return duration.to_sec() < self.fix_timeout.value
        
    def add_diagnotics(self, stat):
        if self.connected and self.receives_fixes():
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Reach RS driver is connected and has a fix')
        elif self.connected:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, 'Reach RS driver is connected but has no fix')
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, 'Reach RS driver has a connection problem')
            
        stat.add('Connected', str(self.connected))
        stat.add('Connection status', self.connection_status)
        
        if self.last_fix:
            stat.add("Fix status", FixStatus[self.last_fix.status.status])
            stat.add("Seconds since last fix", (self.node.get_clock().now() - self.last_fix.header.stamp).to_sec())
        else:
            stat.add("Fix status", FixStatus[NavSatStatus.STATUS_NO_FIX])
            stat.add("Seconds since last fix", '-')
        
    def connect_to_device(self):
        self.node.get_logger().info('Connecting to {0}:{1}...'.format(*self.address))
        
        while rclpy.ok():
            if self.socket:
                self.socket.close()
                self.socket = None

            self.connected = False
            self.connection_status = 'Not Connected'

            self.socket = socket.socket()

            try:
                self.socket.settimeout(5)
                self.socket.connect(self.address)
                self.connected = True
                self.connection_status = 'connected'
                self.node.get_logger().info(f"{self.colorize('Successfully connected to device!', 'green')}")
                return
            except socket.timeout:
                self.connection_status = 'connect timeout'
            except socket.error as msg:
                self.connection_status = 'connect error ({0})'.format(msg)
            
        exit()
        
    def run(self):
        self.connect_to_device()
        
        while rclpy.ok():
            try:
                self.socket.settimeout(float(self.fix_timeout.value))
                data = self.socket.recv(1024)
                
                if data == '':
                    self.node.get_logger().error(f"{self.colorize('Lost connection. Trying to reconnect...', 'red')}")
                    self.connect_to_device()
                else:
                    self.parse_data(data)
                    self.connection_status = 'Receiving NMEA messages'
            except socket.timeout as t:
                self.connection_status = 'No NMEA messages received'
            except socket.error:
                pass
        
    def parse_data(self, data):
        data = data.decode('UTF-8').strip().split()
        
        for sentence in data:
            if 'GGA' in sentence or 'RMC' in sentence:
                if sentence == '$GNGGA,,,,,,,,,,,,,,*48':
                    self.node.get_logger().error(f"{self.colorize('No GPS signal received', 'red')}", throttle_duration_sec=5)
                    break
                try:
                    fix = self.driver.add_sentence(sentence, self.frame_id.value)
                    
                    if fix:
                        self.last_fix = fix
                except ValueError as e:
                    self.node.get_logger().error(f'Value error, likely due to missing fields in the NMEA message. Error was: {e}. Please report this issue at github.com/ros-drivers/nmea_navsat_driver, including a  bag file with the NMEA sentences that caused it.', throttle_duration_sec=1)