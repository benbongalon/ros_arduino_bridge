#!/usr/bin/env python

"""
    A base controller class for the Arduino microcontroller
    
    Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from ros_arduino_python.arduino_driver import Arduino

import os
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped


""" Class to receive Twist commands and publish Odometry data """
class BaseController(Node):
    def __init__(self, arduino: Arduino, base_frame: str, name='base_controller'):
        super().__init__('controller')
        self.get_logger().info("Calling BaseController..")
        self.arduino = arduino
        self.base_frame = base_frame
        self.name = name

        self.declare_parameters(
            namespace='',
            parameters=[
                ('base_controller_rate', 5), # 5 Hz
                ('base_controller_timeout', 1.0),
                ('wheel_diameter', 0.1524), # 6 inches -- @TODO: set to '' and feed by YAML
                ('wheel_track', 0.4), # ''
                ('encoder_resolution', 144), # ticks_per_revolution
                ('gear_reduction', 1.0),
                ('Kp', 20),
                ('Kd', 12),
                ('Ki', 0),
                ('Ko', 20), # -- @TODO: default to 50
                ('accel_limit', 0.1),
                ('motors_reversed', False)
            ]
        )
        self.rate = float(self.get_parameter('base_controller_rate').value)
        self.timer = self.create_timer(1/self.rate, self.poll_base_controller_callback)
        
        self.timeout = self.get_parameter('base_controller_timeout').value
        self.stopped = False

        pid_params = dict()
        pid_params['wheel_diameter'] = self.get_parameter('wheel_diameter').value 
        pid_params['wheel_track'] = self.get_parameter('wheel_track').value
        pid_params['encoder_resolution'] = self.get_parameter('encoder_resolution').value 
        pid_params['gear_reduction'] = self.get_parameter('gear_reduction').value
        pid_params['Kp'] = self.get_parameter('Kp').value
        pid_params['Kd'] = self.get_parameter('Kd').value
        pid_params['Ki'] = self.get_parameter('Ki').value
        pid_params['Ko'] = self.get_parameter('Ko').value
        
        self.accel_limit = self.get_parameter('accel_limit').value
        self.motors_reversed = self.get_parameter('motors_reversed').value
        
        # Set up PID parameters and check for missing values
        self.setup_pid(pid_params)
            
        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * pi)
        
        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate
                
        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0

        # Internal data        
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        self.v_left = 0
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.last_cmd_vel = self.get_clock().now()

        # Subscriptions
        qos_profile = QoSProfile(depth=5)
        self.create_subscription(Twist, 'cmd_vel', self.cmdVelCallback, qos_profile=qos_profile)
        
        # Clear any old odometry info
        self.arduino.reset_encoders()
        
        # Set up the odometry broadcaster
        try:
            self.odomPub = self.create_publisher(Odometry, 'odom', qos_profile=qos_profile)
        except AttributeError as ex:
            self.get_logger().error("Could not create Odometry publisher.. try again")
            self.odomPub = self.create_publisher(Odometry, 'odom', qos_profile=qos_profile)
        
        try:
            self.odomBroadcaster = TransformBroadcaster(self, qos=qos_profile)
        except AttributeError as ex:
            self.get_logger().error("Could not create TransformBroadcaster.. try again")
            self.odomBroadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.get_logger().info("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        self.get_logger().info("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")

    def setup_pid(self, pid_params):
        # Check to see if any PID parameters are missing
        missing_params = False
        for param in pid_params:
            if pid_params[param] == "":
                self.get_logger().error("*** PID Parameter " + param + " is missing. ***")
                missing_params = True
        
        if missing_params:
            os._exit(1)
                
        self.wheel_diameter = pid_params['wheel_diameter']
        self.wheel_track = pid_params['wheel_track']
        self.encoder_resolution = pid_params['encoder_resolution']
        self.gear_reduction = pid_params['gear_reduction']
        
        self.Kp = pid_params['Kp']
        self.Kd = pid_params['Kd']
        self.Ki = pid_params['Ki']
        self.Ko = pid_params['Ko']
        
        self.arduino.update_pid(self.Kp, self.Kd, self.Ki, self.Ko)

    def poll_base_controller_callback(self):
        now = self.get_clock().now()
        # Read the encoders
        try:
            left_enc, right_enc = self.arduino.get_encoder_counts()
        except:
            self.bad_encoder_count += 1
            self.get_logger().error("Encoder exception count: " + str(self.bad_encoder_count))
            return
                        
        dt = 1 / self.rate # dt in seconds
        
        # Calculate odometry
        if self.enc_left == None:
            dright = 0
            dleft = 0
        else:
            dright = (right_enc - self.enc_right) / self.ticks_per_meter
            dleft = (left_enc - self.enc_left) / self.ticks_per_meter

        self.enc_right = right_enc
        self.enc_left = left_enc
        
        dxy_ave = (dright + dleft) / 2.0
        dth = (dright - dleft) / self.wheel_track
        vxy = dxy_ave / dt
        vth = dth / dt
            
        if (dxy_ave != 0):
            dx = cos(dth) * dxy_ave
            dy = -sin(dth) * dxy_ave
            self.x += (cos(self.th) * dx - sin(self.th) * dy)
            self.y += (sin(self.th) * dx + cos(self.th) * dy)

        if (dth != 0):
            self.th += dth 

        quaternion = Quaternion()
        quaternion.x = 0.0 
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2.0)
        quaternion.w = cos(self.th / 2.0)

        self.get_logger().info("x={:.2f} y={:.2f}, Q={}".format(self.x, self.y, quaternion))
        # Translate it
        odom_trans = TransformStamped()
        odom_trans.header.stamp = now.to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = self.base_frame
        odom_trans.transform.translation.x = float(self.x)
        odom_trans.transform.translation.y = float(self.y)
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = quaternion

        # Create the odometry transform frame broadcaster.
        self.odomBroadcaster.sendTransform(odom_trans)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = float(vxy)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = float(vth)

        self.odomPub.publish(odom)
        
        if now > (self.last_cmd_vel + Duration(seconds=self.timeout)):
            self.v_des_left = 0
            self.v_des_right = 0
            
        if self.v_left < self.v_des_left:
            self.v_left += self.max_accel
            if self.v_left > self.v_des_left:
                self.v_left = self.v_des_left
        else:
            self.v_left -= self.max_accel
            if self.v_left < self.v_des_left:
                self.v_left = self.v_des_left
        
        if self.v_right < self.v_des_right:
            self.v_right += self.max_accel
            if self.v_right > self.v_des_right:
                self.v_right = self.v_des_right
        else:
            self.v_right -= self.max_accel
            if self.v_right < self.v_des_right:
                self.v_right = self.v_des_right
        
        # Set motor speeds in encoder ticks per PID loop
        if not self.stopped:
            self.arduino.drive(self.v_left, self.v_right)
            
    def stop(self):
        self.stopped = True
        self.arduino.drive(0, 0)
            
    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = self.get_clock().now()
        
        x = req.linear.x         # m/s
        th = req.angular.z       # rad/s
        self.get_logger().info("cmdVelCallback(): cmd_vel x:{} th:{}".format(x,th ))

        if x == 0:
            # Turn in place
            right = th * self.wheel_track  * self.gear_reduction / 2.0
            left = -right
        elif th == 0:
            # Pure forward/backward motion
            left = right = x
        else:
            # Rotation about a point in space
            left = x - th * self.wheel_track  * self.gear_reduction / 2.0
            right = x + th * self.wheel_track  * self.gear_reduction / 2.0
            
        self.v_des_left = int(left * self.ticks_per_meter / self.arduino.PID_RATE)
        self.v_des_right = int(right * self.ticks_per_meter / self.arduino.PID_RATE)
        self.get_logger().info("  left:{:.2f} v_des_left:{:.2f}, right:{:.2f}, v_des_right:{:.2f}".format(left, self.v_des_left, right, self.v_des_right))

if __name__ == '__main__':
    myController = None
    try:
        myArduino = Arduino()
        myController = BaseController(myArduino, 'test_base_frame', 'test_base_controller')
    except Exception as ex:
        print("Error:" + ex)
        os._exit(0)