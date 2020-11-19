#!/usr/bin/env python

"""
    A ROS Node for the Arduino microcontroller

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.executors import SingleThreadedExecutor
from ros_arduino_python.arduino_driver import Arduino
from ros_arduino_python.arduino_sensors import *
from ros_arduino_msgs.srv import *
from ros_arduino_python.base_controller import BaseController
from geometry_msgs.msg import Twist
import os, time
from serial.serialutil import SerialException


class ArduinoROS(Node):
    def __init__(self):
        rclpy.init()
        self.name = 'arduino'
        super().__init__(self.name)
        qos_profile = QoSProfile(depth=5)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('port', '/dev/ttyACM0'),
                ('baud', 115200),
                ('timeout', 0.5),
                ('base_frame', 'base_link'),
                ('motors_reversed', False),
                ('rate', 2), # 10
                ('sensorstate_rate', 2), #10
                ('use_base_controller', True)
                #('sensors', {})  # @TODO: how to pass empty dictionary?
            ]
        )
        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.timeout = self.get_parameter('timeout').value
        self.base_frame = self.get_parameter('base_frame').value
        self.motors_reversed = self.get_parameter('motors_reversed').value
        self.use_base_controller = self.get_parameter('use_base_controller').value

        # Rate at which summary SensorState message is published. Individual sensors publish
        # at their own rates.
        self.sensorstate_rate = int(self.get_parameter('sensorstate_rate').value)
        self.sensorStatePub = self.create_publisher(SensorState, 'sensor_state', qos_profile)
        self.timer_sensors = self.create_timer(1/self.sensorstate_rate, self.publish_sensors_callback)

        # A service to position a PWM servo
        self.create_service(ServoWrite, 'servo_write', self.ServoWriteHandler)

        # A service to read the position of a PWM servo
        self.create_service(ServoRead, 'servo_read', self.ServoReadHandler)

        # A service to turn set the direction of a digital pin (0 = input, 1 = output)
        self.create_service(DigitalSetDirection, 'digital_set_direction', self.DigitalSetDirectionHandler)

        # A service to turn a digital sensor on or off
        self.create_service(DigitalWrite, 'digital_write', self.DigitalWriteHandler)

        # A service to read the value of a digital sensor
        self.create_service(DigitalRead, 'digital_read', self.DigitalReadHandler)

        # A service to set pwm values for the pins
        self.create_service(AnalogWrite, 'analog_write', self.AnalogWriteHandler)

        # A service to read the value of an analog sensor
        self.create_service(AnalogRead, 'analog_read', self.AnalogReadHandler)

        # Initialize the controlller
        self.controller = Arduino(self.port, self.baud, self.timeout, self.motors_reversed)
        self.controller.connect()
        self.get_logger().info("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")

        # Initialize any sensors
        self.mySensors = self.initialize_sensors()
        
        # Runs all callbacks in the main thread
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        if self.use_base_controller:
            self.myBaseController = BaseController(self.controller, self.base_frame, self.name + "_base_controller")
            executor.add_node(self.myBaseController)  
        try:
            executor.spin()
        finally:
            executor.shutdown()
            listener.destroy_node()
            self.destroy_node()
            self.shutdown()


    def initialize_sensors(self):
        mySensors = list()
        sensor_params = {} # @TODO: read values from self.get_parameter('sensors')

        for name, params in sensor_params.items():
            # Set the direction to input if not specified
            try:
                params['direction']
            except:
                params['direction'] = 'input'

            if params['type'] == 'Ping':
                sensor = Ping(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'GP2D12':
                sensor = GP2D12(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'Digital':
                sensor = DigitalSensor(self.controller, name, params['pin'], params['rate'], self.base_frame, direction=params['direction'])
            elif params['type'] == 'Analog':
                sensor = AnalogSensor(self.controller, name, params['pin'], params['rate'], self.base_frame, direction=params['direction'])
            elif params['type'] == 'PololuMotorCurrent':
                sensor = PololuMotorCurrent(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'PhidgetsVoltage':
                sensor = PhidgetsVoltage(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'PhidgetsCurrent':
                sensor = PhidgetsCurrent(self.controller, name, params['pin'], params['rate'], self.base_frame)
#                if params['type'] == "MaxEZ1":
#                    self.sensors[len(self.sensors)]['trigger_pin'] = params['trigger_pin']
#                    self.sensors[len(self.sensors)]['output_pin'] = params['output_pin']
            try:
                mySensors.append(sensor)
                self.get_logger().info(name + " " + str(params) + " published on topic " + rclpy.get_name() + "/sensor/" + name)
            except:
                self.get_logger().error("Sensor type " + str(params['type']) + " not recognized.")
        return mySensors

    # Read the sensors and publish their values
    def publish_sensors_callback(self):
        for sensor in self.mySensors:
            sensor.poll()
        msg = SensorState()
        #msg.header.frame_id = self.base_frame
        #msg.header.stamp = self.get_clock().now()
        for i in range(len(self.mySensors)):
            msg.name.append(self.mySensors[i].name)
            msg.value.append(self.mySensors[i].value)
        self.sensorStatePub.publish(msg)
        self.get_logger().info("Publishing sensor states.")

    # Service callback functions
    def ServoWriteHandler(self, req):
        self.controller.servo_write(req.id, req.value)
        return ServoWriteResponse()

    def ServoReadHandler(self, req):
        pos = self.controller.servo_read(req.id)
        return ServoReadResponse(pos)

    def DigitalSetDirectionHandler(self, req):
        self.controller.pin_mode(req.pin, req.direction)
        return DigitalSetDirectionResponse()

    def DigitalWriteHandler(self, req):
        self.controller.digital_write(req.pin, req.value)
        return DigitalWriteResponse()

    def DigitalReadHandler(self, req):
        value = self.controller.digital_read(req.pin)
        return DigitalReadResponse(value)

    def AnalogWriteHandler(self, req):
        self.controller.analog_write(req.pin, req.value)
        return AnalogWriteResponse()

    def AnalogReadHandler(self, req):
        value = self.controller.analog_read(req.pin)
        return AnalogReadResponse(value)

    def shutdown(self):
        self.get_logger().info("Shutting down Arduino Node...")

        # Stop the robot
        try:
            self.get_logger().info("Stopping the robot...")
            self.stop()
        except:
            pass

        # Close the serial port
        try:
            self.controller.close()
        except:
            pass
        finally:
            self.get_logger().info("Serial port closed.")
            rclpy.shutdown()
            #thread.join()
            os._exit(0)

if __name__ == '__main__':
    try:
        myArduino = ArduinoROS()
    except SerialException:
        self.get_logger().error("Serial exception trying to open port.")
        os._exit(0)
