#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#  Copyright 2025 Fraunhofer FKIE - All Rights Reserved
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import rclpy
from rclpy.node import Node
from typing import Dict
import threading

from fkie_measurement_msgs.msg import MeasurementValue, MeasurementArray, MeasurementLocated
from std_msgs.msg import Int32
import tf2_ros
import time

from .subscriptions import SubscriptionManager
from .commands import CommandManager
from .webserver import WebServerManager
from .websocket import WebsocketManager


class MeasurementCollectorNode(Node):
    """
    A ROS node to collect measurement data from ROS Topics, websockets and POST requests
    """
    def __init__(self):
        """
        Initialize the Node and start its managers
        """

        super(MeasurementCollectorNode, self).__init__('measurement_collector_node')

        # unique_serial_id: MeasurementArray with full_history
        self.sensor_histories: Dict[str, MeasurementArray] = {}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.declare_parameter('topic_pub_measurement_array', 'measurement_array_agg')
        self.declare_parameter('topic_sub_measurement_array', 'measurement_array')
        self.declare_parameter('topic_command_in', 'commandin')
        self.declare_parameter('topic_command_out', 'commandout')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('utm_zone_number', 32)
        self.declare_parameter('utm_zone_letter', 'U')
        self.declare_parameter('topic_fix', "fix")
        self.declare_parameter('websocket_port', 8899)
        self.declare_parameter('webserver_port', 8080)

        self.topic_pub_measurement_array = self.get_parameter('topic_pub_measurement_array').get_parameter_value().string_value
        self.topic_sub_measurement_array = self.get_parameter('topic_sub_measurement_array').get_parameter_value().string_value
        self.topic_command_in = self.get_parameter('topic_command_in').get_parameter_value().string_value
        self.topic_command_out = self.get_parameter('topic_command_out').get_parameter_value().string_value
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.utm_zone_number = self.get_parameter('utm_zone_number').get_parameter_value().integer_value
        self.utm_zone_letter = self.get_parameter('utm_zone_letter').get_parameter_value().string_value
        self.topic_fix = self.get_parameter('topic_fix').get_parameter_value().string_value
        self.websocket_port = self.get_parameter('websocket_port').get_parameter_value().integer_value
        self.webserver_port = self.get_parameter('webserver_port').get_parameter_value().integer_value

        self.get_logger().info(' ')
        self.get_logger().info('Started with parameters:')
        self.get_logger().info(f'  topic_pub_measurement_array: {self.topic_pub_measurement_array}')
        self.get_logger().info(f'  topic_sub_measurement_array: {self.topic_sub_measurement_array}')
        self.get_logger().info(f'  topic_command_in: {self.topic_command_in}')
        self.get_logger().info(f'  topic_command_out: {self.topic_command_out}')
        self.get_logger().info(f'  global_frame: {self.global_frame}')
        self.get_logger().info(f'  utm_zone_number: {self.utm_zone_number}')
        self.get_logger().info(f'  utm_zone_letter: {self.utm_zone_letter}')
        self.get_logger().info(f'  topic_fix: {self.topic_fix}')
        self.get_logger().info(f'  websocket port: {self.websocket_port}')
        self.get_logger().info(f'  webserver port: {self.webserver_port}')
        self.get_logger().info(' ')

        self.pub_measurement_array = self.create_publisher(MeasurementArray, self.topic_pub_measurement_array, 5)
        self.get_logger().info(f"Advertising to {self.pub_measurement_array.topic_name}")

        self.sub_manager = SubscriptionManager(node=self)
        self.command_manager = CommandManager(node=self)
        self.websocket_manager = WebsocketManager(node=self, port=self.websocket_port, method=self.create_message)
        self.webserver_manager = WebServerManager(node=self, port=self.webserver_port, method=self.create_message)

        self.get_logger().info(' ')

        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                            depth=1,
                            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.sub_client_count = self.create_subscription(Int32, '/client_count', self.callback_client_count, qos)

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        """
        Send sensor history to topic_pub_measurement_array
        """
        self.get_logger().info(f"New subscription for {topic_name}")

        msg = MeasurementArray()
        msg.full_history = True
        for _id, ma in self.sensor_histories.items():
            msg.measurements.extend(ma.measurements)
            msg.located_measurements.extend(ma.located_measurements)
        self.pub_measurement_array.publish(msg)

    def callback_client_count(self, msg):
        """
        Called when a new client (subscriber) is found on the topic_pub_measurement_array topic
        """
        if (msg.data > 0):
            threading.Timer(3., self.peer_subscribe, ('new_client_count', None, None)).start()

    def stop(self):
        """
        Shut down the node, threads are stopped instantly as they are daemons
        """
        self.get_logger().info("Shutting down...")

    def create_message(self, data, type):
        """
        Function to extract, parse and publish measurements
        """
        # extract json parameters
        frame_id = data.get('frame_id')  # optional
        stamp_sec = data.get('stamp_sec')  # optional
        stamp_nanosec = data.get('stamp_nanosec')  # optional
        position_x = data.get('position_x')  # optional
        position_y = data.get('position_y')  # optional
        position_z = data.get('position_z')  # optional
        orientation_x = data.get('orientation_x')  # optional
        orientation_y = data.get('orientation_y')  # optional
        orientation_z = data.get('orientation_z')  # optional
        orientation_w = data.get('orientation_w')  # optional
        utm_zone_number = data.get('utm_zone_number')  # optional
        utm_zone_letter = data.get('utm_zone_letter')  # optional
        unique_serial_id = data.get('unique_serial_id')  # required
        manufacturer_device_name = data.get('manufacturer_device_name')  # optional
        device_classification = data.get('device_classification')  # optional
        sensor = data.get('sensor')  # required
        source_type = data.get('source_type')  # optional
        unit = data.get('unit')  # optional
        value = data.get('value')  # required

        if not(unique_serial_id and sensor and value):
            self.get_logger().warn(
                f"[{type}] Required fields missing."
            )
            return
        
        if not manufacturer_device_name:
            manufacturer_device_name = unique_serial_id

        if not device_classification:
            device_classification = ""
        
        if not source_type:
            source_type = ""

        if not unit:
            unit = ""
            
        if unique_serial_id not in self.sensor_histories:
            ma = MeasurementArray()
            ma.full_history = True
            self.sensor_histories[unique_serial_id] = ma

        s_history: MeasurementArray = self.sensor_histories[unique_serial_id]

        # Located Measurement
        msg_loc = MeasurementLocated()
        if position_x and position_y and position_x:
            msg_loc.pose.pose.position.x = float(position_x)
            msg_loc.pose.pose.position.y = float(position_y)
            msg_loc.pose.pose.position.z = float(position_z)
        else:
            msg_loc.pose.pose.position.x = 0.0
            msg_loc.pose.pose.position.y = 0.0
            msg_loc.pose.pose.position.z = 0.0

        if orientation_w and orientation_x and orientation_y and orientation_z:
            msg_loc.pose.pose.orientation.x = float(orientation_x)
            msg_loc.pose.pose.orientation.y = float(orientation_y)
            msg_loc.pose.pose.orientation.z = float(orientation_z)
            msg_loc.pose.pose.orientation.w = float(orientation_w)
        else:
            msg_loc.pose.pose.orientation.x = 0.0
            msg_loc.pose.pose.orientation.y = 0.0
            msg_loc.pose.pose.orientation.z = 0.0
            msg_loc.pose.pose.orientation.w = 1.0

        if stamp_sec and stamp_nanosec:
            msg_loc.pose.header.stamp.sec = int(stamp_sec)
            msg_loc.pose.header.stamp.nanosec = int(stamp_nanosec)
            msg_loc.measurement.header.stamp.sec = int(stamp_sec)
            msg_loc.measurement.header.stamp.nanosec = int(stamp_nanosec)
        else:
            msg_loc.pose.header.stamp.sec = int(time.time())
            msg_loc.pose.header.stamp.nanosec = 0
            msg_loc.measurement.header.stamp.sec = int(time.time())
            msg_loc.measurement.header.stamp.nanosec = 0

        if frame_id:
            msg_loc.pose.header.frame_id = frame_id
            msg_loc.measurement.header.frame_id = frame_id
        else: 
            msg_loc.pose.header.frame_id = ""
            msg_loc.measurement.header.frame_id = ""

        msg_loc.measurement.unique_serial_id = unique_serial_id
        msg_loc.measurement.manufacturer_device_name = manufacturer_device_name
        msg_loc.measurement.device_classification = device_classification

        if self.utm_zone_number and self.utm_zone_letter:
            msg_loc.utm_zone_number = self.utm_zone_number
            msg_loc.utm_zone_letter = self.utm_zone_letter

        s_history.located_measurements.append(msg_loc)

        # Measurement Value
        msg_value = MeasurementValue()

        if stamp_sec and stamp_nanosec:
            msg_value.begin.sec = int(stamp_sec)
            msg_value.begin.nanosec = int(stamp_nanosec)
            msg_value.end.sec = int(stamp_sec)
            msg_value.end.nanosec = int(stamp_nanosec)
        else:
            msg_value.begin.sec = int(time.time())
            msg_value.begin.nanosec = 0
            msg_value.end.sec = int(time.time())
            msg_value.end.nanosec = 0

        msg_value.sensor = sensor
        msg_value.source_type = source_type
        msg_value.unit = unit
        msg_value.value_single = float(value)
        msg_loc.measurement.values.append(msg_value)

        # Final Message
        msg_array = MeasurementArray()
        msg_array.full_history = False
        msg_array.located_measurements.append(msg_loc)

        self.pub_measurement_array.publish(msg_array)
