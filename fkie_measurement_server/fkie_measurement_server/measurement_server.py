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

from rclpy.node import Node
from typing import Dict
import threading

from fkie_measurement_msgs.msg import MeasurementArray
from std_msgs.msg import Int32
import tf2_ros

from .subscriptions import SubscriptionManager
from .commands import CommandManager
from .websocket import WebsocketManager


class MeasurementCollectorNode(Node):
    def __init__(self):

        super(MeasurementCollectorNode, self).__init__('measurement_collector_node')

        # unique_serial_id: MeasurementArray with full_history
        self.sensor_histories: Dict[str, MeasurementArray] = {}

        self.tf_buffer = tf2_ros.Buffer()

        self.declare_parameter('topic_pub_measurement_array', 'measurement_array_agg')
        self.declare_parameter('topic_sub_measurement_array', 'measurement_array')
        self.declare_parameter('topic_command_in', 'commandin')
        self.declare_parameter('topic_command_out', 'commandout')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('utm_zone_number', 32)
        self.declare_parameter('utm_zone_letter', 'U')
        self.declare_parameter('topic_fix', "fix")

        self.topic_pub_measurement_array = self.get_parameter('topic_pub_measurement_array').get_parameter_value().string_value
        self.topic_sub_measurement_array = self.get_parameter('topic_sub_measurement_array').get_parameter_value().string_value
        self.topic_command_in = self.get_parameter('topic_command_in').get_parameter_value().string_value
        self.topic_command_out = self.get_parameter('topic_command_out').get_parameter_value().string_value
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.utm_zone_number = self.get_parameter('utm_zone_number').get_parameter_value().integer_value
        self.utm_zone_letter = self.get_parameter('utm_zone_letter').get_parameter_value().string_value
        self.topic_fix = self.get_parameter('topic_fix').get_parameter_value().string_value

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
        self.get_logger().info(' ')

        self.pub_measurement_array = self.create_publisher(MeasurementArray, self.topic_pub_measurement_array, 5)
        self.get_logger().info(f"Advertising to {self.pub_measurement_array.topic_name}")

        self.sub_manager = SubscriptionManager(node=self)
        self.command_manager = CommandManager(node=self)
        self.websocket_manager = WebsocketManager(node=self)

        self.get_logger().info(' ')

        self.sub_client_count = self.create_subscription(Int32, '/client_count', self.callback_client_count, 5)

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        self.get_logger().info(f"New subscription for {topic_name}")
        msg = MeasurementArray()
        msg.full_history = True
        for _id, ma in self.sensor_histories.items():
            msg.measurements.extend(ma.measurements)
            msg.located_measurements.extend(ma.located_measurements)
        self.pub_measurement_array.publish(msg)

    def callback_client_count(self, msg):
        if (msg.data > 0):
            threading.Timer(3., self.peer_subscribe, ('new_client_count', None, None)).start()

    def stop(self):
        self.get_logger().info("Shutting down...")
