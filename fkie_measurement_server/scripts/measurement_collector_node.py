#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#  Copyright 2024 Fraunhofer FKIE - All Rights Reserved
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

from typing import Dict
from typing import Set

import rclpy
import sys
from rclpy.node import Node
import tf2_ros
import threading
from sensor_msgs.msg import NavSatFix
from geodesy import utm

from std_msgs.msg import Int32
from fkie_measurement_msgs.msg import Measurement, MeasurementArray, MeasurementLocated, MeasurementValue
from fkie_measurement_server_msgs.msg import CommandIn, CommandOut


class MeasurementCollectorNode():
    def __init__(self, node: Node):
        super(MeasurementCollectorNode, self).__init__()
        self.ros_node = node
        self.ros_node.get_logger().info('Launch parameters:')

        self.ros_node.declare_parameter('param_topic_pub_measurement_array', '')
        self.ros_node.declare_parameter('param_topic_sub_measurement_array', '')
        self.ros_node.declare_parameter('param_topic_command_in', '')
        self.ros_node.declare_parameter('param_topic_command_out', '')
        self.ros_node.declare_parameter('global_frame', '')
        self.ros_node.declare_parameter('utm_zone_number', 32)
        self.ros_node.declare_parameter('utm_zone_letter', 'U')
        self.ros_node.declare_parameter('topic_fix', "fix")

        self.param_topic_pub_measurement_array = self.ros_node.get_parameter_or(
            'topic_pub_measurement_array', 'measurement_array_agg')
        self.ros_node.get_logger().info(f"  topic_pub_measurement_array: {self.param_topic_pub_measurement_array}")

        self.param_topic_sub_measurement_array = self.ros_node.get_parameter_or(
            'topic_sub_measurement_array', 'measurement_array')
        self.ros_node.get_logger().info(f"  topic_sub_measurement_array: {self.param_topic_sub_measurement_array}")

        self.param_topic_command_out = self.ros_node.get_parameter_or(
            'topic_command_out', 'commandout')
        self.ros_node.get_logger().info(f"  param_topic_command_out: {self.param_topic_command_out}")

        self.param_topic_command_in = self.ros_node.get_parameter_or(
            'topic_command_in', 'commandin')
        self.ros_node.get_logger().info(f"  param_topic_command_in: {self.param_topic_command_in}")

        self.global_frame = self.ros_node.get_parameter_or('frame_global', "map")
        self.ros_node.get_logger().info(f"  frame_global: {self.global_frame}")

        self.utm_zone_number = self.ros_node.get_parameter_or('utm_zone_number', "").get_parameter_value().integer_value
        self.ros_node.get_logger().info(f"  utm_zone_number: {self.utm_zone_number}")

        self.utm_zone_letter = self.ros_node.get_parameter_or('utm_zone_letter', "").get_parameter_value().string_value
        self.ros_node.get_logger().info(f"  utm_zone_letter: {self.utm_zone_letter}")

        self.param_topic_fix = self.ros_node.get_parameter(
            'topic_fix').get_parameter_value().string_value
        self.ros_node.get_logger().info(f"  topic_fix: {self.param_topic_fix}")

        # unique_serial_id: MeasurementArray with full_history
        self.sensor_histories: Dict[str, MeasurementArray] = {}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.ros_node)

        # create subscribers for all registered sensors
        if self.param_topic_fix:
            self.topic_sub_fix = self.ros_node.create_subscription(
                NavSatFix, self.param_topic_fix, self.cb_fix, 5)
            self.ros_node.get_logger().info(
                f"subscribed to {self.param_topic_fix}")

        self.pub_measurement_array = self.ros_node.create_publisher(
            MeasurementArray, self.param_topic_pub_measurement_array, 5)
        self.ros_node.get_logger().info(f"advertised to {self.pub_measurement_array.topic_name}")

        self.sub_m = self.ros_node.create_subscription(Measurement, '/in_measurement', self.callback_measurement, 5)
        self.ros_node.get_logger().info(f"subscribed to {self.sub_m.topic_name}")

        self.sub_m_located = self.ros_node.create_subscription(
            MeasurementLocated, '/in_measurement_located', self.callback_measurement_located, 5)
        self.ros_node.get_logger().info(f"subscribed to {self.sub_m_located.topic_name}")

        self.sub_m_array = self.ros_node.create_subscription(
            MeasurementArray, self.param_topic_sub_measurement_array, self.callback_measurement_array, 5)
        self.ros_node.get_logger().info(f"subscribed to {self.sub_m_array.topic_name}")

        self.sub_client_count = self.ros_node.create_subscription(Int32, '/client_count', self.callback_client_count, 5)

        # create subscribers for command manager
        self.commandout = self.ros_node.create_publisher(
            CommandOut, self.param_topic_command_out, 5)
        self.ros_node.get_logger().info(f"advertised to {self.commandout.topic_name}")

        self.commandin = self.ros_node.create_subscription(
            CommandIn, self.param_topic_command_in, self.callback_command, 5)
        self.ros_node.get_logger().info(f"subscribed to {self.commandin.topic_name}")

    def cb_fix(self, msg):
        utm_point = utm.fromLatLong(
            msg.latitude, msg.longitude, msg.altitude)
        if utm_point.valid():
            self.fix_timestamp = msg.header.stamp
            self.utm_position = utm_point

    def callback_command(self, msg):
        self.ros_node.get_logger().info(f"Command called: {msg.command}")
        match msg.command:
            case "history":
                history = MeasurementArray()
                history.full_history = True
                for _id, ma in self.sensor_histories.items():
                    history.measurements.extend(ma.measurements)
                    history.located_measurements.extend(ma.located_measurements)
                self.pub_measurement_array.publish(history)
            case _:
                pass
        

    def callback_measurement(self, msg):
        # type: (Measurement) -> None
        if not msg.unique_serial_id:
            self.ros_node.get_logger().error("[callback_measurement] empty [unique_serial_id].")
            return

        if msg.unique_serial_id not in self.sensor_histories:
            ma = MeasurementArray()
            ma.full_history = True
            self.sensor_histories[msg.unique_serial_id] = ma

        s_history: MeasurementArray = self.sensor_histories[msg.unique_serial_id]
        msg_loc = MeasurementLocated()
        msg_loc.measurement = msg

        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame, msg.header.frame_id, self.ros_node.get_clock().now())
            # get current position
            msg_loc.pose.pose.position.x = trans.transform.translation.x
            msg_loc.pose.pose.position.y = trans.transform.translation.y
            msg_loc.pose.pose.position.z = trans.transform.translation.z
            msg_loc.pose.header.frame_id = trans.header.frame_id
            msg_loc.pose.header.stamp = trans.header.stamp
            s_history.located_measurements.append(msg_loc)
            if self.utm_zone_number and self.utm_zone_letter:
                msg_loc.utm_zone_number = self.utm_zone_number
                msg_loc.utm_zone_letter = self.utm_zone_letter
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.ros_node.get_logger().info("[callback_measurement] Could not find TF2 lookup between frames [{0}] and [{1}]".format(
                self.global_frame, msg.header.frame_id
            ))
            return
        msg_array = MeasurementArray()
        msg_array.full_history = False
        msg_array.located_measurements.append(msg_loc)
        self.pub_measurement_array.publish(msg_array)

    def callback_measurement_located(self, msg):
        # type: (MeasurementLocated) -> None
        if not msg.measurement.unique_serial_id:
            self.ros_node.get_logger().error("[callback_measurement_located] empty [unique_serial_id].")
            return

        if msg.measurement.unique_serial_id not in self.sensor_histories:
            ma = MeasurementArray()
            ma.full_history = True
            self.sensor_histories[msg.measurement.unique_serial_id] = ma

        s_history: MeasurementArray = self.sensor_histories[msg.measurement.unique_serial_id]
        s_history.located_measurements.append(msg)

        msg_array = MeasurementArray()
        msg_array.full_history = False
        msg_array.located_measurements.append(msg)
        self.pub_measurement_array.publish(msg_array)

    def callback_measurement_array(self, msg):
        # clear if this message has full history
        if msg.full_history:
            ids: Set[str] = set()
            for item in msg.measurements:
                ids.add(item.unique_serial_id)
            for item in msg.located_measurements:
                ids.add(item.measurement.unique_serial_id)
            for id in ids:
                if id in self.sensor_histories:
                    del self.sensor_histories[id]

        # add to history
        for item in msg.measurements:
            self.callback_measurement(item)

        for item in msg.located_measurements:
            self.callback_measurement_located(item)
        self.pub_measurement_array.publish(msg)

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        self.ros_node.get_logger().info(f"New subscription for {topic_name}")
        msg = MeasurementArray()
        msg.full_history = True
        for _id, ma in self.sensor_histories.items():
            msg.measurements.extend(ma.measurements)
            msg.located_measurements.extend(ma.located_measurements)
        self.pub_measurement_array.publish(msg)

    def callback_client_count(self, msg):
        if (msg.data > 0):
            threading.Timer(3., self.peer_subscribe, ('new_client_count', None, None)).start()


# Main function
if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('measurement_collector_node')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    mes = MeasurementCollectorNode(node)
    try:
        executor.spin()
    except KeyboardInterrupt as e:
        pass
