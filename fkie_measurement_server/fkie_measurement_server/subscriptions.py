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

from fkie_measurement_msgs.msg import Measurement, MeasurementArray, MeasurementLocated
from sensor_msgs.msg import NavSatFix
from geodesy import utm
from typing import Set
import tf2_ros


class SubscriptionManager:
    """
    Manage subscriptions for the measurement collector
    """
    def __init__(self, node=None):
        """
        Create subscriptions to measurement topics
        """
        self.node = node

        self.node.get_logger().info(' ')
        self.node.get_logger().info('Initializing Subscription Manager...')

        self.sub_m = self.node.create_subscription(Measurement, '/in_measurement', self.callback_measurement, 5)
        self.node.get_logger().info(f" Subscribed to {self.sub_m.topic_name}")

        self.sub_m_located = self.node.create_subscription(
            MeasurementLocated, '/in_measurement_located', self.callback_measurement_located, 5)
        self.node.get_logger().info(f" Subscribed to {self.sub_m_located.topic_name}")

        self.sub_m_array = self.node.create_subscription(
            MeasurementArray, self.node.topic_sub_measurement_array, self.callback_measurement_array, 5)
        self.node.get_logger().info(f" Subscribed to {self.sub_m_array.topic_name}")

        if self.node.topic_fix:
            self.topic_sub_fix = self.node.create_subscription(NavSatFix, self.node.topic_fix, self.cb_fix, 5)
            self.node.get_logger().info(f" Subscribed to {self.topic_sub_fix.topic_name}")

        self.node.get_logger().info('Subscription Manager Initialized.')

    def callback_measurement(self, msg):
        """
        Method called when a measurement is received on /in_measurement 
        """
        if not msg.unique_serial_id:
            self.node.get_logger().error(
                "[callback_measurement] empty [unique_serial_id]."
            )
            return

        if msg.unique_serial_id not in self.node.sensor_histories:
            ma = MeasurementArray()
            ma.full_history = True
            self.node.sensor_histories[msg.unique_serial_id] = ma

        s_history: MeasurementArray = self.node.sensor_histories[msg.unique_serial_id]
        msg_loc = MeasurementLocated()
        msg_loc.measurement = msg

        try:
            trans = self.node.tf_buffer.lookup_transform(
                self.node.global_frame, msg.header.frame_id, self.node.get_clock().now()
            )
            # get current position
            msg_loc.pose.pose.position.x = trans.transform.translation.x
            msg_loc.pose.pose.position.y = trans.transform.translation.y
            msg_loc.pose.pose.position.z = trans.transform.translation.z
            msg_loc.pose.header.frame_id = trans.header.frame_id
            msg_loc.pose.header.stamp = trans.header.stamp
            s_history.located_measurements.append(msg_loc)
            if self.node.utm_zone_number and self.node.utm_zone_letter:
                msg_loc.utm_zone_number = self.node.utm_zone_number
                msg_loc.utm_zone_letter = self.node.utm_zone_letter
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            self.node.get_logger().info(
                "[callback_measurement] Could not find TF2 lookup between frames [{0}] and [{1}]".format(
                    self.global_frame, msg.header.frame_id
                )
            )
            return
        msg_array = MeasurementArray()
        msg_array.full_history = False
        msg_array.located_measurements.append(msg_loc)
        self.node.pub_measurement_array.publish(msg_array)

    def callback_measurement_located(self, msg):
        """
        Method called when a measurement is received on /in_measurement_located 
        """
        if not msg.measurement.unique_serial_id:
            self.node.get_logger().error(
                "[callback_measurement_located] empty [unique_serial_id]."
            )
            return

        if msg.measurement.unique_serial_id not in self.node.sensor_histories:
            ma = MeasurementArray()
            ma.full_history = True
            self.node.sensor_histories[msg.measurement.unique_serial_id] = ma

        s_history: MeasurementArray = self.node.sensor_histories[
            msg.measurement.unique_serial_id
        ]
        s_history.located_measurements.append(msg)

        msg_array = MeasurementArray()
        msg_array.full_history = False
        msg_array.located_measurements.append(msg)
        self.node.pub_measurement_array.publish(msg_array)

    def callback_measurement_array(self, msg):
        """
        Method called when a measurement is received on topic_sub_measurement_array 
        """
        # clear if this message has full history
        if msg.full_history:
            ids: Set[str] = set()
            for item in msg.measurements:
                ids.add(item.unique_serial_id)
            for item in msg.located_measurements:
                ids.add(item.measurement.unique_serial_id)
            for id in ids:
                if id in self.node.sensor_histories:
                    del self.node.sensor_histories[id]
    
        # add to history
        for item in msg.measurements:
            self.callback_measurement(item)
    
        for item in msg.located_measurements:
            self.callback_measurement_located(item)
        self.node.pub_measurement_array.publish(msg)

    def cb_fix(self, msg):
        utm_point = utm.fromLatLong(
            msg.latitude, msg.longitude, msg.altitude)
        if utm_point.valid():
            self.fix_timestamp = msg.header.stamp
            self.utm_position = utm_point
