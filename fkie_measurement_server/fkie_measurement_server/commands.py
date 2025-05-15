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

from fkie_measurement_server_msgs.msg import CommandIn, CommandOut
from fkie_measurement_msgs.msg import MeasurementArray


class CommandManager:
    def __init__(self, node=None):
        self.node = node

        self.node.get_logger().info(" ")
        self.node.get_logger().info("Initializing Command Manager...")

        self.commandin = self.node.create_subscription(
            CommandIn, self.node.topic_command_in, self.callback_command, 5
        )
        self.node.get_logger().info(f" Subscribed to {self.commandin.topic_name}")

        self.commandout = self.node.create_publisher(
            CommandOut, self.node.topic_command_out, 5
        )
        self.node.get_logger().info(f" Advertising to {self.commandout.topic_name}")

        self.node.get_logger().info("Command Manager Initialized.")

    def callback_command(self, msg):
        self.node.get_logger().info(f"Command called: {msg.command}")
        match msg.command:
            case "history":
                history = MeasurementArray()
                history.full_history = True
                for _id, ma in self.node.sensor_histories.items():
                    history.measurements.extend(ma.measurements)
                    history.located_measurements.extend(ma.located_measurements)
                self.node.pub_measurement_array.publish(history)
            case _:
                pass
