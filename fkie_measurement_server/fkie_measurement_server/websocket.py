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

from fkie_measurement_msgs.msg import MeasurementValue, MeasurementArray, MeasurementLocated

from threading import Thread
import websockets
import asyncio
import json
import time


class WebsocketManager:
    def __init__(self, node=None, port=8899):
        self.node = node
        self.websocket = None
        self.websocket_port = port
        self.loop = asyncio.new_event_loop()

        self.node.get_logger().info(' ')
        self.node.get_logger().info('Starting Websocket...')

        self.thread = Thread(target=self.run_server, daemon=True)
        self.thread.start()

        self.node.get_logger().info('Websocket started successfully.')

    def run_server(self):
        asyncio.set_event_loop(self.loop)
        self.websocket = websockets.serve(self.handler, 'localhost', self.websocket_port)

        # this is misleading, as this only creates the websocket server but does not run it
        # it converts the websockets.legacy.server.Serve class into a Future and awaits its completion
        self.loop.run_until_complete(self.websocket)

        # this actually starts the server
        self.loop.run_forever()
        self.loop.close()

    async def handler(self, websocket):
        try:
            while True:
                message = await websocket.recv()
                self.handle_message(message)
        except websockets.exceptions.ConnectionClosedOK:
            pass

    def handle_message(self, message):
        try:
            json_data = json.loads(message)

            # extract json parameters
            frame_id = json_data.get('frame_id')  # optional
            stamp_sec = json_data.get('stamp_sec')  # optional
            stamp_nanosec = json_data.get('stamp_nanosec')  # optional
            position_x = json_data.get('position_x')  # optional
            position_y = json_data.get('position_y')  # optional
            position_z = json_data.get('position_z')  # optional
            orientation_x = json_data.get('orientation_x')  # optional
            orientation_y = json_data.get('orientation_y')  # optional
            orientation_z = json_data.get('orientation_z')  # optional
            orientation_w = json_data.get('orientation_w')  # optional
            utm_zone_number = json_data.get('utm_zone_number')  # optional
            utm_zone_letter = json_data.get('utm_zone_letter')  # optional
            unique_serial_id = json_data.get('unique_serial_id')  # required
            manufacturer_device_name = json_data.get('manufacturer_device_name')  # required
            device_classification = json_data.get('device_classification')  # required
            sensor = json_data.get('sensor')  # required
            source_type = json_data.get('source_type')  # required
            unit = json_data.get('unit')  # required
            value = json_data.get('value')  # required

            if not unique_serial_id:
                self.node.get_logger().error(
                    "[callback_measurement] empty [unique_serial_id]."
                )
                return
            
            if unique_serial_id not in self.node.sensor_histories:
                ma = MeasurementArray()
                ma.full_history = True
                self.node.sensor_histories[unique_serial_id] = ma

            s_history: MeasurementArray = self.node.sensor_histories[unique_serial_id]

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

            if self.node.utm_zone_number and self.node.utm_zone_letter:
                msg_loc.utm_zone_number = self.node.utm_zone_number
                msg_loc.utm_zone_letter = self.node.utm_zone_letter

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

            self.node.pub_measurement_array.publish(msg_array)

        except Exception as error:
            self.node.get_logger().warn(f'Could not parse message: {message}')
            self.node.get_logger().warn(f'Error: {error}')
        

'''
Websocket JSON Format:
{
    frame_id: string
    stamp: {
        sec: number
        nanosec: number
    }
    position: {
        x: number
        y: number
        z: number
    }
    orientation: {
        x: number
        y: number
        z: number
        w: number
    }
    utm_zone_number: number
    utm_zone_letter: string
    unique_serial_id: string
    manufacturer_device_name: string
    device_classification: string
    sensor: string
    source_type: string
    unit: string
    value: number
}

Test JSON:
{
    "frame_id": "world",
    "stamp": {
        "sec": 5000,
        "nanosec": 5000
    },
    "position": {
        "x": 366188,
        "y": 5609559,
        "z": 255
    },
    "orientation": {
        "x": 0,
        "y": 0,
        "z": 0,
        "w": 1
    },
    "utm_zone_number": 32,
    "utm_zone_letter": "U",
    "unique_serial_id": "T-4000",
    "manufacturer_device_name": "Terminator Modell 4000",
    "device_classification": "T",
    "sensor": "threat-level",
    "source_type": "visual",
    "unit": "of 10",
    "value": 6
}

Test string:
{"frame_id":"world","stamp":{"sec":5000,"nanosec":5000},"position":{"x":366188,"y":5609559,"z":255},"orientation":{"x":0,"y":0,"z":0,"w":1},"utm_zone_number":32,"utm_zone_letter":"U","unique_serial_id":"T-4000","manufacturer_device_name":"Terminator Modell 4000","device_classification":"T","sensor":"threat-level","source_type":"visual","unit":"of 10","value":6}

Minimal test string:
{"unique_serial_id":"T-4000","manufacturer_device_name":"Terminator Modell 4000","device_classification":"T","sensor":"threat-level","source_type":"visual","unit":"of 10","value":6}

'''
