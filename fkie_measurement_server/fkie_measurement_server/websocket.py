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


class WebsocketManager():
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

            if not json_data['unique_serial_id']:
                self.node.get_logger().error(
                    "[callback_measurement] empty [unique_serial_id]."
                )
                return
            
            if json_data['unique_serial_id'] not in self.node.sensor_histories:
                ma = MeasurementArray()
                ma.full_history = True
                self.node.sensor_histories[json_data['unique_serial_id']] = ma

            s_history: MeasurementArray = self.node.sensor_histories[json_data['unique_serial_id']]

            # Located Measurement
            msg_loc = MeasurementLocated()
            msg_loc.pose.pose.position.x = float(json_data['position']['x'])
            msg_loc.pose.pose.position.y = float(json_data['position']['y'])
            msg_loc.pose.pose.position.z = float(json_data['position']['z'])
            msg_loc.pose.pose.orientation.x = float(json_data['orientation']['x'])
            msg_loc.pose.pose.orientation.y = float(json_data['orientation']['y'])
            msg_loc.pose.pose.orientation.z = float(json_data['orientation']['z'])
            msg_loc.pose.pose.orientation.w = float(json_data['orientation']['w'])
            msg_loc.pose.header.frame_id = json_data['frame_id']
            msg_loc.pose.header.stamp.sec = int(json_data['stamp']['sec'])
            msg_loc.pose.header.stamp.nanosec = int(json_data['stamp']['nanosec'])
            msg_loc.measurement.header.frame_id = json_data['frame_id']
            msg_loc.measurement.header.stamp.sec = int(json_data['stamp']['sec'])
            msg_loc.measurement.header.stamp.nanosec = int(json_data['stamp']['nanosec'])
            msg_loc.measurement.unique_serial_id = json_data['unique_serial_id']
            msg_loc.measurement.manufacturer_device_name = json_data['manufacturer_device_name']
            msg_loc.measurement.device_classification = json_data['device_classification']

            if self.node.utm_zone_number and self.node.utm_zone_letter:
                msg_loc.utm_zone_number = self.node.utm_zone_number
                msg_loc.utm_zone_letter = self.node.utm_zone_letter

            s_history.located_measurements.append(msg_loc)

            # Measurement Value
            msg_value = MeasurementValue()
            msg_value.begin.sec = int(json_data['stamp']['sec'])
            msg_value.begin.nanosec = int(json_data['stamp']['nanosec'])
            msg_value.end.sec = int(json_data['stamp']['sec'])
            msg_value.end.nanosec = int(json_data['stamp']['nanosec'])
            msg_value.sensor = json_data['sensor']
            msg_value.source_type = json_data['source_type']
            msg_value.unit = json_data['unit']
            msg_value.value_single = float(json_data['value'])
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

{"frame_id":"world","stamp":{"sec":5000,"nanosec":5000},"position":{"x":366188,"y":5609559,"z":255},"orientation":{"x":0,"y":0,"z":0,"w":1},"utm_zone_number":32,"utm_zone_letter":"U","unique_serial_id":"T-4000","manufacturer_device_name":"Terminator Modell 4000","device_classification":"T","sensor":"threat-level","source_type":"visual","unit":"of 10","value":6}
'''
