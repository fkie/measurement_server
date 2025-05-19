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

from threading import Thread
import websockets
import asyncio
import json


class WebsocketManager:
    """
    A simple websocket to receive measurement data.
    """
    def __init__(self, node=None, port=8899, method=None):
        """
        Initialize the websocket and start it on a separate thread.
        """
        self.node = node
        self.websocket = None
        self.websocket_port = port
        self.create_message = method

        self.loop = asyncio.new_event_loop()

        self.node.get_logger().info(' ')
        self.node.get_logger().info('Starting Websocket...')

        self.thread = Thread(target=self.run_server, daemon=True)
        self.thread.start()

        self.node.get_logger().info('Websocket started successfully.')

    def run_server(self):
        """
        Start the websocket.
        """
        asyncio.set_event_loop(self.loop)
        self.websocket = websockets.serve(self.handler, 'localhost', self.websocket_port)

        # this is misleading, as this only creates the websocket server but does not run it
        # it converts the websockets.legacy.server.Serve class into a Future and awaits its completion
        self.loop.run_until_complete(self.websocket)

        # this actually starts the server
        self.loop.run_forever()
        self.loop.close()

    async def handler(self, websocket):
        """
        Websocket loop
        """
        try:
            while True:
                message = await websocket.recv()
                self.handle_message(message)
        except websockets.exceptions.ConnectionClosedOK:
            pass

    def handle_message(self, message):
        """
        Method to call when receiving a measurement
        """
        try:
            json_data = json.loads(message)
            self.create_message(json_data, "websocket")

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
{"unique_serial_id":"T-4000","sensor":"threat-level","value":6}
'''
