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

import sys

from fkie_measurement_server.measurement_server import MeasurementCollectorNode

import rclpy
import rclpy.executors


def main(args=None):
    rclpy.init(args=args)
    node = MeasurementCollectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as err:
        import traceback

        print(traceback.format_exc())
        print("Error while initialize ROS-Node: %s" % (err), file=sys.stderr)
    finally:
        print("Shutdown...")
        node.stop()
        node.destroy_node()
        print("Bye!")
