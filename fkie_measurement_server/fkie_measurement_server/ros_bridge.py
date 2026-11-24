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

# ROS packages are not installed in the pre-commit environment
# pyright: reportMissingImports=false

"""ROS 2 node that forwards measurements to the measurement web server (with TF)."""

import time
from typing import Any

import rclpy
import tf2_ros
from fkie_measurement_msgs.msg import Measurement, MeasurementArray, MeasurementLocated
from rclpy.node import Node
from rclpy.time import Time

from .bridge_core import (
    DEFAULT_BATCH_INTERVAL,
    DEFAULT_SINGLE_INTERVAL,
    IDENTITY_POSE,
    PacedSender,
    Pacer,
    Payload,
    Pose,
    ServerClient,
    Transform,
    located_to_payloads,
    measurement_to_payloads,
    pose_from_msg,
    transform_pose,
    valid_zone_letter,
    valid_zone_number,
)


class MeasurementBridgeNode(Node):
    """Subscribes to measurement topics, transforms poses with TF and posts them to the server."""

    def __init__(self) -> None:
        super().__init__("measurement_bridge_node")

        self.declare_parameter("topic_sub_measurement_array", "measurement_array")
        self.declare_parameter("topic_sub_measurement", "/in_measurement")
        self.declare_parameter(
            "topic_sub_measurement_located", "/in_measurement_located"
        )
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("tf_fallback_latest", True)
        self.declare_parameter("server_host", "127.0.0.1")
        self.declare_parameter("server_port", 8080)
        self.declare_parameter("utm_zone_number", 32)
        self.declare_parameter("utm_zone_letter", "U")
        self.declare_parameter("ignore_full_history", True)
        self.declare_parameter("queue_size", 10000)
        self.declare_parameter("max_batch_size", 1000)
        self.declare_parameter("max_retries", 3)
        self.declare_parameter("request_timeout", 5.0)
        # Server limits: max 10 Hz single requests, max one batch per 10 s
        self.declare_parameter("min_single_interval", DEFAULT_SINGLE_INTERVAL)
        self.declare_parameter("min_batch_interval", DEFAULT_BATCH_INTERVAL)

        self.topic_array = str(self.get_parameter("topic_sub_measurement_array").value)
        topic_measurement = str(self.get_parameter("topic_sub_measurement").value)
        topic_located = str(self.get_parameter("topic_sub_measurement_located").value)
        self._global_frame = str(self.get_parameter("global_frame").value).strip()
        self._tf_fallback_latest = bool(self.get_parameter("tf_fallback_latest").value)
        host = str(self.get_parameter("server_host").value)
        port = int(self.get_parameter("server_port").value)
        self.utm_zone_number = valid_zone_number(
            self.get_parameter("utm_zone_number").value
        )
        self.utm_zone_letter = valid_zone_letter(
            self.get_parameter("utm_zone_letter").value
        )
        self.ignore_full_history = bool(self.get_parameter("ignore_full_history").value)
        single_interval = float(self.get_parameter("min_single_interval").value)
        batch_interval = float(self.get_parameter("min_batch_interval").value)

        url = f"http://{host}:{port}/"
        self.get_logger().info("Started with parameters:")
        self.get_logger().info(f"  topic_sub_measurement_array: {self.topic_array}")
        self.get_logger().info(
            f"  topic_sub_measurement: {topic_measurement or '(disabled)'}"
        )
        self.get_logger().info(
            f"  topic_sub_measurement_located: {topic_located or '(disabled)'}"
        )
        self.get_logger().info(
            f"  global_frame: {self._global_frame or '(TF disabled)'}"
        )
        self.get_logger().info(f"  tf_fallback_latest: {self._tf_fallback_latest}")
        self.get_logger().info(f"  server url: {url}")
        self.get_logger().info(f"  utm_zone_number: {self.utm_zone_number}")
        self.get_logger().info(f"  utm_zone_letter: {self.utm_zone_letter}")
        self.get_logger().info(f"  ignore_full_history: {self.ignore_full_history}")
        self.get_logger().info(f"  min_single_interval: {single_interval} s")
        self.get_logger().info(f"  min_batch_interval: {batch_interval} s")
        if single_interval < 0.1 or batch_interval < 10.0:
            self.get_logger().warning(
                "Intervals are below the default server limits (0.1 s / 10 s), "
                "the server may reject requests with HTTP 429"
            )

        # The listener uses the node executor, so lookups must never block (no timeout)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        client = ServerClient(url, float(self.get_parameter("request_timeout").value))
        self._sender = PacedSender(
            client=client,
            pacer=Pacer(single_interval, batch_interval),
            log=self.get_logger(),
            queue_size=int(self.get_parameter("queue_size").value),
            max_batch_size=int(self.get_parameter("max_batch_size").value),
            max_retries=int(self.get_parameter("max_retries").value),
        )
        self._sender.start()

        _ = self.create_subscription(
            MeasurementArray, self.topic_array, self._on_array, 10
        )
        if topic_measurement:
            _ = self.create_subscription(
                Measurement, topic_measurement, self._on_measurement, 10
            )
        if topic_located:
            _ = self.create_subscription(
                MeasurementLocated, topic_located, self._on_located, 10
            )

    # ------------------------------------------------------------------ TF
    @staticmethod
    def _stamp_of(header: Any) -> tuple[int, int]:
        return int(header.stamp.sec), int(header.stamp.nanosec)

    def _lookup_transform(
        self, source_frame: str, stamp: tuple[int, int]
    ) -> Transform | None:
        """Look up the transform source_frame -> global_frame without blocking."""
        times: list[Time] = []
        if stamp != (0, 0):
            times.append(Time(nanoseconds=stamp[0] * 1_000_000_000 + stamp[1]))
        if stamp == (0, 0) or self._tf_fallback_latest:
            times.append(Time())  # latest available transform
        last_error: Exception | None = None
        for when in times:
            try:
                msg = self._tf_buffer.lookup_transform(
                    self._global_frame, source_frame, when
                )
            except tf2_ros.TransformException as exc:  # pyright: ignore[reportAttributeAccessIssue]
                last_error = exc
                continue
            t = msg.transform.translation
            r = msg.transform.rotation
            return Transform(
                (float(t.x), float(t.y), float(t.z)),
                (float(r.x), float(r.y), float(r.z), float(r.w)),
            )
        self.get_logger().warning(
            f"No TF between [{self._global_frame}] and [{source_frame}], measurement dropped: "
            f"{last_error}",
            throttle_duration_sec=5.0,
        )
        return None

    def _to_global(
        self, frame_id: str, stamp: tuple[int, int], pose: Pose
    ) -> tuple[Pose, str] | None:
        """Return the pose in the global frame and the frame name, or None if TF is missing."""
        target = self._global_frame
        if not target:
            return pose, frame_id  # TF disabled, forward as is
        if not frame_id or frame_id == target:
            return pose, target  # a pose without frame is assumed to be global already
        transform = self._lookup_transform(frame_id, stamp)
        if transform is None:
            return None
        return transform_pose(transform, pose), target

    # ------------------------------------------------------------ handlers
    def _submit(self, payloads: list[Payload]) -> None:
        for payload in payloads:
            _ = self._sender.submit(payload)

    def _handle_located(self, located: MeasurementLocated, now: float) -> None:
        stamp = self._stamp_of(located.pose.header)
        if stamp == (0, 0):
            stamp = self._stamp_of(located.measurement.header)
        frame_id = str(
            located.pose.header.frame_id or located.measurement.header.frame_id
        )
        resolved = self._to_global(frame_id, stamp, pose_from_msg(located.pose.pose))
        if resolved is None:
            return
        pose, frame = resolved
        self._submit(
            located_to_payloads(
                located, pose, frame, self.utm_zone_number, self.utm_zone_letter, now
            )
        )

    def _handle_measurement(self, measurement: Measurement, now: float) -> None:
        """A measurement without pose is located at the origin of its sensor frame."""
        if not self._global_frame:
            self.get_logger().warning(
                "Measurement without pose needs a global_frame, dropped",
                throttle_duration_sec=5.0,
            )
            return
        frame_id = str(measurement.header.frame_id)
        if not frame_id:
            self.get_logger().warning(
                "Measurement without pose has no frame_id, dropped",
                throttle_duration_sec=5.0,
            )
            return
        resolved = self._to_global(
            frame_id, self._stamp_of(measurement.header), IDENTITY_POSE
        )
        if resolved is None:
            return
        pose, frame = resolved
        self._submit(
            measurement_to_payloads(
                measurement,
                pose,
                frame,
                self.utm_zone_number,
                self.utm_zone_letter,
                now,
            )
        )

    # ----------------------------------------------------------- callbacks
    def _on_array(self, msg: MeasurementArray) -> None:
        if self.ignore_full_history and msg.full_history:
            self.get_logger().debug("Ignoring full history message")
            return
        now = time.time()
        for measurement in msg.measurements:
            self._safe(self._handle_measurement, measurement, now)
        for located in msg.located_measurements:
            self._safe(self._handle_located, located, now)

    def _on_measurement(self, msg: Measurement) -> None:
        self._safe(self._handle_measurement, msg, time.time())

    def _on_located(self, msg: MeasurementLocated) -> None:
        self._safe(self._handle_located, msg, time.time())

    def _safe(self, handler: Any, msg: Any, now: float) -> None:
        """Run a handler and log errors instead of crashing the executor."""
        try:
            handler(msg, now)
        except Exception as exc:  # pylint: disable=broad-except  # noqa: BLE001
            self.get_logger().error(f"Cannot convert measurement: {exc}")

    def destroy_node(self) -> None:
        """Stop the sender thread before the node is destroyed."""
        self.get_logger().info("Shutting down...")
        self._sender.stop()
        super().destroy_node()


def run(args: list[str] | None = None) -> int:
    """Initialize ROS, spin the node and clean up."""
    rclpy.init(args=args)
    node: MeasurementBridgeNode | None = None
    try:
        node = MeasurementBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
