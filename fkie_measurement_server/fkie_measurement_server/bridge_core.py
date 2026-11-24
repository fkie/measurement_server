"""ROS-independent parts of the bridge: message conversion, pacing and HTTP sending."""

import http.client
import json
import math
import queue
import threading
import time
import urllib.error
import urllib.request
from collections.abc import Callable
from typing import Any, NamedTuple, Protocol

Payload = dict[str, Any]

# Server limits (see ratelimit.py): 10 Hz single, one batch per 10 s. Defaults keep a safety margin.
DEFAULT_SINGLE_INTERVAL = 0.11
DEFAULT_BATCH_INTERVAL = 10.5


class LoggerLike(Protocol):
    """Subset of the ROS 2 logger that is needed here."""

    def info(self, msg: str, /) -> Any: ...

    def warning(self, msg: str, /) -> Any: ...

    def error(self, msg: str, /) -> Any: ...


Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # x, y, z, w


class Pose(NamedTuple):
    """Position and orientation (quaternion x, y, z, w) in a frame."""

    position: Vec3
    orientation: Quat


class Transform(NamedTuple):
    """Transform that maps poses from a source frame into a target frame."""

    translation: Vec3
    rotation: Quat


IDENTITY_POSE = Pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def normalize_quaternion(q: Quat) -> Quat:
    """Return the unit quaternion. A (near) zero quaternion becomes the identity."""
    norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if norm < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm


def quaternion_multiply(a: Quat, b: Quat) -> Quat:
    """Hamilton product a * b (apply b first, then a)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vector(q: Quat, v: Vec3) -> Vec3:
    """Rotate vector v by the unit quaternion q."""
    ux, uy, uz, w = q
    vx, vy, vz = v
    tx = 2.0 * (uy * vz - uz * vy)
    ty = 2.0 * (uz * vx - ux * vz)
    tz = 2.0 * (ux * vy - uy * vx)
    return (
        vx + w * tx + (uy * tz - uz * ty),
        vy + w * ty + (uz * tx - ux * tz),
        vz + w * tz + (ux * ty - uy * tx),
    )


def transform_pose(transform: Transform, pose: Pose) -> Pose:
    """Express a pose, given in the source frame, in the target frame of the transform."""
    rotation = normalize_quaternion(transform.rotation)
    rotated = rotate_vector(rotation, pose.position)
    position = (
        rotated[0] + transform.translation[0],
        rotated[1] + transform.translation[1],
        rotated[2] + transform.translation[2],
    )
    orientation = normalize_quaternion(
        quaternion_multiply(rotation, normalize_quaternion(pose.orientation))
    )
    return Pose(position, orientation)


def pose_from_msg(pose: Any) -> Pose:
    """Convert a geometry_msgs/Pose (duck typed) into a Pose."""
    p = pose.position
    q = pose.orientation
    return Pose(
        (float(p.x), float(p.y), float(p.z)),
        (float(q.x), float(q.y), float(q.z), float(q.w)),
    )


def quaternion_to_rpy(
    x: float, y: float, z: float, w: float
) -> tuple[float, float, float]:
    """Convert a quaternion to (roll, pitch, yaw) in radians. Zero quaternion -> identity."""
    x, y, z, w = normalize_quaternion((x, y, z, w))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def valid_zone_number(value: Any) -> int | None:
    """Return the UTM zone number if it is in 1..60, else None."""
    try:
        number = int(value)
    except (TypeError, ValueError):
        return None
    return number if 1 <= number <= 60 else None


def valid_zone_letter(value: Any) -> str:
    """Return the upper-case UTM zone letter if it is a single letter, else ''."""
    text = str(value).strip().upper() if value is not None else ""
    return text if len(text) == 1 and text.isalpha() else ""


def _pick_stamp(candidates: list[tuple[int, int]], now: float) -> float:
    """Return the first non-zero (sec, nanosec) as float seconds, else now."""
    for sec, nanosec in candidates:
        if sec or nanosec:
            return float(sec) + float(nanosec) * 1e-9
    return now


def measurement_to_payloads(
    measurement: Any,
    pose: Pose,
    frame_id: str,
    zone_number: int | None,
    zone_letter: str,
    now: float,
    extra_stamp: tuple[int, int] = (0, 0),
) -> list[Payload]:
    """Convert one Measurement into one server payload per measurement value.

    The pose must already be given in frame_id (normally the global frame).
    """
    roll, pitch, yaw = quaternion_to_rpy(*pose.orientation)
    position = pose.position
    if not all(math.isfinite(v) for v in position + (roll, pitch, yaw)):
        return []

    payloads: list[Payload] = []
    for value in measurement.values:
        sensor = str(value.sensor or "").strip()
        number = float(value.value_single)
        if not sensor or not math.isfinite(number):
            continue
        stamp = _pick_stamp(
            [
                (int(value.end.sec), int(value.end.nanosec)),
                (
                    int(measurement.header.stamp.sec),
                    int(measurement.header.stamp.nanosec),
                ),
                extra_stamp,
            ],
            now,
        )
        if stamp < 0.0:
            continue  # the server rejects negative stamps
        payloads.append(
            {
                "stamp": stamp,
                "frame_id": frame_id,
                "x": position[0],
                "y": position[1],
                "z": position[2],
                "yaw": yaw,
                "pitch": pitch,
                "roll": roll,
                "utm_zone_number": zone_number,
                "utm_zone_letter": zone_letter,
                "unique_serial_id": str(measurement.unique_serial_id),
                "manufacturer_device_name": str(measurement.manufacturer_device_name),
                "device_classification": str(measurement.device_classification),
                "sensor": sensor,
                "source_type": str(value.source_type),
                "unit": str(value.unit),
                "value": number,
            }
        )
    return payloads


def located_to_payloads(
    located: Any,
    pose: Pose,
    frame_id: str,
    default_zone_number: int | None,
    default_zone_letter: str,
    now: float,
) -> list[Payload]:
    """Convert one MeasurementLocated (with an already resolved pose) into payloads."""
    zone_number = valid_zone_number(located.utm_zone_number) or default_zone_number
    zone_letter = valid_zone_letter(located.utm_zone_letter) or default_zone_letter
    pose_stamp = (
        int(located.pose.header.stamp.sec),
        int(located.pose.header.stamp.nanosec),
    )
    return measurement_to_payloads(
        located.measurement, pose, frame_id, zone_number, zone_letter, now, pose_stamp
    )


class PostResult(NamedTuple):
    """Outcome of one HTTP request: 'ok', 'retry', 'rate_limited' or 'rejected'."""

    status: str
    message: str = ""
    retry_after: float = 0.0


def _parse_retry_after(value: str | None) -> float:
    try:
        seconds = float(value) if value is not None else 1.0
    except ValueError:
        seconds = 1.0
    return min(60.0, max(0.1, seconds))


def _read_detail(exc: urllib.error.HTTPError) -> str:
    try:
        return exc.read().decode("utf-8", errors="replace")[:300]
    except Exception:  # noqa: BLE001
        return ""


class ServerClient:
    """Minimal HTTP client (standard library only) posting a batch to the server."""

    def __init__(self, url: str, timeout: float) -> None:
        self._url = url
        self._timeout = timeout

    def post(self, payloads: list[Payload]) -> PostResult:
        try:
            data = json.dumps(payloads, allow_nan=False).encode("utf-8")
        except ValueError as exc:
            return PostResult("rejected", f"Payload cannot be serialized: {exc}")
        request = urllib.request.Request(
            self._url,
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self._timeout) as response:
                _ = response.read()
            return PostResult("ok")
        except urllib.error.HTTPError as exc:
            detail = _read_detail(exc)
            if exc.code == 429:
                retry_after = _parse_retry_after(exc.headers.get("Retry-After"))
                return PostResult("rate_limited", f"HTTP 429 {detail}", retry_after)
            status = "retry" if exc.code >= 500 else "rejected"
            return PostResult(status, f"HTTP {exc.code} {detail}")
        except (urllib.error.URLError, http.client.HTTPException, OSError) as exc:
            return PostResult("retry", str(exc))


class Pacer:
    """Keeps minimum intervals between single requests and between batch requests."""

    def __init__(self, single_interval: float, batch_interval: float) -> None:
        self._single_interval = max(0.0, single_interval)
        self._batch_interval = max(0.0, batch_interval)
        self._last_single: float | None = None
        self._last_batch: float | None = None

    def delay(self, batch: bool, now: float) -> float:
        """Seconds to wait until a request of the given kind may be sent."""
        last = self._last_batch if batch else self._last_single
        interval = self._batch_interval if batch else self._single_interval
        if last is None:
            return 0.0
        return max(0.0, last + interval - now)

    def mark(self, batch: bool, now: float) -> None:
        """Remember that a request of the given kind was sent at `now`."""
        if batch:
            self._last_batch = now
        else:
            self._last_single = now


class PacedSender:
    """Sends payloads from a queue in a worker thread and respects the server rate limits.

    One pending payload is sent as a single request (at most every single_interval).
    If several payloads are pending and the batch interval is over, they are sent as one batch.
    """

    def __init__(
        self,
        client: ServerClient,
        pacer: Pacer,
        log: LoggerLike,
        queue_size: int,
        max_batch_size: int,
        max_retries: int,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._client = client
        self._pacer = pacer
        self._log = log
        self._max_batch_size = max(2, max_batch_size)
        self._max_retries = max(1, max_retries)
        self._clock = clock
        self._queue: queue.Queue[Payload] = queue.Queue(maxsize=max(1, queue_size))
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="http-sender"
        )
        self._dropped = 0
        self._last_drop_log = 0.0

    def start(self) -> None:
        self._thread.start()

    def submit(self, payload: Payload) -> bool:
        """Queue a payload. Returns False (and logs throttled) if the queue is full."""
        try:
            self._queue.put_nowait(payload)
            return True
        except queue.Full:
            self._dropped += 1
            now = self._clock()
            if now - self._last_drop_log >= 5.0:
                self._last_drop_log = now
                self._log.warning(
                    f"Send queue is full, {self._dropped} measurement(s) dropped so far"
                )
            return False

    def stop(self, timeout: float = 2.0) -> None:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=timeout)
        remaining = self._queue.qsize()
        if remaining:
            self._log.warning(f"{remaining} queued measurement(s) were not sent")

    def _refill(self, pending: list[Payload]) -> None:
        while len(pending) < self._max_batch_size:
            try:
                pending.append(self._queue.get_nowait())
            except queue.Empty:
                return

    def _run(self) -> None:
        pending: list[Payload] = []
        while not self._stop_event.is_set():
            if not pending:
                try:
                    pending.append(self._queue.get(timeout=0.5))
                except queue.Empty:
                    continue
            self._refill(pending)
            now = self._clock()
            if len(pending) > 1 and self._pacer.delay(True, now) <= 0.0:
                batch = list(pending)
                pending.clear()
                self._send(batch)
                continue
            wait = self._pacer.delay(False, now)
            if wait > 0.0:
                # More payloads may arrive meanwhile, so re-evaluate afterwards
                _ = self._stop_event.wait(wait)
                continue
            self._send([pending.pop(0)])

    def _send(self, batch: list[Payload]) -> None:
        """Send one request with retries. Messages are dropped after max_retries failures."""
        is_batch = len(batch) > 1
        for attempt in range(1, self._max_retries + 1):
            wait = self._pacer.delay(is_batch, self._clock())
            if wait > 0.0 and self._stop_event.wait(wait):
                return
            self._pacer.mark(is_batch, self._clock())
            result = self._client.post(batch)
            if result.status == "ok":
                return
            if result.status == "rejected":
                self._log.error(
                    f"Server rejected {len(batch)} measurement(s): {result.message}"
                )
                return
            if result.status == "rate_limited":
                self._log.warning(
                    f"Server rate limit hit (attempt {attempt}/{self._max_retries}): {result.message}"
                )
                if self._stop_event.wait(result.retry_after):
                    return
            else:
                self._log.warning(
                    f"Server not reachable (attempt {attempt}/{self._max_retries}): {result.message}"
                )
                if self._stop_event.wait(1.0):
                    return
        self._log.error(
            f"Dropped {len(batch)} measurement(s) after {self._max_retries} attempts"
        )
