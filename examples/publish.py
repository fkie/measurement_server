import json
import sys
import time
import urllib.error
import urllib.request

SERVER_URL = "http://127.0.0.1:8080/measurements"


def publish(measurement: dict[str, object]) -> None:
    body = json.dumps(measurement, allow_nan=False).encode("utf-8")
    request = urllib.request.Request(
        SERVER_URL,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    with urllib.request.urlopen(request, timeout=5.0) as response:
        print(f"HTTP {response.status}: {response.read().decode('utf-8')}")


if __name__ == "__main__":
    measurement = {
        "stamp": time.time(),
        "frame_id": "map",
        "client": "robot-1",
        "x": 366188.0,
        "y": 5609559.0,
        "yaw": 0.0,
        "sensor": "temperature",
        "unit": "C",
        "value": 21.5,
    }

    try:
        publish(measurement)
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        print(f"Server rejected the request: HTTP {exc.code} {detail}", file=sys.stderr)
        sys.exit(1)
    except (urllib.error.URLError, TimeoutError) as exc:
        print(f"Request failed: {exc}", file=sys.stderr)
        sys.exit(1)
