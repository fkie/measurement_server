"""Entry point of the ROS bridge. Reports clearly if ROS 2 is not available."""

import sys


def main() -> int:
    try:
        from .ros_bridge import run
    except ImportError as exc:
        print(
            f"ROS 2 or fkie_measurement_msgs is not available: {exc}\n"
            "Source your ROS 2 workspace first. The web server (measurement_server) works without ROS.",
            file=sys.stderr,
        )
        return 1
    return run()


if __name__ == "__main__":
    sys.exit(main())
