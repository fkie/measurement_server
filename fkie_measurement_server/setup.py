from glob import glob

from setuptools import find_packages, setup

package_name = "fkie_measurement_server"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py") + glob("launch/*.launch.xml"),
        ),
    ],
    install_requires=["setuptools"],
    python_requires=">=3.10",
    zip_safe=True,
    maintainer="Alexander Tiderko",
    maintainer_email="alexander.tiderko@fkie.fraunhofer.de",
    description="ROS-independent measurement web server with a ROS 2 bridge",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "measurement_server = fkie_measurement_server.server:main",
            "measurement_ros_bridge = fkie_measurement_server.bridge_entry:main",
        ],
    },
)
