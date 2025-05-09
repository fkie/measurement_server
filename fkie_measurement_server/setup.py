from setuptools import find_packages, setup

package_name = "fkie_measurement_server"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # ('share/' + package_name + 'config',
        #  ['config/measurement_server_configs.yaml', 'config/sensor_simulator_config.yaml']),
        (
            "share/" + package_name + "/launch",
            ["launch/measurement_collector.launch.xml"],
        ),
    ],
    install_requires=[
        "setuptools",
        "fkie_measurement_msgs",
        "geometry_msgs",
        "nav_msgs",
        "visualization_msgs",
    ],
    zip_safe=True,
    maintainer="Alexander Tiderko",
    maintainer_email="alexander.tiderko@fkie.fraunhofer.de",
    description="Collects and storages measurement data of any number of sensors",
    license="Apache License 2.0",
    tests_require=["pytest"],
    scripts=[
        "scripts/measurement_collector_node.py",
        # 'scripts/measurement_server_node.py',
    ],
)
