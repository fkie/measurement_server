from threading import Thread

import time
from flask import Flask, jsonify, request
from fkie_measurement_msgs.msg import MeasurementValue, MeasurementArray, MeasurementLocated


class WebServerManager:
    """
    A simple web server to receive measurement data via HTTP POST requests.
    """
    def __init__(self, node=None, port=8080):
        """
        Initialize the web server and start it on a separate thread.
        """
        self.node = node
        self.port = port

        self.app = Flask(__name__)

        # This is a bit confusing, '/' is the actual route, 'receive_measurement' is just the name of the method
        self.app.add_url_rule('/', 'receive_measurement', self.receive_measurement, methods=['POST'])

        self.node.get_logger().info(' ')
        self.node.get_logger().info('Starting Webserver...')

        self.thread = Thread(target=self.start, daemon=True)
        self.thread.start()

        self.node.get_logger().info('Webserver started successfully.')

    def start(self):
        """
        Start the Flask web server.
        """
        # This is the development server bundled with flask
        # self.app.run(port=self.port)

        # This is a production server
        from waitress import serve
        serve(self.app, host="0.0.0.0", port=self.port)

    def receive_measurement(self):
        """
        Endpoint to receive measurement data.
        """
        try:
            # extract query parameters
            frame_id = request.args.get('frame_id')  # optional
            stamp_sec = request.args.get('stamp_sec')  # optional
            stamp_nanosec = request.args.get('stamp_nanosec')  # optional
            position_x = request.args.get('position_x')  # optional
            position_y = request.args.get('position_y')  # optional
            position_z = request.args.get('position_z')  # optional
            orientation_x = request.args.get('orientation_x')  # optional
            orientation_y = request.args.get('orientation_y')  # optional
            orientation_z = request.args.get('orientation_z')  # optional
            orientation_w = request.args.get('orientation_w')  # optional
            utm_zone_number = request.args.get('utm_zone_number')  # optional
            utm_zone_letter = request.args.get('utm_zone_letter')  # optional
            unique_serial_id = request.args.get('unique_serial_id')  # required
            manufacturer_device_name = request.args.get('manufacturer_device_name')  # required
            device_classification = request.args.get('device_classification')  # required
            sensor = request.args.get('sensor')  # required
            source_type = request.args.get('source_type')  # required
            unit = request.args.get('unit')  # required
            value = request.args.get('value')  # required

            if not(manufacturer_device_name and device_classification and sensor and source_type and unit and value):
                return jsonify({"status": "error", "message": "Required values missing."}), 400

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

            return jsonify({"status": "success", "message": "Measurement received"}), 200
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 500

"""
Test string:
curl -X POST "http://localhost:8080/?frame_id=world&stamp_sec=5000&stamp_nanosec=5000&position_x=366188&position_y=5609559&position_z=255&orientation_x=0.0&orientation_y=0&orientation_z=0&orientation_w=1&utm_zone_number=32&utm_zone_letter=U&unique_serial_id=T-4000&manufacturer_device_name=Terminator%20Modell%204000&device_classification=T&sensor=threat-level&source_type=visual&unit=of%2010&value=6"

Minimal test string:
curl -X POST "http://localhost:8080/?unique_serial_id=T-4000&manufacturer_device_name=Terminator%20Modell%204000&device_classification=T&sensor=threat-level&source_type=visual&unit=of%2010&value=6"

"""