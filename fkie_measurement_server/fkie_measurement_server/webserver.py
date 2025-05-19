from threading import Thread

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
            # because we use request.args[] instead of request.args.get(), it will immediately throw an error if a value is missing
            frame_id = request.args['frame_id']
            stamp_sec = request.args['stamp_sec']
            stamp_nanosec = request.args['stamp_nanosec']
            position_x = request.args['position_x']
            position_y = request.args['position_y']
            position_z = request.args['position_z']
            orientation_x = request.args['orientation_x']
            orientation_y = request.args['orientation_y']
            orientation_z = request.args['orientation_z']
            orientation_w = request.args['orientation_w']
            utm_zone_number = request.args['utm_zone_number']
            utm_zone_letter = request.args['utm_zone_letter']
            unique_serial_id = request.args['unique_serial_id']
            manufacturer_device_name = request.args['manufacturer_device_name']
            device_classification = request.args['device_classification']
            sensor = request.args['sensor']
            source_type = request.args['source_type']
            unit = request.args['unit']
            value = request.args['value']

            if unique_serial_id not in self.node.sensor_histories:
                ma = MeasurementArray()
                ma.full_history = True
                self.node.sensor_histories[unique_serial_id] = ma

            s_history: MeasurementArray = self.node.sensor_histories[unique_serial_id]

            # Located Measurement
            msg_loc = MeasurementLocated()
            msg_loc.pose.pose.position.x = float(position_x)
            msg_loc.pose.pose.position.y = float(position_y)
            msg_loc.pose.pose.position.z = float(position_z)
            msg_loc.pose.pose.orientation.x = float(orientation_x)
            msg_loc.pose.pose.orientation.y = float(orientation_y)
            msg_loc.pose.pose.orientation.z = float(orientation_z)
            msg_loc.pose.pose.orientation.w = float(orientation_w)
            msg_loc.pose.header.frame_id = frame_id
            msg_loc.pose.header.stamp.sec = int(stamp_sec)
            msg_loc.pose.header.stamp.nanosec = int(stamp_nanosec)
            msg_loc.measurement.header.frame_id = frame_id
            msg_loc.measurement.header.stamp.sec = int(stamp_sec)
            msg_loc.measurement.header.stamp.nanosec = int(stamp_nanosec)
            msg_loc.measurement.unique_serial_id = unique_serial_id
            msg_loc.measurement.manufacturer_device_name = manufacturer_device_name
            msg_loc.measurement.device_classification = device_classification

            if self.node.utm_zone_number and self.node.utm_zone_letter:
                msg_loc.utm_zone_number = self.node.utm_zone_number
                msg_loc.utm_zone_letter = self.node.utm_zone_letter

            s_history.located_measurements.append(msg_loc)

            # Measurement Value
            msg_value = MeasurementValue()
            msg_value.begin.sec = int(stamp_sec)
            msg_value.begin.nanosec = int(stamp_nanosec)
            msg_value.end.sec = int(stamp_sec)
            msg_value.end.nanosec = int(stamp_nanosec)
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
"""