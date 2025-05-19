from threading import Thread

from flask import Flask, jsonify, request


class WebServerManager:
    """
    A simple web server to receive measurement data via HTTP POST requests.
    """
    def __init__(self, node=None, port=8080, method=None):
        """
        Initialize the web server and start it on a separate thread.
        """
        self.node = node
        self.port = port
        self.create_message = method

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
            self.create_message(request.args, "webserver")
            return jsonify({"status": "success", "message": "Measurement received"}), 200
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 500

"""
Test string:
curl -X POST "http://localhost:8080/?frame_id=world&stamp_sec=5000&stamp_nanosec=5000&position_x=366188&position_y=5609559&position_z=255&orientation_x=0.0&orientation_y=0&orientation_z=0&orientation_w=1&utm_zone_number=32&utm_zone_letter=U&unique_serial_id=T-4000&manufacturer_device_name=Terminator%20Modell%204000&device_classification=T&sensor=threat-level&source_type=visual&unit=of%2010&value=6"

Minimal test string:
curl -X POST "http://localhost:8080/?unique_serial_id=T-4000&sensor=threat-level&source_type=visual&unit=of%2010&value=6"
"""