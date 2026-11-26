import http.client
import json
import tempfile
import threading
import unittest
from pathlib import Path
from typing import Any

from fkie_measurement_server.measurement import validate_measurement
from fkie_measurement_server.ratelimit import RateLimiter
from fkie_measurement_server.server import MeasurementHTTPServer, RequestHandler
from fkie_measurement_server.storage import MeasurementStore, load_measurements

MEASUREMENT = {
    "stamp": 25.5,
    "x": 1,
    "y": 2,
    "yaw": 0,
    "sensor": "sensor-a",
    "value": 1,
}


class ClientMetadataTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.path = Path(self.temp_dir.name) / "measurements.jsonl"
        self.store = MeasurementStore(self.path)
        self.store.open()

        self.server = MeasurementHTTPServer(
            ("127.0.0.1", 0),
            RequestHandler,
            store=self.store,
            limiter=RateLimiter(0, 0),
            name="test",
            read_only=False,
            file_path=self.path,
        )
        self.thread = threading.Thread(
            target=self.server.serve_forever,
            daemon=True,
        )
        self.thread.start()

    def tearDown(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
        self.store.close()
        self.temp_dir.cleanup()

    def post(self, path: str, payload: Any) -> int:
        host, port = self.server.server_address[:2]
        connection = http.client.HTTPConnection(str(host), port, timeout=5)
        try:
            connection.request(
                "POST",
                path,
                body=json.dumps(payload),
                headers={"Content-Type": "application/json"},
            )
            response = connection.getresponse()
            response.read()
            return response.status
        finally:
            connection.close()

    def test_missing_fields_are_empty(self) -> None:
        result = validate_measurement(MEASUREMENT)
        self.assertEqual(result["client"], "")
        self.assertEqual(result["client_id"], "")

    def test_server_overwrites_client_id_and_persists_it(self) -> None:
        payload = {
            **MEASUREMENT,
            "client": "robot-1",
            "client_id": "forged-address",
        }

        self.assertEqual(self.post("/", payload), 201)

        stored, _ = self.store.get()
        self.assertEqual(stored[0]["client"], "robot-1")
        self.assertEqual(stored[0]["client_id"], "127.0.0.1")

        loaded = load_measurements(self.path)
        self.assertEqual(loaded[0]["client"], "robot-1")
        self.assertEqual(loaded[0]["client_id"], "127.0.0.1")

    def test_batch_query_default_and_per_record_override(self) -> None:
        payload = [
            dict(MEASUREMENT),
            {**MEASUREMENT, "client": "robot-2", "client_id": "forged-address"},
        ]

        self.assertEqual(self.post("/batch?client=team", payload), 201)

        stored, _ = self.store.get()
        self.assertEqual([item["client"] for item in stored], ["team", "robot-2"])
        self.assertEqual(
            [item["client_id"] for item in stored],
            ["127.0.0.1", "127.0.0.1"],
        )


if __name__ == "__main__":
    unittest.main()
