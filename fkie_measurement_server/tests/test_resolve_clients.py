import json
import tempfile
import unittest
from pathlib import Path

from fkie_measurement_server.storage import read_jsonl, resolve_file


class ResolveClientTests(unittest.TestCase):
    def test_resolve_uses_client_and_falls_back_to_client_id(self) -> None:
        base = {"stamp": 1, "x": 1, "y": 2, "yaw": 0, "sensor": "a"}
        records = [
            {**base, "client": "robot-a", "client_id": "192.0.2.1", "value": 1},
            {**base, "client": "robot-a", "client_id": "192.0.2.2", "value": 2},
            {**base, "client": "robot-b", "client_id": "192.0.2.1", "value": 3},
            {**base, "client": " ", "client_id": "192.0.2.1", "value": 4},
            {**base, "client_id": "192.0.2.2", "value": 5},
            {**base, "value": 6},
            {**base, "client": "", "client_id": "", "value": 7},
            {**base, "stamp": 2, "client": "robot-a", "value": 8},
        ]

        with tempfile.TemporaryDirectory() as directory:
            src = Path(directory) / "input.jsonl"
            dst = Path(directory) / "resolved.jsonl"
            src.write_text(
                "".join(json.dumps(record) + "\n" for record in records),
                encoding="utf-8",
            )

            self.assertEqual(resolve_file(src, dst), (8, 6, 0))
            self.assertEqual(
                [record["value"] for record in read_jsonl(dst)],
                [2, 3, 4, 5, 7, 8],
            )


if __name__ == "__main__":
    unittest.main()
