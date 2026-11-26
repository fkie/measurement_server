# fkie_measurement_server

Web server for collecting measurements, with a ROS 2 bridge. The server only needs the
Python standard library and has no dependency on ROS.

- Measurements are posted via HTTP POST (single or as a batch).
- They are stored in a JSON Lines file and kept in memory.
- A GUI receives new measurements live via Server-Sent Events (SSE), without polling.
- The ROS 2 bridge forwards measurement messages to the server and applies TF.

## Requirements

- Python >= 3.10
- For the bridge: ROS 2 (Humble or newer) and [fkie_measurement_msgs](https://github.com/fkie/fkie_measurement_msgs)

## Installation

Web server only, without ROS:

    pip install .
    measurement_server --port 8080
    # or: python -m fkie_measurement_server

With ROS 2:

    colcon build --packages-select fkie_measurement_server
    source install/setup.bash

## Starting the web server

    measurement_server --port 8080 --name test

All options are optional:

- `--port N`: port to listen on (default 8080)
- `--host ADDR`: bind address (default 0.0.0.0, use 127.0.0.1 for local use)
- `--name NAME`: file name `<NAME>.jsonl` (default `measurements`)
- `--data-dir DIR`: directory for measurement files (default `./measurement_data`)
- `--append`: append to the last file
- `--clear`: delete all files of this name and start over
- `--play FILE`: replay a file, nothing is stored, new POSTs are rejected (HTTP 409)
- `--play-speed F`: playback speed factor (default 1.0, 0 = no delay)
- `--resolve FILE`: keep only the last measurement per `stamp`, write a new file and exit
- `--resolve-output FILE`: output file for `--resolve` (default `<input>_resolved.jsonl`)
- `--max-rate HZ`: max. single measurements per second and client (default 10, 0 = unlimited)
- `--max-batch-rate HZ`: max. batches per second and client (default 0.1, 0 = unlimited)
- `-v`: debug output

`--append`, `--clear`, `--play` and `--resolve` are mutually exclusive.

### Files

- Current file: `<data-dir>/<name>.jsonl`, one measurement per line (JSON Lines).
- On start without `--append`, an existing file is renamed to
  `<name>_<YYYYmmdd_HHMMSS>.jsonl` (timestamp = modification time of the file).
- `--clear` also deletes these renamed files.
- `--resolve` only compares the field `stamp`. The key can be extended with
  `RESOLVE_KEY_FIELDS` in `storage.py`, e.g. by `unique_serial_id` and `sensor`.

## Sending measurements

Required fields:

- `stamp`: time in seconds (decimals = milliseconds)
- `x`, `y`: position (e.g. UTM)
- `yaw`: heading in rad (east = 0, as in ROS)
- `sensor`: sensor name
- `value`: measured value (number)

Optional: `frame_id`, `z`, `pitch`, `roll`, `utm_zone_number`, `utm_zone_letter`,
`unique_serial_id`, `manufacturer_device_name`, `device_classification`, `source_type`, `unit`, `client`.

Single measurement:

    curl -X POST "http://localhost:8080/?frame_id=world&stamp=25.500&x=366188&y=5609559&z=255&yaw=0.0&pitch=0&roll=0&utm_zone_number=32&utm_zone_letter=U&client=my-system&unique_serial_id=T-4000&manufacturer_device_name=Terminator%20Modell%204000&device_classification=T&sensor=threat-level&source_type=visual&unit=of%2010&value=6"

Several measurements in one call (JSON array or `{"measurements": [...]}`):

    curl -X POST http://localhost:8080/batch -H "Content-Type: application/json" \
      -d '[{"stamp":25.5,"x":1,"y":2,"yaw":0,"sensor":"a","value":1},{"stamp":26.5,"x":1,"y":2,"yaw":0,"sensor":"a","value":0}]'

For batches: if one measurement is invalid, the whole batch is rejected and nothing is stored.

Python and C++ examples can be found [here](./examples/readme.md).

### Responses

- `201`: stored (`accepted`, `first_seq`, `last_seq`)
- `400`: required fields missing or invalid values (the text names the fields, batches include the index)
- `409`: server runs in play mode
- `413`: body or batch too large (max. 10 MB or 10000 measurements)
- `429`: rate limit exceeded (header `Retry-After`)
- `500`: file not writable

### Rate limits (per client IP)

- Single measurements: at most 10 Hz
- Batches (more than 1 measurement): at most one batch every 10 s (0.1 Hz)
- Both counters are separate. Rejected requests are not counted.
- A tolerance of 5 % against timing jitter applies.

## Retrieving data (GUI)

- `GET /measurements?since=N&limit=M`: existing measurements (`since` = last known `seq`)
- `GET /events[?since=N]`: live stream (SSE). With `since`, missed measurements are sent
  first, then live data. `EventSource` resumes after a connection loss via `Last-Event-ID`.
- `GET /info`: status (mode, file, count, clients, limits)

Example:

    const base = "http://localhost:8080";
    const { measurements, last_seq } = await (await fetch(`${base}/measurements`)).json();
    measurements.forEach(show);
    const es = new EventSource(`${base}/events?since=${last_seq}`);
    es.onmessage = (ev) => show(JSON.parse(ev.data));

## ROS 2 bridge

    ros2 run fkie_measurement_server measurement_ros_bridge \
      --ros-args -p server_host:=127.0.0.1 -p server_port:=8080 -p global_frame:=map

Subscribed topics:

- `topic_sub_measurement_array` (`MeasurementArray`, default `measurement_array`)
- `topic_sub_measurement` (`Measurement`, default `/in_measurement`, empty = disabled)
- `topic_sub_measurement_located` (`MeasurementLocated`, default `/in_measurement_located`, empty = disabled)

Parameters:

- `server_host` (default `127.0.0.1`),
- `server_port` (default `8080`)
- `client` (default ``),
- `global_frame` (default `map`): target frame for all poses, empty = TF disabled
- `tf_fallback_latest` (default `true`): use the latest transform if none exists for the message stamp
- `utm_zone_number` (default 32), `utm_zone_letter` (default `U`): fallback if the message has no zone
- `ignore_full_history` (default `true`): ignore messages with `full_history`
- `queue_size` (default 10000): send queue, on overflow measurements are dropped and logged
- `max_batch_size` (default 1000)
- `max_retries` (default 3), `request_timeout` (default 5.0 s)
- `min_single_interval` (default 0.11 s), `min_batch_interval` (default 10.5 s): keep the server limits

### TF handling

- Located measurements: the pose is transformed from its frame (`pose.header.frame_id`, otherwise
  `measurement.header.frame_id`) into `global_frame`. Position and orientation are transformed.
- Measurements without pose: the position is the origin of the sensor frame
  (`header.frame_id`) in `global_frame`. Without `frame_id` they are dropped.
- A pose without frame, or already in `global_frame`, is sent unchanged.
- The lookup uses the message stamp first. If that fails and `tf_fallback_latest` is true, the
  latest transform is used. If there is still no transform, the measurement is dropped and a
  warning is logged (throttled).
- Lookups never block. Measurements that arrive before the first TF data are dropped.
- The server receives `x`, `y`, `z` in `global_frame`, so `global_frame` should be a UTM-based frame.
- Orientation is converted from the quaternion to roll, pitch and yaw.
- One measurement is sent per `MeasurementValue`.

## Launch files

The XML start the server and the bridge:

    ros2 launch fkie_measurement_server measurement_server.launch.xml port:=8080 name:=test

    # append to the last file
    ros2 launch fkie_measurement_server measurement_server.launch.xml server_args:="--append"

    # play mode without bridge
    ros2 launch fkie_measurement_server measurement_server.launch.xml \
      server_args:="--play measurement_data/test.jsonl" start_bridge:=false

The server is started as a normal process with `python3 -m fkie_measurement_server`.
The workspace must be sourced for this (or the package installed with `pip`).

## Development

    pip install pre-commit
    pre-commit install
    pre-commit run --all-files

- Type checking with basedpyright (configuration in `pyproject.toml`)
- Update the versions in `.pre-commit-config.yaml` with `pre-commit autoupdate`

## Security and limits

- The server has no authentication. Use `--host 127.0.0.1` for local operation.
- The whole history is kept in memory.
- Several senders behind the same IP share the rate limit.

## License

MIT
