"""File handling, in-memory history and live subscriptions (no ROS dependency)."""

import contextlib
import json
import logging
import os
import queue
import re
import threading
from datetime import datetime
from pathlib import Path
from typing import Any

from .measurement import ValidationError, validate_measurement

logger = logging.getLogger("fkie_measurement_server")

# Measurements with an identical key are reduced to the last one by --resolve.
# Resolve also includes the client name, or client_id if the name is absent.
# Change this tuple if the key should be e.g. ("stamp", "unique_serial_id", "sensor").
RESOLVE_KEY_FIELDS = ("stamp",)


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    """Read a JSON Lines file. Broken lines are skipped and counted."""
    records: list[dict[str, Any]] = []
    skipped = 0
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line_no, line in enumerate(handle, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                record = json.loads(text)
            except json.JSONDecodeError:
                skipped += 1
                logger.warning("%s:%d: invalid JSON, line skipped", path, line_no)
                continue
            if not isinstance(record, dict):
                skipped += 1
                logger.warning("%s:%d: not a JSON object, line skipped", path, line_no)
                continue
            records.append(record)  # pyright: ignore[reportUnknownArgumentType]
    if skipped:
        logger.warning("%s: %d line(s) skipped", path, skipped)
    return records


def load_measurements(path: Path) -> list[dict[str, Any]]:
    """Read and validate all measurements of a file. Invalid ones are skipped."""
    result: list[dict[str, Any]] = []
    for index, record in enumerate(read_jsonl(path), start=1):
        try:
            result.append(validate_measurement(record))
        except ValidationError as exc:
            logger.warning("%s: record %d invalid (%s), skipped", path, index, exc)
    return result


def rotate_existing(path: Path) -> Path | None:
    """Rename an existing non-empty file to <stem>_<mtime>.<suffix>."""
    if not path.exists() or path.stat().st_size == 0:
        return None
    stamp = datetime.fromtimestamp(path.stat().st_mtime).strftime("%Y%m%d_%H%M%S")
    target = path.with_name(f"{path.stem}_{stamp}{path.suffix}")
    counter = 1
    while target.exists():
        target = path.with_name(f"{path.stem}_{stamp}_{counter}{path.suffix}")
        counter += 1
    path.rename(target)
    logger.info("Existing file renamed to %s", target)
    return target


def clear_files(data_dir: Path, name: str) -> int:
    """Delete the current file and all archived files of the given name."""
    pattern = re.compile(rf"^{re.escape(name)}(_\d{{8}}_\d{{6}}(_\d+)?)?\.jsonl$")
    removed = 0
    if not data_dir.is_dir():
        return 0
    for entry in data_dir.iterdir():
        if entry.is_file() and pattern.match(entry.name):
            try:
                entry.unlink()
                removed += 1
            except OSError as exc:
                logger.error("Cannot delete %s: %s", entry, exc)
    return removed


def _resolve_key(record: dict[str, Any]) -> tuple[Any, ...] | None:
    values: list[Any] = []
    for field in RESOLVE_KEY_FIELDS:
        value = record.get(field)
        if value is None or value == "" or isinstance(value, (list, dict)):
            return None
        try:
            value = float(value)
        except (TypeError, ValueError):
            pass
        values.append(value)

    raw_client = record.get("client")
    client = "" if raw_client is None else str(raw_client).strip()

    if not client:
        raw_client_id = record.get("client_id")
        client = "" if raw_client_id is None else str(raw_client_id).strip()

    values.append(client)
    return tuple(values)


def resolve_file(src: Path, dst: Path) -> tuple[int, int, int]:
    """Keep only the last record per key and write the result to dst.

    Returns (records_read, records_kept, records_skipped).
    """
    if src.resolve() == dst.resolve():
        raise ValueError("Output file must differ from the input file")

    records = read_jsonl(src)
    latest: dict[tuple[Any, ...], dict[str, Any]] = {}
    skipped = 0
    for record in records:
        key = _resolve_key(record)
        if key is None:
            skipped += 1
            continue
        latest.pop(key, None)  # re-insert so that the order follows the last occurrence
        latest[key] = record

    dst.parent.mkdir(parents=True, exist_ok=True)
    rotate_existing(dst)
    tmp = dst.with_name(dst.name + ".tmp")
    try:
        with open(tmp, "w", encoding="utf-8") as handle:
            for record in latest.values():
                handle.write(
                    json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n"
                )
        os.replace(tmp, dst)
    except OSError:
        with contextlib.suppress(OSError):
            tmp.unlink()
        raise
    return len(records), len(latest), skipped


class Subscription:
    """A live subscriber (e.g. one SSE connection)."""

    def __init__(self, maxsize: int) -> None:
        self.inbox: queue.Queue[dict[str, Any] | None] = queue.Queue(maxsize=maxsize)
        self.closed = False


class MeasurementStore:
    """Thread-safe history with optional file persistence and live subscribers."""

    def __init__(
        self, file_path: Path | None = None, subscriber_queue_size: int = 1000
    ) -> None:
        self._file_path = file_path
        self._lock = threading.Lock()
        self._items: list[dict[str, Any]] = []
        self._subscribers: set[Subscription] = set()
        self._queue_size = subscriber_queue_size

    def open(self) -> None:
        """Create the output file if missing and check that it is writable (no-op without file)."""
        if self._file_path is not None:
            with open(self._file_path, "a", encoding="utf-8"):
                pass

    def preload(self, measurements: list[dict[str, Any]]) -> None:
        """Add already persisted measurements to memory without writing them again."""
        with self._lock:
            for measurement in measurements:
                item = dict(measurement)
                item["seq"] = len(self._items) + 1
                self._items.append(item)

    def add_many(self, measurements: list[dict[str, Any]]) -> list[dict[str, Any]]:
        """Persist (if enabled), store and broadcast measurements.

        Raises OSError if the file cannot be written; nothing is stored then.
        """
        with self._lock:
            if self._file_path is not None:
                payload = "".join(
                    json.dumps(m, ensure_ascii=False, separators=(",", ":")) + "\n"
                    for m in measurements
                )
                # Open per write: the file is always closed and flushed afterwards
                with open(self._file_path, "a", encoding="utf-8") as handle:
                    _ = handle.write(payload)

            stored: list[dict[str, Any]] = []
            for measurement in measurements:
                item = dict(measurement)
                item["seq"] = len(self._items) + 1
                self._items.append(item)
                stored.append(item)

            for sub in list(self._subscribers):
                try:
                    for item in stored:
                        sub.inbox.put_nowait(item)
                except queue.Full:
                    # The client can reconnect and resume via Last-Event-ID
                    sub.closed = True
                    self._subscribers.discard(sub)
                    logger.warning("Slow subscriber dropped")
            return stored

    def get(
        self, since: int = 0, limit: int | None = None
    ) -> tuple[list[dict[str, Any]], int]:
        """Return measurements with seq > since and the total count."""
        with self._lock:
            start = max(0, since)
            end = None if limit is None else start + limit
            return self._items[start:end], len(self._items)

    def subscribe(
        self, since: int | None = None
    ) -> tuple[Subscription, list[dict[str, Any]]]:
        """Register a subscriber; backlog contains items with seq > since (if given)."""
        sub = Subscription(self._queue_size)
        with self._lock:
            backlog = [] if since is None else self._items[max(0, since) :]
            self._subscribers.add(sub)
        return sub, backlog

    def unsubscribe(self, sub: Subscription) -> None:
        with self._lock:
            self._subscribers.discard(sub)

    @property
    def count(self) -> int:
        with self._lock:
            return len(self._items)

    @property
    def subscriber_count(self) -> int:
        with self._lock:
            return len(self._subscribers)

    def close(self) -> None:
        """Wake up all subscribers (the file is closed after every write)."""
        with self._lock:
            for sub in self._subscribers:
                sub.closed = True
                with contextlib.suppress(queue.Full):
                    sub.inbox.put_nowait(None)
            self._subscribers.clear()
