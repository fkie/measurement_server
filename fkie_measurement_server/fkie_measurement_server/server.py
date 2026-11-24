"""ROS-independent measurement web server (HTTP + Server-Sent Events)."""

import argparse
import json
import logging
import math
import queue
import re
import signal
import sys
import threading
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from socketserver import BaseRequestHandler
from types import FrameType
from typing import Any, cast
from urllib.parse import parse_qsl, urlsplit

from . import __version__
from .measurement import ValidationError, validate_measurement
from .ratelimit import (
    DEFAULT_MAX_BATCH_RATE,
    DEFAULT_MAX_RATE,
    RateLimiter,
    RateLimitExceeded,
)
from .storage import (
    MeasurementStore,
    clear_files,
    load_measurements,
    resolve_file,
    rotate_existing,
)

logger = logging.getLogger("fkie_measurement_server")

DEFAULT_PORT = 8080
MAX_BODY_BYTES = 10 * 1024 * 1024
MAX_BATCH_SIZE = 10000
KEEPALIVE_SECONDS = 15.0
POST_PATHS = ("/", "/measurements", "/batch")
NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")


class HttpError(Exception):
    """Error that is reported to the client as JSON."""

    def __init__(
        self, status: int, message: str, retry_after: float | None = None
    ) -> None:
        super().__init__(message)
        self.status = status
        self.message = message
        self.retry_after = retry_after


class MeasurementHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    request_queue_size = 64

    def __init__(
        self,
        address: tuple[str, int],
        handler: Callable[..., BaseRequestHandler],
        store: MeasurementStore,
        limiter: RateLimiter,
        name: str,
        read_only: bool,
        file_path: Path | None,
    ) -> None:
        super().__init__(address, handler)
        self.store = store
        self.limiter = limiter
        self.name = name
        self.read_only = read_only
        self.file_path = file_path

    def info(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "version": __version__,
            "mode": "play" if self.read_only else "record",
            "file": str(self.file_path) if self.file_path else None,
            "count": self.store.count,
            "clients": self.store.subscriber_count,
            "max_rate_hz": self.limiter.max_rate,
            "max_batch_rate_hz": self.limiter.max_batch_rate,
            "rate_limited_requests": self.limiter.rejected,
        }


class RequestHandler(BaseHTTPRequestHandler):
    server_version = "FkieMeasurementServer/" + __version__
    timeout = 30

    @property
    def app(self) -> MeasurementHTTPServer:
        """The typed server instance."""
        return cast(MeasurementHTTPServer, self.server)

    def log_message(self, format: str, *args: Any) -> None:
        logger.debug("%s - %s", self.address_string(), format % args)

    # ---------------------------------------------------------------- helpers
    def _send_cors_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, Last-Event-ID")
        self.send_header("Access-Control-Expose-Headers", "Retry-After")

    def _send_json(
        self, status: int, payload: Any, headers: dict[str, str] | None = None
    ) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        try:
            self.send_response(status)
            self._send_cors_headers()
            for key, value in (headers or {}).items():
                self.send_header(key, value)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            logger.debug("Client closed the connection before the response was sent")

    def _send_error_json(
        self, status: int, message: str, retry_after: float | None = None
    ) -> None:
        payload: dict[str, Any] = {"status": "error", "error": message}
        headers: dict[str, str] | None = None
        if retry_after is not None:
            payload["retry_after"] = round(retry_after, 3)
            headers = {"Retry-After": str(max(1, math.ceil(retry_after)))}
        self._send_json(status, payload, headers)

    @staticmethod
    def _int_param(
        query: dict[str, str], name: str, default: int | None, minimum: int = 0
    ) -> int | None:
        raw = query.get(name)
        if raw is None or raw == "":
            return default
        try:
            value = int(raw)
        except ValueError:
            raise HttpError(400, f"Parameter '{name}' must be an integer") from None
        if value < minimum:
            raise HttpError(400, f"Parameter '{name}' must be >= {minimum}")
        return value

    # ---------------------------------------------------------------- methods
    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self._send_cors_headers()
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self) -> None:
        try:
            parts = urlsplit(self.path)
            path = parts.path.rstrip("/") or "/"
            query = dict(parse_qsl(parts.query, keep_blank_values=True))
            if path in ("/", "/info"):
                self._send_json(200, self.app.info())
            elif path == "/measurements":
                self._handle_history(query)
            elif path == "/events":
                self._handle_events(query)
            else:
                raise HttpError(404, f"Unknown path '{path}'")
        except HttpError as exc:
            self._send_error_json(exc.status, exc.message, exc.retry_after)
        except Exception:  # pylint: disable=broad-except
            logger.exception("Unhandled error in GET %s", self.path)
            self._send_error_json(500, "Internal server error")

    def do_POST(self) -> None:
        try:
            parts = urlsplit(self.path)
            path = parts.path.rstrip("/") or "/"
            if path not in POST_PATHS:
                raise HttpError(404, f"Unknown path '{path}'")
            app = self.app
            if app.read_only:
                raise HttpError(
                    409, "Server is in play mode, new measurements are not accepted"
                )

            body = self._read_body()
            query = dict(parse_qsl(parts.query, keep_blank_values=True))
            records = self._collect_records(
                query, body, self.headers.get("Content-Type", "")
            )

            validated: list[dict[str, Any]] = []
            problems: list[dict[str, Any]] = []
            for index, record in enumerate(records):
                try:
                    validated.append(validate_measurement(record))
                except ValidationError as exc:
                    problems.append({"index": index, "errors": exc.errors})
            if problems:
                if len(records) == 1:
                    message = "; ".join(problems[0]["errors"])
                else:
                    message = f"{len(problems)} of {len(records)} measurements are invalid (nothing stored)"
                logger.warning(
                    "Rejected POST from %s: %s", self.client_address[0], message
                )
                self._send_json(
                    400,
                    {"status": "error", "error": message, "details": problems[:100]},
                )
                return

            # More than one measurement in a request counts as a batch
            try:
                app.limiter.check(str(self.client_address[0]), len(validated))
            except RateLimitExceeded as exc:
                logger.debug("Rate limit hit by %s: %s", self.client_address[0], exc)
                raise HttpError(429, str(exc), exc.retry_after) from None

            try:
                stored = app.store.add_many(validated)
            except OSError as exc:
                logger.error("Cannot write measurement file: %s", exc)
                raise HttpError(500, "Could not write measurement file") from exc

            logger.info(
                "Accepted %d measurement(s) from %s",
                len(stored),
                self.client_address[0],
            )
            self._send_json(
                201,
                {
                    "status": "ok",
                    "accepted": len(stored),
                    "first_seq": stored[0]["seq"],
                    "last_seq": stored[-1]["seq"],
                },
            )
        except HttpError as exc:
            self._send_error_json(exc.status, exc.message, exc.retry_after)
        except Exception:  # pylint: disable=broad-except
            logger.exception("Unhandled error in POST %s", self.path)
            self._send_error_json(500, "Internal server error")

    # ------------------------------------------------------------ POST parsing
    def _read_body(self) -> bytes:
        raw_length = self.headers.get("Content-Length")
        if raw_length is None:
            if "chunked" in self.headers.get("Transfer-Encoding", "").lower():
                raise HttpError(
                    411, "Chunked bodies are not supported, send Content-Length"
                )
            return b""
        try:
            length = int(raw_length)
        except ValueError:
            raise HttpError(400, "Invalid Content-Length") from None
        if length < 0:
            raise HttpError(400, "Invalid Content-Length")
        if length > MAX_BODY_BYTES:
            raise HttpError(413, f"Body too large (max {MAX_BODY_BYTES} bytes)")
        body = self.rfile.read(length)
        if len(body) < length:
            raise HttpError(400, "Incomplete request body")
        return body

    @staticmethod
    def _collect_records(
        query: dict[str, str], body: bytes, content_type: str
    ) -> list[Any]:
        """Build the list of raw records from query string and body.

        Supported: query only, form body, JSON object, JSON array or
        {"measurements": [...]}. Query parameters act as defaults.
        """
        if not body.strip():
            return [dict(query)]

        ctype = content_type.split(";")[0].strip().lower()
        looks_like_json = body.lstrip()[:1] in (b"{", b"[")
        if ctype.endswith("json") or (looks_like_json and "form" not in ctype):
            try:
                data: Any = json.loads(body.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise HttpError(400, f"Invalid JSON body: {exc}") from None
            if isinstance(data, dict) and isinstance(data.get("measurements"), list):  # pyright: ignore[reportUnknownMemberType]
                data = data["measurements"]
            items: list[Any] = data if isinstance(data, list) else [data]  # pyright: ignore[reportUnknownVariableType]
            if not items:
                raise HttpError(400, "Empty batch")
            if len(items) > MAX_BATCH_SIZE:
                raise HttpError(413, f"Too many measurements (max {MAX_BATCH_SIZE})")
            return [
                {**query, **item} if isinstance(item, dict) else item for item in items
            ]  # pyright: ignore[reportUnknownVariableType]

        try:
            text = body.decode("utf-8")
        except UnicodeDecodeError:
            raise HttpError(400, "Body is not valid UTF-8") from None
        return [{**query, **dict(parse_qsl(text, keep_blank_values=True))}]

    # ------------------------------------------------------------- GET handlers
    def _handle_history(self, query: dict[str, str]) -> None:
        since = self._int_param(query, "since", 0) or 0
        limit = self._int_param(query, "limit", None, minimum=1)
        items, total = self.app.store.get(since, limit)
        self._send_json(
            200, {"count": len(items), "last_seq": total, "measurements": items}
        )

    def _handle_events(self, query: dict[str, str]) -> None:
        """Server-Sent Events stream. Without 'since' only new measurements are sent."""
        since_raw = query.get("since") or self.headers.get("Last-Event-ID")
        since: int | None = None
        if since_raw:
            try:
                since = int(since_raw)
            except ValueError:
                raise HttpError(400, "Parameter 'since' must be an integer") from None
            if since < 0:
                raise HttpError(400, "Parameter 'since' must be >= 0")

        store = self.app.store
        sub, backlog = store.subscribe(since)
        logger.info("Event client connected: %s", self.client_address[0])
        try:
            self.send_response(200)
            self._send_cors_headers()
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(b"retry: 3000\n\n")
            for old_item in backlog:
                self._write_event(old_item)
            while True:
                try:
                    item = sub.inbox.get(timeout=KEEPALIVE_SECONDS)
                except queue.Empty:
                    if sub.closed:
                        break
                    self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
                    continue
                if item is None:
                    break
                self._write_event(item)
        except OSError:
            pass  # client disconnected
        finally:
            store.unsubscribe(sub)
            logger.info("Event client disconnected: %s", self.client_address[0])

    def _write_event(self, item: dict[str, Any]) -> None:
        data = json.dumps(item, ensure_ascii=False, separators=(",", ":"))
        self.wfile.write(f"id: {item['seq']}\ndata: {data}\n\n".encode())
        self.wfile.flush()


# ------------------------------------------------------------------------ CLI
def _port_type(text: str) -> int:
    try:
        value = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError("port must be an integer") from None
    if not 1 <= value <= 65535:
        raise argparse.ArgumentTypeError("port must be between 1 and 65535")
    return value


def _name_type(text: str) -> str:
    if not NAME_PATTERN.match(text):
        raise argparse.ArgumentTypeError(
            "name may only contain letters, digits, '.', '_' and '-' and must start with a letter or digit"
        )
    return text


def _non_negative_float(text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError("value must be a number") from None
    if not math.isfinite(value) or value < 0:
        raise argparse.ArgumentTypeError("value must be a finite number >= 0")
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="measurement_server",
        description="Measurement web server (HTTP POST in, file storage, live events out).",
    )
    _ = parser.add_argument(
        "--port",
        type=_port_type,
        default=DEFAULT_PORT,
        help=f"port to listen on (default {DEFAULT_PORT})",
    )
    _ = parser.add_argument(
        "--host", default="0.0.0.0", help="address to bind (default 0.0.0.0)"
    )
    _ = parser.add_argument(
        "--name",
        type=_name_type,
        default="measurements",
        help="name of the measurement file (default 'measurements')",
    )
    _ = parser.add_argument(
        "--data-dir",
        default="measurement_data",
        help="directory for measurement files (default ./measurement_data)",
    )
    mode = parser.add_mutually_exclusive_group()
    _ = mode.add_argument(
        "--append",
        action="store_true",
        help="append to the last file instead of renaming it",
    )
    _ = mode.add_argument(
        "--clear",
        action="store_true",
        help="delete all files of this name and start fresh",
    )
    _ = mode.add_argument(
        "--play",
        metavar="FILE",
        help="replay measurements from FILE (nothing is stored)",
    )
    _ = mode.add_argument(
        "--resolve",
        metavar="FILE",
        help="keep only the last measurement per stamp, write a new file and exit",
    )
    _ = parser.add_argument(
        "--play-speed",
        type=_non_negative_float,
        default=1.0,
        help="playback speed factor (default 1.0, 0 = no delay)",
    )
    _ = parser.add_argument(
        "--resolve-output",
        metavar="FILE",
        help="output file for --resolve (default <input>_resolved.jsonl)",
    )
    _ = parser.add_argument(
        "--max-rate",
        type=_non_negative_float,
        default=DEFAULT_MAX_RATE,
        help=f"max. single measurements per second and client (default {DEFAULT_MAX_RATE:g}, 0 = unlimited)",
    )
    _ = parser.add_argument(
        "--max-batch-rate",
        type=_non_negative_float,
        default=DEFAULT_MAX_BATCH_RATE,
        help=f"max. batches (>1 measurement) per second and client (default {DEFAULT_MAX_BATCH_RATE:g}, 0 = unlimited)",
    )
    _ = parser.add_argument(
        "-v", "--verbose", action="store_true", help="enable debug logging"
    )
    return parser


def _playback(
    store: MeasurementStore,
    records: list[dict[str, Any]],
    speed: float,
    stop_event: threading.Event,
) -> None:
    """Replay records with the original timing (scaled by speed)."""
    logger.info("Playback started (%d measurements)", len(records))
    previous: float | None = None
    for record in records:
        if previous is not None and speed > 0:
            delay = max(0.0, (record["stamp"] - previous) / speed)
            if stop_event.wait(delay):
                return
        elif stop_event.is_set():
            return
        previous = record["stamp"]
        _ = store.add_many([record])
    logger.info("Playback finished")


def _run_resolve(args: argparse.Namespace) -> int:
    src = Path(str(args.resolve)).expanduser()
    if args.resolve_output:
        dst = Path(str(args.resolve_output)).expanduser()
    else:
        dst = src.with_name(f"{src.stem}_resolved{src.suffix or '.jsonl'}")
    try:
        total, kept, skipped = resolve_file(src, dst)
    except FileNotFoundError:
        logger.error("Input file not found: %s", src)
        return 1
    except (OSError, ValueError) as exc:
        logger.error("Resolve failed: %s", exc)
        return 1
    logger.info(
        "Resolved %s -> %s: %d read, %d kept, %d skipped",
        src,
        dst,
        total,
        kept,
        skipped,
    )
    return 0


def _run_server(args: argparse.Namespace) -> int:
    file_path: Path | None = None
    play_records: list[dict[str, Any]] = []
    name = str(args.name)

    try:
        if args.play:
            play_path = Path(str(args.play)).expanduser()
            play_records = load_measurements(play_path)
            if not play_records:
                logger.warning("No valid measurements found in %s", play_path)
            store = MeasurementStore(None)
        else:
            data_dir = Path(str(args.data_dir)).expanduser()
            data_dir.mkdir(parents=True, exist_ok=True)
            file_path = data_dir / f"{name}.jsonl"
            existing: list[dict[str, Any]] = []
            if args.clear:
                logger.warning(
                    "Deleted %d file(s) of '%s'", clear_files(data_dir, name), name
                )
            elif args.append:
                if file_path.exists():
                    existing = load_measurements(file_path)
                    logger.info(
                        "Appending to %s (%d existing measurements)",
                        file_path,
                        len(existing),
                    )
            else:
                _ = rotate_existing(file_path)
            store = MeasurementStore(file_path)
            store.preload(existing)
            store.open()
    except OSError as exc:
        logger.error("File error: %s", exc)
        return 1

    limiter = RateLimiter(float(args.max_rate), float(args.max_batch_rate))
    host = str(args.host)
    port = int(args.port)
    try:
        httpd = MeasurementHTTPServer(
            (host, port),
            RequestHandler,
            store=store,
            limiter=limiter,
            name=name,
            read_only=bool(args.play),
            file_path=file_path,
        )
    except OSError as exc:
        logger.error("Cannot listen on %s:%d: %s", host, port, exc)
        store.close()
        return 1

    stop_event = threading.Event()
    if args.play:
        threading.Thread(
            target=_playback,
            args=(store, play_records, float(args.play_speed), stop_event),
            daemon=True,
            name="playback",
        ).start()

    logger.info(
        "Server '%s' listening on %s:%d (%s mode, file: %s)",
        name,
        host,
        port,
        "play" if args.play else "record",
        file_path,
    )
    logger.info(
        "Rate limits per client: %g Hz single, %g Hz batch (0 = unlimited)",
        limiter.max_rate,
        limiter.max_batch_rate,
    )
    if host == "0.0.0.0":
        logger.warning("Listening on all interfaces, there is no authentication")

    shutdown_requested = threading.Event()

    def request_shutdown(_signum: int, _frame: FrameType | None) -> None:
        """Stop serve_forever() without raising. Repeated signals are ignored."""
        if shutdown_requested.is_set():
            return  # e.g. "ros2 run" forwards SIGINT in addition to the terminal
        shutdown_requested.set()
        # shutdown() blocks until serve_forever() returns, so it must not run in this thread
        threading.Thread(target=httpd.shutdown, daemon=True, name="shutdown").start()

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            _ = signal.signal(sig, request_shutdown)
    except ValueError:
        logger.debug("Signal handlers not installed (not in main thread)")

    try:
        httpd.serve_forever()
    finally:
        logger.info("Shutting down...")
        stop_event.set()
        store.close()
        httpd.server_close()
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.resolve_output and not args.resolve:
        parser.error("--resolve-output requires --resolve")
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )
    if args.resolve:
        return _run_resolve(args)
    return _run_server(args)


if __name__ == "__main__":
    sys.exit(main())
