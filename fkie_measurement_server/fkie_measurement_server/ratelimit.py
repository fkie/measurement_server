"""Per-client rate limiting for incoming measurements (no ROS dependency)."""

import threading
import time
from collections.abc import Callable

DEFAULT_MAX_RATE = 10.0  # single measurements per second
DEFAULT_MAX_BATCH_RATE = 0.1  # batches per second (one batch every 10 s)
DEFAULT_TOLERANCE = 0.05  # relative tolerance against timing jitter
_PRUNE_THRESHOLD = 1024


class RateLimitExceeded(Exception):
    """Raised when a client sends too fast."""

    def __init__(self, message: str, retry_after: float) -> None:
        super().__init__(message)
        self.retry_after = retry_after


def _min_interval(rate: float, tolerance: float) -> float:
    """Minimum accepted time between two requests (0 = unlimited)."""
    if rate <= 0.0:
        return 0.0
    return (1.0 / rate) * (1.0 - tolerance)


class RateLimiter:
    """Allows at most max_rate single requests/s and max_batch_rate batch requests/s per client.

    A request with more than one measurement is a batch. A rate of 0 disables the limit.
    Rejected requests are not counted, so a client can continue as soon as the interval is over.
    """

    def __init__(
        self,
        max_rate: float = DEFAULT_MAX_RATE,
        max_batch_rate: float = DEFAULT_MAX_BATCH_RATE,
        tolerance: float = DEFAULT_TOLERANCE,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not 0.0 <= tolerance < 1.0:
            raise ValueError("tolerance must be in [0, 1)")
        self._max_rate = max(0.0, max_rate)
        self._max_batch_rate = max(0.0, max_batch_rate)
        self._single_interval = _min_interval(self._max_rate, tolerance)
        self._batch_interval = _min_interval(self._max_batch_rate, tolerance)
        self._clock = clock
        self._lock = threading.Lock()
        self._last: dict[tuple[str, bool], float] = {}
        self._rejected = 0

    @property
    def max_rate(self) -> float:
        return self._max_rate

    @property
    def max_batch_rate(self) -> float:
        return self._max_batch_rate

    @property
    def rejected(self) -> int:
        with self._lock:
            return self._rejected

    def check(self, client: str, count: int) -> None:
        """Register a request with `count` measurements or raise RateLimitExceeded."""
        is_batch = count > 1
        interval = self._batch_interval if is_batch else self._single_interval
        if interval <= 0.0:
            return
        key = (client, is_batch)
        with self._lock:
            now = self._clock()
            last = self._last.get(key)
            if last is not None:
                remaining = interval - (now - last)
                if remaining > 0.0:
                    self._rejected += 1
                    raise RateLimitExceeded(self._message(is_batch), remaining)
            self._last[key] = now
            if len(self._last) > _PRUNE_THRESHOLD:
                self._prune(now)

    def _message(self, is_batch: bool) -> str:
        if is_batch:
            return (
                "Batch rate exceeded: at most one batch (more than one measurement) "
                f"every {1.0 / self._max_batch_rate:g} s is allowed"
            )
        return f"Message rate exceeded: at most {self._max_rate:g} Hz is allowed"

    def _prune(self, now: float) -> None:
        """Drop entries that cannot limit anything anymore (lock must be held)."""
        horizon = max(self._single_interval, self._batch_interval)
        stale = [key for key, last in self._last.items() if now - last >= horizon]
        for key in stale:
            del self._last[key]
