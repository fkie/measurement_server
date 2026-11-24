"""Validation and normalization of measurements (no ROS dependency)."""

import math
from collections.abc import Mapping
from typing import Any

REQUIRED_FIELDS = ("stamp", "x", "y", "yaw", "sensor", "value")
_OPTIONAL_FLOATS = ("z", "pitch", "roll")
_OPTIONAL_STRINGS = ("unique_serial_id", "device_classification", "source_type", "unit")


class ValidationError(ValueError):
    """Raised when a measurement is incomplete or contains invalid values."""

    def __init__(self, errors: list[str]) -> None:
        super().__init__("; ".join(errors))
        self.errors = list(errors)


def _is_blank(value: Any) -> bool:
    return value is None or (isinstance(value, str) and value.strip() == "")


def _text(value: Any) -> str:
    return "" if value is None else str(value).strip()


def _parse_float(name: str, value: Any, errors: list[str]) -> float | None:
    """Convert value to a finite float or append an error message."""
    if isinstance(value, bool):
        errors.append(f"Field '{name}' must be a number, got a boolean")
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        errors.append(f"Field '{name}' must be a number, got {value!r}")
        return None
    if not math.isfinite(number):
        errors.append(f"Field '{name}' must be a finite number, got {value!r}")
        return None
    return number


def validate_measurement(raw: Any) -> dict[str, Any]:
    """Validate a raw measurement and return a normalized dict.

    Required: stamp (s), x, y, yaw (rad), sensor, value.
    Raises ValidationError with all found problems.
    """
    if not isinstance(raw, Mapping):
        raise ValidationError(["Measurement must be a key/value object"])

    errors: list[str] = []
    missing = [name for name in REQUIRED_FIELDS if _is_blank(raw.get(name))]
    if missing:
        errors.append("Missing required field(s): " + ", ".join(missing))

    numbers: dict[str, float] = {}
    for name in ("stamp", "x", "y", "yaw", "value"):
        if name in missing:
            continue
        number = _parse_float(name, raw[name], errors)
        if number is not None:
            numbers[name] = number
    if "stamp" in numbers and numbers["stamp"] < 0:
        errors.append("Field 'stamp' must not be negative")

    for name in _OPTIONAL_FLOATS:
        if _is_blank(raw.get(name)):
            numbers[name] = 0.0
        else:
            number = _parse_float(name, raw[name], errors)
            if number is not None:
                numbers[name] = number

    zone_number: int | None = None
    if not _is_blank(raw.get("utm_zone_number")):
        zone_value = _parse_float("utm_zone_number", raw["utm_zone_number"], errors)
        if zone_value is not None:
            if zone_value.is_integer() and 1 <= zone_value <= 60:
                zone_number = int(zone_value)
            else:
                errors.append(
                    "Field 'utm_zone_number' must be an integer between 1 and 60"
                )

    zone_letter = _text(raw.get("utm_zone_letter")).upper()
    if zone_letter and not (len(zone_letter) == 1 and zone_letter.isalpha()):
        errors.append("Field 'utm_zone_letter' must be a single letter")

    if errors:
        raise ValidationError(errors)

    strings = {name: _text(raw.get(name)) for name in _OPTIONAL_STRINGS}
    device_name = (
        _text(raw.get("manufacturer_device_name")) or strings["unique_serial_id"]
    )

    # Fixed key order for stable file output
    return {
        "stamp": numbers["stamp"],
        "frame_id": _text(raw.get("frame_id")),
        "x": numbers["x"],
        "y": numbers["y"],
        "z": numbers["z"],
        "yaw": numbers["yaw"],
        "pitch": numbers["pitch"],
        "roll": numbers["roll"],
        "utm_zone_number": zone_number,
        "utm_zone_letter": zone_letter,
        "unique_serial_id": strings["unique_serial_id"],
        "manufacturer_device_name": device_name,
        "device_classification": strings["device_classification"],
        "sensor": _text(raw["sensor"]),
        "source_type": strings["source_type"],
        "unit": strings["unit"],
        "value": numbers["value"],
    }
