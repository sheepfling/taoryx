"""Strict timestamped rotational truth replay for SWIL/HWIL integration."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import cast

import numpy as np

from .common import RuntimeState
from .truth import RotationTruth, RotationTruthProvider


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True, slots=True)
class RotationalReplayFrame:
    """One external rotational truth frame at an accepted timestamp."""

    truth: RotationTruth

    @classmethod
    def from_mapping(cls, value: Mapping[str, object], *, line_number: int | None = None) -> RotationalReplayFrame:
        location = "" if line_number is None else f" on line {line_number}"
        try:
            time_s = float(cast(float | int | str, value["time_s"]))
            orientation = np.asarray(value["orientation_eci_from_body"], dtype=float)
            angular_rate = np.asarray(value["angular_rate_body_radps"], dtype=float)
            angular_acceleration_value = value.get("angular_acceleration_body_radps2")
            angular_acceleration = None if angular_acceleration_value is None else np.asarray(angular_acceleration_value, dtype=float)
            return cls(RotationTruth(time_s, orientation, angular_rate, angular_acceleration))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"invalid rotational replay frame{location}: {error}") from error

    def as_dict(self) -> dict[str, object]:
        truth = self.truth
        return {
            "time_s": truth.time_s,
            "orientation_eci_from_body": truth.orientation_eci_from_body.tolist(),
            "angular_rate_body_radps": truth.angular_rate_body_radps.tolist(),
            "angular_acceleration_body_radps2": None
            if truth.angular_acceleration_body_radps2 is None
            else truth.angular_acceleration_body_radps2.tolist(),
        }


class RotationalTruthReplay:
    """Serve external rotation frames only at declared accepted timestamps."""

    def __init__(
        self,
        frames: tuple[RotationalReplayFrame, ...],
        *,
        source_path: Path | None = None,
        timestamp_tolerance_s: float = 1.0e-9,
    ) -> None:
        if not frames:
            raise ValueError("rotational replay requires at least one frame")
        if not math.isfinite(timestamp_tolerance_s) or timestamp_tolerance_s < 0.0:
            raise ValueError("rotational replay timestamp tolerance must be finite and nonnegative")
        times = tuple(frame.truth.time_s for frame in frames)
        if any(later <= earlier for earlier, later in zip(times, times[1:], strict=False)):
            raise ValueError("rotational replay frame timestamps must be strictly increasing")
        self.frames = frames
        self.source_path = None if source_path is None else source_path.resolve()
        self.timestamp_tolerance_s = timestamp_tolerance_s
        self._last_requested_time_s: float | None = None
        self.request_log: list[dict[str, object]] = []
        self.contract = {
            "mode": "external-rotational-replay",
            "source": "jsonl-replay",
            "timestamp_policy": "accepted-boundary-exact-with-tolerance",
            "timestamp_tolerance_s": timestamp_tolerance_s,
            "frame_count": len(frames),
            "source_path": None if self.source_path is None else str(self.source_path),
            "source_sha256": None if self.source_path is None else _sha256(self.source_path),
            "channel_sources": {"rotation": "external-replay", "translation": "vehicle-or-substituted"},
        }

    @classmethod
    def from_jsonl(cls, path: str | Path, *, timestamp_tolerance_s: float = 1.0e-9) -> RotationalTruthReplay:
        source = Path(path).resolve()
        frames: list[RotationalReplayFrame] = []
        for line_number, line in enumerate(source.read_text(encoding="utf-8").splitlines(), start=1):
            if not line.strip():
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"invalid rotational replay JSON on line {line_number}: {error}") from error
            if not isinstance(payload, Mapping):
                raise ValueError(f"rotational replay line {line_number} must be an object")
            frames.append(RotationalReplayFrame.from_mapping(payload, line_number=line_number))
        return cls(tuple(frames), source_path=source, timestamp_tolerance_s=timestamp_tolerance_s)

    def __call__(self, state: RuntimeState) -> RotationTruth:
        requested = float(state.time)
        if not math.isfinite(requested):
            raise ValueError("rotational replay requested time must be finite")
        if self._last_requested_time_s is not None and requested < self._last_requested_time_s - self.timestamp_tolerance_s:
            raise ValueError("rotational replay requests must be chronological")
        frame = min(self.frames, key=lambda candidate: abs(candidate.truth.time_s - requested))
        error_s = abs(frame.truth.time_s - requested)
        if error_s > self.timestamp_tolerance_s:
            self.request_log.append({"requested_time_s": requested, "status": "rejected", "error": f"no frame within {self.timestamp_tolerance_s:g}s"})
            raise ValueError(
                f"rotational replay has no frame for accepted time {requested:.12g}s; nearest frame is {frame.truth.time_s:.12g}s"
            )
        self._last_requested_time_s = requested
        self.request_log.append(
            {
                "requested_time_s": requested,
                "frame_time_s": frame.truth.time_s,
                "timestamp_error_s": frame.truth.time_s - requested,
                "status": "accepted",
            }
        )
        return frame.truth

    def write_request_log(self, path: str | Path) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text("".join(json.dumps(record, sort_keys=True) + "\n" for record in self.request_log), encoding="utf-8")
        return destination


class RotationalTruthRecorder:
    """Log every rotational provider request and validate returned timestamps."""

    def __init__(self, provider: RotationTruthProvider, *, source: str = "external-rotational-provider", timestamp_tolerance_s: float = 1.0e-9) -> None:
        if not source.strip():
            raise ValueError("rotational truth recorder source must not be empty")
        if not math.isfinite(timestamp_tolerance_s) or timestamp_tolerance_s < 0.0:
            raise ValueError("rotational truth recorder timestamp tolerance must be finite and nonnegative")
        self.provider = provider
        self.source = source
        self.timestamp_tolerance_s = timestamp_tolerance_s
        self.records: list[dict[str, object]] = []
        self.contract = {
            **dict(getattr(provider, "contract", {})),
            "source": source,
            "request_logging": "enabled",
            "timestamp_tolerance_s": timestamp_tolerance_s,
        }

    def __call__(self, state: RuntimeState) -> RotationTruth:
        requested = float(state.time)
        try:
            result = self.provider(state)
            error_s = float(result.time_s) - requested
            if not math.isfinite(error_s) or abs(error_s) > self.timestamp_tolerance_s:
                raise ValueError(
                    f"rotational provider returned time {result.time_s:.12g}s for request {requested:.12g}s"
                )
        except Exception as error:
            self.records.append({"requested_time_s": requested, "status": "rejected", "error": str(error)})
            raise
        self.records.append(
            {
                "requested_time_s": requested,
                "returned_time_s": result.time_s,
                "timestamp_error_s": error_s,
                "angular_rate_body_radps": result.angular_rate_body_radps.tolist(),
                "status": "accepted",
            }
        )
        return result

    def write_request_log(self, path: str | Path) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text("".join(json.dumps(record, sort_keys=True) + "\n" for record in self.records), encoding="utf-8")
        return destination
