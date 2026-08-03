"""Declared sensor sampling from committed plant-truth boundaries only.

This small runtime is intentionally not a sensor-physics library.  It is the
shared episode contract for the first honest sensor path: selected canonical
truth channels are sampled only when the integrator has committed the exact
sensor boundary, then released after a declared latency.  The runtime never
interpolates a future state to fabricate a sensor sample.
"""

from __future__ import annotations

import hashlib
import math
from collections.abc import Mapping
from dataclasses import dataclass

_TIME_TOLERANCE_S = 1.0e-9


@dataclass(frozen=True, slots=True)
class BoundarySensorReading:
    """Latest externally visible sample from one declared sensor instance."""

    time_s: float
    sample_time_s: float | None
    values: Mapping[str, object | None]
    valid: Mapping[str, bool]


@dataclass(frozen=True, slots=True)
class ChannelMeasurementError:
    """One declared scalar measurement transform at capture time.

    This deliberately models only a portable measurement boundary: constant
    additive bias, independent deterministic Gaussian sample noise, and an
    optional nearest-increment quantizer.  It is not an IMU, a GPS receiver,
    an estimator, or a substitute for a family-specific sensor model.
    """

    bias: float = 0.0
    gaussian_stddev: float = 0.0
    quantization_step: float | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.bias):
            raise ValueError("sensor measurement bias must be finite")
        if not math.isfinite(self.gaussian_stddev) or self.gaussian_stddev < 0.0:
            raise ValueError("sensor Gaussian standard deviation must be finite and nonnegative")
        if self.quantization_step is not None and (
            not math.isfinite(self.quantization_step) or self.quantization_step <= 0.0
        ):
            raise ValueError("sensor quantization step must be positive and finite when declared")
        ####

    def as_dict(self) -> dict[str, float | None]:
        """Return the stable profile/checkpoint representation."""

        return {
            "bias": self.bias,
            "gaussian_stddev": self.gaussian_stddev,
            "quantization_step": self.quantization_step,
        }
        ####
    ####


class CommittedBoundarySensor:
    """Deterministic cadence/latency sampler for portable truth channels."""

    def __init__(
        self,
        *,
        profile_id: str,
        channel_ids: tuple[str, ...],
        cadence_s: float,
        latency_s: float,
        channel_errors: Mapping[str, ChannelMeasurementError | Mapping[str, object]] | None = None,
        seed: int = 0,
    ) -> None:
        if not profile_id.strip() or not channel_ids:
            raise ValueError("boundary sensor requires a profile ID and at least one channel")
        if len(channel_ids) != len(set(channel_ids)):
            raise ValueError("boundary sensor channel IDs must be unique")
        if not math.isfinite(cadence_s) or cadence_s <= 0.0:
            raise ValueError("boundary sensor cadence_s must be positive and finite")
        if not math.isfinite(latency_s) or latency_s < 0.0:
            raise ValueError("boundary sensor latency_s must be nonnegative and finite")
        self.profile_id = profile_id
        self.channel_ids = channel_ids
        self.cadence_s = cadence_s
        self.latency_s = latency_s
        self.channel_errors = _channel_errors(channel_ids, channel_errors)
        self.seed = _seed(seed)
        self.reset()
        ####

    def reset(self) -> None:
        """Return to an unsampled state; caller must prime at committed truth."""

        self._next_sample_time_s = 0.0
        self._next_sample_index = 0
        self._pending: list[dict[str, object]] = []
        self._latest_sample_time_s: float | None = None
        self._latest_values: dict[str, object | None] = {channel_id: None for channel_id in self.channel_ids}
        self._latest_valid: dict[str, bool] = {channel_id: False for channel_id in self.channel_ids}
        self._last_boundary_time_s: float | None = None
        ####

    def next_required_boundary(self, current_time_s: float, upper_time_s: float) -> float | None:
        """Return the next cadence or sample-release boundary inside an interval."""

        if upper_time_s <= current_time_s + _TIME_TOLERANCE_S:
            return None
        candidates = [self._next_sample_time_s]
        candidates.extend(_finite(item["release_time_s"], "pending.release_time_s") for item in self._pending)
        future = [value for value in candidates if value > current_time_s + _TIME_TOLERANCE_S]
        if not future:
            return None
        boundary = min(future)
        return boundary if boundary <= upper_time_s + _TIME_TOLERANCE_S else None
        ####

    def advance(self, time_s: float, truth_values: Mapping[str, object]) -> BoundarySensorReading:
        """Consume one committed boundary, sampling/releasing only when due.

        A caller may not jump over a required sensor or delayed-release
        boundary.  That fail-closed rule makes cadence part of integration
        scheduling instead of a post-step interpolation request.
        """

        if not math.isfinite(time_s):
            raise ValueError("boundary sensor time must be finite")
        if self._last_boundary_time_s is not None and time_s < self._last_boundary_time_s - _TIME_TOLERANCE_S:
            raise ValueError("boundary sensor cannot move backwards in time")
        skipped = self.next_required_boundary(self._last_boundary_time_s or 0.0, time_s)
        if skipped is not None and skipped < time_s - _TIME_TOLERANCE_S:
            raise ValueError(f"boundary sensor required truth boundary at {skipped:.12g} s was skipped")
        while self._next_sample_time_s <= time_s + _TIME_TOLERANCE_S:
            if abs(self._next_sample_time_s - time_s) > _TIME_TOLERANCE_S:
                raise ValueError(
                    f"boundary sensor sample at {self._next_sample_time_s:.12g} s requires committed truth, got {time_s:.12g} s"
                )
            missing = sorted(set(self.channel_ids) - set(truth_values))
            if missing:
                raise ValueError(f"boundary sensor cannot sample missing truth channel(s): {missing}")
            self._pending.append(
                {
                    "sample_time_s": self._next_sample_time_s,
                    "release_time_s": self._next_sample_time_s + self.latency_s,
                    "values": self._measure_values(truth_values, self._next_sample_index),
                }
            )
            self._next_sample_time_s += self.cadence_s
            self._next_sample_index += 1
        releasable = [
            item
            for item in self._pending
            if _finite(item["release_time_s"], "pending.release_time_s") <= time_s + _TIME_TOLERANCE_S
        ]
        if releasable:
            latest = max(releasable, key=lambda item: _finite(item["sample_time_s"], "pending.sample_time_s"))
            self._latest_sample_time_s = _finite(latest["sample_time_s"], "pending.sample_time_s")
            values = latest["values"]
            if not isinstance(values, Mapping):
                raise ValueError("boundary sensor checkpoint contains invalid sample values")
            self._latest_values = {channel_id: values[channel_id] for channel_id in self.channel_ids}
            self._latest_valid = {channel_id: True for channel_id in self.channel_ids}
            self._pending = [item for item in self._pending if item not in releasable]
        self._last_boundary_time_s = time_s
        return BoundarySensorReading(time_s, self._latest_sample_time_s, dict(self._latest_values), dict(self._latest_valid))
        ####

    def reading(self, time_s: float) -> BoundarySensorReading:
        """Return the latest released sample without advancing sensor state."""

        return BoundarySensorReading(time_s, self._latest_sample_time_s, dict(self._latest_values), dict(self._latest_valid))
        ####

    def checkpoint_payload(self) -> dict[str, object]:
        """Return deterministic sensor state for an episode checkpoint."""

        return {
            "profile_id": self.profile_id,
            "channel_ids": list(self.channel_ids),
            "cadence_s": self.cadence_s,
            "latency_s": self.latency_s,
            "channel_errors": {channel_id: error.as_dict() for channel_id, error in self.channel_errors.items()},
            "seed": self.seed,
            "next_sample_time_s": self._next_sample_time_s,
            "next_sample_index": self._next_sample_index,
            "pending": [dict(item) for item in self._pending],
            "latest_sample_time_s": self._latest_sample_time_s,
            "latest_values": dict(self._latest_values),
            "latest_valid": dict(self._latest_valid),
            "last_boundary_time_s": self._last_boundary_time_s,
        }
        ####

    def restore_checkpoint(self, payload: Mapping[str, object]) -> None:
        """Restore only matching declared-sensor state from a verified payload."""

        payload_channels = payload.get("channel_ids")
        if not isinstance(payload_channels, list | tuple):
            raise ValueError("boundary sensor checkpoint channel_ids must be a sequence")
        if payload.get("profile_id") != self.profile_id or tuple(payload_channels) != self.channel_ids:
            raise ValueError("boundary sensor checkpoint profile mismatch")
        if payload.get("cadence_s") != self.cadence_s or payload.get("latency_s") != self.latency_s:
            raise ValueError("boundary sensor checkpoint timing mismatch")
        if payload.get("seed") != self.seed:
            raise ValueError("boundary sensor checkpoint seed mismatch")
        checkpoint_errors = _channel_errors(self.channel_ids, _error_mapping(payload.get("channel_errors")))
        if checkpoint_errors != self.channel_errors:
            raise ValueError("boundary sensor checkpoint measurement model mismatch")
        next_time = _finite(payload.get("next_sample_time_s"), "next_sample_time_s")
        next_index = _sample_index(payload.get("next_sample_index"), "next_sample_index")
        last_time_raw = payload.get("last_boundary_time_s")
        last_time = None if last_time_raw is None else _finite(last_time_raw, "last_boundary_time_s")
        pending_raw = payload.get("pending")
        if not isinstance(pending_raw, list):
            raise ValueError("boundary sensor checkpoint pending must be a list")
        pending: list[dict[str, object]] = []
        for item in pending_raw:
            if not isinstance(item, Mapping):
                raise ValueError("boundary sensor checkpoint pending item must be a mapping")
            values = item.get("values")
            if not isinstance(values, Mapping) or set(values) != set(self.channel_ids):
                raise ValueError("boundary sensor checkpoint pending sample channels mismatch")
            pending.append(
                {
                    "sample_time_s": _finite(item.get("sample_time_s"), "pending.sample_time_s"),
                    "release_time_s": _finite(item.get("release_time_s"), "pending.release_time_s"),
                    "values": {channel_id: values[channel_id] for channel_id in self.channel_ids},
                }
            )
        latest_values = payload.get("latest_values")
        latest_valid = payload.get("latest_valid")
        if not isinstance(latest_values, Mapping) or not isinstance(latest_valid, Mapping):
            raise ValueError("boundary sensor checkpoint latest reading must be mappings")
        if set(latest_values) != set(self.channel_ids) or set(latest_valid) != set(self.channel_ids):
            raise ValueError("boundary sensor checkpoint latest reading channels mismatch")
        latest_sample_raw = payload.get("latest_sample_time_s")
        self._next_sample_time_s = next_time
        self._next_sample_index = next_index
        self._pending = pending
        self._latest_sample_time_s = None if latest_sample_raw is None else _finite(latest_sample_raw, "latest_sample_time_s")
        self._latest_values = {channel_id: latest_values[channel_id] for channel_id in self.channel_ids}
        self._latest_valid = {channel_id: bool(latest_valid[channel_id]) for channel_id in self.channel_ids}
        self._last_boundary_time_s = last_time
        ####

    def _measure_values(self, truth_values: Mapping[str, object], sample_index: int) -> dict[str, object]:
        """Capture exactly one declared transformed reading at a truth boundary."""

        result: dict[str, object] = {}
        for channel_id in self.channel_ids:
            value = truth_values[channel_id]
            error = self.channel_errors.get(channel_id)
            if error is None:
                result[channel_id] = value
                continue
            numeric = _finite_measurement(value, channel_id)
            noisy = numeric + error.bias
            if error.gaussian_stddev > 0.0:
                noisy += error.gaussian_stddev * _standard_normal(
                    seed=self.seed,
                    profile_id=self.profile_id,
                    channel_id=channel_id,
                    sample_index=sample_index,
                )
            if error.quantization_step is not None:
                noisy = _quantize(noisy, error.quantization_step)
            result[channel_id] = noisy
        return result
        ####


def _finite(value: object, label: str) -> float:
    if not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"boundary sensor checkpoint {label} must be finite")
    return float(value)
    ####


def _channel_errors(
    channel_ids: tuple[str, ...],
    values: Mapping[str, ChannelMeasurementError | Mapping[str, object]] | None,
) -> dict[str, ChannelMeasurementError]:
    """Normalize a sparse declared error mapping without inferring channels."""

    if values is None:
        return {}
    unknown = sorted(set(values) - set(channel_ids))
    if unknown:
        raise ValueError(f"sensor measurement model references undeclared channel(s): {unknown}")
    result: dict[str, ChannelMeasurementError] = {}
    for channel_id, raw in values.items():
        if isinstance(raw, ChannelMeasurementError):
            result[channel_id] = raw
            continue
        if not isinstance(raw, Mapping):
            raise ValueError(f"sensor measurement model for {channel_id!r} must be a mapping")
        allowed = {"bias", "gaussian_stddev", "quantization_step"}
        extras = sorted(set(raw) - allowed)
        if extras:
            raise ValueError(f"sensor measurement model for {channel_id!r} has unknown field(s): {extras}")
        result[channel_id] = ChannelMeasurementError(
            bias=_finite(raw.get("bias", 0.0), f"sensor measurement bias for {channel_id}"),
            gaussian_stddev=_finite(raw.get("gaussian_stddev", 0.0), f"sensor measurement Gaussian deviation for {channel_id}"),
            quantization_step=None
            if raw.get("quantization_step") is None
            else _finite(raw["quantization_step"], f"sensor measurement quantization step for {channel_id}"),
        )
    return result
    ####


def _error_mapping(value: object) -> Mapping[str, ChannelMeasurementError | Mapping[str, object]]:
    if not isinstance(value, Mapping):
        raise ValueError("boundary sensor checkpoint channel_errors must be a mapping")
    return value
    ####


def _seed(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("boundary sensor seed must be an integer")
    return value
    ####


def _sample_index(value: object, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"boundary sensor checkpoint {label} must be a nonnegative integer")
    return value
    ####


def _finite_measurement(value: object, channel_id: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(
            f"sensor measurement transform for {channel_id!r} requires a finite scalar truth channel"
        )
    return float(value)
    ####


def _standard_normal(*, seed: int, profile_id: str, channel_id: str, sample_index: int) -> float:
    """Return a stateless deterministic N(0,1) draw for one capture/channel."""

    material = f"taoryx.boundary-sensor/v1|{seed}|{profile_id}|{channel_id}|{sample_index}".encode("utf-8")
    digest = hashlib.sha256(material).digest()
    denominator = float(1 << 64)
    first = (int.from_bytes(digest[:8], "big") + 0.5) / denominator
    second = (int.from_bytes(digest[8:16], "big") + 0.5) / denominator
    return math.sqrt(-2.0 * math.log(first)) * math.cos(2.0 * math.pi * second)
    ####


def _quantize(value: float, step: float) -> float:
    """Round symmetrically to the nearest declared measurement increment."""

    return math.copysign(math.floor(abs(value) / step + 0.5) * step, value)
    ####


__all__ = ["BoundarySensorReading", "ChannelMeasurementError", "CommittedBoundarySensor"]
