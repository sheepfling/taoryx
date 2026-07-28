"""Reproducible comparisons between explicit ``imu-error-model`` profiles."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import numpy as np

from .sensors import ImuErrorModelAdapter, ImuIncrement, MeasurementPacket, TruthPoint


@dataclass(frozen=True, slots=True)
class ImuProfileComparison:
    """Comparable packet metrics for two explicitly selected profiles."""

    baseline: dict[str, Any]
    candidate: dict[str, Any]
    difference: dict[str, float]
    configuration: dict[str, Any]

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "artifact_id": "taoryx.imu-profile-comparison.v1",
            "configuration": dict(self.configuration),
            "baseline": dict(self.baseline),
            "candidate": dict(self.candidate),
            "difference": dict(self.difference),
        }


def _truth(time_s: float) -> TruthPoint:
    """Provide a deterministic rotating, accelerating ECI truth sequence."""

    angle = 0.2 * time_s
    orientation = np.array(
        [
            [math.cos(angle), -math.sin(angle), 0.0],
            [math.sin(angle), math.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    velocity = np.array([3.0 * time_s, 0.2 * time_s * time_s, 0.0])
    return TruthPoint(
        time_s=time_s,
        position_eci_m=np.array([1.5 * time_s * time_s, (0.2 / 3.0) * time_s**3, 0.0]),
        velocity_eci_mps=velocity,
        velocity_without_gravity_eci_mps=velocity,
        orientation_eci_from_body=orientation,
        gravity_eci_mps2=np.zeros(3),
        angular_rate_body_radps=np.array([0.0, 0.0, 0.2]),
        acceleration_eci_mps2=np.array([3.0, 0.4 * time_s, 0.0]),
    )


def _packet_metrics(adapter: ImuErrorModelAdapter, *, sample_period_s: float, horizon_s: float) -> dict[str, Any]:
    packets: list[MeasurementPacket[ImuIncrement]] = []
    count = int(round(horizon_s / sample_period_s))
    for index in range(count + 1):
        packets.append(adapter.sample(_truth(index * sample_period_s)))
    payloads = [packet.payload for packet in packets if packet.valid and isinstance(packet.payload, ImuIncrement)]
    delta_v = np.array([np.linalg.norm(item.delta_v_body_mps) for item in payloads])
    delta_theta = np.array([np.linalg.norm(item.delta_theta_body_rad) for item in payloads])
    payload: dict[str, Any] = {
        "packet_count": len(packets),
        "valid_packet_count": len(payloads),
        "invalid_packet_count": len(packets) - len(payloads),
        "first_sampled_at_s": packets[0].sampled_at_s,
        "last_sampled_at_s": packets[-1].sampled_at_s,
        "delta_v_norm_mean_mps": float(np.mean(delta_v)) if len(delta_v) else None,
        "delta_v_norm_max_mps": float(np.max(delta_v)) if len(delta_v) else None,
        "delta_theta_norm_mean_rad": float(np.mean(delta_theta)) if len(delta_theta) else None,
        "delta_theta_norm_max_rad": float(np.max(delta_theta)) if len(delta_theta) else None,
        "packet_sha256": _stable_hash(
            [
                {
                    "sampled_at_s": packet.sampled_at_s,
                    "valid": packet.valid,
                    "delta_v_body_mps": None if packet.payload is None else packet.payload.delta_v_body_mps.tolist(),
                    "delta_theta_body_rad": None if packet.payload is None else packet.payload.delta_theta_body_rad.tolist(),
                }
                for packet in packets
            ]
        ),
    }
    payload["profile"] = dict(adapter.provenance)
    return payload


def compare_imu_profiles(
    baseline_path: str | Path,
    candidate_path: str | Path,
    *,
    seed: int = 41,
    horizon_s: float = 1.0,
) -> ImuProfileComparison:
    """Run two profiles against identical accepted truth and compare metrics."""

    if not math.isfinite(horizon_s) or horizon_s <= 0.0:
        raise ValueError("profile comparison horizon_s must be positive and finite")
    baseline = ImuErrorModelAdapter.from_profile(baseline_path, seed=seed)
    candidate = ImuErrorModelAdapter.from_profile(candidate_path, seed=seed)
    baseline_period = float(cast(float, baseline.provenance["sample_period_s"]))
    candidate_period = float(cast(float, candidate.provenance["sample_period_s"]))
    baseline_metrics = _packet_metrics(adapter=baseline, sample_period_s=baseline_period, horizon_s=horizon_s)
    candidate_metrics = _packet_metrics(adapter=candidate, sample_period_s=candidate_period, horizon_s=horizon_s)
    difference: dict[str, float] = {}
    for name in (
        "valid_packet_count",
        "delta_v_norm_mean_mps",
        "delta_v_norm_max_mps",
        "delta_theta_norm_mean_rad",
        "delta_theta_norm_max_rad",
    ):
        left = baseline_metrics.get(name)
        right = candidate_metrics.get(name)
        if isinstance(left, (int, float)) and isinstance(right, (int, float)):
            difference[f"candidate_minus_baseline.{name}"] = float(right) - float(left)
    return ImuProfileComparison(
        baseline=baseline_metrics,
        candidate=candidate_metrics,
        difference=difference,
        configuration={"seed": seed, "horizon_s": horizon_s, "truth": "deterministic-eci-rotation-acceleration-v1"},
    )


def _stable_hash(value: object) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
