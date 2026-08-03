"""Declared sensor traces replayed only across committed batch-truth samples.

This is the batch counterpart to the interactive episode sensor runtime.  It
does not resample or interpolate a completed trajectory: every requested
capture and delayed-release instant must already be a committed batch truth
row, otherwise the trace fails closed.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from .committed_boundary_sensor import CommittedBoundarySensor
from .vehicle_composition import CompiledVehicleComposition, resolve_vehicle_composition_interface_contract
from .vehicle_interface import project_committed_status_values, validate_projected_status_values


@dataclass(frozen=True, slots=True)
class BatchTruthSample:
    """One source-owned committed batch truth row in adapter-native shape."""

    time_s: float
    raw_values: Mapping[str, object]
    execution_status: str = "active"


def build_declared_sensor_trace(
    composition: CompiledVehicleComposition,
    samples: Sequence[BatchTruthSample],
    *,
    seed: int = 0,
) -> dict[str, object] | None:
    """Emit a held sensor trace, or ``None`` for a truth-only composition.

    The composition owns the profile selection.  A sensor-configured batch
    path must supply every cadence and release boundary in ``samples``; a
    missing boundary raises rather than using a later truth row as a surrogate.
    """

    declared = composition.observation.declared_sensor
    if declared is None:
        return None
    if not samples:
        raise ValueError("declared sensor trace requires committed batch truth samples")
    contract = resolve_vehicle_composition_interface_contract(composition)
    profile = contract.observation_profile(declared.profile_id)
    if profile.source != "sensor" or profile.availability != "available":
        raise ValueError(f"composition sensor profile {declared.profile_id!r} is not available")
    if profile.cadence_s is None or profile.latency_s is None:
        raise ValueError(f"composition sensor profile {declared.profile_id!r} has incomplete timing")
    sensor = CommittedBoundarySensor(
        profile_id=profile.id,
        channel_ids=profile.channel_ids,
        cadence_s=profile.cadence_s,
        latency_s=profile.latency_s,
        channel_errors=profile.channel_errors,
        seed=seed,
    )
    trace_samples: list[dict[str, object]] = []
    for sample in samples:
        status_values = project_committed_status_values(
            contract,
            time_s=sample.time_s,
            execution_status=sample.execution_status,
            raw_values=sample.raw_values,
        )
        validate_projected_status_values(
            contract,
            status_values,
            context=f"declared sensor source t={sample.time_s:.12g} s",
        )
        reading = sensor.advance(sample.time_s, status_values)
        trace_samples.append(
            {
                "time_s": reading.time_s,
                "source_time_s": reading.sample_time_s,
                "values": dict(reading.values),
                "valid": dict(reading.valid),
            }
        )
    return {
        "schema": "taoryx.composition-declared-sensor-trace/v1alpha1",
        "interface_id": contract.id,
        "interface_fingerprint_sha256": contract.fingerprint,
        "observation_profile_id": profile.id,
        "cadence_s": profile.cadence_s,
        "latency_s": profile.latency_s,
        "channel_errors": {identifier: dict(error) for identifier, error in profile.channel_errors.items()},
        "seed": seed,
        "capture_rule": "committed_truth_boundary_only",
        "interpolation": "forbidden",
        "samples": trace_samples,
        "claim_boundary": (
            "This trace replays only the declared committed-boundary measurement transform over supplied batch truth. "
            "It records any configured scalar bias, deterministic white-noise, and quantization transform, but does "
            "not add an IMU, GPS, estimator state, or future-state interpolation."
        ),
    }
    ####


__all__ = ["BatchTruthSample", "build_declared_sensor_trace"]
