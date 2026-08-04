"""Portable batch status traces at committed truth boundaries.

Batch execution artifacts must not force consumers to parse a native state
layout just because an interactive episode happens to have a semantic status
frame.  This module projects the exact resolved vehicle-interface contract
over source-owned committed batch samples.  It does not interpolate, invent
unavailable values, or turn a batch-only channel into an episode observation.
"""

from __future__ import annotations

from collections.abc import Sequence

from .composition_sensor_trace import BatchTruthSample
from .vehicle_composition import CompiledVehicleComposition, resolve_vehicle_composition_interface_contract
from .vehicle_interface import project_committed_status_values


def build_committed_status_trace(
    composition: CompiledVehicleComposition,
    samples: Sequence[BatchTruthSample],
) -> dict[str, object]:
    """Project all declared batch-visible semantic channels without gaps.

    Every channel declared ``available`` or ``available_in_batch`` must bind
    at every supplied source-owned truth row.  Failing instead of dropping an
    advertised channel keeps a missing adapter binding visible to the caller
    and prevents a partly populated status record from being mistaken for a
    generic vehicle API.
    """

    if not samples:
        raise ValueError("committed status trace requires at least one batch truth sample")
    contract = resolve_vehicle_composition_interface_contract(composition)
    expected = {
        channel.id
        for channel in (
            *contract.status_channels,
            *contract.resource_channels,
            *contract.diagnostic_channels,
        )
        if channel.availability in {"available", "available_in_batch"}
    }
    trace_samples: list[dict[str, object]] = []
    previous_time: float | None = None
    for sample in samples:
        if previous_time is not None and sample.time_s < previous_time:
            raise ValueError("committed status trace samples must be time-ordered")
        values = project_committed_status_values(
            contract,
            time_s=sample.time_s,
            execution_status=sample.execution_status,
            raw_values=sample.raw_values,
            include_batch_available=True,
        )
        missing = sorted(expected - set(values))
        if missing:
            raise ValueError(
                "committed status trace cannot resolve declared batch channel(s) at "
                f"t={sample.time_s:.12g} s: {', '.join(missing)}"
            )
        trace_samples.append({"time_s": sample.time_s, "values": values})
        previous_time = sample.time_s
    return {
        "schema": "taoryx.composition-status-trace/v1alpha1",
        "interface_id": contract.id,
        "interface_fingerprint_sha256": contract.fingerprint,
        "sampling": "committed_truth_boundary_only",
        "observation_profile_id": None,
        "channels": sorted(expected),
        "samples": trace_samples,
        "claim_boundary": (
            "This is a portable projection of source-owned committed batch truth. It does not provide a sensor "
            "model, interpolate between steps, manufacture unavailable channels, or make batch-only data visible "
            "to an episode policy."
        ),
    }
    ####


def status_trace_summary(trace: dict[str, object]) -> dict[str, object]:
    """Return the manifest-sized reference to a complete status-trace artifact."""

    samples = trace.get("samples")
    channels = trace.get("channels")
    return {
        "schema": trace["schema"],
        "sample_count": len(samples) if isinstance(samples, list) else 0,
        "channel_count": len(channels) if isinstance(channels, list) else 0,
        "artifact": "status_trace.json",
    }
    ####


__all__ = ["build_committed_status_trace", "status_trace_summary"]
