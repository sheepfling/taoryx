"""Portable resource-ledger projections at committed truth boundaries.

The resource ledger is a semantic artifact, not a cross-family fuel model. It
extracts only the resource channels explicitly declared by the resolved
interface from an already validated committed status trace.  This gives batch
search, UI, and release tooling one identity-bound history for mass, battery,
propellant, or other represented resources without inventing absent energy
physics or interpolating state between accepted truth boundaries.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, TypeGuard

from .composition_status_trace import validate_committed_status_trace
from .vehicle_composition import CompiledVehicleComposition, resolve_vehicle_composition_interface_contract

if TYPE_CHECKING:
    from .plugins import PluginCatalog

_SCHEMA = "taoryx.composition-resource-ledger/v1alpha1"


def build_committed_resource_ledger(
    composition: CompiledVehicleComposition,
    status_trace: Mapping[str, object],
    *,
    plugins: PluginCatalog | None = None,
) -> dict[str, object]:
    """Extract declared resource histories without creating resource values.

    ``status_trace`` must already be a complete committed-boundary projection.
    Every resource value in this artifact is copied from the corresponding
    trace row; no rate integration, depletion prediction, or resampling is
    permitted here.
    """

    validate_committed_status_trace(composition, status_trace, plugins=plugins)
    contract = resolve_vehicle_composition_interface_contract(composition, plugins=plugins)
    resource_channels = tuple(contract.resource_channels)
    visible_ids = {channel.id for channel in resource_channels if channel.availability in {"available", "available_in_batch"}}
    trace_samples = _trace_samples(status_trace)
    samples: list[dict[str, object]] = []
    values_by_channel: dict[str, list[object]] = {channel.id: [] for channel in resource_channels}
    for trace_sample in trace_samples:
        time_s = _finite_time(trace_sample.get("time_s"), "committed status trace resource sample")
        values = trace_sample.get("values")
        if not isinstance(values, Mapping):
            raise ValueError("validated committed status trace has no values mapping")
        resource_values: dict[str, object] = {}
        for channel in resource_channels:
            if channel.id not in visible_ids:
                continue
            if channel.id not in values:
                raise ValueError(f"committed status trace omits visible resource channel {channel.id!r}")
            value = values[channel.id]
            resource_values[channel.id] = value
            values_by_channel[channel.id].append(value)
        samples.append({"time_s": time_s, "values": resource_values})
    channel_records = [
        {
            "id": channel.id,
            "canonical_unit": channel.canonical_unit,
            "frame": channel.frame,
            "value_type": channel.value_type,
            "value_space": channel.value_space.as_dict() if channel.value_space is not None else None,
            "availability": channel.availability,
            "provenance": channel.provenance,
            "sampling": channel.sampling,
            "binding": dict(channel.binding),
            "claim_boundary": channel.claim_boundary,
        }
        for channel in resource_channels
    ]
    summaries = [_resource_summary(channel.id, channel.availability, values_by_channel[channel.id]) for channel in resource_channels]
    return {
        "schema": _SCHEMA,
        "composition_id": composition.id,
        "composition_identity_sha256": composition.identity_sha256,
        "interface_id": contract.id,
        "interface_fingerprint_sha256": contract.fingerprint,
        "sampling": "committed_truth_boundary_only",
        "resource_channels": channel_records,
        "samples": samples,
        "summaries": summaries,
        "claim_boundary": (
            "This is an identity-bound projection of interface-declared resources from committed status truth. "
            "It does not derive absent fuel, energy, mass flow, inertia, or depletion physics; trend labels "
            "describe only emitted samples."
        ),
    }
    ####


def validate_committed_resource_ledger(
    composition: CompiledVehicleComposition,
    ledger: Mapping[str, object],
    *,
    status_trace: Mapping[str, object],
    plugins: PluginCatalog | None = None,
) -> None:
    """Reject a ledger that is detached from its composition or status trace."""

    expected = build_committed_resource_ledger(composition, status_trace, plugins=plugins)
    for field in (
        "schema",
        "composition_id",
        "composition_identity_sha256",
        "interface_id",
        "interface_fingerprint_sha256",
        "sampling",
        "resource_channels",
        "samples",
        "summaries",
    ):
        if ledger.get(field) != expected[field]:
            raise ValueError(f"committed resource ledger {field!r} disagrees with the composition status trace")
    ####


def resource_ledger_summary(ledger: Mapping[str, object]) -> dict[str, object]:
    """Return a manifest-sized reference without collapsing resource values."""

    samples = ledger.get("samples")
    summaries = ledger.get("summaries")
    return {
        "schema": ledger.get("schema"),
        "sample_count": len(samples) if isinstance(samples, list) else 0,
        "resource_count": len(summaries) if isinstance(summaries, list) else 0,
        "artifact": "resource_ledger.json",
    }
    ####


def _trace_samples(status_trace: Mapping[str, object]) -> list[Mapping[str, object]]:
    samples = status_trace.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError("validated committed status trace has no samples")
    if not all(isinstance(sample, Mapping) for sample in samples):
        raise ValueError("validated committed status trace has a malformed sample")
    return list(samples)
    ####


def _resource_summary(identifier: str, availability: str, values: list[object]) -> dict[str, object]:
    if availability not in {"available", "available_in_batch"}:
        return {
            "id": identifier,
            "availability": availability,
            "sample_count": 0,
            "trend": "not_available",
            "initial_value": None,
            "final_value": None,
            "minimum_value": None,
            "maximum_value": None,
        }
    numeric_values = [float(value) for value in values if _finite_scalar(value)]
    if len(numeric_values) != len(values):
        return {
            "id": identifier,
            "availability": availability,
            "sample_count": len(values),
            "trend": "non_numeric_or_discrete",
            "initial_value": values[0] if values else None,
            "final_value": values[-1] if values else None,
            "minimum_value": None,
            "maximum_value": None,
        }
    return {
        "id": identifier,
        "availability": availability,
        "sample_count": len(numeric_values),
        "trend": _numeric_trend(numeric_values),
        "initial_value": numeric_values[0] if numeric_values else None,
        "final_value": numeric_values[-1] if numeric_values else None,
        "minimum_value": min(numeric_values) if numeric_values else None,
        "maximum_value": max(numeric_values) if numeric_values else None,
    }
    ####


def _numeric_trend(values: list[float]) -> str:
    if len(values) < 2 or all(math.isclose(value, values[0], rel_tol=1.0e-12, abs_tol=1.0e-12) for value in values[1:]):
        return "constant"
    nondecreasing = all(later >= earlier for earlier, later in zip(values, values[1:]))
    nonincreasing = all(later <= earlier for earlier, later in zip(values, values[1:]))
    if nondecreasing:
        return "nondecreasing"
    if nonincreasing:
        return "nonincreasing"
    return "variable"
    ####


def _finite_time(value: Any, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"{context} has no finite time")
    return float(value)
    ####


def _finite_scalar(value: object) -> TypeGuard[int | float]:
    return isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(float(value))
    ####


__all__ = [
    "build_committed_resource_ledger",
    "resource_ledger_summary",
    "validate_committed_resource_ledger",
]
