"""Committed interval command and effector evidence for composed batch runs.

The portable status trace answers what was true at committed boundaries. This
separate artifact records what semantic command was held over the immediately
preceding integration interval and any real effector response the selected
fidelity actually publishes. It intentionally does not infer effectors from a
clean trajectory or turn a pseudo-6DOF response coordinate into hardware.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING

from .vehicle_composition import CompiledVehicleComposition, resolve_vehicle_composition_interface_contract
from .vehicle_interface import AuthorityProfile, InterfaceChannel, VehicleInterfaceContract, validate_interface_channel_value

if TYPE_CHECKING:
    from .plugins import PluginCatalog

_SCHEMA = "taoryx.composition-semantic-action-trace/v1alpha1"


@dataclass(frozen=True, slots=True)
class BatchControlSample:
    """One held command interval ending at a committed truth sample."""

    interval_start_time_s: float
    committed_truth_time_s: float
    requested_actions: Mapping[str, object]
    achieved_effectors: Mapping[str, object]


def build_committed_control_trace(
    composition: CompiledVehicleComposition,
    samples: Sequence[BatchControlSample],
    *,
    plugins: PluginCatalog | None = None,
) -> dict[str, object]:
    """Build a complete semantic command trace without synthesizing channels.

    Each sample names the exact command held on ``[start, committed]``. The
    achieved-effector mapping is deliberately empty when a fidelity has no
    declared physical effectors, such as an aggregate-thrust pseudo-6DOF
    response law. Every action/effector that is batch-visible in the resolved
    interface must be present in every respective sample.
    """

    if not samples:
        raise ValueError("committed control trace requires at least one sample")
    contract = resolve_vehicle_composition_interface_contract(composition, plugins=plugins)
    actions, authority_profile = _batch_action_channels(
        contract,
        requested_action_ids=set(samples[0].requested_actions),
    )
    effectors = _batch_visible_channels(contract.effector_channels)
    serialized: list[dict[str, object]] = []
    previous_truth_time: float | None = None
    for index, sample in enumerate(samples):
        start = _finite_time(sample.interval_start_time_s, f"control trace sample {index} interval start")
        committed = _finite_time(sample.committed_truth_time_s, f"control trace sample {index} committed truth time")
        if start > committed:
            raise ValueError(f"control trace sample {index} starts after its committed truth time")
        if previous_truth_time is not None and start < previous_truth_time:
            raise ValueError("committed control trace intervals overlap or are out of order")
        requested = _complete_channel_values(actions, sample.requested_actions, f"control trace sample {index} requested actions")
        achieved = _complete_channel_values(effectors, sample.achieved_effectors, f"control trace sample {index} achieved effectors")
        serialized.append(
            {
                "interval_start_time_s": start,
                "committed_truth_time_s": committed,
                "requested_actions": requested,
                "achieved_effectors": achieved,
            }
        )
        previous_truth_time = committed
    return {
        "schema": _SCHEMA,
        "composition_id": composition.id,
        "composition_identity_sha256": composition.identity_sha256,
        "interface_id": contract.id,
        "interface_fingerprint_sha256": contract.fingerprint,
        "sampling": "held_action_interval_to_committed_truth_boundary",
        "authority_profile_id": authority_profile.id if authority_profile is not None else None,
        "command_owner": authority_profile.command_owner if authority_profile is not None else None,
        "lowering_chain": list(authority_profile.lowering_chain) if authority_profile is not None else [],
        "requested_action_channels": sorted(actions),
        "achieved_effector_channels": sorted(effectors),
        "samples": serialized,
        "claim_boundary": (
            "Each requested action belongs to the named authority profile, or to an explicitly labeled legacy native union, and is held over the interval ending at the named committed truth "
            "sample. Achieved values are limited to explicitly declared effectors; absent effectors remain absent. "
            "The trace neither interpolates commands nor establishes physical allocation beyond the selected fidelity."
        ),
    }
    ####


def build_uncontrolled_committed_control_trace(
    composition: CompiledVehicleComposition,
    committed_truth_times_s: Sequence[float],
    *,
    plugins: PluginCatalog | None = None,
) -> dict[str, object]:
    """Record an action-free source replay without manufacturing controls.

    Some valid batch paths replay a retained source history or an explicitly
    open-loop profile.  They still need an identity-bound interval artifact so
    a consumer can distinguish *no public command exists* from a runner that
    forgot to retain command history.  This helper accepts only interfaces
    with no batch-visible action or effector channels; a controlled path must
    use :func:`build_committed_control_trace` with its actual held values.
    """

    contract = resolve_vehicle_composition_interface_contract(composition, plugins=plugins)
    actions = _batch_visible_channels(contract.action_channels)
    effectors = _batch_visible_channels(contract.effector_channels)
    if actions or effectors:
        raise ValueError("uncontrolled action trace cannot stand in for declared actions or effectors")
    if not committed_truth_times_s:
        raise ValueError("uncontrolled action trace requires at least one committed truth time")
    samples: list[BatchControlSample] = []
    previous_time: float | None = None
    for index, time_s in enumerate(committed_truth_times_s):
        committed = _finite_time(time_s, f"uncontrolled action trace time {index}")
        if previous_time is not None and committed < previous_time:
            raise ValueError("uncontrolled action trace times must be ordered")
        samples.append(
            BatchControlSample(
                interval_start_time_s=committed if previous_time is None else previous_time,
                committed_truth_time_s=committed,
                requested_actions={},
                achieved_effectors={},
            )
        )
        previous_time = committed
    return build_committed_control_trace(composition, samples, plugins=plugins)
    ####


def validate_committed_control_trace(
    composition: CompiledVehicleComposition,
    trace: Mapping[str, object],
    *,
    plugins: PluginCatalog | None = None,
) -> None:
    """Reject a command artifact that disagrees with the exact composition interface."""

    contract = resolve_vehicle_composition_interface_contract(composition, plugins=plugins)
    raw_action_ids = trace.get("requested_action_channels")
    requested_action_ids = (
        set(raw_action_ids)
        if isinstance(raw_action_ids, list) and all(isinstance(item, str) for item in raw_action_ids)
        else None
    )
    actions, authority_profile = _batch_action_channels(
        contract,
        requested_action_ids=requested_action_ids,
    )
    effectors = _batch_visible_channels(contract.effector_channels)
    if trace.get("schema") != _SCHEMA:
        raise ValueError("committed control trace has an unknown schema")
    if trace.get("composition_id") != composition.id:
        raise ValueError("committed control trace composition ID disagrees with the composition")
    if trace.get("composition_identity_sha256") != composition.identity_sha256:
        raise ValueError("committed control trace fingerprint disagrees with the composition")
    if trace.get("interface_id") != contract.id:
        raise ValueError("committed control trace interface ID disagrees with the composition")
    if trace.get("interface_fingerprint_sha256") != contract.fingerprint:
        raise ValueError("committed control trace interface fingerprint disagrees with the composition")
    if trace.get("sampling") != "held_action_interval_to_committed_truth_boundary":
        raise ValueError("committed control trace must declare held-action interval sampling")
    expected_authority_profile_id = authority_profile.id if authority_profile is not None else None
    if trace.get("authority_profile_id") != expected_authority_profile_id:
        raise ValueError("committed control trace authority profile disagrees with the interface")
    expected_command_owner = authority_profile.command_owner if authority_profile is not None else None
    if trace.get("command_owner") != expected_command_owner:
        raise ValueError("committed control trace command owner disagrees with the interface")
    expected_lowering_chain = list(authority_profile.lowering_chain) if authority_profile is not None else []
    if trace.get("lowering_chain") != expected_lowering_chain:
        raise ValueError("committed control trace lowering chain disagrees with the interface")
    if trace.get("requested_action_channels") != sorted(actions):
        raise ValueError("committed control trace action channel set disagrees with the interface")
    if trace.get("achieved_effector_channels") != sorted(effectors):
        raise ValueError("committed control trace effector channel set disagrees with the interface")
    samples = trace.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError("committed control trace requires at least one sample")
    previous_truth_time: float | None = None
    for index, item in enumerate(samples):
        if not isinstance(item, Mapping):
            raise ValueError(f"committed control trace sample {index} is not a mapping")
        start = _finite_time(item.get("interval_start_time_s"), f"control trace sample {index} interval start")
        committed = _finite_time(item.get("committed_truth_time_s"), f"control trace sample {index} committed truth time")
        if start > committed:
            raise ValueError(f"control trace sample {index} starts after its committed truth time")
        if previous_truth_time is not None and start < previous_truth_time:
            raise ValueError("committed control trace intervals overlap or are out of order")
        _complete_channel_values(actions, _mapping(item.get("requested_actions"), f"control trace sample {index} requested actions"), f"control trace sample {index} requested actions")
        _complete_channel_values(effectors, _mapping(item.get("achieved_effectors"), f"control trace sample {index} achieved effectors"), f"control trace sample {index} achieved effectors")
        previous_truth_time = committed
    ####


def validate_committed_control_trace_against_status(
    composition: CompiledVehicleComposition,
    trace: Mapping[str, object],
    *,
    status_trace: Mapping[str, object],
    plugins: PluginCatalog | None = None,
) -> None:
    """Bind held action intervals to actual committed truth boundaries.

    Trace-local ordering is insufficient: a persisted control artifact could
    otherwise name plausible timestamps that do not occur in the matching
    status trace.  This cross-artifact check requires both ends of every held
    interval to be committed status boundaries.  It does not require every
    status sample to carry a separate action row; a family may legitimately
    retain additional truth samples while one command remains held.
    """

    validate_committed_control_trace(composition, trace, plugins=plugins)
    from .composition_status_trace import validate_committed_status_trace

    validate_committed_status_trace(composition, status_trace, plugins=plugins)
    raw_status_samples = status_trace.get("samples")
    raw_control_samples = trace.get("samples")
    if not isinstance(raw_status_samples, list) or not isinstance(raw_control_samples, list):
        raise ValueError("validated control/status traces have malformed samples")
    status_times = [
        _finite_time(item.get("time_s"), f"status trace sample {index} time")
        for index, item in enumerate(raw_status_samples)
        if isinstance(item, Mapping)
    ]
    if len(status_times) != len(raw_status_samples):
        raise ValueError("validated status trace contains a non-mapping sample")
    for index, item in enumerate(raw_control_samples):
        if not isinstance(item, Mapping):
            raise ValueError(f"validated control trace contains non-mapping sample {index}")
        start = _finite_time(item.get("interval_start_time_s"), f"control trace sample {index} interval start")
        committed = _finite_time(item.get("committed_truth_time_s"), f"control trace sample {index} committed truth time")
        for label, value in (("interval start", start), ("committed truth", committed)):
            if not any(math.isclose(value, candidate, rel_tol=1.0e-12, abs_tol=1.0e-12) for candidate in status_times):
                raise ValueError(
                    f"control trace sample {index} {label} time {value:.12g} is not a committed status boundary"
                )
    ####


def control_trace_summary(trace: Mapping[str, object]) -> dict[str, object]:
    """Return a manifest-sized reference to a complete action-trace artifact."""

    samples = trace.get("samples")
    actions = trace.get("requested_action_channels")
    effectors = trace.get("achieved_effector_channels")
    return {
        "schema": trace.get("schema"),
        "sample_count": len(samples) if isinstance(samples, list) else 0,
        "requested_action_channel_count": len(actions) if isinstance(actions, list) else 0,
        "achieved_effector_channel_count": len(effectors) if isinstance(effectors, list) else 0,
        "artifact": "semantic_action_trace.json",
    }
    ####


def _batch_visible_channels(channels: Sequence[InterfaceChannel]) -> dict[str, InterfaceChannel]:
    return {
        channel.id: channel
        for channel in channels
        if channel.availability in {"available", "available_in_batch"}
    }
    ####


def _batch_action_channels(
    contract: VehicleInterfaceContract,
    *,
    requested_action_ids: set[str] | None,
) -> tuple[dict[str, InterfaceChannel], AuthorityProfile | None]:
    """Resolve a profile-shaped batch surface while retaining legacy unions."""

    all_actions = _batch_visible_channels(contract.action_channels)
    if requested_action_ids is None:
        return all_actions, None
    matches = tuple(
        profile
        for profile in contract.authority_profiles
        if profile.availability in {"available", "available_in_batch"}
        and {
            identifier
            for identifier in profile.action_ids
            if identifier in all_actions
        }
        == requested_action_ids
    )
    if matches:
        profile = next(
            (item for item in matches if item.id == contract.default_authority_profile_id),
            matches[0],
        )
        return ({identifier: all_actions[identifier] for identifier in profile.action_ids if identifier in all_actions}, profile)
    return all_actions, None
    ####


def _complete_channel_values(
    channels: Mapping[str, InterfaceChannel],
    values: Mapping[str, object],
    context: str,
) -> dict[str, object]:
    missing = sorted(set(channels) - set(values))
    unexpected = sorted(set(values) - set(channels))
    if missing:
        raise ValueError(f"{context} omits declared channel(s): {', '.join(missing)}")
    if unexpected:
        raise ValueError(f"{context} contains undeclared channel(s): {', '.join(unexpected)}")
    projected: dict[str, object] = {}
    for identifier in sorted(channels):
        value = values[identifier]
        validate_interface_channel_value(channels[identifier], value, context=context)
        projected[identifier] = value
    return projected
    ####


def _finite_time(value: object, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
        raise ValueError(f"{context} must be finite")
    return float(value)
    ####


def _mapping(value: object, context: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{context} must be a mapping")
    return value
    ####


__all__ = [
    "BatchControlSample",
    "build_committed_control_trace",
    "build_uncontrolled_committed_control_trace",
    "control_trace_summary",
    "validate_committed_control_trace",
    "validate_committed_control_trace_against_status",
]
