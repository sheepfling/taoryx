"""Project trajectory metadata into the stateful session interface contract."""

from __future__ import annotations

import math
from typing import cast

from ..composition_episode import EpisodeChannel
from ..fidelity_contracts import FidelityTier
from ..value_space import (
    ValueSpaceSpec,
    ValueSpaceTopology,
    default_value_space_for_value_type,
)
from ..vehicle_interface import (
    AuthorityProfile,
    InterfaceChannel,
    InterfaceValueType,
    ObservationProfile,
    SamplingSemantics,
    VehicleInterfaceContract,
)
from .configuration_contract import (
    TrajectoryControlChannelMetadata,
    TrajectoryModelMetadata,
    TrajectoryOutputChannelMetadata,
)


def build_session_interface_contract(
    model: TrajectoryModelMetadata,
    *,
    realization_id: str,
    fidelity: str,
    family_id: str,
    physical_family: str,
    claim_boundary: str,
) -> tuple[VehicleInterfaceContract, tuple[EpisodeChannel, ...]]:
    """Build one exact step-capable interface from published model metadata."""

    realization = next(item for item in model.realizations if item.id == realization_id)
    action_channels = tuple(
        _action_channel(item)
        for item in realization.controls.channels
        if item.channel_kind == "action" and item.availability == "available" and "step" in item.operations
    )
    action_ids = {item.id for item in action_channels}
    authorities = tuple(
        AuthorityProfile(
            id=item.id,
            authority="native_bridge" if item.authority == "provider_defined" else item.authority,
            availability="available",
            action_ids=tuple(identifier for identifier in item.channel_ids if identifier in action_ids),
            description=item.description,
            command_owner=item.command_owner,
            selection_scope=item.selection_scope,
            switching_policy=item.switching_policy,
            applicable_phase_ids=item.applicable_phase_ids,
            lowering_chain=item.lowering_chain,
            scheme_id=item.scheme_id,
            scheme_layer=item.scheme_layer,
            consumer_roles=item.consumer_roles,
            streaming_preference=item.streaming_preference,
            ui_order=item.ui_order,
            claim_boundary=item.claim_boundary,
        )
        for item in realization.controls.authorities
        if item.availability == "available" and "step" in item.operations
    )
    output_channels = tuple(
        item
        for item in model.output_schema.channels
        if item.availability == "guaranteed" and (not item.operations or "step" in item.operations)
    )
    if not output_channels:
        # Legacy output metadata was historically batch-scoped.  A provider
        # that explicitly advertises a step tuple may still reuse those exact
        # channel definitions while it migrates the operations annotation.
        output_channels = tuple(
            item for item in model.output_schema.channels if item.availability == "guaranteed"
        )
    status_channels = tuple(_status_channel(item) for item in output_channels if item.data_type != "json")
    default_id = realization.controls.default_authority_id
    available_ids = {item.id for item in authorities}
    if default_id not in available_ids:
        default_id = authorities[0].id if authorities else None
    contract = VehicleInterfaceContract(
        vehicle_id=model.id,
        family_id=family_id,
        physical_family=physical_family,
        fidelity=cast(FidelityTier, fidelity),
        control_realization=realization.input_realization,
        evidence_status="reference_fixture",
        parameter_channels=(),
        action_channels=action_channels,
        effector_channels=(),
        status_channels=status_channels,
        resource_channels=(),
        diagnostic_channels=(),
        authority_profiles=authorities,
        observation_profiles=(
            ObservationProfile(
                id="truth_debug",
                availability="available",
                channel_ids=tuple(item.id for item in status_channels),
                description="All committed fixture truth channels at the accepted step boundary.",
                source="truth",
                claim_boundary=claim_boundary,
            ),
        ),
        execution_records=(),
        default_authority_profile_id=default_id,
        claim_boundary=claim_boundary,
    )
    status_by_id = {item.id: item for item in status_channels}
    episode_channels = tuple(
        EpisodeChannel(
            item.id,
            item.canonical_unit,
            _bound(item, "minimum"),
            _bound(item, "maximum"),
            item.description,
            (
                status_by_id[item.id].value_space
                if item.id in status_by_id
                else ValueSpaceSpec(
                    topology="finite_set",
                    representation="json object",
                    error_rule="provider-defined structural comparison",
                    interpolation_rule="not interpolable",
                )
            ),
            item.data_type,
            item.shape,
            item.sampling_semantics,
        )
        for item in output_channels
    )
    return contract, episode_channels
    ####


def _action_channel(channel: TrajectoryControlChannelMetadata) -> InterfaceChannel:
    value_type = _interface_value_type(
        data_type=channel.data_type,
        shape=channel.shape,
        event=channel.sampling_semantics == "event",
    )
    sampling: SamplingSemantics = "event" if channel.sampling_semantics == "event" else "held_action"
    canonical_unit = channel.canonical_unit
    if canonical_unit is None and value_type in {"scalar", "vector3", "vector4"}:
        canonical_unit = "dimensionless"
    feedback_channel_id = channel.provider_binding.get("feedback_channel_id")
    feedback_binding = (
        {"feedback_channel_id": feedback_channel_id}
        if isinstance(feedback_channel_id, str) and feedback_channel_id
        else {}
    )
    semantics = channel.semantics
    semantic_binding = {
        "command_mode": semantics.command_mode,
        "temporal_semantics": semantics.temporal_semantics,
        "repeat_policy": semantics.repeat_policy,
        "agent_normalization": semantics.agent_normalization,
        "agent_center": semantics.agent_center,
        "agent_scale": semantics.agent_scale,
        "agent_clip": semantics.agent_clip,
        "action_values": list(_control_action_values(channel)),
    }
    binding = (
        {
            "native_action": channel.native_channel_id,
            "choices": list(channel.choices),
            **feedback_binding,
            **semantic_binding,
        }
        if channel.native_channel_id is not None
        else {
            "episode_field": channel.id,
            "provider_binding": channel.provider_binding,
            "choices": list(channel.choices),
            **feedback_binding,
            **semantic_binding,
        }
    )
    return InterfaceChannel(
        id=channel.id,
        kind="action",
        value_type=value_type,
        canonical_unit=canonical_unit,
        description=channel.description,
        frame=channel.frame,
        lower=_bound(channel, "minimum"),
        upper=_bound(channel, "maximum"),
        availability="available",
        provenance="synthetic" if "probe" in channel.provenance.casefold() else "derived",
        sampling=sampling,
        binding=binding,
        claim_boundary=channel.claim_boundary,
        value_space=_value_space(channel.value_space, value_type),
    )
    ####


def _control_action_values(
    channel: TrajectoryControlChannelMetadata,
) -> tuple[float, ...]:
    """Materialize a finite numeric detent vocabulary for session clients."""

    quantization = channel.semantics.quantization
    if quantization.mode == "levels":
        return quantization.levels
    if quantization.mode != "step" or quantization.step is None:
        return ()
    lower = _bound(channel, "minimum")
    upper = _bound(channel, "maximum")
    if lower is None or upper is None:
        raise ValueError(f"step control {channel.id!r} requires finite bounds")
    first = math.ceil((lower - quantization.origin) / quantization.step - 1.0e-12)
    last = math.floor((upper - quantization.origin) / quantization.step + 1.0e-12)
    count = last - first + 1
    if count < 2 or count > 4096:
        raise ValueError(f"step control {channel.id!r} has an invalid detent count")
    return tuple(
        quantization.origin + index * quantization.step
        for index in range(first, last + 1)
    )
    ####


def _status_channel(channel: TrajectoryOutputChannelMetadata) -> InterfaceChannel:
    value_type = _interface_value_type(
        data_type=channel.data_type,
        shape=channel.shape,
        event=channel.sampling_semantics == "event" and channel.data_type == "string",
    )
    canonical_unit = channel.canonical_unit
    if canonical_unit is None and value_type in {"vector3", "vector4"}:
        canonical_unit = "dimensionless"
    return InterfaceChannel(
        id=channel.id,
        kind="status",
        value_type=value_type,
        canonical_unit=canonical_unit,
        quantity_semantics=(
            "count"
            if value_type == "scalar" and canonical_unit is None and channel.data_type == "int64"
            else None
        ),
        description=channel.description,
        frame=channel.frame,
        availability="available",
        provenance="derived",
        sampling="event" if channel.sampling_semantics == "event" else "truth_boundary",
        binding={"episode_field": channel.id},
        claim_boundary=channel.claim_boundary,
        value_space=_value_space(channel.value_space, value_type),
    )
    ####


def _interface_value_type(
    *,
    data_type: str,
    shape: tuple[int | str, ...],
    event: bool,
) -> InterfaceValueType:
    if event:
        return "event"
    if data_type == "boolean":
        return "boolean"
    if data_type == "string":
        return "enum"
    if shape == (3,):
        return "vector3"
    if shape == (4,):
        return "vector4"
    if shape:
        raise ValueError(f"session interface cannot represent trajectory shape {shape!r}")
    return "scalar"
    ####


def _bound(channel: object, side: str) -> float | None:
    interval = getattr(channel, "interval", None)
    endpoint = None if interval is None else getattr(interval, side)
    return None if endpoint is None else endpoint.value
    ####


def _value_space(source: object, value_type: InterfaceValueType) -> ValueSpaceSpec:
    """Translate the portable trajectory topology without guessing from IDs."""

    if source is None:
        return default_value_space_for_value_type(value_type)
    topology = str(getattr(source, "topology"))
    normalized = {
        "circle": "periodic_circle",
        "real_line": "euclidean",
        "unit_quaternion": "rotation_group_so3",
    }.get(topology, topology)
    if normalized not in {
        "euclidean",
        "bounded_interval",
        "periodic_circle",
        "unit_sphere",
        "rotation_group_so3",
        "positive_half_line",
        "unit_interval",
        "simplex",
        "finite_set",
        "boolean",
        "event",
        "product",
        "topology_pending",
    }:
        normalized = "euclidean"
    representation = {
        "scalar": "scalar source state code" if normalized == "event" else "scalar",
        "vector3": "vector3",
        "vector4": "vector4 quaternion" if normalized == "rotation_group_so3" else "vector4",
        "boolean": "boolean",
        "enum": "string",
        "event": "string",
    }[value_type]
    components = tuple(
        _value_space(item, "scalar") for item in getattr(source, "components", ())
    )
    return ValueSpaceSpec(
        topology=cast(ValueSpaceTopology, normalized),
        representation=representation,
        error_rule=str(getattr(source, "error_rule")),
        interpolation_rule=str(getattr(source, "interpolation_rule")),
        normalization_rule=getattr(source, "normalization_rule", None),
        period=getattr(source, "period", None) if normalized == "periodic_circle" else None,
        equivalence=getattr(source, "equivalence", None),
        coordinate_chart=getattr(source, "coordinate_chart", None),
        components=components if normalized == "product" else (),
    )
    ####


__all__ = ["build_session_interface_contract"]
