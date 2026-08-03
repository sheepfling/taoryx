"""Resolved semantic interface contracts for selected vehicle realizations.

The composition registry declares which initializations, segments, and
missions a family advertises. Family adapters declare native plant channels.
Neither is, by itself, a stable caller-facing contract: native controls and
state-vector names differ materially between a fixed wing, a multirotor, a
rocket, and a passive body.

This module joins existing authorities into a read-only, versioned
VehicleInterfaceContract. It intentionally does not add a second plant,
infer missing sensors, or promote a source control coordinate into a
physical actuator. Every unavailable channel remains explicit instead of
appearing in a runtime sample as a convenient zero.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Literal

from .fidelity_contracts import FidelityTier
from .vehicle_composition_registry import resolved_control_realization_for
from .vehicle_execution_bindings import bindings_for_family

if TYPE_CHECKING:
    from .vehicle_composition_registry import CompositionParameter, ResolvedVehicleComposition, ResolvedVehicleCompositionCatalog


InterfaceAvailability = Literal[
    "available",
    "available_in_batch",
    "not_applicable",
    "not_available",
    "planned",
    "unavailable_at_runtime",
]
InterfaceChannelKind = Literal[
    "parameter",
    "action",
    "effector",
    "status",
    "observation",
    "resource",
    "diagnostic",
]
InterfaceValueType = Literal["scalar", "vector3", "vector4", "boolean", "enum", "event"]
ParameterScope = Literal[
    "family_model",
    "variant_configuration",
    "episode_reset",
    "segment",
    "step_action",
    "derived_status",
]
AuthorityKind = Literal["mission", "kinematic", "body_motion", "wrench", "effector", "native_bridge", "open_loop"]
SamplingSemantics = Literal["truth_boundary", "held_action", "reset_only", "segment_transition", "event", "not_sampled"]
ProvenanceKind = Literal["source_backed", "derived", "engineering_surrogate", "synthetic", "replayed", "not_applicable"]

SCHEMA_ID = "taoryx.vehicle-interface/v1alpha1"


@dataclass(frozen=True, slots=True)
class InterfaceChannel:
    """One stable semantic channel and its exact native binding boundary."""

    id: str
    kind: InterfaceChannelKind
    value_type: InterfaceValueType
    canonical_unit: str | None
    description: str
    scope: ParameterScope | None = None
    frame: str | None = None
    lower: float | None = None
    upper: float | None = None
    availability: InterfaceAvailability = "available"
    provenance: ProvenanceKind = "derived"
    sampling: SamplingSemantics = "truth_boundary"
    binding: Mapping[str, object] = field(default_factory=dict)
    claim_boundary: str = ""

    def __post_init__(self) -> None:
        if not self.id.strip() or not self.description.strip():
            raise ValueError("interface channels require a stable ID and description")
        if self.kind == "parameter" and self.scope is None:
            raise ValueError(f"parameter channel {self.id!r} requires a mutability scope")
        if self.kind != "parameter" and self.scope is not None:
            raise ValueError(f"non-parameter channel {self.id!r} cannot declare a parameter scope")
        if self.lower is not None and self.upper is not None and self.lower > self.upper:
            raise ValueError(f"interface channel {self.id!r} has inverted bounds")
        if self.availability == "available" and not self.claim_boundary.strip():
            raise ValueError(f"available channel {self.id!r} requires a claim boundary")
        ####
    ####

    def as_dict(self) -> dict[str, object]:
        """Return the channel as a serialized interface-schema entry."""

        return {
            "id": self.id,
            "kind": self.kind,
            "value_type": self.value_type,
            "canonical_unit": self.canonical_unit,
            "frame": self.frame,
            "lower": self.lower,
            "upper": self.upper,
            "availability": self.availability,
            "provenance": self.provenance,
            "sampling": self.sampling,
            "scope": self.scope,
            "description": self.description,
            "binding": dict(self.binding),
            "claim_boundary": self.claim_boundary,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class AuthorityProfile:
    """One mutually exclusive action authority offered by a realization."""

    id: str
    authority: AuthorityKind
    availability: InterfaceAvailability
    action_ids: tuple[str, ...]
    description: str
    claim_boundary: str

    def __post_init__(self) -> None:
        if not self.id.strip() or not self.description.strip() or not self.claim_boundary.strip():
            raise ValueError("authority profiles require identity, description, and claim boundary")
        if len(self.action_ids) != len(set(self.action_ids)):
            raise ValueError(f"authority profile {self.id!r} has duplicate action IDs")
        ####
    ####

    def as_dict(self) -> dict[str, object]:
        """Return a portable authority-profile record."""

        return {
            "id": self.id,
            "authority": self.authority,
            "availability": self.availability,
            "action_ids": list(self.action_ids),
            "description": self.description,
            "claim_boundary": self.claim_boundary,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class ObservationProfile:
    """A declared view of committed truth for one external consumer."""

    id: str
    availability: InterfaceAvailability
    channel_ids: tuple[str, ...]
    description: str
    source: Literal["truth", "sensor", "replay", "none"]
    claim_boundary: str
    cadence_s: float | None = None
    latency_s: float | None = None
    channel_errors: Mapping[str, Mapping[str, float | None]] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.id.strip() or not self.description.strip() or not self.claim_boundary.strip():
            raise ValueError("observation profiles require identity, description, and claim boundary")
        if len(self.channel_ids) != len(set(self.channel_ids)):
            raise ValueError(f"observation profile {self.id!r} has duplicate channels")
        if self.source == "sensor" and self.availability == "available":
            if self.cadence_s is None or not math.isfinite(self.cadence_s) or self.cadence_s <= 0.0:
                raise ValueError(f"available sensor profile {self.id!r} requires a positive cadence_s")
            if self.latency_s is None or not math.isfinite(self.latency_s) or self.latency_s < 0.0:
                raise ValueError(f"available sensor profile {self.id!r} requires a nonnegative latency_s")
        if self.source != "sensor" and self.channel_errors:
            raise ValueError(f"non-sensor observation profile {self.id!r} cannot declare measurement errors")
        ####
    ####

    def as_dict(self) -> dict[str, object]:
        """Return the portable observation-profile record."""

        return {
            "id": self.id,
            "availability": self.availability,
            "channel_ids": list(self.channel_ids),
            "description": self.description,
            "source": self.source,
            "claim_boundary": self.claim_boundary,
            "cadence_s": self.cadence_s,
            "latency_s": self.latency_s,
            "channel_errors": {identifier: dict(error) for identifier, error in self.channel_errors.items()},
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class VehicleInterfaceContract:
    """Immutable caller-facing schema for one family/fidelity realization."""

    vehicle_id: str
    family_id: str
    physical_family: str
    fidelity: FidelityTier
    control_realization: str
    evidence_status: str
    parameter_channels: tuple[InterfaceChannel, ...]
    action_channels: tuple[InterfaceChannel, ...]
    effector_channels: tuple[InterfaceChannel, ...]
    status_channels: tuple[InterfaceChannel, ...]
    resource_channels: tuple[InterfaceChannel, ...]
    diagnostic_channels: tuple[InterfaceChannel, ...]
    authority_profiles: tuple[AuthorityProfile, ...]
    observation_profiles: tuple[ObservationProfile, ...]
    execution_records: tuple[Mapping[str, object], ...]
    claim_boundary: str
    schema: str = SCHEMA_ID

    def __post_init__(self) -> None:
        if not self.vehicle_id.strip() or not self.family_id.strip() or not self.claim_boundary.strip():
            raise ValueError("vehicle interface requires vehicle, family, and claim-boundary metadata")
        channel_groups = (
            self.parameter_channels,
            self.action_channels,
            self.effector_channels,
            self.status_channels,
            self.resource_channels,
            self.diagnostic_channels,
        )
        for channels in channel_groups:
            _require_unique_ids(channels)
        action_ids = {item.id for item in self.action_channels}
        for profile in self.authority_profiles:
            unknown = sorted(set(profile.action_ids) - action_ids)
            if unknown:
                raise ValueError(f"authority profile {profile.id!r} references unknown actions: {unknown}")
        observable_ids = {
            *(item.id for item in self.status_channels),
            *(item.id for item in self.resource_channels),
            *(item.id for item in self.diagnostic_channels),
        }
        for observation_profile in self.observation_profiles:
            unknown = sorted(set(observation_profile.channel_ids) - observable_ids)
            if unknown:
                raise ValueError(f"observation profile {observation_profile.id!r} references unknown channels: {unknown}")
            if observation_profile.channel_errors:
                _normalize_sensor_channel_errors(
                    channel_ids=observation_profile.channel_ids,
                    channel_descriptors={
                        item.id: item
                        for item in (*self.status_channels, *self.resource_channels, *self.diagnostic_channels)
                    },
                    channel_errors=observation_profile.channel_errors,
                )
        ####
    ####

    @property
    def id(self) -> str:
        """Return the stable realization identity without a content hash."""

        return f"{self.family_id}/{self.fidelity}"
        ####
    ####

    @property
    def fingerprint(self) -> str:
        """Return a reproducibility hash over all declared semantic metadata."""

        encoded = json.dumps(self._payload(), sort_keys=True, separators=(",", ":")).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()
        ####
    ####

    def authority_profile(self, identifier: str) -> AuthorityProfile:
        """Resolve one declared profile or return a clear discovery error."""

        for profile in self.authority_profiles:
            if profile.id == identifier:
                return profile
        raise KeyError(f"{self.id}: unknown authority profile {identifier!r}")
        ####
    ####

    def observation_profile(self, identifier: str) -> ObservationProfile:
        """Resolve one declared observation profile or return a clear error."""

        for profile in self.observation_profiles:
            if profile.id == identifier:
                return profile
        raise KeyError(f"{self.id}: unknown observation profile {identifier!r}")
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the complete portable contract for CLI, UI, and artifacts."""

        return {**self._payload(), "interface_id": self.id, "fingerprint_sha256": self.fingerprint}
        ####

    def _payload(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "vehicle_id": self.vehicle_id,
            "family_id": self.family_id,
            "physical_family": self.physical_family,
            "fidelity": self.fidelity,
            "control_realization": self.control_realization,
            "evidence_status": self.evidence_status,
            "parameters": [item.as_dict() for item in self.parameter_channels],
            "actions": [item.as_dict() for item in self.action_channels],
            "effectors": [item.as_dict() for item in self.effector_channels],
            "status": [item.as_dict() for item in self.status_channels],
            "resources": [item.as_dict() for item in self.resource_channels],
            "diagnostics": [item.as_dict() for item in self.diagnostic_channels],
            "authority_profiles": [item.as_dict() for item in self.authority_profiles],
            "observation_profiles": [item.as_dict() for item in self.observation_profiles],
            "execution_records": [dict(item) for item in self.execution_records],
            "claim_boundary": self.claim_boundary,
        }
        ####
    ####


def resolve_vehicle_interface_contract(
    identifier: str,
    fidelity: FidelityTier,
    *,
    catalog: ResolvedVehicleCompositionCatalog | None = None,
) -> VehicleInterfaceContract:
    """Resolve one versioned interface without instantiating a simulation."""

    if catalog is None:
        from .vehicle_composition_registry import load_resolved_vehicle_composition_catalog

        catalog = load_resolved_vehicle_composition_catalog()
    return interface_contract_for_composition(catalog.vehicle(identifier), fidelity)
    ####


def interface_contract_for_composition(
    composition: ResolvedVehicleComposition,
    fidelity: FidelityTier,
) -> VehicleInterfaceContract:
    """Project one registered family/fidelity into explicit semantic channels."""

    family = composition.family
    tier = family.family.tiers[fidelity]
    vehicle_id = family.family.vehicle_registry_id or family.family_id
    episode_runnable = _has_runnable_episode(family.family_id, fidelity)
    batch_runnable = _has_runnable_batch(family.family_id, fidelity)
    action_channels, authority_profiles = _action_contract(family.family_id, fidelity, episode_runnable)
    status, resources, diagnostics = _status_contract(
        family.family_id,
        fidelity,
        episode_runnable,
        batch_runnable,
    )
    execution_records = tuple(
        item.model_dump(mode="json")
        for item in bindings_for_family(family.family_id)
        if item.fidelity == fidelity
    )
    return VehicleInterfaceContract(
        vehicle_id=vehicle_id,
        family_id=family.family_id,
        physical_family=family.family.physical_family,
        fidelity=fidelity,
        control_realization=resolved_control_realization_for(family.family_id, fidelity),
        evidence_status=tier.promotion_status,
        parameter_channels=_parameter_channels(composition),
        action_channels=action_channels,
        effector_channels=_effector_contract(family.family_id, fidelity, family.vehicle_definition, tier.profile_id is not None),
        status_channels=status,
        resource_channels=resources,
        diagnostic_channels=diagnostics,
        authority_profiles=authority_profiles,
        observation_profiles=_observation_contract(status, resources, diagnostics),
        execution_records=execution_records,
        claim_boundary=_contract_claim_boundary(family.family_id, fidelity, tier.promotion_status),
    )
    ####


def _parameter_channels(composition: ResolvedVehicleComposition) -> tuple[InterfaceChannel, ...]:
    channels: list[InterfaceChannel] = []
    for initialization in composition.declaration.initialization_contracts:
        channels.extend(
            _parameter_channel(
                item,
                scope="episode_reset",
                id_prefix=f"initialization.{initialization.id}",
                binding={"initialization_contract": initialization.id, "parameter_id": item.id},
            )
            for item in initialization.parameters
        )
    for segment in composition.declaration.segment_contracts:
        channels.extend(
            _parameter_channel(
                item,
                scope="segment",
                id_prefix=f"segment.{segment.id}",
                binding={"segment_contract": segment.id, "parameter_id": item.id},
            )
            for item in segment.parameters
        )
    return tuple(channels)
    ####


def _parameter_channel(
    parameter: CompositionParameter,
    *,
    scope: ParameterScope,
    id_prefix: str,
    binding: Mapping[str, str],
) -> InterfaceChannel:
    return InterfaceChannel(
        id=f"{id_prefix}.{parameter.id}",
        kind="parameter",
        value_type="vector3" if parameter.id.endswith("_ned_m") else "scalar",
        canonical_unit=parameter.canonical_unit,
        description=parameter.description,
        scope=scope,
        availability="available",
        provenance="derived",
        sampling="reset_only" if scope == "episode_reset" else "segment_transition",
        binding=binding,
        claim_boundary=(
            "This is a declared composition input. Coupled physics, trim, and "
            "qualification requirements remain the selected family adapter's responsibility."
        ),
    )
    ####


def _action_contract(
    family_id: str,
    fidelity: FidelityTier,
    episode_runnable: bool,
) -> tuple[tuple[InterfaceChannel, ...], tuple[AuthorityProfile, ...]]:
    if family_id == "hummingbird" and fidelity == "pseudo_6dof":
        available: InterfaceAvailability = "available" if episode_runnable else "unavailable_at_runtime"
        hummingbird_channels: tuple[InterfaceChannel, ...] = (
            _action("attitude.roll.command", "rad", -1.5707963267948966, 1.5707963267948966, "Commanded roll angle accepted by the named aggregate-thrust response law.", "roll_rad", available),
            _action("attitude.pitch.command", "rad", -1.5707963267948966, 1.5707963267948966, "Commanded pitch angle accepted by the named aggregate-thrust response law.", "pitch_rad", available),
            _action("attitude.yaw.command", "rad", -3.141592653589793, 3.141592653589793, "Commanded yaw angle accepted by the named aggregate-thrust response law.", "yaw_rad", available),
            _action("propulsion.command.fraction", "dimensionless", 0.0, 1.0, "Aggregate thrust fraction, not an individual rotor command.", "thrust_ratio", available),
            InterfaceChannel(
                "propulsion.enable",
                "action",
                "boolean",
                None,
                "Aggregate motor-enable state for the response-law plant.",
                availability=available,
                provenance="engineering_surrogate",
                sampling="held_action",
                binding={"native_action": "motors_enabled"},
                claim_boundary="This is aggregate enable state only; it does not establish individual motor allocation.",
            ),
        )
        return hummingbird_channels, (
            AuthorityProfile(
                "body_motion_response",
                "body_motion",
                available,
                tuple(item.id for item in hummingbird_channels),
                "Bounded roll, pitch, yaw, and aggregate-thrust response command.",
                "Pseudo-6DOF body-motion response only; no individual rotor, motor, or moment-balance claim.",
            ),
        )

    fixed_wing_controls = _fixed_wing_bridge_controls(family_id)
    if fixed_wing_controls and fidelity in {"point_mass_3dof", "pseudo_6dof"}:
        available = "available" if episode_runnable else "unavailable_at_runtime"
        bridge_channels: tuple[InterfaceChannel, ...] = tuple(
            _action(identifier, unit, lower, upper, description, native, available)
            for identifier, unit, lower, upper, description, native in fixed_wing_controls
        )
        return bridge_channels, (
            AuthorityProfile(
                "native_control_bridge",
                "native_bridge",
                available,
                tuple(item.id for item in bridge_channels),
                "Explicit source-runtime control coordinates projected into stable semantic names.",
                "The bridge preserves the selected point-mass or response-law claim. It does not prove physical actuator allocation, servo dynamics, or moment balance.",
            ),
        )

    if fidelity == "rigid_body_6dof_direct_wrench":
        direct_available: InterfaceAvailability = "available" if episode_runnable else "planned"
        wrench_channels: tuple[InterfaceChannel, ...] = (
            InterfaceChannel(
                "wrench.force.command",
                "action",
                "vector3",
                "N",
                "Requested total body-frame force for an explicit direct-wrench bridge.",
                availability=direct_available,
                provenance="engineering_surrogate",
                sampling="held_action",
                binding={"frame": "body", "native_action": "force_body_n"},
                claim_boundary=(
                    "The command is a total direct body force, including any declared local source-load bridge bias. "
                    "It is not an actuator, surface, rotor, gimbal, or thruster command."
                ),
            ),
            InterfaceChannel(
                "wrench.moment.command",
                "action",
                "vector3",
                "N*m",
                "Requested total body-frame moment for an explicit direct-wrench bridge.",
                availability=direct_available,
                provenance="engineering_surrogate",
                sampling="held_action",
                binding={"frame": "body", "native_action": "moment_body_nm"},
                claim_boundary=(
                    "The command is a total direct body moment, including any declared local source-load bridge bias. "
                    "It is not physical effector allocation."
                ),
            ),
        )
        return wrench_channels, (
            AuthorityProfile(
                "direct_wrench",
                "wrench",
                direct_available,
                tuple(item.id for item in wrench_channels),
                "Six-axis direct-wrench request for a declared rigid-body bridge.",
                (
                    "This profile is available only when an exact source-local episode binding and achieved-wrench "
                    "telemetry exist. It remains bridge/screen evidence, never physical-effector allocation."
                    if episode_runnable
                    else "This profile remains unavailable to an episode until an exact native binding and achieved-wrench telemetry exist."
                ),
            ),
        )

    return (), (
        AuthorityProfile(
            "no_external_action",
            "open_loop",
            "not_applicable" if family_id == "tumbling_body" else "not_available",
            (),
            "No declared external action profile is available for this selected realization.",
            "The selected realization must not borrow controls from a neighboring family or fidelity.",
        ),
    )
    ####


def _fixed_wing_bridge_controls(
    family_id: str,
) -> tuple[tuple[str, str, float, float, str, str], ...]:
    if family_id == "skywalker_x8":
        return (
            ("propulsion.command.fraction", "dimensionless", 0.0, 1.0, "Source-runtime propulsion control fraction.", "throttle"),
            ("control.longitudinal.bridge.command", "deg", -20.0, 20.0, "Collective elevon source-table coordinate accepted by the selected runtime.", "collective-elevon-deg"),
            ("control.lateral.bridge.command", "deg", -20.0, 20.0, "Differential elevon source-table coordinate accepted by the selected runtime.", "differential-elevon-deg"),
        )
    if family_id == "b747":
        return (
            ("propulsion.command.fraction", "dimensionless", 0.0, 1.0, "Source-runtime propulsion control fraction.", "throttle"),
            ("control.longitudinal.bridge.command", "deg", -10.0, 10.0, "Elevator source-table coordinate accepted by the selected runtime.", "elevator-deg"),
        )
    return ()
    ####


def _action(
    identifier: str,
    unit: str,
    lower: float,
    upper: float,
    description: str,
    native: str,
    availability: InterfaceAvailability,
) -> InterfaceChannel:
    return InterfaceChannel(
        identifier,
        "action",
        "scalar",
        unit,
        description,
        lower=lower,
        upper=upper,
        availability=availability,
        provenance="source_backed",
        sampling="held_action",
        binding={"native_action": native},
        claim_boundary="This action binds to the named source/runtime coordinate. The selected lower fidelity does not turn that coordinate into physical actuator evidence.",
    )
    ####


def _effector_contract(
    family_id: str,
    fidelity: FidelityTier,
    vehicle_definition: Mapping[str, Any] | None,
    tier_declared: bool,
) -> tuple[InterfaceChannel, ...]:
    if fidelity != "rigid_body_6dof_surface_allocated":
        return ()
    controls = vehicle_definition.get("controls") if isinstance(vehicle_definition, Mapping) else None
    if not isinstance(controls, list | tuple):
        return ()
    availability: InterfaceAvailability = "planned" if tier_declared else "not_available"
    channels: list[InterfaceChannel] = []
    for item in controls:
        if not isinstance(item, Mapping) or not isinstance(item.get("name"), str):
            continue
        name = str(item["name"])
        channels.append(
            InterfaceChannel(
                f"effector.{_canonical_effector_name(name)}.command",
                "effector",
                "scalar",
                "deg" if name.endswith("-deg") else "dimensionless",
                f"Commanded {name} control coordinate.",
                lower=_optional_number(item.get("lower")),
                upper=_optional_number(item.get("upper")),
                availability=availability,
                provenance="source_backed",
                sampling="held_action",
                binding={"native_effector": name, "family_id": family_id},
                claim_boundary="A surface-allocated profile is declared, but this channel is not physical allocation evidence until an exact runtime binding logs commanded/actual positions, limits, and achieved wrench.",
            )
        )
    return tuple(channels)
    ####


def _status_contract(
    family_id: str,
    fidelity: FidelityTier,
    episode_runnable: bool,
    batch_runnable: bool,
) -> tuple[tuple[InterfaceChannel, ...], tuple[InterfaceChannel, ...], tuple[InterfaceChannel, ...]]:
    runtime_availability: InterfaceAvailability = (
        "available"
        if episode_runnable
        else "available_in_batch"
        if batch_runnable
        else "unavailable_at_runtime"
    )
    execution_time_binding = (
        {"episode_field": "time_s"}
        if episode_runnable
        else {"batch_telemetry": "time_s"}
        if batch_runnable
        else {"episode_field": "time_s"}
    )
    execution_status_binding = (
        {"episode_field": "status"}
        if episode_runnable
        else {"batch_report": "status"}
        if batch_runnable
        else {"episode_field": "status"}
    )
    status = [
        InterfaceChannel(
            "execution.time",
            "status",
            "scalar",
            "s",
            "Committed truth timestamp at the external episode boundary.",
            availability=runtime_availability,
            provenance="derived",
            sampling="truth_boundary",
            binding=execution_time_binding,
            claim_boundary=(
                "The timestamp is a committed truth boundary; no future-state interpolation is implied. "
                "Batch-only availability does not make this a policy observation."
            ),
        ),
        InterfaceChannel(
            "execution.status",
            "status",
            "enum",
            None,
            "Episode lifecycle status at the committed truth boundary.",
            availability=runtime_availability,
            provenance="derived",
            sampling="truth_boundary",
            binding=execution_status_binding,
            claim_boundary="This is runtime lifecycle state, not a mission-qualification result.",
        ),
    ]
    resources: list[InterfaceChannel] = []
    diagnostics: list[InterfaceChannel] = []
    if family_id == "hummingbird" and fidelity == "pseudo_6dof":
        status.extend(
            (
                _status("position.north", "m", "Committed NED north position.", "position_ned_m[0]", runtime_availability),
                _status("position.east", "m", "Committed NED east position.", "position_ned_m[1]", runtime_availability),
                _status("position.altitude", "m", "Committed altitude above the NED origin.", "position_ned_m[2] (sign-inverted)", runtime_availability),
                _status("velocity.north", "m/s", "Committed NED north velocity.", "velocity_ned_m_s[0]", runtime_availability),
                _status("velocity.east", "m/s", "Committed NED east velocity.", "velocity_ned_m_s[1]", runtime_availability),
                _status("velocity.down", "m/s", "Committed NED down velocity.", "velocity_ned_m_s[2]", runtime_availability),
                _status("attitude.euler", "rad", "Declared pseudo-6DOF Euler attitude response.", "attitude_rad", runtime_availability, value_type="vector3"),
                _status("body_rate", "rad/s", "Declared pseudo-6DOF body-rate response.", "body_rate_rad_s", runtime_availability, value_type="vector3"),
                _status("contact.state", None, "Declared ground-contact state.", "contact", runtime_availability, value_type="boolean"),
            )
        )
        resources.append(
            InterfaceChannel(
                "resources.battery.fraction_remaining",
                "resource",
                "scalar",
                "dimensionless",
                "Declared bounded engineering battery reserve.",
                lower=0.0,
                upper=1.0,
                availability=runtime_availability,
                provenance="engineering_surrogate",
                sampling="truth_boundary",
                binding={"episode_value": "battery_fraction"},
                claim_boundary="This is the pseudo-plant reserve model; it is not a cell-voltage or motor-current claim.",
            )
        )
        status.append(_status("propulsion.output.thrust.aggregate", "N", "Achieved aggregate thrust in the pseudo response law.", "aggregate_thrust_n", runtime_availability))
        diagnostics.extend((_diagnostic("control.realization", "control_realization", runtime_availability), _diagnostic("control.physical_motor_allocation", "physical_motor_allocation", runtime_availability)))
    elif family_id in {"skywalker_x8", "b747"} and fidelity in {"point_mass_3dof", "pseudo_6dof"}:
        status.extend(
            (
                _status("position.altitude", "m", "Geodetic altitude from the committed runtime state.", "1.alt", runtime_availability, scale=0.3048),
                _status("velocity.speed", "m/s", "Scalar speed from the committed runtime state.", "1.vel", runtime_availability, scale=0.3048),
                _status("flight.path_angle", "deg", "Geodetic flight-path angle from the committed runtime state.", "1.gama", runtime_availability),
                _status("flight.heading", "deg", "Geodetic heading from the committed runtime state.", "1.psi", runtime_availability),
            )
        )
        resources.append(
            InterfaceChannel(
                "resources.mass.total",
                "resource",
                "scalar",
                "kg",
                "Modeled total mass exposed by the committed runtime state.",
                availability=runtime_availability,
                provenance="source_backed",
                sampling="truth_boundary",
                binding={"runtime_state": "1.mass", "scale": 0.45359237, "source_unit": "lb"},
                claim_boundary="The current racetrack bindings retain fixed mass; this channel does not establish fuel depletion.",
            )
        )
        diagnostics.append(
            _diagnostic(
                "control.realization",
                resolved_control_realization_for(family_id, fidelity),
                runtime_availability,
            )
        )
    elif family_id in {"a320_openap_3dof", "f16_s119"} and fidelity in {"point_mass_3dof", "pseudo_6dof"}:
        status.extend(
            (
                _status(
                    "position.north",
                    "m",
                    "Committed local navigation north position.",
                    "north_m",
                    runtime_availability,
                    binding={"batch_telemetry": "north_m", "frame": "local_navigation"},
                    provenance="derived",
                ),
                _status(
                    "position.east",
                    "m",
                    "Committed local navigation east position.",
                    "east_m",
                    runtime_availability,
                    binding={"batch_telemetry": "east_m", "frame": "local_navigation"},
                    provenance="derived",
                ),
                _status(
                    "position.altitude",
                    "m",
                    "Committed altitude above the local navigation origin.",
                    "altitude_m",
                    runtime_availability,
                    binding={"batch_telemetry": "altitude_m", "frame": "local_navigation"},
                    provenance="derived",
                ),
                _status(
                    "velocity.speed",
                    "m/s",
                    "Committed scalar airspeed represented by the reduced model.",
                    "speed_m_s",
                    runtime_availability,
                    binding={"batch_telemetry": "speed_m_s"},
                    provenance="derived",
                ),
                _status(
                    "flight.path_angle",
                    "deg",
                    "Committed navigation flight-path angle from the reduced model.",
                    "flight_path_angle_rad",
                    runtime_availability,
                    binding={
                        "derived_from": "batch_telemetry.flight_path_angle_rad",
                        "transform": "rad_to_deg",
                    },
                    provenance="derived",
                ),
                _status(
                    "flight.heading",
                    "deg",
                    "Committed navigation heading from the reduced model.",
                    "heading_rad",
                    runtime_availability,
                    binding={"derived_from": "batch_telemetry.heading_rad", "transform": "rad_to_deg"},
                    provenance="derived",
                ),
                _status(
                    "aero.dynamic_pressure",
                    "Pa",
                    "Dynamic pressure evaluated by the selected reduced source model.",
                    "dynamic_pressure_pa",
                    runtime_availability,
                    binding={"batch_telemetry": "dynamic_pressure_pa"},
                    provenance="derived",
                ),
            )
        )
        if fidelity == "pseudo_6dof":
            status.extend(
                (
                    _status(
                        "attitude.euler",
                        "deg",
                        "Named pseudo-6DOF achieved attitude response.",
                        "attitude_euler_deg",
                        runtime_availability,
                        value_type="vector3",
                        binding={"batch_telemetry": "attitude_euler_deg"},
                        provenance="engineering_surrogate",
                    ),
                    _status(
                        "body_rate",
                        "rad/s",
                        "Named pseudo-6DOF achieved body-rate response.",
                        "body_rate_rad_s",
                        runtime_availability,
                        value_type="vector3",
                        binding={"batch_telemetry": "body_rate_rad_s"},
                        provenance="engineering_surrogate",
                    ),
                )
            )
        resources.append(
            InterfaceChannel(
                "resources.mass.total",
                "resource",
                "scalar",
                "kg",
                "Modeled total mass retained by the committed reduced-model state.",
                availability=runtime_availability,
                provenance="derived",
                sampling="truth_boundary",
                binding={"batch_telemetry": "mass_kg"},
                claim_boundary=(
                    "The A320 reduced state reports modeled total mass, not an independent fuel-system ledger."
                    if family_id == "a320_openap_3dof"
                    else "The F-16 local source reduction retains a fixed source mass; this does not claim fuel depletion."
                ),
            )
        )
        diagnostics.append(
            _diagnostic(
                "control.realization",
                resolved_control_realization_for(family_id, fidelity),
                runtime_availability,
            )
        )
    elif family_id == "reference_nesc_two_stage_rocket" and fidelity in {"point_mass_3dof", "pseudo_6dof"}:
        status.extend(
            (
                _status(
                    "position.inertial",
                    "m",
                    "Committed ECI position from the retained staged replay.",
                    "position_eci_m",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "position_eci_m", "frame": "eci"},
                    provenance="replayed",
                ),
                _status(
                    "velocity.inertial",
                    "m/s",
                    "Committed ECI velocity from the retained staged replay.",
                    "velocity_eci_mps",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "velocity_eci_mps", "frame": "eci"},
                    provenance="replayed",
                ),
                _status(
                    "velocity.speed",
                    "m/s",
                    "Magnitude of the committed ECI replay velocity.",
                    "velocity_eci_mps",
                    runtime_availability,
                    binding={"derived_from": "batch_telemetry.velocity_eci_mps", "transform": "norm"},
                    provenance="replayed",
                ),
                _status(
                    "phase.mode",
                    None,
                    "Committed staged replay phase.",
                    "phase",
                    runtime_availability,
                    value_type="enum",
                    binding={"batch_telemetry": "phase"},
                    provenance="replayed",
                ),
            )
        )
        if fidelity == "pseudo_6dof":
            status.extend(
                (
                    _status(
                        "attitude.euler",
                        "rad",
                        "Named pseudo-6DOF replay attitude response.",
                        "achieved_attitude_rad",
                        runtime_availability,
                        value_type="vector3",
                        binding={"batch_telemetry": "achieved_attitude_rad"},
                        provenance="engineering_surrogate",
                    ),
                    _status(
                        "body_rate",
                        "rad/s",
                        "Named pseudo-6DOF replay angular-rate response.",
                        "body_rate_rad_s",
                        runtime_availability,
                        value_type="vector3",
                        binding={"batch_telemetry": "body_rate_rad_s"},
                        provenance="engineering_surrogate",
                    ),
                )
            )
        resources.append(
            InterfaceChannel(
                "resources.mass.total",
                "resource",
                "scalar",
                "kg",
                "Committed total mass from the retained staged replay.",
                availability=runtime_availability,
                provenance="replayed",
                sampling="truth_boundary",
                binding={"batch_telemetry": "mass_kg"},
                claim_boundary=(
                    "The retained replay exposes total mass only. It does not infer stage-specific propellant "
                    "quantities, thrust control, or a participating gimbal model."
                ),
            )
        )
        diagnostics.append(
            _diagnostic(
                "control.realization",
                "control_realization",
                runtime_availability,
                binding={"batch_report": "control_realization"},
            )
        )
    elif family_id == "x15" and fidelity in {"point_mass_3dof", "pseudo_6dof"}:
        status.extend(
            (
                _status(
                    "position.local",
                    "m",
                    "Committed local-frame reduced position.",
                    "position_m",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "position_m", "frame": "local_reduced"},
                    provenance="engineering_surrogate",
                ),
                _status(
                    "position.altitude",
                    "m",
                    "Committed local reduced altitude above the launch plane.",
                    "position_m[2]",
                    runtime_availability,
                    binding={"batch_telemetry": "position_m[2]", "frame": "local_reduced"},
                    provenance="engineering_surrogate",
                ),
                _status(
                    "velocity.local",
                    "m/s",
                    "Committed local-frame reduced velocity.",
                    "velocity_m_s",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "velocity_m_s", "frame": "local_reduced"},
                    provenance="engineering_surrogate",
                ),
                _status(
                    "velocity.speed",
                    "m/s",
                    "Magnitude of the committed local reduced velocity.",
                    "velocity_m_s",
                    runtime_availability,
                    binding={"derived_from": "batch_telemetry.velocity_m_s", "transform": "norm"},
                    provenance="engineering_surrogate",
                ),
                _status(
                    "phase.mode",
                    None,
                    "Committed reduced staged phase: boost, coast, or glide.",
                    "phase",
                    runtime_availability,
                    value_type="enum",
                    binding={"batch_telemetry": "phase"},
                    provenance="engineering_surrogate",
                ),
            )
        )
        if fidelity == "pseudo_6dof":
            status.extend(
                (
                    _status(
                        "attitude.euler",
                        "rad",
                        "Named reduced attitude-response state.",
                        "attitude_rad",
                        runtime_availability,
                        value_type="vector3",
                        binding={"batch_telemetry": "attitude_rad"},
                        provenance="engineering_surrogate",
                    ),
                    _status(
                        "body_rate",
                        "rad/s",
                        "Named reduced attitude-response angular-rate state.",
                        "attitude_rate_rad_s",
                        runtime_availability,
                        value_type="vector3",
                        binding={"batch_telemetry": "attitude_rate_rad_s"},
                        provenance="engineering_surrogate",
                    ),
                )
            )
        resources.extend(
            (
                InterfaceChannel(
                    "resources.mass.total",
                    "resource",
                    "scalar",
                    "kg",
                    "Reduced stacked or released-vehicle mass at the committed truth state.",
                    availability=runtime_availability,
                    provenance="engineering_surrogate",
                    sampling="truth_boundary",
                    binding={"batch_telemetry": "mass_kg"},
                    claim_boundary=(
                        "This is the local reduced staged mass schedule. It records the source-pinned cutoff and "
                        "release closure but is not a native X-15 mass-property claim."
                    ),
                ),
                InterfaceChannel(
                    "resources.booster.attached",
                    "resource",
                    "boolean",
                    None,
                    "Whether the reduced witness is before the declared passive booster-release transition.",
                    availability=runtime_availability,
                    provenance="engineering_surrogate",
                    sampling="truth_boundary",
                    binding={"derived_from": "batch_telemetry.phase", "transform": "phase_not_glide"},
                    claim_boundary="This reports only the declared reduced staging state; it is not physical separation dynamics.",
                ),
                InterfaceChannel(
                    "resources.booster.propellant.consumed",
                    "resource",
                    "scalar",
                    "kg",
                    "Source-pinned booster propellant consumed before cutoff.",
                    lower=0.0,
                    upper=2_000.0,
                    availability=runtime_availability,
                    provenance="source_backed",
                    sampling="truth_boundary",
                    binding={
                        "derived_from": "batch_telemetry.time_s",
                        "transform": "100_kg_s_times_min_time_20_s",
                        "source": "examples/showcases/x15_rocket_to_hawaii/mission.prb",
                    },
                    claim_boundary=(
                        "This records 20 s at 100 kg/s from the retained source segment. The source deck's separate "
                        "9,000 kg declared propellant field remains an explicit discrepancy, not additional consumed mass."
                    ),
                ),
            )
        )
        diagnostics.append(
            _diagnostic(
                "control.realization",
                "control_realization",
                runtime_availability,
                binding={"batch_report": "runtime.control_realization"},
            )
        )
    elif family_id == "x15" and fidelity == "rigid_body_6dof_direct_wrench":
        status.extend(
            (
                _status(
                    "velocity.body",
                    "m/s",
                    "Committed local source-plant body velocity for the direct-wrench screen.",
                    "body_velocity_m_s",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "body_velocity_m_s", "frame": "body"},
                    provenance="source_backed",
                ),
                _status(
                    "body_rate",
                    "rad/s",
                    "Committed local source-plant body rate for the direct-wrench screen.",
                    "body_rate_rad_s",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "body_rate_rad_s", "frame": "body"},
                    provenance="source_backed",
                ),
                _status(
                    "control.wrench.requested.force",
                    "N",
                    "Bounded-screen requested body-frame direct force.",
                    "requested_force_body_n",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "requested_force_body_n", "frame": "body"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.requested.moment",
                    "N*m",
                    "Bounded-screen requested body-frame direct moment.",
                    "requested_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "requested_moment_body_nm", "frame": "body"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.achieved.force",
                    "N",
                    "Projected body-frame direct force actually applied to the local source plant.",
                    "achieved_force_body_n",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "achieved_force_body_n", "frame": "body"},
                    provenance="engineering_surrogate",
                ),
                _status(
                    "control.wrench.achieved.moment",
                    "N*m",
                    "Projected body-frame direct moment actually applied to the local source plant.",
                    "achieved_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "achieved_moment_body_nm", "frame": "body"},
                    provenance="engineering_surrogate",
                ),
                _status(
                    "control.wrench.residual.force",
                    "N",
                    "Requested-minus-achieved body-force residual after direct-wrench projection.",
                    "residual_force_body_n",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "residual_force_body_n", "frame": "body"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.residual.moment",
                    "N*m",
                    "Requested-minus-achieved body-moment residual after direct-wrench projection.",
                    "residual_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "residual_moment_body_nm", "frame": "body"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.status",
                    None,
                    "Direct-wrench projection disposition at the committed local-screen state.",
                    "wrench_status",
                    runtime_availability,
                    value_type="enum",
                    binding={"batch_telemetry": "wrench_status"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.saturated",
                    None,
                    "Whether direct-wrench authority or slew projection limited this local-screen request.",
                    "wrench_saturated",
                    runtime_availability,
                    value_type="boolean",
                    binding={"batch_telemetry": "wrench_saturated"},
                    provenance="derived",
                ),
                _status(
                    "control.physical_effector_allocation",
                    None,
                    "Whether the selected control path allocates the direct-wrench request to physical X-15 effectors.",
                    "physical_effector_allocation",
                    runtime_availability,
                    value_type="boolean",
                    binding={"batch_telemetry": "physical_effector_allocation"},
                    provenance="derived",
                ),
            )
        )
        diagnostics.append(
            _diagnostic(
                "control.realization",
                "direct_wrench_screen",
                runtime_availability,
                binding={"batch_telemetry": "control_realization"},
            )
        )
    elif family_id == "tumbling_body" and fidelity in {"point_mass_3dof", "pseudo_6dof"}:
        status.extend(
            (
                _status(
                    "position.local",
                    "m",
                    "Committed local-frame passive-body position.",
                    "position_m",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "position_m", "frame": "local_reduced"},
                    provenance="engineering_surrogate",
                ),
                _status(
                    "position.altitude",
                    "m",
                    "Committed passive-body altitude above the impact plane.",
                    "position_m[2]",
                    runtime_availability,
                    binding={"batch_telemetry": "position_m[2]", "frame": "local_reduced"},
                    provenance="engineering_surrogate",
                ),
                _status(
                    "velocity.local",
                    "m/s",
                    "Committed local-frame passive-body velocity.",
                    "velocity_m_s",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "velocity_m_s", "frame": "local_reduced"},
                    provenance="engineering_surrogate",
                ),
                _status(
                    "velocity.speed",
                    "m/s",
                    "Magnitude of the committed passive-body velocity.",
                    "velocity_m_s",
                    runtime_availability,
                    binding={"derived_from": "batch_telemetry.velocity_m_s", "transform": "norm"},
                    provenance="engineering_surrogate",
                ),
                _status(
                    "phase.mode",
                    None,
                    "Passive direct-release phase, always ballistic after the declared release.",
                    "ballistic",
                    runtime_availability,
                    value_type="enum",
                    binding={"constant": "ballistic"},
                    provenance="derived",
                ),
            )
        )
        if fidelity == "pseudo_6dof":
            status.extend(
                (
                    _status(
                        "attitude.quaternion",
                        "dimensionless",
                        "Native rigid-body attitude reused by the passive pseudo-6DOF profile.",
                        "attitude_quaternion",
                        runtime_availability,
                        value_type="vector4",
                        binding={"batch_telemetry": "attitude_quaternion"},
                        provenance="engineering_surrogate",
                    ),
                    _status(
                        "body_rate",
                        "rad/s",
                        "Native rigid-body angular rate reused by the passive pseudo-6DOF profile.",
                        "attitude_rate_rad_s",
                        runtime_availability,
                        value_type="vector3",
                        binding={"batch_telemetry": "attitude_rate_rad_s"},
                        provenance="engineering_surrogate",
                    ),
                )
            )
        resources.append(
            InterfaceChannel(
                "resources.mass.total",
                "resource",
                "scalar",
                "kg",
                "Passive-body mass at the committed truth state.",
                availability=runtime_availability,
                provenance="engineering_surrogate",
                sampling="truth_boundary",
                binding={"batch_telemetry": "mass_kg"},
                claim_boundary="The witness has no propulsion resource; this is fixed detached-body mass, not fuel or propellant.",
            )
        )
        diagnostics.append(
            _diagnostic(
                "control.realization",
                "uncontrolled",
                runtime_availability,
                binding={"batch_report": "runtime.control_realization"},
            )
        )
    return tuple(status), tuple(resources), tuple(diagnostics)
    ####


def _status(
    identifier: str,
    unit: str | None,
    description: str,
    native: str,
    availability: InterfaceAvailability,
    *,
    value_type: InterfaceValueType = "scalar",
    scale: float | None = None,
    binding: Mapping[str, object] | None = None,
    provenance: ProvenanceKind = "source_backed",
) -> InterfaceChannel:
    resolved_binding: dict[str, object] = {"episode_value": native} if binding is None else dict(binding)
    if scale is not None:
        resolved_binding["scale"] = scale
    return InterfaceChannel(
        identifier,
        "status",
        value_type,
        unit,
        description,
        availability=availability,
        provenance=provenance,
        sampling="truth_boundary",
        binding=resolved_binding,
        claim_boundary="Published only at committed truth boundaries; it is not a policy observation unless an observation profile includes it.",
    )
    ####


def _diagnostic(
    identifier: str,
    native: str,
    availability: InterfaceAvailability,
    *,
    binding: Mapping[str, object] | None = None,
) -> InterfaceChannel:
    return InterfaceChannel(
        identifier,
        "diagnostic",
        "enum" if identifier == "control.realization" else "boolean",
        None,
        "Raw claim-boundary diagnostic retained with the canonical status view.",
        availability=availability,
        provenance="derived",
        sampling="truth_boundary",
        binding={"episode_value": native} if binding is None else dict(binding),
        claim_boundary="Diagnostic provenance; not an independent physical-control claim.",
    )
    ####


def _observation_contract(
    status: Iterable[InterfaceChannel],
    resources: Iterable[InterfaceChannel],
    diagnostics: Iterable[InterfaceChannel],
) -> tuple[ObservationProfile, ...]:
    channels = tuple(item.id for item in (*tuple(status), *tuple(resources), *tuple(diagnostics)) if item.availability == "available")
    truth_availability: InterfaceAvailability = "available" if channels else "unavailable_at_runtime"
    return (
        ObservationProfile(
            "truth_debug",
            truth_availability,
            channels,
            "Committed plant truth for debugging, evaluation, and explicitly selected policy experiments.",
            "truth",
            "Truth is emitted only at committed integration boundaries and is not an implicitly realistic sensor feed.",
        ),
        ObservationProfile(
            "declared_sensor",
            "planned",
            (),
            "Future sensor/estimator observation profile with cadence, latency, and validity masks.",
            "sensor",
            "No sensor profile is invented from truth; this profile remains unavailable until declared sensor instances are bound.",
        ),
    )
    ####


def bind_declared_sensor_profile(
    contract: VehicleInterfaceContract,
    *,
    channel_ids: tuple[str, ...],
    cadence_s: float,
    latency_s: float,
    channel_errors: Mapping[str, Mapping[str, object]] | None = None,
    profile_id: str = "declared_sensor",
) -> VehicleInterfaceContract:
    """Bind one composition-declared committed-boundary sensor profile.

    This deliberately provides neither interpolation nor an inferred sensor
    suite.  The caller must name portable status/resource/diagnostic channels,
    and the episode runtime must visit every cadence and delayed-release
    boundary before emitting a value.  The resulting fingerprint therefore
    identifies the selected observation path as well as the vehicle fidelity.
    """

    if not channel_ids:
        raise ValueError("declared sensor profile must name at least one channel")
    available = {
        item.id
        for item in (*contract.status_channels, *contract.resource_channels, *contract.diagnostic_channels)
        if item.availability == "available"
    }
    unknown = sorted(set(channel_ids) - available)
    if unknown:
        raise ValueError(f"declared sensor profile references unavailable channel(s): {unknown}")
    if len(channel_ids) != len(set(channel_ids)):
        raise ValueError("declared sensor profile has duplicate channel IDs")
    if not math.isfinite(cadence_s) or cadence_s <= 0.0:
        raise ValueError("declared sensor cadence_s must be positive and finite")
    if not math.isfinite(latency_s) or latency_s < 0.0:
        raise ValueError("declared sensor latency_s must be nonnegative and finite")
    normalized_errors = _normalize_sensor_channel_errors(
        channel_ids=channel_ids,
        channel_descriptors={
            item.id: item
            for item in (*contract.status_channels, *contract.resource_channels, *contract.diagnostic_channels)
        },
        channel_errors=channel_errors,
    )

    profile = ObservationProfile(
        profile_id,
        "available",
        channel_ids,
        "Composition-declared sampled observation from committed plant truth.",
        "sensor",
        (
            "Samples are captured only at declared committed-truth cadence boundaries and released after the "
            "declared latency. No sample is interpolated from a future integration state."
        ),
        cadence_s=cadence_s,
        latency_s=latency_s,
        channel_errors=normalized_errors,
    )
    retained = tuple(item for item in contract.observation_profiles if item.id != profile_id)
    return VehicleInterfaceContract(
        vehicle_id=contract.vehicle_id,
        family_id=contract.family_id,
        physical_family=contract.physical_family,
        fidelity=contract.fidelity,
        control_realization=contract.control_realization,
        evidence_status=contract.evidence_status,
        parameter_channels=contract.parameter_channels,
        action_channels=contract.action_channels,
        effector_channels=contract.effector_channels,
        status_channels=contract.status_channels,
        resource_channels=contract.resource_channels,
        diagnostic_channels=contract.diagnostic_channels,
        authority_profiles=contract.authority_profiles,
        observation_profiles=(*retained, profile),
        execution_records=contract.execution_records,
        claim_boundary=contract.claim_boundary,
        schema=contract.schema,
    )
    ####


def _normalize_sensor_channel_errors(
    *,
    channel_ids: tuple[str, ...],
    channel_descriptors: Mapping[str, InterfaceChannel],
    channel_errors: Mapping[str, Mapping[str, object]] | None,
) -> dict[str, dict[str, float | None]]:
    """Validate scalar-only measurement transforms before runtime construction.

    The shared boundary sampler deliberately has no guessed vector, attitude,
    IMU, GPS, or estimator behavior.  A sensor declaration may corrupt only a
    selected portable scalar with an explicit additive/quantized transform.
    """

    if channel_errors is None:
        return {}
    unknown = sorted(set(channel_errors) - set(channel_ids))
    if unknown:
        raise ValueError(f"declared sensor measurement model references unsampled channel(s): {unknown}")
    normalized: dict[str, dict[str, float | None]] = {}
    for channel_id, raw in channel_errors.items():
        descriptor = channel_descriptors[channel_id]
        if descriptor.value_type != "scalar":
            raise ValueError(
                f"declared sensor measurement model for {channel_id!r} requires a scalar channel; "
                f"{descriptor.value_type!r} is not a portable scalar error model"
            )
        allowed = {"bias", "gaussian_stddev", "quantization_step"}
        extras = sorted(set(raw) - allowed)
        if extras:
            raise ValueError(f"declared sensor measurement model for {channel_id!r} has unknown field(s): {extras}")
        bias = _sensor_scalar(raw.get("bias", 0.0), f"sensor bias for {channel_id}")
        stddev = _sensor_scalar(raw.get("gaussian_stddev", 0.0), f"sensor Gaussian deviation for {channel_id}")
        if stddev < 0.0:
            raise ValueError(f"sensor Gaussian deviation for {channel_id!r} must be nonnegative")
        quantization_raw = raw.get("quantization_step")
        quantization = (
            None
            if quantization_raw is None
            else _sensor_scalar(quantization_raw, f"sensor quantization step for {channel_id}")
        )
        if quantization is not None and quantization <= 0.0:
            raise ValueError(f"sensor quantization step for {channel_id!r} must be positive")
        normalized[channel_id] = {
            "bias": bias,
            "gaussian_stddev": stddev,
            "quantization_step": quantization,
        }
    return normalized
    ####


def _sensor_scalar(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"{label} must be finite numeric")
    return float(value)
    ####


def project_committed_status_values(
    contract: VehicleInterfaceContract,
    *,
    time_s: float,
    execution_status: str,
    raw_values: Mapping[str, object],
    include_batch_available: bool = False,
) -> dict[str, object]:
    """Project one committed native-truth mapping to portable status values.

    Episode and batch paths both use the same descriptor bindings.  This
    helper deliberately operates only on an already committed native sample;
    it performs no dynamics, interpolation, or unavailable-value substitution.
    """

    if not math.isfinite(time_s):
        raise ValueError("committed status time_s must be finite")
    values: dict[str, object] = {
        "execution.time": time_s,
        "execution.status": execution_status,
    }
    available = {"available"}
    if include_batch_available:
        available.add("available_in_batch")
    for channel in (*contract.status_channels, *contract.resource_channels, *contract.diagnostic_channels):
        if channel.availability not in available or channel.id in values:
            continue
        value = _bound_status_value(channel, raw_values, contract)
        if value is not _STATUS_MISSING:
            values[channel.id] = value
    return values
    ####


def validate_interface_channel_value(
    channel: InterfaceChannel,
    value: object,
    *,
    context: str,
) -> None:
    """Reject a projected value that contradicts one declared semantic channel.

    Interface metadata is only a reliable AI, artifact, or renderer hook when
    each advertised value keeps its declared primitive shape and scalar bounds.
    This validator deliberately checks only portable representation invariants;
    it does not infer units, repair values, or make a physical-fidelity claim.
    """

    if channel.value_type == "scalar":
        if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
            raise ValueError(f"{context}: channel {channel.id!r} requires a finite scalar")
        numeric = float(value)
        if channel.lower is not None and numeric < channel.lower:
            raise ValueError(f"{context}: channel {channel.id!r} is below its declared lower bound")
        if channel.upper is not None and numeric > channel.upper:
            raise ValueError(f"{context}: channel {channel.id!r} exceeds its declared upper bound")
        return
    if channel.value_type in {"vector3", "vector4"}:
        expected_size = 3 if channel.value_type == "vector3" else 4
        if not isinstance(value, list | tuple) or len(value) != expected_size:
            raise ValueError(f"{context}: channel {channel.id!r} requires a {channel.value_type}")
        for item in value:
            if isinstance(item, bool) or not isinstance(item, int | float) or not math.isfinite(float(item)):
                raise ValueError(f"{context}: channel {channel.id!r} requires finite vector components")
        return
    if channel.value_type == "boolean":
        if not isinstance(value, bool):
            raise ValueError(f"{context}: channel {channel.id!r} requires a boolean")
        return
    if channel.value_type in {"enum", "event"}:
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{context}: channel {channel.id!r} requires a non-empty string")
        return
    raise ValueError(f"{context}: channel {channel.id!r} has an unsupported value type")
    ####


def validate_projected_status_values(
    contract: VehicleInterfaceContract,
    values: Mapping[str, object],
    *,
    include_batch_available: bool = False,
    context: str,
) -> None:
    """Require complete, shape-valid values for every visible status channel."""

    available = {"available"}
    if include_batch_available:
        available.add("available_in_batch")
    for channel in (*contract.status_channels, *contract.resource_channels, *contract.diagnostic_channels):
        if channel.availability not in available:
            continue
        if channel.id not in values:
            raise ValueError(f"{context}: declared channel {channel.id!r} is missing")
        validate_interface_channel_value(channel, values[channel.id], context=context)
    ####


class _StatusMissing:
    """Distinct marker so an explicit null remains a status value."""


_STATUS_MISSING = _StatusMissing()


def _bound_status_value(
    channel: InterfaceChannel,
    raw_values: Mapping[str, object],
    contract: VehicleInterfaceContract,
) -> object:
    """Read one declared native status binding without guessing data."""

    binding = channel.binding
    constant = binding.get("constant")
    if constant is not None:
        return constant
    native = (
        binding.get("episode_value")
        or binding.get("runtime_state")
        or binding.get("batch_telemetry")
        or binding.get("batch_report")
    )
    if native is None:
        derived_from = binding.get("derived_from")
        transform = binding.get("transform")
        if not isinstance(derived_from, str) or not isinstance(transform, str):
            return contract.control_realization if channel.id == "control.realization" else _STATUS_MISSING
        value = _derived_status_value(derived_from, transform, raw_values)
    elif not isinstance(native, str):
        return _STATUS_MISSING
    else:
        value = _nested_status_value(raw_values, native)
    if value is _STATUS_MISSING and channel.id == "control.realization":
        return contract.control_realization
    scale = channel.binding.get("scale")
    if isinstance(scale, int | float) and not isinstance(scale, bool) and isinstance(value, int | float) and not isinstance(value, bool):
        return float(value) * float(scale)
    return value
    ####


def _derived_status_value(
    source: str,
    transform: str,
    raw_values: Mapping[str, object],
) -> object:
    """Apply one explicitly declared portable status transform.

    Status derivations deliberately remain small, named, and checked in.  A
    batch adapter may not introduce an arbitrary expression merely to fill a
    caller-facing channel that its interface contract did not define.
    """

    prefixes = ("batch_telemetry.", "runtime_state.", "episode_value.")
    native = next((source.removeprefix(prefix) for prefix in prefixes if source.startswith(prefix)), source)
    value = _nested_status_value(raw_values, native)
    if value is _STATUS_MISSING:
        return _STATUS_MISSING
    if transform == "norm":
        if not isinstance(value, list | tuple) or not value:
            return _STATUS_MISSING
        components: list[float] = []
        for item in value:
            if isinstance(item, bool) or not isinstance(item, int | float) or not math.isfinite(float(item)):
                return _STATUS_MISSING
            components.append(float(item))
        return math.sqrt(sum(item * item for item in components))
    if transform == "phase_not_glide":
        return str(value) != "glide"
    if transform == "rad_to_deg":
        if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
            return _STATUS_MISSING
        return math.degrees(float(value))
    if transform == "100_kg_s_times_min_time_20_s":
        if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
            return _STATUS_MISSING
        return 100.0 * min(max(float(value), 0.0), 20.0)
    return _STATUS_MISSING
    ####


def _nested_status_value(values: Mapping[str, object], binding: str) -> object:
    """Resolve a simple dotted/indexed native binding from committed truth."""

    source = binding.replace(" (sign-inverted)", "")
    sign = -1.0 if "(sign-inverted)" in binding else 1.0
    index: int | None = None
    if "[" in source and source.endswith("]"):
        source, index_text = source[:-1].rsplit("[", 1)
        try:
            index = int(index_text)
        except ValueError:
            return _STATUS_MISSING
    current: object = values
    for token in source.split("."):
        if not isinstance(current, Mapping) or token not in current:
            return _STATUS_MISSING
        current = current[token]
    if index is not None:
        if not isinstance(current, list | tuple) or index < 0 or index >= len(current):
            return _STATUS_MISSING
        current = current[index]
    if sign != 1.0 and isinstance(current, int | float) and not isinstance(current, bool):
        return sign * float(current)
    return current
    ####


def validate_vehicle_interface_contract(contract: VehicleInterfaceContract) -> tuple[str, ...]:
    """Return fail-closed authoring findings for one resolved contract."""

    findings: list[str] = []
    channels = (
        *contract.parameter_channels,
        *contract.action_channels,
        *contract.effector_channels,
        *contract.status_channels,
        *contract.resource_channels,
        *contract.diagnostic_channels,
    )
    for channel in channels:
        if channel.availability in {"available", "available_in_batch"} and not channel.binding:
            findings.append(f"{channel.id}: available channel has no native or runtime binding")
        if channel.kind == "effector" and channel.availability == "available" and "native_effector" not in channel.binding:
            findings.append(f"{channel.id}: physical effector lacks an explicit native binding")
    for profile in contract.authority_profiles:
        if profile.availability == "available" and not profile.action_ids:
            findings.append(f"{profile.id}: available authority profile exposes no actions")
    for observation_profile in contract.observation_profiles:
        if observation_profile.source == "sensor" and observation_profile.availability == "available" and not observation_profile.channel_ids:
            findings.append(f"{observation_profile.id}: available sensor profile exposes no channels")
    return tuple(findings)
    ####


def build_vehicle_interface_catalog_report(
    catalog: ResolvedVehicleCompositionCatalog | None = None,
) -> dict[str, object]:
    """Validate every advertised interface as one fail-closed catalog gate.

    This report is intentionally declarative: it does not open episodes or
    integrate a plant.  It proves that the registry, execution-binding catalog,
    and semantic interface agree about every currently advertised fidelity.
    A profile declared as available must have a matching runnable episode
    binding; planned and unavailable channels remain visible without becoming
    errors merely because their promotion work is unfinished.
    """

    if catalog is None:
        from .vehicle_composition_registry import load_resolved_vehicle_composition_catalog

        catalog = load_resolved_vehicle_composition_catalog()
    records: list[dict[str, object]] = []
    error_count = 0
    for composition in catalog.vehicles:
        for fidelity in composition.family.family.tiers:
            contract = interface_contract_for_composition(composition, fidelity)
            findings = list(validate_vehicle_interface_contract(contract))
            episode_runnable = any(
                item.get("operation") == "episode" and item.get("status") == "runnable"
                for item in contract.execution_records
            )
            available_authority = tuple(
                item.id for item in contract.authority_profiles if item.availability == "available"
            )
            available_observations = tuple(
                item.id for item in contract.observation_profiles if item.availability == "available"
            )
            if available_authority and not episode_runnable:
                findings.append("available authority profile has no runnable episode execution binding")
            if available_observations and not episode_runnable:
                findings.append("available observation profile has no runnable episode execution binding")
            channel_availability: dict[str, int] = {}
            for channel in (
                *contract.parameter_channels,
                *contract.action_channels,
                *contract.effector_channels,
                *contract.status_channels,
                *contract.resource_channels,
                *contract.diagnostic_channels,
            ):
                channel_availability[channel.availability] = channel_availability.get(channel.availability, 0) + 1
            status = "pass" if not findings else "fail"
            if findings:
                error_count += 1
            records.append(
                {
                    "vehicle_id": contract.vehicle_id,
                    "family_id": contract.family_id,
                    "fidelity": fidelity,
                    "interface_id": contract.id,
                    "fingerprint_sha256": contract.fingerprint,
                    "control_realization": contract.control_realization,
                    "evidence_status": contract.evidence_status,
                    "channel_availability": channel_availability,
                    "available_authority_profiles": list(available_authority),
                    "available_observation_profiles": list(available_observations),
                    "episode_runnable": episode_runnable,
                    "execution_records": [dict(item) for item in contract.execution_records],
                    "status": status,
                    "findings": findings,
                }
            )
    return {
        "schema": "taoryx.vehicle-interface-catalog-report/v1alpha1",
        "status": "pass" if error_count == 0 else "fail",
        "family_count": len(catalog.vehicles),
        "interface_count": len(records),
        "error_count": error_count,
        "interfaces": records,
        "claim_boundary": (
            "This is a registry/interface conformance report. It does not execute, qualify, "
            "or promote any vehicle or control realization."
        ),
    }
    ####


def _has_runnable_episode(family_id: str, fidelity: FidelityTier) -> bool:
    return any(item.operation == "episode" and item.fidelity == fidelity and item.status == "runnable" for item in bindings_for_family(family_id))
    ####


def _has_runnable_batch(family_id: str, fidelity: FidelityTier) -> bool:
    """Return whether an exact family/fidelity batch binding exists."""

    return any(item.operation == "batch" and item.fidelity == fidelity and item.status == "runnable" for item in bindings_for_family(family_id))
    ####


def _canonical_effector_name(name: str) -> str:
    return name.removesuffix("-deg").replace("_", ".").replace("-", ".")
    ####


def _optional_number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, int | float):
        return None
    return float(value)
    ####


def _require_unique_ids(channels: Iterable[InterfaceChannel]) -> None:
    values = tuple(item.id for item in channels)
    if len(values) != len(set(values)):
        raise ValueError(f"interface channel IDs must be unique within a schema: {values!r}")
    ####


def _contract_claim_boundary(family_id: str, fidelity: FidelityTier, evidence_status: str) -> str:
    return (
        f"{family_id}/{fidelity} is declared at evidence status {evidence_status!r}. "
        "This interface advertises only mapped configuration, action, status, resource, and observation channels; "
        "it does not promote planned execution, sensor, actuator, or qualification evidence."
    )
    ####


__all__ = [
    "AuthorityKind",
    "AuthorityProfile",
    "build_vehicle_interface_catalog_report",
    "project_committed_status_values",
    "InterfaceAvailability",
    "InterfaceChannel",
    "InterfaceChannelKind",
    "InterfaceValueType",
    "ObservationProfile",
    "ParameterScope",
    "SCHEMA_ID",
    "VehicleInterfaceContract",
    "bind_declared_sensor_profile",
    "interface_contract_for_composition",
    "resolve_vehicle_interface_contract",
    "validate_interface_channel_value",
    "validate_projected_status_values",
    "validate_vehicle_interface_contract",
]
