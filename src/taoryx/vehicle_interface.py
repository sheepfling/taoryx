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
from .interface_channel_value_spaces import interface_channel_value_space_profile, value_space_for_interface_channel_profile
from .value_space import (
    ValueSpaceSpec,
    default_value_space_for_value_type,
    validate_value_space_value,
)
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
QuantitySemantics = Literal["count", "mixed_wrench_norm", "normalized_error"]
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


def _declared_value_space(
    identifier: str,
    value_type: InterfaceValueType,
    canonical_unit: str | None,
    lower: float | None,
    upper: float | None,
) -> ValueSpaceSpec:
    """Return the centrally reviewed value-space declaration for one channel.

    Public registry channels resolve from the versioned interface-channel
    catalog. The primitive fallback exists only for standalone fixtures and
    is never accepted by the public interface topology audit.
    """

    profile = interface_channel_value_space_profile(identifier)
    if profile is not None:
        return value_space_for_interface_channel_profile(profile, canonical_unit=canonical_unit)
    return default_value_space_for_value_type(value_type)
    ####


@dataclass(frozen=True, slots=True)
class InterfaceChannel:
    """One stable semantic channel and its exact native binding boundary."""

    id: str
    kind: InterfaceChannelKind
    value_type: InterfaceValueType
    canonical_unit: str | None
    description: str
    quantity_semantics: QuantitySemantics | None = None
    scope: ParameterScope | None = None
    frame: str | None = None
    lower: float | None = None
    upper: float | None = None
    availability: InterfaceAvailability = "available"
    provenance: ProvenanceKind = "derived"
    sampling: SamplingSemantics = "truth_boundary"
    binding: Mapping[str, object] = field(default_factory=dict)
    claim_boundary: str = ""
    value_space: ValueSpaceSpec | None = None

    def __post_init__(self) -> None:
        if not self.id.strip() or not self.description.strip():
            raise ValueError("interface channels require a stable ID and description")
        if self.kind == "parameter" and self.scope is None:
            raise ValueError(f"parameter channel {self.id!r} requires a mutability scope")
        if self.kind != "parameter" and self.scope is not None:
            raise ValueError(f"non-parameter channel {self.id!r} cannot declare a parameter scope")
        if self.quantity_semantics is not None and self.value_type != "scalar":
            raise ValueError(
                f"interface channel {self.id!r} declares unitless quantity semantics but is not a scalar"
            )
        if self.value_type in {"scalar", "vector3", "vector4"} and self.canonical_unit is None and self.quantity_semantics is None:
            raise ValueError(
                f"numeric interface channel {self.id!r} requires a canonical unit or explicit quantity semantics"
            )
        if self.lower is not None and self.upper is not None and self.lower > self.upper:
            raise ValueError(f"interface channel {self.id!r} has inverted bounds")
        if self.availability == "available" and not self.claim_boundary.strip():
            raise ValueError(f"available channel {self.id!r} requires a claim boundary")
        if self.value_space is None:
            object.__setattr__(
                self,
                "value_space",
                _declared_value_space(self.id, self.value_type, self.canonical_unit, self.lower, self.upper),
            )
        value_space = self.value_space
        if value_space is None:
            raise ValueError(f"interface channel {self.id!r} requires a value-space declaration")
        expected_representation = {
            "scalar": "scalar",
            "vector3": "vector3",
            "vector4": "vector4",
            "boolean": "boolean",
            "enum": "string",
            "event": "string",
        }[self.value_type]
        if not value_space.representation.startswith(expected_representation):
            raise ValueError(
                f"interface channel {self.id!r} has {self.value_type!r} storage but "
                f"value-space representation {value_space.representation!r}"
            )
        ####
    ####

    def as_dict(self) -> dict[str, object]:
        """Return the channel as a serialized interface-schema entry."""

        value_space = self.value_space
        if value_space is None:
            raise ValueError(f"interface channel {self.id!r} is missing its value-space declaration")
        return {
            "id": self.id,
            "kind": self.kind,
            "value_type": self.value_type,
            "value_space": value_space.as_dict(),
            "value_space_source": (
                "interface_channel_value_space_catalog"
                if interface_channel_value_space_profile(self.id) is not None
                else "ad_hoc_primitive_fallback"
            ),
            "canonical_unit": self.canonical_unit,
            "quantity_semantics": self.quantity_semantics,
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
    action_channels, authority_profiles = _action_contract(
        family.family_id,
        fidelity,
        episode_runnable,
        batch_runnable,
    )
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
        effector_channels=_effector_contract(
            family.family_id,
            fidelity,
            family.vehicle_definition,
            tier.profile_id is not None,
            batch_runnable,
        ),
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
        value_type=parameter.value_type,
        canonical_unit=parameter.canonical_unit,
        description=parameter.description,
        scope=scope,
        lower=parameter.hard_bounds[0],
        upper=parameter.hard_bounds[1],
        availability="available",
        provenance="derived",
        sampling="reset_only" if scope == "episode_reset" else "segment_transition",
        binding={**binding, "options": list(parameter.options)} if parameter.options else binding,
        claim_boundary=(
            "This is a declared composition input. Coupled physics, trim, and "
            "qualification requirements remain the selected family adapter's responsibility."
        ),
        value_space=parameter.value_space,
    )
    ####


def _action_contract(
    family_id: str,
    fidelity: FidelityTier,
    episode_runnable: bool,
    batch_runnable: bool,
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

    if family_id in {"a320_openap_3dof", "f16_s119"} and fidelity in {"point_mass_3dof", "pseudo_6dof"}:
        available = "available" if episode_runnable else "unavailable_at_runtime"
        maximum_speed_m_s = 300.0 if family_id == "a320_openap_3dof" else 500.0
        guidance_channels: tuple[InterfaceChannel, ...] = (
            _action(
                "guidance.speed.command",
                "m/s",
                50.0,
                maximum_speed_m_s,
                "Held kinematic speed target for the declared reduced response law.",
                "speed_m_s",
                available,
            ),
            _action(
                "guidance.flight_path_angle.command",
                "deg",
                -20.0,
                20.0,
                "Held kinematic flight-path-angle target for the declared reduced response law.",
                "flight_path_angle_deg",
                available,
            ),
            _action(
                "guidance.heading.command",
                "deg",
                0.0,
                360.0,
                "Held kinematic heading target in the local navigation frame.",
                "heading_deg",
                available,
            ),
            _action(
                "guidance.bank.command",
                "deg",
                -60.0,
                60.0,
                "Held bank or lift-vector target; realized only by the selected response-law tier.",
                "bank_angle_deg",
                available,
            ),
        )
        return guidance_channels, (
            AuthorityProfile(
                "kinematic_guidance",
                "kinematic",
                available,
                tuple(item.id for item in guidance_channels),
                "Bounded speed, flight-path, heading, and bank intent for a reduced fixed-wing plant.",
                "This is point-mass or named response-law guidance only; it does not establish physical surfaces, "
                "actuator dynamics, allocation, or moment balance.",
            ),
        )

    fixed_wing_controls = _fixed_wing_bridge_controls(family_id)
    if fixed_wing_controls and fidelity in {"point_mass_3dof", "pseudo_6dof"}:
        available = "available" if episode_runnable else "unavailable_at_runtime"
        guidance_channels = _language_backed_guidance_controls(family_id, available, fidelity)
        bridge_channels: tuple[InterfaceChannel, ...] = tuple(
            _action(
                identifier,
                unit,
                lower,
                upper,
                description,
                native,
                available,
                binding={
                    "control_role": "source_runtime_load_probe",
                    "tuning_eligible": False,
                    "state_authority": "diagnostic_only",
                },
                claim_boundary=(
                    "This coordinate is accepted by the source-runtime load evaluation, but the selected lower "
                    "tier's autonomous route resolver owns translational state rates. It is therefore a source-load "
                    "probe, not a lower-tier trajectory-control or tuning input and not physical actuator evidence."
                ),
            )
            for identifier, unit, lower, upper, description, native in fixed_wing_controls
        )
        return (*guidance_channels, *bridge_channels), (
            AuthorityProfile(
                "kinematic_guidance",
                "kinematic",
                available,
                tuple(item.id for item in guidance_channels),
                "Explicit bounded speed, flight-path, and heading targets for the native lower-tier command law.",
                "This authority replaces the autonomous route target only when explicitly enabled. It is a point-mass "
                "or kinematic-response command seam, not source surface allocation, moment balance, or a flight-control qualification.",
            ),
            AuthorityProfile(
                "native_control_bridge",
                "native_bridge",
                available,
                tuple(item.id for item in bridge_channels),
                "Explicit source-runtime load coordinates projected into stable semantic names.",
                "The bridge preserves source-load visibility but does not own lower-tier trajectory state rates. It does not "
                "prove trajectory authority, physical actuator allocation, servo dynamics, or moment balance.",
            ),
        )

    if fidelity == "rigid_body_6dof_direct_wrench":
        batch_internal_controller_trace = batch_runnable and family_id in {
            "hummingbird",
            "hl20_mod_k",
            "x15",
        }
        direct_available: InterfaceAvailability = (
            "available"
            if episode_runnable
            else "available_in_batch"
            if batch_internal_controller_trace
            else "planned"
        )
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
                binding={
                    "frame": "body",
                    "native_action": "force_body_n",
                    "external_override": episode_runnable,
                    "internal_controller_trace": batch_internal_controller_trace,
                },
                claim_boundary=(
                    "The command is a total direct body force, including any declared local source-load bridge bias. "
                    "For batch-only local screens it records the controller-generated request, not a caller override. "
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
                binding={
                    "frame": "body",
                    "native_action": "moment_body_nm",
                    "external_override": episode_runnable,
                    "internal_controller_trace": batch_internal_controller_trace,
                },
                claim_boundary=(
                    "The command is a total direct body moment, including any declared local source-load bridge bias. "
                    "For batch-only local screens it records the controller-generated request, not a caller override. "
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
                    "This profile accepts external actions only when an exact source-local episode binding and achieved-wrench "
                    "telemetry exist. Batch-only screens publish their internal controller requests as an exact action trace. "
                    "It remains bridge/screen evidence, never physical-effector allocation."
                    if episode_runnable
                    else (
                        "This profile remains unavailable to an episode until an exact native binding and achieved-wrench "
                        "telemetry exist; a runnable batch screen can still publish its internally generated requests."
                        if batch_internal_controller_trace
                        else "This profile remains unavailable until an exact native binding and achieved-wrench telemetry exist."
                    )
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


def _language_backed_guidance_controls(
    family_id: str,
    availability: InterfaceAvailability,
    fidelity: FidelityTier,
) -> tuple[InterfaceChannel, ...]:
    """Expose the exact external command seam added by composition materialization.

    X8 and B747 lower-tier racetracks use a native commanded-state response
    law.  These channels drive that law only after the explicit enable action;
    source surface coordinates remain separately available as load probes.
    """

    maximum_speed_m_s = 27.0 if family_id == "skywalker_x8" else 220.0
    minimum_speed_m_s = 2.0 if family_id == "skywalker_x8" else 100.0
    maximum_flight_path_deg = 20.0 if family_id == "skywalker_x8" else 10.0
    maximum_bank_deg = 45.0 if family_id == "skywalker_x8" else 30.0
    common_binding = {
        "control_role": "kinematic_guidance",
        "tuning_eligible": True,
        "override_gate_native_action": "guidance-override-enabled",
        "state_authority": "native_commanded_state_rate",
    }
    controls: tuple[InterfaceChannel, ...] = (
        InterfaceChannel(
            "guidance.override.enabled",
            "action",
            "boolean",
            None,
            "Enable an explicit held external target in place of the autonomous lower-tier racetrack target.",
            availability=availability,
            provenance="derived",
            sampling="held_action",
            binding={"native_action": "guidance-override-enabled", **common_binding},
            claim_boundary=(
                "This selects the native lower-tier kinematic command law. It neither enables a source surface "
                "controller nor changes the physical-effector evidence boundary."
            ),
        ),
        _action(
            "guidance.speed.command",
            "m/s",
            minimum_speed_m_s,
            maximum_speed_m_s,
            "Held speed target for the native lower-tier commanded-state response.",
            "guidance-speed-mps",
            availability,
            binding={**common_binding, "controlled_state_channels": ["velocity.speed"]},
            claim_boundary=(
                "This target is effective only while guidance.override.enabled is true. It drives the declared "
                "lower-tier speed response, not source thrust or physical actuator allocation."
            ),
        ),
        _action(
            "guidance.flight_path_angle.command",
            "deg",
            -maximum_flight_path_deg,
            maximum_flight_path_deg,
            "Held flight-path-angle target for the native lower-tier commanded-state response.",
            "guidance-flight-path-angle-deg",
            availability,
            binding={**common_binding, "controlled_state_channels": ["position.altitude", "flight.path_angle"]},
            claim_boundary=(
                "This target is effective only while guidance.override.enabled is true. It drives the declared "
                "lower-tier flight-path response, not a physical pitch surface or moment."
            ),
        ),
        _action(
            "guidance.heading.command",
            "deg",
            0.0,
            360.0,
            "Held heading target for the native lower-tier commanded-state response.",
            "guidance-heading-deg",
            availability,
            binding={**common_binding, "controlled_state_channels": ["flight.heading"]},
            claim_boundary=(
                "This target is effective only while guidance.override.enabled is true. It drives the declared "
                "lower-tier heading response, not a physical lateral surface or moment."
            ),
        ),
    )
    if fidelity != "pseudo_6dof":
        return controls
    return (
        *controls,
        _action(
            "guidance.bank.command",
            "deg",
            -maximum_bank_deg,
            maximum_bank_deg,
            "Held bank target for the profile-backed pseudo-6DOF kinematic attitude response.",
            "guidance-bank-deg",
            availability,
            binding={**common_binding, "controlled_state_channels": ["attitude.euler"]},
            claim_boundary=(
                "This target is effective only while guidance.override.enabled is true. It drives the declared "
                "pseudo-6DOF Euler response sidecar, not a physical roll surface or moment."
            ),
        ),
    )
    ####


def _action(
    identifier: str,
    unit: str,
    lower: float,
    upper: float,
    description: str,
    native: str,
    availability: InterfaceAvailability,
    *,
    binding: Mapping[str, object] | None = None,
    claim_boundary: str | None = None,
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
        binding={"native_action": native, **(dict(binding) if binding is not None else {})},
        claim_boundary=(
            claim_boundary
            if claim_boundary is not None
            else "This action binds to the named source/runtime coordinate. The selected lower fidelity does not turn that coordinate into physical actuator evidence."
        ),
    )
    ####


def _effector_contract(
    family_id: str,
    fidelity: FidelityTier,
    vehicle_definition: Mapping[str, Any] | None,
    tier_declared: bool,
    batch_runnable: bool,
) -> tuple[InterfaceChannel, ...]:
    if fidelity != "rigid_body_6dof_surface_allocated":
        return ()
    if family_id == "f16_s119":
        availability: InterfaceAvailability = "available_in_batch" if batch_runnable else "planned"
        return tuple(
            InterfaceChannel(
                f"effector.{identifier}.position",
                "effector",
                "scalar",
                unit,
                f"Actual {identifier} position emitted by the bounded F-16 engineering actuator overlay.",
                lower=lower,
                upper=upper,
                availability=availability,
                provenance="engineering_surrogate",
                sampling="held_action",
                binding={"batch_telemetry": telemetry, "family_id": family_id},
                claim_boundary=(
                    "This is a commanded-to-achieved position from the declared F-16 engineering actuator overlay. "
                    "It is batch-visible only for the exact local source-trim screen and is not source-validated servo or envelope evidence."
                ),
            )
            for identifier, unit, lower, upper, telemetry in (
                ("elevator", "deg", -25.0, 25.0, "elevator_deg"),
                ("aileron", "deg", -21.0, 21.0, "aileron_deg"),
                ("rudder", "deg", -30.0, 30.0, "rudder_deg"),
                ("throttle", "dimensionless", 0.0, 1.0, "throttle_fraction"),
            )
        )
    if family_id == "hummingbird":
        availability = "available_in_batch" if batch_runnable else "planned"
        return tuple(
            InterfaceChannel(
                f"effector.rotor.{index}.speed.position",
                "effector",
                "scalar",
                "rad/s",
                f"Actual source motor-{index} speed after the bounded quad-X allocator and declared first-order motor lag.",
                lower=0.0,
                upper=1500.0,
                availability=availability,
                provenance="source_backed",
                sampling="held_action",
                binding={"batch_telemetry": f"rotor_{index}_speed_rad_s", "family_id": family_id},
                claim_boundary=(
                    "This is batch-visible only for the declared local Hummingbird individual-rotor LQI screens. "
                    "It records source motor-speed coordinates and their local lag, not a battery, propulsor, or flight-envelope qualification."
                ),
            )
            for index in range(1, 5)
        )
    if family_id == "skywalker_x8":
        availability = "available_in_batch" if batch_runnable else "planned"
        return tuple(
            InterfaceChannel(
                identifier,
                "effector",
                "scalar",
                unit,
                description,
                lower=lower,
                upper=upper,
                availability=availability,
                provenance="source_backed",
                sampling="held_action",
                binding={"batch_telemetry": telemetry, "native_effector": native, "family_id": family_id},
                claim_boundary=(
                    "This is the actual bounded source-table coordinate emitted by the exact local X8 physical screen. "
                    "Collective/differential values are not asserted to be individual left/right servo telemetry or wiring evidence."
                ),
            )
            for identifier, unit, lower, upper, telemetry, native, description in (
                (
                    "effector.throttle.position",
                    "dimensionless",
                    0.0,
                    1.0,
                    "throttle_fraction",
                    "throttle",
                    "Actual X8 source-table throttle coordinate after the local bounded allocator.",
                ),
                (
                    "effector.elevon.collective.position",
                    "deg",
                    -20.0,
                    20.0,
                    "collective_elevon_deg",
                    "collective-elevon-deg",
                    "Actual X8 collective-elevon source-table coordinate after the local bounded allocator.",
                ),
                (
                    "effector.elevon.differential.position",
                    "deg",
                    -20.0,
                    20.0,
                    "differential_elevon_deg",
                    "differential-elevon-deg",
                    "Actual X8 differential-elevon source-table coordinate after the local bounded allocator.",
                ),
            )
        )
    if family_id == "hl20_mod_k":
        availability = "available_in_batch" if batch_runnable else "planned"
        return tuple(
            InterfaceChannel(
                f"effector.surface.{name}.position",
                "effector",
                "scalar",
                "deg",
                f"Actual named HL-20 DAVE-ML {name.replace('_', ' ')} input after bounded source-surface allocation.",
                lower=lower,
                upper=upper,
                availability=availability,
                provenance="source_backed",
                sampling="held_action",
                binding={
                    "batch_telemetry": f"surface_{name}_deg",
                    "native_effector": name,
                    "family_id": family_id,
                },
                claim_boundary=(
                    "This is an actual bounded named DAVE-ML surface input emitted by the exact frozen-fixture "
                    "HL-20 source-authority screen. It does not establish hardware geometry, surface dynamics outside "
                    "the declared local lag model, six-DOF trim, feedback control, navigation, or flight qualification."
                ),
            )
            for name, lower, upper in (
                ("upper_left_body_flap", -60.0, 0.0),
                ("lower_left_body_flap", 0.0, 60.0),
                ("upper_right_body_flap", -60.0, 0.0),
                ("lower_right_body_flap", 0.0, 60.0),
                ("left_wing_flap", -30.0, 30.0),
                ("right_wing_flap", -30.0, 30.0),
                ("rudder", -30.0, 30.0),
            )
        )
    if family_id == "x15":
        availability = "available_in_batch" if batch_runnable else "planned"
        return tuple(
            InterfaceChannel(
                f"effector.surface.{name}.position",
                "effector",
                "scalar",
                "deg",
                f"Actual X-15 source-table {name.replace('_', ' ')} input after bounded three-axis source-surface allocation.",
                lower=lower,
                upper=upper,
                availability=availability,
                provenance="source_backed",
                sampling="held_action",
                binding={
                    "batch_telemetry": f"surface_{name}_deg",
                    "native_effector": name.replace("_", "-") + "-deg",
                    "family_id": family_id,
                },
                claim_boundary=(
                    "This is an actual bounded X-15 source-table surface input emitted by the exact frozen release "
                    "authority screen. It does not establish actuator dynamics, full vehicle trim, propulsion or RCS "
                    "allocation, feedback control, navigation, high-energy guidance, or flight qualification."
                ),
            )
            for name, lower, upper in (
                ("symmetric_stabilator", -14.9, 34.9),
                ("differential_stabilator", -20.05, 20.05),
                ("rudder", -29.79, 29.79),
            )
        )
    if family_id == "b747":
        availability = "available_in_batch" if batch_runnable else "planned"
        return tuple(
            InterfaceChannel(
                identifier,
                "effector",
                "scalar",
                unit,
                description,
                lower=lower,
                upper=upper,
                availability=availability,
                provenance="source_backed",
                sampling="held_action",
                binding={"batch_telemetry": telemetry, "native_effector": native, "family_id": family_id},
                claim_boundary=(
                    "This is the actual bounded B747 NASA CR-2144 condition-3 source-table coordinate emitted by the "
                    "exact local physical screen. The source package supplies no servo rate or lag data, so this does "
                    "not claim actuator-dynamics, schedule, route, or flight-envelope validation."
                ),
            )
            for identifier, unit, lower, upper, telemetry, native, description in (
                (
                    "effector.throttle.position",
                    "dimensionless",
                    0.0,
                    1.0,
                    "throttle_fraction",
                    "throttle",
                    "Actual B747 condition-3 source-table throttle coordinate during the local physical screen.",
                ),
                (
                    "effector.elevator.position",
                    "deg",
                    -10.0,
                    10.0,
                    "elevator_deg",
                    "elevator-deg",
                    "Actual B747 elevator source-table coordinate after bounded physical allocation.",
                ),
                (
                    "effector.aileron.position",
                    "deg",
                    -10.0,
                    10.0,
                    "aileron_deg",
                    "aileron-deg",
                    "Actual B747 aileron source-table coordinate after bounded physical allocation.",
                ),
                (
                    "effector.rudder.position",
                    "deg",
                    -15.0,
                    15.0,
                    "rudder_deg",
                    "rudder-deg",
                    "Actual B747 rudder source-table coordinate after bounded physical allocation.",
                ),
            )
        )
    controls = vehicle_definition.get("controls") if isinstance(vehicle_definition, Mapping) else None
    if not isinstance(controls, list | tuple):
        return ()
    fallback_availability: InterfaceAvailability = "planned" if tier_declared else "not_available"
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
                availability=fallback_availability,
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
        resources.extend(
            (
            InterfaceChannel(
                "resources.mass.total",
                "resource",
                "scalar",
                "kg",
                "Modeled total mass retained by the aggregate-thrust pseudo-6DOF plant.",
                availability=runtime_availability,
                provenance="engineering_surrogate",
                sampling="truth_boundary",
                binding={"episode_value": "mass_kg"},
                claim_boundary=(
                    "This is the configured aggregate model mass. It does not establish a payload distribution, "
                    "inertia update, fuel mass flow, or a complete mass-property ledger."
                ),
            ),
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
            ),
            )
        )
        status.append(_status("propulsion.output.thrust.aggregate", "N", "Achieved aggregate thrust in the pseudo response law.", "aggregate_thrust_n", runtime_availability))
        diagnostics.extend((_diagnostic("control.realization", "control_realization", runtime_availability), _diagnostic("control.physical_motor_allocation", "physical_motor_allocation", runtime_availability)))
    elif family_id == "hummingbird" and fidelity == "rigid_body_6dof_surface_allocated":
        status.extend(
            (
                _status(
                    "attitude.euler",
                    "rad",
                    "Source-hover local roll, pitch, and yaw error coordinates.",
                    "local_attitude_rad",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "local_attitude_rad", "frame": "source_hover_local"},
                ),
                _status(
                    "body_rate",
                    "rad/s",
                    "Source-hover local body angular rates.",
                    "body_rate_rad_s",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "body_rate_rad_s", "frame": "body"},
                ),
                _status(
                    "position.local",
                    "m",
                    "Committed source-hover local position reconstructed only over the selected bounded local screen; horizontal screens retain zero vertical coordinate and the vertical screen retains the body-z/down reconstruction.",
                    "position_local_m",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "position_local_m", "frame": "source_hover_local"},
                    provenance="derived",
                ),
                _status(
                    "velocity.local",
                    "m/s",
                    "Committed source-hover local velocity; horizontal components are expressed in the initial hover frame.",
                    "velocity_local_m_s",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "velocity_local_m_s", "frame": "source_hover_local"},
                    provenance="derived",
                ),
                _status(
                    "velocity.body",
                    "m/s",
                    "Source-hover local body velocity state.",
                    "body_velocity_m_s",
                    runtime_availability,
                    value_type="vector3",
                    binding=_local_direct_wrench_binding(family_id, "body_velocity_m_s", frame="body"),
                ),
                _status(
                    "control.wrench.requested.moment",
                    "N*m",
                    "LQI requested body moment before physical motor allocation.",
                    "requested_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "requested_moment_body_nm", "frame": "body"},
                ),
                _status(
                    "control.wrench.achieved.moment",
                    "N*m",
                    "Achieved body moment from the allocated source motor speeds.",
                    "achieved_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "achieved_moment_body_nm", "frame": "body"},
                ),
                _status(
                    "control.wrench.residual.moment",
                    "N*m",
                    "Requested-minus-achieved body-moment residual after motor allocation.",
                    "residual_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "residual_moment_body_nm", "frame": "body"},
                ),
                _status(
                    "control.wrench.status",
                    None,
                    "Bounded quad-X allocator status at the committed truth boundary.",
                    "wrench_status",
                    runtime_availability,
                    value_type="enum",
                    binding={"batch_telemetry": "wrench_status"},
                ),
                _status(
                    "control.wrench.saturated",
                    None,
                    "Whether a requested wrench or motor boundary was constrained on the committed interval.",
                    "wrench_saturated",
                    runtime_availability,
                    value_type="boolean",
                    binding={"batch_telemetry": "wrench_saturated"},
                ),
                _status(
                    "control.lqi.integral_error",
                    "rad*s",
                    "Persisted roll, pitch, and yaw output-error integrals of the local source-hover LQI controller.",
                    "lqi_integral_error_rad_s",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "lqi_integral_error_rad_s", "frame": "source_hover_local"},
                    provenance="derived",
                ),
                _status(
                    "control.lqi.integral_error.vertical_speed",
                    "m/s*s",
                    "Persisted local vertical-speed output-error integral; zero for the attitude-only and horizontal LQI screens.",
                    "integral_vertical_speed_m_s_s",
                    runtime_availability,
                    binding={"batch_telemetry": "integral_vertical_speed_m_s_s", "frame": "source_hover_local"},
                    provenance="derived",
                ),
            )
        )
        resources.append(
            InterfaceChannel(
                "resources.mass.total",
                "resource",
                "scalar",
                "kg",
                "Fixed source-hover local rigid-body mass.",
                availability=runtime_availability,
                provenance="source_backed",
                sampling="truth_boundary",
                binding={"batch_telemetry": "mass_kg"},
                claim_boundary="This source-local screen retains a fixed mass and does not establish battery, payload, inertia, or mass-flow behavior.",
            )
        )
        diagnostics.extend(
            (
                _diagnostic(
                    "control.realization",
                    "control_realization",
                    runtime_availability,
                    binding={"batch_telemetry": "control_realization"},
                ),
                _diagnostic(
                    "control.physical_motor_allocation",
                    "physical_motor_allocation",
                    runtime_availability,
                    binding={"batch_telemetry": "physical_motor_allocation"},
                ),
                _diagnostic(
                    "control.allocation.residual_norm",
                    "allocation_residual_norm",
                    runtime_availability,
                    value_type="scalar",
                    quantity_semantics="mixed_wrench_norm",
                    description="Mixed requested-minus-achieved wrench norm at the committed physical-allocation boundary.",
                    binding={"batch_telemetry": "allocation_residual_norm"},
                ),
                _diagnostic(
                    "control.allocation.saturation_count",
                    "saturation_count",
                    runtime_availability,
                    value_type="scalar",
                    quantity_semantics="count",
                    description="Number of physical allocator or actuator constraints active at the committed boundary.",
                    binding={"batch_telemetry": "saturation_count"},
                ),
            )
        )
    elif family_id == "skywalker_x8" and fidelity == "rigid_body_6dof_surface_allocated":
        status.extend(
            (
                _status(
                    "attitude.euler",
                    "rad",
                    "Committed X8 source-table local roll, pitch, and yaw-error coordinates.",
                    "local_attitude_rad",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "local_attitude_rad", "frame": "source_trim_local"},
                    provenance="source_backed",
                ),
                _status(
                    "body_rate",
                    "rad/s",
                    "Committed X8 source-table local body angular rates.",
                    "body_rate_rad_s",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "body_rate_rad_s", "frame": "body_frd"},
                    provenance="source_backed",
                ),
                _status(
                    "velocity.body",
                    "m/s",
                    "Committed X8 source-table local body velocity.",
                    "body_velocity_m_s",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "body_velocity_m_s", "frame": "body_frd"},
                    provenance="source_backed",
                ),
                _status(
                    "control.wrench.requested.moment",
                    "N*m",
                    "LQR requested body moment before bounded X8 source-coordinate allocation.",
                    "requested_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "requested_moment_body_nm", "frame": "body_frd"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.achieved.moment",
                    "N*m",
                    "Achieved body moment from the bounded X8 source-table-coordinate allocation.",
                    "achieved_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "achieved_moment_body_nm", "frame": "body_frd"},
                    provenance="source_backed",
                ),
                _status(
                    "control.wrench.residual.moment",
                    "N*m",
                    "Requested-minus-achieved body-moment residual after X8 source-coordinate allocation.",
                    "residual_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "residual_moment_body_nm", "frame": "body_frd"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.status",
                    None,
                    "Bounded X8 source-coordinate allocator disposition at the committed local-screen state.",
                    "wrench_status",
                    runtime_availability,
                    value_type="enum",
                    binding={"batch_telemetry": "wrench_status"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.saturated",
                    None,
                    "Whether a source-table coordinate or allocator boundary constrained the committed X8 screen interval.",
                    "wrench_saturated",
                    runtime_availability,
                    value_type="boolean",
                    binding={"batch_telemetry": "wrench_saturated"},
                    provenance="derived",
                ),
                _status(
                    "control.physical_effector_allocation",
                    None,
                    "Whether the X8 screen allocated requested moments to bounded source-table coordinates.",
                    "physical_effector_allocation",
                    runtime_availability,
                    value_type="boolean",
                    binding={"batch_telemetry": "physical_effector_allocation"},
                    provenance="derived",
                ),
            )
        )
        resources.append(
            InterfaceChannel(
                "resources.mass.total",
                "resource",
                "scalar",
                "kg",
                "Fixed source-table X8 mass used by the local physical-control screen.",
                availability=runtime_availability,
                provenance="source_backed",
                sampling="truth_boundary",
                binding={"batch_telemetry": "mass_kg"},
                claim_boundary="The local screen retains fixed source mass and does not establish fuel, battery, payload, or mass-property scheduling.",
            )
        )
        diagnostics.extend(
            (
                _diagnostic(
                    "control.realization",
                    "control_realization",
                    runtime_availability,
                    binding={"batch_telemetry": "control_realization"},
                ),
                _diagnostic(
                    "control.physical_effector_allocation",
                    "physical_effector_allocation",
                    runtime_availability,
                    binding={"batch_telemetry": "physical_effector_allocation"},
                ),
                _diagnostic(
                    "control.allocation.residual_norm",
                    "allocation_controlled_residual_norm",
                    runtime_availability,
                    value_type="scalar",
                    unit="N*m",
                    description="Mixed requested-minus-achieved moment norm at the committed X8 allocator boundary.",
                    binding={"batch_telemetry": "allocation_controlled_residual_norm"},
                ),
                _diagnostic(
                    "control.allocation.saturation_count",
                    "saturation_count",
                    runtime_availability,
                    value_type="scalar",
                    quantity_semantics="count",
                    description="Number of X8 allocator or source-coordinate constraints active at the committed boundary.",
                    binding={"batch_telemetry": "saturation_count"},
                ),
            )
        )
    elif family_id == "x15" and fidelity == "rigid_body_6dof_surface_allocated":
        status.extend(
            (
                _status(
                    "velocity.body",
                    "m/s",
                    "Frozen source-release body velocity used by the X-15 source-surface authority screen.",
                    "body_velocity_m_s",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "body_velocity_m_s", "frame": "body"},
                    provenance="source_backed",
                ),
                _status(
                    "body_rate",
                    "rad/s",
                    "Frozen source-release body rate used by the X-15 source-surface authority screen.",
                    "body_rate_rad_s",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "body_rate_rad_s", "frame": "body"},
                    provenance="source_backed",
                ),
                _status(
                    "aerodynamics.mach",
                    "dimensionless",
                    "Mach evaluated by the committed nonlinear X-15 source-table load query.",
                    "source_mach",
                    runtime_availability,
                    binding={"batch_telemetry": "source_mach"},
                    provenance="source_backed",
                ),
                _status(
                    "aerodynamics.alpha",
                    "deg",
                    "Angle of attack evaluated by the committed nonlinear X-15 source-table load query.",
                    "source_alpha_deg",
                    runtime_availability,
                    binding={"batch_telemetry": "source_alpha_deg", "frame": "body"},
                    provenance="source_backed",
                ),
                _status(
                    "aerodynamics.beta",
                    "deg",
                    "Sideslip evaluated by the committed nonlinear X-15 source-table load query.",
                    "source_beta_deg",
                    runtime_availability,
                    binding={"batch_telemetry": "source_beta_deg", "frame": "body"},
                    provenance="source_backed",
                ),
                _status(
                    "control.wrench.requested.moment",
                    "N*m",
                    "Requested three-axis body moment used by the bounded X-15 source-surface allocator.",
                    "requested_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "requested_moment_body_nm", "frame": "body"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.achieved.moment",
                    "N*m",
                    "Actual nonlinear X-15 source-table body moment after the committed named surface positions.",
                    "achieved_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "achieved_moment_body_nm", "frame": "body"},
                    provenance="source_backed",
                ),
                _status(
                    "control.wrench.residual.moment",
                    "N*m",
                    "Requested-minus-nonlinear-source three-axis moment residual at the committed allocation boundary.",
                    "residual_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "residual_moment_body_nm", "frame": "body"},
                    provenance="derived",
                ),
                _status("control.wrench.status", None, "Bounded X-15 source-surface allocator disposition.", "wrench_status", runtime_availability, value_type="enum", binding={"batch_telemetry": "wrench_status"}, provenance="derived"),
                _status("control.wrench.saturated", None, "Whether a surface position constraint limited the committed X-15 allocation.", "wrench_saturated", runtime_availability, value_type="boolean", binding={"batch_telemetry": "wrench_saturated"}, provenance="derived"),
                _status("control.physical_effector_allocation", None, "Whether this screen allocated its three-axis moment request to all three bounded X-15 source surfaces.", "physical_effector_allocation", runtime_availability, value_type="boolean", binding={"batch_telemetry": "physical_effector_allocation"}, provenance="derived"),
                _status("trim.full_state.status", None, "Availability of a full X-15 source equilibrium for the selected screen.", "full_state_trim_status", runtime_availability, value_type="enum", binding={"batch_telemetry": "full_state_trim_status"}, provenance="derived"),
            )
        )
        resources.append(
            InterfaceChannel(
                "resources.mass.total",
                "resource",
                "scalar",
                "kg",
                "Fixed X-15 source-release mass used by the frozen source-surface authority screen.",
                availability=runtime_availability,
                provenance="source_backed",
                sampling="truth_boundary",
                binding={"batch_telemetry": "mass_kg"},
                claim_boundary="The screen holds this source mass fixed and does not establish fuel, propellant, inertia, or mass-property scheduling.",
            )
        )
        diagnostics.extend(
            (
                _diagnostic("control.realization", "control_realization", runtime_availability, binding={"batch_telemetry": "control_realization"}),
                _diagnostic("control.physical_effector_allocation", "physical_effector_allocation", runtime_availability, binding={"batch_telemetry": "physical_effector_allocation"}),
                _diagnostic("control.allocation.residual_norm", "allocation_controlled_residual_norm", runtime_availability, value_type="scalar", unit="N*m", description="Controlled moment residual reported by the bounded X-15 source-surface allocator.", binding={"batch_telemetry": "allocation_controlled_residual_norm"}),
                _diagnostic("control.allocation.saturation_count", "saturation_count", runtime_availability, value_type="scalar", quantity_semantics="count", description="Number of active X-15 source-surface allocator constraints.", binding={"batch_telemetry": "saturation_count"}),
                _diagnostic("control.source_effectiveness_rank", "source_effectiveness_rank", runtime_availability, value_type="scalar", quantity_semantics="count", description="Rank of the source-load finite-difference three-surface effectiveness matrix at the frozen fixture.", binding={"batch_telemetry": "source_effectiveness_rank"}),
            )
        )
    elif family_id == "hl20_mod_k" and fidelity == "rigid_body_6dof_surface_allocated":
        status.extend(
            (
                _status(
                    "attitude.euler",
                    "rad",
                    "Committed HL-20 local roll, pitch, and yaw-error coordinates when the selected source-surface screen closes the local attitude loop.",
                    "local_attitude_rad",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "local_attitude_rad", "frame": "source_pitch_trim_local"},
                    provenance="derived",
                ),
                _status(
                    "velocity.body",
                    "m/s",
                    "Frozen Mach-1 DAVE-ML body-velocity fixture used by the source-surface local screens.",
                    "body_velocity_m_s",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "body_velocity_m_s", "frame": "body"},
                    provenance="source_backed",
                ),
                _status(
                    "body_rate",
                    "rad/s",
                    "Committed DAVE-ML body-rate state at the frozen source-translation fixture.",
                    "body_rate_rad_s",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "body_rate_rad_s", "frame": "body"},
                    provenance="source_backed",
                ),
                _status(
                    "aerodynamics.pitch_coefficient",
                    "dimensionless",
                    "Nonlinear DAVE-ML pitch coefficient evaluated after the committed actual source-surface positions.",
                    "source_pitch_coefficient",
                    runtime_availability,
                    binding={"batch_telemetry": "source_pitch_coefficient"},
                    provenance="source_backed",
                ),
                _status(
                    "control.pitch_moment.requested",
                    "N*m",
                    "Requested total source-fixture pitch moment used by the bounded seven-surface allocator.",
                    "requested_pitch_moment_nm",
                    runtime_availability,
                    binding={"batch_telemetry": "requested_pitch_moment_nm", "frame": "body"},
                    provenance="derived",
                ),
                _status(
                    "control.pitch_moment.achieved",
                    "N*m",
                    "Actual nonlinear DAVE-ML pitch moment after the committed named source-surface positions.",
                    "achieved_pitch_moment_nm",
                    runtime_availability,
                    binding={"batch_telemetry": "achieved_pitch_moment_nm", "frame": "body"},
                    provenance="source_backed",
                ),
                _status(
                    "control.pitch_moment.residual",
                    "N*m",
                    "Requested-minus-nonlinear-source pitch-moment residual at the committed allocation boundary.",
                    "pitch_moment_residual_nm",
                    runtime_availability,
                    binding={"batch_telemetry": "pitch_moment_residual_nm", "frame": "body"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.status",
                    None,
                    "Bounded source-surface allocator disposition for the pitch-authority request.",
                    "wrench_status",
                    runtime_availability,
                    value_type="enum",
                    binding={"batch_telemetry": "wrench_status"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.saturated",
                    None,
                    "Whether a source-surface position, rate, or lag constraint limited the committed allocation interval.",
                    "wrench_saturated",
                    runtime_availability,
                    value_type="boolean",
                    binding={"batch_telemetry": "wrench_saturated"},
                    provenance="derived",
                ),
                _status(
                    "control.physical_effector_allocation",
                    None,
                    "Whether this screen allocated its pitch request to all seven bounded named source surfaces.",
                    "physical_effector_allocation",
                    runtime_availability,
                    value_type="boolean",
                    binding={"batch_telemetry": "physical_effector_allocation"},
                    provenance="derived",
                ),
                _status(
                    "trim.full_state.status",
                    None,
                    "Availability of a full-state HL-20 source equilibrium for the selected screen.",
                    "full_state_trim_status",
                    runtime_availability,
                    value_type="enum",
                    binding={"batch_telemetry": "full_state_trim_status"},
                    provenance="derived",
                ),
            )
        )
        resources.append(
            InterfaceChannel(
                "resources.mass.total",
                "resource",
                "scalar",
                "kg",
                "Fixed DAVE-ML HL-20 source mass used by the frozen source-surface authority screen.",
                availability=runtime_availability,
                provenance="source_backed",
                sampling="truth_boundary",
                binding={"batch_telemetry": "mass_kg"},
                claim_boundary="The screen holds this source mass fixed and does not establish fuel, propellant, inertia, or mass-property scheduling.",
            )
        )
        diagnostics.extend(
            (
                _diagnostic("control.realization", "control_realization", runtime_availability, binding={"batch_telemetry": "control_realization"}),
                _diagnostic("control.physical_effector_allocation", "physical_effector_allocation", runtime_availability, binding={"batch_telemetry": "physical_effector_allocation"}),
                _diagnostic(
                    "control.allocation.residual_norm",
                    "allocation_controlled_residual_norm",
                    runtime_availability,
                    value_type="scalar",
                    unit="N*m",
                    description="Linearized controlled-axis residual reported by the bounded HL-20 source-surface allocator.",
                    binding={"batch_telemetry": "allocation_controlled_residual_norm"},
                ),
                _diagnostic(
                    "control.allocation.saturation_count",
                    "saturation_count",
                    runtime_availability,
                    value_type="scalar",
                    quantity_semantics="count",
                    description="Number of active HL-20 source-surface allocator or local actuator constraints.",
                    binding={"batch_telemetry": "saturation_count"},
                ),
                _diagnostic(
                    "control.source_effectiveness_rank",
                    "source_effectiveness_rank",
                    runtime_availability,
                    value_type="scalar",
                    quantity_semantics="count",
                    description="Rank of the source-load finite-difference seven-surface effectiveness matrix at the frozen fixture.",
                    binding={"batch_telemetry": "source_effectiveness_rank"},
                ),
            )
        )
    elif family_id == "b747" and fidelity == "rigid_body_6dof_surface_allocated":
        status.extend(
            (
                _status(
                    "attitude.euler",
                    "rad",
                    "Committed B747 condition-3 local roll, pitch, and yaw-error coordinates.",
                    "local_attitude_rad",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "local_attitude_rad", "frame": "source_trim_local"},
                    provenance="source_backed",
                ),
                _status(
                    "body_rate",
                    "rad/s",
                    "Committed B747 condition-3 local body angular rates.",
                    "body_rate_rad_s",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "body_rate_rad_s", "frame": "body_frd"},
                    provenance="source_backed",
                ),
                _status(
                    "velocity.body",
                    "m/s",
                    "Committed B747 condition-3 local body velocity.",
                    "body_velocity_m_s",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "body_velocity_m_s", "frame": "body_frd"},
                    provenance="source_backed",
                ),
                _status(
                    "control.wrench.requested.moment",
                    "N*m",
                    "LQR requested body moment before bounded B747 source-table surface allocation.",
                    "requested_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "requested_moment_body_nm", "frame": "body_frd"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.achieved.moment",
                    "N*m",
                    "Achieved body moment from bounded B747 condition-3 source-table surface allocation.",
                    "achieved_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "achieved_moment_body_nm", "frame": "body_frd"},
                    provenance="source_backed",
                ),
                _status(
                    "control.wrench.residual.moment",
                    "N*m",
                    "Requested-minus-achieved body-moment residual after B747 source-table allocation.",
                    "residual_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "residual_moment_body_nm", "frame": "body_frd"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.status",
                    None,
                    "Bounded B747 source-table allocator disposition at the committed condition-3 screen state.",
                    "wrench_status",
                    runtime_availability,
                    value_type="enum",
                    binding={"batch_telemetry": "wrench_status"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.saturated",
                    None,
                    "Whether a B747 source-table coordinate or allocator boundary constrained the committed screen interval.",
                    "wrench_saturated",
                    runtime_availability,
                    value_type="boolean",
                    binding={"batch_telemetry": "wrench_saturated"},
                    provenance="derived",
                ),
                _status(
                    "control.physical_effector_allocation",
                    None,
                    "Whether the B747 screen allocated requested moments to bounded source-table surface coordinates.",
                    "physical_effector_allocation",
                    runtime_availability,
                    value_type="boolean",
                    binding={"batch_telemetry": "physical_effector_allocation"},
                    provenance="derived",
                ),
            )
        )
        resources.append(
            InterfaceChannel(
                "resources.mass.total",
                "resource",
                "scalar",
                "kg",
                "Fixed B747 condition-3 source-table mass used by the local physical-control screen.",
                availability=runtime_availability,
                provenance="source_backed",
                sampling="truth_boundary",
                binding={"batch_telemetry": "mass_kg"},
                claim_boundary="The local screen retains fixed source mass and does not establish fuel, payload, inertia, or mass-property scheduling.",
            )
        )
        diagnostics.extend(
            (
                _diagnostic(
                    "control.realization",
                    "control_realization",
                    runtime_availability,
                    binding={"batch_telemetry": "control_realization"},
                ),
                _diagnostic(
                    "control.physical_effector_allocation",
                    "physical_effector_allocation",
                    runtime_availability,
                    binding={"batch_telemetry": "physical_effector_allocation"},
                ),
                _diagnostic(
                    "control.allocation.residual_norm",
                    "allocation_controlled_residual_norm",
                    runtime_availability,
                    value_type="scalar",
                    unit="N*m",
                    description="Mixed requested-minus-achieved moment norm at the committed B747 allocator boundary.",
                    binding={"batch_telemetry": "allocation_controlled_residual_norm"},
                ),
                _diagnostic(
                    "control.allocation.saturation_count",
                    "saturation_count",
                    runtime_availability,
                    value_type="scalar",
                    quantity_semantics="count",
                    description="Number of B747 allocator or source-surface constraints active at the committed boundary.",
                    binding={"batch_telemetry": "saturation_count"},
                ),
            )
        )
    elif family_id in {"skywalker_x8", "b747"} and fidelity in {"point_mass_3dof", "pseudo_6dof"}:
        status.extend(
            (
                _status("position.altitude", "m", "Geodetic altitude from the committed runtime state.", "1.alt", runtime_availability, scale=0.3048),
                _status("velocity.speed", "m/s", "Scalar speed from the committed runtime state.", "1.vel", runtime_availability, scale=0.3048),
                _status("flight.path_angle", "deg", "Geodetic flight-path angle from the committed runtime state.", "1.gama", runtime_availability),
                _status("flight.heading", "deg", "Geodetic heading from the committed runtime state.", "1.psi", runtime_availability),
                _status(
                    "guidance.override.active",
                    None,
                    "Whether the committed lower-tier state-rate command came from the explicit external guidance authority.",
                    "1.guidance_override_active",
                    runtime_availability,
                    value_type="boolean",
                    binding={
                        "derived_from": "episode_value.1.guidance_override_active",
                        "transform": "positive_boolean",
                    },
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
                        "Profile-backed kinematic pseudo-6DOF Euler attitude at the committed truth boundary.",
                        "1",
                        runtime_availability,
                        value_type="vector3",
                        binding={
                            "derived_from": "episode_value.1",
                            "transform": "kinematic_attitude_deg_vector",
                            "frame": "kinematic_body_to_reference",
                        },
                        provenance="derived",
                    ),
                    _status(
                        "body_rate",
                        "rad/s",
                        "Profile-backed kinematic pseudo-6DOF body rate at the committed truth boundary.",
                        "1",
                        runtime_availability,
                        value_type="vector3",
                        binding={
                            "derived_from": "episode_value.1",
                            "transform": "kinematic_body_rate_vector",
                            "frame": "body",
                        },
                        provenance="derived",
                    ),
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
        if family_id == "a320_openap_3dof":
            status.extend(
                (
                    _status(
                        "propulsion.thrust",
                        "N",
                        "Installed thrust evaluated by the committed OpenAP operating point.",
                        "thrust_n",
                        runtime_availability,
                        binding={"batch_telemetry": "thrust_n", "frame": "body"},
                        provenance="derived",
                    ),
                    _status(
                        "control.throttle.realized",
                        "1",
                        "Throttle ratio realized by the committed OpenAP reduced-model operating point.",
                        "throttle_ratio",
                        runtime_availability,
                        binding={"batch_telemetry": "throttle_ratio"},
                        provenance="derived",
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
        if family_id == "a320_openap_3dof":
            resources.append(
                InterfaceChannel(
                    "resources.mass.fuel_flow",
                    "resource",
                    "scalar",
                    "kg/s",
                    "OpenAP fuel-flow estimate at the committed reduced-model operating point.",
                    availability=runtime_availability,
                    provenance="derived",
                    sampling="truth_boundary",
                    binding={"batch_telemetry": "fuel_flow_kg_s"},
                    claim_boundary=(
                        "This is the OpenAP performance-model fuel-flow estimate for the selected composition. It does "
                        "not establish an independent fuel-system state, engine spool model, reserve policy, or physical "
                        "A320 propulsion qualification."
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
    elif family_id == "f16_s119" and fidelity in {
        "rigid_body_6dof_direct_wrench",
        "rigid_body_6dof_surface_allocated",
    }:
        status.extend(
            (
                _status(
                    "position.north",
                    "m",
                    "Committed F-16 north context: route-integrated for the route-entry screen or fixed local origin for the exact fixed-altitude LQI screen.",
                    "north_m",
                    runtime_availability,
                    binding={"batch_telemetry": "north_m", "frame": "local_navigation"},
                ),
                _status(
                    "position.east",
                    "m",
                    "Committed F-16 east context: route-integrated for the route-entry screen or fixed local origin for the exact fixed-altitude LQI screen.",
                    "east_m",
                    runtime_availability,
                    binding={"batch_telemetry": "east_m", "frame": "local_navigation"},
                ),
                _status(
                    "position.altitude",
                    "m",
                    "Committed F-16 altitude context: route-integrated for the route-entry screen or fixed source derivative altitude for the exact LQI screen.",
                    "altitude_m",
                    runtime_availability,
                    binding={"batch_telemetry": "altitude_m", "frame": "local_navigation"},
                ),
                _status(
                    "velocity.speed",
                    "m/s",
                    "Committed F-16 body-speed magnitude.",
                    "speed_m_s",
                    runtime_availability,
                    binding={"batch_telemetry": "speed_m_s"},
                ),
                _status(
                    "velocity.body",
                    "m/s",
                    "Committed F-16 source-plant body velocity.",
                    "body_velocity_m_s",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "body_velocity_m_s", "frame": "body_frd"},
                ),
                _status(
                    "attitude.euler",
                    "deg",
                    "Committed F-16 Euler context: route-integrated for the route-entry screen or fixed source trim attitude for the exact LQI screen.",
                    "attitude_euler_deg",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "attitude_euler_deg", "frame": "local_navigation"},
                ),
                _status(
                    "body_rate",
                    "rad/s",
                    "Committed F-16 source-plant body angular rate.",
                    "body_rate_rad_s",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "body_rate_rad_s", "frame": "body_frd"},
                ),
                _status(
                    "control.wrench.requested.force",
                    "N",
                    "Requested local physical-controller body force.",
                    "requested_force_body_n",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "requested_force_body_n", "frame": "body_frd"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.requested.moment",
                    "N*m",
                    "Requested local physical-controller body moment.",
                    "requested_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "requested_moment_body_nm", "frame": "body_frd"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.achieved.force",
                    "N",
                    "Achieved F-16 source-plant body force on the local screen.",
                    "achieved_force_body_n",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "achieved_force_body_n", "frame": "body_frd"},
                ),
                _status(
                    "control.wrench.achieved.moment",
                    "N*m",
                    "Achieved F-16 source-plant body moment on the local screen.",
                    "achieved_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "achieved_moment_body_nm", "frame": "body_frd"},
                ),
                _status(
                    "control.wrench.residual.force",
                    "N",
                    "Requested-minus-achieved F-16 body force on the local screen.",
                    "residual_force_body_n",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "residual_force_body_n", "frame": "body_frd"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.residual.moment",
                    "N*m",
                    "Requested-minus-achieved F-16 body moment on the local screen.",
                    "residual_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding={"batch_telemetry": "residual_moment_body_nm", "frame": "body_frd"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.status",
                    None,
                    "F-16 direct-wrench or allocator disposition at the committed local-screen state.",
                    "wrench_status",
                    runtime_availability,
                    value_type="enum",
                    binding={"batch_telemetry": "wrench_status"},
                    provenance="derived",
                ),
                _status(
                    "control.wrench.saturated",
                    None,
                    "Whether the F-16 local screen reported an actuator or allocation saturation.",
                    "wrench_saturated",
                    runtime_availability,
                    value_type="boolean",
                    binding={"batch_telemetry": "wrench_saturated"},
                    provenance="derived",
                ),
                _status(
                    "control.physical_effector_allocation",
                    None,
                    "Whether this F-16 physical-control screen allocates the requested wrench to effectors.",
                    "physical_effector_allocation",
                    runtime_availability,
                    value_type="boolean",
                    binding={"batch_telemetry": "physical_effector_allocation"},
                    provenance="derived",
                ),
                _status(
                    "control.schedule.node_id",
                    None,
                    "Selected source-trim F-16 node for the committed local control sample.",
                    "schedule_node_id",
                    runtime_availability,
                    value_type="enum",
                    binding={"batch_telemetry": "schedule_node_id"},
                    provenance="source_backed",
                ),
                _status(
                    "control.schedule.selection",
                    None,
                    "How the F-16 local controller selected its source schedule node for the committed sample.",
                    "controller_selection",
                    runtime_availability,
                    value_type="enum",
                    binding={"batch_telemetry": "controller_selection"},
                    provenance="derived",
                ),
            )
        )
        resources.append(
            InterfaceChannel(
                "resources.mass.total",
                "resource",
                "scalar",
                "kg",
                "Fixed source-bound F-16 mass used by the local physical-control screen.",
                availability=runtime_availability,
                provenance="source_backed",
                sampling="truth_boundary",
                binding={"batch_telemetry": "mass_kg"},
                claim_boundary="The screen uses a fixed source-bound mass and does not model fuel depletion or mass-property scheduling.",
            )
        )
        diagnostics.extend(
            (
                _diagnostic(
                    "control.realization",
                    resolved_control_realization_for(family_id, fidelity),
                    runtime_availability,
                    binding={"batch_telemetry": "control_realization"},
                ),
                _diagnostic(
                    "control.allocation.residual_norm",
                    "allocation_residual_norm",
                    runtime_availability,
                    value_type="scalar",
                    quantity_semantics="mixed_wrench_norm",
                    description="Mixed requested-minus-achieved wrench norm at the committed F-16 control boundary.",
                    binding={"batch_telemetry": "allocation_residual_norm"},
                ),
                _diagnostic(
                    "control.allocation.saturation_count",
                    "saturation_count",
                    runtime_availability,
                    value_type="scalar",
                    quantity_semantics="count",
                    description="Number of F-16 allocator or actuator constraints active at the committed boundary.",
                    binding={"batch_telemetry": "saturation_count"},
                ),
            )
        )
    elif family_id in {"hummingbird", "x15", "hl20_mod_k"} and fidelity == "rigid_body_6dof_direct_wrench":
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
                    binding=_local_direct_wrench_binding(family_id, "body_rate_rad_s", frame="body"),
                    provenance="source_backed",
                ),
                _status(
                    "control.wrench.requested.force",
                    "N",
                    "Bounded-screen requested body-frame direct force.",
                    "requested_force_body_n",
                    runtime_availability,
                    value_type="vector3",
                    binding=_local_direct_wrench_binding(family_id, "requested_force_body_n", frame="body"),
                    provenance="derived",
                ),
                _status(
                    "control.wrench.requested.moment",
                    "N*m",
                    "Bounded-screen requested body-frame direct moment.",
                    "requested_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding=_local_direct_wrench_binding(family_id, "requested_moment_body_nm", frame="body"),
                    provenance="derived",
                ),
                _status(
                    "control.wrench.achieved.force",
                    "N",
                    "Projected body-frame direct force actually applied to the local source plant.",
                    "achieved_force_body_n",
                    runtime_availability,
                    value_type="vector3",
                    binding=_local_direct_wrench_binding(family_id, "achieved_force_body_n", frame="body"),
                    provenance="engineering_surrogate",
                ),
                _status(
                    "control.wrench.achieved.moment",
                    "N*m",
                    "Projected body-frame direct moment actually applied to the local source plant.",
                    "achieved_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding=_local_direct_wrench_binding(family_id, "achieved_moment_body_nm", frame="body"),
                    provenance="engineering_surrogate",
                ),
                _status(
                    "control.wrench.residual.force",
                    "N",
                    "Requested-minus-achieved body-force residual after direct-wrench projection.",
                    "residual_force_body_n",
                    runtime_availability,
                    value_type="vector3",
                    binding=_local_direct_wrench_binding(family_id, "residual_force_body_n", frame="body"),
                    provenance="derived",
                ),
                _status(
                    "control.wrench.residual.moment",
                    "N*m",
                    "Requested-minus-achieved body-moment residual after direct-wrench projection.",
                    "residual_moment_body_nm",
                    runtime_availability,
                    value_type="vector3",
                    binding=_local_direct_wrench_binding(family_id, "residual_moment_body_nm", frame="body"),
                    provenance="derived",
                ),
                _status(
                    "control.wrench.status",
                    None,
                    "Direct-wrench projection disposition at the committed local-screen state.",
                    "wrench_status",
                    runtime_availability,
                    value_type="enum",
                    binding=_local_direct_wrench_binding(family_id, "wrench_status"),
                    provenance="derived",
                ),
                _status(
                    "control.wrench.saturated",
                    None,
                    "Whether direct-wrench authority or slew projection limited this local-screen request.",
                    "wrench_saturated",
                    runtime_availability,
                    value_type="boolean",
                    binding=_local_direct_wrench_binding(family_id, "wrench_saturated"),
                    provenance="derived",
                ),
                _status(
                    "control.physical_effector_allocation",
                    None,
                    "Whether the selected control path allocates the direct-wrench request to physical effectors.",
                    "physical_effector_allocation",
                    runtime_availability,
                    value_type="boolean",
                    binding=_local_direct_wrench_binding(family_id, "physical_effector_allocation"),
                    provenance="derived",
                ),
            )
        )
        if family_id == "hummingbird":
            resources.append(
                InterfaceChannel(
                    "resources.mass.total",
                    "resource",
                    "scalar",
                    "kg",
                    "Pinned source-hover rigid-body mass used by the local direct-wrench comparator.",
                    availability=runtime_availability,
                    provenance="source_backed",
                    sampling="truth_boundary",
                    binding={"batch_telemetry": "mass_kg"},
                    claim_boundary=(
                        "The comparator holds this source-hover mass fixed. It does not establish battery, payload, "
                        "inertia, or mass-flow behavior."
                    ),
                )
            )
        elif family_id == "x15":
            resources.append(
                InterfaceChannel(
                    "resources.mass.total",
                    "resource",
                    "scalar",
                    "kg",
                    "Fixed source-release mass used by the local X-15 direct-wrench screen.",
                    availability=runtime_availability,
                    provenance="source_backed",
                    sampling="truth_boundary",
                    binding={"batch_telemetry": "mass_kg", "episode_value": "mass_kg"},
                    claim_boundary=(
                        "The local source-release screen holds the declared mass fixed. It does not establish fuel, "
                        "propellant, inertia, or mass-property scheduling."
                    ),
                )
            )
        elif family_id == "hl20_mod_k":
            resources.append(
                InterfaceChannel(
                    "resources.mass.total",
                    "resource",
                    "scalar",
                    "kg",
                    "Fixed DAVE-ML source mass used by the local HL-20 direct-wrench screen.",
                    availability=runtime_availability,
                    provenance="source_backed",
                    sampling="truth_boundary",
                    binding={"batch_telemetry": "mass_kg", "episode_value": "mass_kg"},
                    claim_boundary=(
                        "The local source screen holds the declared mass fixed. It does not establish fuel, propellant, "
                        "inertia, or mass-property scheduling."
                    ),
                )
            )
        diagnostics.extend(
            (
                _diagnostic(
                    "control.realization",
                    "direct_wrench_screen",
                    runtime_availability,
                    binding=_local_direct_wrench_binding(family_id, "control_realization"),
                ),
                _diagnostic(
                    "control.wrench.residual_norm",
                    "wrench_residual_norm",
                    runtime_availability,
                    value_type="scalar",
                    quantity_semantics="mixed_wrench_norm",
                    description="Mixed requested-minus-achieved wrench norm at the committed direct-wrench boundary.",
                    binding=_local_direct_wrench_binding(family_id, "wrench_residual_norm"),
                ),
                _diagnostic(
                    "control.feedback_norm",
                    "feedback_norm",
                    runtime_availability,
                    value_type="scalar",
                    quantity_semantics="normalized_error",
                    description="Native feedback-error norm retained from the direct-wrench local screen.",
                    binding=_local_direct_wrench_binding(family_id, "feedback_norm"),
                ),
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
                    "aerodynamics.drag_force",
                    "N",
                    "Committed aerodynamic drag-force magnitude from the passive-body truth model.",
                    "drag_force_n",
                    runtime_availability,
                    binding={"batch_telemetry": "drag_force_n", "frame": "local_reduced"},
                    provenance="engineering_surrogate",
                ),
                _status(
                    "aerodynamics.projected_area",
                    "m^2",
                    "Committed projected area used by the selected passive-body aerodynamic representation.",
                    "projected_area_m2",
                    runtime_availability,
                    binding={"batch_telemetry": "projected_area_m2", "frame": "body"},
                    provenance="engineering_surrogate",
                ),
                _status(
                    "angular_rate.norm",
                    "rad/s",
                    "Committed angular-rate magnitude; zero in the orientation-averaged 3DOF reduction.",
                    "angular_rate_norm_rad_s",
                    runtime_availability,
                    binding={"batch_telemetry": "angular_rate_norm_rad_s", "frame": "body"},
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
    diagnostics.append(
        _diagnostic(
            "control.controller.method",
            "controller_method",
            runtime_availability,
            binding={"batch_telemetry": "controller_method", "episode_value": "controller_method"},
            value_type="enum",
            description=(
                "Executed reusable controller family for this committed sample: lqr, lqi, or not_applicable "
                "when this execution mode has no such controller."
            ),
        )
    )
    return tuple(status), tuple(resources), tuple(diagnostics)
    ####


def _local_direct_wrench_binding(
    family_id: str,
    native: str,
    *,
    frame: str | None = None,
) -> dict[str, object]:
    """Declare batch and, where available, episode projection of local-screen truth."""

    binding: dict[str, object] = {"batch_telemetry": native}
    if family_id in {"x15", "hl20_mod_k"}:
        binding["episode_value"] = native
    if frame is not None:
        binding["frame"] = frame
    return binding
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
    value_type: InterfaceValueType | None = None,
    unit: str | None = None,
    quantity_semantics: QuantitySemantics | None = None,
    description: str | None = None,
) -> InterfaceChannel:
    return InterfaceChannel(
        identifier,
        "diagnostic",
        value_type or ("enum" if identifier in {"control.realization", "control.controller.method"} else "boolean"),
        unit,
        description or "Raw claim-boundary diagnostic retained with the canonical status view.",
        quantity_semantics=quantity_semantics,
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
    enforce_bounds: bool = True,
    enforce_value_space: bool = True,
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
        if enforce_bounds and channel.lower is not None and numeric < channel.lower:
            raise ValueError(f"{context}: channel {channel.id!r} is below its declared lower bound")
        if enforce_bounds and channel.upper is not None and numeric > channel.upper:
            raise ValueError(f"{context}: channel {channel.id!r} exceeds its declared upper bound")
    elif channel.value_type in {"vector3", "vector4"}:
        expected_size = 3 if channel.value_type == "vector3" else 4
        if not isinstance(value, list | tuple) or len(value) != expected_size:
            raise ValueError(f"{context}: channel {channel.id!r} requires a {channel.value_type}")
        for item in value:
            if isinstance(item, bool) or not isinstance(item, int | float) or not math.isfinite(float(item)):
                raise ValueError(f"{context}: channel {channel.id!r} requires finite vector components")
    elif channel.value_type == "boolean":
        if not isinstance(value, bool):
            raise ValueError(f"{context}: channel {channel.id!r} requires a boolean")
    elif channel.value_type in {"enum", "event"}:
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{context}: channel {channel.id!r} requires a non-empty string")
    else:
        raise ValueError(f"{context}: channel {channel.id!r} has an unsupported value type")
    if channel.value_space is None:
        raise ValueError(f"{context}: channel {channel.id!r} has no declared value space")
    if enforce_value_space:
        validate_value_space_value(channel.value_space, value, context=f"{context}: channel {channel.id!r}")
    ####


def validate_authority_action_values(
    contract: VehicleInterfaceContract,
    authority_profile_id: str,
    values: Mapping[str, object],
    *,
    context: str = "semantic action",
    enforce_bounds: bool = True,
) -> AuthorityProfile:
    """Validate one public action payload against its selected authority.

    This is the common semantic boundary for every future batch, episode, or
    policy adapter. It checks authority availability, membership, declared
    primitive bounds, and value-space invariants before an implementation maps
    the request into native controls, a wrench bridge, or physical effectors.
    Partial payloads remain valid when the selected authority permits held
    commands; a caller never receives undeclared action coordinates.
    """

    profile = contract.authority_profile(authority_profile_id)
    if profile.availability != "available":
        raise ValueError(f"authority profile {profile.id!r} is {profile.availability}, not executable")
    unknown = sorted(set(values) - set(profile.action_ids))
    if unknown:
        raise ValueError(f"{context} contains values outside {profile.id!r}: {', '.join(unknown)}")
    channels = {channel.id: channel for channel in contract.action_channels}
    for identifier, value in values.items():
        channel = channels[identifier]
        if channel.availability != "available":
            raise ValueError(f"{context}: action channel {identifier!r} is {channel.availability}, not executable")
        validate_interface_channel_value(
            channel,
            value,
            context=context,
            enforce_bounds=enforce_bounds,
            enforce_value_space=enforce_bounds,
        )
    return profile
    ####


def project_authority_action_values(
    contract: VehicleInterfaceContract,
    authority_profile_id: str,
    values: Mapping[str, object],
    *,
    context: str = "semantic action",
) -> dict[str, object]:
    """Project a finite semantic command into its declared scalar bounds.

    Policy and interactive callers may propose a command outside the portable
    action interval.  That proposal remains useful diagnostic evidence, but
    the plant must receive the bounded command declared by the interface.  The
    projection is explicit and deterministic; unsupported coordinates and
    malformed values remain errors rather than being silently repaired.
    """

    validate_authority_action_values(
        contract,
        authority_profile_id,
        values,
        context=context,
        enforce_bounds=False,
    )
    channels = {channel.id: channel for channel in contract.action_channels}
    projected: dict[str, object] = {}
    for identifier, value in values.items():
        channel = channels[identifier]
        if channel.value_type != "scalar":
            projected[identifier] = value
            continue
        if isinstance(value, bool) or not isinstance(value, int | float):
            raise AssertionError(f"validated scalar action {identifier!r} lost its numeric representation")
        numeric = float(value)
        if channel.lower is not None:
            numeric = max(channel.lower, numeric)
        if channel.upper is not None:
            numeric = min(channel.upper, numeric)
        projected[identifier] = numeric
    validate_authority_action_values(contract, authority_profile_id, projected, context=context)
    return projected
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
    if value is _STATUS_MISSING:
        if channel.id == "control.realization":
            return contract.control_realization
        if channel.id == "control.controller.method":
            return "not_applicable"
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
    if transform == "positive_boolean":
        if isinstance(value, bool):
            return value
        if isinstance(value, int | float) and math.isfinite(float(value)):
            return float(value) >= 0.5
        return _STATUS_MISSING
    if transform in {"kinematic_attitude_deg_vector", "kinematic_body_rate_vector"}:
        if not isinstance(value, Mapping):
            return _STATUS_MISSING
        component_names = (
            ("kinematic_roll_deg", "kinematic_pitch_deg", "kinematic_yaw_deg")
            if transform == "kinematic_attitude_deg_vector"
            else (
                "kinematic_body_rate_p_rad_s",
                "kinematic_body_rate_q_rad_s",
                "kinematic_body_rate_r_rad_s",
            )
        )
        kinematic_components: list[float] = []
        for name in component_names:
            component = value.get(name)
            if isinstance(component, bool) or not isinstance(component, int | float) or not math.isfinite(float(component)):
                return _STATUS_MISSING
            kinematic_components.append(float(component))
        return kinematic_components
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
        if channel.value_space is None:
            findings.append(f"{channel.id}: channel has no declared value space")
        elif channel.availability in {"available", "available_in_batch"} and channel.value_space.topology == "topology_pending":
            findings.append(f"{channel.id}: available channel remains topology_pending")
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
    "project_authority_action_values",
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
    "validate_authority_action_values",
    "validate_interface_channel_value",
    "validate_projected_status_values",
    "validate_vehicle_interface_contract",
]
