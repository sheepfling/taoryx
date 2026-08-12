"""Composition-owned interactive episodes without a second physics kernel.

The vehicle-composition compiler freezes the selected family, fidelity,
initialization, and mission graph.  This module projects that immutable
selection into the already-declared stepping implementation for a small set
of source-owned witnesses.  It is deliberately fail-closed: an episode is
never created by swapping in another family, a generic force model, or an
undeclared effector path.

The initial witnesses are intentionally narrow:

* X8/B747 language-backed racetracks reuse :class:`InteractiveSession`.
* Hummingbird pseudo-6DOF reuses its bounded aggregate-thrust-vector model.

Both expose the same reset/observe/step/checkpoint/close vocabulary at
accepted truth boundaries.  A passing episode says nothing about mission
qualification or physical actuator realization beyond the selected fidelity's
explicit claim boundary.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import tempfile
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, Protocol, cast

from .committed_boundary_sensor import CommittedBoundarySensor
from .direct_wrench import DIRECT_WRENCH_NAMES, DirectWrenchProjection
from .language.grammar_contracts import GrammarProfile
from .language_backed_execution import (
    _kinematic_attitude_from_row,
    _kinematic_body_rate_from_row,
    _mission,
    _mission_tables,
)
from .language_backed_racetrack import materialize_powered_fixed_wing_composition
from .local_direct_wrench_screen_registry import resolve_local_direct_wrench_screen_definition
from .plugins import PluginCatalog, discover_plugins
from .runtime.interactive import InteractiveSession, InteractiveStatus
from .scenario import ResolvedScenario, ScenarioCompiler
from .trajectory.pseudo6dof_profiles import Pseudo6DOFProfile, load_pseudo6dof_catalog
from .value_space import (
    ValueSpaceSpec,
    boolean,
    bounded_interval,
    euclidean,
    finite_set,
    periodic_circle,
    positive_half_line,
    product,
    unit_interval,
)
from .vehicle_composition import CompiledVehicleComposition, resolve_vehicle_composition_interface_contract
from .vehicle_composition_registry import mission_graph_execution_contract
from .vehicle_execution_bindings import VehicleExecutionBindingError, resolve_vehicle_execution_binding
from .vehicle_execution_preflight import compile_powered_fixed_wing_racetrack_from_composition, preflight_vehicle_composition
from .vehicle_interface import (
    ObservationProfile,
    VehicleInterfaceContract,
    project_committed_status_values,
    validate_authority_action_values,
    validate_projected_status_values,
)

if TYPE_CHECKING:
    from .trajectory import (
        A320GuidanceOverride,
        A320RacetrackStepper,
        A320RacetrackStepperState,
        F16GuidanceOverride,
        F16ReducedRacetrackStepper,
        F16ReducedRacetrackStepperState,
        HummingbirdPseudo6DOFCommand,
        HummingbirdPseudo6DOFState,
    )

EpisodeStatus = Literal["ready", "active", "completed", "closed"]


@dataclass(frozen=True, slots=True)
class EpisodeChannel:
    """One user-commandable or observable episode channel."""

    name: str
    unit: str | None
    lower: float | None = None
    upper: float | None = None
    description: str = ""
    value_space: ValueSpaceSpec | None = field(default=None, repr=False)
    data_type: Literal["float64", "int64", "boolean", "string", "json"] | None = None
    shape: tuple[int | Literal["variable"], ...] = ()
    sampling_semantics: Literal[
        "continuous_sample",
        "discrete_sample",
        "event",
        "interval",
        "static",
        "provider_reported",
    ] | None = None

    def __post_init__(self) -> None:
        if not self.name.strip() or not self.description.strip():
            raise ValueError("episode channels require a stable name and description")
        if self.lower is not None and self.upper is not None and self.lower > self.upper:
            raise ValueError(f"episode channel {self.name!r} has inverted bounds")
        if any(item != "variable" and item <= 0 for item in self.shape):
            raise ValueError(f"episode channel {self.name!r} has a non-positive shape dimension")
        if self.value_space is None:
            object.__setattr__(
                self,
                "value_space",
                episode_channel_value_space(self.name, self.unit, self.lower, self.upper),
            )
        ####

    ####

    def as_dict(self) -> dict[str, object]:
        """Return the public channel schema."""

        if self.value_space is None:
            raise ValueError(f"episode channel {self.name!r} is missing a value-space declaration")

        return {
            "name": self.name,
            "unit": self.unit,
            "lower": self.lower,
            "upper": self.upper,
            "description": self.description,
            "value_space": self.value_space.as_dict(),
            "data_type": self.data_type,
            "shape": list(self.shape),
            "sampling_semantics": self.sampling_semantics,
        }
        ####

    ####


def episode_channel_value_space(
    name: str,
    unit: str | None,
    lower: float | None,
    upper: float | None,
) -> ValueSpaceSpec:
    """Return a reviewed topology for a legacy native episode channel.

    The semantic interface remains the authority for AI/RL-facing channels.
    This adapter makes the older native-name episode schema equally explicit
    instead of asking a caller to guess from a unit or identifier. It covers
    the current source-owned episode bindings only; a new native channel must
    extend this table or provide its own ``ValueSpaceSpec`` at construction.
    """

    normalized = name.casefold()
    source_name = normalized.split(".", maxsplit=1)[1] if normalized.partition(".")[0].isdigit() else normalized
    if unit == "boolean" or normalized in {
        "guidance-override-enabled",
        "motors_enabled",
        "contact",
        "wrench_saturated",
        "physical_motor_allocation",
        "physical_effector_allocation",
    }:
        return boolean()
    if normalized in {"response_profile_id", "control_realization", "wrench_status"}:
        return finite_set(representation="scalar declared identifier")
    if normalized in {"yaw_rad", "attitude.yaw.command"} or normalized.endswith(".heading_rad"):
        return periodic_circle(2.0 * math.pi)
    if source_name in {"psi", "psigd", "yawgd", "_command_psi", "long"}:
        return periodic_circle(360.0, representation="scalar source-degree")
    if normalized == "attitude_rad":
        return product(
            bounded_interval(),
            bounded_interval(),
            periodic_circle(2.0 * math.pi),
            representation="vector3 [roll, pitch, yaw]",
        )
    vector_channels = {
        "position_ned_m",
        "velocity_ned_m_s",
        "body_rate_rad_s",
        "force_body_n",
        "moment_body_nm",
        "body_velocity_m_s",
        "requested_force_body_n",
        "requested_moment_body_nm",
        "achieved_force_body_n",
        "achieved_moment_body_nm",
        "residual_force_body_n",
        "residual_moment_body_nm",
    }
    if normalized in vector_channels:
        return euclidean(3)
    if normalized in {"battery_fraction", "thrust_ratio"}:
        return unit_interval()
    if source_name in {"throttle"}:
        return unit_interval()
    if source_name in {
        "collective-elevon-deg",
        "differential-elevon-deg",
        "gama",
        "gamgd",
        "_command_gamgd",
        "pitchgd",
        "alpha",
        "alphat",
        "_aero_alpha_reference_deg",
        "lat",
        "latgd",
    }:
        return bounded_interval(representation="scalar source-angle")
    if source_name in {"_geodetic_state", "_point_mass_si_contract", "_dtprnt", "_segment"}:
        return finite_set(representation="scalar source state code")
    if source_name in {
        "aggregate_thrust_n",
        "time_s",
        "duration_s",
        "speed_m_s",
        "mass_kg",
        "alt",
        "vel",
        "vair",
        "mass",
        "mass_kg",
        "rho",
        "mach",
        "dynprs",
        "ntotal",
        "plength",
        "tseg",
        "tmark",
        "time",
        "temp",
        "pres",
        "sndspd",
        "nu",
        "rotor_speed",
        "_command_vel",
        "power",
        "wt",
        "fuel",
        "rcm",
        "thrust",
        "mdot",
        "range",
    }:
        return positive_half_line()
    if lower is not None or upper is not None:
        return bounded_interval()
    return euclidean()
    ####


@dataclass(frozen=True, slots=True)
class ActionFrame:
    """One semantic external action held across an explicit truth interval.

    The selected authority profile is part of the action identity.  A caller
    cannot accidentally send direct effector coordinates through a
    body-motion profile, and an action replay remains bound to the exact
    resolved interface that accepted it.
    """

    interface_id: str
    interface_fingerprint_sha256: str
    authority_profile_id: str
    values: Mapping[str, object]
    duration_s: float

    def __post_init__(self) -> None:
        if not self.interface_id.strip() or not self.interface_fingerprint_sha256.strip() or not self.authority_profile_id.strip():
            raise ValueError("action frames require interface and authority-profile identity")
        if not math.isfinite(self.duration_s) or self.duration_s <= 0.0:
            raise ValueError("action frame duration_s must be positive and finite")
        ####

    ####

    def as_dict(self) -> dict[str, object]:
        """Return the replayable semantic action payload."""

        return {
            "interface_id": self.interface_id,
            "interface_fingerprint_sha256": self.interface_fingerprint_sha256,
            "authority_profile_id": self.authority_profile_id,
            "values": _json_safe(self.values),
            "duration_s": self.duration_s,
        }
        ####

    ####


@dataclass(frozen=True, slots=True)
class StatusFrame:
    """Canonical committed-truth status with raw values retained separately."""

    interface_id: str
    interface_fingerprint_sha256: str
    time_s: float
    values: Mapping[str, object]
    raw_values: Mapping[str, object]
    status: EpisodeStatus

    def as_dict(self) -> dict[str, object]:
        """Return the normalized status and auditable raw sidecar."""

        return {
            "interface_id": self.interface_id,
            "interface_fingerprint_sha256": self.interface_fingerprint_sha256,
            "time_s": self.time_s,
            "values": _json_safe(self.values),
            "raw_values": _json_safe(self.raw_values),
            "status": self.status,
        }
        ####

    ####


@dataclass(frozen=True, slots=True)
class ObservationFrame:
    """A selected policy/debug observation profile at one committed boundary."""

    interface_id: str
    interface_fingerprint_sha256: str
    observation_profile_id: str
    time_s: float
    values: Mapping[str, object]
    valid: Mapping[str, bool]
    status: EpisodeStatus
    source_time_s: float | None = None

    def as_dict(self) -> dict[str, object]:
        """Return the profile-bound observation payload."""

        return {
            "interface_id": self.interface_id,
            "interface_fingerprint_sha256": self.interface_fingerprint_sha256,
            "observation_profile_id": self.observation_profile_id,
            "time_s": self.time_s,
            "values": _json_safe(self.values),
            "valid": dict(self.valid),
            "status": self.status,
            "source_time_s": self.source_time_s,
        }
        ####

    ####


@dataclass(frozen=True, slots=True)
class EpisodeObservation:
    """One committed truth observation emitted at an episode boundary."""

    time_s: float
    values: Mapping[str, object]
    status: EpisodeStatus

    def as_dict(self) -> dict[str, object]:
        """Return the normalized observation payload."""

        return {"time_s": self.time_s, "values": _json_safe(self.values), "status": self.status}
        ####

    ####


@dataclass(frozen=True, slots=True)
class EpisodeStep:
    """One external action held through one or more accepted inner steps."""

    time_start_s: float
    time_end_s: float
    requested_action: Mapping[str, object]
    applied_action: Mapping[str, object]
    observation: EpisodeObservation
    events: tuple[str, ...] = ()
    diagnostics: tuple[str, ...] = ()
    action_frame: ActionFrame | None = None
    applied_semantic_action: Mapping[str, object] | None = None
    observation_frame: ObservationFrame | None = None
    status_frame: StatusFrame | None = None

    def as_dict(self) -> dict[str, object]:
        """Return an auditable action-to-truth transition record."""

        return {
            "time_start_s": self.time_start_s,
            "time_end_s": self.time_end_s,
            "requested_action": _json_safe(self.requested_action),
            "applied_action": _json_safe(self.applied_action),
            "observation": self.observation.as_dict(),
            "events": list(self.events),
            "diagnostics": list(self.diagnostics),
            "action_frame": None if self.action_frame is None else self.action_frame.as_dict(),
            "applied_semantic_action": None if self.applied_semantic_action is None else _json_safe(self.applied_semantic_action),
            "observation_frame": None if self.observation_frame is None else self.observation_frame.as_dict(),
            "status_frame": None if self.status_frame is None else self.status_frame.as_dict(),
        }
        ####

    ####


class MissionCompositionEpisode(Protocol):
    """Provider-neutral interactive episode accepted by the session API."""

    claim_boundary: str
    interface_contract: VehicleInterfaceContract

    @property
    def action_schema(self) -> tuple[EpisodeChannel, ...]:
        """Return declared external action channels only."""
        ...

    @property
    def observation_schema(self) -> tuple[EpisodeChannel, ...]:
        """Return the stable committed-truth observation channels."""
        ...

    def reset(self, *, seed: int | None = None) -> EpisodeObservation:
        """Restore the declared composition initialization."""
        ...

    def observe(self) -> EpisodeObservation:
        """Return the current committed truth state without interpolation."""
        ...

    def step(self, action: Mapping[str, object], duration_s: float) -> EpisodeStep:
        """Apply one declared action for an explicit external duration."""
        ...

    def step_frame(self, action: ActionFrame) -> EpisodeStep:
        """Apply one semantic action frame through its declared authority profile."""
        ...

    def status_frame(self) -> StatusFrame:
        """Return the canonical status frame at the committed truth boundary."""
        ...

    def observe_frame(self, observation_profile_id: str = "truth_debug") -> ObservationFrame:
        """Return a selected declared observation profile without interpolation."""
        ...

    def save_checkpoint(self, path: str | Path) -> Path:
        """Persist a fingerprinted episode boundary."""
        ...

    def load_checkpoint(self, path: str | Path) -> EpisodeObservation:
        """Restore a fingerprinted episode boundary."""
        ...

    def close(self) -> None:
        """Close provider-owned state without mutating the prepared configuration."""
        ...


class VehicleCompositionEpisode(MissionCompositionEpisode, Protocol):
    """Interactive projection of one immutable canonical vehicle composition."""

    composition: CompiledVehicleComposition


class LanguageBackedCompositionEpisode:
    """Episode wrapper around the existing language-backed interactive kernel."""

    claim_boundary = (
        "This episode reuses the declared language-backed runtime and its accepted truth boundaries. "
        "Its external controls do not promote a pseudo-6DOF response-law or point-mass result into "
        "physical control-surface, moment, or actuator evidence."
    )

    def __init__(self, composition: CompiledVehicleComposition, *, seed: int | None = None) -> None:
        if composition.family_id not in {"skywalker_x8", "b747"}:
            raise ValueError(f"language-backed episode has no adapter for family {composition.family_id!r}")
        preflight = preflight_vehicle_composition(composition)
        if preflight.status != "translation_ready":
            detail = "; ".join(preflight.diagnostics) or "no translation-ready route"
            raise ValueError(f"cannot create episode for {composition.id!r}: {preflight.status}: {detail}")
        self.composition = composition
        self.interface_contract = _interface_contract_for_composition(composition)
        self.preflight = preflight
        self.seed = seed
        self._scenario = _compile_language_backed_scenario(composition, seed=seed)
        self._session = self._scenario.interactive_session()
        self._closed = False
        self._sensor = _sensor_for_contract(self.interface_contract, seed=seed)
        self._advance_sensor()
        ####

    @property
    def action_schema(self) -> tuple[EpisodeChannel, ...]:
        """Expose only runtime-declared external controls."""

        semantic_by_native = {
            str(channel.binding["native_action"]): channel
            for channel in self.interface_contract.action_channels
            if isinstance(channel.binding.get("native_action"), str)
        }
        return tuple(
            EpisodeChannel(
                control.name,
                semantic.canonical_unit if semantic is not None else control.unit,
                semantic.lower if semantic is not None and semantic.value_type == "boolean" else control.lower,
                semantic.upper if semantic is not None and semantic.value_type == "boolean" else control.upper,
                semantic.description if semantic is not None and semantic.value_type == "boolean" else "language-backed runtime control",
                semantic.value_space if semantic is not None and semantic.value_type == "boolean" else None,
            )
            for control in self._session.controls
            for semantic in (semantic_by_native.get(control.name),)
        )
        ####

    @property
    def observation_schema(self) -> tuple[EpisodeChannel, ...]:
        """Expose current runtime state fields as committed truth diagnostics."""

        channels: list[EpisodeChannel] = []
        for vehicle in self._session.problem.vehicles.values():
            names = tuple(dict.fromkeys((*vehicle.state.value_names, *vehicle.state.named)))
            channels.extend(EpisodeChannel(f"{vehicle.name}.{name}", None, description="committed runtime truth") for name in names)
            if self.composition.fidelity == "pseudo_6dof":
                channels.extend(
                    EpisodeChannel(f"{vehicle.name}.{name}", unit, description="profile-backed kinematic sidecar truth")
                    for name, unit in (
                        ("kinematic_roll_deg", "deg"),
                        ("kinematic_pitch_deg", "deg"),
                        ("kinematic_yaw_deg", "deg"),
                        ("kinematic_body_rate_p_rad_s", "rad/s"),
                        ("kinematic_body_rate_q_rad_s", "rad/s"),
                        ("kinematic_body_rate_r_rad_s", "rad/s"),
                    )
                    if name not in names
                )
        return tuple(channels)
        ####

    def reset(self, *, seed: int | None = None) -> EpisodeObservation:
        """Rebuild the immutable composition at a declared deterministic seed.

        A reset may change only the seed used by declared random sources. It
        cannot mutate the compiled family, loadout, initial condition, or
        model tables; those require a newly compiled composition so their
        provenance and interface fingerprint remain explicit.
        """

        self._require_open()
        self.seed = self.seed if seed is None else seed
        self._scenario = _compile_language_backed_scenario(self.composition, seed=self.seed)
        self._session = self._scenario.interactive_session()
        self._sensor = _sensor_for_contract(self.interface_contract, seed=self.seed)
        self._advance_sensor()
        return self.observe()
        ####

    def observe(self) -> EpisodeObservation:
        """Return current accepted runtime truth without a synthetic sample."""

        self._require_open()
        values: dict[str, object] = {}
        for vehicle in self._session.problem.vehicles.values():
            state = {
                **dict(zip(vehicle.state.value_names, vehicle.state.values, strict=False)),
                **dict(vehicle.state.named),
            }
            if self.composition.fidelity == "pseudo_6dof":
                attitude = _kinematic_attitude_from_row(state)
                body_rate = _kinematic_body_rate_from_row(state)
                state.update(
                    {
                        "kinematic_roll_deg": attitude[0],
                        "kinematic_pitch_deg": attitude[1],
                        "kinematic_yaw_deg": attitude[2],
                        "kinematic_body_rate_p_rad_s": body_rate[0],
                        "kinematic_body_rate_q_rad_s": body_rate[1],
                        "kinematic_body_rate_r_rad_s": body_rate[2],
                    }
                )
            values[vehicle.name] = state
        return EpisodeObservation(self._session.time, values, _episode_status(self._session.status, self._closed))
        ####

    def status_frame(self) -> StatusFrame:
        """Return canonical status and raw runtime truth at the current boundary."""

        self._require_open()
        return _status_frame(self.interface_contract, self.observe())
        ####

    def observe_frame(self, observation_profile_id: str = "truth_debug") -> ObservationFrame:
        """Return one explicitly selected observation view without interpolation."""

        self._require_open()
        status = self.status_frame()
        profile = self.interface_contract.observation_profile(observation_profile_id)
        return _episode_observation_frame(status, profile, self._sensor)
        ####

    def step(self, action: Mapping[str, object], duration_s: float) -> EpisodeStep:
        """Hold declared runtime controls across the requested truth interval."""

        self._require_open()
        commands = _numeric_action(action)
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("episode duration_s must be positive and finite")
        target_time = self._session.time + duration_s
        snapshots = []
        while self._session.time < target_time - 1.0e-12:
            boundary = _next_sensor_boundary(self._sensor, self._session.time, target_time)
            interval = (boundary if boundary is not None else target_time) - self._session.time
            snapshots.append(self._session.step(interval, commands))
            self._advance_sensor()
            if self._session.status in {InteractiveStatus.COMPLETED, InteractiveStatus.FAILED}:
                break
        snapshot = snapshots[-1]
        return EpisodeStep(
            snapshots[0].time_start,
            snapshot.time_end,
            dict(action),
            {item.name: item.applied for item in snapshot.commands},
            self.observe(),
            snapshot.events,
            snapshot.diagnostics,
        )
        ####

    def step_frame(self, action: ActionFrame) -> EpisodeStep:
        """Translate a semantic action frame into declared runtime controls."""

        self._require_open()
        step = self.step(_native_action_for_frame(self.interface_contract, action), action.duration_s)
        status = self.status_frame()
        observation = _episode_observation_frame(
            status,
            self.interface_contract.observation_profile(self.composition.observation.profile_id),
            self._sensor,
        )
        return replace(
            step,
            action_frame=action,
            applied_semantic_action=_semantic_action_from_native(self.interface_contract, action, step.applied_action),
            observation_frame=observation,
            status_frame=status,
        )
        ####

    def save_checkpoint(self, path: str | Path) -> Path:
        """Save through the native runtime checkpoint format and fingerprint."""

        self._require_open()
        if self._sensor is None:
            return self._session.save_checkpoint(path, model_fingerprint=self.composition.identity_sha256)
        destination = Path(path)
        native_path = destination.with_name(f".{destination.name}.runtime.tmp")
        self._session.save_checkpoint(native_path, model_fingerprint=self.composition.identity_sha256)
        try:
            payload: dict[str, object] = {
                "schema": "taoryx.composition-episode-sensor-checkpoint/v1alpha1",
                "composition_identity_sha256": self.composition.identity_sha256,
                "interface_fingerprint_sha256": self.interface_contract.fingerprint,
                "runtime_checkpoint": json.loads(native_path.read_text(encoding="utf-8")),
                "declared_sensor": self._sensor.checkpoint_payload(),
            }
        finally:
            native_path.unlink(missing_ok=True)
        payload["integrity"] = _payload_digest(payload)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return destination
        ####

    def load_checkpoint(self, path: str | Path) -> EpisodeObservation:
        """Rebuild callbacks from the source scenario before restoring state."""

        self._require_open()
        source = Path(path)
        payload = json.loads(source.read_text(encoding="utf-8"))
        sensor_payload: Mapping[str, object] | None = None
        restore_path = source
        temporary_path: Path | None = None
        if payload.get("schema") == "taoryx.composition-episode-sensor-checkpoint/v1alpha1":
            integrity = payload.pop("integrity", None)
            if integrity != _payload_digest(payload):
                raise ValueError("composition sensor checkpoint integrity verification failed")
            if payload.get("composition_identity_sha256") != self.composition.identity_sha256:
                raise ValueError("composition sensor checkpoint composition mismatch")
            if payload.get("interface_fingerprint_sha256") != self.interface_contract.fingerprint:
                raise ValueError("composition sensor checkpoint interface mismatch")
            runtime_payload = payload.get("runtime_checkpoint")
            sensor_value = payload.get("declared_sensor")
            if not isinstance(runtime_payload, Mapping) or not isinstance(sensor_value, Mapping) or self._sensor is None:
                raise ValueError("composition sensor checkpoint has no compatible declared sensor state")
            temporary_path = source.with_name(f".{source.name}.runtime.restore.tmp")
            temporary_path.write_text(json.dumps(runtime_payload, sort_keys=True), encoding="utf-8")
            restore_path = temporary_path
            sensor_payload = sensor_value
        template = self._scenario.interactive_session()
        try:
            self._session = InteractiveSession.load_checkpoint(
                restore_path,
                template.problem,
                model_fingerprint=self.composition.identity_sha256,
                controls=template.controls,
                control_model=template.control_model,
                segment_controllers=template.segment_controllers,
                status_specs=template.status_specs,
                event_specs=template.event_specs,
                output_subscriptions=template.output_subscriptions,
            )
        finally:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)
        if sensor_payload is not None:
            sensor = self._sensor
            if sensor is None:
                raise ValueError("composition sensor checkpoint is unavailable for this episode")
            sensor.restore_checkpoint(sensor_payload)
        return self.observe()
        ####

    def close(self) -> None:
        """Interrupt an unfinished native session and make the wrapper inert."""

        if not self._closed and self._session.status not in {InteractiveStatus.COMPLETED, InteractiveStatus.FAILED}:
            self._session.interrupt()
        self._closed = True
        ####

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("vehicle composition episode is closed")
        ####

    def _advance_sensor(self) -> None:
        if self._sensor is not None:
            status = self.status_frame()
            self._sensor.advance(status.time_s, status.values)
        ####

    def _reset_sensor(self) -> None:
        if self._sensor is not None:
            self._sensor.reset()
            self._advance_sensor()
        ####

    ####


class HummingbirdPseudoCompositionEpisode:
    """Episode wrapper around the declared Hummingbird aggregate-thrust model."""

    claim_boundary = (
        "This episode uses the named Hummingbird aggregate-thrust-vector pseudo-6DOF response law. "
        "It exposes bounded attitude, thrust, contact, and battery behavior but does not allocate individual "
        "rotors or establish physical motor, aerodynamic-moment, or actuator evidence."
    )

    _ACTION_SCHEMA = (
        EpisodeChannel("roll_rad", "rad", -math.pi / 2.0, math.pi / 2.0, "commanded roll response-law angle"),
        EpisodeChannel("pitch_rad", "rad", -math.pi / 2.0, math.pi / 2.0, "commanded pitch response-law angle"),
        EpisodeChannel("yaw_rad", "rad", -math.pi, math.pi, "commanded yaw response-law angle"),
        EpisodeChannel("thrust_ratio", "dimensionless", 0.0, 1.0, "aggregate thrust ratio"),
        EpisodeChannel("motors_enabled", "boolean", description="aggregate rotor enable state"),
    )
    _OBSERVATION_SCHEMA = (
        EpisodeChannel("position_ned_m", "m", description="committed NED position"),
        EpisodeChannel("velocity_ned_m_s", "m/s", description="committed NED velocity"),
        EpisodeChannel("attitude_rad", "rad", description="declared response-law attitude"),
        EpisodeChannel("body_rate_rad_s", "rad/s", description="declared response-law body rate"),
        EpisodeChannel("battery_fraction", "dimensionless", 0.0, 1.0, "bounded engineering reserve"),
        EpisodeChannel("aggregate_thrust_n", "N", 0.0, description="achieved aggregate thrust"),
        EpisodeChannel("contact", "boolean", description="ground-contact state"),
        EpisodeChannel("mass_kg", "kg", 0.0, description="fixed pseudo-plant operating mass"),
        EpisodeChannel("response_profile_id", None, description="selected named pseudo-6DOF response profile"),
        EpisodeChannel("control_realization", None, description="declared pseudo-plant control realization"),
        EpisodeChannel("physical_motor_allocation", "boolean", description="individual motor allocation availability"),
    )

    def __init__(self, composition: CompiledVehicleComposition, *, seed: int | None = None, integration_step_s: float = 0.02) -> None:
        from .trajectory.hummingbird_pseudo6dof import HummingbirdPseudo6DOFModel

        if composition.family_id != "hummingbird" or composition.fidelity != "pseudo_6dof":
            raise ValueError("Hummingbird pseudo episode requires the hummingbird pseudo_6dof composition")
        if composition.initialization.id not in {"grounded_idle", "airborne_hover"}:
            raise ValueError("Hummingbird pseudo episode requires grounded_idle or airborne_hover initialization")
        if not math.isfinite(integration_step_s) or integration_step_s <= 0.0:
            raise ValueError("Hummingbird episode integration_step_s must be positive and finite")
        self.composition = composition
        self.interface_contract = _interface_contract_for_composition(composition)
        self.seed = seed
        self.integration_step_s = integration_step_s
        self.model = HummingbirdPseudo6DOFModel(mass_kg=_hummingbird_initial_mass_kg(composition))
        self._closed = False
        self._status: EpisodeStatus = "ready"
        self._state = self._initial_state()
        self._last_action = self._initial_action()
        self._sensor = _sensor_for_contract(self.interface_contract, seed=seed)
        self._advance_sensor()
        ####

    @property
    def action_schema(self) -> tuple[EpisodeChannel, ...]:
        return self._ACTION_SCHEMA
        ####

    @property
    def observation_schema(self) -> tuple[EpisodeChannel, ...]:
        return self._OBSERVATION_SCHEMA
        ####

    def reset(self, *, seed: int | None = None) -> EpisodeObservation:
        """Restore the exact declared hover or ground state.

        The current pseudo plant contains no randomized subsystem.  The seed
        is still retained as episode provenance so later declared
        randomization can extend this boundary without changing its API.
        """

        self._require_open()
        self.seed = self.seed if seed is None else seed
        self._state = self._initial_state()
        self._last_action = self._initial_action()
        self._status = "ready"
        self._reset_sensor()
        return self.observe()
        ####

    def observe(self) -> EpisodeObservation:
        """Return the current committed pseudo-plant truth state."""

        self._require_open()
        position = self._state.position_m
        velocity = self._state.velocity_m_s
        return EpisodeObservation(
            self._state.time_s,
            {
                "position_ned_m": [position[0], position[1], -position[2]],
                "velocity_ned_m_s": [velocity[0], velocity[1], -velocity[2]],
                "attitude_rad": list(self._state.attitude_rad),
                "body_rate_rad_s": list(self._state.attitude_rate_rad_s),
                "battery_fraction": self._state.battery_fraction,
                "mass_kg": self.model.mass_kg,
                "aggregate_thrust_n": self._state.thrust_n,
                "contact": self._state.contact,
                "response_profile_id": self.model.profile_id,
                "control_realization": "aggregate_thrust_vector_surrogate",
                "physical_motor_allocation": False,
            },
            self._status,
        )
        ####

    def status_frame(self) -> StatusFrame:
        """Return canonical status and raw pseudo-plant truth at this boundary."""

        self._require_open()
        return _status_frame(self.interface_contract, self.observe())
        ####

    def observe_frame(self, observation_profile_id: str = "truth_debug") -> ObservationFrame:
        """Return one selected declared observation profile without interpolation."""

        self._require_open()
        status = self.status_frame()
        profile = self.interface_contract.observation_profile(observation_profile_id)
        return _episode_observation_frame(status, profile, self._sensor)
        ####

    def step(self, action: Mapping[str, object], duration_s: float) -> EpisodeStep:
        """Apply one bounded semantic action over one or more fixed substeps."""

        self._require_open()
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("episode duration_s must be positive and finite")
        requested, applied = self._resolve_action(action)
        start = self._state.time_s
        remaining = duration_s
        events: list[str] = []
        contact_before = self._state.contact
        motors_before = bool(self._last_action["motors_enabled"])
        while remaining > 1.0e-12:
            upper_time = self._state.time_s + remaining
            boundary = _next_sensor_boundary(self._sensor, self._state.time_s, upper_time)
            step_s = min(
                self.integration_step_s,
                remaining,
                remaining if boundary is None else max(0.0, boundary - self._state.time_s),
            )
            if step_s <= 1.0e-12:
                self._advance_sensor()
                continue
            self._state, _ = self.model.step(self._state, self._command_from_action(applied), step_s)
            remaining -= step_s
            if not contact_before and self._state.contact:
                events.append("contact")
                contact_before = True
            self._advance_sensor()
        if motors_before and not bool(applied["motors_enabled"]):
            events.append("motor_shutdown")
        self._last_action = applied
        self._status = "active"
        return EpisodeStep(start, self._state.time_s, requested, applied, self.observe(), tuple(events))
        ####

    def step_frame(self, action: ActionFrame) -> EpisodeStep:
        """Translate semantic pseudo-6DOF commands into the declared response law."""

        self._require_open()
        step = self.step(_native_action_for_frame(self.interface_contract, action), action.duration_s)
        status = self.status_frame()
        observation = _episode_observation_frame(
            status,
            self.interface_contract.observation_profile(self.composition.observation.profile_id),
            self._sensor,
        )
        return replace(
            step,
            action_frame=action,
            applied_semantic_action=_semantic_action_from_native(self.interface_contract, action, step.applied_action),
            observation_frame=observation,
            status_frame=status,
        )
        ####

    def save_checkpoint(self, path: str | Path) -> Path:
        """Persist this bounded pseudo state with an integrity fingerprint."""

        self._require_open()
        payload: dict[str, object] = {
            "schema": "taoryx.hummingbird-pseudo-composition-episode/v1alpha1",
            "composition_identity_sha256": self.composition.identity_sha256,
            "seed": self.seed,
            "integration_step_s": self.integration_step_s,
            "status": self._status,
            "state": _hummingbird_state_payload(self._state),
            "last_action": dict(self._last_action),
        }
        if self._sensor is not None:
            payload["declared_sensor"] = self._sensor.checkpoint_payload()
        payload["integrity"] = _payload_digest(payload)
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(f".{destination.name}.tmp")
        temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        os.replace(temporary, destination)
        return destination
        ####

    def load_checkpoint(self, path: str | Path) -> EpisodeObservation:
        """Restore a composition-bound pseudo state without serializing code."""

        self._require_open()
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if payload.get("schema") != "taoryx.hummingbird-pseudo-composition-episode/v1alpha1":
            raise ValueError("unsupported Hummingbird composition episode checkpoint schema")
        integrity = payload.pop("integrity", None)
        if integrity != _payload_digest(payload):
            raise ValueError("Hummingbird composition episode checkpoint integrity verification failed")
        if payload.get("composition_identity_sha256") != self.composition.identity_sha256:
            raise ValueError("Hummingbird composition episode checkpoint composition mismatch")
        self.seed = payload.get("seed") if isinstance(payload.get("seed"), int) else None
        self._state = _hummingbird_state_from_payload(payload["state"])
        self._last_action = _hummingbird_action(payload["last_action"])
        self._status = _episode_status_literal(payload.get("status"))
        sensor_payload = payload.get("declared_sensor")
        if self._sensor is not None:
            if not isinstance(sensor_payload, Mapping):
                raise ValueError("Hummingbird sensor-configured checkpoint has no declared sensor state")
            sensor = self._sensor
            sensor.restore_checkpoint(sensor_payload)
        elif sensor_payload is not None:
            raise ValueError("Hummingbird checkpoint declares a sensor unavailable in this composition")
        return self.observe()
        ####

    def close(self) -> None:
        self._closed = True
        self._status = "closed"
        ####

    def _initial_state(self) -> HummingbirdPseudo6DOFState:
        inputs = self.composition.initialization.inputs
        altitude_m = _composition_number(inputs, "altitude_m", default=0.0)
        north_m = _composition_number(inputs, "north_m", default=_composition_number(inputs, "pad_north_m", default=0.0))
        east_m = _composition_number(inputs, "east_m", default=_composition_number(inputs, "pad_east_m", default=0.0))
        heading_rad = math.radians(_composition_number(inputs, "heading_deg", default=0.0))
        state = self.model.initial_state(altitude_m=altitude_m)
        return replace(
            state,
            position_m=(north_m, east_m, altitude_m),
            attitude_rad=(0.0, 0.0, heading_rad),
            contact=altitude_m == 0.0,
        )
        ####

    def _initial_action(self) -> dict[str, object]:
        return {
            "roll_rad": 0.0,
            "pitch_rad": 0.0,
            "yaw_rad": self._state.attitude_rad[2],
            "thrust_ratio": self.model.mass_kg * self.model.gravity_m_s2 / self.model.maximum_thrust_n,
            "motors_enabled": True,
        }
        ####

    def _resolve_action(self, action: Mapping[str, object]) -> tuple[dict[str, object], dict[str, object]]:
        unknown = sorted(set(action) - {channel.name for channel in self._ACTION_SCHEMA})
        if unknown:
            raise ValueError("unknown Hummingbird episode action(s): " + ", ".join(unknown))
        requested = {**self._last_action, **dict(action)}
        applied = _hummingbird_action(requested)
        return requested, applied
        ####

    def _command_from_action(self, action: Mapping[str, object]) -> HummingbirdPseudo6DOFCommand:
        from .trajectory.hummingbird_pseudo6dof import HummingbirdPseudo6DOFCommand

        return HummingbirdPseudo6DOFCommand(
            roll_rad=_finite_number(action["roll_rad"], "roll_rad"),
            pitch_rad=_finite_number(action["pitch_rad"], "pitch_rad"),
            yaw_rad=_finite_number(action["yaw_rad"], "yaw_rad"),
            thrust_ratio=_finite_number(action["thrust_ratio"], "thrust_ratio"),
            motors_enabled=bool(action["motors_enabled"]),
            thrust_frame="body_euler",
        )
        ####

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("vehicle composition episode is closed")
        ####

    def _advance_sensor(self) -> None:
        if self._sensor is not None:
            status = self.status_frame()
            self._sensor.advance(status.time_s, status.values)
        ####

    def _reset_sensor(self) -> None:
        if self._sensor is not None:
            self._sensor = _sensor_for_contract(self.interface_contract, seed=self.seed)
            self._advance_sensor()
        ####

    ####


def _reduced_fixed_wing_high_order_episode_channels(maximum_speed_m_s: float) -> tuple[EpisodeChannel, ...]:
    """Return native adapter coordinates shared by reduced fixed-wing profiles."""

    return (
        EpisodeChannel("pilot-throttle-fraction", "dimensionless", 0.0, 1.0, "held normalized reduced energy request"),
        EpisodeChannel("pilot-longitudinal-normalized", "dimensionless", -1.0, 1.0, "held normalized reduced longitudinal request"),
        EpisodeChannel("pilot-lateral-normalized", "dimensionless", -1.0, 1.0, "held normalized reduced roll-to-turn request"),
        EpisodeChannel("waypoint-north-m", "m", -1_000_000.0, 1_000_000.0, "held live-waypoint north coordinate"),
        EpisodeChannel("waypoint-east-m", "m", -1_000_000.0, 1_000_000.0, "held live-waypoint east coordinate"),
        EpisodeChannel("waypoint-altitude-m", "m", -1_000.0, 100_000.0, "held live-waypoint altitude coordinate"),
        EpisodeChannel("waypoint-capture-radius-m", "m", 1.0, 100_000.0, "held live-waypoint capture radius"),
        EpisodeChannel("waypoint-speed-mps", "m/s", 50.0, maximum_speed_m_s, "held live-waypoint speed request"),
    )
    ####


def _f16_reduced_body_rate_episode_channels() -> tuple[EpisodeChannel, ...]:
    """Return the F-16 pseudo-6DOF rate-reference adapter coordinates."""

    return (
        EpisodeChannel("body-roll-rate-command-rad-s", "rad/s", -0.5, 0.5, "held body roll-rate reference p"),
        EpisodeChannel("body-pitch-rate-command-rad-s", "rad/s", -0.35, 0.35, "held body pitch-rate reference q"),
        EpisodeChannel("body-yaw-rate-command-rad-s", "rad/s", -0.35, 0.35, "held body yaw-rate reference r"),
    )
    ####


class ReducedFixedWingCompositionEpisode:
    """Reusable accepted-truth contract for source-owned reduced fixed wing.

    A family subclass supplies only source-trim initialization, native
    guidance lowering, and checkpoint-state shape. The public action,
    status, observation, committed-truth, and checkpoint semantics remain
    identical across the reduced fixed-wing family members. It does not
    expose a surface, actuator, or moment command and makes no such claim.
    """

    claim_boundary = (
        "This episode uses the declared source-owned A320 reduced-flight stepper. It accepts bounded kinematic "
        "guidance through the point-mass or named route-lag response law; it does not establish physical surface "
        "allocation, actuator dynamics, moment balance, batch/episode parity, or mission qualification."
    )

    _ACTION_SCHEMA = (
        EpisodeChannel("speed_m_s", "m/s", 50.0, 300.0, "held reduced-model speed target"),
        EpisodeChannel("flight_path_angle_deg", "deg", -20.0, 20.0, "held reduced-model flight-path target"),
        EpisodeChannel("heading_deg", "deg", 0.0, 360.0, "held local-navigation heading target"),
        EpisodeChannel("bank_angle_deg", "deg", -60.0, 60.0, "held response-law bank or lift-vector target"),
    )

    _FAMILY_ID = "a320_openap_3dof"
    _FAMILY_LABEL = "A320"
    _CHECKPOINT_SCHEMA = "taoryx.a320-reduced-composition-episode/v1alpha1"

    def __init__(self, composition: CompiledVehicleComposition, *, seed: int | None = None) -> None:
        if composition.family_id != self._FAMILY_ID or composition.fidelity not in {
            "point_mass_3dof",
            "pseudo_6dof",
        }:
            raise ValueError(f"{self._FAMILY_LABEL} reduced episode requires a 3DOF or pseudo-6DOF composition")
        preflight = preflight_vehicle_composition(composition)
        if preflight.status != "translation_ready":
            details = "; ".join(preflight.diagnostics) or "no translation-ready route"
            raise ValueError(f"cannot create A320 episode for {composition.id!r}: {preflight.status}: {details}")
        self.composition = composition
        self.interface_contract = _interface_contract_for_composition(composition)
        self._authority_action_schema_cache: dict[str, tuple[EpisodeChannel, ...]] = {}
        self.seed = seed
        self.preflight = preflight
        self._stepper = self._build_stepper(composition)
        self._held_native_action: dict[str, float] = {}
        self._active_authority_profile_id: str | None = None
        self._control_lowering_state: dict[str, object] = {}
        self._closed = False
        self._status: EpisodeStatus = "ready"
        ####

    @property
    def action_schema(self) -> tuple[EpisodeChannel, ...]:
        return self._ACTION_SCHEMA
        ####

    def authority_action_schema(self, authority_profile_id: str) -> tuple[EpisodeChannel, ...]:
        """Return the adapter coordinates accepted by one semantic profile.

        The legacy episode action schema remains the source-native kinematic
        seam. Higher-order adapter coordinates are visible only after an
        authority profile is selected through the semantic session contract.
        """

        cached = self._authority_action_schema_cache.get(authority_profile_id)
        if cached is not None:
            return cached
        profile = self.interface_contract.authority_profile(authority_profile_id)
        maximum_speed_m_s = 300.0 if self._FAMILY_ID == "a320_openap_3dof" else 500.0
        candidates = {
            channel.name: channel
            for channel in (
                *self.action_schema,
                *_reduced_fixed_wing_high_order_episode_channels(maximum_speed_m_s),
                *(
                    _f16_reduced_body_rate_episode_channels()
                    if self._FAMILY_ID == "f16_s119" and self.composition.fidelity == "pseudo_6dof"
                    else ()
                ),
            )
        }
        semantic_channels = {channel.id: channel for channel in self.interface_contract.action_channels}
        resolved: list[EpisodeChannel] = []
        for identifier in profile.action_ids:
            native = semantic_channels[identifier].binding.get("native_action")
            if not isinstance(native, str) or native not in candidates:
                raise ValueError(
                    f"authority profile {authority_profile_id!r} has no reduced-episode adapter for {identifier!r}"
                )
            resolved.append(candidates[native])
        schema = tuple(resolved)
        self._authority_action_schema_cache[authority_profile_id] = schema
        return schema
        ####

    @property
    def observation_schema(self) -> tuple[EpisodeChannel, ...]:
        channels = (
            *self.interface_contract.status_channels,
            *self.interface_contract.resource_channels,
            *self.interface_contract.diagnostic_channels,
        )
        return tuple(
            EpisodeChannel(channel.id, channel.canonical_unit, channel.lower, channel.upper, channel.description, channel.value_space)
            for channel in channels
            if channel.availability == "available"
        )
        ####

    def reset(self, *, seed: int | None = None) -> EpisodeObservation:
        """Restore the immutable trim and composition initialization."""

        self._require_open()
        self.seed = self.seed if seed is None else seed
        self._stepper.reset()
        self._held_native_action = {}
        self._control_lowering_state = {}
        self._status = "ready"
        return self.observe()
        ####

    def observe(self) -> EpisodeObservation:
        """Return current committed reduced-plant truth without interpolation."""

        self._require_open()
        row = self._stepper.current_row(self._control_override(self._held_native_action))
        values = self._status_values(row)
        values["control_authority_profile_id"] = self._active_authority_profile_id or "legacy_native_union"
        values["control_lowering"] = dict(self._control_lowering_state)
        return EpisodeObservation(self._stepper.state.time_s, values, self._status)
        ####

    def status_frame(self) -> StatusFrame:
        """Project the committed reduced state through the canonical interface."""

        self._require_open()
        return _status_frame(self.interface_contract, self.observe())
        ####

    def observe_frame(self, observation_profile_id: str = "truth_debug") -> ObservationFrame:
        """Return a selected committed-truth observation profile."""

        self._require_open()
        status = self.status_frame()
        return _observation_frame(status, self.interface_contract.observation_profile(observation_profile_id))
        ####

    def step(self, action: Mapping[str, object], duration_s: float) -> EpisodeStep:
        """Hold one native reduced-guidance request through integrated truth steps."""

        self._require_open()
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("A320 episode duration_s must be positive and finite")
        requested, applied = self._resolve_action(action)
        start = self._stepper.state.time_s
        rows = self._stepper.step(duration_s, self._control_override(applied, duration_s=duration_s))
        self._held_native_action = applied
        events: tuple[str, ...]
        if not self._stepper.state.numerical_valid:
            self._status = "completed"
            events = ("numerical_failure",)
        elif self._stepper.completed:
            self._status = "completed"
            events = ("horizon_reached",)
        else:
            self._status = "active"
            events = ()
        diagnostics = () if rows else ("no_advance_after_terminal_boundary",)
        return EpisodeStep(start, self._stepper.state.time_s, requested, applied, self.observe(), events, diagnostics)
        ####

    def step_frame(self, action: ActionFrame) -> EpisodeStep:
        """Map one selected high- or low-order profile into reduced native coordinates."""

        self._require_open()
        previous_profile_id = self._active_authority_profile_id
        self.select_authority_profile(action.authority_profile_id)
        step = self.step(_native_action_for_frame(self.interface_contract, action), action.duration_s)
        transition_event = (
            ()
            if previous_profile_id in {None, action.authority_profile_id}
            else (f"authority_profile_changed:{previous_profile_id}->{action.authority_profile_id}",)
        )
        status = self.status_frame()
        observation = _observation_frame(status, self.interface_contract.observation_profile(self.composition.observation.profile_id))
        return replace(
            step,
            events=(*step.events, *transition_event),
            action_frame=action,
            applied_semantic_action=_semantic_action_from_native(self.interface_contract, action, step.applied_action),
            observation_frame=observation,
            status_frame=status,
        )
        ####

    def select_authority_profile(self, authority_profile_id: str) -> None:
        """Select or explicitly transfer between state-continuous reduced profiles."""

        profile = self.interface_contract.authority_profile(authority_profile_id)
        if profile.availability != "available":
            raise ValueError(f"authority profile {authority_profile_id!r} is {profile.availability}, not executable")
        previous_id = self._active_authority_profile_id
        if previous_id == authority_profile_id:
            return
        if previous_id is not None:
            previous = self.interface_contract.authority_profile(previous_id)
            if previous.switching_policy != "explicit_bumpless" or profile.switching_policy != "explicit_bumpless":
                raise ValueError(f"authority transfer {previous_id!r} -> {authority_profile_id!r} is not declared")
        self._held_native_action = {}
        self._control_lowering_state = {
            "active_profile_id": authority_profile_id,
            "lowering_chain": list(profile.lowering_chain),
            "transfer": "initial_selection" if previous_id is None else "state_continuous_reference_handoff",
        }
        self._active_authority_profile_id = authority_profile_id
        ####

    def save_checkpoint(self, path: str | Path) -> Path:
        """Persist a composition-bound A320 stepper state without serializing code."""

        self._require_open()
        snapshot = self._stepper.state
        payload: dict[str, object] = {
            "schema": self._CHECKPOINT_SCHEMA,
            "composition_identity_sha256": self.composition.identity_sha256,
            "interface_fingerprint_sha256": self.interface_contract.fingerprint,
            "seed": self.seed,
            "status": self._status,
            "held_native_action": dict(self._held_native_action),
            "active_authority_profile_id": self._active_authority_profile_id,
            "control_lowering_state": dict(self._control_lowering_state),
            "stepper_state": self._serialize_stepper_state(snapshot),
        }
        payload["integrity"] = _payload_digest(payload)
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(f".{destination.name}.tmp")
        temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        os.replace(temporary, destination)
        return destination
        ####

    def load_checkpoint(self, path: str | Path) -> EpisodeObservation:
        """Restore an integrity-checked composition-bound A320 stepper state."""

        self._require_open()
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if payload.get("schema") != self._CHECKPOINT_SCHEMA:
            raise ValueError(f"unsupported {self._FAMILY_LABEL} composition episode checkpoint schema")
        integrity = payload.pop("integrity", None)
        if integrity != _payload_digest(payload):
            raise ValueError(f"{self._FAMILY_LABEL} composition episode checkpoint integrity verification failed")
        if payload.get("composition_identity_sha256") != self.composition.identity_sha256:
            raise ValueError(f"{self._FAMILY_LABEL} composition episode checkpoint composition mismatch")
        if payload.get("interface_fingerprint_sha256") != self.interface_contract.fingerprint:
            raise ValueError(f"{self._FAMILY_LABEL} composition episode checkpoint interface mismatch")
        stepper_state = payload.get("stepper_state")
        if not isinstance(stepper_state, Mapping):
            raise ValueError(f"{self._FAMILY_LABEL} checkpoint has no stepper state")
        self._stepper.restore(self._deserialize_stepper_state(stepper_state))
        held = payload.get("held_native_action")
        if not isinstance(held, Mapping):
            raise ValueError(f"{self._FAMILY_LABEL} checkpoint has no held native action")
        active_authority_profile_id = payload.get("active_authority_profile_id")
        if active_authority_profile_id is not None and not isinstance(active_authority_profile_id, str):
            raise ValueError(f"{self._FAMILY_LABEL} checkpoint has invalid authority-profile identity")
        if active_authority_profile_id is not None:
            profile = self.interface_contract.authority_profile(active_authority_profile_id)
            if profile.availability != "available":
                raise ValueError(f"{self._FAMILY_LABEL} checkpoint authority profile is unavailable")
        lowering_state = payload.get("control_lowering_state", {})
        if not isinstance(lowering_state, Mapping):
            raise ValueError(f"{self._FAMILY_LABEL} checkpoint has invalid control-lowering state")
        action_schema = (
            self.action_schema
            if active_authority_profile_id is None
            else self.authority_action_schema(active_authority_profile_id)
        )
        _, held_native_action = self._resolve_action_against_schema(held, action_schema, {})
        self._held_native_action = held_native_action
        self._active_authority_profile_id = active_authority_profile_id
        self._control_lowering_state = dict(lowering_state)
        self.seed = payload.get("seed") if isinstance(payload.get("seed"), int) else None
        self._status = _episode_status_literal(payload.get("status"))
        return self.observe()
        ####

    def close(self) -> None:
        self._closed = True
        self._status = "closed"
        ####

    def _resolve_action(self, action: Mapping[str, object]) -> tuple[dict[str, float], dict[str, float]]:
        action_schema = (
            self.action_schema
            if self._active_authority_profile_id is None
            else self.authority_action_schema(self._active_authority_profile_id)
        )
        return self._resolve_action_against_schema(action, action_schema, self._held_native_action)
        ####

    def _resolve_action_against_schema(
        self,
        action: Mapping[str, object],
        action_schema: tuple[EpisodeChannel, ...],
        held_action: Mapping[str, float],
    ) -> tuple[dict[str, float], dict[str, float]]:
        unknown = sorted(set(action) - {channel.name for channel in action_schema})
        if unknown:
            raise ValueError("unknown A320 reduced episode action(s): " + ", ".join(unknown))
        requested = {**held_action, **dict(action)}
        applied: dict[str, float] = {}
        for channel in action_schema:
            value = requested.get(channel.name)
            if value is None:
                continue
            numeric = _finite_number(value, channel.name)
            if channel.lower is not None and numeric < channel.lower or channel.upper is not None and numeric > channel.upper:
                raise ValueError(f"{self._FAMILY_LABEL} action {channel.name!r} is outside its declared bounds")
            applied[channel.name] = numeric
        return {name: _finite_number(value, name) for name, value in requested.items()}, applied
        ####

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("vehicle composition episode is closed")
        ####

    def _build_stepper(self, composition: CompiledVehicleComposition) -> Any:
        return _a320_stepper_for_composition(composition)
        ####

    @staticmethod
    def _guidance_override(action: Mapping[str, float]) -> Any:
        return _a320_guidance_override(action)
        ####

    def _control_override(self, action: Mapping[str, float], *, duration_s: float | None = None) -> Any:
        """Lower the selected semantic adapter into the existing guidance seam."""

        profile_id = self._active_authority_profile_id
        row = self._stepper.current_row()
        current_north_m = _finite_number(row.get("north_m"), "reduced current north_m")
        current_east_m = _finite_number(row.get("east_m"), "reduced current east_m")
        current_altitude_m = _finite_number(row.get("altitude_m"), "reduced current altitude_m")
        current_speed_m_s = _finite_number(row.get("speed_m_s"), "reduced current speed_m_s")
        current_heading_rad = _finite_number(row.get("heading_rad"), "reduced current heading_rad")
        if profile_id == "live_waypoint_guidance":
            retained = self._control_lowering_state.get("lowered_guidance")
            if duration_s is None and isinstance(retained, Mapping):
                return self._guidance_override(retained)
            required = ("waypoint-north-m", "waypoint-east-m", "waypoint-altitude-m")
            missing = tuple(name for name in required if name not in action)
            if missing:
                if not any(name.startswith("waypoint-") for name in action):
                    self._control_lowering_state.update(
                        {
                            "active_profile_id": profile_id,
                            "waypoint_status": "awaiting_initial_target",
                            "lowered_guidance": {},
                        }
                    )
                    return self._guidance_override({})
                raise ValueError("live waypoint guidance requires an initial north/east/altitude target; missing " + ", ".join(missing))
            north_error_m = action["waypoint-north-m"] - current_north_m
            east_error_m = action["waypoint-east-m"] - current_east_m
            altitude_error_m = action["waypoint-altitude-m"] - current_altitude_m
            horizontal_range_m = math.hypot(north_error_m, east_error_m)
            range_m = math.hypot(horizontal_range_m, altitude_error_m)
            capture_radius_m = action.get("waypoint-capture-radius-m", 100.0)
            captured = range_m <= capture_radius_m
            heading_rad = current_heading_rad if horizontal_range_m <= 1.0e-9 else math.atan2(east_error_m, north_error_m)
            flight_path_angle_deg = 0.0 if captured else math.degrees(math.atan2(altitude_error_m, max(horizontal_range_m, 1.0e-9)))
            flight_path_angle_deg = min(20.0, max(-20.0, flight_path_angle_deg))
            heading_error_rad = (heading_rad - current_heading_rad + math.pi) % (2.0 * math.pi) - math.pi
            bank_angle_deg = 0.0 if captured else min(60.0, max(-60.0, math.degrees(heading_error_rad)))
            waypoint_guidance = {
                "speed_m_s": current_speed_m_s if captured else action.get("waypoint-speed-mps", current_speed_m_s),
                "flight_path_angle_deg": flight_path_angle_deg,
                "heading_deg": math.degrees(heading_rad) % 360.0,
                "bank_angle_deg": bank_angle_deg,
            }
            self._control_lowering_state.update(
                {
                    "active_profile_id": profile_id,
                    "waypoint_range_m": range_m,
                    "waypoint_captured": captured,
                    "lowered_guidance": dict(waypoint_guidance),
                }
            )
            return self._guidance_override(waypoint_guidance)
        if profile_id == "reduced_pilot_command":
            retained = self._control_lowering_state.get("lowered_guidance")
            if duration_s is None and isinstance(retained, Mapping):
                return self._guidance_override(retained)
            pilot_guidance: dict[str, float] = {}
            throttle = action.get("pilot-throttle-fraction")
            if throttle is not None:
                maximum_speed_m_s = 300.0 if self._FAMILY_ID == "a320_openap_3dof" else 500.0
                pilot_guidance["speed_m_s"] = 50.0 + throttle * (maximum_speed_m_s - 50.0)
            longitudinal = action.get("pilot-longitudinal-normalized")
            if longitudinal is not None:
                pilot_guidance["flight_path_angle_deg"] = 20.0 * longitudinal
            lateral = action.get("pilot-lateral-normalized")
            if lateral is not None:
                pilot_guidance["heading_deg"] = (math.degrees(current_heading_rad) + 45.0 * lateral) % 360.0
                pilot_guidance["bank_angle_deg"] = 60.0 * lateral
            self._control_lowering_state.update(
                {
                    "active_profile_id": profile_id,
                    "lowered_guidance": dict(pilot_guidance),
                }
            )
            return self._guidance_override(pilot_guidance)
        if profile_id == "body_rate_command":
            retained = self._control_lowering_state.get("lowered_guidance")
            if duration_s is None and isinstance(retained, Mapping):
                return self._guidance_override(retained)
            rate_guidance: dict[str, float] = {}
            hold_duration_s = 0.0 if duration_s is None else duration_s
            throttle = action.get("pilot-throttle-fraction")
            if throttle is not None:
                rate_guidance["speed_m_s"] = 50.0 + throttle * 450.0
            roll_rate = action.get("body-roll-rate-command-rad-s")
            if roll_rate is not None:
                current_bank_deg = _finite_number(row.get("route_bank_achieved_deg"), "reduced current bank")
                rate_guidance["bank_angle_deg"] = min(
                    60.0,
                    max(-60.0, current_bank_deg + math.degrees(roll_rate * hold_duration_s)),
                )
            pitch_rate = action.get("body-pitch-rate-command-rad-s")
            if pitch_rate is not None:
                current_path_deg = math.degrees(_finite_number(row.get("flight_path_angle_rad"), "reduced current flight path"))
                rate_guidance["flight_path_angle_deg"] = min(
                    20.0,
                    max(-20.0, current_path_deg + math.degrees(pitch_rate * hold_duration_s)),
                )
            yaw_rate = action.get("body-yaw-rate-command-rad-s")
            if yaw_rate is not None:
                rate_guidance["heading_deg"] = (
                    math.degrees(current_heading_rad) + math.degrees(yaw_rate * hold_duration_s)
                ) % 360.0
            self._control_lowering_state.update(
                {
                    "active_profile_id": profile_id,
                    "rate_reference_frame": "body",
                    "rate_reference_coupling": "pseudo_6dof_roll_yaw_coupled",
                    "reference_hold_duration_s": hold_duration_s,
                    "lowered_guidance": dict(rate_guidance),
                }
            )
            return self._guidance_override(rate_guidance)
        self._control_lowering_state.update(
            {
                "active_profile_id": profile_id or "legacy_native_union",
                "lowered_guidance": {
                    name: value
                    for name, value in action.items()
                    if name in {"speed_m_s", "flight_path_angle_deg", "heading_deg", "bank_angle_deg"}
                },
            }
        )
        return self._guidance_override(action)
        ####

    @staticmethod
    def _status_values(row: Mapping[str, object]) -> dict[str, object]:
        return _a320_status_values(row)
        ####

    @staticmethod
    def _serialize_stepper_state(snapshot: Any) -> dict[str, object]:
        return {
            "time_s": snapshot.time_s,
            "north_m": snapshot.north_m,
            "east_m": snapshot.east_m,
            "state": dict(snapshot.state),
            "controls": dict(snapshot.controls),
            "terminal_hold": snapshot.terminal_hold,
            "numerical_valid": snapshot.numerical_valid,
            "failure": snapshot.failure,
        }
        ####

    @staticmethod
    def _deserialize_stepper_state(payload: Mapping[str, object]) -> Any:
        return _a320_stepper_state(payload)
        ####

    ####


class A320ReducedCompositionEpisode(ReducedFixedWingCompositionEpisode):
    """A320/OpenAP binding of the shared reduced fixed-wing episode contract."""

    pass
    ####


class F16ReducedCompositionEpisode(ReducedFixedWingCompositionEpisode):
    """Accepted-truth F-16 episode using the common reduced-flight contract.

    It shares the same held kinematic-guidance action and canonical status
    projection as the A320 adapter, while retaining the F-16 source trim,
    source-load diagnostics, and named attitude-response bridge.  Its sampled
    source controls remain diagnostics only: they are not allocated surfaces.
    """

    claim_boundary = (
        "This episode uses the declared source-owned F-16 reduced-flight stepper. It accepts bounded kinematic "
        "guidance through the point-mass or named attitude/rate response law; it does not establish physical "
        "surface allocation, actuator dynamics, moment balance, batch/episode parity, or mission qualification."
    )

    _ACTION_SCHEMA = (
        EpisodeChannel("speed_m_s", "m/s", 50.0, 500.0, "held reduced-model speed target"),
        EpisodeChannel("flight_path_angle_deg", "deg", -20.0, 20.0, "held reduced-model flight-path target"),
        EpisodeChannel("heading_deg", "deg", 0.0, 360.0, "held local-navigation heading target"),
        EpisodeChannel("bank_angle_deg", "deg", -60.0, 60.0, "held response-law bank or lift-vector target"),
    )

    _FAMILY_ID = "f16_s119"
    _FAMILY_LABEL = "F-16"
    _CHECKPOINT_SCHEMA = "taoryx.f16-reduced-composition-episode/v1alpha1"

    def _build_stepper(self, composition: CompiledVehicleComposition) -> F16ReducedRacetrackStepper:
        return _f16_stepper_for_composition(composition)
        ####

    @staticmethod
    def _guidance_override(action: Mapping[str, float]) -> F16GuidanceOverride:
        return _f16_guidance_override(action)
        ####

    @staticmethod
    def _serialize_stepper_state(snapshot: F16ReducedRacetrackStepperState) -> dict[str, object]:
        return {
            "time_s": snapshot.time_s,
            "north_m": snapshot.north_m,
            "east_m": snapshot.east_m,
            "altitude_m": snapshot.altitude_m,
            "speed_m_s": snapshot.speed_m_s,
            "heading_rad": snapshot.heading_rad,
            "flight_path_angle_rad": snapshot.flight_path_angle_rad,
            "roll_rad": snapshot.roll_rad,
            "pitch_rad": snapshot.pitch_rad,
            "yaw_rad": snapshot.yaw_rad,
            "p_rad_s": snapshot.p_rad_s,
            "q_rad_s": snapshot.q_rad_s,
            "r_rad_s": snapshot.r_rad_s,
            "controls": dict(snapshot.controls),
            "numerical_valid": snapshot.numerical_valid,
            "failure": snapshot.failure,
        }
        ####

    @staticmethod
    def _deserialize_stepper_state(payload: Mapping[str, object]) -> F16ReducedRacetrackStepperState:
        return _f16_stepper_state(payload)
        ####

    ####


class LocalDirectWrenchCompositionEpisode:
    """Bounded source-local direct-wrench bridge episode.

    This is intentionally a local controller-screen episode.  It exposes the
    same requested-to-achieved direct-wrench bridge that the public batch
    screen uses, but it neither grows into a route trajectory nor rebrands
    injected body loads as physical effectors.
    """

    claim_boundary = (
        "This episode applies an explicit bounded direct wrench to one pinned local source-load state. "
        "It is bridge/screen evidence only: not physical effector allocation, source physical trim, or a flight mission."
    )

    _ACTION_SCHEMA = (
        EpisodeChannel("force_body_n", "N", description="requested total body-frame direct force [Fx, Fy, Fz]"),
        EpisodeChannel("moment_body_nm", "N*m", description="requested total body-frame direct moment [Mx, My, Mz]"),
    )
    _OBSERVATION_SCHEMA = (
        EpisodeChannel("body_velocity_m_s", "m/s", description="committed source-local body velocity [u, v, w]"),
        EpisodeChannel("body_rate_rad_s", "rad/s", description="committed source-local body rate [p, q, r]"),
        EpisodeChannel("requested_force_body_n", "N", description="requested direct body force"),
        EpisodeChannel("requested_moment_body_nm", "N*m", description="requested direct body moment"),
        EpisodeChannel("achieved_force_body_n", "N", description="projected direct body force actually applied"),
        EpisodeChannel("achieved_moment_body_nm", "N*m", description="projected direct body moment actually applied"),
        EpisodeChannel("residual_force_body_n", "N", description="unachieved direct body-force request"),
        EpisodeChannel("residual_moment_body_nm", "N*m", description="unachieved direct body-moment request"),
        EpisodeChannel("wrench_residual_norm", None, description="mixed requested-to-achieved wrench residual norm"),
        EpisodeChannel("feedback_norm", None, description="normalized local-screen state-feedback error norm"),
        EpisodeChannel("mass_kg", "kg", 0.0, description="fixed source-local mass declared by the selected screen"),
        EpisodeChannel("wrench_saturated", "boolean", description="direct-wrench authority or slew projection status"),
        EpisodeChannel("wrench_status", None, description="direct-wrench projection disposition"),
        EpisodeChannel("control_realization", None, description="declared direct-wrench bridge realization"),
        EpisodeChannel("physical_effector_allocation", "boolean", description="physical-effector allocation availability"),
    )

    def __init__(self, composition: CompiledVehicleComposition, *, seed: int | None = None) -> None:
        definition = resolve_local_direct_wrench_screen_definition(composition)
        if definition is None:
            raise ValueError("direct-wrench episode requires a declared local screen composition")
        preflight = preflight_vehicle_composition(composition)
        if preflight.status != "translation_ready":
            detail = "; ".join(preflight.diagnostics) or "no source-local direct-wrench translator"
            raise ValueError(f"cannot create {definition.family_id} direct-wrench episode for {composition.id!r}: {preflight.status}: {detail}")
        self.composition = composition
        self.interface_contract = _interface_contract_for_composition(composition)
        self.preflight = preflight
        self.seed = seed
        self.definition = definition
        self.config = definition.config_factory()
        self._closed = False
        self._status: EpisodeStatus = "ready"
        self._time_s = 0.0
        self._state = dict(self.config.initial_state)
        self._last_requested = self._initial_requested_wrench()
        self._projection = self.config.limits.project(
            self._last_requested,
            {name: 0.0 for name in DIRECT_WRENCH_NAMES},
            self.config.dt_s,
        )
        ####

    @property
    def action_schema(self) -> tuple[EpisodeChannel, ...]:
        return self._ACTION_SCHEMA
        ####

    @property
    def observation_schema(self) -> tuple[EpisodeChannel, ...]:
        return self._OBSERVATION_SCHEMA
        ####

    def reset(self, *, seed: int | None = None) -> EpisodeObservation:
        """Restore the pinned local source state and direct-load bridge bias."""

        self._require_open()
        self.seed = self.seed if seed is None else seed
        self._time_s = 0.0
        self._state = dict(self.config.initial_state)
        self._last_requested = self._initial_requested_wrench()
        self._projection = self.config.limits.project(
            self._last_requested,
            {name: 0.0 for name in DIRECT_WRENCH_NAMES},
            self.config.dt_s,
        )
        self._status = "ready"
        return self.observe()
        ####

    def observe(self) -> EpisodeObservation:
        """Return source-local truth and the most recent projected wrench."""

        self._require_open()
        return EpisodeObservation(self._time_s, self._raw_values(), self._status)
        ####

    def status_frame(self) -> StatusFrame:
        """Project canonical source-local bridge status at the committed boundary."""

        self._require_open()
        return _status_frame(self.interface_contract, self.observe())
        ####

    def observe_frame(self, observation_profile_id: str = "truth_debug") -> ObservationFrame:
        """Return the selected committed truth debug view; no sensor is fabricated."""

        self._require_open()
        return _observation_frame(self.status_frame(), self.interface_contract.observation_profile(observation_profile_id))
        ####

    def step(self, action: Mapping[str, object], duration_s: float) -> EpisodeStep:
        """Hold a bounded total direct wrench over source-local fixed substeps."""

        self._require_open()
        if self._status == "completed":
            raise RuntimeError("local direct-wrench screen has completed; reset before stepping again")
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("episode duration_s must be positive and finite")
        requested = self._resolve_action(action)
        start = self._time_s
        limit = self.config.duration_s
        remaining = min(duration_s, max(0.0, limit - self._time_s))
        while remaining > 1.0e-12:
            dt_s = min(self.config.dt_s, remaining)
            projection = self.config.limits.project(requested, self._projection.achieved, dt_s)
            derivative = self.config.source_derivative(self._state, projection.achieved)
            self._state = {
                name: self._state[name] + dt_s * _finite_number(derivative.get(name), f"local direct-wrench derivative {name}")
                for name in self.config.state_names
            }
            if any(not math.isfinite(value) for value in self._state.values()):
                raise RuntimeError("local direct-wrench episode produced a nonfinite state")
            self._time_s += dt_s
            remaining -= dt_s
            self._projection = projection
        self._last_requested = requested
        completed = self._time_s >= limit - 1.0e-12
        self._status = "completed" if completed else "active"
        events = ("local_screen_duration_complete",) if completed else ()
        return EpisodeStep(
            start,
            self._time_s,
            {
                "force_body_n": _force_vector(requested),
                "moment_body_nm": _moment_vector(requested),
            },
            {
                "force_body_n": _force_vector(self._projection.achieved),
                "moment_body_nm": _moment_vector(self._projection.achieved),
            },
            self.observe(),
            events,
        )
        ####

    def step_frame(self, action: ActionFrame) -> EpisodeStep:
        """Apply an interface-bound direct-wrench frame without native fallback."""

        self._require_open()
        step = self.step(_native_action_for_frame(self.interface_contract, action), action.duration_s)
        status = self.status_frame()
        return replace(
            step,
            action_frame=action,
            applied_semantic_action=_semantic_action_from_native(self.interface_contract, action, step.applied_action),
            observation_frame=_observation_frame(
                status,
                self.interface_contract.observation_profile(self.composition.observation.profile_id),
            ),
            status_frame=status,
        )
        ####

    def save_checkpoint(self, path: str | Path) -> Path:
        """Persist only local numeric state and the exact composition identity."""

        self._require_open()
        payload: dict[str, object] = {
            "schema": "taoryx.local-direct-wrench-composition-episode/v1alpha1",
            "composition_identity_sha256": self.composition.identity_sha256,
            "interface_fingerprint_sha256": self.interface_contract.fingerprint,
            "seed": self.seed,
            "time_s": self._time_s,
            "status": self._status,
            "state": dict(self._state),
            "last_requested": dict(self._last_requested),
            "projection": self._projection.as_dict(),
        }
        payload["integrity"] = _payload_digest(payload)
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(f".{destination.name}.tmp")
        temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        os.replace(temporary, destination)
        return destination
        ####

    def load_checkpoint(self, path: str | Path) -> EpisodeObservation:
        """Restore a validated local boundary while reconstructing source callbacks."""

        self._require_open()
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if payload.get("schema") != "taoryx.local-direct-wrench-composition-episode/v1alpha1":
            raise ValueError("unsupported local direct-wrench episode checkpoint schema")
        integrity = payload.pop("integrity", None)
        if integrity != _payload_digest(payload):
            raise ValueError("local direct-wrench episode checkpoint integrity verification failed")
        if payload.get("composition_identity_sha256") != self.composition.identity_sha256:
            raise ValueError("local direct-wrench episode checkpoint composition mismatch")
        if payload.get("interface_fingerprint_sha256") != self.interface_contract.fingerprint:
            raise ValueError("local direct-wrench episode checkpoint interface mismatch")
        self.seed = payload.get("seed") if isinstance(payload.get("seed"), int) else None
        self._time_s = _finite_number(payload.get("time_s"), "local direct-wrench episode checkpoint time_s")
        if not 0.0 <= self._time_s <= self.config.duration_s + 1.0e-12:
            raise ValueError("local direct-wrench episode checkpoint time is outside the local-screen duration")
        self._status = _episode_status_literal(payload.get("status"))
        state = payload.get("state")
        request = payload.get("last_requested")
        projection = payload.get("projection")
        if not isinstance(state, Mapping) or not isinstance(request, Mapping) or not isinstance(projection, Mapping):
            raise ValueError("local direct-wrench episode checkpoint has malformed local state")
        self._state = {name: _finite_number(state.get(name), f"local direct-wrench checkpoint state {name}") for name in self.config.state_names}
        self._last_requested = _direct_wrench_mapping(request, "local direct-wrench checkpoint request")
        self._projection = _projection_from_payload(projection)
        return self.observe()
        ####

    def close(self) -> None:
        self._closed = True
        self._status = "closed"
        ####

    def _initial_requested_wrench(self) -> dict[str, float]:
        return _direct_wrench_mapping(self.config.balancing_wrench(self.config.reference_state), "local balancing wrench")
        ####

    def _resolve_action(self, action: Mapping[str, object]) -> dict[str, float]:
        unknown = sorted(set(action) - {channel.name for channel in self._ACTION_SCHEMA})
        if unknown:
            raise ValueError("unknown local direct-wrench episode action(s): " + ", ".join(unknown))
        values = dict(self._last_requested)
        if "force_body_n" in action:
            values.update(_direct_wrench_vector(action["force_body_n"], ("force_x_n", "force_y_n", "force_z_n"), "force_body_n"))
        if "moment_body_nm" in action:
            values.update(
                _direct_wrench_vector(
                    action["moment_body_nm"],
                    ("moment_x_nm", "moment_y_nm", "moment_z_nm"),
                    "moment_body_nm",
                )
            )
        return _direct_wrench_mapping(values, "local direct-wrench episode action")
        ####

    def _raw_values(self) -> dict[str, object]:
        wrench = self._projection
        return {
            "body_velocity_m_s": [self._state[name] for name in ("u_m_s", "v_m_s", "w_m_s")],
            "body_rate_rad_s": [self._state[name] for name in ("p_rad_s", "q_rad_s", "r_rad_s")],
            "requested_force_body_n": _force_vector(wrench.requested),
            "requested_moment_body_nm": _moment_vector(wrench.requested),
            "achieved_force_body_n": _force_vector(wrench.achieved),
            "achieved_moment_body_nm": _moment_vector(wrench.achieved),
            "residual_force_body_n": _force_vector(wrench.residual),
            "residual_moment_body_nm": _moment_vector(wrench.residual),
            "wrench_residual_norm": wrench.residual_norm,
            "feedback_norm": self._feedback_norm(),
            "wrench_status": wrench.status,
            "wrench_saturated": bool(wrench.position_saturated or wrench.rate_limited),
            "control_realization": "direct_wrench_screen",
            "physical_effector_allocation": False,
            # Resource values are part of the source-local screen definition,
            # not an inferred episode property.  Project them here so an
            # interactive endpoint that advertises a fixed resource produces
            # the same committed resource view as its batch counterpart.
            **{str(name): float(value) for name, value in self.config.resource_values.items()},
        }
        ####

    def _feedback_norm(self) -> float:
        """Return the batch-screen normalized state error at this boundary."""

        names = self.config.assessment_state_names or self.config.state_names
        scales = dict(zip(self.config.state_names, self.config.state_scales, strict=True))
        return math.sqrt(
            sum(
                ((self._state[name] - self.config.reference_state[name]) / scales[name]) ** 2
                for name in names
            )
        )
        ####

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("vehicle composition episode is closed")
        ####

    ####


# Compatibility name for callers that used the first installed local screen.
X15LocalDirectWrenchCompositionEpisode = LocalDirectWrenchCompositionEpisode


EpisodeFactory = Callable[[CompiledVehicleComposition, int | None, float], VehicleCompositionEpisode]


def _open_language_backed_episode(
    composition: CompiledVehicleComposition,
    seed: int | None,
    integration_step_s: float,
) -> VehicleCompositionEpisode:
    del integration_step_s
    return LanguageBackedCompositionEpisode(composition, seed=seed)
    ####


def _open_hummingbird_episode(
    composition: CompiledVehicleComposition,
    seed: int | None,
    integration_step_s: float,
) -> VehicleCompositionEpisode:
    return HummingbirdPseudoCompositionEpisode(composition, seed=seed, integration_step_s=integration_step_s)
    ####


def _open_a320_reduced_episode(
    composition: CompiledVehicleComposition,
    seed: int | None,
    integration_step_s: float,
) -> VehicleCompositionEpisode:
    del integration_step_s
    return A320ReducedCompositionEpisode(composition, seed=seed)
    ####


def _open_f16_reduced_episode(
    composition: CompiledVehicleComposition,
    seed: int | None,
    integration_step_s: float,
) -> VehicleCompositionEpisode:
    del integration_step_s
    return F16ReducedCompositionEpisode(composition, seed=seed)
    ####


def _open_local_direct_wrench_episode(
    composition: CompiledVehicleComposition,
    seed: int | None,
    integration_step_s: float,
) -> VehicleCompositionEpisode:
    del integration_step_s
    return LocalDirectWrenchCompositionEpisode(composition, seed=seed)
    ####


def _episode_factories(*, plugins: PluginCatalog | None = None) -> dict[str, EpisodeFactory]:
    """Build the installed interactive factory registry."""

    selected = plugins or discover_plugins()
    factories: dict[str, EpisodeFactory] = {}
    for contribution in selected.records("episode_factory"):
        if not callable(contribution.value):
            raise TypeError(f"plug-in {contribution.plugin.id!r} supplied a non-callable episode factory for {contribution.id!r}")
        factories[contribution.id] = cast(EpisodeFactory, contribution.value)
    return factories
    ####


def registered_episode_factory_ids(*, plugins: PluginCatalog | None = None) -> tuple[str, ...]:
    """Return every public episode factory implemented by this runtime.

    The execution-binding catalog remains the authority for whether a factory
    is advertised for a particular family/mission/fidelity. This registry
    asserts the complementary runtime fact: a runnable factory identifier has
    a concrete constructor that preserves a composition rather than selecting
    a nearby fallback model.
    """

    return tuple(sorted(_episode_factories(plugins=plugins)))
    ####


def open_vehicle_composition_episode(
    composition: CompiledVehicleComposition,
    *,
    seed: int | None = None,
    integration_step_s: float = 0.02,
    plugins: PluginCatalog | None = None,
) -> VehicleCompositionEpisode:
    """Open one declared composition episode or fail without a substitute path."""

    try:
        binding = resolve_vehicle_execution_binding(composition, "episode")
    except VehicleExecutionBindingError as error:
        raise ValueError(f"no composition episode adapter is registered: {error}") from error
    if binding.factory_id is None:
        raise ValueError("runnable episode binding lacks a factory identifier")
    try:
        factory = _episode_factories(plugins=plugins)[binding.factory_id]
    except KeyError as error:
        raise ValueError(f"episode execution factory is declared but not implemented: {binding.factory_id!r}") from error
    return factory(composition, seed, integration_step_s)
    ####


def validate_vehicle_composition_episode_contract(
    episode: VehicleCompositionEpisode,
) -> dict[str, object]:
    """Validate one opened episode against its resolved semantic contract.

    Episodes intentionally retain family-owned native action and observation
    names for source-runtime compatibility.  Those names are not a second
    public control API: every externally accepted native action must be bound
    by an available semantic authority profile, and every canonical status or
    observation frame must retain the immutable interface identity at a
    committed truth boundary.  This inspection never steps or resets the
    episode, so it can run as part of endpoint conformance without changing
    the selected simulation state.
    """

    contract = episode.interface_contract
    findings: list[str] = []
    mission_graph = episode.composition.mission_graph
    graph_summary: dict[str, object]
    if mission_graph is None:
        findings.append("episode composition omits its compiled mission graph")
        graph_summary = {
            "status": "missing",
            "execution_observation_status": "not_emitted_by_episode",
        }
    else:
        graph_instance_ids = [node.instance_id for node in mission_graph.nodes]
        graph_segment_ids = [node.segment_id for node in mission_graph.nodes]
        if len(graph_instance_ids) != len(set(graph_instance_ids)):
            findings.append("episode composition mission graph has duplicate instance IDs")
        if tuple(graph_segment_ids) != tuple(segment.id for segment in episode.composition.segments):
            findings.append("episode composition mission graph does not match its compiled segment sequence")
        if tuple(graph_instance_ids) != tuple(segment.instance_id for segment in episode.composition.segments):
            findings.append("episode composition mission graph instance order does not match its compiled segments")
        if mission_graph.entry_instance_id not in graph_instance_ids:
            findings.append("episode composition mission graph entry is not a declared instance")
        graph_summary = {
            "status": "bound",
            "graph_status": mission_graph.status,
            "entry_instance_id": mission_graph.entry_instance_id,
            "instance_ids": graph_instance_ids,
            "segment_ids": graph_segment_ids,
            "execution_contract": mission_graph_execution_contract(
                episode.composition.family_id,
                episode.composition.mission,
                episode.composition.fidelity,
            ),
            "execution_observation_status": "not_emitted_by_episode",
            "claim_boundary": (
                "The episode is bound to this immutable graph, but opening or stepping it does not claim that a "
                "family mission translator dispatched graph transitions. Batch graph-execution evidence remains separate."
            ),
        }
    raw_action_channels = {native_channel.name: native_channel for native_channel in episode.action_schema}
    raw_observation_channels = {native_channel.name: native_channel for native_channel in episode.observation_schema}

    if len(raw_action_channels) != len(episode.action_schema):
        findings.append("native episode action schema contains duplicate names")
    if len(raw_observation_channels) != len(episode.observation_schema):
        findings.append("native episode observation schema contains duplicate names")
    for native_channel in (*episode.action_schema, *episode.observation_schema):
        if native_channel.value_space is None:
            findings.append(f"native episode channel {native_channel.name!r} omits a value-space declaration")

    available_profiles = tuple(profile for profile in contract.authority_profiles if profile.availability == "available")
    semantic_actions_by_id = {semantic_channel.id: semantic_channel for semantic_channel in contract.action_channels}
    semantic_to_native: dict[str, str] = {}
    profile_schema_resolver = getattr(episode, "authority_action_schema", None)
    for profile in available_profiles:
        profile_action_channels = raw_action_channels
        if callable(profile_schema_resolver):
            resolved_profile_schema = profile_schema_resolver(profile.id)
            profile_action_channels = {channel.name: channel for channel in resolved_profile_schema}
            if len(profile_action_channels) != len(resolved_profile_schema):
                findings.append(f"authority profile {profile.id!r} adapter schema contains duplicate names")
            for adapter_channel in resolved_profile_schema:
                if adapter_channel.value_space is None:
                    findings.append(
                        f"authority profile {profile.id!r} adapter action {adapter_channel.name!r} omits a value-space declaration"
                    )
        for identifier in profile.action_ids:
            semantic_channel = semantic_actions_by_id[identifier]
            if semantic_channel.availability != "available":
                findings.append(f"available authority profile {profile.id!r} exposes unavailable action {identifier!r}")
                continue
            native = semantic_channel.binding.get("native_action")
            if not isinstance(native, str) or not native.strip():
                findings.append(f"semantic action {identifier!r} has no episode-native binding")
                continue
            semantic_to_native[identifier] = native
            bound_native_channel = profile_action_channels.get(native)
            if bound_native_channel is None:
                findings.append(
                    f"semantic action {identifier!r} maps to absent {profile.id!r} episode-adapter action {native!r}"
                )
                continue
            if bound_native_channel.value_space is None:
                findings.append(f"native action {native!r} has no declared value space")
            if semantic_channel.value_space is None:
                findings.append(f"semantic action {identifier!r} has no declared value space")
            unit_compatible = semantic_channel.canonical_unit == bound_native_channel.unit or (
                semantic_channel.value_type == "boolean" and semantic_channel.canonical_unit is None and bound_native_channel.unit == "boolean"
            )
            if not unit_compatible:
                findings.append(
                    f"semantic action {identifier!r} unit {semantic_channel.canonical_unit!r} does not match "
                    f"native action {native!r} unit {bound_native_channel.unit!r}"
                )
            if semantic_channel.lower is not None and bound_native_channel.lower is not None and bound_native_channel.lower > semantic_channel.lower:
                findings.append(
                    f"native action {native!r} lower bound {bound_native_channel.lower!r} excludes "
                    f"semantic action {identifier!r} lower bound {semantic_channel.lower!r}"
                )
            if semantic_channel.upper is not None and bound_native_channel.upper is not None and bound_native_channel.upper < semantic_channel.upper:
                findings.append(
                    f"native action {native!r} upper bound {bound_native_channel.upper!r} excludes "
                    f"semantic action {identifier!r} upper bound {semantic_channel.upper!r}"
                )

    bound_native_actions = set(semantic_to_native.values())
    unbound_native_actions = sorted(set(raw_action_channels) - bound_native_actions)
    if unbound_native_actions:
        findings.append("native episode actions lack an available semantic binding: " + ", ".join(unbound_native_actions))
    duplicate_native_bindings = sorted(
        native for native in set(semantic_to_native.values()) if sum(bound == native for bound in semantic_to_native.values()) > 1
    )
    if duplicate_native_bindings:
        findings.append("multiple semantic actions map to one native episode action: " + ", ".join(duplicate_native_bindings))

    raw_observation = episode.observe()
    flattened_native_observation = _flatten_episode_native_values(raw_observation.values)
    semantic_observation_ids = {
        semantic_channel.id
        for semantic_channel in (*contract.status_channels, *contract.resource_channels, *contract.diagnostic_channels)
        if semantic_channel.availability == "available"
    }
    if set(raw_observation_channels) == semantic_observation_ids:
        observation_schema_projection = "semantic_contract"
    else:
        observation_schema_projection = "native_truth"
        missing_native_observation_schema = sorted(set(flattened_native_observation) - set(raw_observation_channels))
        if missing_native_observation_schema:
            findings.append("committed native observation keys lack schema entries: " + ", ".join(missing_native_observation_schema))
        missing_native_observation_values = sorted(set(raw_observation_channels) - set(flattened_native_observation))
        if missing_native_observation_values:
            findings.append("native observation schema keys are absent from committed truth: " + ", ".join(missing_native_observation_values))

    status = episode.status_frame()
    if status.interface_id != contract.id:
        findings.append(f"status frame interface ID {status.interface_id!r} does not match {contract.id!r}")
    if status.interface_fingerprint_sha256 != contract.fingerprint:
        findings.append("status frame interface fingerprint does not match the resolved contract")
    if status.time_s != raw_observation.time_s:
        findings.append("status frame time does not match the committed native observation time")
    if status.status != raw_observation.status:
        findings.append("status frame lifecycle state does not match the committed native observation")

    available_observations: list[dict[str, object]] = []
    for observation_profile in contract.observation_profiles:
        if observation_profile.availability != "available":
            continue
        observation = episode.observe_frame(observation_profile.id)
        if observation.interface_id != contract.id:
            findings.append(f"observation profile {observation_profile.id!r} has interface ID {observation.interface_id!r}, expected {contract.id!r}")
        if observation.interface_fingerprint_sha256 != contract.fingerprint:
            findings.append(f"observation profile {observation_profile.id!r} has a mismatched interface fingerprint")
        if observation.observation_profile_id != observation_profile.id:
            findings.append(f"observation profile {observation_profile.id!r} returned identity {observation.observation_profile_id!r}")
        if observation.time_s != status.time_s or observation.status != status.status:
            findings.append(f"observation profile {observation_profile.id!r} is not aligned to the committed status boundary")
        unknown = sorted(set(observation.values) - set(observation_profile.channel_ids))
        if unknown:
            findings.append(f"observation profile {observation_profile.id!r} emitted undeclared channels: {', '.join(unknown)}")
        available_observations.append(
            {
                "id": observation_profile.id,
                "source": observation_profile.source,
                "channel_count": len(observation_profile.channel_ids),
                "present_channel_count": len(observation.values),
            }
        )

    return {
        "schema": "taoryx.vehicle-composition-episode-contract/v1alpha1",
        "status": "pass" if not findings else "fail",
        "composition_id": episode.composition.id,
        "composition_identity_sha256": episode.composition.identity_sha256,
        "interface_id": contract.id,
        "interface_fingerprint_sha256": contract.fingerprint,
        "available_authority_profiles": [profile.id for profile in available_profiles],
        "semantic_action_channel_count": len(semantic_to_native),
        "profile_adapter_action_channel_count": len(set(semantic_to_native.values())),
        "native_action_channel_count": len(raw_action_channels),
        "native_observation_channel_count": len(raw_observation_channels),
        "observation_schema_projection": observation_schema_projection,
        "available_observation_profiles": available_observations,
        "mission_graph": graph_summary,
        "findings": findings,
        "claim_boundary": (
            "This confirms that the opened source-owned episode exposes its native and profile-adapter semantic action, status, "
            "observation, and immutable mission-graph contract without a native-control bypass. It does not establish "
            "graph transition execution, physical actuator realization, mission qualification, or batch/episode trajectory parity."
        ),
    }
    ####


def _flatten_episode_native_values(
    values: Mapping[str, object],
    *,
    prefix: str = "",
) -> dict[str, object]:
    """Flatten source-table nested truth into the episode schema's dotted keys."""

    flattened: dict[str, object] = {}
    for name, value in values.items():
        identifier = f"{prefix}.{name}" if prefix else str(name)
        if isinstance(value, Mapping):
            flattened.update(_flatten_episode_native_values(value, prefix=identifier))
        else:
            flattened[identifier] = value
    return flattened
    ####


def _a320_stepper_for_composition(composition: CompiledVehicleComposition) -> A320RacetrackStepper:
    """Build the exact A320 reduced plant selected by one composition."""

    from taoryx_reference_models.resources import model_resource_root

    from .reduced_fixed_wing_execution import _a320_operating_point_for_speed
    from .trajectory.a320_openap import A320OpenAPModel
    from .trajectory.a320_pseudo6dof import A320Pseudo6DOFModel
    from .trajectory.a320_racetrack import A320RacetrackRunner, A320RacetrackStepper

    route = compile_powered_fixed_wing_racetrack_from_composition(composition).route
    inputs = composition.initialization.inputs
    altitude_m = _composition_number(inputs, "altitude_m", default=6000.0)
    mass_kg = _composition_number(inputs, "mass_kg", default=60000.0)
    root = model_resource_root()
    base_model = A320OpenAPModel.from_repository(root)
    operating_point = _a320_operating_point_for_speed(base_model, altitude_m, mass_kg, route.speed_m_s)
    model: A320OpenAPModel | A320Pseudo6DOFModel
    response_profile: Pseudo6DOFProfile | None
    mode: Literal["point_mass_3dof", "pseudo_6dof_kinematic_bridge"]
    if composition.fidelity == "point_mass_3dof":
        model = base_model
        trim = model.trim_level_flight(operating_point)
        response_profile = None
        mode = "point_mass_3dof"
    elif composition.fidelity == "pseudo_6dof":
        model = A320Pseudo6DOFModel.from_repository(root)
        trim = model.trim_pseudo6dof(operating_point)
        _, response_profile = load_pseudo6dof_catalog(root / "verification/pseudo6dof_profiles.yaml").for_family("a320_openap_3dof")
        mode = "pseudo_6dof_kinematic_bridge"
    else:
        raise ValueError(f"A320 reduced episode has no fidelity adapter for {composition.fidelity!r}")
    return A320RacetrackStepper(
        A320RacetrackRunner(
            model,
            trim,
            route,
            mode,
            dt_s=0.2,
            response_profile=response_profile,
            operating_point=operating_point,
        )
    )
    ####


def _f16_stepper_for_composition(composition: CompiledVehicleComposition) -> F16ReducedRacetrackStepper:
    """Build the exact source-trimmed F-16 reduction selected by one composition."""

    from taoryx_reference_models.resources import model_resource_root

    from .reduced_fixed_wing_execution import _f16_source_trim
    from .trajectory.f16_reduced_racetrack import F16ReducedRacetrackRunner, F16ReducedRacetrackStepper
    from .trajectory.f16_reductions import F16AttitudeResponsePseudo6DOFModel, F16PointMass3DOFModel

    route = compile_powered_fixed_wing_racetrack_from_composition(composition).route
    source, trim, trim_pitch_rad = _f16_source_trim()
    observed_mach = float(source.evaluate_loads(trim.state, trim.controls, altitude_m=0.0)["mach"])
    requested_mach = _composition_number(composition.initialization.inputs, "mach", default=observed_mach)
    if not math.isclose(requested_mach, observed_mach, abs_tol=1.0e-9):
        raise ValueError(f"F-16 reduced episode is pinned to its source trim Mach; requested {requested_mach:.12g}, expected {observed_mach:.12g}")
    mode: Literal["point_mass_3dof", "pseudo_6dof_kinematic_bridge"]
    model: F16PointMass3DOFModel | F16AttitudeResponsePseudo6DOFModel
    if composition.fidelity == "point_mass_3dof":
        model = F16PointMass3DOFModel(source, trim, trim_pitch_rad)
        response_profile = None
        mode = "point_mass_3dof"
    elif composition.fidelity == "pseudo_6dof":
        linearization = source.linearize_local(
            trim.state,
            trim.controls,
            trim_pitch_rad=trim_pitch_rad,
            altitude_m=0.0,
            state_step=1.0e-5,
            control_step=1.0e-5,
        )
        model = F16AttitudeResponsePseudo6DOFModel(source, trim, linearization, trim_pitch_rad)
        root = model_resource_root()
        _, response_profile = load_pseudo6dof_catalog(root / "verification/pseudo6dof_profiles.yaml").for_family("f16_s119")
        mode = "pseudo_6dof_kinematic_bridge"
    else:
        raise ValueError(f"F-16 reduced episode has no fidelity adapter for {composition.fidelity!r}")
    return F16ReducedRacetrackStepper(F16ReducedRacetrackRunner(model, trim, route, mode, dt_s=0.2, response_profile=response_profile))
    ####


def _a320_guidance_override(action: Mapping[str, float]) -> A320GuidanceOverride:
    """Translate held public/native A320 guidance coordinates into SI/radians."""

    from .trajectory.a320_racetrack import A320GuidanceOverride

    return A320GuidanceOverride(
        speed_m_s=action.get("speed_m_s"),
        flight_path_angle_rad=(None if "flight_path_angle_deg" not in action else math.radians(action["flight_path_angle_deg"])),
        heading_rad=None if "heading_deg" not in action else math.radians(action["heading_deg"]),
        bank_angle_rad=None if "bank_angle_deg" not in action else math.radians(action["bank_angle_deg"]),
    )
    ####


def _f16_guidance_override(action: Mapping[str, float]) -> F16GuidanceOverride:
    """Translate held public/native F-16 guidance coordinates into SI/radians."""

    from .trajectory.f16_reduced_racetrack import F16GuidanceOverride

    return F16GuidanceOverride(
        speed_m_s=action.get("speed_m_s"),
        flight_path_angle_rad=(None if "flight_path_angle_deg" not in action else math.radians(action["flight_path_angle_deg"])),
        heading_rad=None if "heading_deg" not in action else math.radians(action["heading_deg"]),
        bank_angle_rad=None if "bank_angle_deg" not in action else math.radians(action["bank_angle_deg"]),
    )
    ####


def _a320_status_values(row: Mapping[str, object]) -> dict[str, object]:
    """Enrich one reduced telemetry row with only declared portable vectors."""

    values = dict(row)
    attitude_keys = ("route_bank_achieved_deg", "route_pitch_achieved_deg", "route_heading_achieved_deg")
    rate_keys = ("p_rad_s", "q_rad_s", "r_rad_s")
    if all(key in values for key in attitude_keys):
        values["attitude_euler_deg"] = [values[key] for key in attitude_keys]
    if all(key in values for key in rate_keys):
        values["body_rate_rad_s"] = [values[key] for key in rate_keys]
    control_path = values.get("control_path")
    values["control_realization"] = "response_law" if control_path == "pseudo_6dof_kinematic_bridge" else "force_model"
    return values
    ####


def _f16_stepper_state(payload: Mapping[str, object]) -> F16ReducedRacetrackStepperState:
    """Decode a checkpointed F-16 stepper state with no permissive coercion."""

    from .trajectory.f16_reduced_racetrack import F16ReducedRacetrackStepperState

    controls = payload.get("controls")
    if not isinstance(controls, Mapping):
        raise ValueError("F-16 checkpoint stepper state has no control mapping")
    scalar_controls = {str(name): _finite_number(value, f"F-16 checkpoint control {name!r}") for name, value in controls.items()}
    numerical_valid = payload.get("numerical_valid")
    failure = payload.get("failure")
    if not isinstance(numerical_valid, bool):
        raise ValueError("F-16 checkpoint has invalid numerical status")
    if failure is not None and not isinstance(failure, str):
        raise ValueError("F-16 checkpoint failure must be string or null")
    scalars = [
        _finite_number(payload.get(key), f"F-16 checkpoint {key}")
        for key in (
            "time_s",
            "north_m",
            "east_m",
            "altitude_m",
            "speed_m_s",
            "heading_rad",
            "flight_path_angle_rad",
            "roll_rad",
            "pitch_rad",
            "yaw_rad",
            "p_rad_s",
            "q_rad_s",
            "r_rad_s",
        )
    ]
    return F16ReducedRacetrackStepperState(
        scalars[0],
        scalars[1],
        scalars[2],
        scalars[3],
        scalars[4],
        scalars[5],
        scalars[6],
        scalars[7],
        scalars[8],
        scalars[9],
        scalars[10],
        scalars[11],
        scalars[12],
        scalar_controls,
        numerical_valid,
        failure,
    )
    ####


def _a320_stepper_state(payload: Mapping[str, object]) -> A320RacetrackStepperState:
    """Decode a checkpointed A320 stepper state with no permissive coercion."""

    from .trajectory.a320_racetrack import A320RacetrackStepperState

    state = payload.get("state")
    controls = payload.get("controls")
    if not isinstance(state, Mapping) or not isinstance(controls, Mapping):
        raise ValueError("A320 checkpoint stepper state has no scalar state/control mappings")
    scalar_state = {str(name): _finite_number(value, f"A320 checkpoint state {name!r}") for name, value in state.items()}
    scalar_controls = {str(name): _finite_number(value, f"A320 checkpoint control {name!r}") for name, value in controls.items()}
    terminal_hold = payload.get("terminal_hold")
    numerical_valid = payload.get("numerical_valid")
    failure = payload.get("failure")
    if not isinstance(terminal_hold, bool) or not isinstance(numerical_valid, bool):
        raise ValueError("A320 checkpoint has invalid terminal or numerical status")
    if failure is not None and not isinstance(failure, str):
        raise ValueError("A320 checkpoint failure must be string or null")
    return A320RacetrackStepperState(
        _finite_number(payload.get("time_s"), "A320 checkpoint time_s"),
        _finite_number(payload.get("north_m"), "A320 checkpoint north_m"),
        _finite_number(payload.get("east_m"), "A320 checkpoint east_m"),
        scalar_state,
        scalar_controls,
        terminal_hold,
        numerical_valid,
        failure,
    )
    ####


def _compile_language_backed_scenario(composition: CompiledVehicleComposition, *, seed: int | None) -> ResolvedScenario:
    """Compile disposable materialization into an immutable parsed scenario."""

    with tempfile.TemporaryDirectory(prefix="taoryx-composition-episode-") as temporary:
        materialized = materialize_powered_fixed_wing_composition(composition, Path(temporary))
        mission = _mission(materialized.mission_config, materialized.materialized_mission_id)
        return ScenarioCompiler().compile(
            materialized.problem,
            table_paths=_mission_tables(mission),
            profile=GrammarProfile.TAORYX,
            seed=seed,
            integrator=str(mission.get("integrator", "rk4")),
        )
    ####


def _interface_contract_for_composition(composition: CompiledVehicleComposition) -> VehicleInterfaceContract:
    """Resolve a fingerprinted interface including any declared sensor view."""

    return resolve_vehicle_composition_interface_contract(composition)
    ####


def _sensor_for_contract(
    contract: VehicleInterfaceContract,
    *,
    seed: int | None,
) -> CommittedBoundarySensor | None:
    """Create the one selected declared sensor or leave truth-only runs alone."""

    profiles = tuple(profile for profile in contract.observation_profiles if profile.source == "sensor" and profile.availability == "available")
    if not profiles:
        return None
    if len(profiles) != 1:
        raise ValueError("an initial composition episode supports exactly one available declared sensor profile")
    profile = profiles[0]
    if profile.cadence_s is None or profile.latency_s is None:
        raise ValueError(f"declared sensor profile {profile.id!r} has incomplete timing metadata")
    return CommittedBoundarySensor(
        profile_id=profile.id,
        channel_ids=profile.channel_ids,
        cadence_s=profile.cadence_s,
        latency_s=profile.latency_s,
        channel_errors=profile.channel_errors,
        seed=0 if seed is None else seed,
    )
    ####


def _next_sensor_boundary(
    sensor: CommittedBoundarySensor | None,
    current_time_s: float,
    upper_time_s: float,
) -> float | None:
    """Return a sensor-required truth boundary that an outer action must honor."""

    return None if sensor is None else sensor.next_required_boundary(current_time_s, upper_time_s)
    ####


def _composition_number(values: Mapping[str, Any], name: str, *, default: float) -> float:
    value = values.get(name)
    if value is None:
        return default
    return float(value.value)
    ####


def _hummingbird_initial_mass_kg(composition: CompiledVehicleComposition) -> float:
    """Resolve the one declared mass-bearing Hummingbird reset contract."""

    if composition.initialization.id != "grounded_idle":
        return 0.5
    mass_kg = _composition_number(composition.initialization.inputs, "mass_kg", default=0.5)
    if not math.isfinite(mass_kg) or mass_kg <= 0.0:
        raise ValueError("Hummingbird grounded_idle mass_kg must be positive and finite")
    return mass_kg
    ####


def _numeric_action(action: Mapping[str, object]) -> dict[str, float]:
    """Normalize semantic booleans and numeric controls for the language runtime."""

    values: dict[str, float] = {}
    for name, raw in action.items():
        if isinstance(raw, bool):
            # Language ``*runtime control`` values are numeric.  A semantic
            # boolean remains explicit at the interface boundary, then maps
            # to the native 0/1 convention only for its declared binding.
            numeric = 1.0 if raw else 0.0
        else:
            numeric = _finite_number(raw, f"language-backed action {name!r}")
        values[name] = numeric
    return values
    ####


def _hummingbird_action(action: object) -> dict[str, object]:
    if not isinstance(action, Mapping):
        raise ValueError("Hummingbird episode action must be a mapping")
    result: dict[str, object] = {}
    limits = {"roll_rad": math.pi / 2.0, "pitch_rad": math.pi / 2.0, "yaw_rad": math.pi}
    for name, limit in limits.items():
        raw = action.get(name)
        if isinstance(raw, bool) or raw is None:
            raise ValueError(f"Hummingbird episode action {name!r} must be finite numeric")
        numeric = _finite_number(raw, f"Hummingbird episode action {name!r}")
        result[name] = max(-limit, min(limit, numeric))
    thrust = action.get("thrust_ratio")
    if isinstance(thrust, bool) or thrust is None:
        raise ValueError("Hummingbird episode action 'thrust_ratio' must be finite numeric")
    result["thrust_ratio"] = max(0.0, min(1.0, _finite_number(thrust, "Hummingbird episode action 'thrust_ratio'")))
    motors = action.get("motors_enabled")
    if not isinstance(motors, bool):
        raise ValueError("Hummingbird episode action 'motors_enabled' must be boolean")
    result["motors_enabled"] = motors
    return result
    ####


def _hummingbird_state_payload(state: HummingbirdPseudo6DOFState) -> dict[str, object]:
    return {
        "time_s": state.time_s,
        "position_m": list(state.position_m),
        "velocity_m_s": list(state.velocity_m_s),
        "attitude_rad": list(state.attitude_rad),
        "attitude_rate_rad_s": list(state.attitude_rate_rad_s),
        "battery_fraction": state.battery_fraction,
        "thrust_n": state.thrust_n,
        "contact": state.contact,
    }
    ####


def _hummingbird_state_from_payload(payload: object) -> HummingbirdPseudo6DOFState:
    from .trajectory.hummingbird_pseudo6dof import HummingbirdPseudo6DOFState

    if not isinstance(payload, Mapping):
        raise ValueError("Hummingbird episode checkpoint state must be a mapping")
    vectors = {
        name: _three_vector(payload.get(name), f"Hummingbird episode checkpoint {name}")
        for name in ("position_m", "velocity_m_s", "attitude_rad", "attitude_rate_rad_s")
    }
    return HummingbirdPseudo6DOFState(
        _finite_number(payload.get("time_s"), "Hummingbird episode checkpoint time_s"),
        vectors["position_m"],
        vectors["velocity_m_s"],
        vectors["attitude_rad"],
        vectors["attitude_rate_rad_s"],
        _finite_number(payload.get("battery_fraction"), "Hummingbird episode checkpoint battery_fraction"),
        _finite_number(payload.get("thrust_n"), "Hummingbird episode checkpoint thrust_n"),
        bool(payload["contact"]),
    )
    ####


def _episode_status(status: InteractiveStatus, closed: bool) -> EpisodeStatus:
    if closed:
        return "closed"
    if status is InteractiveStatus.COMPLETED:
        return "completed"
    if status is InteractiveStatus.CREATED:
        return "ready"
    return "active"
    ####


def _episode_status_literal(value: object) -> EpisodeStatus:
    if value not in {"ready", "active", "completed", "closed"}:
        raise ValueError("invalid Hummingbird episode checkpoint status")
    return value
    ####


def _payload_digest(payload: Mapping[str, object]) -> str:
    encoded = json.dumps(_json_safe(payload), separators=(",", ":"), sort_keys=True).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
    ####


def _native_action_for_frame(
    contract: VehicleInterfaceContract,
    action: ActionFrame,
) -> dict[str, object]:
    """Validate one semantic frame and map it to declared native controls."""

    if action.interface_id != contract.id:
        raise ValueError(f"action frame interface mismatch: {action.interface_id!r} != {contract.id!r}")
    if action.interface_fingerprint_sha256 != contract.fingerprint:
        raise ValueError("action frame interface fingerprint mismatch")
    validate_authority_action_values(
        contract,
        action.authority_profile_id,
        action.values,
        context=f"semantic action frame for authority profile {action.authority_profile_id!r}",
    )
    by_id = {channel.id: channel for channel in contract.action_channels}
    result: dict[str, object] = {}
    for identifier, value in action.values.items():
        channel = by_id[identifier]
        native = channel.binding.get("native_action")
        if not isinstance(native, str) or not native:
            raise ValueError(f"action channel {identifier!r} has no episode-native binding")
        result[native] = value
    return result
    ####


def _semantic_action_from_native(
    contract: VehicleInterfaceContract,
    action: ActionFrame,
    native_action: Mapping[str, object],
) -> dict[str, object]:
    """Project applied native controls back to the selected semantic profile."""

    profile = contract.authority_profile(action.authority_profile_id)
    channels = {channel.id: channel for channel in contract.action_channels}
    result: dict[str, object] = {}
    for identifier in profile.action_ids:
        channel = channels[identifier]
        native = channel.binding.get("native_action")
        if isinstance(native, str) and native in native_action:
            value = native_action[native]
            if channel.value_type == "boolean" and isinstance(value, int | float) and not isinstance(value, bool):
                result[identifier] = float(value) >= 0.5
            else:
                result[identifier] = value
    return result
    ####


def _status_frame(contract: VehicleInterfaceContract, observation: EpisodeObservation) -> StatusFrame:
    """Project one raw committed observation into the resolved status schema."""

    values = project_committed_status_values(
        contract,
        time_s=observation.time_s,
        execution_status=observation.status,
        raw_values=observation.values,
    )
    validate_projected_status_values(
        contract,
        values,
        context=f"episode status frame t={observation.time_s:.12g} s",
    )
    return StatusFrame(
        contract.id,
        contract.fingerprint,
        observation.time_s,
        values,
        observation.values,
        observation.status,
    )
    ####


def _observation_frame(status: StatusFrame, profile: ObservationProfile) -> ObservationFrame:
    """Select the declared values for one available observation profile."""

    if profile.availability != "available":
        raise ValueError(f"observation profile {profile.id!r} is {profile.availability}, not executable")
    valid = {identifier: identifier in status.values for identifier in profile.channel_ids}
    return ObservationFrame(
        status.interface_id,
        status.interface_fingerprint_sha256,
        profile.id,
        status.time_s,
        {identifier: status.values[identifier] for identifier, present in valid.items() if present},
        valid,
        status.status,
        status.time_s,
    )
    ####


def _episode_observation_frame(
    status: StatusFrame,
    profile: ObservationProfile,
    sensor: CommittedBoundarySensor | None,
) -> ObservationFrame:
    """Select truth or declared delayed sensor data without a hidden fallback."""

    if profile.source != "sensor":
        return _observation_frame(status, profile)
    if sensor is None or sensor.profile_id != profile.id:
        raise ValueError(f"sensor observation profile {profile.id!r} has no declared runtime instance")
    reading = sensor.reading(status.time_s)
    return ObservationFrame(
        status.interface_id,
        status.interface_fingerprint_sha256,
        profile.id,
        status.time_s,
        reading.values,
        reading.valid,
        status.status,
        reading.sample_time_s,
    )
    ####


def _finite_number(value: object, label: str) -> float:
    """Return one finite numeric payload value with a useful boundary error."""

    if isinstance(value, bool) or value is None:
        raise ValueError(f"{label} must be finite numeric")
    try:
        numeric = float(cast(Any, value))
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} must be finite numeric") from error
    if not math.isfinite(numeric):
        raise ValueError(f"{label} must be finite numeric")
    return numeric
    ####


def _three_vector(value: object, label: str) -> tuple[float, float, float]:
    """Parse an exact finite three-vector from a checkpoint payload."""

    if not isinstance(value, list | tuple) or len(value) != 3:
        raise ValueError(f"{label} must be a finite three-vector")
    return (
        _finite_number(value[0], label),
        _finite_number(value[1], label),
        _finite_number(value[2], label),
    )
    ####


def _direct_wrench_vector(value: object, names: tuple[str, str, str], label: str) -> dict[str, float]:
    """Parse one finite direct-wrench vector into its canonical axis names."""

    vector = _three_vector(value, label)
    return {name: item for name, item in zip(names, vector, strict=True)}
    ####


def _direct_wrench_mapping(value: Mapping[str, object], label: str) -> dict[str, float]:
    """Validate a complete canonical six-axis direct-wrench mapping."""

    missing = set(DIRECT_WRENCH_NAMES) - set(value)
    extra = set(value) - set(DIRECT_WRENCH_NAMES)
    if missing or extra:
        raise ValueError(f"{label} axes mismatch; missing={sorted(missing)}, extra={sorted(extra)}")
    return {name: _finite_number(value[name], f"{label} {name}") for name in DIRECT_WRENCH_NAMES}
    ####


def _projection_from_payload(payload: Mapping[str, object]) -> DirectWrenchProjection:
    """Restore the direct-wrench projection without trusting an opaque blob."""

    requested = payload.get("requested_wrench")
    achieved = payload.get("achieved_wrench")
    residual = payload.get("residual_wrench")
    status = payload.get("status")
    position_saturated = payload.get("position_saturated", [])
    rate_limited = payload.get("rate_limited", [])
    if not isinstance(requested, Mapping) or not isinstance(achieved, Mapping) or not isinstance(residual, Mapping):
        raise ValueError("local direct-wrench checkpoint projection mappings are malformed")
    if status not in {"feasible", "partially_achievable"}:
        raise ValueError("local direct-wrench checkpoint projection status is invalid")
    if not isinstance(position_saturated, list) or not isinstance(rate_limited, list):
        raise ValueError("local direct-wrench checkpoint projection limit flags are malformed")
    return DirectWrenchProjection(
        _direct_wrench_mapping(requested, "local direct-wrench checkpoint requested projection"),
        _direct_wrench_mapping(achieved, "local direct-wrench checkpoint achieved projection"),
        _direct_wrench_mapping(residual, "local direct-wrench checkpoint residual projection"),
        status,
        tuple(str(name) for name in position_saturated),
        tuple(str(name) for name in rate_limited),
    )
    ####


def _force_vector(wrench: Mapping[str, float]) -> list[float]:
    """Project canonical force axes into an ordered portable three-vector."""

    return [float(wrench[name]) for name in ("force_x_n", "force_y_n", "force_z_n")]
    ####


def _moment_vector(wrench: Mapping[str, float]) -> list[float]:
    """Project canonical moment axes into an ordered portable three-vector."""

    return [float(wrench[name]) for name in ("moment_x_nm", "moment_y_nm", "moment_z_nm")]
    ####


def _json_safe(value: object) -> object:
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, tuple | list):
        return [_json_safe(item) for item in value]
    return value
    ####


__all__ = [
    "EpisodeChannel",
    "EpisodeObservation",
    "EpisodeStep",
    "A320ReducedCompositionEpisode",
    "F16ReducedCompositionEpisode",
    "HummingbirdPseudoCompositionEpisode",
    "LanguageBackedCompositionEpisode",
    "MissionCompositionEpisode",
    "ReducedFixedWingCompositionEpisode",
    "X15LocalDirectWrenchCompositionEpisode",
    "VehicleCompositionEpisode",
    "open_vehicle_composition_episode",
    "registered_episode_factory_ids",
    "validate_vehicle_composition_episode_contract",
]
