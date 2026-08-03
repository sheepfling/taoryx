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
from collections.abc import Mapping
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Literal, Protocol, cast

from .committed_boundary_sensor import CommittedBoundarySensor
from .direct_wrench import DIRECT_WRENCH_NAMES, DirectWrenchProjection, DirectWrenchStatus
from .language.grammar_contracts import GrammarProfile
from .language_backed_execution import _mission, _mission_tables
from .language_backed_racetrack import materialize_powered_fixed_wing_composition
from .runtime.interactive import InteractiveSession, InteractiveStatus
from .scenario import ResolvedScenario, ScenarioCompiler
from .trajectory import HummingbirdPseudo6DOFCommand, HummingbirdPseudo6DOFModel, HummingbirdPseudo6DOFState
from .vehicle_composition import CompiledVehicleComposition, resolve_vehicle_composition_interface_contract
from .vehicle_execution_bindings import VehicleExecutionBindingError, resolve_vehicle_execution_binding
from .vehicle_execution_preflight import preflight_vehicle_composition
from .vehicle_interface import (
    ObservationProfile,
    VehicleInterfaceContract,
    project_committed_status_values,
    validate_projected_status_values,
)
from .x15_adapter import build_x15_local_direct_wrench_screen_config

EpisodeStatus = Literal["ready", "active", "completed", "closed"]


@dataclass(frozen=True, slots=True)
class EpisodeChannel:
    """One user-commandable or observable episode channel."""

    name: str
    unit: str | None
    lower: float | None = None
    upper: float | None = None
    description: str = ""

    def as_dict(self) -> dict[str, object]:
        """Return the public channel schema."""

        return {
            "name": self.name,
            "unit": self.unit,
            "lower": self.lower,
            "upper": self.upper,
            "description": self.description,
        }
        ####
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


class VehicleCompositionEpisode(Protocol):
    """Common interactive projection of one immutable vehicle composition."""

    composition: CompiledVehicleComposition
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
        """Close the episode without mutating the resolved composition."""
        ...


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

        return tuple(
            EpisodeChannel(control.name, control.unit, control.lower, control.upper, "language-backed runtime control")
            for control in self._session.controls
        )
        ####

    @property
    def observation_schema(self) -> tuple[EpisodeChannel, ...]:
        """Expose current runtime state fields as committed truth diagnostics."""

        channels: list[EpisodeChannel] = []
        for vehicle in self._session.problem.vehicles.values():
            channels.extend(EpisodeChannel(f"{vehicle.name}.{name}", None, description="committed runtime truth") for name in vehicle.state.value_names)
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
            values[vehicle.name] = {
                **dict(zip(vehicle.state.value_names, vehicle.state.values, strict=False)),
                **dict(vehicle.state.named),
            }
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
    )

    def __init__(self, composition: CompiledVehicleComposition, *, seed: int | None = None, integration_step_s: float = 0.02) -> None:
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
        self.model = HummingbirdPseudo6DOFModel()
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


class X15LocalDirectWrenchCompositionEpisode:
    """Bounded source-local X-15 direct-wrench episode.

    This is intentionally a local controller-screen episode.  It exposes the
    same requested-to-achieved direct-wrench bridge that the public batch
    screen uses, but it neither grows into a release-to-handoff trajectory nor
    rebrands injected body loads as physical X-15 effectors.
    """

    claim_boundary = (
        "This episode applies an explicit bounded direct wrench to one local X-15 source-load state. "
        "It is bridge/screen evidence only: not physical stabilator, rudder, RCS, propulsion, or actuator allocation; "
        "not source physical trim; and not a flight mission."
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
        EpisodeChannel("wrench_saturated", "boolean", description="direct-wrench authority or slew projection status"),
    )

    def __init__(self, composition: CompiledVehicleComposition, *, seed: int | None = None) -> None:
        if (
            composition.family_id != "x15"
            or composition.mission != "x15_local_direct_wrench_screen_v1"
            or composition.fidelity != "rigid_body_6dof_direct_wrench"
        ):
            raise ValueError("X-15 direct-wrench episode requires the declared local screen composition")
        preflight = preflight_vehicle_composition(composition)
        if preflight.status != "translation_ready":
            detail = "; ".join(preflight.diagnostics) or "no source-local direct-wrench translator"
            raise ValueError(f"cannot create X-15 direct-wrench episode for {composition.id!r}: {preflight.status}: {detail}")
        self.composition = composition
        self.interface_contract = _interface_contract_for_composition(composition)
        self.preflight = preflight
        self.seed = seed
        self.config = build_x15_local_direct_wrench_screen_config()
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
            raise RuntimeError("X-15 local direct-wrench screen has completed; reset before stepping again")
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
                name: self._state[name] + dt_s * _finite_number(derivative.get(name), f"X-15 local derivative {name}")
                for name in self.config.state_names
            }
            if any(not math.isfinite(value) for value in self._state.values()):
                raise RuntimeError("X-15 local direct-wrench episode produced a nonfinite state")
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
            "schema": "taoryx.x15-local-direct-wrench-composition-episode/v1alpha1",
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
        if payload.get("schema") != "taoryx.x15-local-direct-wrench-composition-episode/v1alpha1":
            raise ValueError("unsupported X-15 direct-wrench episode checkpoint schema")
        integrity = payload.pop("integrity", None)
        if integrity != _payload_digest(payload):
            raise ValueError("X-15 direct-wrench episode checkpoint integrity verification failed")
        if payload.get("composition_identity_sha256") != self.composition.identity_sha256:
            raise ValueError("X-15 direct-wrench episode checkpoint composition mismatch")
        if payload.get("interface_fingerprint_sha256") != self.interface_contract.fingerprint:
            raise ValueError("X-15 direct-wrench episode checkpoint interface mismatch")
        self.seed = payload.get("seed") if isinstance(payload.get("seed"), int) else None
        self._time_s = _finite_number(payload.get("time_s"), "X-15 direct-wrench episode checkpoint time_s")
        if not 0.0 <= self._time_s <= self.config.duration_s + 1.0e-12:
            raise ValueError("X-15 direct-wrench episode checkpoint time is outside the local-screen duration")
        self._status = _episode_status_literal(payload.get("status"))
        state = payload.get("state")
        request = payload.get("last_requested")
        projection = payload.get("projection")
        if not isinstance(state, Mapping) or not isinstance(request, Mapping) or not isinstance(projection, Mapping):
            raise ValueError("X-15 direct-wrench episode checkpoint has malformed local state")
        self._state = {name: _finite_number(state.get(name), f"X-15 direct-wrench checkpoint state {name}") for name in self.config.state_names}
        self._last_requested = _direct_wrench_mapping(request, "X-15 direct-wrench checkpoint request")
        self._projection = _projection_from_payload(projection)
        return self.observe()
        ####

    def close(self) -> None:
        self._closed = True
        self._status = "closed"
        ####

    def _initial_requested_wrench(self) -> dict[str, float]:
        return _direct_wrench_mapping(self.config.balancing_wrench(self.config.reference_state), "X-15 local balancing wrench")
        ####

    def _resolve_action(self, action: Mapping[str, object]) -> dict[str, float]:
        unknown = sorted(set(action) - {channel.name for channel in self._ACTION_SCHEMA})
        if unknown:
            raise ValueError("unknown X-15 direct-wrench episode action(s): " + ", ".join(unknown))
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
        return _direct_wrench_mapping(values, "X-15 direct-wrench episode action")
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
            "wrench_status": wrench.status,
            "wrench_saturated": bool(wrench.position_saturated or wrench.rate_limited),
            "control_realization": "direct_wrench_screen",
            "physical_effector_allocation": False,
        }
        ####

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("vehicle composition episode is closed")
        ####
    ####


def open_vehicle_composition_episode(
    composition: CompiledVehicleComposition,
    *,
    seed: int | None = None,
    integration_step_s: float = 0.02,
) -> VehicleCompositionEpisode:
    """Open one declared composition episode or fail without a substitute path."""

    try:
        binding = resolve_vehicle_execution_binding(composition, "episode")
    except VehicleExecutionBindingError as error:
        raise ValueError(f"no composition episode adapter is registered: {error}") from error
    if binding.factory_id == "language_backed_interactive.v1":
        return LanguageBackedCompositionEpisode(composition, seed=seed)
    if binding.factory_id == "hummingbird_aggregate_thrust_episode.v1":
        return HummingbirdPseudoCompositionEpisode(composition, seed=seed, integration_step_s=integration_step_s)
    if binding.factory_id == "x15_local_direct_wrench_episode.v1":
        return X15LocalDirectWrenchCompositionEpisode(composition, seed=seed)
    raise ValueError(f"episode execution factory is declared but not implemented: {binding.factory_id!r}")
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


def _numeric_action(action: Mapping[str, object]) -> dict[str, float]:
    """Reject boolean/string controls at the numeric language runtime boundary."""

    values: dict[str, float] = {}
    for name, raw in action.items():
        if isinstance(raw, bool):
            raise ValueError(f"language-backed action {name!r} must be numeric")
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
    return cast(EpisodeStatus, value)
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
    profile = contract.authority_profile(action.authority_profile_id)
    if profile.availability != "available":
        raise ValueError(f"authority profile {profile.id!r} is {profile.availability}, not executable")
    unknown = sorted(set(action.values) - set(profile.action_ids))
    if unknown:
        raise ValueError(f"action frame contains values outside {profile.id!r}: {', '.join(unknown)}")
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
        native = channels[identifier].binding.get("native_action")
        if isinstance(native, str) and native in native_action:
            result[identifier] = native_action[native]
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
        raise ValueError("X-15 direct-wrench checkpoint projection mappings are malformed")
    if status not in {"feasible", "partially_achievable"}:
        raise ValueError("X-15 direct-wrench checkpoint projection status is invalid")
    if not isinstance(position_saturated, list) or not isinstance(rate_limited, list):
        raise ValueError("X-15 direct-wrench checkpoint projection limit flags are malformed")
    return DirectWrenchProjection(
        _direct_wrench_mapping(requested, "X-15 direct-wrench checkpoint requested projection"),
        _direct_wrench_mapping(achieved, "X-15 direct-wrench checkpoint achieved projection"),
        _direct_wrench_mapping(residual, "X-15 direct-wrench checkpoint residual projection"),
        cast(DirectWrenchStatus, status),
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
    "HummingbirdPseudoCompositionEpisode",
    "LanguageBackedCompositionEpisode",
    "X15LocalDirectWrenchCompositionEpisode",
    "VehicleCompositionEpisode",
    "open_vehicle_composition_episode",
]
