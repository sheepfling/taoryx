"""Hummingbird-owned interactive aggregate-thrust composition episode.

The core episode host owns the portable frame, sensor, checkpoint-integrity,
and composition-contract vocabulary.  This plug-in owns the Hummingbird
state, response law, higher-order lowering, and associated readbacks.  The
module is imported only by the Hummingbird episode factory, so installing or
discovering another vehicle does not construct the multirotor episode stack.
"""

from __future__ import annotations

import json
import math
import os
from collections.abc import Mapping
from dataclasses import replace
from pathlib import Path
from typing import Any, Literal

from taoryx.trajectory.hummingbird_pseudo6dof import (
    HummingbirdPseudo6DOFCommand,
    HummingbirdPseudo6DOFModel,
    HummingbirdPseudo6DOFState,
)

from taoryx.composition_episode import (
    ActionFrame,
    EpisodeChannel,
    EpisodeObservation,
    EpisodeStep,
    ObservationFrame,
    StatusFrame,
    _episode_observation_frame,
    _finite_number,
    _interface_contract_for_composition,
    _native_action_for_frame,
    _next_sensor_boundary,
    _payload_digest,
    _semantic_action_from_native,
    _sensor_for_contract,
    _status_frame,
    _three_vector,
)
from taoryx.vehicle_composition import CompiledVehicleComposition
from taoryx.vehicle_interface import project_committed_status_values

EpisodeStatus = Literal["ready", "active", "completed", "closed"]


def _composition_number(values: Mapping[str, Any], name: str, *, default: float) -> float:
    """Read one optional Hummingbird input without importing a private core helper."""

    value = values.get(name)
    return default if value is None else float(value.value)
    ####


def _hummingbird_high_order_episode_channels() -> tuple[EpisodeChannel, ...]:
    """Return held adapter coordinates above the aggregate-thrust seam."""

    return (
        EpisodeChannel("velocity-north-mps", "m/s", -5.0, 5.0, "held local-navigation north-velocity target"),
        EpisodeChannel("velocity-east-mps", "m/s", -5.0, 5.0, "held local-navigation east-velocity target"),
        EpisodeChannel("velocity-vertical-mps", "m/s", -3.0, 3.0, "held positive-up vertical-velocity target"),
        EpisodeChannel("waypoint-north-m", "m", -1_000_000.0, 1_000_000.0, "held live-waypoint north coordinate"),
        EpisodeChannel("waypoint-east-m", "m", -1_000_000.0, 1_000_000.0, "held live-waypoint east coordinate"),
        EpisodeChannel("waypoint-altitude-m", "m", 0.0, 10_000.0, "held live-waypoint altitude coordinate"),
        EpisodeChannel("waypoint-capture-radius-m", "m", 0.05, 1_000.0, "held three-dimensional waypoint capture radius"),
        EpisodeChannel("waypoint-speed-mps", "m/s", 0.0, 5.0, "held horizontal waypoint speed limit"),
        EpisodeChannel("waypoint-vertical-speed-mps", "m/s", 0.0, 3.0, "held waypoint climb and descent speed limit"),
    )
    ####


class HummingbirdPseudoCompositionEpisode:
    """Episode wrapper around the declared Hummingbird aggregate-thrust model."""

    claim_boundary = (
        "This episode uses the named Hummingbird aggregate-thrust-vector pseudo-6DOF response law. "
        "It exposes bounded attitude, thrust, contact, and battery behavior but does not allocate individual "
        "rotors or establish physical motor, aerodynamic-moment, or actuator evidence."
    )
    auto_select_default_authority = True
    _CHECKPOINT_SCHEMA = "taoryx.hummingbird-pseudo-composition-episode/v2alpha1"

    _ACTION_SCHEMA = (
        EpisodeChannel("roll_rad", "rad", -math.pi / 2.0, math.pi / 2.0, "commanded roll response-law angle"),
        EpisodeChannel("pitch_rad", "rad", -math.pi / 2.0, math.pi / 2.0, "commanded pitch response-law angle"),
        EpisodeChannel("yaw_rad", "rad", -math.pi, math.pi, "commanded yaw response-law angle"),
        EpisodeChannel("thrust_ratio", "dimensionless", 0.0, 1.0, "aggregate thrust ratio"),
        EpisodeChannel("motors_enabled", "boolean", description="aggregate rotor enable state"),
    )

    def __init__(
        self,
        composition: CompiledVehicleComposition,
        *,
        seed: int | None = None,
        integration_step_s: float = 0.02,
    ) -> None:
        if composition.family_id != "hummingbird" or composition.fidelity != "pseudo_6dof":
            raise ValueError("Hummingbird pseudo episode requires the hummingbird pseudo_6dof composition")
        if composition.initialization.id not in {"grounded_idle", "airborne_hover"}:
            raise ValueError("Hummingbird pseudo episode requires grounded_idle or airborne_hover initialization")
        if not math.isfinite(integration_step_s) or integration_step_s <= 0.0:
            raise ValueError("Hummingbird episode integration_step_s must be positive and finite")
        self.composition = composition
        self.interface_contract = _interface_contract_for_composition(composition)
        self._authority_action_schema_cache: dict[str, tuple[EpisodeChannel, ...]] = {}
        self.seed = seed
        self.integration_step_s = integration_step_s
        self.model = HummingbirdPseudo6DOFModel(mass_kg=_hummingbird_initial_mass_kg(composition))
        self._closed = False
        self._status: EpisodeStatus = "ready"
        self._state = self._initial_state()
        self._last_action = self._initial_action()
        self._active_authority_profile_id: str | None = None
        self._held_authority_action: dict[str, object] = {}
        self._control_lowering_state: dict[str, object] = {}
        self._sensor = _sensor_for_contract(self.interface_contract, seed=seed)
        self._advance_sensor()
        ####

    @property
    def action_schema(self) -> tuple[EpisodeChannel, ...]:
        """Return direct aggregate-thrust coordinates for compatibility clients."""

        return self._ACTION_SCHEMA
        ####

    def authority_action_schema(self, authority_profile_id: str) -> tuple[EpisodeChannel, ...]:
        """Return the exact native adapter coordinates for one authority."""

        cached = self._authority_action_schema_cache.get(authority_profile_id)
        if cached is not None:
            return cached
        profile = self.interface_contract.authority_profile(authority_profile_id)
        candidates = {
            channel.name: channel
            for channel in (*self.action_schema, *_hummingbird_high_order_episode_channels())
        }
        semantic_channels = {channel.id: channel for channel in self.interface_contract.action_channels}
        resolved: list[EpisodeChannel] = []
        for identifier in profile.action_ids:
            native = semantic_channels[identifier].binding.get("native_action")
            if not isinstance(native, str) or native not in candidates:
                raise ValueError(
                    f"authority profile {authority_profile_id!r} has no Hummingbird episode adapter for {identifier!r}"
                )
            resolved.append(candidates[native])
        schema = tuple(resolved)
        self._authority_action_schema_cache[authority_profile_id] = schema
        return schema
        ####

    @property
    def active_authority_profile_id(self) -> str | None:
        """Return the currently selected external control authority."""

        return self._active_authority_profile_id
        ####

    @property
    def observation_schema(self) -> tuple[EpisodeChannel, ...]:
        """Return the declared available status, resource, and diagnostic channels."""

        channels = (
            *self.interface_contract.status_channels,
            *self.interface_contract.resource_channels,
            *self.interface_contract.diagnostic_channels,
        )
        return tuple(
            EpisodeChannel(
                channel.id,
                channel.canonical_unit,
                channel.lower,
                channel.upper,
                channel.description,
                channel.value_space,
            )
            for channel in channels
            if channel.availability == "available"
        )
        ####

    def reset(self, *, seed: int | None = None) -> EpisodeObservation:
        """Restore the exact declared hover or ground state."""

        self._require_open()
        self.seed = self.seed if seed is None else seed
        selected_profile_id = self._active_authority_profile_id
        self._state = self._initial_state()
        self._last_action = self._initial_action()
        self._active_authority_profile_id = None
        self._held_authority_action = {}
        self._control_lowering_state = {}
        if selected_profile_id is not None:
            self.select_authority_profile(selected_profile_id)
        self._status = "ready"
        self._reset_sensor()
        return self.observe()
        ####

    def observe(self) -> EpisodeObservation:
        """Return the current committed pseudo-plant truth state."""

        self._require_open()
        position = self._state.position_m
        velocity = self._state.velocity_m_s
        waypoint_range_m, waypoint_captured, waypoint_status = self._waypoint_status()
        controller_method = {
            None: "not_applicable",
            "body_motion_response": "attitude_response_law",
            "velocity_yaw_command": "bounded_velocity_response",
            "live_waypoint_guidance": "waypoint_velocity_cascade",
        }[self._active_authority_profile_id]
        raw_values: dict[str, object] = {
            "position_ned_m": [position[0], position[1], -position[2]],
            "velocity_ned_m_s": [velocity[0], velocity[1], -velocity[2]],
            "attitude_rad": list(self._state.attitude_rad),
            "body_rate_rad_s": list(self._state.attitude_rate_rad_s),
            "battery_fraction": self._state.battery_fraction,
            "mass_kg": self.model.mass_kg,
            "aggregate_thrust_n": self._state.thrust_n,
            "aggregate_thrust_fraction": self._state.thrust_n / self.model.maximum_thrust_n,
            "motors_enabled": bool(self._last_action["motors_enabled"]),
            "velocity_horizontal_speed_m_s": math.hypot(velocity[0], velocity[1]),
            "contact": self._state.contact,
            "response_profile_id": self.model.profile_id,
            "control_realization": "aggregate_thrust_vector_surrogate",
            "controller_method": controller_method,
            "physical_motor_allocation": False,
            "control_authority_profile_id": self._active_authority_profile_id or "legacy_native_union",
            "control_lowering": dict(self._control_lowering_state),
            "guidance_waypoint_range_m": waypoint_range_m,
            "guidance_waypoint_captured": waypoint_captured,
            "guidance_waypoint_status": waypoint_status,
        }
        semantic_values = project_committed_status_values(
            self.interface_contract,
            time_s=self._state.time_s,
            execution_status=self._status,
            raw_values=raw_values,
        )
        return EpisodeObservation(self._state.time_s, {**raw_values, **semantic_values}, self._status)
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
        self._last_action = applied
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
        self._status = "active"
        return EpisodeStep(start, self._state.time_s, requested, applied, self.observe(), tuple(events))
        ####

    def step_frame(self, action: ActionFrame) -> EpisodeStep:
        """Lower one selected quadcopter authority into aggregate plant commands."""

        self._require_open()
        previous_profile_id = self._active_authority_profile_id
        self.select_authority_profile(action.authority_profile_id)
        requested_adapter = _native_action_for_frame(self.interface_contract, action)
        requested, held = self._resolve_authority_action(requested_adapter)
        self._held_authority_action = held
        step = self._step_authority_action(requested, held, action.duration_s)
        transition_event = (
            ()
            if previous_profile_id in {None, action.authority_profile_id}
            else (f"authority_profile_changed:{previous_profile_id}->{action.authority_profile_id}",)
        )
        status = self.status_frame()
        observation = _episode_observation_frame(
            status,
            self.interface_contract.observation_profile(self.composition.observation.profile_id),
            self._sensor,
        )
        return replace(
            step,
            events=(*step.events, *transition_event),
            action_frame=action,
            applied_semantic_action=_semantic_action_from_native(self.interface_contract, action, held),
            observation_frame=observation,
            status_frame=status,
        )
        ####

    def select_authority_profile(self, authority_profile_id: str) -> None:
        """Select or transfer between state-continuous quadcopter references."""

        profile = self.interface_contract.authority_profile(authority_profile_id)
        if profile.availability != "available":
            raise ValueError(f"authority profile {authority_profile_id!r} is {profile.availability}, not executable")
        self.authority_action_schema(authority_profile_id)
        previous_id = self._active_authority_profile_id
        if previous_id == authority_profile_id:
            return
        if previous_id is not None:
            previous = self.interface_contract.authority_profile(previous_id)
            if previous.switching_policy != "explicit_bumpless" or profile.switching_policy != "explicit_bumpless":
                raise ValueError(f"authority transfer {previous_id!r} -> {authority_profile_id!r} is not declared")
        self._active_authority_profile_id = authority_profile_id
        self._held_authority_action = self._initial_authority_action(authority_profile_id)
        self._control_lowering_state = {
            "active_profile_id": authority_profile_id,
            "lowering_chain": list(profile.lowering_chain),
            "transfer": "initial_selection" if previous_id is None else "state_continuous_reference_handoff",
        }
        ####

    def control_authority_availability(
        self,
        authority_profile_id: str,
        observation: EpisodeObservation,
    ) -> Mapping[str, object]:
        """Report battery-aware runtime action availability for every profile."""

        profile = self.interface_contract.authority_profile(authority_profile_id)
        battery = observation.values.get("battery_fraction")
        if isinstance(battery, int | float) and not isinstance(battery, bool) and float(battery) <= 1.0e-3:
            reasons = {identifier: ("battery_depleted",) for identifier in profile.action_ids}
            return {
                "runtime_availability": "depleted",
                "reason_codes": ("battery_depleted",),
                "available_action_ids": (),
                "unavailable_action_reasons": reasons,
            }
        return {
            "runtime_availability": "available",
            "reason_codes": (),
            "available_action_ids": profile.action_ids,
            "unavailable_action_reasons": {},
        }
        ####

    def save_checkpoint(self, path: str | Path) -> Path:
        """Persist this bounded pseudo state with an integrity fingerprint."""

        self._require_open()
        payload: dict[str, object] = {
            "schema": self._CHECKPOINT_SCHEMA,
            "composition_identity_sha256": self.composition.identity_sha256,
            "interface_fingerprint_sha256": self.interface_contract.fingerprint,
            "seed": self.seed,
            "integration_step_s": self.integration_step_s,
            "status": self._status,
            "state": _hummingbird_state_payload(self._state),
            "last_action": dict(self._last_action),
            "active_authority_profile_id": self._active_authority_profile_id,
            "held_authority_action": dict(self._held_authority_action),
            "control_lowering_state": dict(self._control_lowering_state),
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
        schema = payload.get("schema")
        if schema not in {self._CHECKPOINT_SCHEMA, "taoryx.hummingbird-pseudo-composition-episode/v1alpha1"}:
            raise ValueError("unsupported Hummingbird composition episode checkpoint schema")
        integrity = payload.pop("integrity", None)
        if integrity != _payload_digest(payload):
            raise ValueError("Hummingbird composition episode checkpoint integrity verification failed")
        if payload.get("composition_identity_sha256") != self.composition.identity_sha256:
            raise ValueError("Hummingbird composition episode checkpoint composition mismatch")
        if schema == self._CHECKPOINT_SCHEMA and payload.get("interface_fingerprint_sha256") != self.interface_contract.fingerprint:
            raise ValueError("Hummingbird composition episode checkpoint interface mismatch")
        self.seed = payload.get("seed") if isinstance(payload.get("seed"), int) else None
        self._state = _hummingbird_state_from_payload(payload["state"])
        self._last_action = _hummingbird_action(payload["last_action"])
        active_authority_profile_id = payload.get("active_authority_profile_id")
        if active_authority_profile_id is not None and not isinstance(active_authority_profile_id, str):
            raise ValueError("Hummingbird checkpoint has invalid authority-profile identity")
        if isinstance(active_authority_profile_id, str):
            profile = self.interface_contract.authority_profile(active_authority_profile_id)
            if profile.availability != "available":
                raise ValueError("Hummingbird checkpoint authority profile is unavailable")
        held = payload.get("held_authority_action", {})
        lowering = payload.get("control_lowering_state", {})
        if not isinstance(held, Mapping) or not isinstance(lowering, Mapping):
            raise ValueError("Hummingbird checkpoint has invalid authority adapter state")
        self._active_authority_profile_id = active_authority_profile_id
        self._held_authority_action = {}
        if isinstance(active_authority_profile_id, str):
            _, self._held_authority_action = self._resolve_authority_action(held, held_action={})
        elif held:
            raise ValueError("Hummingbird checkpoint has held authority values without a selected authority")
        self._control_lowering_state = dict(lowering)
        self._status = _episode_status_literal(payload.get("status"))
        sensor_payload = payload.get("declared_sensor")
        if self._sensor is not None:
            if not isinstance(sensor_payload, Mapping):
                raise ValueError("Hummingbird sensor-configured checkpoint has no declared sensor state")
            self._sensor.restore_checkpoint(sensor_payload)
        elif sensor_payload is not None:
            raise ValueError("Hummingbird checkpoint declares a sensor unavailable in this composition")
        return self.observe()
        ####

    def close(self) -> None:
        """Close the provider-owned state without changing its composition."""

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

    def _resolve_authority_action(
        self,
        action: Mapping[str, object],
        *,
        held_action: Mapping[str, object] | None = None,
    ) -> tuple[dict[str, object], dict[str, object]]:
        profile_id = self._active_authority_profile_id
        if profile_id is None:
            raise ValueError("Hummingbird authority action requires a selected profile")
        schema = self.authority_action_schema(profile_id)
        unknown = sorted(set(action) - {channel.name for channel in schema})
        if unknown:
            raise ValueError("unknown Hummingbird authority action(s): " + ", ".join(unknown))
        base = self._held_authority_action if held_action is None else held_action
        requested = {**base, **dict(action)}
        missing = tuple(channel.name for channel in schema if channel.name not in requested)
        if missing:
            raise ValueError("Hummingbird authority action is missing held coordinate(s): " + ", ".join(missing))
        applied: dict[str, object] = {}
        for channel in schema:
            raw = requested[channel.name]
            if channel.value_space is not None and channel.value_space.topology == "boolean":
                if not isinstance(raw, bool):
                    raise ValueError(f"Hummingbird authority action {channel.name!r} must be boolean")
                applied[channel.name] = raw
                continue
            numeric = _finite_number(raw, f"Hummingbird authority action {channel.name!r}")
            if channel.lower is not None and numeric < channel.lower or channel.upper is not None and numeric > channel.upper:
                raise ValueError(f"Hummingbird authority action {channel.name!r} is outside its declared bounds")
            applied[channel.name] = numeric
        return dict(requested), applied
        ####

    def _step_authority_action(
        self,
        requested: Mapping[str, object],
        held: Mapping[str, object],
        duration_s: float,
    ) -> EpisodeStep:
        """Continuously lower a held high-order target at integration boundaries."""

        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("episode duration_s must be positive and finite")
        start = self._state.time_s
        remaining = duration_s
        events: list[str] = []
        contact_before = self._state.contact
        motors_before = bool(self._last_action["motors_enabled"])
        lowered = dict(self._last_action)
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
            lowered = self._lower_authority_action(held)
            self._state, _ = self.model.step(self._state, self._command_from_action(lowered), step_s)
            self._last_action = lowered
            lowered_thrust_ratio = _finite_number(lowered["thrust_ratio"], "lowered Hummingbird thrust ratio")
            self._control_lowering_state.update(
                {
                    "achieved_thrust_n": self._state.thrust_n,
                    "achieved_thrust_fraction": self._state.thrust_n / self.model.maximum_thrust_n,
                    "battery_fraction": self._state.battery_fraction,
                    "thrust_achievement_limited": (
                        bool(lowered["motors_enabled"])
                        and lowered_thrust_ratio > 0.0
                        and self._state.battery_fraction < 1.0 - 1.0e-12
                    ),
                    "achievement_limit_reason_codes": (
                        ["battery_derate"]
                        if bool(lowered["motors_enabled"])
                        and lowered_thrust_ratio > 0.0
                        and self._state.battery_fraction < 1.0 - 1.0e-12
                        else (["propulsion_disabled"] if not bool(lowered["motors_enabled"]) else [])
                    ),
                }
            )
            remaining -= step_s
            if not contact_before and self._state.contact:
                events.append("contact")
                contact_before = True
            self._advance_sensor()
        if motors_before and not bool(lowered["motors_enabled"]):
            events.append("motor_shutdown")
        self._status = "active"
        return EpisodeStep(start, self._state.time_s, dict(requested), lowered, self.observe(), tuple(events))
        ####

    def _lower_authority_action(self, action: Mapping[str, object]) -> dict[str, object]:
        profile_id = self._active_authority_profile_id
        if profile_id == "body_motion_response":
            lowered = _hummingbird_action(action)
            self._control_lowering_state.update(
                {
                    "active_profile_id": profile_id,
                    "lowering_mode": "direct_attitude_and_aggregate_thrust",
                    "lowered_native_action": dict(lowered),
                    "thrust_command_limited": False,
                }
            )
            return lowered
        if profile_id == "velocity_yaw_command":
            velocity_target = (
                _finite_number(action["velocity-north-mps"], "Hummingbird north-velocity target"),
                _finite_number(action["velocity-east-mps"], "Hummingbird east-velocity target"),
                _finite_number(action["velocity-vertical-mps"], "Hummingbird vertical-velocity target"),
            )
            return self._lower_velocity_target(
                velocity_target,
                yaw_rad=_finite_number(action["yaw_rad"], "Hummingbird yaw target"),
                motors_enabled=bool(action["motors_enabled"]),
                lowering_mode="velocity_yaw_command",
            )
        if profile_id == "live_waypoint_guidance":
            target = (
                _finite_number(action["waypoint-north-m"], "Hummingbird waypoint north"),
                _finite_number(action["waypoint-east-m"], "Hummingbird waypoint east"),
                _finite_number(action["waypoint-altitude-m"], "Hummingbird waypoint altitude"),
            )
            error = tuple(target[index] - self._state.position_m[index] for index in range(3))
            horizontal_range_m = math.hypot(error[0], error[1])
            range_m = math.sqrt(horizontal_range_m * horizontal_range_m + error[2] * error[2])
            capture_radius_m = _finite_number(action["waypoint-capture-radius-m"], "Hummingbird waypoint capture radius")
            captured = range_m <= capture_radius_m
            horizontal_limit_m_s = _finite_number(action["waypoint-speed-mps"], "Hummingbird waypoint speed")
            vertical_limit_m_s = _finite_number(action["waypoint-vertical-speed-mps"], "Hummingbird waypoint vertical speed")
            if captured or horizontal_range_m <= 1.0e-12:
                north_velocity_m_s = 0.0
                east_velocity_m_s = 0.0
            else:
                horizontal_speed_m_s = min(horizontal_limit_m_s, 0.75 * horizontal_range_m)
                north_velocity_m_s = horizontal_speed_m_s * error[0] / horizontal_range_m
                east_velocity_m_s = horizontal_speed_m_s * error[1] / horizontal_range_m
            vertical_velocity_m_s = 0.0 if captured else max(-vertical_limit_m_s, min(vertical_limit_m_s, error[2]))
            lowered = self._lower_velocity_target(
                (north_velocity_m_s, east_velocity_m_s, vertical_velocity_m_s),
                yaw_rad=_finite_number(action["yaw_rad"], "Hummingbird yaw target"),
                motors_enabled=bool(action["motors_enabled"]),
                lowering_mode="live_waypoint_guidance",
                response_gain_s_inv=(1.6, 1.6, 1.8),
                acceleration_limit_m_s2=(3.0, 3.0, 1.0),
            )
            self._control_lowering_state.update(
                {
                    "waypoint_target_m": list(target),
                    "waypoint_capture_radius_m": capture_radius_m,
                    "waypoint_range_m": range_m,
                    "waypoint_captured": captured,
                    "waypoint_status": "captured" if captured else "tracking",
                    "velocity_target_m_s": [north_velocity_m_s, east_velocity_m_s, vertical_velocity_m_s],
                }
            )
            return lowered
        raise ValueError(f"unsupported Hummingbird authority profile {profile_id!r}")
        ####

    def _lower_velocity_target(
        self,
        velocity_target_m_s: tuple[float, float, float],
        *,
        yaw_rad: float,
        motors_enabled: bool,
        lowering_mode: str,
        response_gain_s_inv: tuple[float, float, float] = (2.0, 2.0, 2.0),
        acceleration_limit_m_s2: tuple[float, float, float] = (3.0, 3.0, 1.5),
    ) -> dict[str, object]:
        """Convert a local velocity target into bounded body tilt and thrust."""

        velocity = self._state.velocity_m_s
        acceleration = tuple(
            max(
                -acceleration_limit_m_s2[index],
                min(
                    acceleration_limit_m_s2[index],
                    response_gain_s_inv[index] * (velocity_target_m_s[index] - velocity[index]),
                ),
            )
            for index in range(3)
        )
        force_world = (
            self.model.mass_kg * acceleration[0],
            self.model.mass_kg * acceleration[1],
            self.model.mass_kg * (self.model.gravity_m_s2 + acceleration[2]),
        )
        achieved_yaw = self._state.attitude_rad[2]
        body_x = math.cos(achieved_yaw) * force_world[0] + math.sin(achieved_yaw) * force_world[1]
        body_y = -math.sin(achieved_yaw) * force_world[0] + math.cos(achieved_yaw) * force_world[1]
        body_z = force_world[2]
        thrust_n = math.sqrt(body_x * body_x + body_y * body_y + body_z * body_z)
        roll_rad = math.atan2(-body_y, math.sqrt(body_x * body_x + body_z * body_z))
        pitch_rad = math.atan2(body_x, body_z)
        tilt_limit_rad = math.radians(32.0)
        roll_rad = max(-tilt_limit_rad, min(tilt_limit_rad, roll_rad))
        pitch_rad = max(-tilt_limit_rad, min(tilt_limit_rad, pitch_rad))
        available_battery = max(self._state.battery_fraction, 1.0e-9)
        raw_thrust_fraction = thrust_n / self.model.maximum_thrust_n / available_battery
        thrust_fraction = max(0.0, min(1.0, raw_thrust_fraction)) if motors_enabled else 0.0
        lowered: dict[str, object] = {
            "roll_rad": roll_rad,
            "pitch_rad": pitch_rad,
            "yaw_rad": yaw_rad,
            "thrust_ratio": thrust_fraction,
            "motors_enabled": motors_enabled,
        }
        self._control_lowering_state.update(
            {
                "active_profile_id": self._active_authority_profile_id,
                "lowering_mode": lowering_mode,
                "velocity_target_m_s": list(velocity_target_m_s),
                "velocity_response_gain_s_inv": list(response_gain_s_inv),
                "acceleration_limit_m_s2": list(acceleration_limit_m_s2),
                "desired_acceleration_m_s2": list(acceleration),
                "requested_aggregate_thrust_n": thrust_n,
                "requested_thrust_fraction_before_limit": raw_thrust_fraction,
                "available_battery_fraction": self._state.battery_fraction,
                "thrust_command_limited": raw_thrust_fraction > 1.0 or not motors_enabled,
                "lowered_native_action": dict(lowered),
            }
        )
        return lowered
        ####

    def _initial_authority_action(self, authority_profile_id: str) -> dict[str, object]:
        position = self._state.position_m
        velocity = self._state.velocity_m_s
        yaw_rad = self._state.attitude_rad[2]
        motors_enabled = bool(self._last_action["motors_enabled"])
        if authority_profile_id == "body_motion_response":
            return dict(self._last_action)
        if authority_profile_id == "velocity_yaw_command":
            return {
                "velocity-north-mps": velocity[0],
                "velocity-east-mps": velocity[1],
                "velocity-vertical-mps": velocity[2],
                "yaw_rad": yaw_rad,
                "motors_enabled": motors_enabled,
            }
        if authority_profile_id == "live_waypoint_guidance":
            return {
                "waypoint-north-m": position[0],
                "waypoint-east-m": position[1],
                "waypoint-altitude-m": position[2],
                "waypoint-capture-radius-m": 0.25,
                "waypoint-speed-mps": 2.0,
                "waypoint-vertical-speed-mps": 1.0,
                "yaw_rad": yaw_rad,
                "motors_enabled": motors_enabled,
            }
        raise ValueError(f"unsupported Hummingbird authority profile {authority_profile_id!r}")
        ####

    def _waypoint_status(self) -> tuple[float, bool, str]:
        if self._active_authority_profile_id != "live_waypoint_guidance":
            return 0.0, False, "inactive"
        names = ("waypoint-north-m", "waypoint-east-m", "waypoint-altitude-m", "waypoint-capture-radius-m")
        if any(name not in self._held_authority_action for name in names):
            return 0.0, False, "inactive"
        target = tuple(_finite_number(self._held_authority_action[name], f"held {name}") for name in names[:3])
        radius_m = _finite_number(self._held_authority_action[names[3]], "held waypoint capture radius")
        range_m = math.sqrt(sum((target[index] - self._state.position_m[index]) ** 2 for index in range(3)))
        captured = range_m <= radius_m
        return range_m, captured, "captured" if captured else "tracking"
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


def _hummingbird_initial_mass_kg(composition: CompiledVehicleComposition) -> float:
    """Resolve the one declared mass-bearing Hummingbird reset contract."""

    if composition.initialization.id != "grounded_idle":
        return 0.5
    mass_kg = _composition_number(composition.initialization.inputs, "mass_kg", default=0.5)
    if not math.isfinite(mass_kg) or mass_kg <= 0.0:
        raise ValueError("Hummingbird grounded_idle mass_kg must be positive and finite")
    return mass_kg
    ####


def _hummingbird_action(action: object) -> dict[str, object]:
    """Validate and bound legacy direct aggregate-thrust action coordinates."""

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


def _episode_status_literal(value: object) -> EpisodeStatus:
    if value not in {"ready", "active", "completed", "closed"}:
        raise ValueError("invalid Hummingbird episode checkpoint status")
    return value
    ####


def open_hummingbird_pseudo_composition_episode(
    composition: CompiledVehicleComposition,
    seed: int | None,
    integration_step_s: float,
) -> HummingbirdPseudoCompositionEpisode:
    """Construct the Hummingbird-owned episode through the plug-in factory seam."""

    return HummingbirdPseudoCompositionEpisode(composition, seed=seed, integration_step_s=integration_step_s)
    ####


__all__ = ["HummingbirdPseudoCompositionEpisode", "open_hummingbird_pseudo_composition_episode"]
