"""Installed AGM6 physical-surface air-to-ground vehicle plug-in."""

from __future__ import annotations

import math
from pathlib import Path

from pydantic import Field, model_validator

from .agm6 import (
    Agm6AircraftConfig,
    Agm6GroundTargetConfig,
    Agm6InitialState,
    Agm6RunResult,
    Agm6SourceDefinition,
    load_agm6_source_definition,
    run_agm6_source_compatibility,
)
from .input_ast import CadacModel
from .plugin import CADAC_PLUGIN_CATALOG, CadacVehiclePluginDescriptor


class Agm6PluginOverrides(CadacModel):
    """Caller-owned semantic overrides for an immutable AGM6 source installation."""

    missile_position_ned_m: tuple[float, float, float] | None = None
    missile_speed_mps: float | None = Field(default=None, gt=0.0)
    missile_yaw_deg: float | None = None
    missile_pitch_deg: float | None = None
    missile_roll_deg: float | None = None
    missile_alpha_deg: float | None = None
    missile_beta_deg: float | None = None
    missile_body_rates_deg_s: tuple[float, float, float] | None = None
    target_position_ned_m: tuple[float, float, float] | None = None
    target_speed_mps: float | None = Field(default=None, gt=0.0)
    target_heading_deg: float | None = None
    target_flight_path_deg: float | None = None
    target_longitudinal_acceleration_g: float | None = None
    target_lateral_acceleration_g: float | None = None
    aircraft_position_ned_m: tuple[float, float, float] | None = None
    aircraft_speed_mps: float | None = Field(default=None, gt=0.0)
    aircraft_heading_deg: float | None = None
    aircraft_flight_path_deg: float | None = None
    aircraft_option: int | None = None
    aircraft_turn_g: float | None = None
    aircraft_longitudinal_acceleration_g: float | None = None
    navigation_gain: float | None = Field(default=None, ge=0.0)
    fin_position_limit_deg: float | None = Field(default=None, gt=0.0)
    fin_rate_limit_deg_s: float | None = Field(default=None, gt=0.0)
    fin_natural_frequency_rad_s: float | None = Field(default=None, gt=0.0)
    fin_damping_ratio: float | None = Field(default=None, ge=0.0)
    seeker_acquisition_range_m: float | None = Field(default=None, gt=0.0)
    seeker_filter_gain_per_s: float | None = Field(default=None, ge=0.0)
    seeker_filter_natural_frequency_rad_s: float | None = Field(default=None, gt=0.0)
    seeker_filter_damping_ratio: float | None = Field(default=None, ge=0.0)
    structural_limit_g: float | None = Field(default=None, gt=0.0)
    propulsion_throttle: float | None = Field(default=None, ge=0.0)
    random_seed: int | None = None
    end_time_s: float | None = Field(default=None, gt=0.0)
    sample_step_s: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def validate_values(self) -> "Agm6PluginOverrides":
        values: list[float] = []
        for vector in (
            self.missile_position_ned_m,
            self.missile_body_rates_deg_s,
            self.target_position_ned_m,
            self.aircraft_position_ned_m,
        ):
            if vector is not None:
                values.extend(float(value) for value in vector)
            ####
        ####
        for name in (
            "missile_speed_mps",
            "missile_yaw_deg",
            "missile_pitch_deg",
            "missile_roll_deg",
            "missile_alpha_deg",
            "missile_beta_deg",
            "target_speed_mps",
            "target_heading_deg",
            "target_flight_path_deg",
            "target_longitudinal_acceleration_g",
            "target_lateral_acceleration_g",
            "aircraft_speed_mps",
            "aircraft_heading_deg",
            "aircraft_flight_path_deg",
            "aircraft_turn_g",
            "aircraft_longitudinal_acceleration_g",
            "navigation_gain",
            "fin_position_limit_deg",
            "fin_rate_limit_deg_s",
            "fin_natural_frequency_rad_s",
            "fin_damping_ratio",
            "seeker_acquisition_range_m",
            "seeker_filter_gain_per_s",
            "seeker_filter_natural_frequency_rad_s",
            "seeker_filter_damping_ratio",
            "structural_limit_g",
            "propulsion_throttle",
            "end_time_s",
            "sample_step_s",
        ):
            value = getattr(self, name)
            if value is not None:
                values.append(float(value))
            ####
        ####
        if any(not math.isfinite(value) for value in values):
            raise ValueError("AGM6 plug-in overrides must contain finite numeric values")
        ####
        if self.aircraft_option is not None and self.aircraft_option not in {0, 1, 2}:
            raise ValueError("AGM6 AIRCRAFT3 option must be 0, 1, or 2")
        ####
        return self

    ####


####


class Agm6VehiclePlugin:
    """Installed standard-fin AGM6 three-actor engagement plug-in."""

    def __init__(self, source_case_path: str | Path) -> None:
        self._source_case_path = Path(source_case_path)
        self._definition: Agm6SourceDefinition | None = None

    ####

    @property
    def descriptor(self) -> CadacVehiclePluginDescriptor:
        """Return canonical actor-level discovery metadata."""

        return CADAC_PLUGIN_CATALOG.plugin("cadac.agm6.missile")

    ####

    @property
    def source_case_path(self) -> Path:
        return self._source_case_path

    ####

    def validate_installation(self) -> tuple[str, ...]:
        """Validate source artifacts and the executable standard-fin boundary."""

        if not self._source_case_path.is_file():
            return (f"source case does not exist: {self._source_case_path}",)
        ####
        try:
            definition = self._load_definition()
        except (OSError, ValueError) as error:
            return (f"source case could not be lowered: {error}",)
        ####
        blockers: list[str] = []
        if definition.actuator.mode not in {0, 2}:
            blockers.append("AGM6 standard-fin runtime supports actuator modes 0 and 2")
        ####
        if definition.ins_mode_requested not in {0, 1}:
            blockers.append("AGM6 source INS mode must be 0 or 1")
        ####
        if definition.aircraft.aircraft_option not in {0, 1, 2}:
            blockers.append("AGM6 AIRCRAFT3 runtime supports source options 0, 1, and 2")
        ####
        return tuple(blockers)

    ####

    def source_definition(self) -> Agm6SourceDefinition:
        """Return the immutable installed source definition after validation."""

        blockers = self.validate_installation()
        if blockers:
            raise ValueError("AGM6 plug-in installation is incomplete: " + "; ".join(blockers))
        ####
        return self._load_definition()

    ####

    def run_batch(self, overrides: Agm6PluginOverrides | None = None) -> Agm6RunResult:
        """Execute one exact standard-fin missile/target/carrier composition."""

        resolved = overrides or Agm6PluginOverrides()
        definition = self.prepare_definition(resolved)
        return run_agm6_source_compatibility(
            definition,
            sample_step_s=resolved.sample_step_s,
        )

    ####

    def prepare_definition(self, overrides: Agm6PluginOverrides | None = None) -> Agm6SourceDefinition:
        """Materialize caller overrides into an immutable source-session definition."""

        definition = self.source_definition()
        resolved = overrides or Agm6PluginOverrides()
        updates: dict[str, object] = {
            "initial_state": _override_initial_state(definition.initial_state, resolved),
            "target": _override_target(definition.target, resolved),
            "aircraft": _override_aircraft(definition.aircraft, resolved),
        }
        if resolved.navigation_gain is not None:
            updates["guidance"] = definition.guidance.model_copy(update={"navigation_gain": resolved.navigation_gain})
        ####
        actuator_updates = _component_updates(
            resolved,
            {
                "fin_position_limit_deg": "position_limit_deg",
                "fin_rate_limit_deg_s": "rate_limit_deg_s",
                "fin_natural_frequency_rad_s": "natural_frequency_rad_s",
                "fin_damping_ratio": "damping_ratio",
            },
        )
        if actuator_updates:
            updates["actuator"] = definition.actuator.model_copy(update=actuator_updates)
        ####
        seeker_updates = _component_updates(
            resolved,
            {
                "seeker_acquisition_range_m": "acquisition_range_m",
                "seeker_filter_gain_per_s": "filter_gain_per_s",
                "seeker_filter_natural_frequency_rad_s": "filter_natural_frequency_rad_s",
                "seeker_filter_damping_ratio": "filter_damping_ratio",
            },
        )
        if seeker_updates:
            updates["sensor"] = definition.sensor.model_copy(update=seeker_updates)
        ####
        if resolved.structural_limit_g is not None:
            updates["control"] = definition.control.model_copy(update={"structural_limit_g": resolved.structural_limit_g})
        ####
        if resolved.propulsion_throttle is not None:
            updates["propulsion"] = definition.propulsion.model_copy(update={"throttle": resolved.propulsion_throttle})
        ####
        if resolved.random_seed is not None:
            updates["monte_carlo_seed"] = resolved.random_seed
        ####
        if resolved.end_time_s is not None:
            updates["end_time_s"] = resolved.end_time_s
        ####
        return definition.model_copy(update=updates)

    ####

    def _load_definition(self) -> Agm6SourceDefinition:
        if self._definition is None:
            self._definition = load_agm6_source_definition(self._source_case_path)
        ####
        return self._definition

    ####


####


def _component_updates(
    overrides: Agm6PluginOverrides,
    routes: dict[str, str],
) -> dict[str, float]:
    """Return explicitly supplied scalar fields for one source component."""

    return {
        component_field: float(value)
        for override_field, component_field in routes.items()
        if (value := getattr(overrides, override_field)) is not None
    }


####


def _override_initial_state(
    initial: Agm6InitialState,
    overrides: Agm6PluginOverrides,
) -> Agm6InitialState:
    updates: dict[str, object] = {}
    routes = {
        "missile_position_ned_m": "position_ned_m",
        "missile_speed_mps": "speed_mps",
        "missile_yaw_deg": "yaw_deg",
        "missile_pitch_deg": "pitch_deg",
        "missile_roll_deg": "roll_deg",
        "missile_alpha_deg": "alpha_deg",
        "missile_beta_deg": "beta_deg",
        "missile_body_rates_deg_s": "body_rates_deg_s",
    }
    for override_name, field_name in routes.items():
        value = getattr(overrides, override_name)
        if value is not None:
            updates[field_name] = value
        ####
    ####
    return initial.model_copy(update=updates) if updates else initial


####


def _override_target(
    target: Agm6GroundTargetConfig,
    overrides: Agm6PluginOverrides,
) -> Agm6GroundTargetConfig:
    updates: dict[str, object] = {}
    routes = {
        "target_position_ned_m": "position_ned_m",
        "target_speed_mps": "speed_mps",
        "target_heading_deg": "heading_deg",
        "target_flight_path_deg": "flight_path_deg",
        "target_longitudinal_acceleration_g": "longitudinal_acceleration_g",
        "target_lateral_acceleration_g": "lateral_acceleration_g",
    }
    for override_name, field_name in routes.items():
        value = getattr(overrides, override_name)
        if value is not None:
            updates[field_name] = value
        ####
    ####
    return target.model_copy(update=updates) if updates else target


####


def _override_aircraft(
    aircraft: Agm6AircraftConfig,
    overrides: Agm6PluginOverrides,
) -> Agm6AircraftConfig:
    updates: dict[str, object] = {}
    routes = {
        "aircraft_position_ned_m": "position_ned_m",
        "aircraft_speed_mps": "speed_mps",
        "aircraft_heading_deg": "heading_deg",
        "aircraft_flight_path_deg": "flight_path_deg",
        "aircraft_option": "aircraft_option",
        "aircraft_turn_g": "turn_g",
        "aircraft_longitudinal_acceleration_g": "longitudinal_acceleration_g",
    }
    for override_name, field_name in routes.items():
        value = getattr(overrides, override_name)
        if value is not None:
            updates[field_name] = value
        ####
    ####
    return aircraft.model_copy(update=updates) if updates else aircraft


####


__all__ = ["Agm6PluginOverrides", "Agm6VehiclePlugin"]
