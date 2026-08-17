"""Installed SRAAM6 physical-surface vehicle plug-in."""

from __future__ import annotations

import math
from pathlib import Path

from pydantic import Field, model_validator

from .aim5 import Aim5TargetConfig
from .input_ast import CadacModel
from .plugin import CADAC_PLUGIN_CATALOG, CadacVehiclePluginDescriptor
from .sraam6 import (
    Sraam6InitialState,
    Sraam6RunResult,
    Sraam6SourceDefinition,
    load_sraam6_source_definition,
    run_sraam6_source_compatibility,
)


class Sraam6PluginOverrides(CadacModel):
    """Bounded semantic overrides that leave the installed source artifacts immutable."""

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
    target_option: int | None = None
    target_turn_g: float | None = None
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
    end_time_s: float | None = Field(default=None, gt=0.0)
    sample_step_s: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def validate_finite_values(self) -> "Sraam6PluginOverrides":
        values: list[float] = []
        for vector in (
            self.missile_position_ned_m,
            self.missile_body_rates_deg_s,
            self.target_position_ned_m,
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
            "target_turn_g",
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
            "end_time_s",
            "sample_step_s",
        ):
            value = getattr(self, name)
            if value is not None:
                values.append(float(value))
            ####
        ####
        if any(not math.isfinite(value) for value in values):
            raise ValueError("SRAAM6 plug-in overrides must contain finite numeric values")
        ####
        if self.target_option is not None and self.target_option not in {0, 1}:
            raise ValueError("the first SRAAM6 plug-in runtime supports TARGET3 options 0 and 1")
        ####
        return self

    ####


####


class Sraam6VehiclePlugin:
    """Installed standard-fin SRAAM6 engagement plug-in."""

    def __init__(self, source_case_path: str | Path) -> None:
        self._source_case_path = Path(source_case_path)
        self._definition: Sraam6SourceDefinition | None = None

    ####

    @property
    def descriptor(self) -> CadacVehiclePluginDescriptor:
        """Return canonical actor-level discovery metadata."""

        return CADAC_PLUGIN_CATALOG.plugin("cadac.sraam6.missile")

    ####

    @property
    def source_case_path(self) -> Path:
        return self._source_case_path

    ####

    def validate_installation(self) -> tuple[str, ...]:
        """Validate source resources and the exact executable phase boundary."""

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
            blockers.append("SRAAM6 standard-fin runtime supports actuator modes 0 and 2")
        ####
        if definition.target.aircraft_option not in {0, 1}:
            blockers.append("SRAAM6 standard runtime supports TARGET3 straight or constant-g-turn modes")
        ####
        return tuple(blockers)

    ####

    def source_definition(self) -> Sraam6SourceDefinition:
        """Return the immutable installed source definition after validation."""

        blockers = self.validate_installation()
        if blockers:
            raise ValueError("SRAAM6 plug-in installation is incomplete: " + "; ".join(blockers))
        ####
        return self._load_definition()

    ####

    def run_batch(self, overrides: Sraam6PluginOverrides | None = None) -> Sraam6RunResult:
        """Execute one standard-fin missile/target engagement."""

        resolved = overrides or Sraam6PluginOverrides()
        definition = self.prepare_definition(resolved)
        return run_sraam6_source_compatibility(
            definition,
            sample_step_s=resolved.sample_step_s,
        )

    ####

    def prepare_definition(self, overrides: Sraam6PluginOverrides | None = None) -> Sraam6SourceDefinition:
        """Materialize bounded session-safe overrides into one immutable source definition."""

        definition = self.source_definition()
        resolved = overrides or Sraam6PluginOverrides()
        initial = _override_initial_state(definition.initial_state, resolved)
        target = _override_target(definition.target, resolved)
        updates: dict[str, object] = {"initial_state": initial, "target": target}
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
            updates["seeker"] = definition.seeker.model_copy(update=seeker_updates)
        ####
        if resolved.structural_limit_g is not None:
            updates["control"] = definition.control.model_copy(update={"structural_limit_g": resolved.structural_limit_g})
        ####
        if resolved.end_time_s is not None:
            updates["end_time_s"] = resolved.end_time_s
        ####
        return definition.model_copy(update=updates)
        ####

    ####

    def _load_definition(self) -> Sraam6SourceDefinition:
        if self._definition is None:
            self._definition = load_sraam6_source_definition(self._source_case_path)
        ####
        return self._definition

    ####


####


def _override_initial_state(
    initial: Sraam6InitialState,
    overrides: Sraam6PluginOverrides,
) -> Sraam6InitialState:
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


def _component_updates(
    overrides: Sraam6PluginOverrides,
    routes: dict[str, str],
) -> dict[str, float]:
    """Return explicitly supplied scalar fields for one source component."""

    return {
        component_field: float(value)
        for override_field, component_field in routes.items()
        if (value := getattr(overrides, override_field)) is not None
    }


####


def _override_target(
    target: Aim5TargetConfig,
    overrides: Sraam6PluginOverrides,
) -> Aim5TargetConfig:
    updates: dict[str, object] = {}
    routes = {
        "target_position_ned_m": "position_ned_m",
        "target_speed_mps": "speed_mps",
        "target_heading_deg": "heading_deg",
        "target_flight_path_deg": "flight_path_deg",
        "target_option": "aircraft_option",
        "target_turn_g": "turn_g",
    }
    for override_name, field_name in routes.items():
        value = getattr(overrides, override_name)
        if value is not None:
            updates[field_name] = value
        ####
    ####
    return target.model_copy(update=updates) if updates else target


####


__all__ = ["Sraam6PluginOverrides", "Sraam6VehiclePlugin"]
