"""Executable AIM5 actor plug-in bound to one installed CADAC source case."""

from __future__ import annotations

import math
from pathlib import Path

from pydantic import Field, model_validator

from .aim5_scenario import (
    Aim5ScenarioRunResult,
    Aim5ScenarioSourceDefinition,
    load_aim5_scenario_source_definition,
    run_aim5_scenario_source_compatibility,
)
from .input_ast import CadacModel
from .plugin import CADAC_PLUGIN_CATALOG, CadacVehiclePluginDescriptor


class Aim5PluginOverrides(CadacModel):
    """Bounded run-time overrides applied without mutating the installed source case."""

    missile_position_ned_m: tuple[float, float, float] | None = None
    missile_speed_mps: float | None = Field(default=None, gt=0.0)
    missile_heading_deg: float | None = None
    missile_flight_path_deg: float | None = None
    missile_alpha_deg: float | None = None
    missile_beta_deg: float | None = None
    target_position_ned_m: tuple[float, float, float] | None = None
    target_speed_mps: float | None = Field(default=None, gt=0.0)
    target_heading_deg: float | None = None
    target_flight_path_deg: float | None = None
    navigation_gain: float | None = Field(default=None, ge=0.0)
    target_aircraft_option: int | None = None
    target_turn_g: float | None = None
    end_time_s: float | None = Field(default=None, gt=0.0)
    sample_step_s: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def validate_finite_values(self) -> "Aim5PluginOverrides":
        values: list[float] = []
        for vector in (self.missile_position_ned_m, self.target_position_ned_m):
            if vector is not None:
                values.extend(vector)
            ####
        ####
        for name in (
            "missile_speed_mps",
            "missile_heading_deg",
            "missile_flight_path_deg",
            "missile_alpha_deg",
            "missile_beta_deg",
            "target_speed_mps",
            "target_heading_deg",
            "target_flight_path_deg",
            "navigation_gain",
            "target_turn_g",
            "end_time_s",
            "sample_step_s",
        ):
            value = getattr(self, name)
            if value is not None:
                values.append(float(value))
            ####
        ####
        if any(not math.isfinite(value) for value in values):
            raise ValueError("AIM5 plug-in overrides must contain finite numeric values")
        ####
        return self

    ####


####


class Aim5VehiclePlugin:
    """Installed source-compatible AIM5 missile plug-in for Taoryx discovery/execution."""

    def __init__(self, source_case_path: str | Path) -> None:
        self._source_case_path = Path(source_case_path)
        self._definition: Aim5ScenarioSourceDefinition | None = None

    ####

    @property
    def descriptor(self) -> CadacVehiclePluginDescriptor:
        """Return the canonical actor-level plug-in descriptor."""

        return CADAC_PLUGIN_CATALOG.plugin("cadac.aim5.missile")

    ####

    @property
    def source_case_path(self) -> Path:
        """Return the installed source-case path without implying source ownership."""

        return self._source_case_path

    ####

    def validate_installation(self) -> tuple[str, ...]:
        """Validate exact AIM5 source resources and the one-missile plug-in boundary."""

        blockers: list[str] = []
        if not self._source_case_path.is_file():
            return (f"source case does not exist: {self._source_case_path}",)
        ####
        try:
            definition = self._load_definition()
        except (OSError, ValueError) as error:
            return (f"source case could not be lowered: {error}",)
        ####
        if len(definition.missiles) != 1:
            blockers.append("AIM5 vehicle plug-in currently requires exactly one AIM5 missile actor")
        ####
        if len(definition.targets) != 1:
            blockers.append("AIM5 vehicle plug-in currently requires exactly one AIRCRAFT3 target actor")
        ####
        if definition.missiles and definition.missiles[0].config.target_number != 1:
            blockers.append("single-engagement AIM5 plug-in requires target_number=1")
        ####
        return tuple(blockers)

    ####

    def source_definition(self) -> Aim5ScenarioSourceDefinition:
        """Return the immutable installed source definition after validation."""

        blockers = self.validate_installation()
        if blockers:
            raise ValueError("AIM5 plug-in installation is incomplete: " + "; ".join(blockers))
        ####
        return self._load_definition()

    ####

    def run_batch(self, overrides: Aim5PluginOverrides | None = None) -> Aim5ScenarioRunResult:
        """Run one source-compatible AIM5 engagement through bounded semantic overrides."""

        resolved = overrides or Aim5PluginOverrides()
        prepared = self.prepare_definition(resolved)
        return run_aim5_scenario_source_compatibility(
            prepared,
            sample_step_s=resolved.sample_step_s,
        )

    ####

    def prepare_definition(self, overrides: Aim5PluginOverrides | None = None) -> Aim5ScenarioSourceDefinition:
        """Materialize an immutable source definition for batch or persistent execution."""

        return _apply_overrides(self.source_definition(), overrides or Aim5PluginOverrides())
        ####

    def _load_definition(self) -> Aim5ScenarioSourceDefinition:
        if self._definition is None:
            self._definition = load_aim5_scenario_source_definition(self._source_case_path)
        ####
        return self._definition

    ####


####


def _apply_overrides(
    definition: Aim5ScenarioSourceDefinition,
    overrides: Aim5PluginOverrides,
) -> Aim5ScenarioSourceDefinition:
    """Return a copied source definition with only declared semantic overrides changed."""

    missile_actor = definition.missiles[0]
    target_actor = definition.targets[0]
    missile_updates: dict[str, object] = {}
    target_updates: dict[str, object] = {}
    for override_name, target_name in (
        ("missile_position_ned_m", "position_ned_m"),
        ("missile_speed_mps", "speed_mps"),
        ("missile_heading_deg", "heading_deg"),
        ("missile_flight_path_deg", "flight_path_deg"),
        ("missile_alpha_deg", "alpha_deg"),
        ("missile_beta_deg", "beta_deg"),
        ("navigation_gain", "navigation_gain"),
    ):
        value = getattr(overrides, override_name)
        if value is not None:
            missile_updates[target_name] = value
        ####
    ####
    for override_name, target_name in (
        ("target_position_ned_m", "position_ned_m"),
        ("target_speed_mps", "speed_mps"),
        ("target_heading_deg", "heading_deg"),
        ("target_flight_path_deg", "flight_path_deg"),
        ("target_aircraft_option", "aircraft_option"),
        ("target_turn_g", "turn_g"),
    ):
        value = getattr(overrides, override_name)
        if value is not None:
            target_updates[target_name] = value
        ####
    ####
    missile = missile_actor.model_copy(update={"config": missile_actor.config.model_copy(update=missile_updates)})
    target = target_actor.model_copy(update={"config": target_actor.config.model_copy(update=target_updates)})
    update: dict[str, object] = {
        "missiles": (missile,),
        "targets": (target,),
    }
    if overrides.end_time_s is not None:
        update["end_time_s"] = overrides.end_time_s
    ####
    return definition.model_copy(update=update)


####


__all__ = [
    "Aim5PluginOverrides",
    "Aim5VehiclePlugin",
]
