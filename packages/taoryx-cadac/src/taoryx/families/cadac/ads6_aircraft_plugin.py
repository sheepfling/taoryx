"""Executable ADS6 AIRCRAFT3 plug-in bound to one installed source case."""

from __future__ import annotations

import math
from pathlib import Path

from pydantic import Field, model_validator

from .ads6_aircraft import (
    Ads6AircraftControlConfig,
    Ads6AircraftGuidanceConfig,
    Ads6AircraftInitialState,
    Ads6AircraftRunResult,
    Ads6AircraftSourceDefinition,
    Ads6AircraftThreatTrack,
    load_ads6_aircraft_source_definition,
    run_ads6_aircraft_source_compatibility,
)
from .input_ast import CadacModel
from .plugin import CADAC_PLUGIN_CATALOG, CadacVehiclePluginDescriptor


class Ads6AircraftPluginOverrides(CadacModel):
    """Bounded source-state, maneuver, response, threat, and runtime overrides."""

    position_ned_m: tuple[float, float, float] | None = None
    speed_mps: float | None = Field(default=None, gt=0.0)
    heading_deg: float | None = None
    flight_path_deg: float | None = None
    guidance_option: int | None = None
    guidance_gain: float | None = Field(default=None, ge=0.0)
    turn_load_g: float | None = None
    maneuver_start_s: float | None = Field(default=None, ge=0.0)
    maneuver_stop_s: float | None = Field(default=None, ge=0.0)
    bank_time_constant_s: float | None = Field(default=None, ge=0.0)
    bank_limit_deg: float | None = Field(default=None, gt=0.0)
    load_factor_time_constant_s: float | None = Field(default=None, ge=0.0)
    alpha_limit_deg: float | None = Field(default=None, gt=0.0)
    lift_slope_per_deg: float | None = Field(default=None, gt=0.0)
    wing_loading_n_m2: float | None = Field(default=None, gt=0.0)
    longitudinal_acceleration_g: float | None = None
    threat_track: Ads6AircraftThreatTrack | None = None
    end_time_s: float | None = Field(default=None, gt=0.0)
    sample_step_s: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def validate_overrides(self) -> "Ads6AircraftPluginOverrides":
        scalars = (
            "speed_mps",
            "heading_deg",
            "flight_path_deg",
            "guidance_gain",
            "turn_load_g",
            "maneuver_start_s",
            "maneuver_stop_s",
            "bank_time_constant_s",
            "bank_limit_deg",
            "load_factor_time_constant_s",
            "alpha_limit_deg",
            "lift_slope_per_deg",
            "wing_loading_n_m2",
            "longitudinal_acceleration_g",
            "end_time_s",
            "sample_step_s",
        )
        for name in scalars:
            value = getattr(self, name)
            if value is not None and not math.isfinite(float(value)):
                raise ValueError("ADS6 AIRCRAFT3 plug-in overrides must contain finite numeric values")
            ####
        ####
        if self.position_ned_m is not None and any(not math.isfinite(float(value)) for value in self.position_ned_m):
            raise ValueError("ADS6 AIRCRAFT3 position override must contain finite values")
        ####
        if self.guidance_option is not None and self.guidance_option not in {0, 1, 2}:
            raise ValueError("ADS6 AIRCRAFT3 guidance_option override must be 0, 1, or 2")
        ####
        return self

    ####


####


class Ads6AircraftVehiclePlugin:
    """Installed source-compatible ADS6 AIRCRAFT3 target plug-in."""

    def __init__(self, source_case_path: str | Path) -> None:
        self._source_case_path = Path(source_case_path)
        self._definition: Ads6AircraftSourceDefinition | None = None

    ####

    @property
    def descriptor(self) -> CadacVehiclePluginDescriptor:
        return CADAC_PLUGIN_CATALOG.plugin("cadac.ads6.aircraft")

    ####

    @property
    def source_case_path(self) -> Path:
        return self._source_case_path

    ####

    def validate_installation(self) -> tuple[str, ...]:
        if not self._source_case_path.is_file():
            return (f"source case does not exist: {self._source_case_path}",)
        ####
        try:
            definition = self._load_definition()
        except (OSError, ValueError) as error:
            return (f"source case could not be lowered: {error}",)
        ####
        blockers: list[str] = []
        if definition.taoryx_tier != "point_mass_3dof":
            blockers.append("ADS6 AIRCRAFT3 runtime must remain point_mass_3dof")
        ####
        if definition.control_realization != "force_model":
            blockers.append("ADS6 AIRCRAFT3 runtime must remain a force-model realization")
        ####
        if definition.source_model != "AIRCRAFT3":
            blockers.append("ADS6 AIRCRAFT3 executable source actor must remain AIRCRAFT3")
        ####
        return tuple(blockers)

    ####

    def source_definition(self) -> Ads6AircraftSourceDefinition:
        blockers = self.validate_installation()
        if blockers:
            raise ValueError("ADS6 AIRCRAFT3 plug-in installation is incomplete: " + "; ".join(blockers))
        ####
        return self._load_definition()

    ####

    def run_batch(self, overrides: Ads6AircraftPluginOverrides | None = None) -> Ads6AircraftRunResult:
        definition = self.source_definition()
        resolved = overrides or Ads6AircraftPluginOverrides()
        initial_updates = {
            name: value for name in ("position_ned_m", "speed_mps", "heading_deg", "flight_path_deg") if (value := getattr(resolved, name)) is not None
        }
        guidance_updates = {
            target: value
            for source, target in (
                ("guidance_option", "option"),
                ("guidance_gain", "guidance_gain"),
                ("turn_load_g", "turn_load_g"),
                ("maneuver_start_s", "maneuver_start_s"),
                ("maneuver_stop_s", "maneuver_stop_s"),
            )
            if (value := getattr(resolved, source)) is not None
        }
        control_updates = {
            name: value
            for name in (
                "bank_time_constant_s",
                "bank_limit_deg",
                "load_factor_time_constant_s",
                "alpha_limit_deg",
                "lift_slope_per_deg",
                "wing_loading_n_m2",
                "longitudinal_acceleration_g",
            )
            if (value := getattr(resolved, name)) is not None
        }
        payload = definition.model_dump()
        if initial_updates:
            payload["initial_state"] = Ads6AircraftInitialState.model_validate({**definition.initial_state.model_dump(), **initial_updates})
        ####
        if guidance_updates:
            payload["guidance"] = Ads6AircraftGuidanceConfig.model_validate({**definition.guidance.model_dump(), **guidance_updates})
        ####
        if control_updates:
            payload["control"] = Ads6AircraftControlConfig.model_validate({**definition.control.model_dump(), **control_updates})
        ####
        prepared = Ads6AircraftSourceDefinition.model_validate(payload)
        return run_ads6_aircraft_source_compatibility(
            prepared,
            threat_track=resolved.threat_track,
            end_time_s=resolved.end_time_s,
            sample_step_s=resolved.sample_step_s,
        )

    ####

    def _load_definition(self) -> Ads6AircraftSourceDefinition:
        if self._definition is None:
            self._definition = load_ads6_aircraft_source_definition(self._source_case_path)
        ####
        return self._definition

    ####


####


__all__ = ["Ads6AircraftPluginOverrides", "Ads6AircraftVehiclePlugin"]
