"""Executable ADS6 SRBM plug-in bound to one ``ROCKET5`` source case."""

from __future__ import annotations

import math
from pathlib import Path

from pydantic import Field, model_validator

from .ads6_srbm import (
    Ads6SrbmGuidanceConfig,
    Ads6SrbmInitialState,
    Ads6SrbmRunResult,
    Ads6SrbmSourceDefinition,
    load_ads6_srbm_source_definition,
    run_ads6_srbm_source_compatibility,
)
from .input_ast import CadacModel
from .plugin import CADAC_PLUGIN_CATALOG, CadacVehiclePluginDescriptor


class Ads6SrbmPluginOverrides(CadacModel):
    """Bounded initial-state, target, guidance, and runtime overrides."""

    position_ned_m: tuple[float, float, float] | None = None
    speed_mps: float | None = Field(default=None, gt=0.0)
    heading_deg: float | None = None
    flight_path_deg: float | None = None
    alpha_deg: float | None = None
    beta_deg: float | None = None
    target_position_ned_m: tuple[float, float, float] | None = None
    seeker_mode: int | None = None
    guidance_mode: int | None = None
    navigation_gain: float | None = Field(default=None, ge=0.0)
    maneuver_tgo_start_s: float | None = Field(default=None, ge=0.0)
    maneuver_initial_amplitude_g: float | None = Field(default=None, ge=0.0)
    maneuver_frequency_rad_s: float | None = Field(default=None, ge=0.0)
    maneuver_tgo63_s: float | None = Field(default=None, ge=0.0)
    end_time_s: float | None = Field(default=None, gt=0.0)
    sample_step_s: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def validate_finite_values(self) -> "Ads6SrbmPluginOverrides":
        scalar_names = (
            "speed_mps",
            "heading_deg",
            "flight_path_deg",
            "alpha_deg",
            "beta_deg",
            "navigation_gain",
            "maneuver_tgo_start_s",
            "maneuver_initial_amplitude_g",
            "maneuver_frequency_rad_s",
            "maneuver_tgo63_s",
            "end_time_s",
            "sample_step_s",
        )
        for name in scalar_names:
            value = getattr(self, name)
            if value is not None and not math.isfinite(float(value)):
                raise ValueError("ADS6 SRBM plug-in overrides must contain finite numeric values")
            ####
        ####
        for vector in (self.position_ned_m, self.target_position_ned_m):
            if vector is not None and any(not math.isfinite(float(value)) for value in vector):
                raise ValueError("ADS6 SRBM vector overrides must contain finite values")
            ####
        ####
        if self.seeker_mode is not None and self.seeker_mode not in {0, 1}:
            raise ValueError("ADS6 SRBM seeker_mode override must be 0 or 1")
        ####
        if self.guidance_mode is not None:
            maneuver = self.guidance_mode // 10
            navigation = self.guidance_mode % 10
            if maneuver not in {0, 1} or navigation not in {0, 1}:
                raise ValueError("ADS6 SRBM guidance_mode override must encode optional spiral and optional proportional navigation")
            ####
        ####
        return self

    ####


####


class Ads6SrbmVehiclePlugin:
    """Installed source-compatible ADS6 ROCKET5/SRBM5 vehicle plug-in."""

    def __init__(self, source_case_path: str | Path) -> None:
        self._source_case_path = Path(source_case_path)
        self._definition: Ads6SrbmSourceDefinition | None = None

    ####

    @property
    def descriptor(self) -> CadacVehiclePluginDescriptor:
        return CADAC_PLUGIN_CATALOG.plugin("cadac.ads6.srbm")

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
        if definition.taoryx_tier != "pseudo_6dof":
            blockers.append("ADS6 SRBM runtime must remain pseudo_6dof")
        ####
        if definition.control_realization != "response_law":
            blockers.append("ADS6 SRBM runtime must remain a response-law realization")
        ####
        if definition.source_model != "ROCKET5":
            blockers.append("ADS6 SRBM executable source actor must remain ROCKET5")
        ####
        return tuple(blockers)

    ####

    def source_definition(self) -> Ads6SrbmSourceDefinition:
        blockers = self.validate_installation()
        if blockers:
            raise ValueError("ADS6 SRBM plug-in installation is incomplete: " + "; ".join(blockers))
        ####
        return self._load_definition()

    ####

    def run_batch(self, overrides: Ads6SrbmPluginOverrides | None = None) -> Ads6SrbmRunResult:
        resolved = overrides or Ads6SrbmPluginOverrides()
        prepared = self.prepare_definition(resolved)
        return run_ads6_srbm_source_compatibility(
            prepared,
            sample_step_s=resolved.sample_step_s,
        )

    ####

    def prepare_definition(self, overrides: Ads6SrbmPluginOverrides | None = None) -> Ads6SrbmSourceDefinition:
        """Materialize one immutable source definition for batch or persistent stepping."""

        definition = self.source_definition()
        resolved = overrides or Ads6SrbmPluginOverrides()
        initial_updates = {
            name: value
            for name in ("position_ned_m", "speed_mps", "heading_deg", "flight_path_deg", "alpha_deg", "beta_deg")
            if (value := getattr(resolved, name)) is not None
        }
        guidance_updates = {
            name: value
            for name in (
                "target_position_ned_m",
                "seeker_mode",
                "guidance_mode",
                "navigation_gain",
                "maneuver_tgo_start_s",
                "maneuver_initial_amplitude_g",
                "maneuver_frequency_rad_s",
                "maneuver_tgo63_s",
            )
            if (value := getattr(resolved, name)) is not None
        }
        updates: dict[str, object] = {}
        if initial_updates:
            updates["initial_state"] = Ads6SrbmInitialState.model_validate({**definition.initial_state.model_dump(), **initial_updates})
        ####
        if guidance_updates:
            updates["guidance"] = Ads6SrbmGuidanceConfig.model_validate({**definition.guidance.model_dump(), **guidance_updates})
        ####
        if resolved.end_time_s is not None:
            updates["end_time_s"] = resolved.end_time_s
        ####
        return definition.model_copy(update=updates) if updates else definition

    ####

    def _load_definition(self) -> Ads6SrbmSourceDefinition:
        if self._definition is None:
            self._definition = load_ads6_srbm_source_definition(self._source_case_path)
        ####
        return self._definition

    ####


####


__all__ = ["Ads6SrbmPluginOverrides", "Ads6SrbmVehiclePlugin"]
