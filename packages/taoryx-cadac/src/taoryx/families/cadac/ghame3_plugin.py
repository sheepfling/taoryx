"""Executable GHAME3 point-mass plug-in bound to one installed CADAC source case."""

from __future__ import annotations

import math
from pathlib import Path

from pydantic import Field, model_validator

from .ghame3 import Ghame3RunResult, Ghame3SourceDefinition, load_ghame3_source_definition, run_ghame3_source_compatibility
from .input_ast import CadacModel
from .plugin import CADAC_PLUGIN_CATALOG, CadacVehiclePluginDescriptor


class Ghame3PluginOverrides(CadacModel):
    """Bounded initial-state/runtime overrides over the installed source mission."""

    longitude_deg: float | None = None
    latitude_deg: float | None = None
    altitude_m: float | None = None
    speed_mps: float | None = Field(default=None, gt=0.0)
    heading_deg: float | None = None
    flight_path_deg: float | None = None
    alpha_deg: float | None = None
    bank_deg: float | None = None
    end_time_s: float | None = Field(default=None, gt=0.0)
    sample_step_s: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def validate_finite_values(self) -> "Ghame3PluginOverrides":
        for name in (
            "longitude_deg",
            "latitude_deg",
            "altitude_m",
            "speed_mps",
            "heading_deg",
            "flight_path_deg",
            "alpha_deg",
            "bank_deg",
            "end_time_s",
            "sample_step_s",
        ):
            value = getattr(self, name)
            if value is not None and not math.isfinite(float(value)):
                raise ValueError("GHAME3 plug-in overrides must contain finite numeric values")
            ####
        ####
        return self

    ####


####


class Ghame3VehiclePlugin:
    """Installed source-compatible GHAME3 Round3 point-mass vehicle plug-in."""

    def __init__(self, source_case_path: str | Path) -> None:
        self._source_case_path = Path(source_case_path)
        self._definition: Ghame3SourceDefinition | None = None

    ####

    @property
    def descriptor(self) -> CadacVehiclePluginDescriptor:
        return CADAC_PLUGIN_CATALOG.plugin("cadac.ghame3.hypersonic_vehicle")

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
            blockers.append("GHAME3 must remain a point_mass_3dof source realization")
        ####
        if definition.control_realization != "force_model":
            blockers.append("GHAME3 source realization must remain a force-model point-mass plant")
        ####
        return tuple(blockers)

    ####

    def source_definition(self) -> Ghame3SourceDefinition:
        blockers = self.validate_installation()
        if blockers:
            raise ValueError("GHAME3 plug-in installation is incomplete: " + "; ".join(blockers))
        ####
        return self._load_definition()

    ####

    def run_batch(self, overrides: Ghame3PluginOverrides | None = None) -> Ghame3RunResult:
        definition = self.source_definition()
        resolved = overrides or Ghame3PluginOverrides()
        initial_updates: dict[str, object] = {}
        for name in ("longitude_deg", "latitude_deg", "altitude_m", "speed_mps", "heading_deg", "flight_path_deg"):
            value = getattr(resolved, name)
            if value is not None:
                initial_updates[name] = value
            ####
        ####
        updates: dict[str, object] = {}
        if initial_updates:
            updates["initial_state"] = definition.initial_state.model_copy(update=initial_updates)
        ####
        if resolved.alpha_deg is not None:
            updates["initial_alpha_deg"] = resolved.alpha_deg
        ####
        if resolved.bank_deg is not None:
            updates["initial_bank_deg"] = resolved.bank_deg
        ####
        if resolved.end_time_s is not None:
            updates["end_time_s"] = resolved.end_time_s
        ####
        prepared = definition.model_copy(update=updates) if updates else definition
        return run_ghame3_source_compatibility(prepared, sample_step_s=resolved.sample_step_s)

    ####

    def _load_definition(self) -> Ghame3SourceDefinition:
        if self._definition is None:
            self._definition = load_ghame3_source_definition(self._source_case_path)
        ####
        return self._definition

    ####


####


__all__ = ["Ghame3PluginOverrides", "Ghame3VehiclePlugin"]
