"""Executable MAGSIX trajectory-only plug-in bound to one installed CADAC source case."""

from __future__ import annotations

import math
from pathlib import Path

from pydantic import Field, model_validator

from .input_ast import CadacModel
from .magsix import MagsixRunResult, MagsixSourceDefinition, load_magsix_source_definition, run_magsix_trajectory_source_compatibility
from .plugin import CADAC_PLUGIN_CATALOG, CadacVehiclePluginDescriptor


class MagsixPluginOverrides(CadacModel):
    """Bounded trajectory initial-state/runtime overrides over an installed MAGSIX source case."""

    north_m: float | None = None
    east_m: float | None = None
    altitude_m: float | None = None
    speed_mps: float | None = Field(default=None, gt=0.0)
    heading_deg: float | None = None
    flight_path_deg: float | None = None
    spin_rpm: float | None = None
    end_time_dnt: float | None = Field(default=None, gt=0.0)
    sample_step_dnt: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def validate_finite_values(self) -> "MagsixPluginOverrides":
        for name in (
            "north_m",
            "east_m",
            "altitude_m",
            "speed_mps",
            "heading_deg",
            "flight_path_deg",
            "spin_rpm",
            "end_time_dnt",
            "sample_step_dnt",
        ):
            value = getattr(self, name)
            if value is not None and not math.isfinite(float(value)):
                raise ValueError("MAGSIX plug-in overrides must contain finite numeric values")
            ####
        ####
        return self

    ####


####


class MagsixVehiclePlugin:
    """Installed source-compatible MAGSIX planar trajectory plug-in."""

    def __init__(self, source_case_path: str | Path) -> None:
        self._source_case_path = Path(source_case_path)
        self._definition: MagsixSourceDefinition | None = None

    ####

    @property
    def descriptor(self) -> CadacVehiclePluginDescriptor:
        return CADAC_PLUGIN_CATALOG.plugin("cadac.magsix.vehicle")

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
            blockers.append("MAGSIX trajectory-only runtime must remain point_mass_3dof")
        ####
        if definition.control_realization != "force_model":
            blockers.append("MAGSIX trajectory-only runtime must remain a force-model realization")
        ####
        return tuple(blockers)

    ####

    def source_definition(self) -> MagsixSourceDefinition:
        blockers = self.validate_installation()
        if blockers:
            raise ValueError("MAGSIX plug-in installation is incomplete: " + "; ".join(blockers))
        ####
        return self._load_definition()

    ####

    def run_batch(self, overrides: MagsixPluginOverrides | None = None) -> MagsixRunResult:
        definition = self.source_definition()
        resolved = overrides or MagsixPluginOverrides()
        initial_updates: dict[str, object] = {}
        for name in (
            "north_m",
            "east_m",
            "altitude_m",
            "speed_mps",
            "heading_deg",
            "flight_path_deg",
            "spin_rpm",
        ):
            value = getattr(resolved, name)
            if value is not None:
                initial_updates[name] = value
            ####
        ####
        updates: dict[str, object] = {}
        if initial_updates:
            updates["initial_state"] = definition.initial_state.model_copy(update=initial_updates)
        ####
        if resolved.end_time_dnt is not None:
            updates["end_time_dnt"] = resolved.end_time_dnt
        ####
        prepared = definition.model_copy(update=updates) if updates else definition
        return run_magsix_trajectory_source_compatibility(prepared, sample_step_dnt=resolved.sample_step_dnt)

    ####

    def _load_definition(self) -> MagsixSourceDefinition:
        if self._definition is None:
            self._definition = load_magsix_source_definition(self._source_case_path)
        ####
        return self._definition

    ####


####


__all__ = ["MagsixPluginOverrides", "MagsixVehiclePlugin"]
