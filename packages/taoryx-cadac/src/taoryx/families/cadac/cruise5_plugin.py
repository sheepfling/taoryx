"""Executable CRUISE5 pseudo-6DoF plug-in bound to one installed CADAC source case."""

from __future__ import annotations

import math
from pathlib import Path

from pydantic import Field, model_validator

from .cruise5 import (
    Cruise5RunResult,
    Cruise5SourceDefinition,
    load_cruise5_source_definition,
    run_cruise5_source_compatibility,
)
from .input_ast import CadacModel
from .plugin import CADAC_PLUGIN_CATALOG, CadacVehiclePluginDescriptor


class Cruise5PluginOverrides(CadacModel):
    """Bounded semantic overrides that preserve the installed source mission logic."""

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
    def validate_finite_values(self) -> "Cruise5PluginOverrides":
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
                raise ValueError("CRUISE5 plug-in overrides must contain finite numeric values")
            ####
        ####
        return self

    ####


####


class Cruise5VehiclePlugin:
    """Installed source-compatible CRUISE3/CRUISE5 waypoint-line vehicle plug-in."""

    def __init__(self, source_case_path: str | Path) -> None:
        self._source_case_path = Path(source_case_path)
        self._definition: Cruise5SourceDefinition | None = None

    ####

    @property
    def descriptor(self) -> CadacVehiclePluginDescriptor:
        """Return the canonical actor-level CRUISE5 descriptor."""

        return CADAC_PLUGIN_CATALOG.plugin("cadac.cruise5.cruise_vehicle")

    ####

    @property
    def source_case_path(self) -> Path:
        """Return the installed source-case path without implying source ownership."""

        return self._source_case_path

    ####

    def validate_installation(self) -> tuple[str, ...]:
        """Validate the exact source bundle supported by the current waypoint/line runtime."""

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
            blockers.append("CRUISE5 installed runtime must retain the pseudo_6dof source realization")
        ####
        if definition.control_realization != "response_law":
            blockers.append("CRUISE5 installed runtime must retain response-law control realization")
        ####
        return tuple(blockers)

    ####

    def source_definition(self) -> Cruise5SourceDefinition:
        """Return the immutable installed source definition after validation."""

        blockers = self.validate_installation()
        if blockers:
            raise ValueError("CRUISE5 plug-in installation is incomplete: " + "; ".join(blockers))
        ####
        return self._load_definition()

    ####

    def run_batch(self, overrides: Cruise5PluginOverrides | None = None) -> Cruise5RunResult:
        """Run one source-compatible round-Earth CRUISE5 waypoint/line mission."""

        definition = self.source_definition()
        resolved = overrides or Cruise5PluginOverrides()
        initial_updates: dict[str, object] = {}
        for override_name, field_name in (
            ("longitude_deg", "longitude_deg"),
            ("latitude_deg", "latitude_deg"),
            ("altitude_m", "altitude_m"),
            ("speed_mps", "speed_mps"),
            ("heading_deg", "heading_deg"),
            ("flight_path_deg", "flight_path_deg"),
        ):
            value = getattr(resolved, override_name)
            if value is not None:
                initial_updates[field_name] = value
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
        return run_cruise5_source_compatibility(
            prepared,
            sample_step_s=resolved.sample_step_s,
        )

    ####

    def _load_definition(self) -> Cruise5SourceDefinition:
        if self._definition is None:
            self._definition = load_cruise5_source_definition(self._source_case_path)
        ####
        return self._definition

    ####


####


__all__ = ["Cruise5PluginOverrides", "Cruise5VehiclePlugin"]
