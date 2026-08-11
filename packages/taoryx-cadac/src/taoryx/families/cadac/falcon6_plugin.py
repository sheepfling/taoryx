"""Executable FALCON6 physical-plant plug-in bound to one installed source case."""

from __future__ import annotations

import math
from pathlib import Path

from pydantic import Field, model_validator

from .falcon6 import (
    Falcon6DirectPlantCommand,
    Falcon6PlantRunResult,
    Falcon6SourceDefinition,
    Falcon6SurfaceCommand,
    load_falcon6_source_definition,
    run_falcon6_physical_plant,
)
from .input_ast import CadacModel
from .plugin import CADAC_PLUGIN_CATALOG, CadacVehiclePluginDescriptor


class Falcon6PluginOverrides(CadacModel):
    """Bounded direct-plant overrides that never mutate source input/decks."""

    position_ned_m: tuple[float, float, float] | None = None
    speed_mps: float | None = Field(default=None, gt=0.0)
    yaw_deg: float | None = None
    pitch_deg: float | None = None
    roll_deg: float | None = None
    alpha_deg: float | None = None
    beta_deg: float | None = None
    body_rates_deg_s: tuple[float, float, float] | None = None
    aileron_command_deg: float = 0.0
    elevator_command_deg: float = 0.0
    rudder_command_deg: float = 0.0
    throttle_override: float | None = Field(default=None, ge=0.0, le=1.0)
    end_time_s: float | None = Field(default=None, gt=0.0)
    sample_step_s: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def validate_finite_values(self) -> "Falcon6PluginOverrides":
        values: list[float] = []
        for vector in (self.position_ned_m, self.body_rates_deg_s):
            if vector is not None:
                values.extend(float(value) for value in vector)
            ####
        ####
        for name in (
            "speed_mps",
            "yaw_deg",
            "pitch_deg",
            "roll_deg",
            "alpha_deg",
            "beta_deg",
            "aileron_command_deg",
            "elevator_command_deg",
            "rudder_command_deg",
            "throttle_override",
            "end_time_s",
            "sample_step_s",
        ):
            value = getattr(self, name)
            if value is not None:
                values.append(float(value))
            ####
        ####
        if any(not math.isfinite(value) for value in values):
            raise ValueError("FALCON6 plug-in overrides must contain finite numeric values")
        ####
        return self

    ####


####


class Falcon6VehiclePlugin:
    """Installed source-grounded FALCON6 rigid-body physical-surface plug-in."""

    def __init__(self, source_case_path: str | Path) -> None:
        self._source_case_path = Path(source_case_path)
        self._definition: Falcon6SourceDefinition | None = None

    ####

    @property
    def descriptor(self) -> CadacVehiclePluginDescriptor:
        """Return the canonical actor-level FALCON6 descriptor."""

        return CADAC_PLUGIN_CATALOG.plugin("cadac.falcon6.aircraft")

    ####

    @property
    def source_case_path(self) -> Path:
        return self._source_case_path

    ####

    def validate_installation(self) -> tuple[str, ...]:
        """Validate source resources and the current direct-plant execution boundary."""

        if not self._source_case_path.is_file():
            return (f"source case does not exist: {self._source_case_path}",)
        ####
        try:
            definition = self._load_definition()
        except (OSError, ValueError) as error:
            return (f"source case could not be lowered: {error}",)
        ####
        blockers: list[str] = []
        if definition.initial_state.speed_mps <= 0.0:
            blockers.append("FALCON6 source case requires positive initial speed")
        ####
        if definition.actuator.mode not in {0, 2}:
            blockers.append("FALCON6 direct plant supports source actuator modes 0 and 2 only")
        ####
        return tuple(blockers)

    ####

    def source_definition(self) -> Falcon6SourceDefinition:
        """Return the immutable installed source definition after validation."""

        blockers = self.validate_installation()
        if blockers:
            raise ValueError("FALCON6 plug-in installation is incomplete: " + "; ".join(blockers))
        ####
        return self._load_definition()

    ####

    def run_batch(self, overrides: Falcon6PluginOverrides | None = None) -> Falcon6PlantRunResult:
        """Run the physical plant with direct surface commands at the source actuator boundary."""

        definition = self.source_definition()
        resolved = overrides or Falcon6PluginOverrides()
        updates: dict[str, object] = {}
        for override_name, field_name in (
            ("position_ned_m", "position_ned_m"),
            ("speed_mps", "speed_mps"),
            ("yaw_deg", "yaw_deg"),
            ("pitch_deg", "pitch_deg"),
            ("roll_deg", "roll_deg"),
            ("alpha_deg", "alpha_deg"),
            ("beta_deg", "beta_deg"),
            ("body_rates_deg_s", "body_rates_deg_s"),
        ):
            value = getattr(resolved, override_name)
            if value is not None:
                updates[field_name] = value
            ####
        ####
        if updates:
            definition = definition.model_copy(update={"initial_state": definition.initial_state.model_copy(update=updates)})
        ####
        command = Falcon6DirectPlantCommand(
            surfaces=Falcon6SurfaceCommand(
                aileron_deg=resolved.aileron_command_deg,
                elevator_deg=resolved.elevator_command_deg,
                rudder_deg=resolved.rudder_command_deg,
            ),
            throttle_override=resolved.throttle_override,
        )
        return run_falcon6_physical_plant(
            definition,
            command,
            end_time_s=resolved.end_time_s,
            sample_step_s=resolved.sample_step_s,
        )

    ####

    def _load_definition(self) -> Falcon6SourceDefinition:
        if self._definition is None:
            self._definition = load_falcon6_source_definition(self._source_case_path)
        ####
        return self._definition

    ####


####


__all__ = ["Falcon6PluginOverrides", "Falcon6VehiclePlugin"]
