"""Executable ROCKET6G phase-aware vehicle plug-in bound to one source case."""

from __future__ import annotations

import math
from pathlib import Path

from pydantic import Field, model_validator

from .input_ast import CadacModel
from .plugin import CADAC_PLUGIN_CATALOG, CadacVehiclePluginDescriptor
from .rocket6g import (
    Rocket6gDirectCommand,
    Rocket6gPlantRunResult,
    Rocket6gSourceDefinition,
    load_rocket6g_source_definition,
    run_rocket6g_phase_aware_plant,
)


class Rocket6gPluginOverrides(CadacModel):
    """Bounded launch/command overrides that do not mutate source files or decks."""

    longitude_deg: float | None = None
    latitude_deg: float | None = None
    altitude_m: float | None = None
    geographic_speed_mps: float | None = Field(default=None, gt=0.0)
    roll_deg: float | None = None
    pitch_deg: float | None = None
    yaw_deg: float | None = None
    alpha_deg: float | None = None
    beta_deg: float | None = None
    body_rates_deg_s: tuple[float, float, float] | None = None
    tvc_pitch_command_deg: float = 0.0
    tvc_yaw_command_deg: float = 0.0
    thrust_vector_unit_body: tuple[float, float, float] = (1.0, 0.0, 0.0)
    roll_command_deg: float | None = None
    pitch_command_deg: float | None = None
    yaw_command_deg: float | None = None
    boost_cutoff_time_s: float | None = Field(default=None, gt=0.0)
    end_time_s: float | None = Field(default=None, gt=0.0)
    sample_step_s: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def validate_finite_values(self) -> "Rocket6gPluginOverrides":
        values: list[float] = []
        for vector in (self.body_rates_deg_s, self.thrust_vector_unit_body):
            if vector is not None:
                values.extend(float(value) for value in vector)
            ####
        ####
        for name in (
            "longitude_deg",
            "latitude_deg",
            "altitude_m",
            "geographic_speed_mps",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "alpha_deg",
            "beta_deg",
            "tvc_pitch_command_deg",
            "tvc_yaw_command_deg",
            "roll_command_deg",
            "pitch_command_deg",
            "yaw_command_deg",
            "boost_cutoff_time_s",
            "end_time_s",
            "sample_step_s",
        ):
            value = getattr(self, name)
            if value is not None:
                values.append(float(value))
            ####
        ####
        if any(not math.isfinite(value) for value in values):
            raise ValueError("ROCKET6G plug-in overrides must be finite")
        ####
        if not -90.0 <= (self.latitude_deg if self.latitude_deg is not None else 0.0) <= 90.0:
            raise ValueError("ROCKET6G latitude override must be within [-90, 90] deg")
        ####
        magnitude = math.sqrt(sum(value * value for value in self.thrust_vector_unit_body))
        if magnitude <= 0.0:
            raise ValueError("ROCKET6G thrust-vector override must have positive magnitude")
        ####
        return self

    ####


####


class Rocket6gVehiclePlugin:
    """Installed exact-source ROCKET6G phase-program batch plug-in."""

    def __init__(self, source_path: str | Path) -> None:
        self.source_path = Path(source_path).resolve()
        self.source_definition = load_rocket6g_source_definition(self.source_path)
        self._descriptor = CADAC_PLUGIN_CATALOG.plugin("cadac.rocket6g.launch_vehicle")
        blockers = self.validate_installation()
        if blockers:
            raise ValueError("ROCKET6G plug-in installation is invalid: " + "; ".join(blockers))
        ####

    ####

    @property
    def descriptor(self) -> CadacVehiclePluginDescriptor:
        """Return immutable actor-level discovery metadata."""

        return self._descriptor

    ####

    def validate_installation(self) -> tuple[str, ...]:
        """Check the exact source binding without executing a mission."""

        errors: list[str] = []
        if self.descriptor.status != "runnable":
            errors.append("catalog descriptor does not mark ROCKET6G runnable")
        ####
        if self.descriptor.batch_factory_id != "cadac.rocket6g.phase_aware.batch":
            errors.append("catalog ROCKET6G batch factory does not match the installed implementation")
        ####
        if len(self.source_definition.stages) != 3:
            errors.append("source definition does not contain three stages")
        ####
        if not self.source_definition.events:
            errors.append("source definition does not contain the phase/event program")
        ####
        return tuple(errors)

    ####

    def run_batch(self, overrides: Rocket6gPluginOverrides | None = None) -> Rocket6gPlantRunResult:
        """Execute one source program with bounded launch and direct-command overrides."""

        selected = overrides or Rocket6gPluginOverrides()
        definition = _definition_with_overrides(self.source_definition, selected)
        command = Rocket6gDirectCommand(
            tvc_pitch_command_deg=selected.tvc_pitch_command_deg,
            tvc_yaw_command_deg=selected.tvc_yaw_command_deg,
            thrust_vector_unit_body=selected.thrust_vector_unit_body,
            roll_command_deg=selected.roll_command_deg,
            pitch_command_deg=selected.pitch_command_deg,
            yaw_command_deg=selected.yaw_command_deg,
            boost_cutoff_time_s=selected.boost_cutoff_time_s,
        )
        return run_rocket6g_phase_aware_plant(
            definition,
            command,
            end_time_s=selected.end_time_s,
            sample_step_s=selected.sample_step_s,
        )

    ####


####


def _definition_with_overrides(
    definition: Rocket6gSourceDefinition,
    overrides: Rocket6gPluginOverrides,
) -> Rocket6gSourceDefinition:
    initial = definition.initial_state
    updates: dict[str, object] = {}
    for field_name in (
        "longitude_deg",
        "latitude_deg",
        "altitude_m",
        "geographic_speed_mps",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "alpha_deg",
        "beta_deg",
        "body_rates_deg_s",
    ):
        value = getattr(overrides, field_name)
        if value is not None:
            updates[field_name] = value
        ####
    ####
    if not updates:
        return definition
    ####
    return definition.model_copy(update={"initial_state": initial.model_copy(update=updates)})


####


__all__ = ["Rocket6gPluginOverrides", "Rocket6gVehiclePlugin"]
