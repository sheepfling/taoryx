"""Executable GHAME6 phase-aware vehicle plug-in bound to one source case."""

from __future__ import annotations

import math
from pathlib import Path

from pydantic import Field, model_validator

from .ghame6 import (
    Ghame6DirectCommand,
    Ghame6RunResult,
    Ghame6SourceDefinition,
    load_ghame6_source_definition,
    run_ghame6_phase_aware_mission,
)
from .input_ast import CadacModel
from .plugin import CADAC_PLUGIN_CATALOG, CadacVehiclePluginDescriptor


class Ghame6PluginOverrides(CadacModel):
    """Bounded launch, direct-command, and runtime overrides."""

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
    aileron_command_deg: float = 0.0
    elevator_command_deg: float = 0.0
    rudder_command_deg: float = 0.0
    thrust_vector_unit_body: tuple[float, float, float] = (1.0, 0.0, 0.0)
    roll_command_deg: float | None = None
    pitch_command_deg: float | None = None
    yaw_command_deg: float | None = None
    alpha_command_deg: float = 0.0
    beta_command_deg: float = 0.0
    lateral_acceleration_command_g: float = 0.0
    normal_acceleration_command_g: float = 0.0
    boost_cutoff_time_s: float | None = Field(default=None, gt=0.0)
    terminal_lock_time_s: float | None = Field(default=None, gt=0.0)
    end_time_s: float | None = Field(default=None, gt=0.0)
    sample_step_s: float | None = Field(default=None, gt=0.0)
    random_seed: int = Field(default=12345, ge=0)

    @model_validator(mode="after")
    def validate_finite_values(self) -> "Ghame6PluginOverrides":
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
            "aileron_command_deg",
            "elevator_command_deg",
            "rudder_command_deg",
            "roll_command_deg",
            "pitch_command_deg",
            "yaw_command_deg",
            "alpha_command_deg",
            "beta_command_deg",
            "lateral_acceleration_command_g",
            "normal_acceleration_command_g",
            "boost_cutoff_time_s",
            "terminal_lock_time_s",
            "end_time_s",
            "sample_step_s",
        ):
            value = getattr(self, name)
            if value is not None:
                values.append(float(value))
            ####
        ####
        if any(not math.isfinite(value) for value in values):
            raise ValueError("GHAME6 plug-in overrides must be finite")
        ####
        if self.latitude_deg is not None and not -90.0 <= self.latitude_deg <= 90.0:
            raise ValueError("GHAME6 latitude override must be within [-90, 90] deg")
        ####
        magnitude = math.sqrt(sum(float(value) * float(value) for value in self.thrust_vector_unit_body))
        if magnitude <= 0.0:
            raise ValueError("GHAME6 thrust-vector override must have positive magnitude")
        ####
        return self

    ####


####


class Ghame6VehiclePlugin:
    """Installed exact-source GHAME6 phase-program batch plug-in."""

    def __init__(self, source_path: str | Path) -> None:
        self.source_path = Path(source_path).resolve()
        self.source_definition = load_ghame6_source_definition(self.source_path)
        self._descriptor = CADAC_PLUGIN_CATALOG.plugin("cadac.ghame6.hypersonic_vehicle")
        blockers = self.validate_installation()
        if blockers:
            raise ValueError("GHAME6 plug-in installation is invalid: " + "; ".join(blockers))
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
            errors.append("catalog descriptor does not mark GHAME6 runnable")
        ####
        if self.descriptor.batch_factory_id != "cadac.ghame6.phase_aware.batch":
            errors.append("catalog GHAME6 batch factory does not match the installed implementation")
        ####
        if self.source_definition.actor_order != ("HYPER6", "SAT3", "RADAR0"):
            errors.append("source definition does not preserve HYPER6, SAT3, RADAR0 actor order")
        ####
        if "tvc" in self.source_definition.module_order:
            errors.append("GHAME6 source definition incorrectly advertises a TVC module")
        ####
        return tuple(errors)

    ####

    def run_batch(self, overrides: Ghame6PluginOverrides | None = None) -> Ghame6RunResult:
        """Execute one source program with bounded launch and direct-command overrides."""

        selected = overrides or Ghame6PluginOverrides()
        definition = _definition_with_overrides(self.source_definition, selected)
        command = Ghame6DirectCommand(
            aileron_command_deg=selected.aileron_command_deg,
            elevator_command_deg=selected.elevator_command_deg,
            rudder_command_deg=selected.rudder_command_deg,
            thrust_vector_unit_body=selected.thrust_vector_unit_body,
            roll_command_deg=selected.roll_command_deg,
            pitch_command_deg=selected.pitch_command_deg,
            yaw_command_deg=selected.yaw_command_deg,
            alpha_command_deg=selected.alpha_command_deg,
            beta_command_deg=selected.beta_command_deg,
            lateral_acceleration_command_g=selected.lateral_acceleration_command_g,
            normal_acceleration_command_g=selected.normal_acceleration_command_g,
            boost_cutoff_time_s=selected.boost_cutoff_time_s,
            terminal_lock_time_s=selected.terminal_lock_time_s,
        )
        return run_ghame6_phase_aware_mission(
            definition,
            command,
            end_time_s=selected.end_time_s,
            sample_step_s=selected.sample_step_s,
            random_seed=selected.random_seed,
        )

    ####


####


def _definition_with_overrides(
    definition: Ghame6SourceDefinition,
    overrides: Ghame6PluginOverrides,
) -> Ghame6SourceDefinition:
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


__all__ = ["Ghame6PluginOverrides", "Ghame6VehiclePlugin"]
