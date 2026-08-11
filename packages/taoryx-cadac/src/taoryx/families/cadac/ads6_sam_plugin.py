"""Executable ADS6 SAM plug-in bound to one standalone source case."""

from __future__ import annotations

import math
from pathlib import Path

from pydantic import Field, model_validator

from .ads6_sam import (
    Ads6SamControlCommand,
    Ads6SamDirectCommand,
    Ads6SamPhase,
    Ads6SamRunResult,
    Ads6SamSourceDefinition,
    load_ads6_sam_selected_actor_definition,
    load_ads6_sam_source_definition,
    run_ads6_sam_physical_plant,
)
from .input_ast import CadacModel
from .plugin import CADAC_PLUGIN_CATALOG, CadacVehiclePluginDescriptor


class Ads6SamPluginOverrides(CadacModel):
    """Bounded caller overrides for one exact ADS6 SAM realization."""

    source_phase: Ads6SamPhase = "fin_control"
    position_ned_m: tuple[float, float, float] | None = None
    speed_mps: float | None = Field(default=None, gt=0.0)
    yaw_deg: float | None = None
    pitch_deg: float | None = None
    roll_deg: float | None = None
    alpha_deg: float | None = None
    beta_deg: float | None = None
    body_rates_deg_s: tuple[float, float, float] | None = None
    roll_command_deg: float = 0.0
    pitch_command_deg: float = 0.0
    yaw_command_deg: float = 0.0
    tvc_mode: int | None = None
    rcs_moment_mode: int | None = None
    rcs_force_mode: int | None = None
    roll_attitude_command_deg: float | None = None
    pitch_attitude_command_deg: float | None = None
    yaw_attitude_command_deg: float | None = None
    alpha_command_deg: float = 0.0
    beta_command_deg: float = 0.0
    lateral_acceleration_command_g: float = 0.0
    normal_acceleration_command_g: float = 0.0
    thrust_vector_unit_body: tuple[float, float, float] = (1.0, 0.0, 0.0)
    end_time_s: float | None = Field(default=None, gt=0.0)
    sample_step_s: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def validate_finite_values(self) -> "Ads6SamPluginOverrides":
        values: list[float] = []
        for vector in (self.position_ned_m, self.body_rates_deg_s, self.thrust_vector_unit_body):
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
            "roll_command_deg",
            "pitch_command_deg",
            "yaw_command_deg",
            "roll_attitude_command_deg",
            "pitch_attitude_command_deg",
            "yaw_attitude_command_deg",
            "alpha_command_deg",
            "beta_command_deg",
            "lateral_acceleration_command_g",
            "normal_acceleration_command_g",
            "end_time_s",
            "sample_step_s",
        ):
            value = getattr(self, name)
            if value is not None:
                values.append(float(value))
            ####
        ####
        if any(not math.isfinite(value) for value in values):
            raise ValueError("ADS6 SAM plug-in overrides must contain finite numeric values")
        ####
        return self

    ####


####


class Ads6SamVehiclePlugin:
    """Installed ADS6 SAM rigid-body multi-realization plug-in."""

    def __init__(self, source_case_path: str | Path, *, missile_actor_index: int | None = None) -> None:
        self._source_case_path = Path(source_case_path)
        self._missile_actor_index = missile_actor_index
        self._definition: Ads6SamSourceDefinition | None = None

    ####

    @property
    def descriptor(self) -> CadacVehiclePluginDescriptor:
        """Return the canonical actor-level ADS6 SAM descriptor."""

        return CADAC_PLUGIN_CATALOG.plugin("cadac.ads6.sam")

    ####

    @property
    def source_case_path(self) -> Path:
        return self._source_case_path

    ####

    @property
    def missile_actor_index(self) -> int | None:
        """Return the explicit selected package actor, if this is not a standalone case."""

        return self._missile_actor_index

    ####

    def validate_installation(self) -> tuple[str, ...]:
        """Validate source resources and the standalone vehicle boundary."""

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
            blockers.append("ADS6 SAM source case requires positive initial speed")
        ####
        required_realization_modules = {
            "actuator": "physical fin actuator",
            "tvc": "physical thrust-vector-control",
            "rcs": "aggregate reaction-control",
        }
        for module_name, label in required_realization_modules.items():
            if module_name not in definition.module_order:
                blockers.append(f"ADS6 SAM source case must include the {label} module")
            ####
        ####
        return tuple(blockers)

    ####

    def source_definition(self) -> Ads6SamSourceDefinition:
        """Return the immutable installed source definition after validation."""

        blockers = self.validate_installation()
        if blockers:
            raise ValueError("ADS6 SAM plug-in installation is incomplete: " + "; ".join(blockers))
        ####
        return self._load_definition()

    ####

    def validate_realization(self, phase: Ads6SamPhase, overrides: Ads6SamPluginOverrides | None = None) -> tuple[str, ...]:
        """Return phase-specific source/runtime blockers without executing."""

        definition = self.source_definition()
        resolved = overrides or Ads6SamPluginOverrides(source_phase=phase)
        blockers: list[str] = []
        if phase == "fin_control":
            if "actuator" not in definition.module_order:
                blockers.append("fin_control requires the source actuator module")
            ####
        elif phase == "tvc_control":
            mode = definition.tvc.mode if resolved.tvc_mode is None else resolved.tvc_mode
            if "tvc" not in definition.module_order:
                blockers.append("tvc_control requires the source TVC module")
            ####
            if mode == 0:
                blockers.append("tvc_control requires a nonzero source or override TVC mode")
            ####
        elif phase == "aggregate_rcs":
            moment_mode = definition.rcs.moment_mode if resolved.rcs_moment_mode is None else resolved.rcs_moment_mode
            force_mode = definition.rcs.force_mode if resolved.rcs_force_mode is None else resolved.rcs_force_mode
            if "rcs" not in definition.module_order:
                blockers.append("aggregate_rcs requires the source RCS module")
            ####
            if moment_mode == 0 and force_mode == 0:
                blockers.append("aggregate_rcs requires a nonzero moment or force mode")
            ####
        ####
        return tuple(blockers)

    ####

    def run_batch(self, overrides: Ads6SamPluginOverrides | None = None) -> Ads6SamRunResult:
        """Run one exact fin, TVC, or aggregate-RCS realization."""

        definition = self.source_definition()
        resolved = overrides or Ads6SamPluginOverrides()
        phase_blockers = self.validate_realization(resolved.source_phase, resolved)
        if phase_blockers:
            raise ValueError("ADS6 SAM realization is unavailable: " + "; ".join(phase_blockers))
        ####
        updates: dict[str, object] = {}
        for override_name in (
            "position_ned_m",
            "speed_mps",
            "yaw_deg",
            "pitch_deg",
            "roll_deg",
            "alpha_deg",
            "beta_deg",
            "body_rates_deg_s",
        ):
            value = getattr(resolved, override_name)
            if value is not None:
                updates[override_name] = value
            ####
        ####
        if updates:
            definition = definition.model_copy(update={"initial_state": definition.initial_state.model_copy(update=updates)})
        ####
        command = Ads6SamDirectCommand(
            phase=resolved.source_phase,
            control=Ads6SamControlCommand(
                roll_deg=resolved.roll_command_deg,
                pitch_deg=resolved.pitch_command_deg,
                yaw_deg=resolved.yaw_command_deg,
            ),
            tvc_mode=resolved.tvc_mode,
            rcs_moment_mode=resolved.rcs_moment_mode,
            rcs_force_mode=resolved.rcs_force_mode,
            roll_attitude_command_deg=resolved.roll_attitude_command_deg,
            pitch_attitude_command_deg=resolved.pitch_attitude_command_deg,
            yaw_attitude_command_deg=resolved.yaw_attitude_command_deg,
            incidence_commands_deg=(resolved.alpha_command_deg, resolved.beta_command_deg),
            acceleration_commands_g=(
                resolved.lateral_acceleration_command_g,
                resolved.normal_acceleration_command_g,
            ),
            thrust_vector_unit_body=resolved.thrust_vector_unit_body,
        )
        return run_ads6_sam_physical_plant(
            definition,
            command,
            end_time_s=resolved.end_time_s,
            sample_step_s=resolved.sample_step_s,
        )

    ####

    def _load_definition(self) -> Ads6SamSourceDefinition:
        if self._definition is None:
            if self._missile_actor_index is None:
                self._definition = load_ads6_sam_source_definition(self._source_case_path)
            else:
                self._definition = load_ads6_sam_selected_actor_definition(
                    self._source_case_path,
                    missile_actor_index=self._missile_actor_index,
                )
            ####
        ####
        return self._definition

    ####


####


__all__ = ["Ads6SamPluginOverrides", "Ads6SamVehiclePlugin"]
