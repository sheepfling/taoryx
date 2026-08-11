"""Executable source-compatibility vertical slice for the CADAC AIM5 case."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal, TypeAlias

import numpy as np
from numpy.typing import NDArray
from pydantic import Field, model_validator

from .bundle import CadacSourceArtifact, CadacSourceBundle, load_cadac_source_bundle
from .compatibility import cadac_stored_derivative_step
from .deck import CadacDeck
from .events import CadacEventApplication, CadacEventCursor, CadacRuntimeScalar
from .input_ast import CadacDeckKind, CadacEventBlock, CadacModel, CadacModuleStage, CadacVehicleBlock
from .sensor_adapter import cadac_local_ned_relative_state_track
from .source_environment import atmosphere76, cadac_source_inverse_square_gravity_mps2

FloatVector: TypeAlias = NDArray[np.float64]
FloatMatrix: TypeAlias = NDArray[np.float64]

_R_AIR_J_PER_KG_K = 287.053
_RAD_PER_DEG = 0.0174532925199432
_DEG_PER_RAD = 57.2957795130823
_SMALL = 1.0e-7

_AIM5_MODULES = (
    "environment",
    "kinematics",
    "aerodynamics",
    "propulsion",
    "seeker",
    "guidance",
    "control",
    "forces",
    "newton",
    "intercept",
)
_AIM5_AERO_TABLES = (
    "cl_aim_vs_alpha_mach",
    "cd_aim_on_vs_alpha_mach",
    "cd_aim_off_vs_alpha_mach",
)
_AIM5_PROP_TABLES = ("mass_vs_time", "thrust_vs_time")


class Aim5SourceError(ValueError):
    """Source-bundle incompatibility with the executable AIM5 vertical slice."""


####


class Aim5MissileConfig(CadacModel):
    """Typed AIM5 missile inputs lowered from one source vehicle block."""

    position_ned_m: tuple[float, float, float]
    speed_mps: float = Field(gt=0.0)
    heading_deg: float
    flight_path_deg: float
    alpha_deg: float
    beta_deg: float
    area_m2: float = Field(gt=0.0)
    alpha_max_deg: float = Field(gt=0.0)
    propulsion_mode: int
    initial_mass_kg: float = Field(gt=0.0)
    nozzle_exit_area_m2: float = Field(ge=0.0)
    seeker_mode: int
    target_number: int = Field(ge=1)
    guidance_mode: int
    navigation_gain: float = Field(ge=0.0)
    rate_loop_time_constant_s: float = Field(gt=0.0)
    proportional_integral_ratio: float = Field(gt=0.0)
    acceleration_loop_gain_rad_s2: float = Field(gt=0.0)
    spiral_tgo_start_s: float = 0.0
    spiral_initial_amplitude_g: float = 0.0
    spiral_frequency_rad_s: float = 0.0
    spiral_tgo63_s: float = 0.0
    sea_level_pressure_pa: float = 101_325.0


####


class Aim5TargetConfig(CadacModel):
    """Typed AIRCRAFT3 target inputs used by the AIM5 source case."""

    position_ned_m: tuple[float, float, float]
    speed_mps: float = Field(gt=0.0)
    heading_deg: float
    flight_path_deg: float
    aircraft_option: int = 0
    guidance_gain: float = 0.0
    turn_g: float = 0.0
    bank_time_constant_s: float = 0.0
    bank_limit_deg: float = 0.0
    normal_load_time_constant_s: float = 0.0
    alpha_limit_deg: float = 0.0
    lift_slope_per_deg: float = 0.0
    wing_loading_n_m2: float = 0.0
    longitudinal_acceleration_g: float = 0.0


####


class Aim5ActorFidelity(CadacModel):
    """Taoryx horizontal-fidelity claim for one actor in the AIM5 source case."""

    model_name: Literal["AIM5", "AIRCRAFT3"]
    taoryx_tier: Literal["point_mass_3dof", "pseudo_6dof"]
    control_realization: Literal["force_model", "response_law"]


####


class Aim5SourceDefinition(CadacModel):
    """Prepared AIM5/AIRCRAFT3 engagement preserving source schedule semantics."""

    source_name: str = Field(min_length=1)
    integration_step_s: float = Field(gt=0.0)
    plot_step_s: float | None = Field(default=None, gt=0.0)
    end_time_s: float = Field(gt=0.0)
    module_order: tuple[str, ...]
    missile: Aim5MissileConfig
    target: Aim5TargetConfig
    aerodynamic_deck: CadacDeck
    propulsion_deck: CadacDeck
    missile_events: tuple[CadacEventBlock, ...] = ()
    target_events: tuple[CadacEventBlock, ...] = ()
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)
    actor_fidelity: tuple[Aim5ActorFidelity, Aim5ActorFidelity] = (
        Aim5ActorFidelity(model_name="AIM5", taoryx_tier="pseudo_6dof", control_realization="response_law"),
        Aim5ActorFidelity(model_name="AIRCRAFT3", taoryx_tier="point_mass_3dof", control_realization="force_model"),
    )

    @model_validator(mode="after")
    def validate_required_tables(self) -> "Aim5SourceDefinition":
        aero_names = {table.name.casefold() for table in self.aerodynamic_deck.tables}
        prop_names = {table.name.casefold() for table in self.propulsion_deck.tables}
        missing_aero = [name for name in _AIM5_AERO_TABLES if name.casefold() not in aero_names]
        missing_prop = [name for name in _AIM5_PROP_TABLES if name.casefold() not in prop_names]
        if missing_aero or missing_prop:
            missing = ", ".join(missing_aero + missing_prop)
            raise ValueError(f"AIM5 source definition is missing required tables: {missing}")
        ####
        return self

    ####


####


class Aim5Intercept(CadacModel):
    """Closest-approach result emitted when CADAC's intercept gate trips."""

    time_s: float
    miss_distance_m: float = Field(ge=0.0)
    differential_speed_mps: float = Field(ge=0.0)
    aspect_azimuth_deg: float
    aspect_elevation_deg: float


####


class Aim5Sample(CadacModel):
    """Compact parity-oriented telemetry sample from one compatibility run."""

    time_s: float
    missile_position_ned_m: tuple[float, float, float]
    missile_velocity_ned_mps: tuple[float, float, float]
    target_position_ned_m: tuple[float, float, float]
    target_velocity_ned_mps: tuple[float, float, float]
    target_relative_position_ned_m: tuple[float, float, float]
    unit_los_vehicle: tuple[float, float, float]
    line_of_sight_rate_vehicle_rad_s: tuple[float, float, float]
    range_m: float = Field(ge=0.0)
    closing_speed_mps: float
    missile_speed_mps: float = Field(ge=0.0)
    dynamic_pressure_pa: float = Field(ge=0.0)
    mach: float = Field(ge=0.0)
    heading_deg: float
    flight_path_deg: float
    altitude_m: float
    alpha_deg: float
    beta_deg: float
    normal_command_g: float
    lateral_command_g: float
    normal_acceleration_g: float
    lateral_acceleration_g: float
    mass_kg: float = Field(gt=0.0)
    thrust_n: float = Field(ge=0.0)


####


class Aim5ExecutionSemantics(CadacModel):
    """Source-order semantics that materially affect AIM5 numerical parity."""

    module_order: tuple[str, ...]
    vehicle_order: tuple[str, ...] = ("AIM5", "AIRCRAFT3")
    communication_refresh: str = "after_each_vehicle_module_pass"
    target_snapshot_seen_by_missile: str = "previous_target_vehicle_pass"
    event_evaluation: str = "pre_vehicle_module_pass"
    integration_rule: str = "stored_derivative_trapezoid"


####


class Aim5ModuleTrace(CadacModel):
    """State snapshot immediately after one source-ordered module dispatch."""

    time_s: float
    vehicle_model: str
    object_id: str | None = None
    module: str
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]
    speed_mps: float = Field(ge=0.0)
    mach: float = Field(ge=0.0)
    specific_force_vehicle_mps2: tuple[float, float, float]
    alpha_deg: float | None = None
    beta_deg: float | None = None
    range_m: float | None = Field(default=None, ge=0.0)
    closing_speed_mps: float | None = None
    normal_command_g: float | None = None
    lateral_command_g: float | None = None
    mass_kg: float | None = Field(default=None, gt=0.0)
    thrust_n: float | None = Field(default=None, ge=0.0)


####


class Aim5EventTrace(CadacModel):
    """One source event applied immediately before an AIM5 actor module pass."""

    time_s: float
    vehicle_model: Literal["AIM5", "AIRCRAFT3"]
    object_id: str | None = None
    event_index: int = Field(ge=0)
    source_line: int = Field(ge=1)
    watch_variable: str = Field(min_length=1)
    operator: str = Field(min_length=1, max_length=1)
    criterion: int | float
    previous_values: tuple[tuple[str, int | float], ...]
    updated_values: tuple[tuple[str, int | float], ...]


####


class Aim5RunResult(CadacModel):
    """Result and telemetry from one deterministic AIM5 compatibility run."""

    schema_id: str = "taoryx.cadac.aim5-run/v0alpha2"
    source_name: str
    integration_step_s: float
    requested_end_time_s: float
    executed_steps: int = Field(ge=0)
    terminated_reason: str
    execution_semantics: Aim5ExecutionSemantics
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)
    intercept: Aim5Intercept | None = None
    samples: tuple[Aim5Sample, ...]
    module_trace: tuple[Aim5ModuleTrace, ...] = ()
    event_trace: tuple[Aim5EventTrace, ...] = ()


####


@dataclass(slots=True)
class _Flat3State:
    time_s: float
    event_time_s: float
    gravity_mps2: float
    density_kg_m3: float
    dynamic_pressure_pa: float
    mach: float
    speed_of_sound_mps: float
    pressure_pa: float
    specific_force_vehicle_mps2: FloatVector
    bank_rad: float
    vehicle_to_local: FloatMatrix
    velocity_to_local: FloatMatrix
    speed_mps: float
    position_ned_m: FloatVector
    velocity_ned_mps: FloatVector
    acceleration_ned_mps2: FloatVector
    heading_rad: float
    flight_path_rad: float
    altitude_m: float


####


@dataclass(slots=True)
class _AimState:
    flat: _Flat3State
    config: Aim5MissileConfig
    propulsion_mode: int
    thrust_n: float
    mass_kg: float
    alpha_rad: float
    alpha_rate_rps: float = 0.0
    pitch_rate_rps: float = 0.0
    pitch_rate_derivative_rps2: float = 0.0
    pitch_integral_rps: float = 0.0
    pitch_integral_derivative_rps2: float = 0.0
    beta_rad: float = 0.0
    beta_rate_rps: float = 0.0
    yaw_rate_rps: float = 0.0
    yaw_rate_derivative_rps2: float = 0.0
    yaw_integral_rps: float = 0.0
    yaw_integral_derivative_rps2: float = 0.0
    total_alpha_deg: float = 0.0
    aero_roll_deg: float = 0.0
    axial_coefficient: float = 0.0
    side_coefficient: float = 0.0
    normal_coefficient: float = 0.0
    normal_derivative_per_rad: float = 0.0
    side_derivative_per_rad: float = 0.0
    max_g: float = 0.0
    range_m: float = math.inf
    closing_speed_mps: float = 0.0
    tgo_s: float = math.inf
    unit_los_vehicle: FloatVector = field(default_factory=lambda: _zeros3())
    los_rate_vehicle_rps: FloatVector = field(default_factory=lambda: _zeros3())
    target_relative_position_ned_m: FloatVector = field(default_factory=lambda: _zeros3())
    normal_command_g: float = 0.0
    lateral_command_g: float = 0.0
    normal_acceleration_g: float = 0.0
    lateral_acceleration_g: float = 0.0
    axial_acceleration_g: float = 0.0


####


@dataclass(slots=True)
class _AircraftState:
    flat: _Flat3State
    config: Aim5TargetConfig
    commanded_acceleration_local_mps2: FloatVector = field(default_factory=lambda: _zeros3())
    bank_rad: float = 0.0
    bank_derivative_rps: float = 0.0
    normal_load_g: float = 0.0
    normal_load_derivative_gps: float = 0.0


####


@dataclass(frozen=True, slots=True)
class _BusPacket:
    position_ned_m: FloatVector
    velocity_ned_mps: FloatVector
    heading_deg: float
    flight_path_deg: float
    alive: bool


####


def lower_aim5_source_bundle(bundle: CadacSourceBundle) -> Aim5SourceDefinition:
    """Lower one single-engagement AIM5 source bundle into typed execution inputs."""

    missiles = bundle.case.vehicles_named("AIM5")
    targets = bundle.case.vehicles_named("AIRCRAFT3")
    if len(missiles) != 1 or len(targets) != 1:
        raise Aim5SourceError(
            f"the initial executable vertical slice requires exactly one AIM5 and one AIRCRAFT3; found {len(missiles)} AIM5 and {len(targets)} AIRCRAFT3"
        )
    ####
    if tuple(vehicle.model_name.casefold() for vehicle in bundle.case.vehicles) != ("aim5", "aircraft3"):
        raise Aim5SourceError("the compatibility runner currently requires AIM5 before AIRCRAFT3 to preserve combus lag semantics")
    ####
    module_order = tuple(module.name.casefold() for module in bundle.case.modules if CadacModuleStage.EXECUTE in module.stages)
    unknown = tuple(name for name in module_order if name not in _AIM5_MODULES)
    if unknown:
        raise Aim5SourceError(f"AIM5 compatibility runner does not implement source modules: {unknown!r}")
    ####
    missing = tuple(name for name in _AIM5_MODULES if name not in module_order)
    if missing:
        raise Aim5SourceError(f"AIM5 compatibility runner requires source modules: {missing!r}")
    ####
    timing = bundle.case.timing_values
    try:
        integration_step_s = timing["int_step"]
    except KeyError as error:
        raise Aim5SourceError("AIM5 source case must declare TIMING int_step") from error
    ####
    missile_block = missiles[0]
    target_block = targets[0]
    aerodynamic_deck = bundle.deck_for("AIM5", CadacDeckKind.AERODYNAMIC)
    propulsion_deck = bundle.deck_for("AIM5", CadacDeckKind.PROPULSION)
    return Aim5SourceDefinition(
        source_name=bundle.case.source_name,
        integration_step_s=integration_step_s,
        plot_step_s=timing.get("plot_step"),
        end_time_s=bundle.case.end_time_s,
        module_order=module_order,
        missile=_lower_missile(missile_block),
        target=_lower_target(target_block),
        aerodynamic_deck=aerodynamic_deck,
        propulsion_deck=propulsion_deck,
        missile_events=missile_block.events,
        target_events=target_block.events,
        source_artifacts=bundle.artifacts,
    )


####


def load_aim5_source_definition(path: str | Path) -> Aim5SourceDefinition:
    """Parse, resolve, validate, and lower one AIM5 ``input.asc`` file."""

    return lower_aim5_source_bundle(load_cadac_source_bundle(path))


####


def run_aim5_source_compatibility(
    definition: Aim5SourceDefinition,
    *,
    sample_step_s: float | None = None,
    trace_steps: int = 0,
) -> Aim5RunResult:
    """Execute the first deterministic closed-loop AIM5 compatibility slice."""

    dt = definition.integration_step_s
    requested_sample_step = sample_step_s if sample_step_s is not None else max(dt, 0.02)
    if not math.isfinite(requested_sample_step) or requested_sample_step <= 0.0:
        raise ValueError("sample_step_s must be positive and finite")
    ####
    if trace_steps < 0:
        raise ValueError("trace_steps must be nonnegative")
    ####
    missile = _initialize_missile(definition.missile)
    target = _initialize_target(definition.target)
    target_bus = _packet_for_target(target, alive=True)
    missile_alive = True
    target_alive = True
    intercept: Aim5Intercept | None = None
    samples: list[Aim5Sample] = []
    module_trace: list[Aim5ModuleTrace] = []
    event_trace: list[Aim5EventTrace] = []
    missile_event_cursor = CadacEventCursor.from_events(definition.missile_events)
    target_event_cursor = CadacEventCursor.from_events(definition.target_events)
    next_sample_time = 0.0
    sim_time = 0.0
    steps = 0

    while sim_time <= definition.end_time_s + dt and missile_alive and target_alive:
        steps += 1
        missile.flat.time_s = sim_time
        missile_event = _evaluate_aim_event(missile_event_cursor, missile)
        if missile_event is not None:
            missile.flat.event_time_s = 0.0
            event_trace.append(_event_trace(sim_time, "AIM5", missile_event))
        ####
        for module in definition.module_order:
            if module == "environment":
                _environment(missile.flat)
            elif module == "kinematics":
                missile.flat.time_s = sim_time
            elif module == "aerodynamics":
                _aim_aerodynamics(missile, definition.aerodynamic_deck)
            elif module == "propulsion":
                _aim_propulsion(missile, definition.propulsion_deck)
            elif module == "seeker":
                _aim_seeker(missile, target_bus)
            elif module == "guidance":
                _aim_guidance(missile)
            elif module == "control":
                _aim_control(missile, dt)
            elif module == "forces":
                _aim_forces(missile)
            elif module == "newton":
                _newton(missile.flat, dt)
            elif module == "intercept":
                intercept = _aim_intercept(missile, target_bus, sim_time)
                if intercept is not None:
                    missile_alive = False
                    target_alive = False
                ####
            ####
            if steps <= trace_steps:
                module_trace.append(_module_trace_aim(sim_time, module, missile))
            ####
        ####

        if sim_time + 0.5 * dt >= next_sample_time or intercept is not None:
            samples.append(_sample(sim_time, missile, target, target_bus))
            while next_sample_time <= sim_time + 0.5 * dt:
                next_sample_time += requested_sample_step
            ####
        ####
        if not missile_alive or not target_alive:
            break
        ####

        target_event = _evaluate_target_event(target_event_cursor, target)
        if target_event is not None:
            target.flat.event_time_s = 0.0
            event_trace.append(_event_trace(sim_time, "AIRCRAFT3", target_event))
        ####
        for module in definition.module_order:
            if module == "environment":
                _environment(target.flat)
            elif module == "kinematics":
                target.flat.time_s = sim_time
            elif module == "guidance":
                _target_guidance(target)
            elif module == "control":
                _target_control(target, dt)
            elif module == "forces":
                _target_forces(target)
            elif module == "newton":
                _newton(target.flat, dt)
            ####
            if steps <= trace_steps:
                module_trace.append(_module_trace_target(sim_time, module, target))
            ####
        ####
        target_bus = _packet_for_target(target, alive=target_alive)
        missile.flat.event_time_s += dt
        target.flat.event_time_s += dt
        sim_time += dt
    ####

    if samples and samples[-1].time_s < min(sim_time, definition.end_time_s) - 0.5 * dt and intercept is None:
        samples.append(_sample(min(sim_time, definition.end_time_s), missile, target, target_bus))
    ####
    return Aim5RunResult(
        source_name=definition.source_name,
        integration_step_s=dt,
        requested_end_time_s=definition.end_time_s,
        executed_steps=steps,
        terminated_reason="intercept" if intercept is not None else "end_time",
        execution_semantics=Aim5ExecutionSemantics(module_order=definition.module_order),
        source_artifacts=definition.source_artifacts,
        intercept=intercept,
        samples=tuple(samples),
        module_trace=tuple(module_trace),
        event_trace=tuple(event_trace),
    )


####


def _evaluate_aim_event(cursor: CadacEventCursor, aim: _AimState) -> CadacEventApplication | None:
    """Evaluate one source event against the AIM5 runtime-variable registry."""

    values = _aim_event_values(aim)
    try:
        application = cursor.evaluate_and_apply(values)
    except (KeyError, TypeError) as error:
        raise Aim5SourceError(f"AIM5 event cannot be bound to the executable runtime: {error}") from error
    ####
    if application is None:
        return None
    ####
    for name, value in application.updated_values:
        _set_aim_event_value(aim, name, value)
    ####
    return application


####


def _evaluate_target_event(cursor: CadacEventCursor, target: _AircraftState) -> CadacEventApplication | None:
    """Evaluate one source event against the AIRCRAFT3 runtime-variable registry."""

    values = _target_event_values(target)
    try:
        application = cursor.evaluate_and_apply(values)
    except (KeyError, TypeError) as error:
        raise Aim5SourceError(f"AIRCRAFT3 event cannot be bound to the executable runtime: {error}") from error
    ####
    if application is None:
        return None
    ####
    for name, value in application.updated_values:
        _set_target_event_value(target, name, value)
    ####
    return application


####


def _event_trace(
    time_s: float,
    vehicle_model: Literal["AIM5", "AIRCRAFT3"],
    application: CadacEventApplication,
    *,
    object_id: str | None = None,
) -> Aim5EventTrace:
    return Aim5EventTrace(
        time_s=time_s,
        vehicle_model=vehicle_model,
        object_id=object_id,
        event_index=application.event_index,
        source_line=application.source_line,
        watch_variable=application.watch_variable,
        operator=application.operator.value,
        criterion=application.criterion,
        previous_values=application.previous_values,
        updated_values=application.updated_values,
    )


####


def _aim_event_values(aim: _AimState) -> dict[str, CadacRuntimeScalar]:
    """Expose source-named AIM5 scalars that may participate in an event."""

    return {
        "time": aim.flat.time_s,
        "event_time": aim.flat.event_time_s,
        "grav": aim.flat.gravity_mps2,
        "rho": aim.flat.density_kg_m3,
        "pdynmc": aim.flat.dynamic_pressure_pa,
        "mach": aim.flat.mach,
        "vsound": aim.flat.speed_of_sound_mps,
        "press": aim.flat.pressure_pa,
        "dvae": aim.flat.speed_mps,
        "psivlx": aim.flat.heading_rad * _DEG_PER_RAD,
        "thtvlx": aim.flat.flight_path_rad * _DEG_PER_RAD,
        "alt": aim.flat.altitude_m,
        "area": aim.config.area_m2,
        "alpmax": aim.config.alpha_max_deg,
        "mprop": aim.propulsion_mode,
        "pres_sl": aim.config.sea_level_pressure_pa,
        "aexit": aim.config.nozzle_exit_area_m2,
        "thrust": aim.thrust_n,
        "mass": aim.mass_kg,
        "mseek": aim.config.seeker_mode,
        "tgt_num": aim.config.target_number,
        "mguid": aim.config.guidance_mode,
        "gnav": aim.config.navigation_gain,
        "tgo_manvr": aim.config.spiral_tgo_start_s,
        "amp_manvr": aim.config.spiral_initial_amplitude_g,
        "frq_manvr": aim.config.spiral_frequency_rad_s,
        "tgo63_manvr": aim.config.spiral_tgo63_s,
        "tr": aim.config.rate_loop_time_constant_s,
        "ta": aim.config.proportional_integral_ratio,
        "gacp": aim.config.acceleration_loop_gain_rad_s2,
        "alppx": aim.total_alpha_deg,
        "phipx": aim.aero_roll_deg,
        "caaim": aim.axial_coefficient,
        "cyaim": aim.side_coefficient,
        "cnaim": aim.normal_coefficient,
        "cnalp": aim.normal_derivative_per_rad,
        "cybet": aim.side_derivative_per_rad,
        "gmax": aim.max_g,
        "dta": aim.range_m,
        "dvta": aim.closing_speed_mps,
        "tgo_aim": aim.tgo_s,
        "ancomx": aim.normal_command_g,
        "alcomx": aim.lateral_command_g,
        "alphax": aim.alpha_rad * _DEG_PER_RAD,
        "betax": aim.beta_rad * _DEG_PER_RAD,
        "aax": aim.axial_acceleration_g,
        "alx": aim.lateral_acceleration_g,
        "anx": aim.normal_acceleration_g,
    }


####


def _set_aim_event_value(aim: _AimState, name: str, value: CadacRuntimeScalar) -> None:
    """Apply one event mutation with the same source-variable separation as CADAC."""

    key = name.casefold()
    scalar = float(value)
    integer = int(value)
    config_field_by_source = {
        "area": "area_m2",
        "alpmax": "alpha_max_deg",
        "pres_sl": "sea_level_pressure_pa",
        "aexit": "nozzle_exit_area_m2",
        "mseek": "seeker_mode",
        "tgt_num": "target_number",
        "mguid": "guidance_mode",
        "gnav": "navigation_gain",
        "tgo_manvr": "spiral_tgo_start_s",
        "amp_manvr": "spiral_initial_amplitude_g",
        "frq_manvr": "spiral_frequency_rad_s",
        "tgo63_manvr": "spiral_tgo63_s",
        "tr": "rate_loop_time_constant_s",
        "ta": "proportional_integral_ratio",
        "gacp": "acceleration_loop_gain_rad_s2",
    }
    if key in config_field_by_source:
        field_name = config_field_by_source[key]
        replacement: int | float = integer if key in {"mseek", "tgt_num", "mguid"} else scalar
        try:
            payload = aim.config.model_dump()
            payload[field_name] = replacement
            aim.config = Aim5MissileConfig.model_validate(payload)
        except ValueError as error:
            raise Aim5SourceError(f"AIM5 event produced invalid {name!r}={value!r}: {error}") from error
        ####
        return
    ####
    if key == "mprop":
        aim.propulsion_mode = integer
    elif key == "mass":
        aim.mass_kg = scalar
    elif key == "thrust":
        aim.thrust_n = scalar
    elif key == "alphax":
        aim.alpha_rad = scalar * _RAD_PER_DEG
    elif key == "betax":
        aim.beta_rad = scalar * _RAD_PER_DEG
    elif key == "dvae":
        aim.flat.speed_mps = scalar
    elif key == "time":
        aim.flat.time_s = scalar
    elif key == "event_time":
        aim.flat.event_time_s = scalar
    elif key == "ancomx":
        aim.normal_command_g = scalar
    elif key == "alcomx":
        aim.lateral_command_g = scalar
    elif key == "aax":
        aim.axial_acceleration_g = scalar
    elif key == "alx":
        aim.lateral_acceleration_g = scalar
    elif key == "anx":
        aim.normal_acceleration_g = scalar
    else:
        raise Aim5SourceError(f"AIM5 event assignment {name!r} is a readable source diagnostic but is not mutable in the executable slice")
    ####


####


def _target_event_values(target: _AircraftState) -> dict[str, CadacRuntimeScalar]:
    """Expose source-named AIRCRAFT3 scalars that may participate in an event."""

    return {
        "time": target.flat.time_s,
        "event_time": target.flat.event_time_s,
        "grav": target.flat.gravity_mps2,
        "rho": target.flat.density_kg_m3,
        "pdynmc": target.flat.dynamic_pressure_pa,
        "mach": target.flat.mach,
        "vsound": target.flat.speed_of_sound_mps,
        "press": target.flat.pressure_pa,
        "dvae": target.flat.speed_mps,
        "psivlx": target.flat.heading_rad * _DEG_PER_RAD,
        "thtvlx": target.flat.flight_path_rad * _DEG_PER_RAD,
        "alt": target.flat.altitude_m,
        "acft_option": target.config.aircraft_option,
        "guid_gain": target.config.guidance_gain,
        "gturn": target.config.turn_g,
        "tphi": target.config.bank_time_constant_s,
        "philimx": target.config.bank_limit_deg,
        "tanx": target.config.normal_load_time_constant_s,
        "alplimx": target.config.alpha_limit_deg,
        "clalpha": target.config.lift_slope_per_deg,
        "wingloading": target.config.wing_loading_n_m2,
        "acc_longx": target.config.longitudinal_acceleration_g,
        "phiav": target.bank_rad,
        "phiavd": target.bank_derivative_rps,
        "phiavx": target.flat.bank_rad * _DEG_PER_RAD,
        "anx": target.normal_load_g,
        "anxd": target.normal_load_derivative_gps,
    }


####


def _set_target_event_value(target: _AircraftState, name: str, value: CadacRuntimeScalar) -> None:
    key = name.casefold()
    scalar = float(value)
    integer = int(value)
    config_field_by_source = {
        "acft_option": "aircraft_option",
        "guid_gain": "guidance_gain",
        "gturn": "turn_g",
        "tphi": "bank_time_constant_s",
        "philimx": "bank_limit_deg",
        "tanx": "normal_load_time_constant_s",
        "alplimx": "alpha_limit_deg",
        "clalpha": "lift_slope_per_deg",
        "wingloading": "wing_loading_n_m2",
        "acc_longx": "longitudinal_acceleration_g",
    }
    if key in config_field_by_source:
        field_name = config_field_by_source[key]
        replacement: int | float = integer if key == "acft_option" else scalar
        try:
            payload = target.config.model_dump()
            payload[field_name] = replacement
            target.config = Aim5TargetConfig.model_validate(payload)
        except ValueError as error:
            raise Aim5SourceError(f"AIRCRAFT3 event produced invalid {name!r}={value!r}: {error}") from error
        ####
        return
    ####
    if key == "dvae":
        target.flat.speed_mps = scalar
    elif key == "time":
        target.flat.time_s = scalar
    elif key == "event_time":
        target.flat.event_time_s = scalar
    elif key == "phiav":
        target.bank_rad = scalar
    elif key == "phiavd":
        target.bank_derivative_rps = scalar
    elif key == "anx":
        target.normal_load_g = scalar
    elif key == "anxd":
        target.normal_load_derivative_gps = scalar
    else:
        raise Aim5SourceError(f"AIRCRAFT3 event assignment {name!r} is a readable source diagnostic but is not mutable in the executable slice")
    ####


####


def mat2tr(heading_rad: float, flight_path_rad: float) -> FloatMatrix:
    """Return CADAC's heading-to-flight-path transformation matrix."""

    matrix = np.zeros((3, 3), dtype=np.float64)
    matrix[0, 2] = -math.sin(flight_path_rad)
    matrix[1, 0] = -math.sin(heading_rad)
    matrix[1, 1] = math.cos(heading_rad)
    matrix[2, 2] = math.cos(flight_path_rad)
    matrix[0, 0] = matrix[2, 2] * matrix[1, 1]
    matrix[0, 1] = -matrix[2, 2] * matrix[1, 0]
    matrix[2, 0] = -matrix[0, 2] * matrix[1, 1]
    matrix[2, 1] = matrix[0, 2] * matrix[1, 0]
    return matrix


####


def _lower_missile(vehicle: CadacVehicleBlock) -> Aim5MissileConfig:
    return Aim5MissileConfig(
        position_ned_m=(_number(vehicle, "sael1"), _number(vehicle, "sael2"), _number(vehicle, "sael3")),
        speed_mps=_number(vehicle, "dvae"),
        heading_deg=_number(vehicle, "psivlx"),
        flight_path_deg=_number(vehicle, "thtvlx"),
        alpha_deg=_number(vehicle, "alphax", 0.0),
        beta_deg=_number(vehicle, "betax", 0.0),
        area_m2=_number(vehicle, "area"),
        alpha_max_deg=_number(vehicle, "alpmax"),
        propulsion_mode=_integer(vehicle, "mprop", 0),
        initial_mass_kg=_number(vehicle, "mass"),
        nozzle_exit_area_m2=_number(vehicle, "aexit", 0.0),
        seeker_mode=_integer(vehicle, "mseek", 0),
        target_number=_integer(vehicle, "tgt_num", 1),
        guidance_mode=_integer(vehicle, "mguid", 0),
        navigation_gain=_number(vehicle, "gnav", 0.0),
        rate_loop_time_constant_s=_number(vehicle, "tr"),
        proportional_integral_ratio=_number(vehicle, "ta"),
        acceleration_loop_gain_rad_s2=_number(vehicle, "gacp"),
        spiral_tgo_start_s=_number(vehicle, "tgo_manvr", 0.0),
        spiral_initial_amplitude_g=_number(vehicle, "amp_manvr", 0.0),
        spiral_frequency_rad_s=_number(vehicle, "frq_manvr", 0.0),
        spiral_tgo63_s=_number(vehicle, "tgo63_manvr", 0.0),
    )


####


def _lower_target(vehicle: CadacVehicleBlock) -> Aim5TargetConfig:
    return Aim5TargetConfig(
        position_ned_m=(_number(vehicle, "sael1"), _number(vehicle, "sael2"), _number(vehicle, "sael3")),
        speed_mps=_number(vehicle, "dvae"),
        heading_deg=_number(vehicle, "psivlx"),
        flight_path_deg=_number(vehicle, "thtvlx"),
        aircraft_option=_integer(vehicle, "acft_option", 0),
        guidance_gain=_number(vehicle, "guid_gain", 0.0),
        turn_g=_number(vehicle, "gturn", 0.0),
        bank_time_constant_s=_number(vehicle, "tphi", 0.0),
        bank_limit_deg=_number(vehicle, "philimx", 0.0),
        normal_load_time_constant_s=_number(vehicle, "tanx", 0.0),
        alpha_limit_deg=_number(vehicle, "alplimx", 0.0),
        lift_slope_per_deg=_number(vehicle, "clalpha", 0.0),
        wing_loading_n_m2=_number(vehicle, "wingloading", 0.0),
        longitudinal_acceleration_g=_number(vehicle, "acc_longx", 0.0),
    )


####


def _number(vehicle: CadacVehicleBlock, name: str, default: float | None = None) -> float:
    try:
        value = vehicle.parameter(name)
    except KeyError:
        if default is None:
            raise Aim5SourceError(f"{vehicle.model_name} is missing required parameter {name!r}") from None
        ####
        return default
    ####
    if not isinstance(value, (int, float)):
        raise Aim5SourceError(f"{vehicle.model_name} parameter {name!r} must be numeric")
    ####
    result = float(value)
    if not math.isfinite(result):
        raise Aim5SourceError(f"{vehicle.model_name} parameter {name!r} must be finite")
    ####
    return result


####


def _integer(vehicle: CadacVehicleBlock, name: str, default: int | None = None) -> int:
    value = _number(vehicle, name, float(default) if default is not None else None)
    if not value.is_integer():
        raise Aim5SourceError(f"{vehicle.model_name} parameter {name!r} must be integer-valued")
    ####
    return int(value)


####


def _initialize_flat(position: tuple[float, float, float], speed: float, heading_deg: float, flight_path_deg: float) -> _Flat3State:
    heading = heading_deg * _RAD_PER_DEG
    flight_path = flight_path_deg * _RAD_PER_DEG
    velocity = _cart_from_pol(speed, heading, flight_path)
    tvl = mat2tr(heading, flight_path)
    return _Flat3State(
        time_s=0.0,
        event_time_s=0.0,
        gravity_mps2=0.0,
        density_kg_m3=0.0,
        dynamic_pressure_pa=0.0,
        mach=0.0,
        speed_of_sound_mps=0.0,
        pressure_pa=0.0,
        specific_force_vehicle_mps2=_zeros3(),
        bank_rad=0.0,
        vehicle_to_local=tvl.copy(),
        velocity_to_local=tvl.copy(),
        speed_mps=speed,
        position_ned_m=np.asarray(position, dtype=np.float64),
        velocity_ned_mps=velocity,
        acceleration_ned_mps2=_zeros3(),
        heading_rad=heading,
        flight_path_rad=flight_path,
        altitude_m=-position[2],
    )


####


def _initialize_missile(config: Aim5MissileConfig) -> _AimState:
    return _AimState(
        flat=_initialize_flat(config.position_ned_m, config.speed_mps, config.heading_deg, config.flight_path_deg),
        config=config,
        propulsion_mode=config.propulsion_mode,
        thrust_n=0.0,
        mass_kg=config.initial_mass_kg,
        alpha_rad=config.alpha_deg * _RAD_PER_DEG,
        beta_rad=config.beta_deg * _RAD_PER_DEG,
    )


####


def _initialize_target(config: Aim5TargetConfig) -> _AircraftState:
    return _AircraftState(flat=_initialize_flat(config.position_ned_m, config.speed_mps, config.heading_deg, config.flight_path_deg), config=config)


####


def _environment(flat: _Flat3State) -> None:
    altitude = -float(flat.position_ned_m[2])
    gravity = cadac_source_inverse_square_gravity_mps2(altitude)
    density, pressure, temperature = atmosphere76(altitude)
    sound = math.sqrt(1.4 * _R_AIR_J_PER_KG_K * temperature)
    flat.altitude_m = altitude
    flat.gravity_mps2 = gravity
    flat.density_kg_m3 = density
    flat.pressure_pa = pressure
    flat.speed_of_sound_mps = sound
    flat.mach = abs(flat.speed_mps / sound)
    flat.dynamic_pressure_pa = 0.5 * density * flat.speed_mps**2


####


def _aim_aerodynamics(aim: _AimState, deck: CadacDeck) -> None:
    alpha = aim.alpha_rad
    beta = aim.beta_rad
    combined = math.acos(max(-1.0, min(1.0, math.cos(alpha) * math.cos(beta))))
    denominator = math.sin(alpha)
    if abs(denominator) < _SMALL:
        denominator = _SMALL * _sign(denominator)
    ####
    aero_roll = math.atan2(math.tan(beta), denominator)
    total_alpha_deg = combined * _DEG_PER_RAD
    lift = deck.table("cl_aim_vs_alpha_mach").interpolate((total_alpha_deg, aim.flat.mach))
    drag_name = "cd_aim_on_vs_alpha_mach" if aim.propulsion_mode else "cd_aim_off_vs_alpha_mach"
    drag = deck.table(drag_name).interpolate((total_alpha_deg, aim.flat.mach))
    axial = drag * math.cos(alpha) - lift * math.sin(alpha)
    normal_plane = drag * math.sin(alpha) + lift * math.cos(alpha)
    normal = abs(normal_plane) * math.cos(aero_roll)
    side = -abs(normal_plane) * math.sin(aero_roll)
    alpha_max_rad = aim.config.alpha_max_deg * _RAD_PER_DEG
    lift_max = deck.table("cl_aim_vs_alpha_mach").interpolate((aim.config.alpha_max_deg, aim.flat.mach))
    drag_max = deck.table(drag_name).interpolate((aim.config.alpha_max_deg, aim.flat.mach))
    normal_max = drag_max * math.sin(alpha_max_rad) + lift_max * math.cos(alpha_max_rad)
    alpha_abs = abs(aim.alpha_rad * _DEG_PER_RAD)
    beta_abs = abs(aim.beta_rad * _DEG_PER_RAD)
    normal_derivative = ((0.123 + 0.013 * alpha_abs) if alpha_abs < 10.0 else 0.06 * alpha_abs**0.625) * _DEG_PER_RAD
    side_derivative = -((0.123 + 0.013 * beta_abs) if beta_abs < 10.0 else 0.06 * beta_abs**0.625) * _DEG_PER_RAD
    weight = aim.mass_kg * aim.flat.gravity_mps2
    max_g = normal_max * aim.flat.dynamic_pressure_pa * aim.config.area_m2 / weight
    aim.total_alpha_deg = total_alpha_deg
    aim.aero_roll_deg = aero_roll * _DEG_PER_RAD
    aim.axial_coefficient = axial
    aim.side_coefficient = side
    aim.normal_coefficient = normal
    aim.normal_derivative_per_rad = normal_derivative
    aim.side_derivative_per_rad = side_derivative
    aim.max_g = max_g


####


def _aim_propulsion(aim: _AimState, deck: CadacDeck) -> None:
    thrust_sl = 0.0
    thrust = 0.0
    if aim.propulsion_mode == 1:
        thrust_sl = deck.table("thrust_vs_time").interpolate((aim.flat.time_s,))
        thrust = thrust_sl + (aim.config.sea_level_pressure_pa - aim.flat.pressure_pa) * aim.config.nozzle_exit_area_m2
        aim.mass_kg = deck.table("mass_vs_time").interpolate((aim.flat.time_s,))
    ####
    if aim.flat.time_s > 0.0 and thrust_sl == 0.0:
        aim.propulsion_mode = 0
        thrust = 0.0
    ####
    aim.thrust_n = max(0.0, thrust)


####


def _aim_seeker(aim: _AimState, target: _BusPacket) -> None:
    if not aim.config.seeker_mode:
        return
    ####
    sensor = cadac_local_ned_relative_state_track(
        time_s=aim.flat.time_s,
        host_position_ned_m=aim.flat.position_ned_m,
        host_velocity_ned_mps=aim.flat.velocity_ned_mps,
        target_id="aim5-target",
        target_position_ned_m=target.position_ned_m,
        target_velocity_ned_mps=target.velocity_ned_mps,
        body_from_local=aim.flat.vehicle_to_local,
    )
    if isinstance(sensor, str):
        relative_position = _zeros3()
        distance = 0.0
        unit_los_vehicle = _zeros3()
        los_rate_vehicle = _zeros3()
        closing = 0.0
        tgo = 0.0
    else:
        relative_position = aim.flat.vehicle_to_local.T @ np.asarray(sensor.relative_position_sensor_m, dtype=np.float64)
        distance = sensor.range_m
        unit_los_vehicle = np.asarray(sensor.unit_los_sensor, dtype=np.float64)
        # AIM5's source channel calls positive target-range increase "closing".
        closing = -sensor.closing_speed_mps
        tgo = distance / abs(closing) if abs(closing) > 0.0 else math.inf
        los_rate_vehicle = np.asarray(sensor.line_of_sight_rate_sensor_rad_s, dtype=np.float64)
    ####
    aim.target_relative_position_ned_m = relative_position
    aim.range_m = distance
    aim.closing_speed_mps = closing
    aim.tgo_s = tgo
    aim.unit_los_vehicle = unit_los_vehicle
    aim.los_rate_vehicle_rps = los_rate_vehicle


####


def _aim_guidance(aim: _AimState) -> None:
    guidance_maneuver = aim.config.guidance_mode // 10
    guidance_mode = aim.config.guidance_mode % 10
    normal = 0.0
    lateral = 0.0
    if guidance_mode == 1:
        command = np.cross(aim.los_rate_vehicle_rps, aim.unit_los_vehicle) * aim.config.navigation_gain * abs(aim.closing_speed_mps)
        normal = -float(command[2]) / aim.flat.gravity_mps2
        lateral = float(command[1]) / aim.flat.gravity_mps2
    ####
    if guidance_maneuver == 1 and aim.tgo_s < aim.config.spiral_tgo_start_s:
        if aim.config.spiral_tgo63_s <= 0.0:
            raise Aim5SourceError("spiral guidance requires positive tgo63_manvr")
        ####
        amplitude = aim.config.spiral_initial_amplitude_g * (1.0 - math.exp(-aim.tgo_s / aim.config.spiral_tgo63_s))
        normal += amplitude * math.sin(aim.config.spiral_frequency_rad_s * aim.tgo_s)
        lateral += amplitude * math.cos(aim.config.spiral_frequency_rad_s * aim.tgo_s)
    ####
    total = math.hypot(lateral, normal)
    if total > aim.max_g:
        total = aim.max_g
    ####
    if abs(normal) < _SMALL or abs(lateral) < _SMALL:
        angle = 0.0
    else:
        angle = math.atan2(normal, lateral)
    ####
    aim.lateral_command_g = total * math.cos(angle)
    aim.normal_command_g = total * math.sin(angle)


####


def _aim_control(aim: _AimState, dt: float) -> None:
    config = aim.config
    q = aim.flat.dynamic_pressure_pa
    speed = aim.flat.speed_mps
    gravity = aim.flat.gravity_mps2
    tip = speed * aim.mass_kg / (q * config.area_m2 * abs(aim.normal_derivative_per_rad) + aim.thrust_n)
    pitch_specific_force = -q * config.area_m2 * aim.normal_coefficient / aim.mass_kg
    gr = config.acceleration_loop_gain_rad_s2 * tip * config.rate_loop_time_constant_s / speed
    gi = gr / config.proportional_integral_ratio
    desired_pitch_specific_force = -aim.normal_command_g * gravity
    pitch_error = desired_pitch_specific_force - pitch_specific_force
    pitch_integral_derivative_new = gi * pitch_error
    aim.pitch_integral_rps = _integrate_scalar(
        pitch_integral_derivative_new,
        aim.pitch_integral_derivative_rps2,
        aim.pitch_integral_rps,
        dt,
    )
    aim.pitch_integral_derivative_rps2 = pitch_integral_derivative_new
    pitch_rate_command = -(pitch_error * gr + aim.pitch_integral_rps)
    pitch_rate_derivative_new = (pitch_rate_command - aim.pitch_rate_rps) / config.rate_loop_time_constant_s
    aim.pitch_rate_rps = _integrate_scalar(pitch_rate_derivative_new, aim.pitch_rate_derivative_rps2, aim.pitch_rate_rps, dt)
    aim.pitch_rate_derivative_rps2 = pitch_rate_derivative_new
    alpha_rate_new = (tip * aim.pitch_rate_rps - aim.alpha_rad) / tip
    aim.alpha_rad = _integrate_scalar(alpha_rate_new, aim.alpha_rate_rps, aim.alpha_rad, dt)
    aim.alpha_rate_rps = alpha_rate_new
    aim.alpha_rad = _limit_angle(aim.alpha_rad, config.alpha_max_deg)

    tiy = speed * aim.mass_kg / (q * config.area_m2 * abs(aim.side_derivative_per_rad) + aim.thrust_n)
    yaw_specific_force = q * config.area_m2 * aim.side_coefficient / aim.mass_kg
    gr = config.acceleration_loop_gain_rad_s2 * tiy * config.rate_loop_time_constant_s / speed
    gi = gr / config.proportional_integral_ratio
    desired_yaw_specific_force = aim.lateral_command_g * gravity
    yaw_error = desired_yaw_specific_force - yaw_specific_force
    yaw_integral_derivative_new = gi * yaw_error
    aim.yaw_integral_rps = _integrate_scalar(yaw_integral_derivative_new, aim.yaw_integral_derivative_rps2, aim.yaw_integral_rps, dt)
    aim.yaw_integral_derivative_rps2 = yaw_integral_derivative_new
    yaw_rate_command = yaw_error * gr + aim.yaw_integral_rps
    yaw_rate_derivative_new = (yaw_rate_command - aim.yaw_rate_rps) / config.rate_loop_time_constant_s
    aim.yaw_rate_rps = _integrate_scalar(yaw_rate_derivative_new, aim.yaw_rate_derivative_rps2, aim.yaw_rate_rps, dt)
    aim.yaw_rate_derivative_rps2 = yaw_rate_derivative_new
    beta_rate_new = -(tiy * aim.yaw_rate_rps + aim.beta_rad) / tiy
    aim.beta_rad = _integrate_scalar(beta_rate_new, aim.beta_rate_rps, aim.beta_rad, dt)
    aim.beta_rate_rps = beta_rate_new
    aim.beta_rad = _limit_angle(aim.beta_rad, config.alpha_max_deg)


####


def _aim_forces(aim: _AimState) -> None:
    q_area = aim.flat.dynamic_pressure_pa * aim.config.area_m2
    force = np.array(
        (
            (aim.thrust_n - aim.axial_coefficient * q_area) / aim.mass_kg,
            aim.side_coefficient * q_area / aim.mass_kg,
            -aim.normal_coefficient * q_area / aim.mass_kg,
        ),
        dtype=np.float64,
    )
    aim.flat.specific_force_vehicle_mps2 = force
    aim.axial_acceleration_g = float(force[0]) / aim.flat.gravity_mps2
    aim.lateral_acceleration_g = float(force[1]) / aim.flat.gravity_mps2
    aim.normal_acceleration_g = -float(force[2]) / aim.flat.gravity_mps2


####


def _target_guidance(target: _AircraftState) -> None:
    config = target.config
    gravity = target.flat.gravity_mps2
    if config.aircraft_option == 0:
        target.commanded_acceleration_local_mps2 = np.array((0.0, 0.0, -gravity), dtype=np.float64)
    elif config.aircraft_option == 1:
        command_velocity = np.array((0.0, config.turn_g * gravity, -gravity), dtype=np.float64)
        target.commanded_acceleration_local_mps2 = target.flat.velocity_to_local.T @ command_velocity
    else:
        raise Aim5SourceError("AIRCRAFT3 escape option 2 is not yet executable in the first AIM5 vertical slice")
    ####


####


def _target_control(target: _AircraftState, dt: float) -> None:
    config = target.config
    gravity = target.flat.gravity_mps2
    command_velocity = target.flat.velocity_to_local @ target.commanded_acceleration_local_mps2
    lateral = float(command_velocity[1])
    normal = float(command_velocity[2])
    bank_command = 0.0 if abs(lateral) < 1.0e-10 and abs(normal) < 1.0e-10 else math.atan2(lateral, -normal)
    if config.bank_time_constant_s:
        derivative_new = (bank_command - target.bank_rad) / config.bank_time_constant_s
        target.bank_rad = _integrate_scalar(derivative_new, target.bank_derivative_rps, target.bank_rad, dt)
        target.bank_derivative_rps = derivative_new
    else:
        target.bank_rad = bank_command
    ####
    bank_deg = target.bank_rad * _DEG_PER_RAD
    if config.bank_limit_deg and abs(bank_deg) >= config.bank_limit_deg:
        bank_deg = config.bank_limit_deg * _sign(bank_deg)
    ####
    target.flat.bank_rad = bank_deg * _RAD_PER_DEG
    normal_command_g = math.hypot(lateral, normal) / gravity
    if config.normal_load_time_constant_s:
        derivative_new = (normal_command_g - target.normal_load_g) / config.normal_load_time_constant_s
        target.normal_load_g = _integrate_scalar(derivative_new, target.normal_load_derivative_gps, target.normal_load_g, dt)
        target.normal_load_derivative_gps = derivative_new
    else:
        target.normal_load_g = normal_command_g
    ####
    if config.aircraft_option > 0:
        if config.wing_loading_n_m2 <= 0.0:
            raise Aim5SourceError("maneuvering AIRCRAFT3 requires positive wingloading")
        ####
        limit = target.flat.dynamic_pressure_pa * config.lift_slope_per_deg * config.alpha_limit_deg / config.wing_loading_n_m2
        if abs(target.normal_load_g) >= limit:
            target.normal_load_g = limit * _sign(target.normal_load_g)
        ####


####


def _target_forces(target: _AircraftState) -> None:
    gravity = target.flat.gravity_mps2
    target.flat.specific_force_vehicle_mps2 = np.array(
        (
            target.config.longitudinal_acceleration_g * gravity,
            0.0,
            -target.normal_load_g * gravity,
        ),
        dtype=np.float64,
    )


####


def _newton(flat: _Flat3State, dt: float) -> None:
    gravity_local = np.array((0.0, 0.0, flat.gravity_mps2), dtype=np.float64)
    acceleration_new = flat.vehicle_to_local.T @ flat.specific_force_vehicle_mps2 + gravity_local
    velocity_new = _integrate_vector(acceleration_new, flat.acceleration_ned_mps2, flat.velocity_ned_mps, dt)
    position_new = _integrate_vector(velocity_new, flat.velocity_ned_mps, flat.position_ned_m, dt)
    flat.acceleration_ned_mps2 = acceleration_new
    flat.velocity_ned_mps = velocity_new
    flat.position_ned_m = position_new
    speed, heading, flight_path = _pol_from_cart(velocity_new)
    flat.speed_mps = speed
    flat.heading_rad = heading
    flat.flight_path_rad = flight_path
    flat.velocity_to_local = mat2tr(heading, flight_path)
    cphi = math.cos(flat.bank_rad)
    sphi = math.sin(flat.bank_rad)
    vehicle_to_velocity = np.eye(3, dtype=np.float64)
    vehicle_to_velocity[1, 1] = cphi
    vehicle_to_velocity[2, 2] = cphi
    vehicle_to_velocity[1, 2] = sphi
    vehicle_to_velocity[2, 1] = -sphi
    flat.vehicle_to_local = vehicle_to_velocity @ flat.velocity_to_local
    flat.altitude_m = -float(position_new[2])


####


def _aim_intercept(aim: _AimState, target: _BusPacket, sim_time: float) -> Aim5Intercept | None:
    if aim.range_m >= 500.0 or aim.closing_speed_mps <= 0.0:
        return None
    ####
    relative_velocity = target.velocity_ned_mps - aim.flat.velocity_ned_mps
    differential_speed = float(np.linalg.norm(relative_velocity))
    target_transform = mat2tr(target.heading_deg * _RAD_PER_DEG, target.flight_path_deg * _RAD_PER_DEG)
    relative_target = target_transform @ relative_velocity
    _, azimuth, elevation = _pol_from_cart(relative_target)
    return Aim5Intercept(
        time_s=sim_time,
        miss_distance_m=aim.range_m,
        differential_speed_mps=differential_speed,
        aspect_azimuth_deg=azimuth * _DEG_PER_RAD,
        aspect_elevation_deg=elevation * _DEG_PER_RAD,
    )


####


def aim5_plot_projection(
    result: Aim5RunResult,
    *,
    times_s: tuple[float, ...] | None = None,
) -> dict[str, tuple[float, ...]]:
    """Project compatibility samples onto common AIM5 ``plot*.asc`` channel names."""

    samples = result.samples if times_s is None else _samples_at_times(result, times_s)
    return {
        "time": tuple(sample.time_s for sample in samples),
        "pdynmc": tuple(sample.dynamic_pressure_pa for sample in samples),
        "mach": tuple(sample.mach for sample in samples),
        "psivlx": tuple(sample.heading_deg for sample in samples),
        "thtvlx": tuple(sample.flight_path_deg for sample in samples),
        "alt": tuple(sample.altitude_m for sample in samples),
        "alphax": tuple(sample.alpha_deg for sample in samples),
        "betax": tuple(sample.beta_deg for sample in samples),
        "ancomx": tuple(sample.normal_command_g for sample in samples),
        "alcomx": tuple(sample.lateral_command_g for sample in samples),
        "anx": tuple(sample.normal_acceleration_g for sample in samples),
        "alx": tuple(sample.lateral_acceleration_g for sample in samples),
    }


####


def _samples_at_times(result: Aim5RunResult, times_s: tuple[float, ...]) -> tuple[Aim5Sample, ...]:
    matched: list[Aim5Sample] = []
    tolerance = 0.51 * result.integration_step_s
    for requested in times_s:
        if requested < 0.0:
            continue
        ####
        sample = min(result.samples, key=lambda item: abs(item.time_s - requested))
        if abs(sample.time_s - requested) > tolerance:
            raise ValueError(f"AIM5 run has no sample aligned to source plot time {requested}")
        ####
        matched.append(sample)
    ####
    return tuple(matched)


####


def _module_trace_aim(
    time_s: float,
    module: str,
    aim: _AimState,
    *,
    object_id: str | None = None,
) -> Aim5ModuleTrace:
    return Aim5ModuleTrace(
        time_s=time_s,
        vehicle_model="AIM5",
        object_id=object_id,
        module=module,
        position_ned_m=_tuple3(aim.flat.position_ned_m),
        velocity_ned_mps=_tuple3(aim.flat.velocity_ned_mps),
        speed_mps=aim.flat.speed_mps,
        mach=aim.flat.mach,
        specific_force_vehicle_mps2=_tuple3(aim.flat.specific_force_vehicle_mps2),
        alpha_deg=aim.alpha_rad * _DEG_PER_RAD,
        beta_deg=aim.beta_rad * _DEG_PER_RAD,
        range_m=None if not math.isfinite(aim.range_m) else aim.range_m,
        closing_speed_mps=aim.closing_speed_mps,
        normal_command_g=aim.normal_command_g,
        lateral_command_g=aim.lateral_command_g,
        mass_kg=aim.mass_kg,
        thrust_n=aim.thrust_n,
    )


####


def _module_trace_target(
    time_s: float,
    module: str,
    target: _AircraftState,
    *,
    object_id: str | None = None,
) -> Aim5ModuleTrace:
    return Aim5ModuleTrace(
        time_s=time_s,
        vehicle_model="AIRCRAFT3",
        object_id=object_id,
        module=module,
        position_ned_m=_tuple3(target.flat.position_ned_m),
        velocity_ned_mps=_tuple3(target.flat.velocity_ned_mps),
        speed_mps=target.flat.speed_mps,
        mach=target.flat.mach,
        specific_force_vehicle_mps2=_tuple3(target.flat.specific_force_vehicle_mps2),
    )


####


def _packet_for_target(target: _AircraftState, *, alive: bool) -> _BusPacket:
    return _BusPacket(
        position_ned_m=target.flat.position_ned_m.copy(),
        velocity_ned_mps=target.flat.velocity_ned_mps.copy(),
        heading_deg=target.flat.heading_rad * _DEG_PER_RAD,
        flight_path_deg=target.flat.flight_path_rad * _DEG_PER_RAD,
        alive=alive,
    )


####


def _sample(time_s: float, aim: _AimState, target: _AircraftState, target_bus: _BusPacket) -> Aim5Sample:
    del target
    return Aim5Sample(
        time_s=time_s,
        missile_position_ned_m=_tuple3(aim.flat.position_ned_m),
        missile_velocity_ned_mps=_tuple3(aim.flat.velocity_ned_mps),
        target_position_ned_m=_tuple3(target_bus.position_ned_m),
        target_velocity_ned_mps=_tuple3(target_bus.velocity_ned_mps),
        target_relative_position_ned_m=_tuple3(aim.target_relative_position_ned_m),
        unit_los_vehicle=_tuple3(aim.unit_los_vehicle),
        line_of_sight_rate_vehicle_rad_s=_tuple3(aim.los_rate_vehicle_rps),
        range_m=aim.range_m,
        closing_speed_mps=aim.closing_speed_mps,
        missile_speed_mps=aim.flat.speed_mps,
        dynamic_pressure_pa=aim.flat.dynamic_pressure_pa,
        mach=aim.flat.mach,
        heading_deg=aim.flat.heading_rad * _DEG_PER_RAD,
        flight_path_deg=aim.flat.flight_path_rad * _DEG_PER_RAD,
        altitude_m=aim.flat.altitude_m,
        alpha_deg=aim.alpha_rad * _DEG_PER_RAD,
        beta_deg=aim.beta_rad * _DEG_PER_RAD,
        normal_command_g=aim.normal_command_g,
        lateral_command_g=aim.lateral_command_g,
        normal_acceleration_g=aim.normal_acceleration_g,
        lateral_acceleration_g=aim.lateral_acceleration_g,
        mass_kg=aim.mass_kg,
        thrust_n=aim.thrust_n,
    )


####


def _integrate_scalar(rate_new: float, rate_previous: float, state: float, dt: float) -> float:
    return cadac_stored_derivative_step((state,), (rate_new,), (rate_previous,), dt)[0]


####


def _integrate_vector(rate_new: FloatVector, rate_previous: FloatVector, state: FloatVector, dt: float) -> FloatVector:
    return np.asarray(cadac_stored_derivative_step(state, rate_new, rate_previous, dt), dtype=np.float64)


####


def _cart_from_pol(magnitude: float, azimuth: float, elevation: float) -> FloatVector:
    return np.array(
        (
            magnitude * math.cos(elevation) * math.cos(azimuth),
            magnitude * math.cos(elevation) * math.sin(azimuth),
            -magnitude * math.sin(elevation),
        ),
        dtype=np.float64,
    )


####


def _pol_from_cart(vector: FloatVector) -> tuple[float, float, float]:
    first, second, third = (float(value) for value in vector)
    magnitude = math.sqrt(first * first + second * second + third * third)
    azimuth = math.atan2(second, first)
    denominator = math.sqrt(first * first + second * second)
    if denominator > 0.0:
        elevation = math.atan2(-third, denominator)
    elif third > 0.0:
        elevation = -math.pi / 2.0
    elif third < 0.0:
        elevation = math.pi / 2.0
    else:
        elevation = 0.0
    ####
    return magnitude, azimuth, elevation


####


def _limit_angle(angle_rad: float, limit_deg: float) -> float:
    angle_deg = angle_rad * _DEG_PER_RAD
    if abs(angle_deg) > limit_deg:
        angle_deg = limit_deg * _sign(angle_deg)
    ####
    return angle_deg * _RAD_PER_DEG


####


def _sign(value: float) -> int:
    return -1 if value < 0.0 else 1


####


def _zeros3() -> FloatVector:
    return np.zeros(3, dtype=np.float64)


####


def _tuple3(vector: FloatVector) -> tuple[float, float, float]:
    return float(vector[0]), float(vector[1]), float(vector[2])


####


__all__ = [
    "Aim5ActorFidelity",
    "Aim5ExecutionSemantics",
    "Aim5Intercept",
    "Aim5MissileConfig",
    "Aim5ModuleTrace",
    "Aim5RunResult",
    "Aim5Sample",
    "Aim5SourceDefinition",
    "Aim5SourceError",
    "Aim5TargetConfig",
    "aim5_plot_projection",
    "atmosphere76",
    "load_aim5_source_definition",
    "lower_aim5_source_bundle",
    "mat2tr",
    "run_aim5_source_compatibility",
]
