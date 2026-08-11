"""Source-grounded CRUISE5 round-Earth pseudo-6DoF reconstruction."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import TypeAlias

import numpy as np
from numpy.typing import NDArray
from pydantic import Field, model_validator

from .bundle import CadacSourceArtifact, CadacSourceBundle, load_cadac_source_bundle
from .compatibility import cadac_stored_derivative_step
from .deck import CadacDeck
from .events import CadacEventCursor
from .input_ast import CadacDeckKind, CadacEventBlock, CadacModel, CadacModuleStage, CadacVehicleBlock
from .round3 import (
    DEG_PER_RAD,
    RAD_PER_DEG,
    CadacRound3Environment,
    CadacRound3InitialState,
    CadacRound3RuntimeState,
    cadac_cadine,
    cadac_cadtbv,
    cadac_mat2tr,
    cadac_round3_environment,
    cadac_round3_initialize,
    cadac_round3_newton_step,
)

FloatVector: TypeAlias = NDArray[np.float64]

_CRUISE5_MODULES = (
    "environment",
    "aerodynamics",
    "propulsion",
    "forces",
    "newton",
    "guidance",
    "control",
    "intercept",
)
_CRUISE5_AERO_TABLES = (
    "cd0_vs_mach",
    "cl0_vs_mach",
    "ckk_vs_mach",
    "cla_vs_mach",
    "cla0_vs_mach",
)
_CRUISE5_PROP_TABLES = (
    "cg_vs_mass",
    "iff_vs_alt",
    "tav_vs_alt_mach",
    "fidle_vs_alt_mach",
    "ff_vs_thrust_alt_mach",
)

_SUPPORTED_GUIDANCE_MODES = frozenset({0, 3, 30, 33})
_SUPPORTED_CONTROL_MODES = frozenset({0, 40, 44, 46})
_SUPPORTED_EVENT_VARIABLES = frozenset(
    {
        "mguidance",
        "mcontrol",
        "wp_lonx",
        "wp_latx",
        "wp_alt",
        "psifgx",
        "thtfgx",
        "line_gain",
        "nl_gain_fact",
        "decrement",
        "wp_grdrange",
        "wp_flag",
        "altcom",
    }
)


class Cruise5SourceError(ValueError):
    """Source-bundle incompatibility with the CRUISE5 reconstruction."""


####


class Cruise5PropulsionConfig(CadacModel):
    """Source propulsion and fuel configuration."""

    mode: int
    mach_command: float = Field(ge=0.0)
    initial_mass_kg: float = Field(gt=0.0)
    initial_fuel_kg: float = Field(ge=0.0)
    mach_hold_gain_n: float = Field(ge=0.0)
    mach_hold_time_constant_s: float = Field(gt=0.0)
    thrust_command_n: float = Field(default=0.0, ge=0.0)


####


class Cruise5GuidanceConfig(CadacModel):
    """Source waypoint/line-guidance configuration."""

    mode: int
    line_gain_s_inv: float = Field(ge=0.0)
    nonlinear_gain_factor: float = Field(ge=0.0)
    decrement_m: float = Field(gt=0.0)
    waypoint_longitude_deg: float
    waypoint_latitude_deg: float
    waypoint_altitude_m: float = 0.0
    line_heading_deg: float = 0.0
    line_flight_path_deg: float = 0.0


####


class Cruise5ControlConfig(CadacModel):
    """Source pseudo-6DoF response-law configuration."""

    mode: int
    positive_load_limit_g: float
    negative_load_limit_g: float
    acceleration_gain_rad_s2: float = Field(ge=0.0)
    proportional_integral_ratio: float = Field(ge=0.0)
    positive_alpha_limit_deg: float
    negative_alpha_limit_deg: float
    lateral_roll_gain_rad: float = Field(ge=0.0)
    lateral_acceleration_limit_g: float = Field(ge=0.0)
    bank_limit_deg: float = Field(gt=0.0)
    bank_time_constant_s: float = Field(gt=0.0)
    altitude_command_m: float
    altitude_rate_limit_mps: float = Field(gt=0.0)
    altitude_gain_g_per_m: float = Field(ge=0.0)
    altitude_rate_gain_g_per_mps: float = Field(ge=0.0)


####


class Cruise5SourceDefinition(CadacModel):
    """Prepared single-vehicle CRUISE5 source case."""

    source_name: str = Field(min_length=1)
    source_model: str = "CRUISE3"
    integration_step_s: float = Field(gt=0.0)
    plot_step_s: float | None = Field(default=None, gt=0.0)
    end_time_s: float = Field(gt=0.0)
    module_order: tuple[str, ...]
    initial_state: CadacRound3InitialState
    initial_alpha_deg: float
    initial_bank_deg: float
    reference_area_m2: float = Field(gt=0.0)
    propulsion: Cruise5PropulsionConfig
    guidance: Cruise5GuidanceConfig
    control: Cruise5ControlConfig
    aerodynamic_deck: CadacDeck
    propulsion_deck: CadacDeck
    events: tuple[CadacEventBlock, ...] = ()
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)
    taoryx_tier: str = "pseudo_6dof"
    runtime_fidelity: str = "pseudo_6dof"
    control_realization: str = "response_law"

    @model_validator(mode="after")
    def validate_tables(self) -> "Cruise5SourceDefinition":
        aero = {table.name.casefold() for table in self.aerodynamic_deck.tables}
        prop = {table.name.casefold() for table in self.propulsion_deck.tables}
        missing = [name for name in _CRUISE5_AERO_TABLES if name.casefold() not in aero]
        missing.extend(name for name in _CRUISE5_PROP_TABLES if name.casefold() not in prop)
        if missing:
            raise ValueError("CRUISE5 source definition is missing required tables: " + ", ".join(missing))
        ####
        return self

    ####


####


class Cruise5Sample(CadacModel):
    """One source-ordered CRUISE5 truth/response sample."""

    time_s: float = Field(ge=0.0)
    longitude_deg: float
    latitude_deg: float
    altitude_m: float
    speed_mps: float = Field(ge=0.0)
    heading_deg: float
    flight_path_deg: float
    displacement_geographic_m: tuple[float, float, float]
    velocity_geographic_mps: tuple[float, float, float]
    specific_force_velocity_mps2: tuple[float, float, float]
    dynamic_pressure_pa: float = Field(ge=0.0)
    mach: float = Field(ge=0.0)
    propulsion_mode: int
    center_of_gravity_in: float
    thrust_n: float = Field(ge=0.0)
    mass_kg: float = Field(gt=0.0)
    fuel_mass_kg: float
    lift_to_drag: float
    alpha_deg: float
    bank_deg: float
    bank_command_deg: float
    load_command_g: float
    lateral_command_g: float
    waypoint_ground_range_m: float = Field(ge=0.0)
    waypoint_flag: int


####


class Cruise5EventTrace(CadacModel):
    """One sequential source event accepted before a CRUISE5 module pass."""

    event_index: int = Field(ge=0)
    time_s: float = Field(ge=0.0)
    source_line: int = Field(ge=1)
    watch_variable: str = Field(min_length=1)
    criterion: int | float
    updates: tuple[tuple[str, int | float], ...]


####


class Cruise5RunResult(CadacModel):
    """Source-compatible CRUISE5 batch result."""

    schema_id: str = "taoryx.cadac.cruise5-source-compatibility/v0alpha1"
    source_name: str = Field(min_length=1)
    integration_step_s: float = Field(gt=0.0)
    requested_end_time_s: float = Field(gt=0.0)
    executed_steps: int = Field(ge=0)
    terminated_reason: str = Field(min_length=1)
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)
    samples: tuple[Cruise5Sample, ...] = Field(min_length=1)
    events: tuple[Cruise5EventTrace, ...] = ()
    claim_boundary: str = (
        "Source-compatible CRUISE5 single-vehicle round/rotating-Earth 3-DoF translation with source turbojet, "
        "waypoint/line guidance, and lagged alpha/bank response. This is Taoryx pseudo-6DoF; no rigid-body moment closure is claimed."
    )


####


@dataclass(slots=True)
class _Cruise5Runtime:
    round3: CadacRound3RuntimeState
    values: dict[str, int | float]
    alpha_deg: float
    bank_deg: float
    phix_deg: float
    phixd_deg_s: float
    xi_rad_s: float
    xid_rad_s2: float
    alp_rad: float
    alpd_rad_s: float
    treqd_n_s: float
    treq_n: float
    fmassed_kg_s: float
    fmasse_kg: float
    fuel_mass_kg: float
    mass_kg: float
    specific_force_velocity_mps2: FloatVector
    lift_coefficient: float = 0.0
    drag_coefficient: float = 0.0
    lift_slope_per_deg: float = 0.0
    lift_to_drag: float = 0.0
    thrust_n: float = 0.0
    cg_in: float = 0.0
    propulsion_mode: int = 0
    waypoint_ground_range_m: float = 999_999.0
    waypoint_slant_range_m: float = 999_999.0
    waypoint_flag: int = 0
    bank_command_deg: float = 0.0
    load_command_g: float = 0.0
    lateral_command_g: float = 0.0


####


def lower_cruise5_source_bundle(bundle: CadacSourceBundle) -> Cruise5SourceDefinition:
    """Lower one single-vehicle CRUISE5 case into a typed pseudo-6DoF definition."""

    cruises = bundle.case.vehicles_named("CRUISE3")
    if len(cruises) != 1 or len(bundle.case.vehicles) != 1:
        raise Cruise5SourceError(f"CRUISE5 source lowering requires exactly one CRUISE3; found {len(cruises)}")
    ####
    module_order = tuple(module.name.casefold() for module in bundle.case.modules if CadacModuleStage.EXECUTE in module.stages)
    unknown = tuple(name for name in module_order if name not in _CRUISE5_MODULES)
    if unknown:
        raise Cruise5SourceError(f"CRUISE5 reconstruction does not implement source modules: {unknown!r}")
    ####
    missing = tuple(name for name in _CRUISE5_MODULES if name not in module_order)
    if missing:
        raise Cruise5SourceError(f"CRUISE5 reconstruction requires source modules: {missing!r}")
    ####
    timing = bundle.case.timing_values
    if "int_step" not in timing:
        raise Cruise5SourceError("CRUISE5 source case must declare TIMING int_step")
    ####
    vehicle = cruises[0]
    guidance_mode = _integer(vehicle, "mguidance", 0)
    control_mode = _integer(vehicle, "mcontrol", 0)
    _validate_supported_modes(guidance_mode, control_mode, vehicle.events)
    return Cruise5SourceDefinition(
        source_name=bundle.case.source_name,
        integration_step_s=timing["int_step"],
        plot_step_s=timing.get("plot_step"),
        end_time_s=bundle.case.end_time_s,
        module_order=module_order,
        initial_state=CadacRound3InitialState(
            longitude_deg=_number(vehicle, "lonx"),
            latitude_deg=_number(vehicle, "latx"),
            altitude_m=_number(vehicle, "alt"),
            speed_mps=_number(vehicle, "dvbe"),
            heading_deg=_number(vehicle, "psivgx"),
            flight_path_deg=_number(vehicle, "thtvgx"),
        ),
        initial_alpha_deg=_number(vehicle, "alphax", 0.0),
        initial_bank_deg=_number(vehicle, "phimvx", 0.0),
        reference_area_m2=_number(vehicle, "area", 0.929),
        propulsion=Cruise5PropulsionConfig(
            mode=_integer(vehicle, "mprop", 0),
            mach_command=_number(vehicle, "mach_com", 0.0),
            initial_mass_kg=_number(vehicle, "mass_init"),
            initial_fuel_kg=_number(vehicle, "fuel_init", 0.0),
            mach_hold_gain_n=_number(vehicle, "gfthm", 0.0),
            mach_hold_time_constant_s=_number(vehicle, "tfth", 1.0),
            thrust_command_n=_number(vehicle, "thrust_com", 0.0),
        ),
        guidance=Cruise5GuidanceConfig(
            mode=guidance_mode,
            line_gain_s_inv=_number(vehicle, "line_gain", 0.0),
            nonlinear_gain_factor=_number(vehicle, "nl_gain_fact", 1.0),
            decrement_m=_number(vehicle, "decrement", 1.0),
            waypoint_longitude_deg=_number(vehicle, "wp_lonx", 0.0),
            waypoint_latitude_deg=_number(vehicle, "wp_latx", 0.0),
            waypoint_altitude_m=_number(vehicle, "wp_alt", 0.0),
            line_heading_deg=_number(vehicle, "psifgx", 0.0),
            line_flight_path_deg=_number(vehicle, "thtfgx", 0.0),
        ),
        control=Cruise5ControlConfig(
            mode=control_mode,
            positive_load_limit_g=_number(vehicle, "anposlimx", 0.0),
            negative_load_limit_g=_number(vehicle, "anneglimx", 0.0),
            acceleration_gain_rad_s2=_number(vehicle, "gacp", 0.0),
            proportional_integral_ratio=_number(vehicle, "ta", 0.0),
            positive_alpha_limit_deg=_number(vehicle, "alpposlimx", 90.0),
            negative_alpha_limit_deg=_number(vehicle, "alpneglimx", -90.0),
            lateral_roll_gain_rad=_number(vehicle, "gcp", 0.0),
            lateral_acceleration_limit_g=_number(vehicle, "allimx", 0.0),
            bank_limit_deg=_number(vehicle, "philimx", 90.0),
            bank_time_constant_s=_number(vehicle, "tphi", 1.0),
            altitude_command_m=_number(vehicle, "altcom", 0.0),
            altitude_rate_limit_mps=_number(vehicle, "altdlim", 1.0),
            altitude_gain_g_per_m=_number(vehicle, "gh", 0.0),
            altitude_rate_gain_g_per_mps=_number(vehicle, "gv", 0.0),
        ),
        aerodynamic_deck=bundle.deck_for("CRUISE3", CadacDeckKind.AERODYNAMIC),
        propulsion_deck=bundle.deck_for("CRUISE3", CadacDeckKind.PROPULSION),
        events=vehicle.events,
        source_artifacts=bundle.artifacts,
    )


####


def load_cruise5_source_definition(path: str | Path) -> Cruise5SourceDefinition:
    """Parse, fingerprint, and lower one CRUISE5 source case."""

    return lower_cruise5_source_bundle(load_cadac_source_bundle(path))


####


def run_cruise5_source_compatibility(
    definition: Cruise5SourceDefinition,
    *,
    end_time_s: float | None = None,
    sample_step_s: float | None = None,
) -> Cruise5RunResult:
    """Execute the source module order for the single-vehicle CRUISE5 waypoint case."""

    dt_s = definition.integration_step_s
    requested_end = definition.end_time_s if end_time_s is None else float(end_time_s)
    if not math.isfinite(requested_end) or requested_end <= 0.0:
        raise ValueError("CRUISE5 end_time_s must be positive and finite")
    ####
    cadence = (definition.plot_step_s or dt_s) if sample_step_s is None else float(sample_step_s)
    if not math.isfinite(cadence) or cadence <= 0.0:
        raise ValueError("CRUISE5 sample_step_s must be positive and finite")
    ####
    runtime = _initialize_runtime(definition)
    event_cursor = CadacEventCursor.from_events(definition.events)
    samples: list[Cruise5Sample] = []
    event_trace: list[Cruise5EventTrace] = []
    sim_time = 0.0
    next_sample = 0.0
    steps = 0
    terminated_reason = "end_time"
    while sim_time <= requested_end + 0.5 * dt_s:
        steps += 1
        runtime.round3.time_s = sim_time
        application = event_cursor.evaluate_and_apply(runtime.values)
        if application is not None:
            event_trace.append(
                Cruise5EventTrace(
                    event_index=application.event_index,
                    time_s=sim_time,
                    source_line=application.source_line,
                    watch_variable=application.watch_variable,
                    criterion=application.criterion,
                    updates=application.updated_values,
                )
            )
        ####
        environment = cadac_round3_environment(runtime.round3)
        for module in definition.module_order:
            if module == "environment":
                environment = cadac_round3_environment(runtime.round3)
            elif module == "aerodynamics":
                _aerodynamics(definition, runtime, environment.mach)
            elif module == "propulsion":
                _propulsion(definition, runtime, environment, dt_s)
            elif module == "forces":
                _forces(definition, runtime, environment.dynamic_pressure_pa)
            elif module == "newton":
                cadac_round3_newton_step(
                    runtime.round3,
                    runtime.specific_force_velocity_mps2,
                    environment.gravity_mps2,
                    dt_s,
                )
            elif module == "guidance":
                _guidance(definition, runtime, environment.gravity_mps2)
            elif module == "control":
                _control(definition, runtime, environment, dt_s)
            elif module == "intercept":
                if _terminal_impact(runtime):
                    terminated_reason = "terminal_waypoint_altitude"
                ####
            ####
        ####
        if sim_time + 0.5 * dt_s >= next_sample or terminated_reason != "end_time":
            samples.append(_sample(sim_time, runtime, environment))
            while next_sample <= sim_time + 0.5 * dt_s:
                next_sample += cadence
            ####
        ####
        if terminated_reason != "end_time":
            break
        ####
        if not _runtime_is_finite(runtime):
            terminated_reason = "nonfinite_state"
            break
        ####
        sim_time += dt_s
    ####
    return Cruise5RunResult(
        source_name=definition.source_name,
        integration_step_s=dt_s,
        requested_end_time_s=requested_end,
        executed_steps=steps,
        terminated_reason=terminated_reason,
        source_artifacts=definition.source_artifacts,
        samples=tuple(samples),
        events=tuple(event_trace),
    )


####


def _initialize_runtime(definition: Cruise5SourceDefinition) -> _Cruise5Runtime:
    values: dict[str, int | float] = {
        "mguidance": definition.guidance.mode,
        "mcontrol": definition.control.mode,
        "wp_lonx": definition.guidance.waypoint_longitude_deg,
        "wp_latx": definition.guidance.waypoint_latitude_deg,
        "wp_alt": definition.guidance.waypoint_altitude_m,
        "psifgx": definition.guidance.line_heading_deg,
        "thtfgx": definition.guidance.line_flight_path_deg,
        "line_gain": definition.guidance.line_gain_s_inv,
        "nl_gain_fact": definition.guidance.nonlinear_gain_factor,
        "decrement": definition.guidance.decrement_m,
        "wp_grdrange": 999_999.0,
        "wp_flag": 0,
        "altcom": definition.control.altitude_command_m,
    }
    return _Cruise5Runtime(
        round3=cadac_round3_initialize(definition.initial_state),
        values=values,
        alpha_deg=definition.initial_alpha_deg,
        bank_deg=definition.initial_bank_deg,
        phix_deg=0.0,
        phixd_deg_s=0.0,
        xi_rad_s=0.0,
        xid_rad_s2=0.0,
        alp_rad=0.0,
        alpd_rad_s=0.0,
        treqd_n_s=0.0,
        treq_n=0.0,
        fmassed_kg_s=0.0,
        fmasse_kg=0.0,
        fuel_mass_kg=0.0,
        mass_kg=definition.propulsion.initial_mass_kg,
        specific_force_velocity_mps2=np.zeros(3, dtype=np.float64),
        propulsion_mode=definition.propulsion.mode,
    )


####


def _aerodynamics(definition: Cruise5SourceDefinition, runtime: _Cruise5Runtime, mach: float) -> None:
    deck = definition.aerodynamic_deck
    cd0 = deck.table("cd0_vs_mach").interpolate((mach,))
    cl0 = deck.table("cl0_vs_mach").interpolate((mach,))
    ckk = deck.table("ckk_vs_mach").interpolate((mach,))
    cla = deck.table("cla_vs_mach").interpolate((mach,))
    cla0 = deck.table("cla0_vs_mach").interpolate((mach,))
    lift = cla0 + cla * runtime.alpha_deg
    drag = cd0 + ckk * (lift - cl0) ** 2
    runtime.lift_coefficient = lift
    runtime.drag_coefficient = drag
    runtime.lift_slope_per_deg = cla
    runtime.lift_to_drag = lift / drag


####


def _propulsion(
    definition: Cruise5SourceDefinition,
    runtime: _Cruise5Runtime,
    environment: CadacRound3Environment,
    dt_s: float,
) -> None:
    mach = environment.mach
    pdynmc = environment.dynamic_pressure_pa
    altitude = runtime.round3.altitude_m
    deck = definition.propulsion_deck
    mode = runtime.propulsion_mode
    if mode == 0:
        runtime.thrust_n = 0.0
        runtime.cg_in = deck.table("cg_vs_mass").interpolate((runtime.mass_kg,))
        return
    ####
    idle = deck.table("fidle_vs_alt_mach").interpolate((altitude, mach))
    available = deck.table("tav_vs_alt_mach").interpolate((altitude, mach))
    fuel_flow = 0.0
    if mode == 1:
        thrust = definition.propulsion.thrust_command_n
        fuel_flow = deck.table("ff_vs_thrust_alt_mach").interpolate((thrust, altitude, mach))
        runtime.treq_n = thrust
    elif mode == 2:
        thrust = idle
        fuel_flow = deck.table("iff_vs_alt").interpolate((altitude,))
    elif mode == 3:
        thrust = available
        fuel_flow = deck.table("ff_vs_thrust_alt_mach").interpolate((thrust, altitude, mach))
    else:
        mode = 4
        required_stability = runtime.drag_coefficient * pdynmc * definition.reference_area_m2
        error = definition.propulsion.mach_command - mach
        command = error * definition.propulsion.mach_hold_gain_n + required_stability
        derivative_new = (command - 2.0 * runtime.treq_n) / definition.propulsion.mach_hold_time_constant_s
        runtime.treq_n = _integrate(runtime.treq_n, derivative_new, runtime.treqd_n_s, dt_s)
        runtime.treqd_n_s = derivative_new
        thrust_body = runtime.treq_n / max(1.0e-12, math.cos(runtime.alpha_deg * RAD_PER_DEG))
        if thrust_body < idle:
            mode = 5
            thrust_body = idle
        ####
        if thrust_body > available:
            mode = 6
            thrust_body = available
        ####
        thrust = thrust_body
        fuel_flow = deck.table("ff_vs_thrust_alt_mach").interpolate((thrust, altitude, mach))
    ####
    fmassed_new = fuel_flow
    runtime.fmasse_kg = _integrate(runtime.fmasse_kg, fmassed_new, runtime.fmassed_kg_s, dt_s)
    runtime.fmassed_kg_s = fmassed_new
    runtime.mass_kg = definition.propulsion.initial_mass_kg - runtime.fmasse_kg
    runtime.fuel_mass_kg = definition.propulsion.initial_fuel_kg - runtime.fmasse_kg
    runtime.cg_in = deck.table("cg_vs_mass").interpolate((runtime.mass_kg,))
    if runtime.fuel_mass_kg <= 0.0:
        thrust = 0.0
    ####
    runtime.propulsion_mode = mode
    runtime.thrust_n = max(0.0, thrust)


####


def _forces(definition: Cruise5SourceDefinition, runtime: _Cruise5Runtime, dynamic_pressure_pa: float) -> None:
    alpha = runtime.alpha_deg * RAD_PER_DEG
    bank = runtime.bank_deg * RAD_PER_DEG
    aero_force = dynamic_pressure_pa * definition.reference_area_m2
    normal = aero_force * runtime.lift_coefficient + runtime.thrust_n * math.sin(alpha)
    runtime.specific_force_velocity_mps2 = np.asarray(
        (
            (-aero_force * runtime.drag_coefficient + runtime.thrust_n * math.cos(alpha)) / runtime.mass_kg,
            math.sin(bank) * normal / runtime.mass_kg,
            -math.cos(bank) * normal / runtime.mass_kg,
        ),
        dtype=np.float64,
    )


####


def _guidance(definition: Cruise5SourceDefinition, runtime: _Cruise5Runtime, gravity_mps2: float) -> None:
    mode = int(runtime.values["mguidance"])
    if mode not in _SUPPORTED_GUIDANCE_MODES:
        raise Cruise5SourceError(f"CRUISE5 waypoint/line runtime supports guidance modes {sorted(_SUPPORTED_GUIDANCE_MODES)}; got {mode}")
    ####
    ancom = 0.0
    alcom = 0.0
    if mode != 0:
        algv = _guidance_line(definition, runtime, gravity_mps2)
        if mode in {30, 33}:
            alcom = float(algv[1]) / gravity_mps2
        ####
        if mode in {3, 33}:
            ancom = -float(algv[2]) / gravity_mps2
        ####
    ####
    ancom = min(definition.control.positive_load_limit_g, max(definition.control.negative_load_limit_g, ancom))
    alcom = min(definition.control.lateral_acceleration_limit_g, max(-definition.control.lateral_acceleration_limit_g, alcom))
    runtime.load_command_g = ancom
    runtime.lateral_command_g = alcom


####


def _guidance_line(definition: Cruise5SourceDefinition, runtime: _Cruise5Runtime, gravity_mps2: float) -> FloatVector:
    line_heading = float(runtime.values["psifgx"]) * RAD_PER_DEG
    line_flight_path = float(runtime.values["thtfgx"]) * RAD_PER_DEG
    tfg = cadac_mat2tr(line_heading, line_flight_path)
    waypoint = cadac_cadine(
        float(runtime.values["wp_lonx"]) * RAD_PER_DEG,
        float(runtime.values["wp_latx"]) * RAD_PER_DEG,
        float(runtime.values["wp_alt"]),
        runtime.round3.time_s,
    )
    swbg = runtime.round3.tig.T @ (waypoint - runtime.round3.sbii_m)
    slant, los_heading, los_flight_path = _polar(swbg)
    tog = cadac_mat2tr(los_heading, los_flight_path)
    ground = math.hypot(float(swbg[0]), float(swbg[1]))
    vbeo = tog @ runtime.round3.vbeg_mps
    vbef = tfg @ runtime.round3.vbeg_mps
    nl_gain = float(runtime.values["nl_gain_fact"]) * (1.0 - math.exp(-slant / float(runtime.values["decrement"])))
    line_gain = float(runtime.values["line_gain"])
    algv = np.asarray(
        (
            gravity_mps2 * math.sin(runtime.round3.flight_path_rad),
            line_gain * (-float(vbeo[1]) + nl_gain * float(vbef[1])),
            line_gain * (-float(vbeo[2]) + nl_gain * float(vbef[2])) - gravity_mps2 * math.cos(runtime.round3.flight_path_rad),
        ),
        dtype=np.float64,
    )
    speed = float(np.linalg.norm(runtime.round3.vbeg_mps))
    radius_min = speed * speed / (gravity_mps2 * math.tan(definition.control.bank_limit_deg * RAD_PER_DEG))
    if ground < 2.0 * radius_min:
        horizontal_velocity = np.asarray((runtime.round3.vbeg_mps[0], runtime.round3.vbeg_mps[1], 0.0), dtype=np.float64)
        horizontal_displacement = np.asarray((swbg[0], swbg[1], 0.0), dtype=np.float64)
        runtime.waypoint_flag = _sign(float(horizontal_velocity @ horizontal_displacement))
    else:
        runtime.waypoint_flag = 0
    ####
    runtime.waypoint_ground_range_m = ground
    runtime.waypoint_slant_range_m = slant
    runtime.values["wp_grdrange"] = ground
    runtime.values["wp_flag"] = runtime.waypoint_flag
    return algv


####


def _control(
    definition: Cruise5SourceDefinition,
    runtime: _Cruise5Runtime,
    environment: CadacRound3Environment,
    dt_s: float,
) -> None:
    mode = int(runtime.values["mcontrol"])
    previous_alpha = runtime.alpha_deg
    previous_bank = runtime.bank_deg
    phic = runtime.bank_command_deg
    alpha = previous_alpha
    bank = previous_bank
    if mode == 0:
        alpha = 0.0
        bank = 0.0
    elif mode == 40:
        phic = _control_lateral(definition, runtime, runtime.lateral_command_g, environment.gravity_mps2, previous_alpha, previous_bank)
        bank = _control_bank(definition, runtime, phic, dt_s)
    elif mode == 44:
        phic = _control_lateral(definition, runtime, runtime.lateral_command_g, environment.gravity_mps2, previous_alpha, previous_bank)
        bank = _control_bank(definition, runtime, phic, dt_s)
        alpha = _control_load(definition, runtime, runtime.load_command_g, environment, dt_s, previous_alpha, previous_bank)
    elif mode == 46:
        phic = _control_lateral(definition, runtime, runtime.lateral_command_g, environment.gravity_mps2, previous_alpha, previous_bank)
        bank = _control_bank(definition, runtime, phic, dt_s)
        runtime.load_command_g = _control_altitude(
            definition,
            runtime,
            float(runtime.values["altcom"]),
            bank,
            environment.gravity_mps2,
        )
        alpha = _control_load(definition, runtime, runtime.load_command_g, environment, dt_s, previous_alpha, previous_bank)
    else:
        raise Cruise5SourceError(f"CRUISE5 waypoint/line runtime supports control modes {sorted(_SUPPORTED_CONTROL_MODES)}; got {mode}")
    ####
    runtime.bank_command_deg = phic
    runtime.bank_deg = bank
    runtime.alpha_deg = alpha


####


def _control_bank(definition: Cruise5SourceDefinition, runtime: _Cruise5Runtime, command_deg: float, dt_s: float) -> float:
    command = min(definition.control.bank_limit_deg, max(-definition.control.bank_limit_deg, command_deg))
    derivative_new = (command - runtime.phix_deg) / definition.control.bank_time_constant_s
    runtime.phix_deg = _integrate(runtime.phix_deg, derivative_new, runtime.phixd_deg_s, dt_s)
    runtime.phixd_deg_s = derivative_new
    return runtime.phix_deg


####


def _control_lateral(
    definition: Cruise5SourceDefinition,
    runtime: _Cruise5Runtime,
    command_g: float,
    gravity_mps2: float,
    alpha_deg: float,
    bank_deg: float,
) -> float:
    tbv = cadac_cadtbv(bank_deg * RAD_PER_DEG, alpha_deg * RAD_PER_DEG)
    fspb = tbv @ runtime.specific_force_velocity_mps2
    normal_load = -float(fspb[2]) / gravity_mps2
    command = min(definition.control.lateral_acceleration_limit_g, max(-definition.control.lateral_acceleration_limit_g, command_g))
    sign = 1.0 if normal_load >= 0.0 else -1.0
    return definition.control.lateral_roll_gain_rad * sign / (abs(normal_load) + 0.001) * command * DEG_PER_RAD


####


def _control_altitude(
    definition: Cruise5SourceDefinition,
    runtime: _Cruise5Runtime,
    altitude_command_m: float,
    bank_deg: float,
    gravity_mps2: float,
) -> float:
    altitude_rate_command = definition.control.altitude_gain_g_per_m * (altitude_command_m - runtime.round3.altitude_m)
    limit = definition.control.altitude_rate_limit_mps
    altitude_rate_command = min(limit, max(-limit, altitude_rate_command))
    altitude_rate = -float(runtime.round3.vbeg_mps[2])
    command = (definition.control.altitude_rate_gain_g_per_mps * (altitude_rate_command - altitude_rate) / gravity_mps2 + 1.0) / math.cos(
        bank_deg * RAD_PER_DEG
    )
    return min(definition.control.positive_load_limit_g, max(definition.control.negative_load_limit_g, command))


####


def _control_load(
    definition: Cruise5SourceDefinition,
    runtime: _Cruise5Runtime,
    command_g: float,
    environment: CadacRound3Environment,
    dt_s: float,
    alpha_deg: float,
    bank_deg: float,
) -> float:
    gravity = environment.gravity_mps2
    pdynmc = environment.dynamic_pressure_pa
    tbv = cadac_cadtbv(bank_deg * RAD_PER_DEG, alpha_deg * RAD_PER_DEG)
    fspb = tbv @ runtime.specific_force_velocity_mps2
    command = min(definition.control.positive_load_limit_g, max(definition.control.negative_load_limit_g, command_g))
    normal_load = -float(fspb[2]) / gravity
    error = command - normal_load
    denominator = pdynmc * definition.reference_area_m2 * runtime.lift_slope_per_deg / RAD_PER_DEG + runtime.thrust_n
    tip = runtime.round3.speed_mps * runtime.mass_kg / denominator
    gr = 0.0
    if definition.control.proportional_integral_ratio > 0.0:
        gr = definition.control.acceleration_gain_rad_s2 * tip / runtime.round3.speed_mps
        gi = gr / definition.control.proportional_integral_ratio
        xid_new = gi * error
        runtime.xi_rad_s = _integrate(runtime.xi_rad_s, xid_new, runtime.xid_rad_s2, dt_s)
        runtime.xid_rad_s2 = xid_new
    else:
        runtime.xi_rad_s = 0.0
    ####
    pitch_rate = gr * error + runtime.xi_rad_s
    alpd_new = pitch_rate - runtime.alp_rad / tip
    runtime.alp_rad = _integrate(runtime.alp_rad, alpd_new, runtime.alpd_rad_s, dt_s)
    runtime.alpd_rad_s = alpd_new
    alpha = runtime.alp_rad * DEG_PER_RAD
    return min(definition.control.positive_alpha_limit_deg, max(definition.control.negative_alpha_limit_deg, alpha))


####


def _terminal_impact(runtime: _Cruise5Runtime) -> bool:
    mode = int(runtime.values["mguidance"])
    return mode in {33, 43} and runtime.round3.altitude_m <= float(runtime.values["wp_alt"])


####


def _sample(time_s: float, runtime: _Cruise5Runtime, environment: CadacRound3Environment) -> Cruise5Sample:
    return Cruise5Sample(
        time_s=time_s,
        longitude_deg=runtime.round3.longitude_rad * DEG_PER_RAD,
        latitude_deg=runtime.round3.latitude_rad * DEG_PER_RAD,
        altitude_m=runtime.round3.altitude_m,
        speed_mps=runtime.round3.speed_mps,
        heading_deg=runtime.round3.heading_rad * DEG_PER_RAD,
        flight_path_deg=runtime.round3.flight_path_rad * DEG_PER_RAD,
        displacement_geographic_m=_tuple3(runtime.round3.sbeg_m),
        velocity_geographic_mps=_tuple3(runtime.round3.vbeg_mps),
        specific_force_velocity_mps2=_tuple3(runtime.specific_force_velocity_mps2),
        dynamic_pressure_pa=environment.dynamic_pressure_pa,
        mach=environment.mach,
        propulsion_mode=runtime.propulsion_mode,
        center_of_gravity_in=runtime.cg_in,
        thrust_n=runtime.thrust_n,
        mass_kg=runtime.mass_kg,
        fuel_mass_kg=runtime.fuel_mass_kg,
        lift_to_drag=runtime.lift_to_drag,
        alpha_deg=runtime.alpha_deg,
        bank_deg=runtime.bank_deg,
        bank_command_deg=runtime.bank_command_deg,
        load_command_g=runtime.load_command_g,
        lateral_command_g=runtime.lateral_command_g,
        waypoint_ground_range_m=runtime.waypoint_ground_range_m,
        waypoint_flag=runtime.waypoint_flag,
    )


####


def _polar(vector: FloatVector) -> tuple[float, float, float]:
    x, y, z = (float(value) for value in vector)
    magnitude = math.sqrt(x * x + y * y + z * z)
    azimuth = math.atan2(y, x)
    horizontal = math.hypot(x, y)
    elevation = math.atan2(-z, horizontal) if horizontal > 0.0 else 0.0
    return magnitude, azimuth, elevation


####


def _sign(value: float) -> int:
    return -1 if value < 0.0 else 1


####


def _integrate(value: float, new_rate: float, old_rate: float, dt_s: float) -> float:
    return cadac_stored_derivative_step((value,), (new_rate,), (old_rate,), dt_s)[0]


####


def _validate_supported_modes(
    guidance_mode: int,
    control_mode: int,
    events: tuple[CadacEventBlock, ...],
) -> None:
    if guidance_mode not in _SUPPORTED_GUIDANCE_MODES:
        raise Cruise5SourceError(f"CRUISE5 waypoint/line runtime supports guidance modes {sorted(_SUPPORTED_GUIDANCE_MODES)}; got {guidance_mode}")
    ####
    if control_mode not in _SUPPORTED_CONTROL_MODES:
        raise Cruise5SourceError(f"CRUISE5 waypoint/line runtime supports control modes {sorted(_SUPPORTED_CONTROL_MODES)}; got {control_mode}")
    ####
    for event in events:
        watch = event.condition.variable.casefold()
        if watch not in _SUPPORTED_EVENT_VARIABLES:
            raise Cruise5SourceError(f"CRUISE5 event at source line {event.source_line} watches unsupported runtime variable {event.condition.variable!r}")
        ####
        for assignment in event.assignments:
            key = assignment.name.casefold()
            if key not in _SUPPORTED_EVENT_VARIABLES:
                raise Cruise5SourceError(f"CRUISE5 event at source line {event.source_line} mutates unsupported runtime variable {assignment.name!r}")
            ####
            if key == "mguidance":
                value = int(assignment.value) if isinstance(assignment.value, (int, float)) else -999_999
                if value not in _SUPPORTED_GUIDANCE_MODES:
                    raise Cruise5SourceError(f"CRUISE5 event at source line {event.source_line} selects unsupported guidance mode {assignment.value!r}")
                ####
            elif key == "mcontrol":
                value = int(assignment.value) if isinstance(assignment.value, (int, float)) else -999_999
                if value not in _SUPPORTED_CONTROL_MODES:
                    raise Cruise5SourceError(f"CRUISE5 event at source line {event.source_line} selects unsupported control mode {assignment.value!r}")
                ####
            ####
        ####
    ####


####


def _number(vehicle: CadacVehicleBlock, name: str, default: float | None = None) -> float:
    try:
        value = vehicle.parameter(name)
    except KeyError:
        if default is None:
            raise Cruise5SourceError(f"CRUISE5 source vehicle is missing required parameter {name!r}") from None
        ####
        return float(default)
    ####
    if not isinstance(value, (int, float)):
        raise Cruise5SourceError(f"CRUISE5 parameter {name!r} must be numeric")
    ####
    return float(value)


####


def _integer(vehicle: CadacVehicleBlock, name: str, default: int | None = None) -> int:
    value = _number(vehicle, name, None if default is None else float(default))
    if not value.is_integer():
        raise Cruise5SourceError(f"CRUISE5 parameter {name!r} must be integer-valued")
    ####
    return int(value)


####


def _tuple3(vector: FloatVector) -> tuple[float, float, float]:
    return (float(vector[0]), float(vector[1]), float(vector[2]))


####


def _runtime_is_finite(runtime: _Cruise5Runtime) -> bool:
    scalars = (
        runtime.round3.altitude_m,
        runtime.round3.speed_mps,
        runtime.alpha_deg,
        runtime.bank_deg,
        runtime.mass_kg,
        runtime.thrust_n,
    )
    arrays = (
        runtime.round3.sbeg_m,
        runtime.round3.vbeg_mps,
        runtime.round3.sbii_m,
        runtime.round3.vbii_mps,
        runtime.specific_force_velocity_mps2,
    )
    return all(math.isfinite(value) for value in scalars) and all(np.all(np.isfinite(array)) for array in arrays)


####


__all__ = [
    "Cruise5ControlConfig",
    "Cruise5EventTrace",
    "Cruise5GuidanceConfig",
    "Cruise5PropulsionConfig",
    "Cruise5RunResult",
    "Cruise5Sample",
    "Cruise5SourceDefinition",
    "Cruise5SourceError",
    "load_cruise5_source_definition",
    "lower_cruise5_source_bundle",
    "run_cruise5_source_compatibility",
]
