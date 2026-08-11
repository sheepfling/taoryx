"""Source-grounded FALCON6 lowering and physical-effector plant kernels."""

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
from .input_ast import CadacDeckKind, CadacEventBlock, CadacModel, CadacModuleStage, CadacVehicleBlock
from .source_environment import atmosphere76, cadac_source_inverse_square_gravity_mps2

FloatVector: TypeAlias = NDArray[np.float64]
FloatMatrix: TypeAlias = NDArray[np.float64]

_RAD_PER_DEG = math.pi / 180.0
_DEG_PER_RAD = 180.0 / math.pi
_LBF_TO_N = 4.448

_FALCON6_MODULES = (
    "environment",
    "kinematics",
    "aerodynamics",
    "propulsion",
    "forces",
    "guidance",
    "control",
    "actuator",
    "euler",
    "newton",
)
_FALCON6_AERO_TABLES = (
    "cx_vs_elev_alpha",
    "cxq_vs_alpha",
    "cyr_vs_alpha",
    "cyp_vs_alpha",
    "cz_vs_alpha",
    "czq_vs_alpha",
    "cl_vs_beta_alpha",
    "cldr_vs_beta_alpha",
    "clda_vs_beta_alpha",
    "clr_vs_alpha",
    "clp_vs_alpha",
    "cm_vs_elev_alpha",
    "cmq_vs_alpha",
    "cn_vs_beta_alpha",
    "cnda_vs_beta_alpha",
    "cndr_vs_beta_alpha",
    "cnr_vs_alpha",
    "cnp_vs_alpha",
)
_FALCON6_PROP_TABLES = (
    "idle_vs_mach_alt",
    "mil_vs_mach_alt",
    "max_vs_mach_alt",
    "ff_vs_thrust_alt_mach",
)


class Falcon6SourceError(ValueError):
    """Source-bundle incompatibility with the FALCON6 reconstruction."""


####


class Falcon6InitialState(CadacModel):
    """Source initial truth state for one FALCON6 ``PLANE6`` actor."""

    position_ned_m: tuple[float, float, float]
    speed_mps: float = Field(gt=0.0)
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    alpha_deg: float
    beta_deg: float
    body_rates_deg_s: tuple[float, float, float] = (0.0, 0.0, 0.0)


####


class Falcon6AirframeConstants(CadacModel):
    """Fixed source-owned geometry, mass, inertia, and engine momentum."""

    reference_area_m2: float = 27.87
    reference_span_m: float = 9.14
    reference_chord_m: float = 3.45
    mass_kg: float = 9496.0
    inertia_kg_m2: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ] = (
        (12_875.0, 0.0, -1_331.4),
        (0.0, 75_673.0, 0.0),
        (-1_331.4, 0.0, 85_551.0),
    )
    engine_angular_momentum_kg_m2_s: float = 70_000.0

    @model_validator(mode="after")
    def validate_airframe(self) -> "Falcon6AirframeConstants":
        if any(value <= 0.0 for value in (self.reference_area_m2, self.reference_span_m, self.reference_chord_m, self.mass_kg)):
            raise ValueError("FALCON6 source airframe scalars must be positive")
        ####
        inertia = np.asarray(self.inertia_kg_m2, dtype=np.float64)
        if inertia.shape != (3, 3) or not np.all(np.isfinite(inertia)):
            raise ValueError("FALCON6 source inertia must be a finite 3x3 matrix")
        ####
        if not np.allclose(inertia, inertia.T, atol=0.0, rtol=0.0):
            raise ValueError("FALCON6 source inertia must be symmetric")
        ####
        if np.min(np.linalg.eigvalsh(inertia)) <= 0.0:
            raise ValueError("FALCON6 source inertia must be positive definite")
        ####
        return self

    ####


####


class Falcon6ActuatorConfig(CadacModel):
    """Physical aileron/elevator/rudder actuator configuration."""

    mode: int
    position_limit_deg: float = Field(gt=0.0)
    rate_limit_deg_s: float = Field(gt=0.0)
    natural_frequency_rad_s: float = Field(gt=0.0)
    damping_ratio: float = Field(ge=0.0)

    @model_validator(mode="after")
    def validate_mode(self) -> "Falcon6ActuatorConfig":
        if self.mode not in {0, 2}:
            raise ValueError("FALCON6 actuator mode must be 0 (ideal) or 2 (second order); source mode 1 is not implemented")
        ####
        return self

    ####


####


class Falcon6ControlLimits(CadacModel):
    """Source control and structural limits used by FALCON6."""

    autopilot_mode: int
    aileron_limit_deg: float = Field(gt=0.0)
    elevator_limit_deg: float = Field(gt=0.0)
    rudder_limit_deg: float = Field(gt=0.0)
    positive_load_limit_g: float = Field(gt=0.0)
    negative_load_limit_g: float = Field(gt=0.0)
    bank_limit_deg: float = Field(gt=0.0)


####


class Falcon6PropulsionConfig(CadacModel):
    """Source F-16 engine command configuration."""

    mode: int
    mach_command: float = Field(ge=0.0)
    throttle: float = Field(default=0.0, ge=0.0, le=1.0)
    mach_gain: float = Field(ge=0.0)

    @model_validator(mode="after")
    def validate_mode(self) -> "Falcon6PropulsionConfig":
        if self.mode not in {0, 1, 2}:
            raise ValueError("FALCON6 propulsion mode must be 0, 1, or 2")
        ####
        return self

    ####


####


class Falcon6AeroLimits(CadacModel):
    """Source aerodynamic limits and center-of-gravity locations."""

    positive_alpha_limit_deg: float
    negative_alpha_limit_deg: float
    reference_cg_m: float
    actual_cg_m: float


####


class Falcon6SourceDefinition(CadacModel):
    """Prepared FALCON6 source case before a full batch runtime is promoted."""

    source_name: str = Field(min_length=1)
    integration_step_s: float = Field(gt=0.0)
    plot_step_s: float | None = Field(default=None, gt=0.0)
    end_time_s: float = Field(gt=0.0)
    module_order: tuple[str, ...]
    initial_state: Falcon6InitialState
    airframe: Falcon6AirframeConstants = Field(default_factory=Falcon6AirframeConstants)
    aerodynamics: Falcon6AeroLimits
    propulsion: Falcon6PropulsionConfig
    actuator: Falcon6ActuatorConfig
    control_limits: Falcon6ControlLimits
    aerodynamic_deck: CadacDeck
    propulsion_deck: CadacDeck
    events: tuple[CadacEventBlock, ...] = ()
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)
    taoryx_tier: str = "rigid_body_6dof_surface_allocated"
    runtime_fidelity: str = "rigid_body_6dof"
    control_realization: str = "effector_allocated"

    @model_validator(mode="after")
    def validate_source_definition(self) -> "Falcon6SourceDefinition":
        aero_names = {table.name.casefold() for table in self.aerodynamic_deck.tables}
        prop_names = {table.name.casefold() for table in self.propulsion_deck.tables}
        missing_aero = tuple(name for name in _FALCON6_AERO_TABLES if name.casefold() not in aero_names)
        missing_prop = tuple(name for name in _FALCON6_PROP_TABLES if name.casefold() not in prop_names)
        if missing_aero or missing_prop:
            raise ValueError("FALCON6 source definition is missing required tables: " + ", ".join((*missing_aero, *missing_prop)))
        ####
        return self

    ####


####


class Falcon6SurfaceCommand(CadacModel):
    """Requested or achieved physical control-surface positions."""

    aileron_deg: float = 0.0
    elevator_deg: float = 0.0
    rudder_deg: float = 0.0

    def vector(self) -> tuple[float, float, float]:
        return (self.aileron_deg, self.elevator_deg, self.rudder_deg)

    ####


####


class Falcon6ActuatorState(CadacModel):
    """Stored-derivative source actuator state for three physical surfaces."""

    position_derivative_deg_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    position_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rate_derivative_deg_s2: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rate_deg_s: tuple[float, float, float] = (0.0, 0.0, 0.0)


####


class Falcon6ActuatorStep(CadacModel):
    """One source-compatible actuator transition and its saturation evidence."""

    requested: Falcon6SurfaceCommand
    achieved: Falcon6SurfaceCommand
    state: Falcon6ActuatorState
    position_limited: tuple[bool, bool, bool]
    rate_limited: tuple[bool, bool, bool]


####


class Falcon6EngineState(CadacModel):
    """Source first-order power-spool state."""

    power_derivative_percent_s: float = 0.0
    power_percent: float = 0.0


####


class Falcon6PropulsionStep(CadacModel):
    """One FALCON6 engine transition."""

    throttle: float = Field(ge=0.0, le=1.0)
    commanded_power_percent: float
    achieved_power_percent: float
    spool_time_constant_s: float = Field(gt=0.0)
    idle_thrust_n: float
    military_thrust_n: float
    maximum_thrust_n: float
    thrust_n: float
    state: Falcon6EngineState


####


class Falcon6AeroState(CadacModel):
    """Instantaneous state consumed by the source aerodynamic coefficient closure."""

    alpha_deg: float
    beta_deg: float
    speed_mps: float = Field(gt=0.0)
    body_rates_deg_s: tuple[float, float, float]


####


class Falcon6AeroCoefficients(CadacModel):
    """Source total body-axis force and moment coefficients."""

    cx: float
    cy: float
    cz: float
    cl: float
    cm: float
    cn: float


####


class Falcon6BodyWrench(CadacModel):
    """Non-gravitational body force and aerodynamic body moment."""

    force_n: tuple[float, float, float]
    moment_nm: tuple[float, float, float]


####


class Falcon6RotationalDerivative(CadacModel):
    """Source rigid-body rotational acceleration at one operating point."""

    body_rates_rad_s: tuple[float, float, float]
    angular_acceleration_rad_s2: tuple[float, float, float]


####


class Falcon6DirectPlantCommand(CadacModel):
    """Direct physical-effector command used by the first runnable FALCON6 plant slice."""

    surfaces: Falcon6SurfaceCommand = Field(default_factory=Falcon6SurfaceCommand)
    throttle_override: float | None = Field(default=None, ge=0.0, le=1.0)


####


class Falcon6PlantSample(CadacModel):
    """One source-ordered FALCON6 physical-plant truth sample."""

    time_s: float = Field(ge=0.0)
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]
    velocity_body_mps: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    body_rates_rad_s: tuple[float, float, float]
    alpha_deg: float
    beta_deg: float
    mach: float = Field(ge=0.0)
    dynamic_pressure_pa: float = Field(ge=0.0)
    altitude_m: float
    requested_surfaces_deg: tuple[float, float, float]
    achieved_surfaces_deg: tuple[float, float, float]
    aero_surfaces_deg: tuple[float, float, float]
    throttle: float = Field(ge=0.0, le=1.0)
    thrust_n: float
    force_body_n: tuple[float, float, float]
    moment_body_nm: tuple[float, float, float]


####


class Falcon6PlantRunResult(CadacModel):
    """First direct-surface FALCON6 batch execution result."""

    schema_id: str = "taoryx.cadac.falcon6-physical-plant/v0alpha1"
    source_name: str = Field(min_length=1)
    integration_step_s: float = Field(gt=0.0)
    requested_end_time_s: float = Field(gt=0.0)
    executed_steps: int = Field(ge=0)
    terminated_reason: str = Field(min_length=1)
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)
    samples: tuple[Falcon6PlantSample, ...] = Field(min_length=1)
    claim_boundary: str = (
        "Source-grounded rigid-body plant with physical aileron/elevator/rudder actuator dynamics and source propulsion. "
        "Commands are injected at the physical-surface command boundary; CADAC guidance/autopilot modules are not yet reproduced."
    )


####


@dataclass(slots=True)
class _Falcon6RuntimeState:
    position_ned_m: FloatVector
    position_derivative_ned_mps: FloatVector
    velocity_body_mps: FloatVector
    velocity_body_derivative_mps2: FloatVector
    velocity_ned_mps: FloatVector
    quaternion_wxyz: FloatVector
    quaternion_derivative: FloatVector
    body_rates_rad_s: FloatVector
    body_rate_derivative_rad_s2: FloatVector
    actuator_state: Falcon6ActuatorState
    achieved_surfaces: Falcon6SurfaceCommand
    engine_state: Falcon6EngineState
    alpha_deg: float
    beta_deg: float
    altitude_m: float
    gravity_mps2: float = 0.0
    mach: float = 0.0
    dynamic_pressure_pa: float = 0.0
    pressure_pa: float = 0.0
    density_kg_m3: float = 0.0
    temperature_k: float = 0.0
    speed_of_sound_mps: float = 0.0


####


def lower_falcon6_source_bundle(bundle: CadacSourceBundle) -> Falcon6SourceDefinition:
    """Lower one single-aircraft FALCON6 source bundle into typed plant inputs."""

    planes = bundle.case.vehicles_named("PLANE6")
    if len(planes) != 1 or len(bundle.case.vehicles) != 1:
        raise Falcon6SourceError(f"FALCON6 source lowering requires exactly one PLANE6; found {len(planes)}")
    ####
    module_order = tuple(module.name.casefold() for module in bundle.case.modules if CadacModuleStage.EXECUTE in module.stages)
    unknown = tuple(name for name in module_order if name not in _FALCON6_MODULES)
    if unknown:
        raise Falcon6SourceError(f"FALCON6 reconstruction does not implement source modules: {unknown!r}")
    ####
    missing = tuple(name for name in _FALCON6_MODULES if name not in module_order)
    if missing:
        raise Falcon6SourceError(f"FALCON6 reconstruction requires source modules: {missing!r}")
    ####
    timing = bundle.case.timing_values
    try:
        integration_step_s = timing["int_step"]
    except KeyError as error:
        raise Falcon6SourceError("FALCON6 source case must declare TIMING int_step") from error
    ####
    vehicle = planes[0]
    return Falcon6SourceDefinition(
        source_name=bundle.case.source_name,
        integration_step_s=integration_step_s,
        plot_step_s=timing.get("plot_step"),
        end_time_s=bundle.case.end_time_s,
        module_order=module_order,
        initial_state=Falcon6InitialState(
            position_ned_m=(_number(vehicle, "sbel1"), _number(vehicle, "sbel2"), _number(vehicle, "sbel3")),
            speed_mps=_number(vehicle, "dvbe"),
            yaw_deg=_number(vehicle, "psiblx"),
            pitch_deg=_number(vehicle, "thtblx"),
            roll_deg=_number(vehicle, "phiblx"),
            alpha_deg=_number(vehicle, "alpha0x", 0.0),
            beta_deg=_number(vehicle, "beta0x", 0.0),
            body_rates_deg_s=(
                _number(vehicle, "ppx", 0.0),
                _number(vehicle, "qqx", 0.0),
                _number(vehicle, "rrx", 0.0),
            ),
        ),
        aerodynamics=Falcon6AeroLimits(
            positive_alpha_limit_deg=_number(vehicle, "alplimpx"),
            negative_alpha_limit_deg=_number(vehicle, "alplimnx"),
            reference_cg_m=_number(vehicle, "xcgr"),
            actual_cg_m=_number(vehicle, "xcg"),
        ),
        propulsion=Falcon6PropulsionConfig(
            mode=_integer(vehicle, "mprop", 0),
            mach_command=_number(vehicle, "vmachcom", 0.0),
            throttle=_number(vehicle, "throttle", 0.0),
            mach_gain=_number(vehicle, "gmach", 0.0),
        ),
        actuator=Falcon6ActuatorConfig(
            mode=_integer(vehicle, "mact", 0),
            position_limit_deg=_number(vehicle, "dlimx"),
            rate_limit_deg_s=_number(vehicle, "ddlimx"),
            natural_frequency_rad_s=_number(vehicle, "wnact"),
            damping_ratio=_number(vehicle, "zetact"),
        ),
        control_limits=Falcon6ControlLimits(
            autopilot_mode=_integer(vehicle, "maut", 0),
            aileron_limit_deg=_number(vehicle, "dalimx"),
            elevator_limit_deg=_number(vehicle, "delimx"),
            rudder_limit_deg=_number(vehicle, "drlimx"),
            positive_load_limit_g=_number(vehicle, "anlimpx"),
            negative_load_limit_g=_number(vehicle, "anlimnx"),
            bank_limit_deg=_number(vehicle, "philimx"),
        ),
        aerodynamic_deck=bundle.deck_for("PLANE6", CadacDeckKind.AERODYNAMIC),
        propulsion_deck=bundle.deck_for("PLANE6", CadacDeckKind.PROPULSION),
        events=vehicle.events,
        source_artifacts=bundle.artifacts,
    )


####


def load_falcon6_source_definition(path: str | Path) -> Falcon6SourceDefinition:
    """Parse, fingerprint, and lower one FALCON6 ``input.asc`` file."""

    return lower_falcon6_source_bundle(load_cadac_source_bundle(path))


####


def falcon6_initial_quaternion(initial: Falcon6InitialState) -> tuple[float, float, float, float]:
    """Return the source scalar-first quaternion initialized from yaw/pitch/roll."""

    psi = initial.yaw_deg * _RAD_PER_DEG
    theta = initial.pitch_deg * _RAD_PER_DEG
    phi = initial.roll_deg * _RAD_PER_DEG
    spsi, cpsi = math.sin(psi / 2.0), math.cos(psi / 2.0)
    stheta, ctheta = math.sin(theta / 2.0), math.cos(theta / 2.0)
    sphi, cphi = math.sin(phi / 2.0), math.cos(phi / 2.0)
    return (
        cpsi * ctheta * cphi + spsi * stheta * sphi,
        cpsi * ctheta * sphi - spsi * stheta * cphi,
        cpsi * stheta * cphi + spsi * ctheta * sphi,
        -cpsi * stheta * sphi + spsi * ctheta * cphi,
    )


####


def falcon6_actuator_step(
    config: Falcon6ActuatorConfig,
    state: Falcon6ActuatorState,
    command: Falcon6SurfaceCommand,
    dt_s: float,
) -> Falcon6ActuatorStep:
    """Advance the source physical surface actuator without hiding saturation."""

    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("FALCON6 actuator dt_s must be positive and finite")
    ####
    requested = np.asarray(command.vector(), dtype=np.float64)
    if config.mode == 0:
        achieved = np.clip(requested, -config.position_limit_deg, config.position_limit_deg)
        position_limited = tuple(bool(abs(value) > config.position_limit_deg) for value in requested)
        state_next = Falcon6ActuatorState(position_deg=_tuple3(achieved))
        return Falcon6ActuatorStep(
            requested=command,
            achieved=_surfaces_from_vector(achieved),
            state=state_next,
            position_limited=position_limited,
            rate_limited=(False, False, False),
        )
    ####
    position_derivative = np.asarray(state.position_derivative_deg_s, dtype=np.float64).copy()
    position = np.asarray(state.position_deg, dtype=np.float64).copy()
    rate_derivative = np.asarray(state.rate_derivative_deg_s2, dtype=np.float64).copy()
    rate = np.asarray(state.rate_deg_s, dtype=np.float64).copy()
    position_limited = [False, False, False]
    rate_limited = [False, False, False]
    wn = config.natural_frequency_rad_s
    damping = config.damping_ratio
    for index in range(3):
        if abs(position[index]) > config.position_limit_deg:
            position_limited[index] = True
            position[index] = math.copysign(config.position_limit_deg, position[index])
            if position[index] * rate[index] > 0.0:
                rate[index] = 0.0
            ####
        ####
        limited_rate = abs(rate[index]) > config.rate_limit_deg_s
        if limited_rate:
            rate_limited[index] = True
            rate[index] = math.copysign(config.rate_limit_deg_s, rate[index])
        ####
        position_derivative_new = rate[index]
        position[index] = _integrate_scalar(
            position[index],
            position_derivative_new,
            position_derivative[index],
            dt_s,
        )
        position_derivative[index] = position_derivative_new
        error = requested[index] - position[index]
        rate_derivative_new = wn * wn * error - 2.0 * damping * wn * position_derivative[index]
        rate[index] = _integrate_scalar(
            rate[index],
            rate_derivative_new,
            rate_derivative[index],
            dt_s,
        )
        rate_derivative[index] = rate_derivative_new
        if limited_rate and rate[index] * rate_derivative[index] > 0.0:
            rate_derivative[index] = 0.0
        ####
    ####
    achieved = position.copy()
    state_next = Falcon6ActuatorState(
        position_derivative_deg_s=_tuple3(position_derivative),
        position_deg=_tuple3(position),
        rate_derivative_deg_s2=_tuple3(rate_derivative),
        rate_deg_s=_tuple3(rate),
    )
    return Falcon6ActuatorStep(
        requested=command,
        achieved=_surfaces_from_vector(achieved),
        state=state_next,
        position_limited=tuple(position_limited),
        rate_limited=tuple(rate_limited),
    )


####


def falcon6_propulsion_step(
    definition: Falcon6SourceDefinition,
    state: Falcon6EngineState,
    *,
    mach: float,
    altitude_m: float,
    dt_s: float,
    throttle_override: float | None = None,
) -> Falcon6PropulsionStep:
    """Advance the FALCON6 source turbojet spool and thrust tables."""

    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("FALCON6 propulsion dt_s must be positive and finite")
    ####
    if mach < 0.0 or altitude_m < 0.0:
        raise ValueError("FALCON6 propulsion Mach and altitude must be nonnegative")
    ####
    config = definition.propulsion
    if throttle_override is not None:
        throttle = float(throttle_override)
    elif config.mode == 2:
        throttle = config.mach_gain * (config.mach_command - mach)
    else:
        throttle = config.throttle
    ####
    if config.mode == 0:
        throttle = 0.0
    ####
    throttle = min(1.0, max(0.0, throttle))
    if config.mode == 2:
        throttle = min(0.77, throttle)
    ####
    power_command = 64.94 * throttle if throttle <= 0.77 else 217.38 * throttle - 117.38
    spool_time = 1.0 if power_command <= 50.0 else 0.2
    derivative_new = (power_command - state.power_percent) / spool_time
    power = _integrate_scalar(
        state.power_percent,
        derivative_new,
        state.power_derivative_percent_s,
        dt_s,
    )
    engine_state = Falcon6EngineState(power_derivative_percent_s=derivative_new, power_percent=power)
    altitude_ft = altitude_m / 0.3048
    deck = definition.propulsion_deck
    idle = deck.table("idle_vs_mach_alt").interpolate((mach, altitude_ft)) * _LBF_TO_N
    military = deck.table("mil_vs_mach_alt").interpolate((mach, altitude_ft)) * _LBF_TO_N
    maximum = deck.table("max_vs_mach_alt").interpolate((mach, altitude_ft)) * _LBF_TO_N
    if config.mode == 0:
        thrust = 0.0
    elif power < 50.0:
        thrust = idle + power * 0.02 * (military - idle)
    else:
        thrust = military + (power - 50.0) * 0.02 * (maximum - military)
    ####
    return Falcon6PropulsionStep(
        throttle=throttle,
        commanded_power_percent=power_command,
        achieved_power_percent=power,
        spool_time_constant_s=spool_time,
        idle_thrust_n=idle,
        military_thrust_n=military,
        maximum_thrust_n=maximum,
        thrust_n=thrust,
        state=engine_state,
    )


####


def falcon6_aerodynamic_coefficients(
    definition: Falcon6SourceDefinition,
    state: Falcon6AeroState,
    surfaces: Falcon6SurfaceCommand,
) -> Falcon6AeroCoefficients:
    """Evaluate the source FALCON6 aerodynamic force/moment coefficient closure."""

    airframe = definition.airframe
    deck = definition.aerodynamic_deck
    alpha = state.alpha_deg
    beta = state.beta_deg
    roll_rate_deg_s, pitch_rate_deg_s, yaw_rate_deg_s = state.body_rates_deg_s
    aileron = surfaces.aileron_deg
    elevator = surfaces.elevator_deg
    rudder = surfaces.rudder_deg
    chord_factor = airframe.reference_chord_m / (2.0 * state.speed_mps)
    span_factor = airframe.reference_span_m / (2.0 * state.speed_mps)

    cx = deck.table("cx_vs_elev_alpha").interpolate((elevator, alpha))
    cxq = deck.table("cxq_vs_alpha").interpolate((alpha,))
    total_cx = cx + chord_factor * cxq * pitch_rate_deg_s * _RAD_PER_DEG

    cyr = deck.table("cyr_vs_alpha").interpolate((alpha,))
    cyp = deck.table("cyp_vs_alpha").interpolate((alpha,))
    total_cy = (
        -0.02 * beta
        + 0.021 * aileron / 20.0
        + 0.086 * rudder / 30.0
        + span_factor * (cyr * yaw_rate_deg_s * _RAD_PER_DEG + cyp * roll_rate_deg_s * _RAD_PER_DEG)
    )

    cz = deck.table("cz_vs_alpha").interpolate((alpha,))
    czq = deck.table("czq_vs_alpha").interpolate((alpha,))
    total_cz = cz * (1.0 - (beta * _RAD_PER_DEG) ** 2) - 0.19 * elevator / 25.0
    total_cz += chord_factor * czq * pitch_rate_deg_s * _RAD_PER_DEG

    cl = deck.table("cl_vs_beta_alpha").interpolate((beta, alpha))
    cldr = deck.table("cldr_vs_beta_alpha").interpolate((beta, alpha))
    clda = -deck.table("clda_vs_beta_alpha").interpolate((beta, alpha))
    clp = deck.table("clp_vs_alpha").interpolate((alpha,))
    # Source compatibility note: the C++ routine looks up ``clr`` but uses the
    # distinct local ``cllr`` variable, which remains initialized to zero.
    total_cl = cl + clda * aileron / 20.0 + cldr * rudder / 30.0
    total_cl += span_factor * clp * roll_rate_deg_s * _RAD_PER_DEG

    cm = deck.table("cm_vs_elev_alpha").interpolate((elevator, alpha))
    cmq = deck.table("cmq_vs_alpha").interpolate((alpha,))
    cg_term_chord = (definition.aerodynamics.reference_cg_m - definition.aerodynamics.actual_cg_m) / airframe.reference_chord_m
    total_cm = cm + chord_factor * cmq * pitch_rate_deg_s * _RAD_PER_DEG + total_cz * cg_term_chord

    cn = deck.table("cn_vs_beta_alpha").interpolate((beta, alpha))
    cnda = deck.table("cnda_vs_beta_alpha").interpolate((beta, alpha))
    cndr = deck.table("cndr_vs_beta_alpha").interpolate((beta, alpha))
    cnr = deck.table("cnr_vs_alpha").interpolate((alpha,))
    cnp = deck.table("cnp_vs_alpha").interpolate((alpha,))
    cg_term_span = (definition.aerodynamics.reference_cg_m - definition.aerodynamics.actual_cg_m) / airframe.reference_span_m
    total_cn = cn + cnda * aileron / 20.0 + cndr * rudder / 30.0 - total_cy * cg_term_span
    total_cn += span_factor * (cnr * yaw_rate_deg_s * _RAD_PER_DEG + cnp * roll_rate_deg_s * _RAD_PER_DEG)

    return Falcon6AeroCoefficients(cx=total_cx, cy=total_cy, cz=total_cz, cl=total_cl, cm=total_cm, cn=total_cn)


####


def falcon6_body_wrench(
    definition: Falcon6SourceDefinition,
    coefficients: Falcon6AeroCoefficients,
    *,
    dynamic_pressure_pa: float,
    thrust_n: float,
) -> Falcon6BodyWrench:
    """Convert source total coefficients and thrust to physical body wrench."""

    if dynamic_pressure_pa < 0.0 or thrust_n < 0.0:
        raise ValueError("FALCON6 dynamic pressure and thrust must be nonnegative")
    ####
    airframe = definition.airframe
    q_area = dynamic_pressure_pa * airframe.reference_area_m2
    force = (
        q_area * coefficients.cx + thrust_n,
        q_area * coefficients.cy,
        q_area * coefficients.cz,
    )
    moment = (
        q_area * airframe.reference_span_m * coefficients.cl,
        q_area * airframe.reference_chord_m * coefficients.cm,
        q_area * airframe.reference_span_m * coefficients.cn,
    )
    return Falcon6BodyWrench(force_n=force, moment_nm=moment)


####


def falcon6_rotational_derivative(
    definition: Falcon6SourceDefinition,
    *,
    body_rates_rad_s: tuple[float, float, float],
    moment_nm: tuple[float, float, float],
) -> Falcon6RotationalDerivative:
    """Evaluate the source rigid-body Euler derivative including engine momentum."""

    omega = np.asarray(body_rates_rad_s, dtype=np.float64)
    moment = np.asarray(moment_nm, dtype=np.float64)
    if not np.all(np.isfinite(omega)) or not np.all(np.isfinite(moment)):
        raise ValueError("FALCON6 body rates and moment must be finite")
    ####
    inertia = np.asarray(definition.airframe.inertia_kg_m2, dtype=np.float64)
    engine_momentum = np.asarray(
        (definition.airframe.engine_angular_momentum_kg_m2_s, 0.0, 0.0),
        dtype=np.float64,
    )
    angular_acceleration = np.linalg.solve(
        inertia,
        moment - np.cross(omega, inertia @ omega + engine_momentum),
    )
    return Falcon6RotationalDerivative(
        body_rates_rad_s=_tuple3(omega),
        angular_acceleration_rad_s2=_tuple3(angular_acceleration),
    )


####


def run_falcon6_physical_plant(
    definition: Falcon6SourceDefinition,
    command: Falcon6DirectPlantCommand | None = None,
    *,
    end_time_s: float | None = None,
    sample_step_s: float | None = None,
) -> Falcon6PlantRunResult:
    """Run the source rigid-body plant with direct physical-surface commands.

    The source module order is preserved where it affects plant state.
    ``guidance`` and ``control`` are intentionally command-boundary no-ops in
    this first plug-in realization; the requested surface command feeds the
    source ``actuator`` module at its original location in the schedule.
    """

    dt_s = definition.integration_step_s
    requested_end = definition.end_time_s if end_time_s is None else float(end_time_s)
    if not math.isfinite(requested_end) or requested_end <= 0.0:
        raise ValueError("FALCON6 end_time_s must be positive and finite")
    ####
    cadence = max(dt_s, 0.02) if sample_step_s is None else float(sample_step_s)
    if not math.isfinite(cadence) or cadence <= 0.0:
        raise ValueError("FALCON6 sample_step_s must be positive and finite")
    ####
    resolved_command = command or Falcon6DirectPlantCommand()
    runtime = _initialize_runtime(definition)
    samples: list[Falcon6PlantSample] = []
    next_sample_time = 0.0
    sim_time = 0.0
    steps = 0
    last_wrench = Falcon6BodyWrench(force_n=(0.0, 0.0, 0.0), moment_nm=(0.0, 0.0, 0.0))
    last_thrust = Falcon6PropulsionStep(
        throttle=0.0,
        commanded_power_percent=0.0,
        achieved_power_percent=0.0,
        spool_time_constant_s=1.0,
        idle_thrust_n=0.0,
        military_thrust_n=0.0,
        maximum_thrust_n=0.0,
        thrust_n=0.0,
        state=runtime.engine_state,
    )
    aero_surfaces = runtime.achieved_surfaces
    while sim_time <= requested_end + 0.5 * dt_s:
        steps += 1
        for module in definition.module_order:
            if module == "environment":
                _runtime_environment(runtime)
            elif module == "kinematics":
                _runtime_kinematics(runtime, dt_s)
            elif module == "aerodynamics":
                aero_surfaces = runtime.achieved_surfaces
                coefficients = falcon6_aerodynamic_coefficients(
                    definition,
                    Falcon6AeroState(
                        alpha_deg=runtime.alpha_deg,
                        beta_deg=runtime.beta_deg,
                        speed_mps=max(1.0e-9, float(np.linalg.norm(runtime.velocity_ned_mps))),
                        body_rates_deg_s=_tuple3(runtime.body_rates_rad_s * _DEG_PER_RAD),
                    ),
                    aero_surfaces,
                )
            elif module == "propulsion":
                last_thrust = falcon6_propulsion_step(
                    definition,
                    runtime.engine_state,
                    mach=runtime.mach,
                    altitude_m=max(0.0, runtime.altitude_m),
                    dt_s=dt_s,
                    throttle_override=resolved_command.throttle_override,
                )
                runtime.engine_state = last_thrust.state
            elif module == "forces":
                last_wrench = falcon6_body_wrench(
                    definition,
                    coefficients,
                    dynamic_pressure_pa=runtime.dynamic_pressure_pa,
                    thrust_n=max(0.0, last_thrust.thrust_n),
                )
            elif module in {"guidance", "control"}:
                # Direct physical-surface command boundary for the first plug-in realization.
                pass
            elif module == "actuator":
                actuator = falcon6_actuator_step(
                    definition.actuator,
                    runtime.actuator_state,
                    resolved_command.surfaces,
                    dt_s,
                )
                runtime.actuator_state = actuator.state
                runtime.achieved_surfaces = actuator.achieved
            elif module == "euler":
                derivative = falcon6_rotational_derivative(
                    definition,
                    body_rates_rad_s=_tuple3(runtime.body_rates_rad_s),
                    moment_nm=last_wrench.moment_nm,
                )
                derivative_new = np.asarray(derivative.angular_acceleration_rad_s2, dtype=np.float64)
                runtime.body_rates_rad_s = _integrate_vector(
                    runtime.body_rates_rad_s,
                    derivative_new,
                    runtime.body_rate_derivative_rad_s2,
                    dt_s,
                )
                runtime.body_rate_derivative_rad_s2 = derivative_new
            elif module == "newton":
                _runtime_newton(runtime, definition, last_wrench, dt_s)
            ####
        ####
        if sim_time + 0.5 * dt_s >= next_sample_time or sim_time + 0.5 * dt_s >= requested_end:
            samples.append(
                _runtime_sample(
                    sim_time,
                    runtime,
                    resolved_command,
                    aero_surfaces,
                    last_thrust,
                    last_wrench,
                )
            )
            while next_sample_time <= sim_time + 0.5 * dt_s:
                next_sample_time += cadence
            ####
        ####
        if not _runtime_is_finite(runtime):
            return Falcon6PlantRunResult(
                source_name=definition.source_name,
                integration_step_s=dt_s,
                requested_end_time_s=requested_end,
                executed_steps=steps,
                terminated_reason="nonfinite_state",
                source_artifacts=definition.source_artifacts,
                samples=tuple(samples),
            )
        ####
        sim_time += dt_s
    ####
    return Falcon6PlantRunResult(
        source_name=definition.source_name,
        integration_step_s=dt_s,
        requested_end_time_s=requested_end,
        executed_steps=steps,
        terminated_reason="end_time",
        source_artifacts=definition.source_artifacts,
        samples=tuple(samples),
    )


####


def _initialize_runtime(definition: Falcon6SourceDefinition) -> _Falcon6RuntimeState:
    initial = definition.initial_state
    quaternion = np.asarray(falcon6_initial_quaternion(initial), dtype=np.float64)
    transform = _dcm_body_from_local(quaternion)
    alpha = initial.alpha_deg * _RAD_PER_DEG
    beta = initial.beta_deg * _RAD_PER_DEG
    body_velocity = np.asarray(
        (
            math.cos(alpha) * math.cos(beta) * initial.speed_mps,
            math.sin(beta) * initial.speed_mps,
            math.sin(alpha) * math.cos(beta) * initial.speed_mps,
        ),
        dtype=np.float64,
    )
    local_velocity = transform.T @ body_velocity
    return _Falcon6RuntimeState(
        position_ned_m=np.asarray(initial.position_ned_m, dtype=np.float64),
        position_derivative_ned_mps=np.zeros(3, dtype=np.float64),
        velocity_body_mps=body_velocity,
        velocity_body_derivative_mps2=np.zeros(3, dtype=np.float64),
        velocity_ned_mps=local_velocity,
        quaternion_wxyz=quaternion,
        quaternion_derivative=np.zeros(4, dtype=np.float64),
        body_rates_rad_s=np.asarray(initial.body_rates_deg_s, dtype=np.float64) * _RAD_PER_DEG,
        body_rate_derivative_rad_s2=np.zeros(3, dtype=np.float64),
        actuator_state=Falcon6ActuatorState(),
        achieved_surfaces=Falcon6SurfaceCommand(),
        engine_state=Falcon6EngineState(),
        alpha_deg=initial.alpha_deg,
        beta_deg=initial.beta_deg,
        altitude_m=-float(initial.position_ned_m[2]),
    )


####


def _runtime_environment(runtime: _Falcon6RuntimeState) -> None:
    runtime.gravity_mps2 = cadac_source_inverse_square_gravity_mps2(runtime.altitude_m)
    density, pressure, temperature = atmosphere76(runtime.altitude_m)
    runtime.density_kg_m3 = density
    runtime.pressure_pa = pressure
    runtime.temperature_k = temperature
    runtime.speed_of_sound_mps = math.sqrt(1.4 * 287.053 * temperature)
    relative_speed = float(np.linalg.norm(runtime.velocity_ned_mps))
    runtime.mach = abs(relative_speed / runtime.speed_of_sound_mps)
    runtime.dynamic_pressure_pa = 0.5 * density * relative_speed * relative_speed


####


def _runtime_kinematics(runtime: _Falcon6RuntimeState, dt_s: float) -> None:
    q0, q1, q2, q3 = runtime.quaternion_wxyz
    p_rate, q_rate, r_rate = runtime.body_rates_rad_s
    error = 1.0 - float(runtime.quaternion_wxyz @ runtime.quaternion_wxyz)
    correction = 50.0
    derivative_new = np.asarray(
        (
            0.5 * (-p_rate * q1 - q_rate * q2 - r_rate * q3) + correction * error * q0,
            0.5 * (p_rate * q0 + r_rate * q2 - q_rate * q3) + correction * error * q1,
            0.5 * (q_rate * q0 - r_rate * q1 + p_rate * q3) + correction * error * q2,
            0.5 * (r_rate * q0 + q_rate * q1 - p_rate * q2) + correction * error * q3,
        ),
        dtype=np.float64,
    )
    runtime.quaternion_wxyz = _integrate_vector(
        runtime.quaternion_wxyz,
        derivative_new,
        runtime.quaternion_derivative,
        dt_s,
    )
    runtime.quaternion_derivative = derivative_new
    transform = _dcm_body_from_local(runtime.quaternion_wxyz)
    body_air_velocity = transform @ runtime.velocity_ned_mps
    speed_air = float(np.linalg.norm(body_air_velocity))
    if speed_air <= 1.0e-12:
        runtime.alpha_deg = 0.0
        runtime.beta_deg = 0.0
        return
    ####
    runtime.alpha_deg = math.atan2(float(body_air_velocity[2]), float(body_air_velocity[0])) * _DEG_PER_RAD
    beta_argument = min(1.0, max(-1.0, float(body_air_velocity[1]) / speed_air))
    runtime.beta_deg = math.asin(beta_argument) * _DEG_PER_RAD


####


def _runtime_newton(
    runtime: _Falcon6RuntimeState,
    definition: Falcon6SourceDefinition,
    wrench: Falcon6BodyWrench,
    dt_s: float,
) -> None:
    transform = _dcm_body_from_local(runtime.quaternion_wxyz)
    force_body = np.asarray(wrench.force_n, dtype=np.float64)
    specific_force = force_body / definition.airframe.mass_kg
    tangent_acceleration = np.cross(runtime.body_rates_rad_s, runtime.velocity_body_mps)
    gravity_local = np.asarray((0.0, 0.0, runtime.gravity_mps2), dtype=np.float64)
    derivative_new = specific_force - tangent_acceleration + transform @ gravity_local
    runtime.velocity_body_mps = _integrate_vector(
        runtime.velocity_body_mps,
        derivative_new,
        runtime.velocity_body_derivative_mps2,
        dt_s,
    )
    runtime.velocity_body_derivative_mps2 = derivative_new
    runtime.velocity_ned_mps = transform.T @ runtime.velocity_body_mps
    position_derivative_new = runtime.velocity_ned_mps
    runtime.position_ned_m = _integrate_vector(
        runtime.position_ned_m,
        position_derivative_new,
        runtime.position_derivative_ned_mps,
        dt_s,
    )
    runtime.position_derivative_ned_mps = position_derivative_new
    runtime.altitude_m = -float(runtime.position_ned_m[2])


####


def _runtime_sample(
    time_s: float,
    runtime: _Falcon6RuntimeState,
    command: Falcon6DirectPlantCommand,
    aero_surfaces: Falcon6SurfaceCommand,
    propulsion: Falcon6PropulsionStep,
    wrench: Falcon6BodyWrench,
) -> Falcon6PlantSample:
    quaternion = tuple(float(value) for value in runtime.quaternion_wxyz)
    if len(quaternion) != 4:
        raise RuntimeError("FALCON6 quaternion state dimension changed")
    ####
    return Falcon6PlantSample(
        time_s=max(0.0, time_s),
        position_ned_m=_tuple3(runtime.position_ned_m),
        velocity_ned_mps=_tuple3(runtime.velocity_ned_mps),
        velocity_body_mps=_tuple3(runtime.velocity_body_mps),
        quaternion_wxyz=(quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
        body_rates_rad_s=_tuple3(runtime.body_rates_rad_s),
        alpha_deg=runtime.alpha_deg,
        beta_deg=runtime.beta_deg,
        mach=runtime.mach,
        dynamic_pressure_pa=runtime.dynamic_pressure_pa,
        altitude_m=runtime.altitude_m,
        requested_surfaces_deg=command.surfaces.vector(),
        achieved_surfaces_deg=runtime.achieved_surfaces.vector(),
        aero_surfaces_deg=aero_surfaces.vector(),
        throttle=propulsion.throttle,
        thrust_n=propulsion.thrust_n,
        force_body_n=wrench.force_n,
        moment_body_nm=wrench.moment_nm,
    )


####


def _dcm_body_from_local(quaternion_wxyz: FloatVector) -> FloatMatrix:
    q0, q1, q2, q3 = quaternion_wxyz
    return np.asarray(
        (
            (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3, 2.0 * (q1 * q2 + q0 * q3), 2.0 * (q1 * q3 - q0 * q2)),
            (2.0 * (q1 * q2 - q0 * q3), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3, 2.0 * (q2 * q3 + q0 * q1)),
            (2.0 * (q1 * q3 + q0 * q2), 2.0 * (q2 * q3 - q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3),
        ),
        dtype=np.float64,
    )


####


def _integrate_vector(
    state: FloatVector,
    derivative_current: FloatVector,
    derivative_previous: FloatVector,
    dt_s: float,
) -> FloatVector:
    return np.asarray(
        cadac_stored_derivative_step(
            tuple(float(value) for value in state),
            tuple(float(value) for value in derivative_current),
            tuple(float(value) for value in derivative_previous),
            dt_s,
        ),
        dtype=np.float64,
    )


####


def _runtime_is_finite(runtime: _Falcon6RuntimeState) -> bool:
    vectors = (
        runtime.position_ned_m,
        runtime.position_derivative_ned_mps,
        runtime.velocity_body_mps,
        runtime.velocity_body_derivative_mps2,
        runtime.velocity_ned_mps,
        runtime.quaternion_wxyz,
        runtime.quaternion_derivative,
        runtime.body_rates_rad_s,
        runtime.body_rate_derivative_rad_s2,
    )
    scalars = (
        runtime.alpha_deg,
        runtime.beta_deg,
        runtime.altitude_m,
        runtime.gravity_mps2,
        runtime.mach,
        runtime.dynamic_pressure_pa,
    )
    return all(bool(np.all(np.isfinite(vector))) for vector in vectors) and all(math.isfinite(value) for value in scalars)


####


def _integrate_scalar(state: float, derivative_current: float, derivative_previous: float, dt_s: float) -> float:
    return cadac_stored_derivative_step(
        (state,),
        (derivative_current,),
        (derivative_previous,),
        dt_s,
    )[0]


####


def _number(vehicle: CadacVehicleBlock, name: str, default: float | None = None) -> float:
    try:
        value = vehicle.parameter(name)
    except KeyError:
        if default is None:
            raise Falcon6SourceError(f"{vehicle.model_name} is missing required parameter {name!r}") from None
        ####
        return default
    ####
    if not isinstance(value, (int, float)):
        raise Falcon6SourceError(f"{vehicle.model_name} parameter {name!r} must be numeric")
    ####
    result = float(value)
    if not math.isfinite(result):
        raise Falcon6SourceError(f"{vehicle.model_name} parameter {name!r} must be finite")
    ####
    return result


####


def _integer(vehicle: CadacVehicleBlock, name: str, default: int | None = None) -> int:
    value = _number(vehicle, name, float(default) if default is not None else None)
    if not value.is_integer():
        raise Falcon6SourceError(f"{vehicle.model_name} parameter {name!r} must be integer-valued")
    ####
    return int(value)


####


def _surfaces_from_vector(values: FloatVector) -> Falcon6SurfaceCommand:
    return Falcon6SurfaceCommand(
        aileron_deg=float(values[0]),
        elevator_deg=float(values[1]),
        rudder_deg=float(values[2]),
    )


####


def _tuple3(values: FloatVector) -> tuple[float, float, float]:
    if values.shape != (3,):
        raise ValueError("expected a three-component vector")
    ####
    return (float(values[0]), float(values[1]), float(values[2]))


####


__all__ = [
    "Falcon6ActuatorConfig",
    "Falcon6ActuatorState",
    "Falcon6ActuatorStep",
    "Falcon6AeroCoefficients",
    "Falcon6AeroLimits",
    "Falcon6AeroState",
    "Falcon6AirframeConstants",
    "Falcon6BodyWrench",
    "Falcon6ControlLimits",
    "Falcon6DirectPlantCommand",
    "Falcon6EngineState",
    "Falcon6InitialState",
    "Falcon6PlantRunResult",
    "Falcon6PlantSample",
    "Falcon6PropulsionConfig",
    "Falcon6PropulsionStep",
    "Falcon6RotationalDerivative",
    "Falcon6SourceDefinition",
    "Falcon6SourceError",
    "Falcon6SurfaceCommand",
    "falcon6_actuator_step",
    "falcon6_aerodynamic_coefficients",
    "falcon6_body_wrench",
    "falcon6_initial_quaternion",
    "falcon6_propulsion_step",
    "falcon6_rotational_derivative",
    "run_falcon6_physical_plant",
    "load_falcon6_source_definition",
    "lower_falcon6_source_bundle",
]
