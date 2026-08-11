"""Source-compatible MAGSIX trajectory-only Magnus-rotor runtime."""

from __future__ import annotations

import math
from pathlib import Path

from pydantic import Field, model_validator

from .bundle import CadacSourceArtifact, load_cadac_source_bundle
from .compatibility import cadac_stored_derivative_step
from .input_ast import CadacModel, CadacVehicleBlock
from .source_environment import atmosphere76, cadac_source_inverse_square_gravity_mps2

AGRAV = 9.80675445
RHO_SL = 1.225
R_AIR = 287.053
DEG_TO_RAD = 0.0174532925199432
RAD_TO_DEG = 57.2957795130823
RAD_S_TO_RPM = 9.5493

_EXPECTED_MODULES = ("environment", "trajectory")


class MagsixSourceError(ValueError):
    """Raised when a MAGSIX source case exceeds the trajectory-only runtime boundary."""


####


class MagsixInitialState(CadacModel):
    """Metric source initial state for the planar Magnus-rotor trajectory."""

    north_m: float
    east_m: float
    altitude_m: float
    speed_mps: float = Field(gt=0.0)
    heading_deg: float
    flight_path_deg: float
    spin_rpm: float


####


class MagsixVehicleConfig(CadacModel):
    """Source mass and aerodynamic coefficients used by the uncoupled trajectory equations."""

    mass_kg: float = Field(gt=0.0)
    spin_inertia_kgm2: float = Field(gt=0.0)
    reference_area_m2: float = Field(gt=0.0)
    reference_length_m: float = Field(gt=0.0)
    drag_coefficient: float = Field(gt=0.0)
    spin_damping_coefficient_rad: float
    magnus_lift_coefficient_rad: float
    spin_acceleration_coefficient: float
    ground_altitude_m: float = 0.0

    @model_validator(mode="after")
    def validate_dynamic_coefficients(self) -> "MagsixVehicleConfig":
        if abs(self.spin_damping_coefficient_rad) <= 1.0e-15:
            raise ValueError("MAGSIX spin damping coefficient must be nonzero")
        ####
        if abs(self.magnus_lift_coefficient_rad * self.spin_acceleration_coefficient) <= 1.0e-15:
            raise ValueError("MAGSIX steady-state glide requires nonzero Magnus lift and spin acceleration coefficients")
        ####
        return self

    ####


####


class MagsixSourceDefinition(CadacModel):
    """Prepared trajectory-only source case for one MAGSIX ``ROTOR`` actor."""

    source_name: str = Field(min_length=1)
    source_model: str = "ROTOR"
    integration_step_dnt: float = Field(gt=0.0)
    plot_step_dnt: float | None = Field(default=None, gt=0.0)
    end_time_dnt: float = Field(gt=0.0)
    module_order: tuple[str, ...]
    initial_state: MagsixInitialState
    vehicle: MagsixVehicleConfig
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=1)
    taoryx_tier: str = "point_mass_3dof"
    runtime_fidelity: str = "point_mass_3dof"
    control_realization: str = "force_model"
    claim_boundary: str = (
        "MAGSIX planar center-of-mass trajectory and spin state only. The source attitude perturbation module is intentionally excluded; "
        "the source documents trajectory as uncoupled and independently runnable."
    )


####


class MagsixSample(CadacModel):
    """One accepted trajectory-only source sample."""

    time_s: float = Field(ge=0.0)
    source_time_dnt: float = Field(ge=0.0)
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]
    altitude_m: float
    speed_mps: float = Field(ge=0.0)
    heading_deg: float
    flight_path_deg: float
    spin_rpm: float
    tip_speed_ratio: float
    dynamic_time_scale_s: float = Field(gt=0.0)
    density_kg_m3: float = Field(gt=0.0)
    dynamic_pressure_pa: float = Field(ge=0.0)
    mach: float = Field(ge=0.0)


####


class MagsixRunResult(CadacModel):
    """Trajectory-only source run with impact status and source provenance."""

    status: str
    samples: tuple[MagsixSample, ...] = Field(min_length=1)
    impact_time_s: float | None = None
    source_artifacts: tuple[CadacSourceArtifact, ...]
    claim_boundary: str


####


def load_magsix_source_definition(path: str | Path) -> MagsixSourceDefinition:
    """Parse and lower one trajectory-only MAGSIX source case."""

    bundle = load_cadac_source_bundle(path)
    case = bundle.case
    rotors = case.vehicles_named("ROTOR")
    if len(rotors) != 1:
        raise MagsixSourceError(f"MAGSIX trajectory plug-in requires exactly one ROTOR actor; found {len(rotors)}")
    ####
    module_order = tuple(module.name.casefold() for module in case.modules)
    if module_order != _EXPECTED_MODULES:
        raise MagsixSourceError(f"MAGSIX trajectory-only runtime requires source modules {_EXPECTED_MODULES!r}; received {module_order!r}")
    ####
    rotor = rotors[0]
    if rotor.events:
        raise MagsixSourceError("MAGSIX trajectory-only runtime does not yet support source event mutations")
    ####
    if rotor.stochastic_assignments:
        raise MagsixSourceError("MAGSIX trajectory-only runtime does not yet materialize stochastic source declarations")
    ####
    if _integer(rotor, "mwind", 0) != 0:
        raise MagsixSourceError("MAGSIX trajectory-only runtime currently supports the source no-wind environment only")
    ####
    timing = case.timing_values
    try:
        integration_step = timing["int_step"]
    except KeyError as error:
        raise MagsixSourceError("MAGSIX source case is missing TIMING int_step") from error
    ####
    return MagsixSourceDefinition(
        source_name=case.source_name,
        integration_step_dnt=integration_step,
        plot_step_dnt=timing.get("plot_step"),
        end_time_dnt=case.end_time_s,
        module_order=module_order,
        initial_state=MagsixInitialState(
            north_m=_number(rotor, "sbel1"),
            east_m=_number(rotor, "sbel2"),
            altitude_m=_number(rotor, "hbe"),
            speed_mps=_number(rotor, "dvbe"),
            heading_deg=_number(rotor, "psivlx"),
            flight_path_deg=_number(rotor, "thtvlx"),
            spin_rpm=_number(rotor, "omega_rpm"),
        ),
        vehicle=MagsixVehicleConfig(
            mass_kg=_number(rotor, "mass"),
            spin_inertia_kgm2=_number(rotor, "moi_spin"),
            reference_area_m2=_number(rotor, "ref_area"),
            reference_length_m=_number(rotor, "ref_length"),
            drag_coefficient=_number(rotor, "cd"),
            spin_damping_coefficient_rad=_number(rotor, "cmdw"),
            magnus_lift_coefficient_rad=_number(rotor, "clw"),
            spin_acceleration_coefficient=_number(rotor, "cma"),
            ground_altitude_m=_number(rotor, "hbg", 0.0),
        ),
        source_artifacts=bundle.artifacts,
    )


####


class _MagsixRuntime:
    """Mutable numerical state kept internal to the source compatibility loop."""

    def __init__(self, definition: MagsixSourceDefinition) -> None:
        initial = definition.initial_state
        config = definition.vehicle
        gamma_ss = math.atan(
            config.drag_coefficient * config.spin_damping_coefficient_rad / (config.magnus_lift_coefficient_rad * config.spin_acceleration_coefficient)
        )
        self.velocity_ss_mps = math.sqrt(2.0 * AGRAV * config.mass_kg * abs(math.sin(gamma_ss)) / (RHO_SL * config.reference_area_m2 * config.drag_coefficient))
        self.gamma_rad = initial.flight_path_deg * DEG_TO_RAD
        self.velocity_dnu = initial.speed_mps / self.velocity_ss_mps
        rho, _, _ = atmosphere76(initial.altitude_m)
        self.tau_s = 2.0 * config.mass_kg / (rho * config.reference_area_m2 * self.velocity_ss_mps)
        omega_rad_s = initial.spin_rpm / RAD_S_TO_RPM
        self.omega_dnu = omega_rad_s * self.tau_s
        self.velocity_rate_dnu = 0.0
        self.gamma_rate_dnu = 0.0
        self.omega_rate_dnu = 0.0
        self.position_ned_m = [initial.north_m, initial.east_m, -initial.altitude_m]
        self.position_rate_ned_mps = [0.0, 0.0, 0.0]
        self.velocity_ned_mps = list(_velocity_ned(initial.speed_mps, initial.heading_deg, initial.flight_path_deg))
        self.altitude_m = initial.altitude_m
        self.speed_mps = initial.speed_mps
        self.spin_rpm = initial.spin_rpm
        self.tip_speed_ratio = omega_rad_s * config.reference_length_m / initial.speed_mps
        self.rho_kg_m3 = rho
        self.dynamic_pressure_pa = 0.5 * rho * initial.speed_mps * initial.speed_mps
        _, _, temperature_k = atmosphere76(initial.altitude_m)
        self.mach = initial.speed_mps / math.sqrt(1.4 * R_AIR * temperature_k)

    ####


####


def run_magsix_trajectory_source_compatibility(
    definition: MagsixSourceDefinition,
    *,
    end_time_dnt: float | None = None,
    sample_step_dnt: float | None = None,
) -> MagsixRunResult:
    """Execute MAGSIX's independently runnable trajectory equations in source DNT ordering."""

    dt_dnt = definition.integration_step_dnt
    requested_end = definition.end_time_dnt if end_time_dnt is None else float(end_time_dnt)
    cadence = (definition.plot_step_dnt or dt_dnt) if sample_step_dnt is None else float(sample_step_dnt)
    if not math.isfinite(requested_end) or requested_end <= 0.0:
        raise ValueError("MAGSIX end_time_dnt must be positive and finite")
    ####
    if not math.isfinite(cadence) or cadence <= 0.0:
        raise ValueError("MAGSIX sample_step_dnt must be positive and finite")
    ####
    runtime = _MagsixRuntime(definition)
    samples: list[MagsixSample] = []
    source_time = 0.0
    next_sample = 0.0
    impact_time: float | None = None
    status = "completed"
    while source_time <= requested_end + 0.5 * dt_dnt:
        _environment_step(runtime)
        real_time_s = _trajectory_step(definition, runtime, source_time, dt_dnt)
        if source_time + 0.5 * dt_dnt >= next_sample:
            samples.append(_sample(definition, runtime, real_time_s, source_time))
            next_sample += cadence
        ####
        if runtime.altitude_m < definition.vehicle.ground_altitude_m:
            impact_time = real_time_s
            status = "terminated"
            break
        ####
        source_time += dt_dnt
    ####
    return MagsixRunResult(
        status=status,
        samples=tuple(samples),
        impact_time_s=impact_time,
        source_artifacts=definition.source_artifacts,
        claim_boundary=definition.claim_boundary,
    )


####


def _environment_step(runtime: _MagsixRuntime) -> None:
    altitude = runtime.altitude_m
    rho, _, temperature_k = atmosphere76(altitude)
    speed = runtime.speed_mps
    runtime.rho_kg_m3 = rho
    runtime.dynamic_pressure_pa = 0.5 * rho * speed * speed
    runtime.mach = speed / math.sqrt(1.4 * R_AIR * temperature_k)


####


def _trajectory_step(
    definition: MagsixSourceDefinition,
    runtime: _MagsixRuntime,
    source_time_dnt: float,
    dt_dnt: float,
) -> float:
    config = definition.vehicle
    rho = runtime.rho_kg_m3
    grav = cadac_source_inverse_square_gravity_mps2(runtime.altitude_m)
    tau = 2.0 * config.mass_kg / (rho * config.reference_area_m2 * runtime.velocity_ss_mps)
    mu = 2.0 * config.mass_kg / (rho * config.reference_area_m2 * config.reference_length_m)
    moi_dnu = config.spin_inertia_kgm2 / (config.reference_length_m * config.reference_length_m * mu * mu * config.mass_kg)
    velocity_rate_new = (
        -config.drag_coefficient * runtime.velocity_dnu * runtime.velocity_dnu - tau * grav * math.sin(runtime.gamma_rad) / runtime.velocity_ss_mps
    )
    runtime.velocity_dnu = _integrate(runtime.velocity_dnu, velocity_rate_new, runtime.velocity_rate_dnu, dt_dnt)
    runtime.velocity_rate_dnu = velocity_rate_new
    gamma_rate_new = config.magnus_lift_coefficient_rad * runtime.omega_dnu / mu - tau * grav * math.cos(runtime.gamma_rad) / (
        runtime.velocity_ss_mps * runtime.velocity_dnu
    )
    runtime.gamma_rad = _integrate(runtime.gamma_rad, gamma_rate_new, runtime.gamma_rate_dnu, dt_dnt)
    runtime.gamma_rate_dnu = gamma_rate_new
    omega_rate_new = config.spin_acceleration_coefficient * runtime.velocity_dnu**2 / (
        mu * moi_dnu
    ) + config.spin_damping_coefficient_rad * runtime.velocity_dnu * runtime.omega_dnu / (mu * mu * moi_dnu)
    runtime.omega_dnu = _integrate(runtime.omega_dnu, omega_rate_new, runtime.omega_rate_dnu, dt_dnt)
    runtime.omega_rate_dnu = omega_rate_new

    runtime.tau_s = tau
    runtime.speed_mps = runtime.velocity_dnu * runtime.velocity_ss_mps
    flight_path_deg = runtime.gamma_rad * RAD_TO_DEG
    omega_rad_s = runtime.omega_dnu / tau
    runtime.spin_rpm = omega_rad_s * RAD_S_TO_RPM
    runtime.velocity_ned_mps = list(_velocity_ned(runtime.speed_mps, definition.initial_state.heading_deg, flight_path_deg))
    dt_real_s = dt_dnt * tau
    position_new = []
    for value, new_rate, old_rate in zip(
        runtime.position_ned_m,
        runtime.velocity_ned_mps,
        runtime.position_rate_ned_mps,
        strict=True,
    ):
        position_new.append(_integrate(value, new_rate, old_rate, dt_real_s))
    ####
    runtime.position_ned_m = position_new
    runtime.position_rate_ned_mps = list(runtime.velocity_ned_mps)
    runtime.altitude_m = -runtime.position_ned_m[2]
    runtime.tip_speed_ratio = omega_rad_s * config.reference_length_m / runtime.speed_mps
    return tau * source_time_dnt


####


def _sample(
    definition: MagsixSourceDefinition,
    runtime: _MagsixRuntime,
    real_time_s: float,
    source_time_dnt: float,
) -> MagsixSample:
    return MagsixSample(
        time_s=max(0.0, real_time_s),
        source_time_dnt=max(0.0, source_time_dnt),
        position_ned_m=tuple(runtime.position_ned_m),
        velocity_ned_mps=tuple(runtime.velocity_ned_mps),
        altitude_m=runtime.altitude_m,
        speed_mps=runtime.speed_mps,
        heading_deg=definition.initial_state.heading_deg,
        flight_path_deg=runtime.gamma_rad * RAD_TO_DEG,
        spin_rpm=runtime.spin_rpm,
        tip_speed_ratio=runtime.tip_speed_ratio,
        dynamic_time_scale_s=runtime.tau_s,
        density_kg_m3=runtime.rho_kg_m3,
        dynamic_pressure_pa=runtime.dynamic_pressure_pa,
        mach=runtime.mach,
    )


####


def _velocity_ned(speed_mps: float, heading_deg: float, flight_path_deg: float) -> tuple[float, float, float]:
    heading = heading_deg * DEG_TO_RAD
    flight_path = flight_path_deg * DEG_TO_RAD
    horizontal = speed_mps * math.cos(flight_path)
    return (
        horizontal * math.cos(heading),
        horizontal * math.sin(heading),
        -speed_mps * math.sin(flight_path),
    )


####


def _integrate(value: float, new_rate: float, old_rate: float, step: float) -> float:
    return cadac_stored_derivative_step((value,), (new_rate,), (old_rate,), step)[0]


####


def _number(vehicle: CadacVehicleBlock, name: str, default: float | None = None) -> float:
    try:
        value = vehicle.parameter(name)
    except KeyError:
        if default is None:
            raise MagsixSourceError(f"MAGSIX ROTOR source actor is missing parameter {name!r}") from None
        ####
        return float(default)
    ####
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise MagsixSourceError(f"MAGSIX parameter {name!r} must be finite numeric")
    ####
    return float(value)


####


def _integer(vehicle: CadacVehicleBlock, name: str, default: int | None = None) -> int:
    value = _number(vehicle, name, None if default is None else float(default))
    if not value.is_integer():
        raise MagsixSourceError(f"MAGSIX parameter {name!r} must be integer-valued")
    ####
    return int(value)


####


__all__ = [
    "MagsixInitialState",
    "MagsixRunResult",
    "MagsixSample",
    "MagsixSourceDefinition",
    "MagsixSourceError",
    "MagsixVehicleConfig",
    "load_magsix_source_definition",
    "run_magsix_trajectory_source_compatibility",
]
