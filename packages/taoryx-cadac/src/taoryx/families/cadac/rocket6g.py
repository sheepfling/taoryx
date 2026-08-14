"""Phase-aware ROCKET6G source lowering and mixed RCS/TVC rigid-body plant."""

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
from .events import CadacEventApplication, CadacEventCursor
from .input_ast import CadacDeckKind, CadacEventBlock, CadacModel, CadacModuleStage, CadacVehicleBlock
from .rocket6g_rcs import (
    Rocket6gRcsConfig,
    Rocket6gRcsRuntimeInput,
    Rocket6gRcsState,
    Rocket6gRcsStep,
    rocket6g_rcs_step,
)
from .source_environment import atmosphere76

FloatVector: TypeAlias = NDArray[np.float64]
FloatMatrix: TypeAlias = NDArray[np.float64]

# Source constants from ROCKET6G/global_constants.hpp.
AGRAV = 9.80675445
WEII3 = 7.292115e-5
GM = 3.9860044e14
C20 = -4.8416685e-4
FLATTENING = 3.33528106e-3
SMAJOR_AXIS_M = 6_378_137.0
GW_CLONG_RAD = 0.0
RGAS = 287.053
RAD_PER_DEG = 0.0174532925199432
DEG_PER_RAD = 57.2957795130823
SMALL = 1.0e-7

_ROCKET6G_MODULES = (
    "kinematics",
    "environment",
    "propulsion",
    "aerodynamics",
    "gps",
    "startrack",
    "ins",
    "guidance",
    "control",
    "rcs",
    "actuator",
    "tvc",
    "forces",
    "newton",
    "euler",
    "intercept",
)
_ROCKET6G_REQUIRED_MODULES = (
    "kinematics",
    "environment",
    "propulsion",
    "aerodynamics",
    "rcs",
    "tvc",
    "forces",
    "newton",
    "euler",
)
_ROCKET6G_AERO_TABLES = (
    "ca0slv3_vs_mach",
    "caaslv3_vs_mach",
    "ca0bslv3_vs_mach",
    "cn0slv3_vs_mach_alpha",
    "clm0slv3_vs_mach_alpha",
    "clmqslv3_vs_mach",
    "ca0slv2_vs_mach",
    "caaslv2_vs_mach",
    "ca0bslv2_vs_mach",
    "cn0slv2_vs_mach_alpha",
    "clm0slv2_vs_mach_alpha",
    "clmqslv2_vs_mach",
    "cn0slv1_vs_mach_alpha",
    "clm0slv1_vs_mach_alpha",
    "clmqslv1_vs_mach",
)
_STAGE_NUMBER_BY_AERO_MODE = {13: 1, 12: 2, 11: 3}


class Rocket6gSourceError(ValueError):
    """Raised when a CADAC case cannot lower to the bounded ROCKET6G plant."""


####


class Rocket6gInitialState(CadacModel):
    """Source geodetic launch truth and body orientation."""

    longitude_deg: float
    latitude_deg: float
    altitude_m: float
    geographic_speed_mps: float = Field(gt=0.0)
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    alpha_deg: float = 0.0
    beta_deg: float = 0.0
    body_rates_deg_s: tuple[float, float, float] = (0.0, 0.0, 0.0)


####


class Rocket6gStageConfig(CadacModel):
    """One source stage mass-property and motor declaration."""

    stage_number: int = Field(ge=1, le=3)
    aerodynamic_mode: int
    propulsion_mode: int
    reference_cg_m: float
    initial_mass_kg: float = Field(gt=0.0)
    initial_fuel_mass_kg: float = Field(gt=0.0)
    initial_cg_m: float
    final_cg_m: float
    initial_roll_inertia_kgm2: float = Field(gt=0.0)
    final_roll_inertia_kgm2: float = Field(gt=0.0)
    initial_transverse_inertia_kgm2: float = Field(gt=0.0)
    final_transverse_inertia_kgm2: float = Field(gt=0.0)
    specific_impulse_s: float = Field(gt=0.0)
    source_fuel_flow_kg_s: float = Field(gt=0.0)
    nozzle_exit_area_m2: float = Field(default=0.0, ge=0.0)

    @model_validator(mode="after")
    def validate_stage_identity(self) -> "Rocket6gStageConfig":
        expected = _STAGE_NUMBER_BY_AERO_MODE.get(self.aerodynamic_mode)
        if expected != self.stage_number:
            raise ValueError("ROCKET6G stage number and aerodynamic mode disagree")
        ####
        if self.propulsion_mode not in {3, 4}:
            raise ValueError("ROCKET6G source stages require propulsion mode 3 or 4")
        ####
        if self.initial_fuel_mass_kg >= self.initial_mass_kg:
            raise ValueError("ROCKET6G source fuel mass must be below gross stage mass")
        ####
        return self

    ####


####


class Rocket6gTvcConfig(CadacModel):
    """Source physical thrust-vector-control actuator declaration."""

    mode: int
    command_gain: float
    propulsion_arm_from_nose_m: float
    position_limit_deg: float = Field(gt=0.0)
    rate_limit_deg_s: float = Field(gt=0.0)
    natural_frequency_rad_s: float = Field(gt=0.0)
    damping_ratio: float = Field(ge=0.0)
    variable_gain_factor: float = 0.0

    @model_validator(mode="after")
    def validate_mode(self) -> "Rocket6gTvcConfig":
        if self.mode not in {0, 1, 2, 3}:
            raise ValueError("ROCKET6G TVC mode must be 0, 1, 2, or 3")
        ####
        return self

    ####


####


class Rocket6gSourceDefinition(CadacModel):
    """Prepared source case for the phase-aware ROCKET6G vehicle plug-in."""

    source_name: str = Field(min_length=1)
    integration_step_s: float = Field(gt=0.0)
    plot_step_s: float | None = Field(default=None, gt=0.0)
    end_time_s: float = Field(gt=0.0)
    module_order: tuple[str, ...]
    initial_state: Rocket6gInitialState
    reference_area_m2: float = Field(gt=0.0)
    reference_length_m: float = Field(gt=0.0)
    positive_alpha_limit_deg: float = Field(gt=0.0)
    structural_load_limit_g: float = Field(gt=0.0)
    initial_parameters: dict[str, int | float]
    stages: tuple[Rocket6gStageConfig, ...] = Field(min_length=3, max_length=3)
    tvc: Rocket6gTvcConfig
    aerodynamic_deck: CadacDeck
    events: tuple[CadacEventBlock, ...] = Field(min_length=4)
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=2)
    taoryx_tier: str = "rigid_body_6dof_surface_allocated"
    runtime_fidelity: str = "phase_reported_rigid_body_6dof"
    control_realization: str = "mixed_effector"
    claim_boundary: str = (
        "Source-ordered WGS84 rigid-body truth, three-stage motor/mass-property transitions, aerodynamic closure, "
        "aggregate RCS, and physical second-order TVC participate. CADAC LTG guidance, acceleration autopilot, "
        "GPS/INS/star-tracker errors, turbulence, and compiled-executable parity remain outside this realization."
    )

    @model_validator(mode="after")
    def validate_source_definition(self) -> "Rocket6gSourceDefinition":
        table_names = {table.name.casefold() for table in self.aerodynamic_deck.tables}
        missing = tuple(name for name in _ROCKET6G_AERO_TABLES if name.casefold() not in table_names)
        if missing:
            raise ValueError("ROCKET6G source definition is missing required aerodynamic tables: " + ", ".join(missing))
        ####
        stage_numbers = tuple(stage.stage_number for stage in self.stages)
        if stage_numbers != (1, 2, 3):
            raise ValueError("ROCKET6G source stages must be ordered 1, 2, 3")
        ####
        return self

    ####


####


class Rocket6gDirectCommand(CadacModel):
    """Direct command boundary used while source guidance/control remain excluded."""

    tvc_pitch_command_deg: float = 0.0
    tvc_yaw_command_deg: float = 0.0
    thrust_vector_unit_body: tuple[float, float, float] = (1.0, 0.0, 0.0)
    roll_command_deg: float | None = None
    pitch_command_deg: float | None = None
    yaw_command_deg: float | None = None
    boost_cutoff_time_s: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def validate_command(self) -> "Rocket6gDirectCommand":
        values = (
            self.tvc_pitch_command_deg,
            self.tvc_yaw_command_deg,
            *self.thrust_vector_unit_body,
            *(value for value in (self.roll_command_deg, self.pitch_command_deg, self.yaw_command_deg, self.boost_cutoff_time_s) if value is not None),
        )
        if any(not math.isfinite(float(value)) for value in values):
            raise ValueError("ROCKET6G direct commands must be finite")
        ####
        magnitude = math.sqrt(sum(float(value) * float(value) for value in self.thrust_vector_unit_body))
        if magnitude <= 0.0:
            raise ValueError("ROCKET6G thrust-vector command must have positive magnitude")
        ####
        return self

    ####

    def normalized_thrust_vector(self) -> tuple[float, float, float]:
        magnitude = math.sqrt(sum(float(value) * float(value) for value in self.thrust_vector_unit_body))
        x_value, y_value, z_value = self.thrust_vector_unit_body
        return (
            float(x_value) / magnitude,
            float(y_value) / magnitude,
            float(z_value) / magnitude,
        )

    ####


####


class Rocket6gTvcState(CadacModel):
    """Stored-derivative physical nozzle state."""

    position_derivative_rad_s: tuple[float, float] = (0.0, 0.0)
    position_rad: tuple[float, float] = (0.0, 0.0)
    rate_derivative_rad_s2: tuple[float, float] = (0.0, 0.0)
    rate_rad_s: tuple[float, float] = (0.0, 0.0)


####


class Rocket6gTvcStep(CadacModel):
    """One physical TVC transition and achieved thrust wrench."""

    active: bool
    requested_control_deg: tuple[float, float]
    requested_nozzle_deg: tuple[float, float]
    achieved_nozzle_deg: tuple[float, float]
    command_gain: float
    force_body_n: tuple[float, float, float]
    moment_body_nm: tuple[float, float, float]
    position_limited: tuple[bool, bool]
    rate_limited: tuple[bool, bool]
    state: Rocket6gTvcState


####


class Rocket6gPropulsionState(CadacModel):
    """Stored stage fuel and mass-property state."""

    fuel_expended_derivative_kg_s: float = 0.0
    fuel_expended_kg: float = 0.0
    mass_kg: float = Field(gt=0.0)
    center_of_gravity_m: float
    inertia_diagonal_kgm2: tuple[float, float, float]

    @model_validator(mode="after")
    def validate_inertia(self) -> "Rocket6gPropulsionState":
        if any(value <= 0.0 or not math.isfinite(value) for value in self.inertia_diagonal_kgm2):
            raise ValueError("ROCKET6G inertia diagonal must be positive and finite")
        ####
        return self

    ####


####


class Rocket6gPropulsionStep(CadacModel):
    """One source rocket-motor and mass-property transition."""

    stage_number: int = Field(ge=1, le=3)
    propulsion_mode_before: int
    propulsion_mode_after: int
    thrust_n: float
    remaining_fuel_kg: float
    burned_out: bool
    state: Rocket6gPropulsionState


####


class Rocket6gAeroCoefficients(CadacModel):
    """Source total body-axis aerodynamic coefficients."""

    cx: float = 0.0
    cy: float = 0.0
    cz: float = 0.0
    cl: float = 0.0
    cm: float = 0.0
    cn: float = 0.0


####


class Rocket6gBodyWrench(CadacModel):
    """Total non-gravitational body force and moment."""

    force_body_n: tuple[float, float, float]
    moment_body_nm: tuple[float, float, float]


####


class Rocket6gPhaseEvent(CadacModel):
    """One applied source event with phase/stage transition evidence."""

    event_index: int = Field(ge=0)
    time_s: float = Field(ge=0.0)
    source_line: int = Field(ge=1)
    watch_variable: str
    operator: str
    criterion: int | float
    previous_values: tuple[tuple[str, int | float], ...]
    updated_values: tuple[tuple[str, int | float], ...]
    phase_before: str
    phase_after: str
    stage_before: int
    stage_after: int
    runtime_fidelity_before: str
    runtime_fidelity_after: str


####


class Rocket6gPlantSample(CadacModel):
    """One source-ordered ROCKET6G accepted truth sample."""

    time_s: float = Field(ge=0.0)
    position_inertial_m: tuple[float, float, float]
    velocity_inertial_mps: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    body_rates_inertial_rad_s: tuple[float, float, float]
    body_rates_earth_rad_s: tuple[float, float, float]
    longitude_deg: float
    latitude_deg: float
    altitude_m: float
    geographic_speed_mps: float = Field(ge=0.0)
    heading_deg: float
    flight_path_deg: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    alpha_deg: float
    beta_deg: float
    total_alpha_deg: float
    aerodynamic_roll_deg: float
    mach: float = Field(ge=0.0)
    dynamic_pressure_pa: float = Field(ge=0.0)
    active_stage: int = Field(ge=1, le=3)
    source_phase: str
    runtime_fidelity: str
    control_realization: str
    propulsion_mode: int
    rcs_moment_mode: int
    rcs_force_mode: int
    tvc_mode: int
    mass_kg: float = Field(gt=0.0)
    remaining_fuel_kg: float
    center_of_gravity_m: float
    inertia_diagonal_kgm2: tuple[float, float, float]
    thrust_n: float
    requested_tvc_control_deg: tuple[float, float]
    requested_nozzle_deg: tuple[float, float]
    achieved_nozzle_deg: tuple[float, float]
    requested_rcs_attitude_deg: tuple[float, float, float]
    requested_thrust_vector_unit_body: tuple[float, float, float]
    rcs_force_body_n: tuple[float, float, float]
    rcs_moment_body_nm: tuple[float, float, float]
    force_body_n: tuple[float, float, float]
    moment_body_nm: tuple[float, float, float]


####


class Rocket6gPlantRunResult(CadacModel):
    """Phase-aware mixed-fidelity ROCKET6G batch result."""

    schema_id: str = "taoryx.cadac.rocket6g-phase-aware-plant/v0alpha1"
    source_name: str = Field(min_length=1)
    integration_step_s: float = Field(gt=0.0)
    requested_end_time_s: float = Field(gt=0.0)
    executed_steps: int = Field(ge=0)
    terminated_reason: str = Field(min_length=1)
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=2)
    events: tuple[Rocket6gPhaseEvent, ...]
    samples: tuple[Rocket6gPlantSample, ...] = Field(min_length=1)
    claim_boundary: str = (
        "The returned run is one phase-aware rigid-body plant whose runtime fidelity and control realization are reported per sample. "
        "TVC is physical and second-order; RCS remains axis-aggregate direct wrench. LTG/autopilot/navigation parity is not claimed."
    )


####


@dataclass(slots=True)
class _Rocket6gRuntimeState:
    source_values: dict[str, int | float]
    event_cursor: CadacEventCursor
    event_time_s: float
    position_inertial_m: FloatVector
    velocity_inertial_mps: FloatVector
    acceleration_inertial_mps2: FloatVector
    body_from_inertial: FloatMatrix
    body_from_inertial_derivative: FloatMatrix
    body_rates_inertial_rad_s: FloatVector
    body_rate_derivative_rad_s2: FloatVector
    longitude_rad: float
    latitude_rad: float
    altitude_m: float
    geodetic_from_inertial: FloatMatrix
    geocentric_from_inertial: FloatMatrix
    velocity_geodetic_mps: FloatVector
    geographic_speed_mps: float
    heading_deg: float
    flight_path_deg: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    alpha_deg: float
    beta_deg: float
    total_alpha_deg: float
    aerodynamic_roll_deg: float
    body_rates_earth_rad_s: FloatVector
    density_kg_m3: float
    pressure_pa: float
    temperature_k: float
    speed_of_sound_mps: float
    mach: float
    dynamic_pressure_pa: float
    gravity_inertial_mps2: FloatVector
    propulsion: Rocket6gPropulsionState
    remaining_fuel_kg: float
    thrust_n: float
    tvc_state: Rocket6gTvcState
    tvc_step: Rocket6gTvcStep
    rcs_state: Rocket6gRcsState
    rcs_step: Rocket6gRcsStep
    aero_coefficients: Rocket6gAeroCoefficients
    wrench: Rocket6gBodyWrench
    specific_force_body_mps2: FloatVector


####


def load_rocket6g_source_definition(path: str | Path) -> Rocket6gSourceDefinition:
    """Parse, fingerprint, and lower one ROCKET6G ``input.asc`` source case."""

    return lower_rocket6g_source_bundle(load_cadac_source_bundle(path))


####


def lower_rocket6g_source_bundle(bundle: CadacSourceBundle) -> Rocket6gSourceDefinition:
    """Lower the standard single-HYPER6 three-stage source program."""

    vehicles = bundle.case.vehicles_named("HYPER6")
    if len(vehicles) != 1 or len(bundle.case.vehicles) != 1:
        raise Rocket6gSourceError(f"ROCKET6G source lowering requires exactly one HYPER6 vehicle; found {len(vehicles)}")
    ####
    module_order = tuple(module.name.casefold() for module in bundle.case.modules if CadacModuleStage.EXECUTE in module.stages)
    unknown = tuple(name for name in module_order if name not in _ROCKET6G_MODULES)
    if unknown:
        raise Rocket6gSourceError(f"ROCKET6G reconstruction does not implement source modules: {unknown!r}")
    ####
    missing = tuple(name for name in _ROCKET6G_REQUIRED_MODULES if name not in module_order)
    if missing:
        raise Rocket6gSourceError(f"ROCKET6G reconstruction requires source modules: {missing!r}")
    ####
    timing = bundle.case.timing_values
    integration_step_s = timing.get("int_step")
    if integration_step_s is None:
        raise Rocket6gSourceError("ROCKET6G source case must declare TIMING int_step")
    ####
    vehicle = vehicles[0]
    values = _initial_runtime_values(vehicle)
    if _integer_from_values(values, "mair", 0) != 0:
        raise Rocket6gSourceError("the first ROCKET6G plant slice supports the source mair=0 US76/no-wind path; weather/turbulence modes remain excluded")
    ####
    stages = _lower_source_stages(vehicle, values)
    tvc = Rocket6gTvcConfig(
        mode=_integer_from_values(values, "mtvc", 0),
        command_gain=_number_from_values(values, "gtvc", 0.0),
        propulsion_arm_from_nose_m=_number_from_values(values, "parm"),
        position_limit_deg=_number_from_values(values, "tvclimx"),
        rate_limit_deg_s=_number_from_values(values, "dtvclimx"),
        natural_frequency_rad_s=_number_from_values(values, "wntvc"),
        damping_ratio=_number_from_values(values, "zettvc"),
        variable_gain_factor=_number_from_values(values, "factgtvc", 0.0),
    )
    return Rocket6gSourceDefinition(
        source_name=bundle.case.source_name,
        integration_step_s=float(integration_step_s),
        plot_step_s=timing.get("plot_step"),
        end_time_s=bundle.case.end_time_s,
        module_order=module_order,
        initial_state=Rocket6gInitialState(
            longitude_deg=_number(vehicle, "lonx"),
            latitude_deg=_number(vehicle, "latx"),
            altitude_m=_number(vehicle, "alt"),
            geographic_speed_mps=_number(vehicle, "dvbe"),
            roll_deg=_number(vehicle, "phibdx"),
            pitch_deg=_number(vehicle, "thtbdx"),
            yaw_deg=_number(vehicle, "psibdx"),
            alpha_deg=_number(vehicle, "alpha0x", 0.0),
            beta_deg=_number(vehicle, "beta0x", 0.0),
            body_rates_deg_s=(
                _number(vehicle, "ppx", 0.0),
                _number(vehicle, "qqx", 0.0),
                _number(vehicle, "rrx", 0.0),
            ),
        ),
        reference_area_m2=_number(vehicle, "refa"),
        reference_length_m=_number(vehicle, "refd"),
        positive_alpha_limit_deg=_number(vehicle, "alplimx"),
        structural_load_limit_g=_number(vehicle, "alimitx"),
        initial_parameters=values,
        stages=stages,
        tvc=tvc,
        aerodynamic_deck=bundle.deck_for("HYPER6", CadacDeckKind.AERODYNAMIC),
        events=vehicle.events,
        source_artifacts=bundle.artifacts,
    )


####


def rocket6g_tvc_step(
    config: Rocket6gTvcConfig,
    state: Rocket6gTvcState,
    *,
    control_pitch_deg: float,
    control_yaw_deg: float,
    thrust_n: float,
    dynamic_pressure_pa: float,
    center_of_gravity_m: float,
    dt_s: float,
    mode_override: int | None = None,
    gain_override: float | None = None,
) -> Rocket6gTvcStep:
    """Execute the source physical TVC module at the direct control-command boundary."""

    mode = config.mode if mode_override is None else int(mode_override)
    if mode not in {0, 1, 2, 3}:
        raise ValueError("ROCKET6G TVC mode must be 0, 1, 2, or 3")
    ####
    values = (control_pitch_deg, control_yaw_deg, thrust_n, dynamic_pressure_pa, center_of_gravity_m, dt_s)
    if any(not math.isfinite(float(value)) for value in values) or dt_s <= 0.0:
        raise ValueError("ROCKET6G TVC inputs must be finite and dt_s positive")
    ####
    gain = config.command_gain if gain_override is None else float(gain_override)
    if mode == 3:
        gain = 0.0 if dynamic_pressure_pa > 1.0e5 else (-5.0e-6 * dynamic_pressure_pa + 0.5) * (config.variable_gain_factor + 1.0)
    ####
    requested_rad = np.asarray((gain * control_pitch_deg * RAD_PER_DEG, gain * control_yaw_deg * RAD_PER_DEG), dtype=np.float64)
    if mode == 0:
        return Rocket6gTvcStep(
            active=False,
            requested_control_deg=(control_pitch_deg, control_yaw_deg),
            requested_nozzle_deg=_tuple2(requested_rad * DEG_PER_RAD),
            achieved_nozzle_deg=_tuple2(np.asarray(state.position_rad) * DEG_PER_RAD),
            command_gain=gain,
            force_body_n=(0.0, 0.0, 0.0),
            moment_body_nm=(0.0, 0.0, 0.0),
            position_limited=(False, False),
            rate_limited=(False, False),
            state=state,
        )
    ####
    if mode == 1:
        achieved = requested_rad
        next_state = state
        position_limited = (False, False)
        rate_limited = (False, False)
    else:
        position = np.asarray(state.position_rad, dtype=np.float64)
        position_derivative = np.asarray(state.position_derivative_rad_s, dtype=np.float64)
        rate = np.asarray(state.rate_rad_s, dtype=np.float64)
        rate_derivative = np.asarray(state.rate_derivative_rad_s2, dtype=np.float64)
        position_limit = config.position_limit_deg * RAD_PER_DEG
        rate_limit = config.rate_limit_deg_s * RAD_PER_DEG
        position_flags = np.zeros(2, dtype=np.bool_)
        rate_flags = np.zeros(2, dtype=np.bool_)
        for index in range(2):
            if abs(position[index]) > position_limit:
                position_flags[index] = True
                position[index] = math.copysign(position_limit, position[index])
                if position[index] * rate[index] > 0.0:
                    rate[index] = 0.0
                ####
            ####
            if abs(rate[index]) > rate_limit:
                rate_flags[index] = True
                rate[index] = math.copysign(rate_limit, rate[index])
            ####
            position_derivative_new = rate[index]
            position[index] = _integrate_scalar(position[index], position_derivative_new, position_derivative[index], dt_s)
            position_derivative[index] = position_derivative_new
            error = requested_rad[index] - position[index]
            rate_derivative_new = (
                config.natural_frequency_rad_s * config.natural_frequency_rad_s * error
                - 2.0 * config.damping_ratio * config.natural_frequency_rad_s * position_derivative[index]
            )
            rate[index] = _integrate_scalar(rate[index], rate_derivative_new, rate_derivative[index], dt_s)
            rate_derivative[index] = rate_derivative_new
            if rate_flags[index] and rate[index] * rate_derivative[index] > 0.0:
                rate_derivative[index] = 0.0
            ####
        ####
        achieved = position
        next_state = Rocket6gTvcState(
            position_derivative_rad_s=_tuple2(position_derivative),
            position_rad=_tuple2(position),
            rate_derivative_rad_s2=_tuple2(rate_derivative),
            rate_rad_s=_tuple2(rate),
        )
        position_limited = (bool(position_flags[0]), bool(position_flags[1]))
        rate_limited = (bool(rate_flags[0]), bool(rate_flags[1]))
    ####
    eta, zeta = float(achieved[0]), float(achieved[1])
    force = np.asarray(
        (
            math.cos(eta) * math.cos(zeta) * thrust_n,
            math.cos(eta) * math.sin(zeta) * thrust_n,
            -math.sin(eta) * thrust_n,
        ),
        dtype=np.float64,
    )
    arm = config.propulsion_arm_from_nose_m - center_of_gravity_m
    moment = np.asarray((0.0, arm * force[2], -arm * force[1]), dtype=np.float64)
    return Rocket6gTvcStep(
        active=True,
        requested_control_deg=(control_pitch_deg, control_yaw_deg),
        requested_nozzle_deg=_tuple2(requested_rad * DEG_PER_RAD),
        achieved_nozzle_deg=_tuple2(achieved * DEG_PER_RAD),
        command_gain=gain,
        force_body_n=_tuple3(force),
        moment_body_nm=_tuple3(moment),
        position_limited=position_limited,
        rate_limited=rate_limited,
        state=next_state,
    )


####


def rocket6g_aerodynamic_coefficients(
    definition: Rocket6gSourceDefinition,
    *,
    aerodynamic_mode: int,
    propulsion_mode: int,
    mach: float,
    total_alpha_deg: float,
    aerodynamic_roll_deg: float,
    pitch_rate_deg_s: float,
    yaw_rate_deg_s: float,
    airspeed_mps: float,
    center_of_gravity_m: float,
    reference_cg_m: float,
) -> Rocket6gAeroCoefficients:
    """Evaluate the source stage-dependent body-axis aerodynamic closure."""

    if aerodynamic_mode == 0:
        return Rocket6gAeroCoefficients()
    ####
    stage = _STAGE_NUMBER_BY_AERO_MODE.get(aerodynamic_mode)
    if stage is None:
        raise ValueError(f"unsupported ROCKET6G aerodynamic mode {aerodynamic_mode}")
    ####
    if any(
        not math.isfinite(value)
        for value in (
            mach,
            total_alpha_deg,
            aerodynamic_roll_deg,
            pitch_rate_deg_s,
            yaw_rate_deg_s,
            airspeed_mps,
            center_of_gravity_m,
            reference_cg_m,
        )
    ):
        raise ValueError("ROCKET6G aerodynamic inputs must be finite")
    ####
    deck = definition.aerodynamic_deck
    axial_stage = 3 if stage == 1 else 2
    ca0 = deck.table(f"ca0slv{axial_stage}_vs_mach").interpolate((mach,))
    caa = deck.table(f"caaslv{axial_stage}_vs_mach").interpolate((mach,))
    ca0b = deck.table(f"ca0bslv{axial_stage}_vs_mach").interpolate((mach,))
    ca = ca0 + caa * total_alpha_deg + float(propulsion_mode != 0) * ca0b
    normal = deck.table(f"cn0slv{stage}_vs_mach_alpha").interpolate((mach, total_alpha_deg))
    pitch_base = deck.table(f"clm0slv{stage}_vs_mach_alpha").interpolate((mach, total_alpha_deg))
    pitch_damping = deck.table(f"clmqslv{stage}_vs_mach").interpolate((mach,))
    phi = aerodynamic_roll_deg * RAD_PER_DEG
    q_aero_deg_s = pitch_rate_deg_s * math.cos(phi) - yaw_rate_deg_s * math.sin(phi)
    speed = max(abs(airspeed_mps), 1.0e-9)
    pitch_reference = pitch_base + pitch_damping * q_aero_deg_s * definition.reference_length_m / (2.0 * speed)
    pitch = pitch_reference - normal * (reference_cg_m - center_of_gravity_m) / definition.reference_length_m
    return Rocket6gAeroCoefficients(
        cx=-ca,
        cy=-normal * math.sin(phi),
        cz=-normal * math.cos(phi),
        cl=0.0,
        cm=pitch * math.cos(phi),
        cn=-pitch * math.sin(phi),
    )


####


def rocket6g_body_wrench(
    definition: Rocket6gSourceDefinition,
    coefficients: Rocket6gAeroCoefficients,
    *,
    dynamic_pressure_pa: float,
    propulsion_mode: int,
    thrust_n: float,
    tvc: Rocket6gTvcStep,
    rcs: Rocket6gRcsStep,
) -> Rocket6gBodyWrench:
    """Assemble aerodynamic, plain-thrust/TVC, and aggregate-RCS source wrench."""

    q_area = dynamic_pressure_pa * definition.reference_area_m2
    force = np.asarray((q_area * coefficients.cx, q_area * coefficients.cy, q_area * coefficients.cz), dtype=np.float64)
    moment = np.asarray(
        (
            q_area * definition.reference_length_m * coefficients.cl,
            q_area * definition.reference_length_m * coefficients.cm,
            q_area * definition.reference_length_m * coefficients.cn,
        ),
        dtype=np.float64,
    )
    if tvc.active:
        force += np.asarray(tvc.force_body_n, dtype=np.float64)
        moment += np.asarray(tvc.moment_body_nm, dtype=np.float64)
    elif propulsion_mode != 0:
        force[0] += thrust_n
    ####
    force += np.asarray(rcs.force_body_n, dtype=np.float64)
    moment += np.asarray(rcs.moment_body_nm, dtype=np.float64)
    return Rocket6gBodyWrench(force_body_n=_tuple3(force), moment_body_nm=_tuple3(moment))


####


def run_rocket6g_phase_aware_plant(
    definition: Rocket6gSourceDefinition,
    command: Rocket6gDirectCommand | None = None,
    *,
    end_time_s: float | None = None,
    sample_step_s: float | None = None,
) -> Rocket6gPlantRunResult:
    """Run one source-ordered three-stage rigid-body program with phase telemetry."""

    dt_s = definition.integration_step_s
    requested_end = definition.end_time_s if end_time_s is None else float(end_time_s)
    if not math.isfinite(requested_end) or requested_end <= 0.0:
        raise ValueError("ROCKET6G end_time_s must be positive and finite")
    ####
    cadence = max(dt_s, 0.02) if sample_step_s is None else float(sample_step_s)
    if not math.isfinite(cadence) or cadence <= 0.0:
        raise ValueError("ROCKET6G sample_step_s must be positive and finite")
    ####
    resolved_command = command or Rocket6gDirectCommand()
    runtime = _initialize_runtime(definition)
    samples: list[Rocket6gPlantSample] = []
    events: list[Rocket6gPhaseEvent] = []
    next_sample_time = 0.0
    sim_time = 0.0
    steps = 0
    while sim_time <= requested_end + 0.5 * dt_s:
        steps += 1
        application = _evaluate_source_event(runtime, resolved_command, sim_time)
        if application is not None:
            events.append(application)
        ####
        for module in definition.module_order:
            if module == "kinematics":
                _runtime_kinematics(runtime, dt_s, sim_time)
            elif module == "environment":
                _runtime_environment(runtime, sim_time)
            elif module == "propulsion":
                _runtime_propulsion(runtime, dt_s)
            elif module == "aerodynamics":
                runtime.aero_coefficients = rocket6g_aerodynamic_coefficients(
                    definition,
                    aerodynamic_mode=_int_value(runtime.source_values, "maero", 0),
                    propulsion_mode=_int_value(runtime.source_values, "mprop", 0),
                    mach=runtime.mach,
                    total_alpha_deg=runtime.total_alpha_deg,
                    aerodynamic_roll_deg=runtime.aerodynamic_roll_deg,
                    pitch_rate_deg_s=float(runtime.body_rates_earth_rad_s[1]) * DEG_PER_RAD,
                    yaw_rate_deg_s=float(runtime.body_rates_earth_rad_s[2]) * DEG_PER_RAD,
                    airspeed_mps=max(runtime.geographic_speed_mps, 1.0e-9),
                    center_of_gravity_m=runtime.propulsion.center_of_gravity_m,
                    reference_cg_m=_float_value(runtime.source_values, "xcg_ref", 0.0),
                )
            elif module in {"gps", "startrack", "ins", "guidance", "control"}:
                # Direct truth/effector boundary. Navigation and source command generation are intentionally excluded.
                pass
            elif module == "rcs":
                _runtime_rcs(runtime, resolved_command)
            elif module == "actuator":
                # ROCKET6G source includes the aerodynamic-surface module, but the insertion case does not activate it.
                pass
            elif module == "tvc":
                _runtime_tvc(runtime, definition, resolved_command, dt_s)
            elif module == "forces":
                runtime.wrench = rocket6g_body_wrench(
                    definition,
                    runtime.aero_coefficients,
                    dynamic_pressure_pa=runtime.dynamic_pressure_pa,
                    propulsion_mode=_int_value(runtime.source_values, "mprop", 0),
                    thrust_n=runtime.thrust_n,
                    tvc=runtime.tvc_step,
                    rcs=runtime.rcs_step,
                )
            elif module == "newton":
                _runtime_newton(runtime, dt_s, sim_time)
            elif module == "euler":
                _runtime_euler(runtime, dt_s)
            elif module == "intercept":
                pass
            ####
        ####
        if sim_time + 0.5 * dt_s >= next_sample_time or sim_time + 0.5 * dt_s >= requested_end:
            samples.append(_runtime_sample(runtime, resolved_command, sim_time))
            while next_sample_time <= sim_time + 0.5 * dt_s:
                next_sample_time += cadence
            ####
        ####
        if not _runtime_is_finite(runtime):
            return Rocket6gPlantRunResult(
                source_name=definition.source_name,
                integration_step_s=dt_s,
                requested_end_time_s=requested_end,
                executed_steps=steps,
                terminated_reason="nonfinite_state",
                source_artifacts=definition.source_artifacts,
                events=tuple(events),
                samples=tuple(samples),
            )
        ####
        sim_time += dt_s
        runtime.event_time_s += dt_s
    ####
    return Rocket6gPlantRunResult(
        source_name=definition.source_name,
        integration_step_s=dt_s,
        requested_end_time_s=requested_end,
        executed_steps=steps,
        terminated_reason="end_time",
        source_artifacts=definition.source_artifacts,
        events=tuple(events),
        samples=tuple(samples),
    )


####


def _initialize_runtime(definition: Rocket6gSourceDefinition) -> _Rocket6gRuntimeState:
    initial = definition.initial_state
    values = dict(definition.initial_parameters)
    lon = initial.longitude_deg * RAD_PER_DEG
    lat = initial.latitude_deg * RAD_PER_DEG
    position = _cad_in_geo84(lon, lat, initial.altitude_m, 0.0)
    tdi = _cad_tdi84(lon, lat, initial.altitude_m, 0.0)
    tgi = _cad_tgi84(lon, lat, initial.altitude_m, 0.0)
    body_from_geodetic = _mat3tr(initial.yaw_deg * RAD_PER_DEG, initial.pitch_deg * RAD_PER_DEG, initial.roll_deg * RAD_PER_DEG)
    body_from_inertial = body_from_geodetic @ tdi
    alpha = initial.alpha_deg * RAD_PER_DEG
    beta = initial.beta_deg * RAD_PER_DEG
    body_velocity = np.asarray(
        (
            math.cos(alpha) * math.cos(beta) * initial.geographic_speed_mps,
            math.sin(beta) * initial.geographic_speed_mps,
            math.sin(alpha) * math.cos(beta) * initial.geographic_speed_mps,
        ),
        dtype=np.float64,
    )
    geodetic_velocity = body_from_geodetic.T @ body_velocity
    omega_skew = _skew(np.asarray((0.0, 0.0, WEII3), dtype=np.float64))
    velocity_inertial = tdi.T @ geodetic_velocity + omega_skew @ position
    body_rates_earth = np.asarray(initial.body_rates_deg_s, dtype=np.float64) * RAD_PER_DEG
    body_rates_inertial = body_rates_earth + body_from_inertial @ np.asarray((0.0, 0.0, WEII3), dtype=np.float64)
    stage = definition.stages[0]
    propulsion = Rocket6gPropulsionState(
        mass_kg=stage.initial_mass_kg,
        center_of_gravity_m=stage.initial_cg_m,
        inertia_diagonal_kgm2=(
            stage.initial_roll_inertia_kgm2,
            stage.initial_transverse_inertia_kgm2,
            stage.initial_transverse_inertia_kgm2,
        ),
    )
    zero_tvc = Rocket6gTvcStep(
        active=False,
        requested_control_deg=(0.0, 0.0),
        requested_nozzle_deg=(0.0, 0.0),
        achieved_nozzle_deg=(0.0, 0.0),
        command_gain=definition.tvc.command_gain,
        force_body_n=(0.0, 0.0, 0.0),
        moment_body_nm=(0.0, 0.0, 0.0),
        position_limited=(False, False),
        rate_limited=(False, False),
        state=Rocket6gTvcState(),
    )
    zero_rcs = Rocket6gRcsStep(
        force_body_n=(0.0, 0.0, 0.0),
        moment_body_nm=(0.0, 0.0, 0.0),
        errors=(0.0, 0.0, 0.0, 0.0, 0.0),
        state=Rocket6gRcsState(),
    )
    return _Rocket6gRuntimeState(
        source_values=values,
        event_cursor=CadacEventCursor.from_events(definition.events),
        event_time_s=0.0,
        position_inertial_m=position,
        velocity_inertial_mps=velocity_inertial,
        acceleration_inertial_mps2=np.zeros(3, dtype=np.float64),
        body_from_inertial=body_from_inertial,
        body_from_inertial_derivative=np.zeros((3, 3), dtype=np.float64),
        body_rates_inertial_rad_s=body_rates_inertial,
        body_rate_derivative_rad_s2=np.zeros(3, dtype=np.float64),
        longitude_rad=lon,
        latitude_rad=lat,
        altitude_m=initial.altitude_m,
        geodetic_from_inertial=tdi,
        geocentric_from_inertial=tgi,
        velocity_geodetic_mps=geodetic_velocity,
        geographic_speed_mps=initial.geographic_speed_mps,
        heading_deg=_polar_angles(geodetic_velocity)[0],
        flight_path_deg=_polar_angles(geodetic_velocity)[1],
        roll_deg=initial.roll_deg,
        pitch_deg=initial.pitch_deg,
        yaw_deg=initial.yaw_deg,
        alpha_deg=initial.alpha_deg,
        beta_deg=initial.beta_deg,
        total_alpha_deg=math.hypot(initial.alpha_deg, initial.beta_deg),
        aerodynamic_roll_deg=0.0,
        body_rates_earth_rad_s=body_rates_earth,
        density_kg_m3=0.0,
        pressure_pa=101_325.0,
        temperature_k=288.15,
        speed_of_sound_mps=340.0,
        mach=0.0,
        dynamic_pressure_pa=0.0,
        gravity_inertial_mps2=np.zeros(3, dtype=np.float64),
        propulsion=propulsion,
        remaining_fuel_kg=stage.initial_fuel_mass_kg,
        thrust_n=0.0,
        tvc_state=Rocket6gTvcState(),
        tvc_step=zero_tvc,
        rcs_state=Rocket6gRcsState(),
        rcs_step=zero_rcs,
        aero_coefficients=Rocket6gAeroCoefficients(),
        wrench=Rocket6gBodyWrench(force_body_n=(0.0, 0.0, 0.0), moment_body_nm=(0.0, 0.0, 0.0)),
        specific_force_body_mps2=np.zeros(3, dtype=np.float64),
    )


####


def _evaluate_source_event(
    runtime: _Rocket6gRuntimeState,
    command: Rocket6gDirectCommand,
    sim_time_s: float,
) -> Rocket6gPhaseEvent | None:
    runtime.source_values["time"] = float(sim_time_s)
    runtime.source_values["event_time"] = float(runtime.event_time_s)
    runtime.source_values["thrust"] = float(runtime.thrust_n)
    if command.boost_cutoff_time_s is not None and sim_time_s >= command.boost_cutoff_time_s:
        runtime.source_values["beco_flag"] = 1
    ####
    phase_before = _source_phase(runtime.source_values, runtime.thrust_n)
    stage_before = _active_stage(runtime.source_values)
    fidelity_before = _phase_fidelity(phase_before)
    application = runtime.event_cursor.evaluate_and_apply(runtime.source_values)
    if application is None:
        return None
    ####
    runtime.event_time_s = 0.0
    _apply_event_state_mutations(runtime, application)
    phase_after = _source_phase(runtime.source_values, runtime.thrust_n)
    stage_after = _active_stage(runtime.source_values)
    return Rocket6gPhaseEvent(
        event_index=application.event_index,
        time_s=sim_time_s,
        source_line=application.source_line,
        watch_variable=application.watch_variable,
        operator=application.operator.value,
        criterion=application.criterion,
        previous_values=application.previous_values,
        updated_values=application.updated_values,
        phase_before=phase_before,
        phase_after=phase_after,
        stage_before=stage_before,
        stage_after=stage_after,
        runtime_fidelity_before=fidelity_before,
        runtime_fidelity_after=_phase_fidelity(phase_after),
    )


####


def _apply_event_state_mutations(runtime: _Rocket6gRuntimeState, application: CadacEventApplication) -> None:
    updated_names = {name.casefold() for name, _ in application.updated_values}
    if "fmasse" in updated_names:
        runtime.propulsion = runtime.propulsion.model_copy(update={"fuel_expended_kg": _float_value(runtime.source_values, "fmasse", 0.0)})
    ####


####


def _runtime_kinematics(runtime: _Rocket6gRuntimeState, dt_s: float, sim_time_s: float) -> None:
    derivative_new = -_skew(runtime.body_rates_inertial_rad_s) @ runtime.body_from_inertial
    runtime.body_from_inertial = _integrate_array(
        runtime.body_from_inertial,
        derivative_new,
        runtime.body_from_inertial_derivative,
        dt_s,
    )
    runtime.body_from_inertial_derivative = derivative_new
    identity = np.eye(3, dtype=np.float64)
    error = identity - runtime.body_from_inertial @ runtime.body_from_inertial.T
    runtime.body_from_inertial = runtime.body_from_inertial + 0.5 * error @ runtime.body_from_inertial
    lon, lat, alt = _cad_geo84_in(runtime.position_inertial_m, sim_time_s)
    runtime.longitude_rad = lon
    runtime.latitude_rad = lat
    runtime.altitude_m = alt
    runtime.geodetic_from_inertial = _cad_tdi84(lon, lat, alt, sim_time_s)
    runtime.geocentric_from_inertial = _cad_tgi84(lon, lat, alt, sim_time_s)
    body_from_geodetic = runtime.body_from_inertial @ runtime.geodetic_from_inertial.T
    runtime.yaw_deg, runtime.pitch_deg, runtime.roll_deg = _euler_from_dcm(body_from_geodetic)
    earth_rotation_body = runtime.body_from_inertial @ np.asarray((0.0, 0.0, WEII3), dtype=np.float64)
    runtime.body_rates_earth_rad_s = runtime.body_rates_inertial_rad_s - earth_rotation_body
    air_body = body_from_geodetic @ runtime.velocity_geodetic_mps
    speed = max(float(np.linalg.norm(air_body)), 1.0e-12)
    runtime.alpha_deg = math.atan2(float(air_body[2]), float(air_body[0])) * DEG_PER_RAD
    beta_argument = min(1.0, max(-1.0, float(air_body[1]) / speed))
    runtime.beta_deg = math.asin(beta_argument) * DEG_PER_RAD
    x_argument = min(1.0, max(-1.0, float(air_body[0]) / speed))
    total_alpha = math.acos(x_argument)
    if air_body[1] == 0.0 and air_body[2] == 0.0:
        aero_roll = 0.0
    elif abs(float(air_body[1])) < 1.0e-10:
        aero_roll = 0.0 if air_body[2] > 0.0 else math.pi
    else:
        aero_roll = math.atan2(float(air_body[1]), float(air_body[2]))
    ####
    runtime.total_alpha_deg = total_alpha * DEG_PER_RAD
    runtime.aerodynamic_roll_deg = aero_roll * DEG_PER_RAD


####


def _runtime_environment(runtime: _Rocket6gRuntimeState, sim_time_s: float) -> None:
    density, pressure, temperature = atmosphere76(runtime.altitude_m)
    runtime.density_kg_m3 = density
    runtime.pressure_pa = pressure
    runtime.temperature_k = temperature
    runtime.speed_of_sound_mps = math.sqrt(1.4 * RGAS * temperature)
    relative = runtime.velocity_geodetic_mps
    runtime.geographic_speed_mps = float(np.linalg.norm(relative))
    runtime.mach = abs(runtime.geographic_speed_mps / runtime.speed_of_sound_mps)
    runtime.dynamic_pressure_pa = 0.5 * density * runtime.geographic_speed_mps * runtime.geographic_speed_mps
    runtime.gravity_inertial_mps2 = runtime.geocentric_from_inertial.T @ _cad_grav84_geocentric(runtime.position_inertial_m, sim_time_s)


####


def _runtime_propulsion(runtime: _Rocket6gRuntimeState, dt_s: float) -> None:
    mode_before = _int_value(runtime.source_values, "mprop", 0)
    if mode_before == 0:
        runtime.thrust_n = 0.0
        runtime.remaining_fuel_kg = 0.0
        runtime.propulsion = runtime.propulsion.model_copy(update={"fuel_expended_derivative_kg_s": 0.0, "fuel_expended_kg": 0.0})
        runtime.source_values["fmasse"] = 0.0
        runtime.source_values["thrust"] = 0.0
        return
    ####
    if mode_before not in {3, 4}:
        raise ValueError(f"unsupported ROCKET6G propulsion mode {mode_before}")
    ####
    isp = _float_value(runtime.source_values, "spi")
    source_flow = _float_value(runtime.source_values, "fuel_flow_rate")
    exit_area = _float_value(runtime.source_values, "aexit", 0.0)
    thrust = isp * source_flow * AGRAV + (101_325.0 - runtime.pressure_pa) * exit_area
    derivative_new = thrust / (isp * AGRAV)
    fuel_expended = _integrate_scalar(
        runtime.propulsion.fuel_expended_kg,
        derivative_new,
        runtime.propulsion.fuel_expended_derivative_kg_s,
        dt_s,
    )
    initial_mass = _float_value(runtime.source_values, "vmass0")
    initial_fuel = _float_value(runtime.source_values, "fmass0")
    ratio = fuel_expended / initial_fuel
    mass = initial_mass - fuel_expended
    center = (
        _float_value(runtime.source_values, "xcg_0") + (_float_value(runtime.source_values, "xcg_1") - _float_value(runtime.source_values, "xcg_0")) * ratio
    )
    roll_inertia = (
        _float_value(runtime.source_values, "moi_roll_0")
        + (_float_value(runtime.source_values, "moi_roll_1") - _float_value(runtime.source_values, "moi_roll_0")) * ratio
    )
    transverse = (
        _float_value(runtime.source_values, "moi_trans_0")
        + (_float_value(runtime.source_values, "moi_trans_1") - _float_value(runtime.source_values, "moi_trans_0")) * ratio
    )
    remaining = initial_fuel - fuel_expended
    burned_out = remaining <= 0.0
    if burned_out:
        thrust = 0.0
        runtime.source_values["mprop"] = 0
    ####
    runtime.propulsion = Rocket6gPropulsionState(
        fuel_expended_derivative_kg_s=derivative_new,
        fuel_expended_kg=fuel_expended,
        mass_kg=max(mass, 1.0e-9),
        center_of_gravity_m=center,
        inertia_diagonal_kgm2=(max(roll_inertia, 1.0e-9), max(transverse, 1.0e-9), max(transverse, 1.0e-9)),
    )
    runtime.remaining_fuel_kg = remaining
    runtime.thrust_n = thrust
    runtime.source_values["fmasse"] = fuel_expended
    runtime.source_values["thrust"] = thrust
    runtime.source_values["vmass"] = runtime.propulsion.mass_kg
    runtime.source_values["xcg"] = center


####


def _runtime_rcs(runtime: _Rocket6gRuntimeState, command: Rocket6gDirectCommand) -> None:
    source = runtime.source_values
    config = Rocket6gRcsConfig(
        moment_mode=_int_value(source, "mrcs_moment", 0),
        force_mode=_int_value(source, "mrcs_force", 0),
        dead_zone=_float_value(source, "dead_zone", 0.0),
        hysteresis=_float_value(source, "hysteresis", 0.0),
        time_slope_s=_float_value(source, "rcs_tau", 0.0),
        roll_moment_limit_nm=_float_value(source, "roll_mom_max", 0.0),
        pitch_moment_limit_nm=_float_value(source, "pitch_mom_max", 0.0),
        yaw_moment_limit_nm=_float_value(source, "yaw_mom_max", 0.0),
        proportional_damping=_float_value(source, "rcs_zeta", 0.0),
        proportional_frequency_rad_s=_float_value(source, "rcs_freq", 0.0),
        acceleration_gain_n_per_mps2=_float_value(source, "acc_gain", 0.0),
        side_force_limit_n=_float_value(source, "side_force_max", 0.0),
        roll_command_deg=(_float_value(source, "phibdcomx", 0.0) if command.roll_command_deg is None else command.roll_command_deg),
        pitch_command_deg=(_float_value(source, "thtbdcomx", 0.0) if command.pitch_command_deg is None else command.pitch_command_deg),
        yaw_command_deg=(_float_value(source, "psibdcomx", 0.0) if command.yaw_command_deg is None else command.yaw_command_deg),
    )
    runtime.rcs_step = rocket6g_rcs_step(
        config,
        Rocket6gRcsRuntimeInput(
            inertia_diagonal_kgm2=runtime.propulsion.inertia_diagonal_kgm2,
            body_rates_rad_s=_tuple3(runtime.body_rates_earth_rad_s),
            geodetic_angles_deg=(runtime.roll_deg, runtime.pitch_deg, runtime.yaw_deg),
            thrust_vector_unit_body=command.normalized_thrust_vector(),
            incidence_deg=(runtime.alpha_deg, runtime.beta_deg),
            incidence_commands_deg=(
                _float_value(source, "alphacomx", 0.0),
                _float_value(source, "betacomx", 0.0),
            ),
            specific_force_body_mps2=_tuple3(runtime.specific_force_body_mps2),
            acceleration_commands_g=(
                _float_value(source, "aycomx", 0.0),
                _float_value(source, "azcomx", 0.0),
            ),
        ),
        runtime.rcs_state,
    )
    runtime.rcs_state = runtime.rcs_step.state


####


def _runtime_tvc(
    runtime: _Rocket6gRuntimeState,
    definition: Rocket6gSourceDefinition,
    command: Rocket6gDirectCommand,
    dt_s: float,
) -> None:
    runtime.tvc_step = rocket6g_tvc_step(
        definition.tvc,
        runtime.tvc_state,
        control_pitch_deg=command.tvc_pitch_command_deg,
        control_yaw_deg=command.tvc_yaw_command_deg,
        thrust_n=runtime.thrust_n,
        dynamic_pressure_pa=runtime.dynamic_pressure_pa,
        center_of_gravity_m=runtime.propulsion.center_of_gravity_m,
        dt_s=dt_s,
        mode_override=_int_value(runtime.source_values, "mtvc", 0),
        gain_override=_float_value(runtime.source_values, "gtvc", definition.tvc.command_gain),
    )
    runtime.tvc_state = runtime.tvc_step.state


####


def _runtime_newton(runtime: _Rocket6gRuntimeState, dt_s: float, sim_time_s: float) -> None:
    force_body = np.asarray(runtime.wrench.force_body_n, dtype=np.float64)
    runtime.specific_force_body_mps2 = force_body / runtime.propulsion.mass_kg
    acceleration_new = runtime.body_from_inertial.T @ runtime.specific_force_body_mps2 + runtime.gravity_inertial_mps2
    old_velocity = runtime.velocity_inertial_mps.copy()
    next_velocity = _integrate_array(
        runtime.velocity_inertial_mps,
        acceleration_new,
        runtime.acceleration_inertial_mps2,
        dt_s,
    )
    runtime.position_inertial_m = _integrate_array(
        runtime.position_inertial_m,
        next_velocity,
        old_velocity,
        dt_s,
    )
    runtime.acceleration_inertial_mps2 = acceleration_new
    runtime.velocity_inertial_mps = next_velocity
    lon, lat, alt = _cad_geo84_in(runtime.position_inertial_m, sim_time_s)
    runtime.longitude_rad = lon
    runtime.latitude_rad = lat
    runtime.altitude_m = alt
    runtime.geodetic_from_inertial = _cad_tdi84(lon, lat, alt, sim_time_s)
    runtime.geocentric_from_inertial = _cad_tgi84(lon, lat, alt, sim_time_s)
    omega_skew = _skew(np.asarray((0.0, 0.0, WEII3), dtype=np.float64))
    runtime.velocity_geodetic_mps = runtime.geodetic_from_inertial @ (runtime.velocity_inertial_mps - omega_skew @ runtime.position_inertial_m)
    runtime.geographic_speed_mps = float(np.linalg.norm(runtime.velocity_geodetic_mps))
    runtime.heading_deg, runtime.flight_path_deg = _polar_angles(runtime.velocity_geodetic_mps)


####


def _runtime_euler(runtime: _Rocket6gRuntimeState, dt_s: float) -> None:
    inertia = np.diag(np.asarray(runtime.propulsion.inertia_diagonal_kgm2, dtype=np.float64))
    omega = runtime.body_rates_inertial_rad_s
    moment = np.asarray(runtime.wrench.moment_body_nm, dtype=np.float64)
    derivative_new = np.linalg.solve(inertia, moment - np.cross(omega, inertia @ omega))
    runtime.body_rates_inertial_rad_s = _integrate_array(
        omega,
        derivative_new,
        runtime.body_rate_derivative_rad_s2,
        dt_s,
    )
    runtime.body_rate_derivative_rad_s2 = derivative_new


####


def _runtime_sample(
    runtime: _Rocket6gRuntimeState,
    command: Rocket6gDirectCommand,
    sim_time_s: float,
) -> Rocket6gPlantSample:
    phase = _source_phase(runtime.source_values, runtime.thrust_n)
    return Rocket6gPlantSample(
        time_s=max(0.0, sim_time_s),
        position_inertial_m=_tuple3(runtime.position_inertial_m),
        velocity_inertial_mps=_tuple3(runtime.velocity_inertial_mps),
        quaternion_wxyz=_dcm_to_quaternion(runtime.body_from_inertial),
        body_rates_inertial_rad_s=_tuple3(runtime.body_rates_inertial_rad_s),
        body_rates_earth_rad_s=_tuple3(runtime.body_rates_earth_rad_s),
        longitude_deg=runtime.longitude_rad * DEG_PER_RAD,
        latitude_deg=runtime.latitude_rad * DEG_PER_RAD,
        altitude_m=runtime.altitude_m,
        geographic_speed_mps=max(0.0, runtime.geographic_speed_mps),
        heading_deg=runtime.heading_deg,
        flight_path_deg=runtime.flight_path_deg,
        roll_deg=runtime.roll_deg,
        pitch_deg=runtime.pitch_deg,
        yaw_deg=runtime.yaw_deg,
        alpha_deg=runtime.alpha_deg,
        beta_deg=runtime.beta_deg,
        total_alpha_deg=runtime.total_alpha_deg,
        aerodynamic_roll_deg=runtime.aerodynamic_roll_deg,
        mach=max(0.0, runtime.mach),
        dynamic_pressure_pa=max(0.0, runtime.dynamic_pressure_pa),
        active_stage=_active_stage(runtime.source_values),
        source_phase=phase,
        runtime_fidelity=_phase_fidelity(phase),
        control_realization=_phase_control_realization(phase),
        propulsion_mode=_int_value(runtime.source_values, "mprop", 0),
        rcs_moment_mode=_int_value(runtime.source_values, "mrcs_moment", 0),
        rcs_force_mode=_int_value(runtime.source_values, "mrcs_force", 0),
        tvc_mode=_int_value(runtime.source_values, "mtvc", 0),
        mass_kg=runtime.propulsion.mass_kg,
        remaining_fuel_kg=runtime.remaining_fuel_kg,
        center_of_gravity_m=runtime.propulsion.center_of_gravity_m,
        inertia_diagonal_kgm2=runtime.propulsion.inertia_diagonal_kgm2,
        thrust_n=runtime.thrust_n,
        requested_tvc_control_deg=(command.tvc_pitch_command_deg, command.tvc_yaw_command_deg),
        requested_nozzle_deg=runtime.tvc_step.requested_nozzle_deg,
        achieved_nozzle_deg=runtime.tvc_step.achieved_nozzle_deg,
        requested_rcs_attitude_deg=(
            _float_value(runtime.source_values, "phibdcomx", 0.0) if command.roll_command_deg is None else command.roll_command_deg,
            _float_value(runtime.source_values, "thtbdcomx", 0.0) if command.pitch_command_deg is None else command.pitch_command_deg,
            _float_value(runtime.source_values, "psibdcomx", 0.0) if command.yaw_command_deg is None else command.yaw_command_deg,
        ),
        requested_thrust_vector_unit_body=command.normalized_thrust_vector(),
        rcs_force_body_n=runtime.rcs_step.force_body_n,
        rcs_moment_body_nm=runtime.rcs_step.moment_body_nm,
        force_body_n=runtime.wrench.force_body_n,
        moment_body_nm=runtime.wrench.moment_body_nm,
    )


####


def _runtime_is_finite(runtime: _Rocket6gRuntimeState) -> bool:
    arrays = (
        runtime.position_inertial_m,
        runtime.velocity_inertial_mps,
        runtime.acceleration_inertial_mps2,
        runtime.body_from_inertial,
        runtime.body_rates_inertial_rad_s,
        runtime.body_rate_derivative_rad_s2,
        runtime.velocity_geodetic_mps,
        runtime.gravity_inertial_mps2,
        runtime.specific_force_body_mps2,
    )
    scalars = (
        runtime.longitude_rad,
        runtime.latitude_rad,
        runtime.altitude_m,
        runtime.geographic_speed_mps,
        runtime.mach,
        runtime.dynamic_pressure_pa,
        runtime.propulsion.mass_kg,
        runtime.thrust_n,
    )
    return all(np.all(np.isfinite(array)) for array in arrays) and all(math.isfinite(value) for value in scalars)


####


def _initial_runtime_values(vehicle: CadacVehicleBlock) -> dict[str, int | float]:
    values: dict[str, int | float] = {}
    for assignment in vehicle.assignments:
        if isinstance(assignment.value, (int, float)):
            values[assignment.name] = assignment.value
        ####
    ####
    for event in vehicle.events:
        for assignment in event.assignments:
            if assignment.name not in values and isinstance(assignment.value, (int, float)):
                values[assignment.name] = 0 if isinstance(assignment.value, int) else 0.0
            ####
        ####
    ####
    defaults: dict[str, int | float] = {
        "time": 0.0,
        "event_time": 0.0,
        "thrust": 0.0,
        "fmasse": 0.0,
        "vmass": _number(vehicle, "vmass0"),
        "xcg": _number(vehicle, "xcg_0"),
        "beco_flag": 0,
        "mrcs_force": 0,
        "rcs_zeta": 0.0,
        "rcs_freq": 0.0,
        "acc_gain": 0.0,
        "side_force_max": 0.0,
        "phibdcomx": 0.0,
        "aexit": 0.0,
        "factgtvc": 0.0,
        "alphacomx": 0.0,
        "betacomx": 0.0,
        "aycomx": 0.0,
        "azcomx": 0.0,
    }
    for name, value in defaults.items():
        values.setdefault(name, value)
    ####
    return values


####


def _lower_source_stages(
    vehicle: CadacVehicleBlock,
    initial_values: dict[str, int | float],
) -> tuple[Rocket6gStageConfig, ...]:
    values = dict(initial_values)
    stages: dict[int, Rocket6gStageConfig] = {}

    def capture() -> None:
        mode = _int_value(values, "maero", 0)
        stage_number = _STAGE_NUMBER_BY_AERO_MODE.get(mode)
        if stage_number is None or stage_number in stages:
            return
        ####
        stages[stage_number] = Rocket6gStageConfig(
            stage_number=stage_number,
            aerodynamic_mode=mode,
            propulsion_mode=_int_value(values, "mprop", 0),
            reference_cg_m=_float_value(values, "xcg_ref"),
            initial_mass_kg=_float_value(values, "vmass0"),
            initial_fuel_mass_kg=_float_value(values, "fmass0"),
            initial_cg_m=_float_value(values, "xcg_0"),
            final_cg_m=_float_value(values, "xcg_1"),
            initial_roll_inertia_kgm2=_float_value(values, "moi_roll_0"),
            final_roll_inertia_kgm2=_float_value(values, "moi_roll_1"),
            initial_transverse_inertia_kgm2=_float_value(values, "moi_trans_0"),
            final_transverse_inertia_kgm2=_float_value(values, "moi_trans_1"),
            specific_impulse_s=_float_value(values, "spi"),
            source_fuel_flow_kg_s=_float_value(values, "fuel_flow_rate"),
            nozzle_exit_area_m2=_float_value(values, "aexit", 0.0),
        )

    ####

    capture()
    for event in vehicle.events:
        for assignment in event.assignments:
            if isinstance(assignment.value, (int, float)):
                current = values.get(assignment.name)
                if isinstance(current, int) and not isinstance(current, bool):
                    values[assignment.name] = int(assignment.value)
                else:
                    values[assignment.name] = float(assignment.value)
                ####
            ####
        ####
        capture()
    ####
    missing = tuple(number for number in (1, 2, 3) if number not in stages)
    if missing:
        raise Rocket6gSourceError(f"ROCKET6G source case does not declare all three stages: missing {missing!r}")
    ####
    return tuple(stages[number] for number in (1, 2, 3))


####


def _source_phase(values: dict[str, int | float], thrust_n: float) -> str:
    tvc_mode = _int_value(values, "mtvc", 0)
    rcs_active = _int_value(values, "mrcs_moment", 0) > 0 or _int_value(values, "mrcs_force", 0) > 0
    tvc_effective = tvc_mode > 0 and thrust_n > 0.0
    if tvc_effective and rcs_active:
        return "mixed_tvc_rcs"
    ####
    if tvc_effective:
        return "physical_tvc"
    ####
    if rcs_active:
        return "aggregate_rcs"
    ####
    return "ballistic_coast"


####


def _phase_fidelity(phase: str) -> str:
    if phase in {"physical_tvc", "mixed_tvc_rcs"}:
        return "rigid_body_6dof_surface_allocated"
    ####
    return "rigid_body_6dof_direct_wrench"


####


def _phase_control_realization(phase: str) -> str:
    return {
        "aggregate_rcs": "axis_aggregate_direct_wrench",
        "physical_tvc": "physical_tvc_effector",
        "mixed_tvc_rcs": "physical_tvc_plus_axis_aggregate_rcs",
        "ballistic_coast": "uncontrolled_rigid_body",
    }[phase]


####


def _active_stage(values: dict[str, int | float]) -> int:
    mode = _int_value(values, "maero", 13)
    return _STAGE_NUMBER_BY_AERO_MODE.get(mode, 1)


####


def _cad_in_geo84(longitude_rad: float, latitude_rad: float, altitude_m: float, time_s: float) -> FloatVector:
    radius = SMAJOR_AXIS_M * (
        1.0 - FLATTENING * (1.0 - math.cos(2.0 * latitude_rad)) / 2.0 + 5.0 * FLATTENING * FLATTENING * (1.0 - math.cos(4.0 * latitude_rad)) / 16.0
    )
    deflection = FLATTENING * math.sin(2.0 * latitude_rad) * (1.0 - FLATTENING / 2.0 - altitude_m / radius)
    distance = radius + altitude_m
    sbid1 = -distance * math.sin(deflection)
    sbid3 = -distance * math.cos(deflection)
    celestial_longitude = GW_CLONG_RAD + WEII3 * time_s + longitude_rad
    sin_lat, cos_lat = math.sin(latitude_rad), math.cos(latitude_rad)
    sin_lon, cos_lon = math.sin(celestial_longitude), math.cos(celestial_longitude)
    return np.asarray(
        (
            -sin_lat * cos_lon * sbid1 - cos_lat * cos_lon * sbid3,
            -sin_lat * sin_lon * sbid1 - cos_lat * sin_lon * sbid3,
            cos_lat * sbid1 - sin_lat * sbid3,
        ),
        dtype=np.float64,
    )


####


def _cad_geo84_in(position_inertial_m: FloatVector, time_s: float) -> tuple[float, float, float]:
    distance = float(np.linalg.norm(position_inertial_m))
    latitude_geocentric = math.asin(float(position_inertial_m[2]) / distance)
    latitude = latitude_geocentric
    altitude = 0.0
    for _ in range(101):
        previous = latitude
        radius = SMAJOR_AXIS_M * (
            1.0 - FLATTENING * (1.0 - math.cos(2.0 * previous)) / 2.0 + 5.0 * FLATTENING * FLATTENING * (1.0 - math.cos(4.0 * previous)) / 16.0
        )
        altitude = distance - radius
        deflection = FLATTENING * math.sin(2.0 * previous) * (1.0 - FLATTENING / 2.0 - altitude / radius)
        latitude = latitude_geocentric + deflection
        if abs(latitude - previous) <= SMALL:
            break
        ####
    else:
        raise RuntimeError("ROCKET6G WGS84 geodetic latitude did not converge")
    ####
    celestial_longitude = math.atan2(float(position_inertial_m[1]), float(position_inertial_m[0]))
    longitude = _wrap_pi(celestial_longitude - WEII3 * time_s - GW_CLONG_RAD)
    return longitude, latitude, altitude


####


def _cad_tdi84(longitude_rad: float, latitude_rad: float, altitude_m: float, time_s: float) -> FloatMatrix:
    del altitude_m
    celestial_longitude = GW_CLONG_RAD + WEII3 * time_s + longitude_rad
    tdi13 = math.cos(latitude_rad)
    tdi33 = -math.sin(latitude_rad)
    tdi22 = math.cos(celestial_longitude)
    tdi21 = -math.sin(celestial_longitude)
    return np.asarray(
        (
            (tdi33 * tdi22, -tdi33 * tdi21, tdi13),
            (tdi21, tdi22, 0.0),
            (-tdi13 * tdi22, tdi13 * tdi21, tdi33),
        ),
        dtype=np.float64,
    )


####


def _cad_tgi84(longitude_rad: float, latitude_rad: float, altitude_m: float, time_s: float) -> FloatMatrix:
    tdi = _cad_tdi84(longitude_rad, latitude_rad, altitude_m, time_s)
    radius = SMAJOR_AXIS_M * (
        1.0 - FLATTENING * (1.0 - math.cos(2.0 * latitude_rad)) / 2.0 + 5.0 * FLATTENING * FLATTENING * (1.0 - math.cos(4.0 * latitude_rad)) / 16.0
    )
    deflection = FLATTENING * math.sin(2.0 * latitude_rad) * (1.0 - FLATTENING / 2.0 - altitude_m / radius)
    tgd = np.asarray(
        (
            (math.cos(deflection), 0.0, -math.sin(deflection)),
            (0.0, 1.0, 0.0),
            (math.sin(deflection), 0.0, math.cos(deflection)),
        ),
        dtype=np.float64,
    )
    return tgd @ tdi


####


def _cad_grav84_geocentric(position_inertial_m: FloatVector, time_s: float) -> FloatVector:
    del time_s
    distance = float(np.linalg.norm(position_inertial_m))
    latitude_geocentric = math.asin(float(position_inertial_m[2]) / distance)
    base = GM / (distance * distance)
    factor = 3.0 * math.sqrt(5.0)
    radius_ratio = (SMAJOR_AXIS_M / distance) ** 2
    north = -base * factor * C20 * radius_ratio * math.sin(latitude_geocentric) * math.cos(latitude_geocentric)
    down = base * (1.0 + factor / 2.0 * C20 * radius_ratio * (3.0 * math.sin(latitude_geocentric) ** 2 - 1.0))
    return np.asarray((north, 0.0, down), dtype=np.float64)


####


def _mat3tr(yaw_rad: float, pitch_rad: float, roll_rad: float) -> FloatMatrix:
    sy, cy = math.sin(yaw_rad), math.cos(yaw_rad)
    sp, cp = math.sin(pitch_rad), math.cos(pitch_rad)
    sr, cr = math.sin(roll_rad), math.cos(roll_rad)
    return np.asarray(
        (
            (cy * cp, sy * cp, -sp),
            (cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr),
            (cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr),
        ),
        dtype=np.float64,
    )


####


def _euler_from_dcm(body_from_geodetic: FloatMatrix) -> tuple[float, float, float]:
    t13 = float(body_from_geodetic[0, 2])
    if abs(t13) < 1.0:
        pitch = math.asin(-t13)
        cosine_pitch = math.cos(pitch)
    else:
        pitch = math.copysign(math.pi / 2.0, -t13)
        cosine_pitch = 1.0e-10
    ####
    cosine_yaw = min(1.0, max(-1.0, float(body_from_geodetic[0, 0]) / cosine_pitch))
    cosine_roll = min(1.0, max(-1.0, float(body_from_geodetic[2, 2]) / cosine_pitch))
    yaw = math.acos(cosine_yaw) * (-1.0 if body_from_geodetic[0, 1] < 0.0 else 1.0)
    roll = math.acos(cosine_roll) * (-1.0 if body_from_geodetic[1, 2] < 0.0 else 1.0)
    return yaw * DEG_PER_RAD, pitch * DEG_PER_RAD, roll * DEG_PER_RAD


####


def _dcm_to_quaternion(matrix: FloatMatrix) -> tuple[float, float, float, float]:
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.asarray(
            (
                0.25 * scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ),
            dtype=np.float64,
        )
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = np.asarray(
                (
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                ),
                dtype=np.float64,
            )
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = np.asarray(
                (
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                ),
                dtype=np.float64,
            )
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = np.asarray(
                (
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                ),
                dtype=np.float64,
            )
        ####
    ####
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[0] < 0.0:
        quaternion *= -1.0
    ####
    return (float(quaternion[0]), float(quaternion[1]), float(quaternion[2]), float(quaternion[3]))


####


def _polar_angles(vector_ned: FloatVector) -> tuple[float, float]:
    north, east, down = (float(value) for value in vector_ned)
    heading = 0.0 if north == 0.0 and east == 0.0 else math.atan2(east, north) * DEG_PER_RAD
    flight_path = math.atan2(-down, math.hypot(north, east)) * DEG_PER_RAD
    return heading, flight_path


####


def _skew(vector: FloatVector) -> FloatMatrix:
    x, y, z = (float(value) for value in vector)
    return np.asarray(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)), dtype=np.float64)


####


def _integrate_array(
    state: FloatVector | FloatMatrix,
    derivative_current: FloatVector | FloatMatrix,
    derivative_previous: FloatVector | FloatMatrix,
    dt_s: float,
) -> FloatVector | FloatMatrix:
    flat = cadac_stored_derivative_step(
        tuple(float(value) for value in state.ravel()),
        tuple(float(value) for value in derivative_current.ravel()),
        tuple(float(value) for value in derivative_previous.ravel()),
        dt_s,
    )
    return np.asarray(flat, dtype=np.float64).reshape(state.shape)


####


def _integrate_scalar(state: float, derivative_current: float, derivative_previous: float, dt_s: float) -> float:
    return cadac_stored_derivative_step((state,), (derivative_current,), (derivative_previous,), dt_s)[0]


####


def _wrap_pi(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


####


def _number(vehicle: CadacVehicleBlock, name: str, default: float | None = None) -> float:
    try:
        value = vehicle.parameter(name)
    except KeyError:
        if default is None:
            raise Rocket6gSourceError(f"ROCKET6G source vehicle is missing parameter {name!r}") from None
        ####
        return float(default)
    ####
    if not isinstance(value, (int, float)):
        raise Rocket6gSourceError(f"ROCKET6G parameter {name!r} must be numeric")
    ####
    return float(value)


####


def _number_from_values(values: dict[str, int | float], name: str, default: float | None = None) -> float:
    if name not in values:
        if default is None:
            raise Rocket6gSourceError(f"ROCKET6G source values are missing parameter {name!r}")
        ####
        return float(default)
    ####
    return float(values[name])


####


def _integer_from_values(values: dict[str, int | float], name: str, default: int | None = None) -> int:
    value = _number_from_values(values, name, None if default is None else float(default))
    if not value.is_integer():
        raise Rocket6gSourceError(f"ROCKET6G parameter {name!r} must be integer-valued")
    ####
    return int(value)


####


def _float_value(values: dict[str, int | float], name: str, default: float | None = None) -> float:
    if name not in values:
        if default is None:
            raise KeyError(name)
        ####
        return float(default)
    ####
    return float(values[name])


####


def _int_value(values: dict[str, int | float], name: str, default: int | None = None) -> int:
    value = _float_value(values, name, None if default is None else float(default))
    if not value.is_integer():
        raise ValueError(f"ROCKET6G runtime parameter {name!r} must remain integer-valued")
    ####
    return int(value)


####


def _tuple2(values: FloatVector) -> tuple[float, float]:
    return (float(values[0]), float(values[1]))


####


def _tuple3(values: FloatVector) -> tuple[float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]))


####


__all__ = [
    "Rocket6gAeroCoefficients",
    "Rocket6gBodyWrench",
    "Rocket6gDirectCommand",
    "Rocket6gInitialState",
    "Rocket6gPhaseEvent",
    "Rocket6gPlantRunResult",
    "Rocket6gPlantSample",
    "Rocket6gPropulsionState",
    "Rocket6gPropulsionStep",
    "Rocket6gSourceDefinition",
    "Rocket6gSourceError",
    "Rocket6gStageConfig",
    "Rocket6gTvcConfig",
    "Rocket6gTvcState",
    "Rocket6gTvcStep",
    "load_rocket6g_source_definition",
    "lower_rocket6g_source_bundle",
    "rocket6g_aerodynamic_coefficients",
    "rocket6g_body_wrench",
    "rocket6g_tvc_step",
    "run_rocket6g_phase_aware_plant",
]
