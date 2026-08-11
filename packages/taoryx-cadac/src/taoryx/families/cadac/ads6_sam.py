"""Source-grounded ADS6 SAM rigid-body plant and control-realization kernels."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal, Protocol, TypeAlias

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
Ads6SamPhase = Literal["fin_control", "tvc_control", "aggregate_rcs"]

_RAD_PER_DEG = math.pi / 180.0
_DEG_PER_RAD = 180.0 / math.pi
_AGRAV = 9.80675445
_SMALL_AERO = 1.0e-12

_ADS6_SAM_REQUIRED_MODULES = (
    "environment",
    "kinematics",
    "propulsion",
    "aerodynamics",
    "forces",
    "euler",
    "newton",
)
_ADS6_SAM_SUPPORTED_MODULES = {
    *_ADS6_SAM_REQUIRED_MODULES,
    "ins",
    "sensor",
    "guidance",
    "control",
    "actuator",
    "tvc",
    "rcs",
    "intercept",
}
_ADS6_SAM_AERO_TABLES = (
    "ca0_vs_mach,betax,alphax",
    "cad_vs_mach",
    "cab_vs_mach",
    "cy0_vs_mach,betax,alphax",
    "cydr_vs_mach,betax,alphax",
    "cn0_vs_mach,betax,alphax",
    "cndq_vs_mach,betax,alphax",
    "cll0_vs_mach,betax,alphax",
    "cllp_vs_mach",
    "clldp_vs_mach,betax,alphax",
    "clm0_vs_mach,betax,alphax",
    "clmq_vs_mach",
    "clmdq_vs_mach,betax,alphax",
    "cln0_vs_mach,betax,alphax",
    "clnr_vs_mach",
    "clndr_vs_mach,betax,alphax",
)
_ADS6_SAM_PROP_TABLES = (
    "thrust_vs_time",
    "mass_vs_time",
    "cg_vs_time",
    "moipitch_vs_time",
    "moiroll_vs_time",
)


class Ads6SamSourceError(ValueError):
    """Source-bundle incompatibility with the ADS6 SAM reconstruction."""


####


class Ads6SamInitialState(CadacModel):
    """Source initial truth state for one ADS6 ``MISSILE6`` SAM actor."""

    position_ned_m: tuple[float, float, float]
    speed_mps: float = Field(gt=0.0)
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    alpha_deg: float = 0.0
    beta_deg: float = 0.0
    body_rates_deg_s: tuple[float, float, float] = (0.0, 0.0, 0.0)


####


class Ads6SamAerodynamicConfig(CadacModel):
    """Source SAM aerodynamic geometry and limiting values."""

    reference_length_m: float = Field(default=0.25, gt=0.0)
    reference_area_m2: float = Field(default=0.0491, gt=0.0)
    reference_cg_m: float
    alpha_limit_deg: float = Field(default=40.0, gt=0.0)
    structural_limit_g: float = Field(default=50.0, gt=0.0)


####


class Ads6SamPropulsionConfig(CadacModel):
    """Source pressure-corrected time-table rocket configuration."""

    nozzle_exit_area_m2: float = Field(default=0.0314, ge=0.0)
    powered_duration_s: float = Field(default=60.0, gt=0.0)


####


class Ads6SamFinActuatorConfig(CadacModel):
    """Cross-configured four-fin physical actuator parameters."""

    mode: int = 2
    position_limit_deg: float = Field(default=28.0, gt=0.0)
    rate_limit_deg_s: float = Field(default=600.0, gt=0.0)
    natural_frequency_rad_s: float = Field(default=600.0, gt=0.0)
    damping_ratio: float = Field(default=0.7, ge=0.0)

    @model_validator(mode="after")
    def validate_mode(self) -> "Ads6SamFinActuatorConfig":
        if self.mode not in {0, 2}:
            raise ValueError("ADS6 SAM fin actuator mode must be 0 or 2")
        ####
        return self

    ####


####


class Ads6SamTvcConfig(CadacModel):
    """Physical pitch/yaw thrust-vector-control actuator parameters."""

    mode: int = 0
    position_limit_deg: float = Field(default=8.0, gt=0.0)
    rate_limit_deg_s: float = Field(default=200.0, gt=0.0)
    natural_frequency_rad_s: float = Field(default=100.0, gt=0.0)
    damping_ratio: float = Field(default=0.7, ge=0.0)
    pressure_for_36_percent_gain_pa: float = Field(default=100_000.0, gt=0.0)
    initial_gain: float = Field(default=0.5, ge=0.0)
    propulsion_arm_from_nose_m: float = Field(default=5.0, gt=0.0)

    @model_validator(mode="after")
    def validate_mode(self) -> "Ads6SamTvcConfig":
        if self.mode not in {0, 1, 2, 3}:
            raise ValueError("ADS6 SAM TVC mode must be 0, 1, 2, or 3")
        ####
        return self

    ####


####


class Ads6SamRcsConfig(CadacModel):
    """Axis-aggregate ADS6 RCS parameters and source command defaults."""

    moment_mode: int = 0
    force_mode: int = 0
    dead_zone: float = Field(default=0.0, ge=0.0)
    hysteresis: float = Field(default=0.0, ge=0.0)
    time_slope_s: float = Field(default=0.0, ge=0.0)
    roll_moment_limit_nm: float = Field(default=0.0, ge=0.0)
    pitch_moment_limit_nm: float = Field(default=0.0, ge=0.0)
    yaw_moment_limit_nm: float = Field(default=0.0, ge=0.0)
    proportional_damping: float = Field(default=0.0, ge=0.0)
    proportional_frequency_rad_s: float = Field(default=0.0, ge=0.0)
    location_from_nose_m: float = Field(default=0.0, ge=0.0)
    specific_impulse_s: float = Field(default=0.0, ge=0.0)
    rate_damping_gain_nm_per_deg_s: float = Field(default=0.0, ge=0.0)
    acceleration_gain_n_per_mps2: float = Field(default=0.0, ge=0.0)
    side_force_limit_n: float = Field(default=0.0, ge=0.0)
    roll_command_deg: float = 0.0
    pitch_command_deg: float = 0.0
    yaw_command_deg: float = 0.0

    @property
    def moment_type(self) -> int:
        return self.moment_mode // 10

    ####

    @property
    def control_mode(self) -> int:
        return self.moment_mode % 10

    ####

    @model_validator(mode="after")
    def validate_modes(self) -> "Ads6SamRcsConfig":
        if self.moment_type not in {0, 1, 2}:
            raise ValueError("ADS6 SAM RCS moment type must be 0, 1, or 2")
        ####
        if self.control_mode not in {0, 1, 2, 3, 4}:
            raise ValueError("ADS6 SAM RCS control mode must be 0 through 4")
        ####
        if self.force_mode not in {0, 1, 2}:
            raise ValueError("ADS6 SAM RCS force mode must be 0, 1, or 2")
        ####
        return self

    ####


####


class Ads6SamSourceDefinition(CadacModel):
    """Prepared standalone ADS6 SAM source case and immutable source resources."""

    source_name: str = Field(min_length=1)
    integration_step_s: float = Field(gt=0.0)
    plot_step_s: float | None = Field(default=None, gt=0.0)
    end_time_s: float = Field(gt=0.0)
    launch_delay_s: float = Field(default=9_999.0, ge=0.0)
    module_order: tuple[str, ...]
    initial_state: Ads6SamInitialState
    aerodynamics: Ads6SamAerodynamicConfig
    propulsion: Ads6SamPropulsionConfig
    fin_actuator: Ads6SamFinActuatorConfig
    tvc: Ads6SamTvcConfig
    rcs: Ads6SamRcsConfig
    aerodynamic_deck: CadacDeck
    propulsion_deck: CadacDeck
    events: tuple[CadacEventBlock, ...] = ()
    source_parameters: tuple[tuple[str, int | float], ...] = ()
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)

    @model_validator(mode="after")
    def validate_tables_and_modules(self) -> "Ads6SamSourceDefinition":
        aero_names = {table.name.casefold() for table in self.aerodynamic_deck.tables}
        prop_names = {table.name.casefold() for table in self.propulsion_deck.tables}
        missing_aero = tuple(name for name in _ADS6_SAM_AERO_TABLES if name.casefold() not in aero_names)
        missing_prop = tuple(name for name in _ADS6_SAM_PROP_TABLES if name.casefold() not in prop_names)
        if missing_aero or missing_prop:
            raise ValueError("ADS6 SAM source definition is missing required tables: " + ", ".join((*missing_aero, *missing_prop)))
        ####
        missing_modules = tuple(name for name in _ADS6_SAM_REQUIRED_MODULES if name not in self.module_order)
        if missing_modules:
            raise ValueError(f"ADS6 SAM source definition is missing required modules: {missing_modules!r}")
        ####
        return self

    ####


####


class Ads6SamControlCommand(CadacModel):
    """Controller-output coordinates at the ADS6 effector command seam."""

    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0

    def vector(self) -> tuple[float, float, float]:
        return (self.roll_deg, self.pitch_deg, self.yaw_deg)

    ####


####


class Ads6SamFinSet(CadacModel):
    """Requested or achieved cross-configured SAM fin positions."""

    fin1_deg: float = 0.0
    fin2_deg: float = 0.0
    fin3_deg: float = 0.0
    fin4_deg: float = 0.0

    def vector(self) -> tuple[float, float, float, float]:
        return (self.fin1_deg, self.fin2_deg, self.fin3_deg, self.fin4_deg)

    ####


####


class Ads6SamFinActuatorState(CadacModel):
    """Stored-derivative state for four physical fins."""

    position_derivative_deg_s: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    position_deg: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    rate_derivative_deg_s2: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    rate_deg_s: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)


####


class Ads6SamFinActuatorStep(CadacModel):
    """One cross-fin actuator transition and saturation evidence."""

    requested_control: Ads6SamControlCommand
    requested_fins: Ads6SamFinSet
    achieved_control: Ads6SamControlCommand
    achieved_fins: Ads6SamFinSet
    state: Ads6SamFinActuatorState
    position_limited: tuple[bool, bool, bool, bool]
    rate_limited: tuple[bool, bool, bool, bool]


####


class Ads6SamTvcState(CadacModel):
    """Stored-derivative physical nozzle state in source pitch/yaw order."""

    position_derivative_rad_s: tuple[float, float] = (0.0, 0.0)
    position_rad: tuple[float, float] = (0.0, 0.0)
    rate_derivative_rad_s2: tuple[float, float] = (0.0, 0.0)
    rate_rad_s: tuple[float, float] = (0.0, 0.0)


####


class Ads6SamTvcStep(CadacModel):
    """One physical TVC transition and achieved thrust wrench."""

    requested_pitch_yaw_deg: tuple[float, float]
    effective_gain: float
    achieved_pitch_yaw_deg: tuple[float, float]
    force_body_n: tuple[float, float, float]
    moment_body_nm: tuple[float, float, float]
    state: Ads6SamTvcState
    position_limited: tuple[bool, bool]
    rate_limited: tuple[bool, bool]


####


class Ads6SamRcsRuntimeInput(CadacModel):
    """Runtime signals consumed by the source aggregate RCS algorithm."""

    dt_s: float = Field(gt=0.0)
    body_rates_rad_s: tuple[float, float, float]
    body_angles_deg: tuple[float, float, float]
    incidence_deg: tuple[float, float]
    incidence_commands_deg: tuple[float, float] = (0.0, 0.0)
    acceleration_commands_g: tuple[float, float] = (0.0, 0.0)
    thrust_vector_unit_body: tuple[float, float, float] = (1.0, 0.0, 0.0)
    specific_force_body_mps2: tuple[float, float, float] = (0.0, 0.0, 0.0)
    inertia_diagonal_kgm2: tuple[float, float, float]
    center_of_gravity_m: float


####


class Ads6SamRcsState(CadacModel):
    """Saved relay, counter, and fuel-time state for aggregate RCS."""

    roll_saved_error: float = 0.0
    pitch_saved_error: float = 0.0
    yaw_saved_error: float = 0.0
    right_saved_error: float = 0.0
    down_saved_error: float = 0.0
    roll_output: int = 0
    pitch_output: int = 0
    yaw_output: int = 0
    right_output: int = 0
    down_output: int = 0
    roll_switch_count: int = 0
    pitch_switch_count: int = 0
    yaw_switch_count: int = 0
    right_switch_count: int = 0
    down_switch_count: int = 0
    total_side_thruster_on_time_s: float = 0.0


####


class Ads6SamRcsStep(CadacModel):
    """One aggregate RCS force/moment transition."""

    force_body_n: tuple[float, float, float]
    moment_body_nm: tuple[float, float, float]
    errors: tuple[float, float, float, float, float]
    fuel_mass_expended_kg: float
    state: Ads6SamRcsState


####


class Ads6SamPropulsionStep(CadacModel):
    """Source time-table propulsion and mass-property output."""

    motor_on: bool
    thrust_n: float
    mass_kg: float = Field(gt=0.0)
    center_of_gravity_m: float
    roll_inertia_kgm2: float = Field(gt=0.0)
    transverse_inertia_kgm2: float = Field(gt=0.0)


####


class Ads6SamAeroCoefficients(CadacModel):
    """ADS6 SAM coefficients, dimensional derivatives, poles, and maneuver evidence."""

    axial: float
    side: float
    normal: float
    roll: float
    pitch: float
    yaw: float
    maximum_load_g: float
    available_load_g: float
    normal_alpha_derivative_mps2: float = 0.0
    normal_control_derivative_mps2: float = 0.0
    pitch_alpha_derivative_rad_s2: float = 0.0
    pitch_rate_derivative_per_s: float = 0.0
    pitch_control_derivative_rad_s2: float = 0.0
    roll_rate_derivative_per_s: float = 0.0
    roll_control_derivative_rad_s2: float = 0.0
    side_beta_derivative_mps2: float = 0.0
    yaw_beta_derivative_rad_s2: float = 0.0
    yaw_rate_derivative_per_s: float = 0.0
    yaw_control_derivative_rad_s2: float = 0.0
    pitch_real_root_1_rad_s: float = 0.0
    pitch_real_root_2_rad_s: float = 0.0
    pitch_natural_frequency_rad_s: float = 0.0
    pitch_damping_ratio: float = 0.0
    yaw_real_root_1_rad_s: float = 0.0
    yaw_real_root_2_rad_s: float = 0.0
    yaw_natural_frequency_rad_s: float = 0.0
    yaw_damping_ratio: float = 0.0


####


class Ads6SamBodyWrench(CadacModel):
    """Total non-gravitational body-axis force and moment."""

    force_body_n: tuple[float, float, float]
    moment_body_nm: tuple[float, float, float]


####


class Ads6SamPlantObservation(CadacModel):
    """Current source-pass plant values exposed to a package controller."""

    time_s: float = Field(ge=0.0)
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    body_rates_rad_s: tuple[float, float, float]
    body_angles_deg: tuple[float, float, float]
    incidence_deg: tuple[float, float]
    altitude_m: float
    gravity_mps2: float = Field(ge=0.0)
    mach: float = Field(ge=0.0)
    dynamic_pressure_pa: float = Field(ge=0.0)
    specific_force_body_mps2: tuple[float, float, float]
    propulsion: Ads6SamPropulsionStep
    coefficients: Ads6SamAeroCoefficients
    wrench: Ads6SamBodyWrench


####


class Ads6SamDirectCommand(CadacModel):
    """Direct controller-output command for one selected ADS6 SAM realization."""

    phase: Ads6SamPhase = "fin_control"
    control: Ads6SamControlCommand = Field(default_factory=Ads6SamControlCommand)
    tvc_mode: int | None = None
    rcs_moment_mode: int | None = None
    rcs_force_mode: int | None = None
    roll_attitude_command_deg: float | None = None
    pitch_attitude_command_deg: float | None = None
    yaw_attitude_command_deg: float | None = None
    incidence_commands_deg: tuple[float, float] = (0.0, 0.0)
    acceleration_commands_g: tuple[float, float] = (0.0, 0.0)
    thrust_vector_unit_body: tuple[float, float, float] = (1.0, 0.0, 0.0)

    @model_validator(mode="after")
    def validate_phase_modes(self) -> "Ads6SamDirectCommand":
        if self.tvc_mode is not None and self.tvc_mode not in {1, 2, 3}:
            raise ValueError("ADS6 SAM TVC override must be mode 1, 2, or 3")
        ####
        if self.rcs_moment_mode is not None:
            moment_type = self.rcs_moment_mode // 10
            control_mode = self.rcs_moment_mode % 10
            if moment_type not in {0, 1, 2} or control_mode not in {0, 1, 2, 3, 4}:
                raise ValueError("ADS6 SAM RCS moment override must encode a supported source type/mode")
            ####
        ####
        if self.rcs_force_mode is not None and self.rcs_force_mode not in {0, 1, 2}:
            raise ValueError("ADS6 SAM RCS force override must be 0, 1, or 2")
        ####
        vector = np.asarray(self.thrust_vector_unit_body, dtype=np.float64)
        magnitude = float(np.linalg.norm(vector))
        if not math.isfinite(magnitude) or magnitude <= 0.0:
            raise ValueError("ADS6 SAM thrust-vector direction must be finite and nonzero")
        ####
        return self

    ####


####


class Ads6SamModuleController(Protocol):
    """Module hook used to interleave a controller with the source plant order."""

    def begin_epoch(
        self,
        missile_time_s: float,
        sim_time_s: float,
        observation: Ads6SamPlantObservation,
        context: object,
    ) -> None:
        """Apply source events before the vehicle module pass."""

        ...

    ####

    def execute_module(
        self,
        module_name: str,
        observation: Ads6SamPlantObservation,
        context: object,
    ) -> Ads6SamDirectCommand | None:
        """Execute one source controller module and optionally emit a command."""

        ...

    ####


####


class Ads6SamSample(CadacModel):
    """One accepted ADS6 SAM sample with control-realization evidence."""

    time_s: float = Field(ge=0.0)
    source_phase: Ads6SamPhase
    fidelity: Literal["rigid_body_6dof_direct_wrench", "rigid_body_6dof_surface_allocated"]
    control_realization: Literal["direct_wrench", "effector_allocated"]
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    body_rates_rad_s: tuple[float, float, float]
    body_angles_deg: tuple[float, float, float]
    incidence_deg: tuple[float, float]
    altitude_m: float
    speed_mps: float
    mach: float
    dynamic_pressure_pa: float
    mass_kg: float
    center_of_gravity_m: float
    inertia_diagonal_kgm2: tuple[float, float, float]
    thrust_n: float
    requested_control_deg: tuple[float, float, float]
    requested_fins_deg: tuple[float, float, float, float]
    achieved_fins_deg: tuple[float, float, float, float]
    achieved_control_deg: tuple[float, float, float]
    requested_tvc_pitch_yaw_deg: tuple[float, float]
    achieved_tvc_pitch_yaw_deg: tuple[float, float]
    tvc_effective_gain: float
    requested_rcs_attitude_deg: tuple[float, float, float]
    requested_rcs_incidence_deg: tuple[float, float]
    requested_rcs_acceleration_g: tuple[float, float]
    requested_thrust_vector_unit_body: tuple[float, float, float]
    achieved_lateral_normal_acceleration_g: tuple[float, float]
    rcs_force_body_n: tuple[float, float, float]
    rcs_moment_body_nm: tuple[float, float, float]
    force_body_n: tuple[float, float, float]
    moment_body_nm: tuple[float, float, float]
    fin_position_limited: tuple[bool, bool, bool, bool]
    fin_rate_limited: tuple[bool, bool, bool, bool]
    tvc_position_limited: tuple[bool, bool]
    tvc_rate_limited: tuple[bool, bool]


####


class Ads6SamRunResult(CadacModel):
    """One standalone ADS6 SAM physical-plant execution."""

    source_name: str
    source_phase: Ads6SamPhase
    integration_step_s: float = Field(gt=0.0)
    requested_end_time_s: float = Field(gt=0.0)
    executed_steps: int = Field(gt=0)
    terminated_reason: Literal["end_time", "ground_impact", "nonfinite_state"]
    source_artifacts: tuple[CadacSourceArtifact, ...]
    samples: tuple[Ads6SamSample, ...] = Field(min_length=1)
    claim_boundary: str = (
        "Source-grounded ADS6 SAM flat-Earth rigid-body plant with physical cross fins, physical pitch/yaw TVC, "
        "or axis-aggregate RCS selected as an explicit realization. INS, seeker, guidance, radar scheduling, "
        "and compiled-CADAC numerical parity remain outside this vehicle-only runtime."
    )


####


@dataclass(slots=True)
class _Ads6SamRuntime:
    position_ned_m: FloatVector
    position_derivative_ned_mps: FloatVector
    velocity_body_mps: FloatVector
    velocity_body_derivative_mps2: FloatVector
    velocity_ned_mps: FloatVector
    quaternion_wxyz: FloatVector
    quaternion_derivative: FloatVector
    body_rates_rad_s: FloatVector
    body_rate_derivative_rad_s2: FloatVector
    fin_state: Ads6SamFinActuatorState
    tvc_state: Ads6SamTvcState
    rcs_state: Ads6SamRcsState
    achieved_fins: Ads6SamFinSet
    achieved_control: Ads6SamControlCommand
    alpha_deg: float
    beta_deg: float
    altitude_m: float
    body_angles_deg: tuple[float, float, float]
    gravity_mps2: float = 0.0
    density_kg_m3: float = 0.0
    pressure_pa: float = 0.0
    temperature_k: float = 0.0
    speed_of_sound_mps: float = 0.0
    mach: float = 0.0
    dynamic_pressure_pa: float = 0.0
    specific_force_body_mps2: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))


####


@dataclass(slots=True)
class Ads6SamPlantStepper:
    """Persistent one-step ADS6 SAM plant used by package-level schedulers."""

    definition: Ads6SamSourceDefinition
    _runtime: _Ads6SamRuntime = field(init=False, repr=False)
    _last_command: Ads6SamDirectCommand = field(default_factory=Ads6SamDirectCommand, init=False, repr=False)
    _last_propulsion: Ads6SamPropulsionStep = field(init=False, repr=False)
    _last_coefficients: Ads6SamAeroCoefficients = field(init=False, repr=False)
    _last_fin_step: Ads6SamFinActuatorStep = field(init=False, repr=False)
    _last_tvc_step: Ads6SamTvcStep = field(init=False, repr=False)
    _last_rcs_step: Ads6SamRcsStep = field(init=False, repr=False)
    _last_wrench: Ads6SamBodyWrench = field(init=False, repr=False)
    executed_steps: int = 0

    def __post_init__(self) -> None:
        self._runtime = _initialize_runtime(self.definition)
        self._last_propulsion = ads6_sam_propulsion_step(
            self.definition,
            missile_time_s=0.0,
            ambient_pressure_pa=101_325.0,
        )
        self._last_coefficients = Ads6SamAeroCoefficients(
            axial=0.0,
            side=0.0,
            normal=0.0,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            maximum_load_g=0.0,
            available_load_g=0.0,
        )
        self._last_fin_step = Ads6SamFinActuatorStep(
            requested_control=self._last_command.control,
            requested_fins=ads6_sam_mix_fin_commands(self._last_command.control),
            achieved_control=self._runtime.achieved_control,
            achieved_fins=self._runtime.achieved_fins,
            state=self._runtime.fin_state,
            position_limited=(False, False, False, False),
            rate_limited=(False, False, False, False),
        )
        self._last_tvc_step = ads6_sam_tvc_step(
            self.definition.tvc.model_copy(update={"mode": 0}),
            self._runtime.tvc_state,
            self._last_command.control,
            dynamic_pressure_pa=0.0,
            thrust_n=0.0,
            center_of_gravity_m=self._last_propulsion.center_of_gravity_m,
            dt_s=self.definition.integration_step_s,
        )
        self._last_rcs_step = Ads6SamRcsStep(
            force_body_n=(0.0, 0.0, 0.0),
            moment_body_nm=(0.0, 0.0, 0.0),
            errors=(0.0, 0.0, 0.0, 0.0, 0.0),
            fuel_mass_expended_kg=0.0,
            state=self._runtime.rcs_state,
        )
        self._last_wrench = Ads6SamBodyWrench(
            force_body_n=(0.0, 0.0, 0.0),
            moment_body_nm=(0.0, 0.0, 0.0),
        )

    ####

    def step(self, missile_time_s: float, command: Ads6SamDirectCommand) -> None:
        """Execute one source-ordered SAM plant pass with a fixed command."""

        self._execute_step(
            missile_time_s=missile_time_s,
            sim_time_s=max(0.0, missile_time_s),
            command=command,
            controller=None,
            context=None,
        )

    ####

    def step_controlled(
        self,
        missile_time_s: float,
        sim_time_s: float,
        controller: Ads6SamModuleController,
        context: object,
    ) -> Ads6SamDirectCommand:
        """Execute one pass with controller modules interleaved in source order."""

        return self._execute_step(
            missile_time_s=missile_time_s,
            sim_time_s=sim_time_s,
            command=self._last_command,
            controller=controller,
            context=context,
        )

    ####

    def _execute_step(
        self,
        *,
        missile_time_s: float,
        sim_time_s: float,
        command: Ads6SamDirectCommand,
        controller: Ads6SamModuleController | None,
        context: object | None,
    ) -> Ads6SamDirectCommand:
        """Run the source module list and update the active command at ``control``."""

        current_command = command
        if controller is not None:
            if context is None:
                raise ValueError("controlled ADS6 SAM execution requires a controller context")
            ####
            controller.begin_epoch(
                missile_time_s,
                sim_time_s,
                self.observation(sim_time_s),
                context,
            )
        ####
        tvc_config, rcs_config = _ads6_sam_resolve_command_configs(self.definition, current_command)
        for module in self.definition.module_order:
            if module == "environment":
                _runtime_environment(self._runtime)
            elif module == "kinematics":
                _runtime_kinematics(self._runtime, self.definition.integration_step_s)
            elif module == "propulsion":
                self._last_propulsion = ads6_sam_propulsion_step(
                    self.definition,
                    missile_time_s=missile_time_s,
                    ambient_pressure_pa=self._runtime.pressure_pa,
                )
            elif module == "aerodynamics":
                self._last_coefficients = ads6_sam_aerodynamic_coefficients(
                    self.definition,
                    mach=self._runtime.mach,
                    dynamic_pressure_pa=self._runtime.dynamic_pressure_pa,
                    speed_mps=max(1.0e-9, float(np.linalg.norm(self._runtime.velocity_ned_mps))),
                    alpha_deg=self._runtime.alpha_deg,
                    beta_deg=self._runtime.beta_deg,
                    body_rates_deg_s=_tuple3(self._runtime.body_rates_rad_s * _DEG_PER_RAD),
                    achieved_control=self._runtime.achieved_control,
                    achieved_fins=self._runtime.achieved_fins,
                    propulsion=self._last_propulsion,
                )
            elif module in {"ins", "sensor", "guidance", "control"}:
                if controller is not None:
                    if context is None:
                        raise AssertionError("validated controlled execution lost its context")
                    ####
                    updated = controller.execute_module(
                        module,
                        self.observation(sim_time_s),
                        context,
                    )
                    if updated is not None:
                        current_command = updated
                        tvc_config, rcs_config = _ads6_sam_resolve_command_configs(
                            self.definition,
                            current_command,
                        )
                    ####
                ####
            elif module == "actuator":
                _validate_selected_realization(
                    current_command.phase,
                    self.definition.module_order,
                    tvc_config,
                    rcs_config,
                )
                fin_command = current_command.control if current_command.phase == "fin_control" else Ads6SamControlCommand()
                self._last_fin_step = ads6_sam_fin_actuator_step(
                    self.definition.fin_actuator,
                    self._runtime.fin_state,
                    fin_command,
                    self.definition.integration_step_s,
                )
                self._runtime.fin_state = self._last_fin_step.state
                self._runtime.achieved_fins = self._last_fin_step.achieved_fins
                self._runtime.achieved_control = self._last_fin_step.achieved_control
            elif module == "tvc":
                active_tvc = tvc_config if current_command.phase == "tvc_control" else tvc_config.model_copy(update={"mode": 0})
                self._last_tvc_step = ads6_sam_tvc_step(
                    active_tvc,
                    self._runtime.tvc_state,
                    current_command.control,
                    dynamic_pressure_pa=self._runtime.dynamic_pressure_pa,
                    thrust_n=self._last_propulsion.thrust_n,
                    center_of_gravity_m=self._last_propulsion.center_of_gravity_m,
                    dt_s=self.definition.integration_step_s,
                )
                self._runtime.tvc_state = self._last_tvc_step.state
            elif module == "rcs":
                active_rcs = rcs_config if current_command.phase == "aggregate_rcs" else rcs_config.model_copy(update={"moment_mode": 0, "force_mode": 0})
                vector = np.asarray(current_command.thrust_vector_unit_body, dtype=np.float64)
                vector /= float(np.linalg.norm(vector))
                self._last_rcs_step = ads6_sam_rcs_step(
                    active_rcs,
                    Ads6SamRcsRuntimeInput(
                        dt_s=self.definition.integration_step_s,
                        body_rates_rad_s=_tuple3(self._runtime.body_rates_rad_s),
                        body_angles_deg=self._runtime.body_angles_deg,
                        incidence_deg=(self._runtime.alpha_deg, self._runtime.beta_deg),
                        incidence_commands_deg=current_command.incidence_commands_deg,
                        acceleration_commands_g=current_command.acceleration_commands_g,
                        thrust_vector_unit_body=_tuple3(vector),
                        specific_force_body_mps2=_tuple3(self._runtime.specific_force_body_mps2),
                        inertia_diagonal_kgm2=(
                            self._last_propulsion.roll_inertia_kgm2,
                            self._last_propulsion.transverse_inertia_kgm2,
                            self._last_propulsion.transverse_inertia_kgm2,
                        ),
                        center_of_gravity_m=self._last_propulsion.center_of_gravity_m,
                    ),
                    self._runtime.rcs_state,
                )
                self._runtime.rcs_state = self._last_rcs_step.state
            elif module == "forces":
                self._last_wrench = ads6_sam_body_wrench(
                    self.definition,
                    self._last_coefficients,
                    self._last_propulsion,
                    dynamic_pressure_pa=self._runtime.dynamic_pressure_pa,
                    tvc=self._last_tvc_step,
                    rcs=self._last_rcs_step,
                    tvc_active=current_command.phase == "tvc_control" and tvc_config.mode > 0,
                )
            elif module == "euler":
                _runtime_euler(
                    self._runtime,
                    self._last_propulsion,
                    self._last_wrench,
                    self.definition.integration_step_s,
                )
            elif module == "newton":
                _runtime_newton(
                    self._runtime,
                    self._last_propulsion,
                    self._last_wrench,
                    self.definition.integration_step_s,
                )
            elif module == "intercept":
                pass
            ####
        ####
        self._last_command = current_command
        self.executed_steps += 1
        return current_command

    ####

    def observation(self, accepted_time_s: float) -> Ads6SamPlantObservation:
        """Return current plant values at a controller module boundary."""

        return Ads6SamPlantObservation(
            time_s=max(0.0, accepted_time_s),
            position_ned_m=_tuple3(self._runtime.position_ned_m),
            velocity_ned_mps=_tuple3(self._runtime.velocity_ned_mps),
            quaternion_wxyz=_tuple4(self._runtime.quaternion_wxyz),
            body_rates_rad_s=_tuple3(self._runtime.body_rates_rad_s),
            body_angles_deg=self._runtime.body_angles_deg,
            incidence_deg=(self._runtime.alpha_deg, self._runtime.beta_deg),
            altitude_m=self._runtime.altitude_m,
            gravity_mps2=max(0.0, self._runtime.gravity_mps2),
            mach=max(0.0, self._runtime.mach),
            dynamic_pressure_pa=max(0.0, self._runtime.dynamic_pressure_pa),
            specific_force_body_mps2=_tuple3(self._runtime.specific_force_body_mps2),
            propulsion=self._last_propulsion,
            coefficients=self._last_coefficients,
            wrench=self._last_wrench,
        )

    ####

    def sample(self, accepted_time_s: float) -> Ads6SamSample:
        """Project the current persistent SAM plant state."""

        return _runtime_sample(
            accepted_time_s,
            self.definition,
            self._last_command,
            self._runtime,
            self._last_propulsion,
            self._last_fin_step,
            self._last_tvc_step,
            self._last_rcs_step,
            self._last_wrench,
        )

    ####

    @property
    def position_ned_m(self) -> tuple[float, float, float]:
        return _tuple3(self._runtime.position_ned_m)

    ####

    @property
    def velocity_ned_mps(self) -> tuple[float, float, float]:
        return _tuple3(self._runtime.velocity_ned_mps)

    ####

    @property
    def altitude_m(self) -> float:
        return self._runtime.altitude_m

    ####

    @property
    def finite(self) -> bool:
        return _runtime_is_finite(self._runtime)

    ####


####


def _ads6_sam_resolve_command_configs(
    definition: Ads6SamSourceDefinition,
    command: Ads6SamDirectCommand,
) -> tuple[Ads6SamTvcConfig, Ads6SamRcsConfig]:
    tvc_config = definition.tvc.model_copy(update={"mode": command.tvc_mode} if command.tvc_mode is not None else {})
    rcs_updates: dict[str, object] = {}
    if command.rcs_moment_mode is not None:
        rcs_updates["moment_mode"] = command.rcs_moment_mode
    ####
    if command.rcs_force_mode is not None:
        rcs_updates["force_mode"] = command.rcs_force_mode
    ####
    for field_name, value in (
        ("roll_command_deg", command.roll_attitude_command_deg),
        ("pitch_command_deg", command.pitch_attitude_command_deg),
        ("yaw_command_deg", command.yaw_attitude_command_deg),
    ):
        if value is not None:
            rcs_updates[field_name] = value
        ####
    ####
    return tvc_config, definition.rcs.model_copy(update=rcs_updates)


####


def lower_ads6_sam_actor(
    bundle: CadacSourceBundle,
    vehicle: CadacVehicleBlock,
) -> Ads6SamSourceDefinition:
    """Lower one source ``MISSILE6`` block from a standalone or package case."""

    if vehicle.model_name.casefold() != "missile6":
        raise Ads6SamSourceError(f"expected MISSILE6 actor, received {vehicle.model_name!r}")
    ####
    module_order = tuple(module.name.casefold() for module in bundle.case.modules if CadacModuleStage.EXECUTE in module.stages)
    unknown = tuple(name for name in module_order if name not in _ADS6_SAM_SUPPORTED_MODULES)
    if unknown:
        raise Ads6SamSourceError(f"ADS6 SAM reconstruction does not implement source modules: {unknown!r}")
    ####
    timing = bundle.case.timing_values
    try:
        integration_step_s = timing["int_step"]
    except KeyError as error:
        raise Ads6SamSourceError("ADS6 SAM source case must declare TIMING int_step") from error
    ####
    return Ads6SamSourceDefinition(
        source_name=bundle.case.source_name,
        integration_step_s=integration_step_s,
        plot_step_s=timing.get("plot_step"),
        end_time_s=bundle.case.end_time_s,
        launch_delay_s=_number(vehicle, "launch_delay", 9_999.0),
        module_order=module_order,
        initial_state=Ads6SamInitialState(
            position_ned_m=(
                _number(vehicle, "sbel1", 0.0),
                _number(vehicle, "sbel2", 0.0),
                _number(vehicle, "sbel3", 0.0),
            ),
            speed_mps=_number(vehicle, "dvbe"),
            yaw_deg=_number(vehicle, "psiblx", 0.0),
            pitch_deg=_number(vehicle, "thtblx", 0.0),
            roll_deg=_number(vehicle, "phiblx", 0.0),
            alpha_deg=_number(vehicle, "alpha0x", 0.0),
            beta_deg=_number(vehicle, "beta0x", 0.0),
            body_rates_deg_s=(
                _number(vehicle, "ppx", 0.0),
                _number(vehicle, "qqx", 0.0),
                _number(vehicle, "rrx", 0.0),
            ),
        ),
        aerodynamics=Ads6SamAerodynamicConfig(
            reference_length_m=_number(vehicle, "refl", 0.25),
            reference_area_m2=_number(vehicle, "refa", 0.0491),
            reference_cg_m=_number(vehicle, "xcgref", 0.0),
            alpha_limit_deg=_number(vehicle, "alplimx", 40.0),
            structural_limit_g=_number(vehicle, "alimitx", 50.0),
        ),
        propulsion=Ads6SamPropulsionConfig(
            nozzle_exit_area_m2=_number(vehicle, "aexit", 0.0314),
            powered_duration_s=_number(vehicle, "powered_duration", 60.0),
        ),
        fin_actuator=Ads6SamFinActuatorConfig(
            mode=_integer(vehicle, "mact", 0),
            position_limit_deg=_number(vehicle, "dlimx", 28.0),
            rate_limit_deg_s=_number(vehicle, "ddlimx", 600.0),
            natural_frequency_rad_s=_number(vehicle, "wnact", 600.0),
            damping_ratio=_number(vehicle, "zetact", 0.7),
        ),
        tvc=Ads6SamTvcConfig(
            mode=_integer(vehicle, "mtvc", 0),
            position_limit_deg=_number(vehicle, "tvclimx", 8.0),
            rate_limit_deg_s=_number(vehicle, "dtvclimx", 200.0),
            natural_frequency_rad_s=_number(vehicle, "wntvc", 100.0),
            damping_ratio=_number(vehicle, "zettvc", 0.7),
            pressure_for_36_percent_gain_pa=_number(vehicle, "pdynmc_gtvc36", 100_000.0),
            initial_gain=_number(vehicle, "gtvc0", 0.5),
            propulsion_arm_from_nose_m=_number(vehicle, "parm", 5.0),
        ),
        rcs=Ads6SamRcsConfig(
            moment_mode=_integer(vehicle, "mrcs_moment", 0),
            force_mode=_integer(vehicle, "mrcs_force", 0),
            dead_zone=_number(vehicle, "dead_zone", 0.0),
            hysteresis=_number(vehicle, "hysteresis", 0.0),
            time_slope_s=_number(vehicle, "rcs_tau", 0.0),
            roll_moment_limit_nm=_number(vehicle, "roll_mom_max", 0.0),
            pitch_moment_limit_nm=_number(vehicle, "pitch_mom_max", 0.0),
            yaw_moment_limit_nm=_number(vehicle, "yaw_mom_max", 0.0),
            proportional_damping=_number(vehicle, "rcs_zeta", 0.0),
            proportional_frequency_rad_s=_number(vehicle, "rcs_freq", 0.0),
            location_from_nose_m=_number(vehicle, "rcs_arm", 0.0),
            specific_impulse_s=_number(vehicle, "rcs_isp", 0.0),
            rate_damping_gain_nm_per_deg_s=_number(vehicle, "rate_gain_rcs", 0.0),
            acceleration_gain_n_per_mps2=_number(vehicle, "acc_gain", 0.0),
            side_force_limit_n=_number(vehicle, "rcs_thrust", 0.0),
            roll_command_deg=_number(vehicle, "phibdcomx", 0.0),
            pitch_command_deg=_number(vehicle, "thtbdcomx", 0.0),
            yaw_command_deg=_number(vehicle, "psibdcomx", 0.0),
        ),
        aerodynamic_deck=bundle.deck_for("MISSILE6", CadacDeckKind.AERODYNAMIC, vehicle_role=vehicle.role),
        propulsion_deck=bundle.deck_for("MISSILE6", CadacDeckKind.PROPULSION, vehicle_role=vehicle.role),
        events=vehicle.events,
        source_parameters=tuple((assignment.name, assignment.value) for assignment in vehicle.assignments if isinstance(assignment.value, (int, float))),
        source_artifacts=bundle.artifacts,
    )


####


def lower_ads6_sam_source_bundle(bundle: CadacSourceBundle) -> Ads6SamSourceDefinition:
    """Lower one standalone ADS6 ``MISSILE6`` case into a typed SAM plant."""

    missiles = bundle.case.vehicles_named("MISSILE6")
    if len(missiles) != 1 or len(bundle.case.vehicles) != 1:
        raise Ads6SamSourceError(
            "ADS6 SAM vehicle plug-in lowering requires exactly one standalone MISSILE6; "
            f"found {len(missiles)} MISSILE6 actors among {len(bundle.case.vehicles)} vehicles"
        )
    ####
    return lower_ads6_sam_actor(bundle, missiles[0])


####


def lower_ads6_sam_selected_actor_bundle(
    bundle: CadacSourceBundle,
    *,
    missile_actor_index: int,
) -> Ads6SamSourceDefinition:
    """Lower one explicitly selected ``MISSILE6`` actor from a package case.

    This is a direct-plant boundary, not an ADS6 engagement replay. The caller
    must name the missile index so package participants are never discarded by
    an implicit first-actor fallback.
    """

    missiles = bundle.case.vehicles_named("MISSILE6")
    if missile_actor_index < 0 or missile_actor_index >= len(missiles):
        raise Ads6SamSourceError(f"ADS6 SAM source case has {len(missiles)} MISSILE6 actors; cannot select index {missile_actor_index}")
    ####
    return lower_ads6_sam_actor(bundle, missiles[missile_actor_index])


####


def load_ads6_sam_source_definition(path: str | Path) -> Ads6SamSourceDefinition:
    """Parse, fingerprint, and lower one standalone ADS6 SAM source case."""

    return lower_ads6_sam_source_bundle(load_cadac_source_bundle(path))


####


def load_ads6_sam_selected_actor_definition(
    path: str | Path,
    *,
    missile_actor_index: int,
) -> Ads6SamSourceDefinition:
    """Parse, fingerprint, and lower an explicitly selected package SAM actor."""

    return lower_ads6_sam_selected_actor_bundle(
        load_cadac_source_bundle(path),
        missile_actor_index=missile_actor_index,
    )


####


def ads6_sam_initial_quaternion(initial: Ads6SamInitialState) -> tuple[float, float, float, float]:
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


def ads6_sam_mix_fin_commands(command: Ads6SamControlCommand) -> Ads6SamFinSet:
    """Apply the ADS6 cross-fin command mixer."""

    roll, pitch, yaw = command.vector()
    return Ads6SamFinSet(
        fin1_deg=-roll - yaw,
        fin2_deg=-roll + pitch,
        fin3_deg=-roll + yaw,
        fin4_deg=-roll - pitch,
    )


####


def ads6_sam_unmix_fin_positions(fins: Ads6SamFinSet) -> Ads6SamControlCommand:
    """Recover achieved roll/pitch/yaw control coordinates from four fins."""

    fin1, fin2, fin3, fin4 = fins.vector()
    return Ads6SamControlCommand(
        roll_deg=0.25 * (-fin1 - fin2 - fin3 - fin4),
        pitch_deg=0.5 * (fin2 - fin4),
        yaw_deg=0.5 * (-fin1 + fin3),
    )


####


def ads6_sam_fin_actuator_step(
    config: Ads6SamFinActuatorConfig,
    state: Ads6SamFinActuatorState,
    command: Ads6SamControlCommand,
    dt_s: float,
) -> Ads6SamFinActuatorStep:
    """Advance all four source physical fins with source-order limiting."""

    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("ADS6 SAM fin actuator dt_s must be positive and finite")
    ####
    requested_fins = ads6_sam_mix_fin_commands(command)
    requested = np.asarray(requested_fins.vector(), dtype=np.float64)
    if config.mode == 0:
        achieved = np.clip(requested, -config.position_limit_deg, config.position_limit_deg)
        fins = _fin_set(achieved)
        return Ads6SamFinActuatorStep(
            requested_control=command,
            requested_fins=requested_fins,
            achieved_control=ads6_sam_unmix_fin_positions(fins),
            achieved_fins=fins,
            state=state,
            position_limited=tuple(bool(abs(value) > config.position_limit_deg) for value in requested),
            rate_limited=(False, False, False, False),
        )
    ####
    position_derivative = np.asarray(state.position_derivative_deg_s, dtype=np.float64).copy()
    position = np.asarray(state.position_deg, dtype=np.float64).copy()
    rate_derivative = np.asarray(state.rate_derivative_deg_s2, dtype=np.float64).copy()
    rate = np.asarray(state.rate_deg_s, dtype=np.float64).copy()
    position_limited: list[bool] = []
    rate_limited: list[bool] = []
    for index in range(4):
        limited_position = abs(position[index]) > config.position_limit_deg
        if limited_position:
            position[index] = math.copysign(config.position_limit_deg, position[index])
            if position[index] * rate[index] > 0.0:
                rate[index] = 0.0
            ####
        ####
        limited_rate = abs(rate[index]) > config.rate_limit_deg_s
        if limited_rate:
            rate[index] = math.copysign(config.rate_limit_deg_s, rate[index])
        ####
        position_derivative_new = rate[index]
        position[index] = cadac_stored_derivative_step(
            (float(position[index]),),
            (float(position_derivative_new),),
            (float(position_derivative[index]),),
            dt_s,
        )[0]
        position_derivative[index] = position_derivative_new
        error = requested[index] - position[index]
        rate_derivative_new = (
            config.natural_frequency_rad_s * config.natural_frequency_rad_s * error
            - 2.0 * config.damping_ratio * config.natural_frequency_rad_s * position_derivative[index]
        )
        rate[index] = cadac_stored_derivative_step(
            (float(rate[index]),),
            (float(rate_derivative_new),),
            (float(rate_derivative[index]),),
            dt_s,
        )[0]
        rate_derivative[index] = rate_derivative_new
        if limited_rate and rate[index] * rate_derivative[index] > 0.0:
            rate_derivative[index] = 0.0
        ####
        position_limited.append(limited_position)
        rate_limited.append(limited_rate)
    ####
    achieved_fins = _fin_set(position)
    next_state = Ads6SamFinActuatorState(
        position_derivative_deg_s=_tuple4(position_derivative),
        position_deg=_tuple4(position),
        rate_derivative_deg_s2=_tuple4(rate_derivative),
        rate_deg_s=_tuple4(rate),
    )
    return Ads6SamFinActuatorStep(
        requested_control=command,
        requested_fins=requested_fins,
        achieved_control=ads6_sam_unmix_fin_positions(achieved_fins),
        achieved_fins=achieved_fins,
        state=next_state,
        position_limited=tuple(position_limited),
        rate_limited=tuple(rate_limited),
    )


####


def ads6_sam_tvc_step(
    config: Ads6SamTvcConfig,
    state: Ads6SamTvcState,
    command: Ads6SamControlCommand,
    *,
    dynamic_pressure_pa: float,
    thrust_n: float,
    center_of_gravity_m: float,
    dt_s: float,
) -> Ads6SamTvcStep:
    """Advance the source physical TVC actuator and derive its achieved wrench."""

    values = (dynamic_pressure_pa, thrust_n, center_of_gravity_m, dt_s)
    if any(not math.isfinite(value) for value in values) or dynamic_pressure_pa < 0.0 or thrust_n < 0.0 or dt_s <= 0.0:
        raise ValueError("ADS6 SAM TVC runtime values must be finite and physically valid")
    ####
    if config.mode == 0 or thrust_n <= 0.0:
        return Ads6SamTvcStep(
            requested_pitch_yaw_deg=(command.pitch_deg, command.yaw_deg),
            effective_gain=0.0,
            achieved_pitch_yaw_deg=(0.0, 0.0),
            force_body_n=(0.0, 0.0, 0.0),
            moment_body_nm=(0.0, 0.0, 0.0),
            state=state,
            position_limited=(False, False),
            rate_limited=(False, False),
        )
    ####
    if config.mode == 2:
        gain = config.initial_gain
    elif config.mode == 3:
        gain = config.initial_gain * math.exp(-dynamic_pressure_pa / config.pressure_for_36_percent_gain_pa)
    else:
        # The literal source leaves ``gtvc`` at zero for mode 1. Preserve that
        # compatibility behavior rather than silently repairing the source.
        gain = 0.0
    ####
    requested_rad = np.asarray((gain * command.pitch_deg * _RAD_PER_DEG, gain * command.yaw_deg * _RAD_PER_DEG))
    position_limited = [False, False]
    rate_limited = [False, False]
    if config.mode == 1:
        achieved_rad = requested_rad
        next_state = state
    else:
        position_derivative = np.asarray(state.position_derivative_rad_s, dtype=np.float64).copy()
        position = np.asarray(state.position_rad, dtype=np.float64).copy()
        rate_derivative = np.asarray(state.rate_derivative_rad_s2, dtype=np.float64).copy()
        rate = np.asarray(state.rate_rad_s, dtype=np.float64).copy()
        position_limit = config.position_limit_deg * _RAD_PER_DEG
        rate_limit = config.rate_limit_deg_s * _RAD_PER_DEG
        for index in range(2):
            position_limited[index] = abs(position[index]) > position_limit
            if position_limited[index]:
                position[index] = math.copysign(position_limit, position[index])
                if position[index] * rate[index] > 0.0:
                    rate[index] = 0.0
                ####
            ####
            rate_limited[index] = abs(rate[index]) > rate_limit
            if rate_limited[index]:
                rate[index] = math.copysign(rate_limit, rate[index])
            ####
            position_derivative_new = rate[index]
            position[index] = cadac_stored_derivative_step(
                (float(position[index]),),
                (float(position_derivative_new),),
                (float(position_derivative[index]),),
                dt_s,
            )[0]
            position_derivative[index] = position_derivative_new
            error = requested_rad[index] - position[index]
            rate_derivative_new = (
                config.natural_frequency_rad_s * config.natural_frequency_rad_s * error
                - 2.0 * config.damping_ratio * config.natural_frequency_rad_s * position_derivative[index]
            )
            rate[index] = cadac_stored_derivative_step(
                (float(rate[index]),),
                (float(rate_derivative_new),),
                (float(rate_derivative[index]),),
                dt_s,
            )[0]
            rate_derivative[index] = rate_derivative_new
            if rate_limited[index] and rate[index] * rate_derivative[index] > 0.0:
                rate_derivative[index] = 0.0
            ####
        ####
        achieved_rad = position
        next_state = Ads6SamTvcState(
            position_derivative_rad_s=_tuple2(position_derivative),
            position_rad=_tuple2(position),
            rate_derivative_rad_s2=_tuple2(rate_derivative),
            rate_rad_s=_tuple2(rate),
        )
    ####
    eta, zeta = achieved_rad
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
    return Ads6SamTvcStep(
        requested_pitch_yaw_deg=(command.pitch_deg, command.yaw_deg),
        effective_gain=gain,
        achieved_pitch_yaw_deg=_tuple2(achieved_rad * _DEG_PER_RAD),
        force_body_n=_tuple3(force),
        moment_body_nm=_tuple3(moment),
        state=next_state,
        position_limited=tuple(position_limited),
        rate_limited=tuple(rate_limited),
    )


####


def ads6_sam_rcs_step(
    config: Ads6SamRcsConfig,
    runtime: Ads6SamRcsRuntimeInput,
    state: Ads6SamRcsState | None = None,
) -> Ads6SamRcsStep:
    """Execute one literal axis-aggregate ADS6 RCS source epoch."""

    previous = state or Ads6SamRcsState()
    force = [0.0, 0.0, 0.0]
    moment = [0.0, 0.0, 0.0]
    p_rate, q_rate, r_rate = runtime.body_rates_rad_s
    roll_deg, pitch_deg, yaw_deg = runtime.body_angles_deg
    alpha_deg, beta_deg = runtime.incidence_deg
    alpha_command_deg, beta_command_deg = runtime.incidence_commands_deg
    lateral_command_g, normal_command_g = runtime.acceleration_commands_g
    vector = runtime.thrust_vector_unit_body
    roll_inertia, pitch_inertia, yaw_inertia = runtime.inertia_diagonal_kgm2

    e_roll = 0.0
    e_pitch = 0.0
    e_yaw = 0.0
    e_right = 0.0
    e_down = 0.0

    roll_output = previous.roll_output
    pitch_output = previous.pitch_output
    yaw_output = previous.yaw_output
    right_output = previous.right_output
    down_output = previous.down_output
    roll_count = previous.roll_switch_count
    pitch_count = previous.pitch_switch_count
    yaw_count = previous.yaw_switch_count
    right_count = previous.right_switch_count
    down_count = previous.down_switch_count
    roll_saved = previous.roll_saved_error
    pitch_saved = previous.pitch_saved_error
    yaw_saved = previous.yaw_saved_error
    right_saved = previous.right_saved_error
    down_saved = previous.down_saved_error
    on_time = previous.total_side_thruster_on_time_s

    if config.moment_type == 1:
        roll_gain = 2.0 * config.proportional_damping * config.proportional_frequency_rad_s * roll_inertia
        pitch_gain = 2.0 * config.proportional_damping * config.proportional_frequency_rad_s * pitch_inertia
        yaw_gain = 2.0 * config.proportional_damping * config.proportional_frequency_rad_s * yaw_inertia
        position_gain = config.proportional_frequency_rad_s / (2.0 * config.proportional_damping) if config.proportional_damping else 0.0
        e_roll = roll_gain * (position_gain * (config.roll_command_deg - roll_deg) - p_rate)
        moment[0] = ads6_sam_rcs_proportional(e_roll, config.roll_moment_limit_nm)
        if config.control_mode == 1:
            e_pitch = pitch_gain * (position_gain * (config.pitch_command_deg - pitch_deg) - q_rate)
            e_yaw = yaw_gain * (position_gain * (config.yaw_command_deg - yaw_deg) - r_rate)
        elif config.control_mode == 2:
            e_pitch = pitch_gain * (position_gain * (-vector[2]) * _DEG_PER_RAD - q_rate)
            e_yaw = yaw_gain * (position_gain * vector[1] * _DEG_PER_RAD - r_rate)
        elif config.control_mode == 4:
            e_pitch = config.rate_damping_gain_nm_per_deg_s * (-q_rate)
            e_yaw = config.rate_damping_gain_nm_per_deg_s * (-r_rate)
        ####
        moment[1] = ads6_sam_rcs_proportional(e_pitch, config.pitch_moment_limit_nm)
        moment[2] = ads6_sam_rcs_proportional(e_yaw, config.yaw_moment_limit_nm)
    ####

    if config.force_mode == 1:
        e_right = config.acceleration_gain_n_per_mps2 * (lateral_command_g * _AGRAV - runtime.specific_force_body_mps2[1])
        e_down = -config.acceleration_gain_n_per_mps2 * (normal_command_g * _AGRAV + runtime.specific_force_body_mps2[2])
        force[1] = ads6_sam_rcs_proportional(e_right, config.side_force_limit_n)
        force[2] = ads6_sam_rcs_proportional(e_down, config.side_force_limit_n)
    ####

    if config.moment_type == 2:
        e_roll = config.roll_command_deg - (config.time_slope_s * p_rate + roll_deg)
        next_output = ads6_sam_rcs_schmitt(e_roll, roll_saved, config.dead_zone, config.hysteresis)
        roll_count += int(next_output != roll_output)
        roll_output = next_output
        roll_saved = e_roll
        if config.control_mode == 1:
            e_pitch = config.pitch_command_deg - (config.time_slope_s * q_rate + pitch_deg)
            e_yaw = config.yaw_command_deg - (config.time_slope_s * r_rate + yaw_deg)
        ####
        if config.control_mode == 3:
            e_pitch = alpha_command_deg - (config.time_slope_s * q_rate + alpha_deg)
            e_yaw = -beta_command_deg - (config.time_slope_s * r_rate - beta_deg)
        elif config.control_mode == 2:
            e_pitch = -config.time_slope_s * q_rate - vector[2] * _DEG_PER_RAD
            e_yaw = -config.time_slope_s * r_rate + vector[1] * _DEG_PER_RAD
        elif config.control_mode == 4:
            e_pitch = -config.time_slope_s * q_rate
            e_yaw = -config.time_slope_s * r_rate
        ####
        next_output = ads6_sam_rcs_schmitt(e_pitch, pitch_saved, config.dead_zone, config.hysteresis)
        pitch_count += int(next_output != pitch_output)
        pitch_output = next_output
        pitch_saved = e_pitch
        next_output = ads6_sam_rcs_schmitt(e_yaw, yaw_saved, config.dead_zone, config.hysteresis)
        yaw_count += int(next_output != yaw_output)
        yaw_output = next_output
        yaw_saved = e_yaw
        moment = [
            roll_output * config.roll_moment_limit_nm,
            pitch_output * config.pitch_moment_limit_nm,
            yaw_output * config.yaw_moment_limit_nm,
        ]
    ####

    if config.force_mode == 2:
        e_right = config.acceleration_gain_n_per_mps2 * (lateral_command_g * _AGRAV - runtime.specific_force_body_mps2[1])
        next_output = ads6_sam_rcs_schmitt(e_right, right_saved, config.dead_zone, config.hysteresis)
        right_count += int(next_output != right_output)
        right_output = next_output
        right_saved = e_right
        e_down = -config.acceleration_gain_n_per_mps2 * (normal_command_g * _AGRAV + runtime.specific_force_body_mps2[2])
        next_output = ads6_sam_rcs_schmitt(e_down, down_saved, config.dead_zone, config.hysteresis)
        down_count += int(next_output != down_output)
        down_output = next_output
        down_saved = e_down
        force = [0.0, right_output * config.side_force_limit_n, down_output * config.side_force_limit_n]
        arm = config.location_from_nose_m - runtime.center_of_gravity_m
        # Literal source behavior overwrites pitch/yaw moment channels with
        # the side-thruster parasitic moments when this mode is active.
        moment[1] = force[2] * arm
        moment[2] = -force[1] * arm
        if down_output:
            on_time += runtime.dt_s
        ####
        if right_output:
            on_time += runtime.dt_s
        ####
    ####
    fuel_mass = config.side_force_limit_n * on_time / (config.specific_impulse_s * _AGRAV) if config.specific_impulse_s > 0.0 else 0.0
    next_state = Ads6SamRcsState(
        roll_saved_error=roll_saved,
        pitch_saved_error=pitch_saved,
        yaw_saved_error=yaw_saved,
        right_saved_error=right_saved,
        down_saved_error=down_saved,
        roll_output=roll_output,
        pitch_output=pitch_output,
        yaw_output=yaw_output,
        right_output=right_output,
        down_output=down_output,
        roll_switch_count=roll_count,
        pitch_switch_count=pitch_count,
        yaw_switch_count=yaw_count,
        right_switch_count=right_count,
        down_switch_count=down_count,
        total_side_thruster_on_time_s=on_time,
    )
    return Ads6SamRcsStep(
        force_body_n=tuple(force),
        moment_body_nm=tuple(moment),
        errors=(e_roll, e_pitch, e_yaw, e_right, e_down),
        fuel_mass_expended_kg=fuel_mass,
        state=next_state,
    )


####


def ads6_sam_rcs_proportional(value: float, limit: float) -> float:
    """Apply the source symmetric aggregate-RCS limiter."""

    if not math.isfinite(value) or not math.isfinite(limit) or limit < 0.0:
        raise ValueError("ADS6 SAM RCS proportional values must be finite with a nonnegative limit")
    ####
    return math.copysign(limit, value) if abs(value) > limit else value


####


def ads6_sam_rcs_schmitt(input_new: float, previous_input: float, dead_zone: float, hysteresis: float) -> int:
    """Replicate the source relay's previous-input trend and side semantics."""

    values = (input_new, previous_input, dead_zone, hysteresis)
    if any(not math.isfinite(value) for value in values) or dead_zone < 0.0 or hysteresis < 0.0:
        raise ValueError("ADS6 SAM RCS Schmitt values must be finite and nonnegative where required")
    ####
    trend = -1 if input_new - previous_input < 0.0 else 1
    side = -1 if previous_input < 0.0 else 1
    trigger = (dead_zone * side + hysteresis * trend) / 2.0
    if previous_input >= trigger and side == 1:
        return 1
    ####
    if previous_input <= trigger and side == -1:
        return -1
    ####
    return 0


####


def ads6_sam_propulsion_step(
    definition: Ads6SamSourceDefinition,
    *,
    missile_time_s: float,
    ambient_pressure_pa: float,
) -> Ads6SamPropulsionStep:
    """Evaluate source thrust and mass-property tables for one missile epoch."""

    if any(not math.isfinite(value) for value in (missile_time_s, ambient_pressure_pa)):
        raise ValueError("ADS6 SAM propulsion inputs must be finite")
    ####
    deck = definition.propulsion_deck
    thrust_sea_level = deck.table("thrust_vs_time").interpolate((missile_time_s,))
    mass = deck.table("mass_vs_time").interpolate((missile_time_s,))
    xcg = deck.table("cg_vs_time").interpolate((missile_time_s,))
    transverse = deck.table("moipitch_vs_time").interpolate((missile_time_s,))
    roll = deck.table("moiroll_vs_time").interpolate((missile_time_s,))
    motor_on = missile_time_s <= definition.propulsion.powered_duration_s
    thrust = thrust_sea_level + (101_325.0 - ambient_pressure_pa) * definition.propulsion.nozzle_exit_area_m2
    if not motor_on:
        # The source continues to evaluate its table but declares mprop=0;
        # properly prepared source decks are zero-thrust after burnout.
        thrust = max(0.0, thrust)
    ####
    return Ads6SamPropulsionStep(
        motor_on=motor_on,
        thrust_n=max(0.0, thrust),
        mass_kg=mass,
        center_of_gravity_m=xcg,
        roll_inertia_kgm2=roll,
        transverse_inertia_kgm2=transverse,
    )


####


def ads6_sam_aerodynamic_coefficients(
    definition: Ads6SamSourceDefinition,
    *,
    mach: float,
    dynamic_pressure_pa: float,
    speed_mps: float,
    alpha_deg: float,
    beta_deg: float,
    body_rates_deg_s: tuple[float, float, float],
    achieved_control: Ads6SamControlCommand,
    achieved_fins: Ads6SamFinSet,
    propulsion: Ads6SamPropulsionStep,
) -> Ads6SamAeroCoefficients:
    """Evaluate the source 3-D coefficient family using achieved fin positions."""

    values = (mach, dynamic_pressure_pa, speed_mps, alpha_deg, beta_deg, *body_rates_deg_s)
    if any(not math.isfinite(value) for value in values) or mach < 0.0 or dynamic_pressure_pa < 0.0 or speed_mps <= 0.0:
        raise ValueError("ADS6 SAM aerodynamic state must be finite and physically valid")
    ####
    deck = definition.aerodynamic_deck
    query = (mach, beta_deg, alpha_deg)
    roll_command, pitch_command, yaw_command = achieved_control.vector()
    effective_fin = sum(abs(value) for value in achieved_fins.vector()) / 4.0
    ca = deck.table("ca0_vs_mach,betax,alphax").interpolate(query)
    ca += deck.table("cad_vs_mach").interpolate((mach,)) * effective_fin
    if not propulsion.motor_on:
        ca += deck.table("cab_vs_mach").interpolate((mach,))
    ####
    cy0 = deck.table("cy0_vs_mach,betax,alphax").interpolate(query)
    cy = cy0 + deck.table("cydr_vs_mach,betax,alphax").interpolate(query) * yaw_command
    cn0 = deck.table("cn0_vs_mach,betax,alphax").interpolate(query)
    cn = cn0 + deck.table("cndq_vs_mach,betax,alphax").interpolate(query) * pitch_command
    p_deg_s, q_deg_s, r_deg_s = body_rates_deg_s
    geometry = definition.aerodynamics
    roll = deck.table("cll0_vs_mach,betax,alphax").interpolate(query)
    roll += deck.table("cllp_vs_mach").interpolate((mach,)) * p_deg_s * _RAD_PER_DEG * geometry.reference_length_m / (2.0 * speed_mps)
    roll += deck.table("clldp_vs_mach,betax,alphax").interpolate(query) * roll_command
    pitch = deck.table("clm0_vs_mach,betax,alphax").interpolate(query)
    pitch += deck.table("clmq_vs_mach").interpolate((mach,)) * q_deg_s * _RAD_PER_DEG * geometry.reference_length_m / (2.0 * speed_mps)
    pitch += deck.table("clmdq_vs_mach,betax,alphax").interpolate(query) * pitch_command
    pitch -= cn / geometry.reference_length_m * (geometry.reference_cg_m - propulsion.center_of_gravity_m)
    yaw = deck.table("cln0_vs_mach,betax,alphax").interpolate(query)
    yaw += deck.table("clnr_vs_mach").interpolate((mach,)) * r_deg_s * _RAD_PER_DEG * geometry.reference_length_m / (2.0 * speed_mps)
    yaw += deck.table("clndr_vs_mach,betax,alphax").interpolate(query) * yaw_command
    yaw -= cy / geometry.reference_length_m * (geometry.reference_cg_m - propulsion.center_of_gravity_m)
    cn_limit = deck.table("cn0_vs_mach,betax,alphax").interpolate((mach, 0.0, geometry.alpha_limit_deg))
    weight = propulsion.mass_kg * _AGRAV
    maximum_load = cn_limit * dynamic_pressure_pa * geometry.reference_area_m2 / weight
    maximum_load = min(maximum_load, geometry.structural_limit_g)
    current_load = math.hypot(cn0, cy0) * dynamic_pressure_pa * geometry.reference_area_m2 / weight
    available = min(geometry.structural_limit_g, max(0.0, maximum_load - current_load))

    alpha_plus_deg = abs(alpha_deg) + 3.0
    if alpha_plus_deg < 3.0:
        alpha_plus_deg = 3.0
    ####
    alpha_minus_deg = max(0.0, abs(alpha_deg) - 3.0)
    alpha_span_rad = max(_SMALL_AERO, (alpha_plus_deg - alpha_minus_deg) * _RAD_PER_DEG)
    cn_alpha_plus = deck.table("cn0_vs_mach,betax,alphax").interpolate((mach, beta_deg, alpha_plus_deg))
    cn_alpha_minus = deck.table("cn0_vs_mach,betax,alphax").interpolate((mach, beta_deg, alpha_minus_deg))
    cna = (cn_alpha_plus - cn_alpha_minus) / alpha_span_rad
    cm_alpha_plus = deck.table("clm0_vs_mach,betax,alphax").interpolate((mach, beta_deg, alpha_plus_deg))
    cm_alpha_minus = deck.table("clm0_vs_mach,betax,alphax").interpolate((mach, beta_deg, alpha_minus_deg))
    cma = (cm_alpha_plus - cm_alpha_minus) / alpha_span_rad
    cma -= cna / geometry.reference_length_m * (geometry.reference_cg_m - propulsion.center_of_gravity_m)

    beta_plus_deg = abs(beta_deg) + 3.0
    if beta_plus_deg < 3.0:
        beta_plus_deg = 3.0
    ####
    beta_minus_deg = max(0.0, abs(beta_deg) - 3.0)
    beta_span_rad = max(_SMALL_AERO, (beta_plus_deg - beta_minus_deg) * _RAD_PER_DEG)
    cy_beta_plus = deck.table("cy0_vs_mach,betax,alphax").interpolate((mach, beta_plus_deg, alpha_deg))
    cy_beta_minus = deck.table("cy0_vs_mach,betax,alphax").interpolate((mach, beta_minus_deg, alpha_deg))
    cyb = (cy_beta_plus - cy_beta_minus) / beta_span_rad
    cn_beta_plus = deck.table("cln0_vs_mach,betax,alphax").interpolate((mach, beta_plus_deg, alpha_deg))
    cn_beta_minus = deck.table("cln0_vs_mach,betax,alphax").interpolate((mach, beta_minus_deg, alpha_deg))
    cnb = (cn_beta_plus - cn_beta_minus) / beta_span_rad
    cnb -= cyb / geometry.reference_length_m * (geometry.reference_cg_m - propulsion.center_of_gravity_m)

    cndq = deck.table("cndq_vs_mach,betax,alphax").interpolate(query)
    clmdq = deck.table("clmdq_vs_mach,betax,alphax").interpolate(query)
    clmq = deck.table("clmq_vs_mach").interpolate((mach,))
    cllp = deck.table("cllp_vs_mach").interpolate((mach,))
    clldp = deck.table("clldp_vs_mach,betax,alphax").interpolate(query)
    clnr = deck.table("clnr_vs_mach").interpolate((mach,))
    clndr = deck.table("clndr_vs_mach,betax,alphax").interpolate(query)

    force_scale = dynamic_pressure_pa * geometry.reference_area_m2 / propulsion.mass_kg
    moment_scale = dynamic_pressure_pa * geometry.reference_area_m2 * geometry.reference_length_m / propulsion.transverse_inertia_kgm2
    roll_scale = dynamic_pressure_pa * geometry.reference_area_m2 * geometry.reference_length_m / propulsion.roll_inertia_kgm2
    dna = force_scale * cna
    dnd = force_scale * (_DEG_PER_RAD * cndq)
    dma = moment_scale * cma
    dmq = moment_scale * (geometry.reference_length_m / (2.0 * speed_mps)) * (_DEG_PER_RAD * clmq)
    dmd = moment_scale * (_DEG_PER_RAD * clmdq)
    dlp = roll_scale * (geometry.reference_length_m / (2.0 * speed_mps)) * (_DEG_PER_RAD * cllp)
    dld = roll_scale * (_DEG_PER_RAD * clldp)
    dyb = force_scale * cyb
    dnb = moment_scale * cnb
    dnr = moment_scale * (geometry.reference_length_m / (2.0 * speed_mps)) * (_DEG_PER_RAD * clnr)
    dnd_yaw = moment_scale * (_DEG_PER_RAD * clndr)

    pitch_roots = _ads6_sam_airframe_roots(
        a11=dmq,
        a12=dma / dna if abs(dna) >= _SMALL_AERO else 0.0,
        a21=dna,
        a22=-dna / speed_mps,
        active=abs(dna) >= _SMALL_AERO,
    )
    yaw_roots = _ads6_sam_airframe_roots(
        a11=dnr,
        a12=dnb / dyb if abs(dyb) >= _SMALL_AERO else 0.0,
        a21=-dyb,
        a22=dyb / speed_mps,
        active=abs(dyb) >= _SMALL_AERO,
    )
    return Ads6SamAeroCoefficients(
        axial=ca,
        side=cy,
        normal=cn,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        maximum_load_g=maximum_load,
        available_load_g=available,
        normal_alpha_derivative_mps2=dna,
        normal_control_derivative_mps2=dnd,
        pitch_alpha_derivative_rad_s2=dma,
        pitch_rate_derivative_per_s=dmq,
        pitch_control_derivative_rad_s2=dmd,
        roll_rate_derivative_per_s=dlp,
        roll_control_derivative_rad_s2=dld,
        side_beta_derivative_mps2=dyb,
        yaw_beta_derivative_rad_s2=dnb,
        yaw_rate_derivative_per_s=dnr,
        yaw_control_derivative_rad_s2=dnd_yaw,
        pitch_real_root_1_rad_s=pitch_roots[0],
        pitch_real_root_2_rad_s=pitch_roots[1],
        pitch_natural_frequency_rad_s=pitch_roots[2],
        pitch_damping_ratio=pitch_roots[3],
        yaw_real_root_1_rad_s=yaw_roots[0],
        yaw_real_root_2_rad_s=yaw_roots[1],
        yaw_natural_frequency_rad_s=yaw_roots[2],
        yaw_damping_ratio=yaw_roots[3],
    )


####


def _ads6_sam_airframe_roots(
    *,
    a11: float,
    a12: float,
    a21: float,
    a22: float,
    active: bool,
) -> tuple[float, float, float, float]:
    """Return source-style real roots or complex-pair frequency and damping."""

    if not active:
        return (0.0, 0.0, 0.0, 0.0)
    ####
    trace = a11 + a22
    determinant = a11 * a22 - a12 * a21
    discriminant = trace * trace - 4.0 * determinant
    if discriminant >= 0.0:
        root = math.sqrt(discriminant)
        return ((trace + root) / 2.0, (trace - root) / 2.0, 0.0, 0.0)
    ####
    natural_frequency = math.sqrt(max(0.0, determinant))
    damping_ratio = -trace / (2.0 * natural_frequency) if natural_frequency > _SMALL_AERO else 0.0
    return (0.0, 0.0, natural_frequency, damping_ratio)


####


def ads6_sam_body_wrench(
    definition: Ads6SamSourceDefinition,
    coefficients: Ads6SamAeroCoefficients,
    propulsion: Ads6SamPropulsionStep,
    *,
    dynamic_pressure_pa: float,
    tvc: Ads6SamTvcStep,
    rcs: Ads6SamRcsStep,
    tvc_active: bool,
) -> Ads6SamBodyWrench:
    """Close aerodynamic, thrust, TVC, and aggregate-RCS loads in source order."""

    geometry = definition.aerodynamics
    force = np.asarray(
        (
            -dynamic_pressure_pa * geometry.reference_area_m2 * coefficients.axial,
            dynamic_pressure_pa * geometry.reference_area_m2 * coefficients.side,
            -dynamic_pressure_pa * geometry.reference_area_m2 * coefficients.normal,
        ),
        dtype=np.float64,
    )
    if tvc_active:
        force += np.asarray(tvc.force_body_n, dtype=np.float64)
    else:
        force[0] += propulsion.thrust_n
    ####
    force += np.asarray(rcs.force_body_n, dtype=np.float64)
    moment = np.asarray(
        (
            dynamic_pressure_pa * geometry.reference_area_m2 * geometry.reference_length_m * coefficients.roll,
            dynamic_pressure_pa * geometry.reference_area_m2 * geometry.reference_length_m * coefficients.pitch,
            dynamic_pressure_pa * geometry.reference_area_m2 * geometry.reference_length_m * coefficients.yaw,
        ),
        dtype=np.float64,
    )
    if tvc_active:
        moment += np.asarray(tvc.moment_body_nm, dtype=np.float64)
    ####
    moment += np.asarray(rcs.moment_body_nm, dtype=np.float64)
    return Ads6SamBodyWrench(force_body_n=_tuple3(force), moment_body_nm=_tuple3(moment))


####


def run_ads6_sam_physical_plant(
    definition: Ads6SamSourceDefinition,
    command: Ads6SamDirectCommand | None = None,
    *,
    end_time_s: float | None = None,
    sample_step_s: float | None = None,
) -> Ads6SamRunResult:
    """Run one exact ADS6 SAM plant realization with direct effector-boundary commands."""

    resolved_command = command or Ads6SamDirectCommand()
    dt_s = definition.integration_step_s
    requested_end = definition.end_time_s if end_time_s is None else float(end_time_s)
    cadence = max(dt_s, 0.02) if sample_step_s is None else float(sample_step_s)
    if not math.isfinite(requested_end) or requested_end <= 0.0:
        raise ValueError("ADS6 SAM end_time_s must be positive and finite")
    ####
    if not math.isfinite(cadence) or cadence <= 0.0:
        raise ValueError("ADS6 SAM sample_step_s must be positive and finite")
    ####
    tvc_config = definition.tvc.model_copy(update={"mode": resolved_command.tvc_mode} if resolved_command.tvc_mode is not None else {})
    rcs_updates: dict[str, object] = {}
    if resolved_command.rcs_moment_mode is not None:
        rcs_updates["moment_mode"] = resolved_command.rcs_moment_mode
    ####
    if resolved_command.rcs_force_mode is not None:
        rcs_updates["force_mode"] = resolved_command.rcs_force_mode
    ####
    for field_name, value in (
        ("roll_command_deg", resolved_command.roll_attitude_command_deg),
        ("pitch_command_deg", resolved_command.pitch_attitude_command_deg),
        ("yaw_command_deg", resolved_command.yaw_attitude_command_deg),
    ):
        if value is not None:
            rcs_updates[field_name] = value
        ####
    ####
    rcs_config = definition.rcs.model_copy(update=rcs_updates)
    _validate_selected_realization(
        resolved_command.phase,
        definition.module_order,
        tvc_config,
        rcs_config,
    )
    runtime = _initialize_runtime(definition)
    samples: list[Ads6SamSample] = []
    sim_time = 0.0
    next_sample_time = 0.0
    steps = 0
    last_propulsion = ads6_sam_propulsion_step(definition, missile_time_s=0.0, ambient_pressure_pa=101_325.0)
    last_coefficients = Ads6SamAeroCoefficients(
        axial=0.0,
        side=0.0,
        normal=0.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        maximum_load_g=0.0,
        available_load_g=0.0,
    )
    last_fin_step = Ads6SamFinActuatorStep(
        requested_control=resolved_command.control,
        requested_fins=ads6_sam_mix_fin_commands(resolved_command.control),
        achieved_control=runtime.achieved_control,
        achieved_fins=runtime.achieved_fins,
        state=runtime.fin_state,
        position_limited=(False, False, False, False),
        rate_limited=(False, False, False, False),
    )
    last_tvc_step = ads6_sam_tvc_step(
        tvc_config.model_copy(update={"mode": 0}),
        runtime.tvc_state,
        resolved_command.control,
        dynamic_pressure_pa=0.0,
        thrust_n=0.0,
        center_of_gravity_m=last_propulsion.center_of_gravity_m,
        dt_s=dt_s,
    )
    last_rcs_step = Ads6SamRcsStep(
        force_body_n=(0.0, 0.0, 0.0),
        moment_body_nm=(0.0, 0.0, 0.0),
        errors=(0.0, 0.0, 0.0, 0.0, 0.0),
        fuel_mass_expended_kg=0.0,
        state=runtime.rcs_state,
    )
    last_wrench = Ads6SamBodyWrench(force_body_n=(0.0, 0.0, 0.0), moment_body_nm=(0.0, 0.0, 0.0))
    while sim_time <= requested_end + 0.5 * dt_s:
        steps += 1
        for module in definition.module_order:
            if module == "environment":
                _runtime_environment(runtime)
            elif module == "kinematics":
                _runtime_kinematics(runtime, dt_s)
            elif module == "propulsion":
                last_propulsion = ads6_sam_propulsion_step(
                    definition,
                    missile_time_s=sim_time,
                    ambient_pressure_pa=runtime.pressure_pa,
                )
            elif module == "aerodynamics":
                last_coefficients = ads6_sam_aerodynamic_coefficients(
                    definition,
                    mach=runtime.mach,
                    dynamic_pressure_pa=runtime.dynamic_pressure_pa,
                    speed_mps=max(1.0e-9, float(np.linalg.norm(runtime.velocity_ned_mps))),
                    alpha_deg=runtime.alpha_deg,
                    beta_deg=runtime.beta_deg,
                    body_rates_deg_s=_tuple3(runtime.body_rates_rad_s * _DEG_PER_RAD),
                    achieved_control=runtime.achieved_control,
                    achieved_fins=runtime.achieved_fins,
                    propulsion=last_propulsion,
                )
            elif module in {"ins", "sensor", "guidance", "control"}:
                # Direct controller-output command seam. These source command
                # generators are intentionally outside this vehicle-only run.
                pass
            elif module == "actuator":
                fin_command = resolved_command.control if resolved_command.phase == "fin_control" else Ads6SamControlCommand()
                last_fin_step = ads6_sam_fin_actuator_step(
                    definition.fin_actuator,
                    runtime.fin_state,
                    fin_command,
                    dt_s,
                )
                runtime.fin_state = last_fin_step.state
                runtime.achieved_fins = last_fin_step.achieved_fins
                runtime.achieved_control = last_fin_step.achieved_control
            elif module == "tvc":
                active_config = tvc_config if resolved_command.phase == "tvc_control" else tvc_config.model_copy(update={"mode": 0})
                last_tvc_step = ads6_sam_tvc_step(
                    active_config,
                    runtime.tvc_state,
                    resolved_command.control,
                    dynamic_pressure_pa=runtime.dynamic_pressure_pa,
                    thrust_n=last_propulsion.thrust_n,
                    center_of_gravity_m=last_propulsion.center_of_gravity_m,
                    dt_s=dt_s,
                )
                runtime.tvc_state = last_tvc_step.state
            elif module == "rcs":
                active_rcs = rcs_config if resolved_command.phase == "aggregate_rcs" else rcs_config.model_copy(update={"moment_mode": 0, "force_mode": 0})
                vector = np.asarray(resolved_command.thrust_vector_unit_body, dtype=np.float64)
                vector /= float(np.linalg.norm(vector))
                last_rcs_step = ads6_sam_rcs_step(
                    active_rcs,
                    Ads6SamRcsRuntimeInput(
                        dt_s=dt_s,
                        body_rates_rad_s=_tuple3(runtime.body_rates_rad_s),
                        body_angles_deg=runtime.body_angles_deg,
                        incidence_deg=(runtime.alpha_deg, runtime.beta_deg),
                        incidence_commands_deg=resolved_command.incidence_commands_deg,
                        acceleration_commands_g=resolved_command.acceleration_commands_g,
                        thrust_vector_unit_body=_tuple3(vector),
                        specific_force_body_mps2=_tuple3(runtime.specific_force_body_mps2),
                        inertia_diagonal_kgm2=(
                            last_propulsion.roll_inertia_kgm2,
                            last_propulsion.transverse_inertia_kgm2,
                            last_propulsion.transverse_inertia_kgm2,
                        ),
                        center_of_gravity_m=last_propulsion.center_of_gravity_m,
                    ),
                    runtime.rcs_state,
                )
                runtime.rcs_state = last_rcs_step.state
            elif module == "forces":
                last_wrench = ads6_sam_body_wrench(
                    definition,
                    last_coefficients,
                    last_propulsion,
                    dynamic_pressure_pa=runtime.dynamic_pressure_pa,
                    tvc=last_tvc_step,
                    rcs=last_rcs_step,
                    tvc_active=resolved_command.phase == "tvc_control" and tvc_config.mode > 0,
                )
            elif module == "euler":
                _runtime_euler(runtime, last_propulsion, last_wrench, dt_s)
            elif module == "newton":
                _runtime_newton(runtime, last_propulsion, last_wrench, dt_s)
            elif module == "intercept":
                pass
            ####
        ####
        if sim_time + 0.5 * dt_s >= next_sample_time or sim_time + 0.5 * dt_s >= requested_end:
            samples.append(
                _runtime_sample(
                    sim_time,
                    definition,
                    resolved_command,
                    runtime,
                    last_propulsion,
                    last_fin_step,
                    last_tvc_step,
                    last_rcs_step,
                    last_wrench,
                )
            )
            while next_sample_time <= sim_time + 0.5 * dt_s:
                next_sample_time += cadence
            ####
        ####
        if not _runtime_is_finite(runtime):
            return Ads6SamRunResult(
                source_name=definition.source_name,
                source_phase=resolved_command.phase,
                integration_step_s=dt_s,
                requested_end_time_s=requested_end,
                executed_steps=steps,
                terminated_reason="nonfinite_state",
                source_artifacts=definition.source_artifacts,
                samples=tuple(samples),
            )
        ####
        if runtime.altitude_m < 0.0 and sim_time > 0.0:
            return Ads6SamRunResult(
                source_name=definition.source_name,
                source_phase=resolved_command.phase,
                integration_step_s=dt_s,
                requested_end_time_s=requested_end,
                executed_steps=steps,
                terminated_reason="ground_impact",
                source_artifacts=definition.source_artifacts,
                samples=tuple(samples),
            )
        ####
        sim_time += dt_s
    ####
    return Ads6SamRunResult(
        source_name=definition.source_name,
        source_phase=resolved_command.phase,
        integration_step_s=dt_s,
        requested_end_time_s=requested_end,
        executed_steps=steps,
        terminated_reason="end_time",
        source_artifacts=definition.source_artifacts,
        samples=tuple(samples),
    )


####


def _validate_selected_realization(
    phase: Ads6SamPhase,
    module_order: tuple[str, ...],
    tvc: Ads6SamTvcConfig,
    rcs: Ads6SamRcsConfig,
) -> None:
    if phase == "fin_control" and "actuator" not in module_order:
        raise ValueError("ADS6 SAM fin-control phase requires the source actuator module")
    ####
    if phase == "tvc_control":
        if "tvc" not in module_order:
            raise ValueError("ADS6 SAM TVC phase requires the source TVC module")
        ####
        if tvc.mode == 0:
            raise ValueError("ADS6 SAM TVC phase requires a nonzero source or override TVC mode")
        ####
    ####
    if phase == "aggregate_rcs":
        if "rcs" not in module_order:
            raise ValueError("ADS6 SAM aggregate-RCS phase requires the source RCS module")
        ####
        if rcs.moment_mode == 0 and rcs.force_mode == 0:
            raise ValueError("ADS6 SAM aggregate-RCS phase requires a nonzero moment or force mode")
        ####
    ####


####


def _initialize_runtime(definition: Ads6SamSourceDefinition) -> _Ads6SamRuntime:
    initial = definition.initial_state
    quaternion = np.asarray(ads6_sam_initial_quaternion(initial), dtype=np.float64)
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
    return _Ads6SamRuntime(
        position_ned_m=np.asarray(initial.position_ned_m, dtype=np.float64),
        position_derivative_ned_mps=np.zeros(3, dtype=np.float64),
        velocity_body_mps=body_velocity,
        velocity_body_derivative_mps2=np.zeros(3, dtype=np.float64),
        velocity_ned_mps=transform.T @ body_velocity,
        quaternion_wxyz=quaternion,
        quaternion_derivative=np.zeros(4, dtype=np.float64),
        body_rates_rad_s=np.asarray(initial.body_rates_deg_s, dtype=np.float64) * _RAD_PER_DEG,
        body_rate_derivative_rad_s2=np.zeros(3, dtype=np.float64),
        fin_state=Ads6SamFinActuatorState(),
        tvc_state=Ads6SamTvcState(),
        rcs_state=Ads6SamRcsState(),
        achieved_fins=Ads6SamFinSet(),
        achieved_control=Ads6SamControlCommand(),
        alpha_deg=initial.alpha_deg,
        beta_deg=initial.beta_deg,
        altitude_m=-float(initial.position_ned_m[2]),
        body_angles_deg=(initial.roll_deg, initial.pitch_deg, initial.yaw_deg),
        specific_force_body_mps2=np.zeros(3, dtype=np.float64),
    )


####


def _runtime_environment(runtime: _Ads6SamRuntime) -> None:
    runtime.gravity_mps2 = cadac_source_inverse_square_gravity_mps2(runtime.altitude_m)
    density, pressure, temperature = atmosphere76(runtime.altitude_m)
    runtime.density_kg_m3 = density
    runtime.pressure_pa = pressure
    runtime.temperature_k = temperature
    runtime.speed_of_sound_mps = math.sqrt(1.4 * 287.053 * temperature)
    speed = float(np.linalg.norm(runtime.velocity_ned_mps))
    runtime.mach = abs(speed / runtime.speed_of_sound_mps)
    runtime.dynamic_pressure_pa = 0.5 * density * speed * speed


####


def _runtime_kinematics(runtime: _Ads6SamRuntime, dt_s: float) -> None:
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
    speed = float(np.linalg.norm(body_air_velocity))
    if speed <= 1.0e-12:
        runtime.alpha_deg = 0.0
        runtime.beta_deg = 0.0
    else:
        runtime.alpha_deg = math.atan2(float(body_air_velocity[2]), float(body_air_velocity[0])) * _DEG_PER_RAD
        argument = max(-1.0, min(1.0, float(body_air_velocity[1]) / speed))
        runtime.beta_deg = math.asin(argument) * _DEG_PER_RAD
    ####
    runtime.body_angles_deg = _euler_from_dcm(transform)


####


def _runtime_euler(
    runtime: _Ads6SamRuntime,
    propulsion: Ads6SamPropulsionStep,
    wrench: Ads6SamBodyWrench,
    dt_s: float,
) -> None:
    p_rate, q_rate, r_rate = runtime.body_rates_rad_s
    mx, my, mz = wrench.moment_body_nm
    roll_inertia = propulsion.roll_inertia_kgm2
    transverse = propulsion.transverse_inertia_kgm2
    derivative_new = np.asarray(
        (
            mx / roll_inertia,
            ((transverse - roll_inertia) * p_rate * r_rate + my) / transverse,
            (-(transverse - roll_inertia) * p_rate * q_rate + mz) / transverse,
        ),
        dtype=np.float64,
    )
    runtime.body_rates_rad_s = _integrate_vector(
        runtime.body_rates_rad_s,
        derivative_new,
        runtime.body_rate_derivative_rad_s2,
        dt_s,
    )
    runtime.body_rate_derivative_rad_s2 = derivative_new


####


def _runtime_newton(
    runtime: _Ads6SamRuntime,
    propulsion: Ads6SamPropulsionStep,
    wrench: Ads6SamBodyWrench,
    dt_s: float,
) -> None:
    transform = _dcm_body_from_local(runtime.quaternion_wxyz)
    omega_cross_velocity = np.cross(runtime.body_rates_rad_s, runtime.velocity_body_mps)
    gravity_local = np.asarray((0.0, 0.0, runtime.gravity_mps2), dtype=np.float64)
    runtime.specific_force_body_mps2 = np.asarray(wrench.force_body_n, dtype=np.float64) / propulsion.mass_kg
    derivative_new = runtime.specific_force_body_mps2 - omega_cross_velocity + transform @ gravity_local
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
    definition: Ads6SamSourceDefinition,
    command: Ads6SamDirectCommand,
    runtime: _Ads6SamRuntime,
    propulsion: Ads6SamPropulsionStep,
    fins: Ads6SamFinActuatorStep,
    tvc: Ads6SamTvcStep,
    rcs: Ads6SamRcsStep,
    wrench: Ads6SamBodyWrench,
) -> Ads6SamSample:
    highest = command.phase in {"fin_control", "tvc_control"}
    _, effective_rcs = _ads6_sam_resolve_command_configs(definition, command)
    return Ads6SamSample(
        time_s=time_s,
        source_phase=command.phase,
        fidelity="rigid_body_6dof_surface_allocated" if highest else "rigid_body_6dof_direct_wrench",
        control_realization="effector_allocated" if highest else "direct_wrench",
        position_ned_m=_tuple3(runtime.position_ned_m),
        velocity_ned_mps=_tuple3(runtime.velocity_ned_mps),
        quaternion_wxyz=_tuple4(runtime.quaternion_wxyz),
        body_rates_rad_s=_tuple3(runtime.body_rates_rad_s),
        body_angles_deg=runtime.body_angles_deg,
        incidence_deg=(runtime.alpha_deg, runtime.beta_deg),
        altitude_m=runtime.altitude_m,
        speed_mps=float(np.linalg.norm(runtime.velocity_ned_mps)),
        mach=runtime.mach,
        dynamic_pressure_pa=runtime.dynamic_pressure_pa,
        mass_kg=propulsion.mass_kg,
        center_of_gravity_m=propulsion.center_of_gravity_m,
        inertia_diagonal_kgm2=(
            propulsion.roll_inertia_kgm2,
            propulsion.transverse_inertia_kgm2,
            propulsion.transverse_inertia_kgm2,
        ),
        thrust_n=propulsion.thrust_n,
        requested_control_deg=command.control.vector(),
        requested_fins_deg=fins.requested_fins.vector(),
        achieved_fins_deg=fins.achieved_fins.vector(),
        achieved_control_deg=fins.achieved_control.vector(),
        requested_tvc_pitch_yaw_deg=tvc.requested_pitch_yaw_deg,
        achieved_tvc_pitch_yaw_deg=tvc.achieved_pitch_yaw_deg,
        tvc_effective_gain=tvc.effective_gain,
        requested_rcs_attitude_deg=(
            effective_rcs.roll_command_deg,
            effective_rcs.pitch_command_deg,
            effective_rcs.yaw_command_deg,
        ),
        requested_rcs_incidence_deg=command.incidence_commands_deg,
        requested_rcs_acceleration_g=command.acceleration_commands_g,
        requested_thrust_vector_unit_body=_tuple3(
            np.asarray(command.thrust_vector_unit_body, dtype=np.float64) / float(np.linalg.norm(command.thrust_vector_unit_body))
        ),
        achieved_lateral_normal_acceleration_g=(
            float(runtime.specific_force_body_mps2[1]) / max(runtime.gravity_mps2, 1.0e-12),
            -float(runtime.specific_force_body_mps2[2]) / max(runtime.gravity_mps2, 1.0e-12),
        ),
        rcs_force_body_n=rcs.force_body_n,
        rcs_moment_body_nm=rcs.moment_body_nm,
        force_body_n=wrench.force_body_n,
        moment_body_nm=wrench.moment_body_nm,
        fin_position_limited=fins.position_limited,
        fin_rate_limited=fins.rate_limited,
        tvc_position_limited=tvc.position_limited,
        tvc_rate_limited=tvc.rate_limited,
    )


####


def _runtime_is_finite(runtime: _Ads6SamRuntime) -> bool:
    arrays = (
        runtime.position_ned_m,
        runtime.position_derivative_ned_mps,
        runtime.velocity_body_mps,
        runtime.velocity_body_derivative_mps2,
        runtime.velocity_ned_mps,
        runtime.quaternion_wxyz,
        runtime.quaternion_derivative,
        runtime.body_rates_rad_s,
        runtime.body_rate_derivative_rad_s2,
        runtime.specific_force_body_mps2,
    )
    scalars = (
        runtime.alpha_deg,
        runtime.beta_deg,
        runtime.altitude_m,
        runtime.gravity_mps2,
        runtime.mach,
        runtime.dynamic_pressure_pa,
        runtime.pressure_pa,
    )
    return all(np.all(np.isfinite(array)) for array in arrays) and all(math.isfinite(value) for value in scalars)


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


def _euler_from_dcm(transform: FloatMatrix) -> tuple[float, float, float]:
    pitch = math.asin(max(-1.0, min(1.0, -float(transform[0, 2]))))
    cosine = max(1.0e-12, math.cos(pitch))
    yaw = math.atan2(float(transform[0, 1]) / cosine, float(transform[0, 0]) / cosine)
    roll = math.atan2(float(transform[1, 2]) / cosine, float(transform[2, 2]) / cosine)
    return (roll * _DEG_PER_RAD, pitch * _DEG_PER_RAD, yaw * _DEG_PER_RAD)


####


def _integrate_vector(
    state: FloatVector,
    derivative_new: FloatVector,
    derivative_previous: FloatVector,
    dt_s: float,
) -> FloatVector:
    return state + (derivative_new + derivative_previous) * (0.5 * dt_s)


####


def _number(vehicle: CadacVehicleBlock, name: str, default: float | None = None) -> float:
    try:
        value = vehicle.parameter(name)
    except KeyError:
        if default is None:
            raise Ads6SamSourceError(f"ADS6 SAM source vehicle is missing parameter {name!r}") from None
        ####
        return float(default)
    ####
    if not isinstance(value, (int, float)):
        raise Ads6SamSourceError(f"ADS6 SAM parameter {name!r} must be numeric")
    ####
    return float(value)


####


def _integer(vehicle: CadacVehicleBlock, name: str, default: int | None = None) -> int:
    value = _number(vehicle, name, None if default is None else float(default))
    if not value.is_integer():
        raise Ads6SamSourceError(f"ADS6 SAM parameter {name!r} must be integer-valued")
    ####
    return int(value)


####


def _tuple2(values: NDArray[np.float64]) -> tuple[float, float]:
    return (float(values[0]), float(values[1]))


####


def _tuple3(values: NDArray[np.float64] | tuple[float, float, float]) -> tuple[float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]))


####


def _tuple4(values: NDArray[np.float64]) -> tuple[float, float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]), float(values[3]))


####


def _fin_set(values: NDArray[np.float64]) -> Ads6SamFinSet:
    return Ads6SamFinSet(
        fin1_deg=float(values[0]),
        fin2_deg=float(values[1]),
        fin3_deg=float(values[2]),
        fin4_deg=float(values[3]),
    )


####


__all__ = [
    "Ads6SamAeroCoefficients",
    "Ads6SamAerodynamicConfig",
    "Ads6SamBodyWrench",
    "Ads6SamControlCommand",
    "Ads6SamDirectCommand",
    "Ads6SamFinActuatorConfig",
    "Ads6SamFinActuatorState",
    "Ads6SamFinActuatorStep",
    "Ads6SamFinSet",
    "Ads6SamInitialState",
    "Ads6SamModuleController",
    "Ads6SamPlantObservation",
    "Ads6SamPlantStepper",
    "Ads6SamPhase",
    "Ads6SamPropulsionConfig",
    "Ads6SamPropulsionStep",
    "Ads6SamRcsConfig",
    "Ads6SamRcsRuntimeInput",
    "Ads6SamRcsState",
    "Ads6SamRcsStep",
    "Ads6SamRunResult",
    "Ads6SamSample",
    "Ads6SamSourceDefinition",
    "Ads6SamSourceError",
    "Ads6SamTvcConfig",
    "Ads6SamTvcState",
    "Ads6SamTvcStep",
    "ads6_sam_aerodynamic_coefficients",
    "ads6_sam_body_wrench",
    "ads6_sam_fin_actuator_step",
    "ads6_sam_initial_quaternion",
    "ads6_sam_mix_fin_commands",
    "ads6_sam_propulsion_step",
    "ads6_sam_rcs_proportional",
    "ads6_sam_rcs_schmitt",
    "ads6_sam_rcs_step",
    "ads6_sam_tvc_step",
    "ads6_sam_unmix_fin_positions",
    "load_ads6_sam_selected_actor_definition",
    "load_ads6_sam_source_definition",
    "lower_ads6_sam_selected_actor_bundle",
    "lower_ads6_sam_source_bundle",
    "run_ads6_sam_physical_plant",
]
