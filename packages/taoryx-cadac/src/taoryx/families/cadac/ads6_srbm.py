"""Source-compatible ADS6 ``ROCKET5`` short-range ballistic-missile runtime."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal, TypeAlias

import numpy as np
from numpy.typing import NDArray
from pydantic import Field, model_validator

from taoryx.sensor_api import SensorContext

from .aim5 import mat2tr
from .bundle import CadacSourceArtifact, CadacSourceBundle, load_cadac_source_bundle
from .compatibility import cadac_stored_derivative_step
from .deck import CadacDeck
from .ghame6 import ghame6_atmosphere
from .input_ast import CadacDeckKind, CadacModel, CadacModuleStage, CadacVehicleBlock
from .sensor_adapter import cadac_local_ned_relative_state_track, cadac_local_ned_sensor_context
from .source_environment import cadac_source_inverse_square_gravity_mps2

FloatVector: TypeAlias = NDArray[np.float64]
FloatMatrix: TypeAlias = NDArray[np.float64]
Ads6SrbmPhase = Literal["endo_ascent", "exo_ballistic", "endo_reentry"]

_RAD_PER_DEG = math.pi / 180.0
_DEG_PER_RAD = 180.0 / math.pi
_AGRAV = 9.80675445
_SMALL = 1.0e-7

_ADS6_SRBM_REQUIRED_MODULES = (
    "environment",
    "kinematics",
    "propulsion",
    "aerodynamics",
    "guidance",
    "control",
    "forces",
    "newton",
    "intercept",
)
_ADS6_SRBM_SUPPORTED_MODULES = {
    *_ADS6_SRBM_REQUIRED_MODULES,
    "sensor",
    # Source-wide ADS6 module lists can include these; ROCKET5 dispatches them as no-ops.
    "ins",
    "actuator",
    "tvc",
    "rcs",
    "euler",
}
_ADS6_SRBM_AERO_TABLES = ("cltgt_vs_alpha_mach", "cdtgt_vs_alpha_mach")


class Ads6SrbmSourceError(ValueError):
    """Source-bundle incompatibility with the ADS6 SRBM response-law runtime."""


####


class Ads6SrbmInitialState(CadacModel):
    """Flat-Earth source initial state for one ``ROCKET5`` actor."""

    position_ned_m: tuple[float, float, float]
    speed_mps: float = Field(gt=0.0)
    heading_deg: float
    flight_path_deg: float
    alpha_deg: float
    beta_deg: float


####


class Ads6SrbmAerodynamicConfig(CadacModel):
    """SRBM reference geometry and source incidence limits."""

    reference_area_m2: float = Field(default=0.636, gt=0.0)
    alpha_limit_deg: float = Field(gt=0.0)
    normal_derivative_per_rad: float = 7.468
    side_derivative_per_rad: float = -7.468


####


class Ads6SrbmPropulsionConfig(CadacModel):
    """Pressure-corrected single-stage source rocket parameters."""

    mode: int = 1
    sea_level_pressure_pa: float = Field(default=101_325.0, gt=0.0)
    nozzle_exit_area_m2: float = Field(default=0.282, ge=0.0)
    launch_mass_kg: float = Field(default=6_000.0, gt=0.0)
    fuel_mass_kg: float = Field(default=4_000.0, gt=0.0)
    specific_impulse_s: float = Field(default=230.0, gt=0.0)
    sea_level_thrust_n: float = Field(default=128_600.0, gt=0.0)

    @model_validator(mode="after")
    def validate_propulsion(self) -> "Ads6SrbmPropulsionConfig":
        if self.mode not in {0, 1}:
            raise ValueError("ADS6 SRBM propulsion mode must be 0 or 1")
        ####
        if self.fuel_mass_kg >= self.launch_mass_kg:
            raise ValueError("ADS6 SRBM fuel mass must be less than launch mass")
        ####
        return self

    ####


####


class Ads6SrbmGuidanceConfig(CadacModel):
    """Kinematic sensor, proportional navigation, and spiral-maneuver settings."""

    seeker_mode: int = 0
    guidance_mode: int = 0
    navigation_gain: float = Field(default=0.0, ge=0.0)
    target_position_ned_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    maneuver_tgo_start_s: float = Field(default=0.0, ge=0.0)
    maneuver_initial_amplitude_g: float = Field(default=0.0, ge=0.0)
    maneuver_frequency_rad_s: float = Field(default=0.0, ge=0.0)
    maneuver_tgo63_s: float = Field(default=0.0, ge=0.0)

    @property
    def maneuver_mode(self) -> int:
        return self.guidance_mode // 10

    ####

    @property
    def navigation_mode(self) -> int:
        return self.guidance_mode % 10

    ####

    @model_validator(mode="after")
    def validate_guidance(self) -> "Ads6SrbmGuidanceConfig":
        if self.seeker_mode not in {0, 1}:
            raise ValueError("ADS6 SRBM seeker mode must be 0 or 1")
        ####
        if self.maneuver_mode not in {0, 1} or self.navigation_mode not in {0, 1}:
            raise ValueError("ADS6 SRBM guidance mode must encode optional spiral and optional proportional navigation")
        ####
        if self.maneuver_mode and self.maneuver_tgo63_s <= 0.0:
            raise ValueError("ADS6 SRBM spiral guidance requires positive maneuver_tgo63_s")
        ####
        if self.navigation_mode and self.seeker_mode == 0:
            raise ValueError("ADS6 SRBM proportional navigation requires the source kinematic seeker")
        ####
        return self

    ####


####


class Ads6SrbmControlConfig(CadacModel):
    """Reduced-order endo-atmospheric acceleration-response law."""

    mode: int = 1
    endo_boundary_altitude_m: float = Field(gt=0.0)
    ascent_normal_bias_g: float = 0.0

    @model_validator(mode="after")
    def validate_mode(self) -> "Ads6SrbmControlConfig":
        if self.mode not in {0, 1}:
            raise ValueError("ADS6 SRBM control mode must be 0 or 1")
        ####
        return self

    ####


####


class Ads6SrbmSourceDefinition(CadacModel):
    """Prepared ADS6 SRBM source case and immutable aerodynamic resource."""

    source_name: str = Field(min_length=1)
    source_model: str = "ROCKET5"
    integration_step_s: float = Field(gt=0.0)
    trajectory_step_s: float | None = Field(default=None, gt=0.0)
    end_time_s: float = Field(gt=0.0)
    launch_delay_s: float = Field(default=0.0, ge=0.0)
    module_order: tuple[str, ...]
    initial_state: Ads6SrbmInitialState
    aerodynamics: Ads6SrbmAerodynamicConfig
    propulsion: Ads6SrbmPropulsionConfig
    guidance: Ads6SrbmGuidanceConfig
    control: Ads6SrbmControlConfig
    aerodynamic_deck: CadacDeck
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=2)
    taoryx_tier: str = "pseudo_6dof"
    runtime_fidelity: str = "pseudo_6dof"
    control_realization: str = "response_law"
    claim_boundary: str = (
        "ADS6 ROCKET5/SRBM5 flat-Earth translation plus source pitch/yaw-rate and alpha/beta response-law states. "
        "The source has no rigid-body attitude, body-moment closure, or physical actuator allocation for this actor."
    )

    @model_validator(mode="after")
    def validate_source_boundary(self) -> "Ads6SrbmSourceDefinition":
        names = {table.name.casefold() for table in self.aerodynamic_deck.tables}
        missing_tables = tuple(name for name in _ADS6_SRBM_AERO_TABLES if name.casefold() not in names)
        if missing_tables:
            raise ValueError(f"ADS6 SRBM source definition is missing required tables: {missing_tables!r}")
        ####
        missing_modules = tuple(name for name in _ADS6_SRBM_REQUIRED_MODULES if name not in self.module_order)
        if missing_modules:
            raise ValueError(f"ADS6 SRBM source definition is missing required modules: {missing_modules!r}")
        ####
        if self.guidance.seeker_mode and "sensor" not in self.module_order:
            raise ValueError("ADS6 SRBM seeker-enabled source definition requires the sensor module")
        ####
        if self.control.mode != 1:
            raise ValueError("ADS6 SRBM runnable pseudo-6DoF claim requires source control mode 1")
        ####
        return self

    ####


####


class Ads6SrbmPhaseTransition(CadacModel):
    """One source phase transition caused by the sticky exo flag and altitude boundary."""

    time_s: float = Field(ge=0.0)
    previous_phase: Ads6SrbmPhase
    phase: Ads6SrbmPhase
    altitude_m: float


####


class Ads6SrbmImpact(CadacModel):
    """Ground or guided closest-approach termination result."""

    kind: Literal["ground_impact", "target_closest_approach"]
    time_s: float = Field(ge=0.0)
    position_ned_m: tuple[float, float, float]
    miss_distance_m: float | None = Field(default=None, ge=0.0)


####


class Ads6SrbmSample(CadacModel):
    """One accepted source-compatible SRBM sample."""

    time_s: float = Field(ge=0.0)
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]
    altitude_m: float
    speed_mps: float = Field(ge=0.0)
    heading_deg: float
    flight_path_deg: float
    alpha_deg: float
    beta_deg: float
    pitch_response_rate_rad_s: float
    yaw_response_rate_rad_s: float
    phase: Ads6SrbmPhase
    exo_flag: bool
    density_kg_m3: float = Field(ge=0.0)
    pressure_pa: float = Field(ge=0.0)
    dynamic_pressure_pa: float = Field(ge=0.0)
    mach: float = Field(ge=0.0)
    mass_kg: float = Field(gt=0.0)
    thrust_n: float = Field(ge=0.0)
    propulsion_mode: int
    lift_coefficient: float
    drag_coefficient: float
    axial_coefficient: float
    side_coefficient: float
    normal_coefficient: float
    max_g: float = Field(ge=0.0)
    normal_command_g: float
    lateral_command_g: float
    normal_acceleration_g: float
    lateral_acceleration_g: float
    range_to_target_m: float = Field(ge=0.0)
    closing_speed_mps: float
    time_to_go_s: float = Field(ge=0.0)
    target_displacement_ned_m: tuple[float, float, float]
    target_unit_body: tuple[float, float, float]
    line_of_sight_rate_body_rad_s: tuple[float, float, float]
    specific_force_body_mps2: tuple[float, float, float]
    fidelity: str = "pseudo_6dof"
    control_realization: str = "response_law"


####


class Ads6SrbmRunResult(CadacModel):
    """Deterministic source-compatible SRBM batch result."""

    schema_id: str = "taoryx.cadac.ads6-srbm-run/v0alpha1"
    source_name: str
    integration_step_s: float = Field(gt=0.0)
    requested_end_time_s: float = Field(gt=0.0)
    executed_steps: int = Field(ge=0)
    terminated_reason: str
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=2)
    samples: tuple[Ads6SrbmSample, ...] = Field(min_length=1)
    phase_transitions: tuple[Ads6SrbmPhaseTransition, ...] = ()
    impact: Ads6SrbmImpact | None = None
    claim_boundary: str


####


@dataclass(slots=True)
class _Ads6SrbmRuntime:
    """Mutable source state hidden behind immutable public records."""

    position_ned_m: FloatVector
    velocity_ned_mps: FloatVector
    acceleration_ned_mps2: FloatVector
    vehicle_to_local: FloatMatrix
    speed_mps: float
    heading_rad: float
    flight_path_rad: float
    altitude_m: float
    alpha_deg: float
    beta_deg: float
    alpha_rad: float = 0.0
    alpha_rate_rad_s: float = 0.0
    pitch_rate_rad_s: float = 0.0
    pitch_rate_derivative_rad_s2: float = 0.0
    pitch_integral_rad_s: float = 0.0
    pitch_integral_derivative_rad_s2: float = 0.0
    beta_rad: float = 0.0
    beta_rate_rad_s: float = 0.0
    yaw_rate_rad_s: float = 0.0
    yaw_rate_derivative_rad_s2: float = 0.0
    yaw_integral_rad_s: float = 0.0
    yaw_integral_derivative_rad_s2: float = 0.0
    exo_flag: bool = False
    density_kg_m3: float = 0.0
    pressure_pa: float = 0.0
    temperature_k: float = 0.0
    sound_speed_mps: float = 0.0
    gravity_mps2: float = _AGRAV
    dynamic_pressure_pa: float = 0.0
    mach: float = 0.0
    propulsion_mode: int = 1
    mass_kg: float = 6_000.0
    thrust_n: float = 0.0
    total_incidence_deg: float = 0.0
    aero_roll_deg: float = 0.0
    lift_coefficient: float = 0.0
    drag_coefficient: float = 0.0
    axial_coefficient: float = 0.0
    side_coefficient: float = 0.0
    normal_coefficient: float = 0.0
    max_g: float = 0.0
    range_to_target_m: float = 0.0
    closing_speed_mps: float = 0.0
    time_to_go_s: float = 0.0
    target_unit_body: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    line_of_sight_rate_body_rad_s: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    target_displacement_ned_m: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    normal_command_g: float = 0.0
    lateral_command_g: float = 0.0
    normal_spiral_command_g: float = 0.0
    lateral_spiral_command_g: float = 0.0
    specific_force_body_mps2: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))


####


@dataclass(slots=True)
class Ads6SrbmActorRuntime:
    """Step-owned ROCKET5 runtime for ADS6 package composition."""

    definition: Ads6SrbmSourceDefinition
    _runtime: _Ads6SrbmRuntime = field(init=False, repr=False)
    executed_steps: int = 0

    def __post_init__(self) -> None:
        self._runtime = _initial_runtime(self.definition)

    ####

    def step(self, sim_time_s: float) -> Ads6SrbmPhase:
        """Execute one source-ordered ROCKET5 module pass and return its phase."""

        _run_modules(
            self.definition,
            self._runtime,
            sim_time_s,
            self.definition.integration_step_s,
        )
        self.executed_steps += 1
        return _phase(self.definition, self._runtime)

    ####

    def sample(self, time_s: float) -> Ads6SrbmSample:
        """Project the current mutable actor state into an immutable sample."""

        return _sample(self._runtime, time_s, _phase(self.definition, self._runtime))

    ####

    def impact(self, time_s: float) -> Ads6SrbmImpact | None:
        """Evaluate the source target/ground termination conditions."""

        return _impact(self._runtime, self.definition, time_s)

    ####

    @property
    def phase(self) -> Ads6SrbmPhase:
        return _phase(self.definition, self._runtime)

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


class Ads6SrbmSession:
    """Persistent source-ordered ROCKET5 state for Mission Composition stepping.

    It reuses :class:`Ads6SrbmActorRuntime`, retaining its complete
    reduced-order plant, response-law, propulsion, and phase state between
    holds.  It deliberately accepts only exact source-step multiples so no
    synthetic fractional source controller pass is introduced.
    """

    def __init__(self, definition: Ads6SrbmSourceDefinition) -> None:
        self.definition = definition
        self.reset()
        ####

    def reset(self) -> None:
        """Reconstruct source state and terminal tracking from the immutable definition."""

        self.actor = Ads6SrbmActorRuntime(self.definition)
        self.sim_time_s = 0.0
        self.executed_steps = 0
        self.phase_transitions: list[Ads6SrbmPhaseTransition] = []
        self.impact_result: Ads6SrbmImpact | None = None
        self.terminated_reason: str | None = None
        ####

    @property
    def completed(self) -> bool:
        return self.terminated_reason is not None
        ####

    @property
    def sample(self) -> Ads6SrbmSample:
        """Return the latest committed source state projected at session time."""

        return self.actor.sample(self.sim_time_s)
        ####

    def native_sensor_context(self) -> SensorContext:
        """Return the fixed-target raw-track context at the accepted source state."""

        runtime = self.actor._runtime
        return cadac_local_ned_sensor_context(
            time_s=self.sim_time_s,
            host_position_ned_m=runtime.position_ned_m,
            host_velocity_ned_mps=runtime.velocity_ned_mps,
            target_id="ads6-srbm-target",
            target_position_ned_m=np.asarray(self.definition.guidance.target_position_ned_m, dtype=np.float64),
            target_velocity_ned_mps=np.zeros(3, dtype=np.float64),
            body_from_local=runtime.vehicle_to_local,
        )
        ####

    def advance(self, duration_s: float) -> tuple[Ads6SrbmPhaseTransition, ...]:
        """Advance through source module passes and retain each phase transition."""

        steps = _session_step_count(duration_s, self.definition.integration_step_s, "ADS6 SRBM")
        start = len(self.phase_transitions)
        for _ in range(steps):
            if self.completed:
                break
            ####
            self._advance_one()
        ####
        return tuple(self.phase_transitions[start:])
        ####

    def _advance_one(self) -> None:
        previous_phase = self.actor.phase
        current_phase = self.actor.step(self.sim_time_s)
        self.executed_steps += 1
        if current_phase != previous_phase:
            self.phase_transitions.append(
                Ads6SrbmPhaseTransition(
                    time_s=self.sim_time_s,
                    previous_phase=previous_phase,
                    phase=current_phase,
                    altitude_m=self.actor.altitude_m,
                )
            )
        ####
        impact = self.actor.impact(self.sim_time_s)
        if impact is not None:
            self.impact_result = impact
            self.terminated_reason = impact.kind
        elif not self.actor.finite:
            self.terminated_reason = "nonfinite_state"
        ####
        self.sim_time_s = min(self.definition.end_time_s, self.sim_time_s + self.definition.integration_step_s)
        if self.terminated_reason is None and self.sim_time_s >= self.definition.end_time_s - 1.0e-12:
            self.terminated_reason = "end_time"
        ####

    ####


def lower_ads6_srbm_actor(
    bundle: CadacSourceBundle,
    rocket: CadacVehicleBlock,
    *,
    allow_events: bool = False,
) -> Ads6SrbmSourceDefinition:
    """Lower one source ``ROCKET5`` block from a standalone or package case."""

    if rocket.model_name.casefold() != "rocket5":
        raise Ads6SrbmSourceError(f"expected ROCKET5 actor, received {rocket.model_name!r}")
    ####
    module_order = tuple(module.name.casefold() for module in bundle.case.modules if CadacModuleStage.EXECUTE in module.stages)
    unsupported = tuple(name for name in module_order if name not in _ADS6_SRBM_SUPPORTED_MODULES)
    if unsupported:
        raise Ads6SrbmSourceError(f"ADS6 SRBM source case contains unsupported executable modules: {unsupported!r}")
    ####
    if rocket.events and not allow_events:
        raise Ads6SrbmSourceError("ADS6 SRBM vehicle plug-in does not yet execute ROCKET5 source event mutations")
    ####
    if rocket.stochastic_assignments:
        raise Ads6SrbmSourceError("ADS6 SRBM vehicle plug-in does not yet materialize ROCKET5 stochastic declarations")
    ####
    try:
        integration_step = bundle.case.timing_values["int_step"]
    except KeyError as error:
        raise Ads6SrbmSourceError("ADS6 SRBM source case is missing TIMING int_step") from error
    ####
    try:
        aerodynamic_deck = bundle.deck_for("ROCKET5", CadacDeckKind.AERODYNAMIC, vehicle_role=rocket.role)
    except KeyError as error:
        raise Ads6SrbmSourceError("ADS6 SRBM source case requires exactly one ROCKET5 aerodynamic deck") from error
    ####
    return Ads6SrbmSourceDefinition(
        source_name=bundle.case.source_name,
        integration_step_s=integration_step,
        trajectory_step_s=bundle.case.timing_values.get("traj_step"),
        end_time_s=bundle.case.end_time_s,
        launch_delay_s=_number(rocket, "launch_delay", 0.0),
        module_order=module_order,
        initial_state=Ads6SrbmInitialState(
            position_ned_m=(
                _number(rocket, "sael1"),
                _number(rocket, "sael2"),
                _number(rocket, "sael3"),
            ),
            speed_mps=_number(rocket, "dvae"),
            heading_deg=_number(rocket, "psivlx"),
            flight_path_deg=_number(rocket, "thtvlx"),
            alpha_deg=_number(rocket, "alpha_t0x", 0.0),
            beta_deg=_number(rocket, "beta_t0x", 0.0),
        ),
        aerodynamics=Ads6SrbmAerodynamicConfig(
            reference_area_m2=_number(rocket, "area", 0.636),
            alpha_limit_deg=_number(rocket, "alpmax"),
        ),
        propulsion=Ads6SrbmPropulsionConfig(
            mode=_integer(rocket, "mprop", 1),
            sea_level_pressure_pa=_number(rocket, "pres_sl", 101_325.0),
            nozzle_exit_area_m2=_number(rocket, "aexit", 0.282),
            launch_mass_kg=_number(rocket, "mass_launch", 6_000.0),
            fuel_mass_kg=_number(rocket, "mass_fuel", 4_000.0),
            specific_impulse_s=_number(rocket, "isp", 230.0),
            sea_level_thrust_n=_number(rocket, "thrust_sl", 128_600.0),
        ),
        guidance=Ads6SrbmGuidanceConfig(
            seeker_mode=_integer(rocket, "mseek", 0),
            guidance_mode=_integer(rocket, "mguide", 0),
            navigation_gain=_number(rocket, "gnav", 0.0),
            target_position_ned_m=(
                _number(rocket, "stel1", 0.0),
                _number(rocket, "stel2", 0.0),
                _number(rocket, "stel3", 0.0),
            ),
            maneuver_tgo_start_s=_number(rocket, "tgo_manvr", 0.0),
            maneuver_initial_amplitude_g=_number(rocket, "amp_manvr", 0.0),
            maneuver_frequency_rad_s=_number(rocket, "frq_manvr", 0.0),
            maneuver_tgo63_s=_number(rocket, "tgo63_manvr", 0.0),
        ),
        control=Ads6SrbmControlConfig(
            mode=_integer(rocket, "maut", 1),
            endo_boundary_altitude_m=_number(rocket, "alt_endo"),
            ascent_normal_bias_g=_number(rocket, "ancomx_bias", 0.0),
        ),
        aerodynamic_deck=aerodynamic_deck,
        source_artifacts=bundle.artifacts,
    )


####


def load_ads6_srbm_source_definition(path: str | Path) -> Ads6SrbmSourceDefinition:
    """Parse and lower one standalone ADS6 ``ROCKET5`` source case."""

    bundle = load_cadac_source_bundle(path)
    rockets = bundle.case.vehicles_named("ROCKET5")
    if len(rockets) != 1:
        raise Ads6SrbmSourceError(f"ADS6 SRBM plug-in requires exactly one ROCKET5 actor; found {len(rockets)}")
    ####
    return lower_ads6_srbm_actor(bundle, rockets[0])


####


def run_ads6_srbm_source_compatibility(
    definition: Ads6SrbmSourceDefinition,
    *,
    end_time_s: float | None = None,
    sample_step_s: float | None = None,
) -> Ads6SrbmRunResult:
    """Execute the source-ordered ADS6 ROCKET5 response-law plant."""

    dt_s = definition.integration_step_s
    requested_end = definition.end_time_s if end_time_s is None else float(end_time_s)
    cadence = (definition.trajectory_step_s or dt_s) if sample_step_s is None else float(sample_step_s)
    if not math.isfinite(requested_end) or requested_end <= 0.0:
        raise ValueError("ADS6 SRBM end_time_s must be positive and finite")
    ####
    if not math.isfinite(cadence) or cadence <= 0.0:
        raise ValueError("ADS6 SRBM sample_step_s must be positive and finite")
    ####
    runtime = _initial_runtime(definition)
    samples: list[Ads6SrbmSample] = []
    transitions: list[Ads6SrbmPhaseTransition] = []
    previous_phase = _phase(definition, runtime)
    impact: Ads6SrbmImpact | None = None
    terminated_reason = "end_time"
    sim_time = 0.0
    next_sample = 0.0
    steps = 0
    while sim_time <= requested_end + 0.5 * dt_s:
        _run_modules(definition, runtime, sim_time, dt_s)
        steps += 1
        current_phase = _phase(definition, runtime)
        if current_phase != previous_phase:
            transitions.append(
                Ads6SrbmPhaseTransition(
                    time_s=sim_time,
                    previous_phase=previous_phase,
                    phase=current_phase,
                    altitude_m=runtime.altitude_m,
                )
            )
            previous_phase = current_phase
        ####
        if sim_time + 0.5 * dt_s >= next_sample or sim_time + 0.5 * dt_s >= requested_end:
            samples.append(_sample(runtime, sim_time, current_phase))
            while next_sample <= sim_time + 0.5 * dt_s:
                next_sample += cadence
            ####
        ####
        impact = _impact(runtime, definition, sim_time)
        if impact is not None:
            terminated_reason = impact.kind
            if not samples or abs(samples[-1].time_s - sim_time) > 0.25 * dt_s:
                samples.append(_sample(runtime, sim_time, current_phase))
            ####
            break
        ####
        if not _runtime_is_finite(runtime):
            terminated_reason = "nonfinite_state"
            break
        ####
        sim_time += dt_s
    ####
    if not samples:
        samples.append(_sample(runtime, 0.0, previous_phase))
    ####
    return Ads6SrbmRunResult(
        source_name=definition.source_name,
        integration_step_s=dt_s,
        requested_end_time_s=requested_end,
        executed_steps=steps,
        terminated_reason=terminated_reason,
        source_artifacts=definition.source_artifacts,
        samples=tuple(samples),
        phase_transitions=tuple(transitions),
        impact=impact,
        claim_boundary=definition.claim_boundary,
    )


####


def _initial_runtime(definition: Ads6SrbmSourceDefinition) -> _Ads6SrbmRuntime:
    initial = definition.initial_state
    heading = initial.heading_deg * _RAD_PER_DEG
    flight_path = initial.flight_path_deg * _RAD_PER_DEG
    velocity = _cart_from_polar(initial.speed_mps, heading, flight_path)
    return _Ads6SrbmRuntime(
        position_ned_m=np.asarray(initial.position_ned_m, dtype=np.float64),
        velocity_ned_mps=velocity,
        acceleration_ned_mps2=np.zeros(3, dtype=np.float64),
        vehicle_to_local=mat2tr(heading, flight_path),
        speed_mps=initial.speed_mps,
        heading_rad=heading,
        flight_path_rad=flight_path,
        altitude_m=-initial.position_ned_m[2],
        alpha_deg=initial.alpha_deg,
        beta_deg=initial.beta_deg,
        propulsion_mode=definition.propulsion.mode,
        mass_kg=definition.propulsion.launch_mass_kg,
    )


####


def _run_modules(
    definition: Ads6SrbmSourceDefinition,
    runtime: _Ads6SrbmRuntime,
    sim_time_s: float,
    dt_s: float,
) -> None:
    for module in definition.module_order:
        if module == "environment":
            _environment(runtime)
        elif module == "kinematics":
            pass
        elif module == "propulsion":
            _propulsion(definition, runtime, sim_time_s)
        elif module == "aerodynamics":
            _aerodynamics(definition, runtime)
        elif module == "sensor":
            _sensor(definition, runtime, sim_time_s)
        elif module == "guidance":
            _guidance(definition, runtime)
        elif module == "control":
            _control(definition, runtime, dt_s)
        elif module == "forces":
            _forces(definition, runtime)
        elif module == "newton":
            _newton(runtime, dt_s)
        elif module in {"intercept", "ins", "actuator", "tvc", "rcs", "euler"}:
            pass
        else:
            raise Ads6SrbmSourceError(f"unsupported ADS6 SRBM runtime module {module!r}")
        ####
    ####


####


def _environment(runtime: _Ads6SrbmRuntime) -> None:
    altitude = max(0.0, runtime.altitude_m)
    density, pressure, temperature, sound_speed = ghame6_atmosphere(100, altitude)
    runtime.gravity_mps2 = cadac_source_inverse_square_gravity_mps2(runtime.altitude_m)
    runtime.density_kg_m3 = density
    runtime.pressure_pa = pressure
    runtime.temperature_k = temperature
    runtime.sound_speed_mps = sound_speed
    runtime.mach = abs(runtime.speed_mps / sound_speed) if sound_speed > _SMALL else 999.0
    runtime.dynamic_pressure_pa = 0.5 * density * runtime.speed_mps * runtime.speed_mps


####


def _propulsion(definition: Ads6SrbmSourceDefinition, runtime: _Ads6SrbmRuntime, launch_time_s: float) -> None:
    config = definition.propulsion
    if runtime.propulsion_mode == 1:
        mass_flow_kg_s = config.sea_level_thrust_n / (config.specific_impulse_s * 9.81)
        runtime.mass_kg = config.launch_mass_kg - mass_flow_kg_s * launch_time_s
        runtime.thrust_n = config.sea_level_thrust_n + (config.sea_level_pressure_pa - runtime.pressure_pa) * config.nozzle_exit_area_m2
        if runtime.mass_kg <= config.launch_mass_kg - config.fuel_mass_kg:
            runtime.thrust_n = 0.0
            runtime.propulsion_mode = 0
        ####
    else:
        runtime.thrust_n = 0.0
    ####


####


def _aerodynamics(definition: Ads6SrbmSourceDefinition, runtime: _Ads6SrbmRuntime) -> None:
    alpha = runtime.alpha_deg * _RAD_PER_DEG
    beta = runtime.beta_deg * _RAD_PER_DEG
    cosine = _clamp(math.cos(alpha) * math.cos(beta), -1.0, 1.0)
    total_incidence = math.acos(cosine)
    tangent_beta = math.tan(beta)
    sine_alpha = math.sin(alpha)
    aero_roll = math.atan2(tangent_beta, sine_alpha) if tangent_beta * tangent_beta > _SMALL and sine_alpha * sine_alpha > _SMALL else 0.0
    total_incidence_deg = total_incidence * _DEG_PER_RAD
    lift = definition.aerodynamic_deck.table("cltgt_vs_alpha_mach").interpolate((total_incidence_deg, runtime.mach))
    drag = definition.aerodynamic_deck.table("cdtgt_vs_alpha_mach").interpolate((total_incidence_deg, runtime.mach))
    axial = drag * math.cos(alpha) - lift * math.sin(alpha)
    if runtime.propulsion_mode == 0:
        axial *= 1.1
    ####
    normal_plane = drag * math.sin(alpha) + lift * math.cos(alpha)
    normal = abs(normal_plane) * math.cos(aero_roll)
    side = -abs(normal_plane) * math.sin(aero_roll)
    max_lift = definition.aerodynamic_deck.table("cltgt_vs_alpha_mach").interpolate((definition.aerodynamics.alpha_limit_deg, runtime.mach))
    max_drag = definition.aerodynamic_deck.table("cdtgt_vs_alpha_mach").interpolate((definition.aerodynamics.alpha_limit_deg, runtime.mach))
    max_normal = max_drag * math.sin(definition.aerodynamics.alpha_limit_deg * _RAD_PER_DEG) + max_lift * math.cos(
        definition.aerodynamics.alpha_limit_deg * _RAD_PER_DEG
    )
    weight = runtime.mass_kg * runtime.gravity_mps2
    runtime.total_incidence_deg = total_incidence_deg
    runtime.aero_roll_deg = aero_roll * _DEG_PER_RAD
    runtime.lift_coefficient = lift
    runtime.drag_coefficient = drag
    runtime.axial_coefficient = axial
    runtime.side_coefficient = side
    runtime.normal_coefficient = normal
    runtime.max_g = max(0.0, max_normal * runtime.dynamic_pressure_pa * definition.aerodynamics.reference_area_m2 / weight)


####


def _sensor(
    definition: Ads6SrbmSourceDefinition,
    runtime: _Ads6SrbmRuntime,
    sim_time_s: float,
) -> None:
    if not (definition.guidance.seeker_mode and runtime.exo_flag and runtime.altitude_m < definition.control.endo_boundary_altitude_m):
        runtime.range_to_target_m = 0.0
        runtime.closing_speed_mps = 0.0
        runtime.time_to_go_s = 0.0
        runtime.target_unit_body = np.zeros(3, dtype=np.float64)
        runtime.line_of_sight_rate_body_rad_s = np.zeros(3, dtype=np.float64)
        runtime.target_displacement_ned_m = np.zeros(3, dtype=np.float64)
        return
    ####
    sensor = cadac_local_ned_relative_state_track(
        time_s=sim_time_s,
        host_position_ned_m=runtime.position_ned_m,
        host_velocity_ned_mps=runtime.velocity_ned_mps,
        target_id="ads6-srbm-target",
        target_position_ned_m=np.asarray(definition.guidance.target_position_ned_m, dtype=np.float64),
        target_velocity_ned_mps=np.zeros(3, dtype=np.float64),
        body_from_local=runtime.vehicle_to_local,
    )
    if isinstance(sensor, str):
        return
    ####
    displacement = runtime.vehicle_to_local.T @ np.asarray(sensor.relative_position_sensor_m, dtype=np.float64)
    distance = sensor.range_m
    # This source channel uses the sign opposite to the native closing-speed convention.
    closing = -sensor.closing_speed_mps
    runtime.target_displacement_ned_m = displacement
    runtime.range_to_target_m = distance
    runtime.closing_speed_mps = closing
    runtime.time_to_go_s = distance / abs(closing) if abs(closing) > _SMALL else 0.0
    runtime.target_unit_body = np.asarray(sensor.unit_los_sensor, dtype=np.float64)
    runtime.line_of_sight_rate_body_rad_s = np.asarray(sensor.line_of_sight_rate_sensor_rad_s, dtype=np.float64)


####


def _guidance(definition: Ads6SrbmSourceDefinition, runtime: _Ads6SrbmRuntime) -> None:
    guidance = definition.guidance
    maneuver_mode = guidance.maneuver_mode
    navigation_mode = guidance.navigation_mode
    unrestricted_normal = 0.0
    unrestricted_lateral = 0.0
    if navigation_mode == 1:
        command_body = np.cross(runtime.line_of_sight_rate_body_rad_s, runtime.target_unit_body) * guidance.navigation_gain * abs(runtime.closing_speed_mps)
        unrestricted_normal = -float(command_body[2]) / runtime.gravity_mps2
        unrestricted_lateral = float(command_body[1]) / runtime.gravity_mps2
    ####
    runtime.normal_spiral_command_g = 0.0
    runtime.lateral_spiral_command_g = 0.0
    if maneuver_mode == 1 and runtime.exo_flag and runtime.time_to_go_s < guidance.maneuver_tgo_start_s:
        amplitude = guidance.maneuver_initial_amplitude_g * (1.0 - math.exp(-runtime.time_to_go_s / guidance.maneuver_tgo63_s))
        runtime.normal_spiral_command_g = amplitude * math.sin(guidance.maneuver_frequency_rad_s * runtime.time_to_go_s)
        runtime.lateral_spiral_command_g = amplitude * math.cos(guidance.maneuver_frequency_rad_s * runtime.time_to_go_s)
        unrestricted_normal += runtime.normal_spiral_command_g
        unrestricted_lateral += runtime.lateral_spiral_command_g
    ####
    magnitude = math.hypot(unrestricted_lateral, unrestricted_normal)
    if magnitude > runtime.max_g:
        magnitude = runtime.max_g
    ####
    # Preserve the source's OR condition rather than replacing it with a cleaner both-zero test.
    angle = (
        0.0
        if abs(unrestricted_normal) < _SMALL or abs(unrestricted_lateral) < _SMALL
        else math.atan2(
            unrestricted_normal,
            unrestricted_lateral,
        )
    )
    runtime.lateral_command_g = magnitude * math.cos(angle)
    runtime.normal_command_g = magnitude * math.sin(angle)


####


def _control(definition: Ads6SrbmSourceDefinition, runtime: _Ads6SrbmRuntime, dt_s: float) -> None:
    config = definition.control
    aero = definition.aerodynamics
    if runtime.altitude_m > config.endo_boundary_altitude_m:
        runtime.exo_flag = True
        _reset_response_states(runtime)
    ####
    if not runtime.exo_flag:
        runtime.normal_command_g = config.ascent_normal_bias_g
    ####
    if config.mode != 1 or runtime.altitude_m >= config.endo_boundary_altitude_m:
        runtime.alpha_deg = 0.0
        runtime.beta_deg = 0.0
        return
    ####
    q = runtime.dynamic_pressure_pa
    speed = runtime.speed_mps
    mass = runtime.mass_kg
    area = aero.reference_area_m2
    rate_time_constant = -2.0e-7 * q + 0.22
    acceleration_gain = (0.002 * q) ** 0.575 * 0.5
    proportional_integral_ratio = 2.2
    if q <= _SMALL or speed <= _SMALL or rate_time_constant <= _SMALL:
        return
    ####
    incidence_time_constant = speed * mass / (q * area * abs(aero.normal_derivative_per_rad) + runtime.thrust_n)
    pitch_specific_force = -q * area * runtime.normal_coefficient / mass
    proportional_gain = acceleration_gain * incidence_time_constant * rate_time_constant / speed
    integral_gain = proportional_gain / proportional_integral_ratio
    desired_pitch_force = -runtime.normal_command_g * runtime.gravity_mps2
    pitch_error = desired_pitch_force - pitch_specific_force
    pitch_integral_rate_new = integral_gain * pitch_error
    runtime.pitch_integral_rad_s = _integrate_scalar(
        pitch_integral_rate_new,
        runtime.pitch_integral_derivative_rad_s2,
        runtime.pitch_integral_rad_s,
        dt_s,
    )
    runtime.pitch_integral_derivative_rad_s2 = pitch_integral_rate_new
    pitch_rate_command = -(pitch_error * proportional_gain + runtime.pitch_integral_rad_s)
    pitch_rate_derivative_new = (pitch_rate_command - runtime.pitch_rate_rad_s) / rate_time_constant
    runtime.pitch_rate_rad_s = _integrate_scalar(
        pitch_rate_derivative_new,
        runtime.pitch_rate_derivative_rad_s2,
        runtime.pitch_rate_rad_s,
        dt_s,
    )
    runtime.pitch_rate_derivative_rad_s2 = pitch_rate_derivative_new
    alpha_rate_new = (incidence_time_constant * runtime.pitch_rate_rad_s - runtime.alpha_rad) / incidence_time_constant
    runtime.alpha_rad = _integrate_scalar(alpha_rate_new, runtime.alpha_rate_rad_s, runtime.alpha_rad, dt_s)
    runtime.alpha_rate_rad_s = alpha_rate_new
    runtime.alpha_deg = _limit_deg(runtime.alpha_rad * _DEG_PER_RAD, aero.alpha_limit_deg)

    yaw_incidence_time_constant = speed * mass / (q * area * abs(aero.side_derivative_per_rad) + runtime.thrust_n)
    yaw_specific_force = q * area * runtime.side_coefficient / mass
    proportional_gain = acceleration_gain * yaw_incidence_time_constant * rate_time_constant / speed
    integral_gain = proportional_gain / proportional_integral_ratio
    desired_yaw_force = runtime.lateral_command_g * runtime.gravity_mps2
    yaw_error = desired_yaw_force - yaw_specific_force
    yaw_integral_rate_new = integral_gain * yaw_error
    runtime.yaw_integral_rad_s = _integrate_scalar(
        yaw_integral_rate_new,
        runtime.yaw_integral_derivative_rad_s2,
        runtime.yaw_integral_rad_s,
        dt_s,
    )
    runtime.yaw_integral_derivative_rad_s2 = yaw_integral_rate_new
    yaw_rate_command = yaw_error * proportional_gain + runtime.yaw_integral_rad_s
    yaw_rate_derivative_new = (yaw_rate_command - runtime.yaw_rate_rad_s) / rate_time_constant
    runtime.yaw_rate_rad_s = _integrate_scalar(
        yaw_rate_derivative_new,
        runtime.yaw_rate_derivative_rad_s2,
        runtime.yaw_rate_rad_s,
        dt_s,
    )
    runtime.yaw_rate_derivative_rad_s2 = yaw_rate_derivative_new
    beta_rate_new = -(yaw_incidence_time_constant * runtime.yaw_rate_rad_s + runtime.beta_rad) / yaw_incidence_time_constant
    runtime.beta_rad = _integrate_scalar(beta_rate_new, runtime.beta_rate_rad_s, runtime.beta_rad, dt_s)
    runtime.beta_rate_rad_s = beta_rate_new
    runtime.beta_deg = _limit_deg(runtime.beta_rad * _DEG_PER_RAD, aero.alpha_limit_deg)


####


def _reset_response_states(runtime: _Ads6SrbmRuntime) -> None:
    runtime.pitch_integral_rad_s = 0.0
    runtime.pitch_integral_derivative_rad_s2 = 0.0
    runtime.pitch_rate_rad_s = 0.0
    runtime.pitch_rate_derivative_rad_s2 = 0.0
    runtime.alpha_rad = 0.0
    runtime.alpha_rate_rad_s = 0.0
    runtime.yaw_integral_rad_s = 0.0
    runtime.yaw_integral_derivative_rad_s2 = 0.0
    runtime.yaw_rate_rad_s = 0.0
    runtime.yaw_rate_derivative_rad_s2 = 0.0
    runtime.beta_rad = 0.0
    runtime.beta_rate_rad_s = 0.0
    runtime.alpha_deg = 0.0
    runtime.beta_deg = 0.0


####


def _forces(definition: Ads6SrbmSourceDefinition, runtime: _Ads6SrbmRuntime) -> None:
    q_area = runtime.dynamic_pressure_pa * definition.aerodynamics.reference_area_m2
    runtime.specific_force_body_mps2 = np.array(
        (
            (runtime.thrust_n - runtime.axial_coefficient * q_area) / runtime.mass_kg,
            runtime.side_coefficient * q_area / runtime.mass_kg,
            -runtime.normal_coefficient * q_area / runtime.mass_kg,
        ),
        dtype=np.float64,
    )


####


def _newton(runtime: _Ads6SrbmRuntime, dt_s: float) -> None:
    gravity_local = np.array((0.0, 0.0, runtime.gravity_mps2), dtype=np.float64)
    acceleration_new = runtime.vehicle_to_local.T @ runtime.specific_force_body_mps2 + gravity_local
    velocity_new = _integrate_vector(acceleration_new, runtime.acceleration_ned_mps2, runtime.velocity_ned_mps, dt_s)
    position_new = _integrate_vector(velocity_new, runtime.velocity_ned_mps, runtime.position_ned_m, dt_s)
    runtime.acceleration_ned_mps2 = acceleration_new
    runtime.velocity_ned_mps = velocity_new
    runtime.position_ned_m = position_new
    speed, heading, flight_path = _polar_from_cart(velocity_new)
    runtime.speed_mps = speed
    runtime.heading_rad = heading
    runtime.flight_path_rad = flight_path
    runtime.vehicle_to_local = mat2tr(heading, flight_path)
    runtime.altitude_m = -float(position_new[2])


####


def _phase(definition: Ads6SrbmSourceDefinition, runtime: _Ads6SrbmRuntime) -> Ads6SrbmPhase:
    if not runtime.exo_flag:
        return "endo_ascent"
    ####
    if runtime.altitude_m > definition.control.endo_boundary_altitude_m:
        return "exo_ballistic"
    ####
    return "endo_reentry"


####


def _impact(
    runtime: _Ads6SrbmRuntime,
    definition: Ads6SrbmSourceDefinition,
    time_s: float,
) -> Ads6SrbmImpact | None:
    if runtime.range_to_target_m < 1_000.0 and definition.guidance.guidance_mode > 0 and runtime.closing_speed_mps > 0.0:
        return Ads6SrbmImpact(
            kind="target_closest_approach",
            time_s=time_s,
            position_ned_m=_tuple3(runtime.position_ned_m),
            miss_distance_m=runtime.range_to_target_m,
        )
    ####
    if runtime.altitude_m < 0.0:
        return Ads6SrbmImpact(
            kind="ground_impact",
            time_s=time_s,
            position_ned_m=_tuple3(runtime.position_ned_m),
        )
    ####
    return None


####


def _sample(runtime: _Ads6SrbmRuntime, time_s: float, phase: Ads6SrbmPhase) -> Ads6SrbmSample:
    return Ads6SrbmSample(
        time_s=time_s,
        position_ned_m=_tuple3(runtime.position_ned_m),
        velocity_ned_mps=_tuple3(runtime.velocity_ned_mps),
        altitude_m=runtime.altitude_m,
        speed_mps=runtime.speed_mps,
        heading_deg=runtime.heading_rad * _DEG_PER_RAD,
        flight_path_deg=runtime.flight_path_rad * _DEG_PER_RAD,
        alpha_deg=runtime.alpha_deg,
        beta_deg=runtime.beta_deg,
        pitch_response_rate_rad_s=runtime.pitch_rate_rad_s,
        yaw_response_rate_rad_s=runtime.yaw_rate_rad_s,
        phase=phase,
        exo_flag=runtime.exo_flag,
        density_kg_m3=runtime.density_kg_m3,
        pressure_pa=runtime.pressure_pa,
        dynamic_pressure_pa=runtime.dynamic_pressure_pa,
        mach=runtime.mach,
        mass_kg=runtime.mass_kg,
        thrust_n=runtime.thrust_n,
        propulsion_mode=runtime.propulsion_mode,
        lift_coefficient=runtime.lift_coefficient,
        drag_coefficient=runtime.drag_coefficient,
        axial_coefficient=runtime.axial_coefficient,
        side_coefficient=runtime.side_coefficient,
        normal_coefficient=runtime.normal_coefficient,
        max_g=runtime.max_g,
        normal_command_g=runtime.normal_command_g,
        lateral_command_g=runtime.lateral_command_g,
        normal_acceleration_g=-float(runtime.specific_force_body_mps2[2]) / max(runtime.gravity_mps2, _SMALL),
        lateral_acceleration_g=float(runtime.specific_force_body_mps2[1]) / max(runtime.gravity_mps2, _SMALL),
        range_to_target_m=runtime.range_to_target_m,
        closing_speed_mps=runtime.closing_speed_mps,
        time_to_go_s=runtime.time_to_go_s,
        target_displacement_ned_m=_tuple3(runtime.target_displacement_ned_m),
        target_unit_body=_tuple3(runtime.target_unit_body),
        line_of_sight_rate_body_rad_s=_tuple3(runtime.line_of_sight_rate_body_rad_s),
        specific_force_body_mps2=_tuple3(runtime.specific_force_body_mps2),
    )


####


def _runtime_is_finite(runtime: _Ads6SrbmRuntime) -> bool:
    scalars = (
        runtime.speed_mps,
        runtime.altitude_m,
        runtime.alpha_deg,
        runtime.beta_deg,
        runtime.mass_kg,
        runtime.thrust_n,
        runtime.dynamic_pressure_pa,
        runtime.mach,
    )
    return all(math.isfinite(value) for value in scalars) and bool(
        np.all(np.isfinite(runtime.position_ned_m)) and np.all(np.isfinite(runtime.velocity_ned_mps)) and np.all(np.isfinite(runtime.specific_force_body_mps2))
    )


####


def _number(vehicle: CadacVehicleBlock, name: str, default: float | None = None) -> float:
    try:
        value = vehicle.parameter(name)
    except KeyError:
        if default is None:
            raise Ads6SrbmSourceError(f"ROCKET5 source actor is missing required parameter {name!r}") from None
        ####
        return default
    ####
    if isinstance(value, str):
        raise Ads6SrbmSourceError(f"ROCKET5 parameter {name!r} must be numeric")
    ####
    result = float(value)
    if not math.isfinite(result):
        raise Ads6SrbmSourceError(f"ROCKET5 parameter {name!r} must be finite")
    ####
    return result


####


def _integer(vehicle: CadacVehicleBlock, name: str, default: int | None = None) -> int:
    value = _number(vehicle, name, None if default is None else float(default))
    result = int(value)
    if value != result:
        raise Ads6SrbmSourceError(f"ROCKET5 parameter {name!r} must be integral")
    ####
    return result


####


def _integrate_scalar(rate_new: float, rate_previous: float, state: float, dt_s: float) -> float:
    return cadac_stored_derivative_step((state,), (rate_new,), (rate_previous,), dt_s)[0]


####


def _integrate_vector(rate_new: FloatVector, rate_previous: FloatVector, state: FloatVector, dt_s: float) -> FloatVector:
    return np.asarray(cadac_stored_derivative_step(state, rate_new, rate_previous, dt_s), dtype=np.float64)


####


def _cart_from_polar(magnitude: float, azimuth_rad: float, elevation_rad: float) -> FloatVector:
    return np.array(
        (
            magnitude * math.cos(elevation_rad) * math.cos(azimuth_rad),
            magnitude * math.cos(elevation_rad) * math.sin(azimuth_rad),
            -magnitude * math.sin(elevation_rad),
        ),
        dtype=np.float64,
    )


####


def _polar_from_cart(vector: FloatVector) -> tuple[float, float, float]:
    north, east, down = (float(value) for value in vector)
    magnitude = math.sqrt(north * north + east * east + down * down)
    heading = math.atan2(east, north)
    horizontal = math.hypot(north, east)
    flight_path = math.atan2(-down, horizontal)
    return magnitude, heading, flight_path


####


def _limit_deg(value: float, limit: float) -> float:
    if abs(value) > limit:
        return math.copysign(limit, value)
    ####
    return value


####


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


####


def _session_step_count(duration_s: float, source_step_s: float, model_name: str) -> int:
    """Validate one external hold against the immutable source timestep."""

    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError(f"{model_name} session duration_s must be positive and finite")
    steps = round(duration_s / source_step_s)
    tolerance_s = max(1.0e-12, source_step_s * 1.0e-9)
    if steps <= 0 or not math.isclose(duration_s, steps * source_step_s, rel_tol=0.0, abs_tol=tolerance_s):
        raise ValueError(f"{model_name} session duration_s must be an integral multiple of source step {source_step_s:.17g} s")
    return steps
    ####


def _tuple3(vector: FloatVector) -> tuple[float, float, float]:
    return float(vector[0]), float(vector[1]), float(vector[2])


####


__all__ = [
    "Ads6SrbmAerodynamicConfig",
    "Ads6SrbmControlConfig",
    "Ads6SrbmGuidanceConfig",
    "Ads6SrbmImpact",
    "Ads6SrbmInitialState",
    "Ads6SrbmPhase",
    "Ads6SrbmPhaseTransition",
    "Ads6SrbmPropulsionConfig",
    "Ads6SrbmRunResult",
    "Ads6SrbmSample",
    "Ads6SrbmSession",
    "Ads6SrbmSourceDefinition",
    "Ads6SrbmSourceError",
    "load_ads6_srbm_source_definition",
    "run_ads6_srbm_source_compatibility",
]
