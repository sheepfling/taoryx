"""Source-compatible ADS6 ``AIRCRAFT3`` point-mass target runtime."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal, TypeAlias

import numpy as np
from numpy.typing import NDArray
from pydantic import Field, model_validator

from .aim5 import mat2tr
from .bundle import CadacSourceArtifact, CadacSourceBundle, load_cadac_source_bundle
from .compatibility import cadac_stored_derivative_step
from .ghame6 import ghame6_atmosphere
from .input_ast import CadacModel, CadacModuleStage, CadacVehicleBlock
from .source_environment import cadac_source_inverse_square_gravity_mps2

FloatVector: TypeAlias = NDArray[np.float64]
FloatMatrix: TypeAlias = NDArray[np.float64]
Ads6AircraftMode = Literal["steady", "g_turn", "escape"]

_RAD_PER_DEG = math.pi / 180.0
_DEG_PER_RAD = 180.0 / math.pi
_SMALL = 1.0e-9

_ADS6_AIRCRAFT_REQUIRED_MODULES = (
    "environment",
    "kinematics",
    "guidance",
    "control",
    "forces",
    "newton",
)
_ADS6_AIRCRAFT_SUPPORTED_MODULES = {
    *_ADS6_AIRCRAFT_REQUIRED_MODULES,
    # Source-wide ADS6 module lists dispatch these as no-ops for AIRCRAFT3.
    "propulsion",
    "aerodynamics",
    "ins",
    "sensor",
    "actuator",
    "tvc",
    "rcs",
    "euler",
    "intercept",
}


class Ads6AircraftSourceError(ValueError):
    """Source-bundle incompatibility with the ADS6 AIRCRAFT3 runtime."""


####


class Ads6AircraftInitialState(CadacModel):
    """Flat-Earth source initial state for one ``AIRCRAFT3`` actor."""

    position_ned_m: tuple[float, float, float]
    speed_mps: float = Field(gt=0.0)
    heading_deg: float
    flight_path_deg: float


####


class Ads6AircraftGuidanceConfig(CadacModel):
    """Source steady-flight, g-turn, or external-threat escape guidance."""

    option: int = 0
    guidance_gain: float = Field(default=0.0, ge=0.0)
    turn_load_g: float = 0.0
    maneuver_start_s: float = Field(default=0.0, ge=0.0)
    maneuver_stop_s: float = Field(default=0.0, ge=0.0)

    @model_validator(mode="after")
    def validate_guidance(self) -> "Ads6AircraftGuidanceConfig":
        if self.option not in {0, 1, 2}:
            raise ValueError("ADS6 AIRCRAFT3 guidance option must be 0, 1, or 2")
        ####
        if self.option > 0 and self.maneuver_stop_s <= self.maneuver_start_s:
            raise ValueError("maneuvering ADS6 AIRCRAFT3 cases require maneuver_stop_s > maneuver_start_s")
        ####
        return self

    ####


####


class Ads6AircraftControlConfig(CadacModel):
    """Source bank/load-factor response and point-mass force parameters."""

    bank_time_constant_s: float = Field(default=0.0, ge=0.0)
    bank_limit_deg: float = Field(gt=0.0)
    load_factor_time_constant_s: float = Field(default=0.0, ge=0.0)
    alpha_limit_deg: float = Field(gt=0.0)
    lift_slope_per_deg: float = Field(gt=0.0)
    wing_loading_n_m2: float = Field(gt=0.0)
    longitudinal_acceleration_g: float = 0.0
    initial_bank_state_rad: float = 0.0
    initial_bank_rate_rad_s: float = 0.0
    initial_load_factor_g: float = 0.0
    initial_load_factor_rate_g_s: float = 0.0


####


class Ads6AircraftThreatTrack(CadacModel):
    """Constant-velocity external threat observation for standalone escape-mode execution."""

    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]
    reference_time_s: float = 0.0

    @model_validator(mode="after")
    def validate_track(self) -> "Ads6AircraftThreatTrack":
        values = (*self.position_ned_m, *self.velocity_ned_mps, self.reference_time_s)
        if any(not math.isfinite(float(value)) for value in values):
            raise ValueError("ADS6 AIRCRAFT3 threat track must contain finite values")
        ####
        if _norm(np.asarray(self.velocity_ned_mps, dtype=np.float64)) <= _SMALL:
            raise ValueError("ADS6 AIRCRAFT3 escape-mode threat velocity must be nonzero")
        ####
        return self

    ####


####


class Ads6AircraftSourceDefinition(CadacModel):
    """Prepared ADS6 AIRCRAFT3 source case and explicit T1 claim boundary."""

    source_name: str = Field(min_length=1)
    source_role: str = Field(min_length=1)
    source_model: str = "AIRCRAFT3"
    integration_step_s: float = Field(gt=0.0)
    trajectory_step_s: float | None = Field(default=None, gt=0.0)
    end_time_s: float = Field(gt=0.0)
    launch_delay_s: float = Field(default=0.0, ge=0.0)
    module_order: tuple[str, ...]
    initial_state: Ads6AircraftInitialState
    guidance: Ads6AircraftGuidanceConfig
    control: Ads6AircraftControlConfig
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=1)
    taoryx_tier: str = "point_mass_3dof"
    runtime_fidelity: str = "point_mass_3dof"
    control_realization: str = "force_model"
    claim_boundary: str = (
        "ADS6 AIRCRAFT3 integrates Flat3 position and velocity. Bank angle and normal-load response are source control/force "
        "states used to orient point-mass specific force; no rigid-body attitude, body-rate, moment, or physical-effector "
        "state is claimed. Escape mode consumes an external threat observation through an explicit standalone seam."
    )

    @model_validator(mode="after")
    def validate_source_boundary(self) -> "Ads6AircraftSourceDefinition":
        missing_modules = tuple(name for name in _ADS6_AIRCRAFT_REQUIRED_MODULES if name not in self.module_order)
        if missing_modules:
            raise ValueError(f"ADS6 AIRCRAFT3 source definition is missing required modules: {missing_modules!r}")
        ####
        return self

    ####


####


class Ads6AircraftManeuverTransition(CadacModel):
    """One transition into or out of the strict source maneuver window."""

    time_s: float = Field(ge=0.0)
    active: bool
    mode: Ads6AircraftMode


####


class Ads6AircraftSample(CadacModel):
    """One accepted source-compatible AIRCRAFT3 point-mass sample."""

    time_s: float = Field(ge=0.0)
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]
    altitude_m: float
    speed_mps: float = Field(ge=0.0)
    heading_deg: float
    flight_path_deg: float
    mode: Ads6AircraftMode
    maneuver_active: bool
    density_kg_m3: float = Field(ge=0.0)
    pressure_pa: float = Field(ge=0.0)
    dynamic_pressure_pa: float = Field(ge=0.0)
    mach: float = Field(ge=0.0)
    gravity_mps2: float = Field(gt=0.0)
    commanded_acceleration_ned_mps2: tuple[float, float, float]
    commanded_acceleration_velocity_mps2: tuple[float, float, float]
    commanded_bank_deg: float
    bank_state_deg: float
    bank_deg: float
    bank_limited: bool
    commanded_load_factor_g: float = Field(ge=0.0)
    normal_load_factor_g: float
    load_factor_limit_g: float = Field(ge=0.0)
    load_factor_limited: bool
    longitudinal_acceleration_g: float
    specific_force_body_mps2: tuple[float, float, float]
    threat_range_m: float | None = Field(default=None, ge=0.0)
    fidelity: str = "point_mass_3dof"
    control_realization: str = "force_model"


####


class Ads6AircraftRunResult(CadacModel):
    """Deterministic source-compatible AIRCRAFT3 batch result."""

    schema_id: str = "taoryx.cadac.ads6-aircraft-run/v0alpha1"
    source_name: str
    integration_step_s: float = Field(gt=0.0)
    requested_end_time_s: float = Field(gt=0.0)
    executed_steps: int = Field(ge=0)
    terminated_reason: str
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=1)
    samples: tuple[Ads6AircraftSample, ...] = Field(min_length=1)
    maneuver_transitions: tuple[Ads6AircraftManeuverTransition, ...] = ()
    claim_boundary: str


####


@dataclass(slots=True)
class _Ads6AircraftRuntime:
    """Mutable source state hidden behind immutable public records."""

    position_ned_m: FloatVector
    velocity_ned_mps: FloatVector
    acceleration_ned_mps2: FloatVector
    velocity_to_local: FloatMatrix
    aircraft_to_local: FloatMatrix
    speed_mps: float
    heading_rad: float
    flight_path_rad: float
    altitude_m: float
    bank_state_rad: float
    bank_state_derivative_rad_s: float
    bank_output_rad: float = 0.0
    bank_command_rad: float = 0.0
    bank_limited: bool = False
    load_factor_g: float = 0.0
    load_factor_derivative_g_s: float = 0.0
    load_factor_command_g: float = 0.0
    load_factor_limit_g: float = 0.0
    load_factor_limited: bool = False
    density_kg_m3: float = 0.0
    pressure_pa: float = 0.0
    temperature_k: float = 0.0
    sound_speed_mps: float = 0.0
    gravity_mps2: float = 9.80665
    dynamic_pressure_pa: float = 0.0
    mach: float = 0.0
    commanded_acceleration_ned_mps2: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    commanded_acceleration_velocity_mps2: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    specific_force_body_mps2: FloatVector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    mode: Ads6AircraftMode = "steady"
    maneuver_active: bool = False
    threat_range_m: float | None = None


####


@dataclass(slots=True)
class Ads6AircraftActorRuntime:
    """Step-owned AIRCRAFT3 runtime for ADS6 package composition."""

    definition: Ads6AircraftSourceDefinition
    _runtime: _Ads6AircraftRuntime = field(init=False, repr=False)
    executed_steps: int = 0

    def __post_init__(self) -> None:
        self._runtime = _initial_runtime(self.definition)

    ####

    def step(
        self,
        sim_time_s: float,
        *,
        threat_track: Ads6AircraftThreatTrack | None = None,
    ) -> None:
        """Execute one source-ordered AIRCRAFT3 module pass."""

        _run_modules(
            self.definition,
            self._runtime,
            sim_time_s,
            self.definition.integration_step_s,
            threat_track,
        )
        self.executed_steps += 1

    ####

    def sample(self, time_s: float) -> Ads6AircraftSample:
        """Project the current mutable actor state into an immutable sample."""

        return _sample(self._runtime, time_s)

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


def lower_ads6_aircraft_actor(
    bundle: CadacSourceBundle,
    actor: CadacVehicleBlock,
    *,
    allow_events: bool = False,
) -> Ads6AircraftSourceDefinition:
    """Lower one source ``AIRCRAFT3`` block from a standalone or package case."""

    if actor.model_name.casefold() != "aircraft3":
        raise Ads6AircraftSourceError(f"expected AIRCRAFT3 actor, received {actor.model_name!r}")
    ####
    module_order = tuple(module.name.casefold() for module in bundle.case.modules if CadacModuleStage.EXECUTE in module.stages)
    unsupported = tuple(name for name in module_order if name not in _ADS6_AIRCRAFT_SUPPORTED_MODULES)
    if unsupported:
        raise Ads6AircraftSourceError(f"ADS6 AIRCRAFT3 source case contains unsupported executable modules: {unsupported!r}")
    ####
    if actor.deck_references:
        raise Ads6AircraftSourceError("ADS6 AIRCRAFT3 source actor must not bind aerodynamic or propulsion decks")
    ####
    if actor.events and not allow_events:
        raise Ads6AircraftSourceError("ADS6 AIRCRAFT3 vehicle plug-in does not yet execute actor-local source events")
    ####
    if actor.stochastic_assignments:
        raise Ads6AircraftSourceError("ADS6 AIRCRAFT3 vehicle plug-in does not yet materialize actor-local stochastic declarations")
    ####
    try:
        integration_step = bundle.case.timing_values["int_step"]
    except KeyError as error:
        raise Ads6AircraftSourceError("ADS6 AIRCRAFT3 source case is missing TIMING int_step") from error
    ####
    return Ads6AircraftSourceDefinition(
        source_name=bundle.case.source_name,
        source_role=actor.role,
        integration_step_s=integration_step,
        trajectory_step_s=bundle.case.timing_values.get("traj_step"),
        end_time_s=bundle.case.end_time_s,
        launch_delay_s=_number(actor, "launch_delay", 0.0),
        module_order=module_order,
        initial_state=Ads6AircraftInitialState(
            position_ned_m=(
                _number(actor, "sael1"),
                _number(actor, "sael2"),
                _number(actor, "sael3"),
            ),
            speed_mps=_number(actor, "dvae"),
            heading_deg=_number(actor, "psivlx"),
            flight_path_deg=_number(actor, "thtvlx", 0.0),
        ),
        guidance=Ads6AircraftGuidanceConfig(
            option=_integer(actor, "acft_option", 0),
            guidance_gain=_number(actor, "guid_gain", 0.0),
            turn_load_g=_number(actor, "gturn", 0.0),
            maneuver_start_s=_number(actor, "man_start", 0.0),
            maneuver_stop_s=_number(actor, "man_stop", 0.0),
        ),
        control=Ads6AircraftControlConfig(
            bank_time_constant_s=_number(actor, "tphi", 0.0),
            bank_limit_deg=_number(actor, "philimx"),
            load_factor_time_constant_s=_number(actor, "tanx", 0.0),
            alpha_limit_deg=_number(actor, "alplimx"),
            lift_slope_per_deg=_number(actor, "clalpha"),
            wing_loading_n_m2=_number(actor, "wingloading"),
            longitudinal_acceleration_g=_number(actor, "acc_longx", 0.0),
            initial_bank_state_rad=_number(actor, "phiav", 0.0),
            initial_bank_rate_rad_s=_number(actor, "phiavd", 0.0),
            initial_load_factor_g=_number(actor, "anx", 0.0),
            initial_load_factor_rate_g_s=_number(actor, "anxd", 0.0),
        ),
        source_artifacts=bundle.artifacts,
    )


####


def load_ads6_aircraft_source_definition(path: str | Path) -> Ads6AircraftSourceDefinition:
    """Parse and lower one standalone ADS6 ``AIRCRAFT3`` source case."""

    bundle = load_cadac_source_bundle(path)
    aircraft = bundle.case.vehicles_named("AIRCRAFT3")
    if len(aircraft) != 1:
        raise Ads6AircraftSourceError(f"ADS6 AIRCRAFT3 plug-in requires exactly one AIRCRAFT3 actor; found {len(aircraft)}")
    ####
    return lower_ads6_aircraft_actor(bundle, aircraft[0])


####


def run_ads6_aircraft_source_compatibility(
    definition: Ads6AircraftSourceDefinition,
    *,
    threat_track: Ads6AircraftThreatTrack | None = None,
    end_time_s: float | None = None,
    sample_step_s: float | None = None,
) -> Ads6AircraftRunResult:
    """Execute the source-ordered ADS6 AIRCRAFT3 point-mass plant."""

    dt_s = definition.integration_step_s
    requested_end = definition.end_time_s if end_time_s is None else float(end_time_s)
    cadence = (definition.trajectory_step_s or dt_s) if sample_step_s is None else float(sample_step_s)
    if not math.isfinite(requested_end) or requested_end <= 0.0:
        raise ValueError("ADS6 AIRCRAFT3 end_time_s must be positive and finite")
    ####
    if not math.isfinite(cadence) or cadence <= 0.0:
        raise ValueError("ADS6 AIRCRAFT3 sample_step_s must be positive and finite")
    ####
    if definition.guidance.option == 2 and threat_track is None:
        raise Ads6AircraftSourceError("ADS6 AIRCRAFT3 escape mode requires an external threat track in standalone execution")
    ####
    runtime = _initial_runtime(definition)
    samples: list[Ads6AircraftSample] = []
    transitions: list[Ads6AircraftManeuverTransition] = []
    previous_active = False
    terminated_reason = "end_time"
    sim_time = 0.0
    next_sample = 0.0
    steps = 0
    while sim_time <= requested_end + 0.5 * dt_s:
        if sim_time + 0.5 * dt_s >= definition.launch_delay_s:
            _run_modules(definition, runtime, sim_time, dt_s, threat_track)
            steps += 1
        ####
        if runtime.maneuver_active != previous_active:
            transitions.append(
                Ads6AircraftManeuverTransition(
                    time_s=sim_time,
                    active=runtime.maneuver_active,
                    mode=runtime.mode,
                )
            )
            previous_active = runtime.maneuver_active
        ####
        if sim_time + 0.5 * dt_s >= next_sample or sim_time + 0.5 * dt_s >= requested_end:
            samples.append(_sample(runtime, sim_time))
            while next_sample <= sim_time + 0.5 * dt_s:
                next_sample += cadence
            ####
        ####
        if runtime.altitude_m < 0.0:
            terminated_reason = "ground_impact"
            if not samples or abs(samples[-1].time_s - sim_time) > 0.25 * dt_s:
                samples.append(_sample(runtime, sim_time))
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
        samples.append(_sample(runtime, 0.0))
    ####
    return Ads6AircraftRunResult(
        source_name=definition.source_name,
        integration_step_s=dt_s,
        requested_end_time_s=requested_end,
        executed_steps=steps,
        terminated_reason=terminated_reason,
        source_artifacts=definition.source_artifacts,
        samples=tuple(samples),
        maneuver_transitions=tuple(transitions),
        claim_boundary=definition.claim_boundary,
    )


####


def _initial_runtime(definition: Ads6AircraftSourceDefinition) -> _Ads6AircraftRuntime:
    initial = definition.initial_state
    control = definition.control
    heading = initial.heading_deg * _RAD_PER_DEG
    flight_path = initial.flight_path_deg * _RAD_PER_DEG
    velocity = _cart_from_polar(initial.speed_mps, heading, flight_path)
    velocity_to_local = mat2tr(heading, flight_path)
    aircraft_to_local = _roll_matrix(0.0) @ velocity_to_local
    return _Ads6AircraftRuntime(
        position_ned_m=np.asarray(initial.position_ned_m, dtype=np.float64),
        velocity_ned_mps=velocity,
        acceleration_ned_mps2=np.zeros(3, dtype=np.float64),
        velocity_to_local=velocity_to_local,
        aircraft_to_local=aircraft_to_local,
        speed_mps=initial.speed_mps,
        heading_rad=heading,
        flight_path_rad=flight_path,
        altitude_m=-initial.position_ned_m[2],
        bank_state_rad=control.initial_bank_state_rad,
        bank_state_derivative_rad_s=control.initial_bank_rate_rad_s,
        load_factor_g=control.initial_load_factor_g,
        load_factor_derivative_g_s=control.initial_load_factor_rate_g_s,
    )


####


def _run_modules(
    definition: Ads6AircraftSourceDefinition,
    runtime: _Ads6AircraftRuntime,
    sim_time_s: float,
    dt_s: float,
    threat_track: Ads6AircraftThreatTrack | None,
) -> None:
    for module in definition.module_order:
        if module == "environment":
            _environment(runtime)
        elif module == "kinematics":
            pass
        elif module == "guidance":
            _guidance(definition, runtime, sim_time_s, threat_track)
        elif module == "control":
            _control(definition, runtime, dt_s)
        elif module == "forces":
            _forces(definition, runtime)
        elif module == "newton":
            _newton(runtime, dt_s)
        elif module in {
            "propulsion",
            "aerodynamics",
            "ins",
            "sensor",
            "actuator",
            "tvc",
            "rcs",
            "euler",
            "intercept",
        }:
            pass
        else:
            raise Ads6AircraftSourceError(f"unsupported ADS6 AIRCRAFT3 runtime module {module!r}")
        ####
    ####


####


def _environment(runtime: _Ads6AircraftRuntime) -> None:
    altitude = runtime.altitude_m
    density, pressure, temperature, sound_speed = ghame6_atmosphere(100, altitude)
    runtime.density_kg_m3 = density
    runtime.pressure_pa = pressure
    runtime.temperature_k = temperature
    runtime.sound_speed_mps = sound_speed
    runtime.gravity_mps2 = cadac_source_inverse_square_gravity_mps2(altitude)
    runtime.mach = abs(runtime.speed_mps / sound_speed) if sound_speed > _SMALL else 999.0
    runtime.dynamic_pressure_pa = 0.5 * density * runtime.speed_mps * runtime.speed_mps


####


def _guidance(
    definition: Ads6AircraftSourceDefinition,
    runtime: _Ads6AircraftRuntime,
    sim_time_s: float,
    threat_track: Ads6AircraftThreatTrack | None,
) -> None:
    guidance = definition.guidance
    gravity_bias = np.array((0.0, 0.0, -runtime.gravity_mps2), dtype=np.float64)
    active = guidance.option > 0 and guidance.maneuver_start_s < sim_time_s < guidance.maneuver_stop_s
    runtime.maneuver_active = active
    runtime.threat_range_m = None
    if not active or guidance.option == 0:
        runtime.mode = "steady"
        runtime.commanded_acceleration_ned_mps2 = gravity_bias
        return
    ####
    if guidance.option == 1:
        runtime.mode = "g_turn"
        command_velocity = np.array(
            (0.0, guidance.turn_load_g * runtime.gravity_mps2, -runtime.gravity_mps2),
            dtype=np.float64,
        )
        runtime.commanded_acceleration_ned_mps2 = runtime.velocity_to_local.T @ command_velocity
        return
    ####
    if threat_track is None:
        raise Ads6AircraftSourceError("escape-mode AIRCRAFT3 guidance lost its required external threat track")
    ####
    runtime.mode = "escape"
    elapsed = sim_time_s - threat_track.reference_time_s
    threat_position = np.asarray(threat_track.position_ned_m, dtype=np.float64) + elapsed * np.asarray(
        threat_track.velocity_ned_mps,
        dtype=np.float64,
    )
    threat_velocity = np.asarray(threat_track.velocity_ned_mps, dtype=np.float64)
    displacement = runtime.position_ned_m - threat_position
    distance = _norm(displacement)
    runtime.threat_range_m = distance
    own_speed = _norm(runtime.velocity_ned_mps)
    threat_speed = _norm(threat_velocity)
    if distance <= _SMALL or own_speed <= _SMALL or threat_speed <= _SMALL:
        raise Ads6AircraftSourceError("escape-mode AIRCRAFT3 guidance encountered degenerate threat geometry")
    ####
    gain = guidance.guidance_gain * _norm(np.cross(runtime.velocity_ned_mps, threat_velocity)) / distance
    own_unit = runtime.velocity_ned_mps / own_speed
    threat_unit = threat_velocity / threat_speed
    epsilon = np.cross(own_unit, threat_unit)
    runtime.commanded_acceleration_ned_mps2 = np.cross(epsilon, own_unit) * gain + gravity_bias


####


def _control(
    definition: Ads6AircraftSourceDefinition,
    runtime: _Ads6AircraftRuntime,
    dt_s: float,
) -> None:
    control = definition.control
    command_velocity = runtime.velocity_to_local @ runtime.commanded_acceleration_ned_mps2
    runtime.commanded_acceleration_velocity_mps2 = command_velocity
    lateral = float(command_velocity[1])
    normal_down = float(command_velocity[2])
    if abs(lateral) < _SMALL and abs(normal_down) < _SMALL:
        bank_command = 0.0
    else:
        bank_command = math.atan2(lateral, -normal_down)
    ####
    runtime.bank_command_rad = bank_command
    if control.bank_time_constant_s > 0.0:
        bank_derivative_new = (bank_command - runtime.bank_state_rad) / control.bank_time_constant_s
        runtime.bank_state_rad = _integrate_scalar(
            bank_derivative_new,
            runtime.bank_state_derivative_rad_s,
            runtime.bank_state_rad,
            dt_s,
        )
        runtime.bank_state_derivative_rad_s = bank_derivative_new
    else:
        runtime.bank_state_rad = bank_command
    ####
    bank_state_deg = runtime.bank_state_rad * _DEG_PER_RAD
    runtime.bank_limited = abs(bank_state_deg) >= control.bank_limit_deg
    bank_output_deg = _limit_signed(bank_state_deg, control.bank_limit_deg)
    runtime.bank_output_rad = bank_output_deg * _RAD_PER_DEG

    load_command = math.sqrt(lateral * lateral + normal_down * normal_down) / runtime.gravity_mps2
    runtime.load_factor_command_g = load_command
    if control.load_factor_time_constant_s > 0.0:
        load_derivative_new = (load_command - runtime.load_factor_g) / control.load_factor_time_constant_s
        runtime.load_factor_g = _integrate_scalar(
            load_derivative_new,
            runtime.load_factor_derivative_g_s,
            runtime.load_factor_g,
            dt_s,
        )
        runtime.load_factor_derivative_g_s = load_derivative_new
    else:
        runtime.load_factor_g = load_command
    ####
    runtime.load_factor_limit_g = runtime.dynamic_pressure_pa * control.lift_slope_per_deg * control.alpha_limit_deg / control.wing_loading_n_m2
    runtime.load_factor_limited = False
    if definition.guidance.option > 0 and abs(runtime.load_factor_g) >= runtime.load_factor_limit_g:
        runtime.load_factor_limited = True
        runtime.load_factor_g = _limit_signed(runtime.load_factor_g, runtime.load_factor_limit_g)
    ####


####


def _forces(definition: Ads6AircraftSourceDefinition, runtime: _Ads6AircraftRuntime) -> None:
    runtime.specific_force_body_mps2 = np.array(
        (
            definition.control.longitudinal_acceleration_g * runtime.gravity_mps2,
            0.0,
            -runtime.load_factor_g * runtime.gravity_mps2,
        ),
        dtype=np.float64,
    )


####


def _newton(runtime: _Ads6AircraftRuntime, dt_s: float) -> None:
    gravity_local = np.array((0.0, 0.0, runtime.gravity_mps2), dtype=np.float64)
    acceleration_new = runtime.aircraft_to_local.T @ runtime.specific_force_body_mps2 + gravity_local
    velocity_new = _integrate_vector(
        acceleration_new,
        runtime.acceleration_ned_mps2,
        runtime.velocity_ned_mps,
        dt_s,
    )
    position_new = _integrate_vector(velocity_new, runtime.velocity_ned_mps, runtime.position_ned_m, dt_s)
    runtime.acceleration_ned_mps2 = acceleration_new
    runtime.velocity_ned_mps = velocity_new
    runtime.position_ned_m = position_new
    speed, heading, flight_path = _polar_from_cart(velocity_new)
    runtime.speed_mps = speed
    runtime.heading_rad = heading
    runtime.flight_path_rad = flight_path
    runtime.velocity_to_local = mat2tr(heading, flight_path)
    runtime.aircraft_to_local = _roll_matrix(runtime.bank_output_rad) @ runtime.velocity_to_local
    runtime.altitude_m = -float(position_new[2])


####


def _sample(runtime: _Ads6AircraftRuntime, time_s: float) -> Ads6AircraftSample:
    return Ads6AircraftSample(
        time_s=time_s,
        position_ned_m=_tuple3(runtime.position_ned_m),
        velocity_ned_mps=_tuple3(runtime.velocity_ned_mps),
        altitude_m=runtime.altitude_m,
        speed_mps=runtime.speed_mps,
        heading_deg=runtime.heading_rad * _DEG_PER_RAD,
        flight_path_deg=runtime.flight_path_rad * _DEG_PER_RAD,
        mode=runtime.mode,
        maneuver_active=runtime.maneuver_active,
        density_kg_m3=runtime.density_kg_m3,
        pressure_pa=runtime.pressure_pa,
        dynamic_pressure_pa=runtime.dynamic_pressure_pa,
        mach=runtime.mach,
        gravity_mps2=runtime.gravity_mps2,
        commanded_acceleration_ned_mps2=_tuple3(runtime.commanded_acceleration_ned_mps2),
        commanded_acceleration_velocity_mps2=_tuple3(runtime.commanded_acceleration_velocity_mps2),
        commanded_bank_deg=runtime.bank_command_rad * _DEG_PER_RAD,
        bank_state_deg=runtime.bank_state_rad * _DEG_PER_RAD,
        bank_deg=runtime.bank_output_rad * _DEG_PER_RAD,
        bank_limited=runtime.bank_limited,
        commanded_load_factor_g=runtime.load_factor_command_g,
        normal_load_factor_g=runtime.load_factor_g,
        load_factor_limit_g=runtime.load_factor_limit_g,
        load_factor_limited=runtime.load_factor_limited,
        longitudinal_acceleration_g=runtime.specific_force_body_mps2[0] / runtime.gravity_mps2,
        specific_force_body_mps2=_tuple3(runtime.specific_force_body_mps2),
        threat_range_m=runtime.threat_range_m,
    )


####


def _runtime_is_finite(runtime: _Ads6AircraftRuntime) -> bool:
    scalars = (
        runtime.speed_mps,
        runtime.altitude_m,
        runtime.bank_state_rad,
        runtime.bank_output_rad,
        runtime.load_factor_g,
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
            raise Ads6AircraftSourceError(f"AIRCRAFT3 source actor is missing required parameter {name!r}") from None
        ####
        return default
    ####
    if isinstance(value, str):
        raise Ads6AircraftSourceError(f"AIRCRAFT3 parameter {name!r} must be numeric")
    ####
    result = float(value)
    if not math.isfinite(result):
        raise Ads6AircraftSourceError(f"AIRCRAFT3 parameter {name!r} must be finite")
    ####
    return result


####


def _integer(vehicle: CadacVehicleBlock, name: str, default: int | None = None) -> int:
    value = _number(vehicle, name, None if default is None else float(default))
    result = int(value)
    if value != result:
        raise Ads6AircraftSourceError(f"AIRCRAFT3 parameter {name!r} must be integral")
    ####
    return result


####


def _integrate_scalar(rate_new: float, rate_previous: float, state: float, dt_s: float) -> float:
    return cadac_stored_derivative_step((state,), (rate_new,), (rate_previous,), dt_s)[0]


####


def _integrate_vector(
    rate_new: FloatVector,
    rate_previous: FloatVector,
    state: FloatVector,
    dt_s: float,
) -> FloatVector:
    return np.asarray(
        cadac_stored_derivative_step(state, rate_new, rate_previous, dt_s),
        dtype=np.float64,
    )


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
    magnitude = _norm(vector)
    if magnitude <= _SMALL:
        return 0.0, 0.0, 0.0
    ####
    heading = math.atan2(float(vector[1]), float(vector[0]))
    flight_path = math.atan2(-float(vector[2]), math.hypot(float(vector[0]), float(vector[1])))
    return magnitude, heading, flight_path


####


def _roll_matrix(bank_rad: float) -> FloatMatrix:
    cosine = math.cos(bank_rad)
    sine = math.sin(bank_rad)
    result = np.eye(3, dtype=np.float64)
    result[1, 1] = cosine
    result[2, 2] = cosine
    result[1, 2] = sine
    result[2, 1] = -sine
    return result


####


def _limit_signed(value: float, limit: float) -> float:
    if abs(value) < limit:
        return value
    ####
    if value == 0.0:
        return 0.0
    ####
    return math.copysign(limit, value)


####


def _norm(vector: FloatVector) -> float:
    return float(np.linalg.norm(vector))


####


def _tuple3(vector: FloatVector) -> tuple[float, float, float]:
    return (float(vector[0]), float(vector[1]), float(vector[2]))


####


__all__ = [
    "Ads6AircraftControlConfig",
    "Ads6AircraftGuidanceConfig",
    "Ads6AircraftInitialState",
    "Ads6AircraftManeuverTransition",
    "Ads6AircraftMode",
    "Ads6AircraftRunResult",
    "Ads6AircraftSample",
    "Ads6AircraftSourceDefinition",
    "Ads6AircraftSourceError",
    "Ads6AircraftThreatTrack",
    "load_ads6_aircraft_source_definition",
    "run_ads6_aircraft_source_compatibility",
]
