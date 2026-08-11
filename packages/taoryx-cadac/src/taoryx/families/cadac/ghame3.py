"""Source-grounded GHAME3 round-Earth point-mass reconstruction."""

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
    cadac_round3_environment,
    cadac_round3_initialize,
    cadac_round3_newton_step,
)

FloatVector: TypeAlias = NDArray[np.float64]

_STANDARD_GRAVITY_MPS2 = 9.80675445
_GHAME3_MODULES = ("environment", "aerodynamics", "propulsion", "forces", "newton")
_GHAME3_AERO_TABLES = ("cd0_vs_mach", "cl0_vs_mach", "ckk_vs_mach", "cla_vs_mach", "cla0_vs_mach")
_GHAME3_PROP_TABLES = ("ca_vs_alpha_mach", "spi_vs_throttle_mach")
_SUPPORTED_PROPULSION_MODES = frozenset({0, 1, 2})
_SUPPORTED_EVENT_VARIABLES = frozenset({"time", "mprop", "qhold", "tq", "alphax", "phimvx", "throttle"})


class Ghame3SourceError(ValueError):
    """Source-bundle incompatibility with the GHAME3 point-mass reconstruction."""


####


class Ghame3PropulsionConfig(CadacModel):
    """Source hypersonic engine and fuel configuration."""

    mode: int
    initial_mass_kg: float = Field(gt=0.0)
    initial_fuel_kg: float = Field(ge=0.0)
    cowl_area_m2: float = Field(gt=0.0)
    throttle: float = Field(ge=0.0)
    idle_throttle: float = Field(ge=0.0)
    maximum_throttle: float = Field(gt=0.0)
    dynamic_pressure_command_pa: float = Field(default=0.0, ge=0.0)
    autothrottle_time_constant_s: float = Field(default=1.0, gt=0.0)


####


class Ghame3SourceDefinition(CadacModel):
    """Prepared single-vehicle GHAME3 source case."""

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
    propulsion: Ghame3PropulsionConfig
    aerodynamic_deck: CadacDeck
    propulsion_deck: CadacDeck
    events: tuple[CadacEventBlock, ...] = ()
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)
    taoryx_tier: str = "point_mass_3dof"
    runtime_fidelity: str = "point_mass_3dof"
    control_realization: str = "force_model"

    @model_validator(mode="after")
    def validate_tables(self) -> "Ghame3SourceDefinition":
        aero = {table.name.casefold() for table in self.aerodynamic_deck.tables}
        prop = {table.name.casefold() for table in self.propulsion_deck.tables}
        missing = [name for name in _GHAME3_AERO_TABLES if name.casefold() not in aero]
        missing.extend(name for name in _GHAME3_PROP_TABLES if name.casefold() not in prop)
        if missing:
            raise ValueError("GHAME3 source definition is missing required tables: " + ", ".join(missing))
        ####
        return self

    ####


####


class Ghame3Sample(CadacModel):
    """One source-ordered GHAME3 point-mass sample."""

    time_s: float = Field(ge=0.0)
    longitude_deg: float
    latitude_deg: float
    altitude_m: float
    speed_mps: float = Field(ge=0.0)
    heading_deg: float
    flight_path_deg: float
    velocity_geographic_mps: tuple[float, float, float]
    specific_force_velocity_mps2: tuple[float, float, float]
    dynamic_pressure_pa: float = Field(ge=0.0)
    mach: float = Field(ge=0.0)
    alpha_deg: float
    bank_deg: float
    lift_to_drag: float
    propulsion_mode: int
    throttle: float
    thrust_n: float = Field(ge=0.0)
    specific_impulse_s: float = Field(ge=0.0)
    capture_area_factor: float = Field(ge=0.0)
    mass_kg: float = Field(gt=0.0)
    fuel_mass_kg: float


####


class Ghame3EventTrace(CadacModel):
    """One source event applied before a GHAME3 module pass."""

    event_index: int = Field(ge=0)
    time_s: float = Field(ge=0.0)
    source_line: int = Field(ge=1)
    watch_variable: str = Field(min_length=1)
    criterion: int | float
    updates: tuple[tuple[str, int | float], ...]


####


class Ghame3RunResult(CadacModel):
    """Source-compatible GHAME3 point-mass batch result."""

    schema_id: str = "taoryx.cadac.ghame3-source-compatibility/v0alpha1"
    source_name: str = Field(min_length=1)
    integration_step_s: float = Field(gt=0.0)
    requested_end_time_s: float = Field(gt=0.0)
    executed_steps: int = Field(ge=0)
    terminated_reason: str = Field(min_length=1)
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)
    samples: tuple[Ghame3Sample, ...] = Field(min_length=1)
    events: tuple[Ghame3EventTrace, ...] = ()
    claim_boundary: str = (
        "Source-compatible GHAME3 round/rotating-Earth 3-DoF translation with prescribed alpha/bank and fixed/Q-hold hypersonic propulsion. "
        "No pseudo-6DoF or rigid-body rotational response is claimed."
    )


####


@dataclass(slots=True)
class _Ghame3Runtime:
    round3: CadacRound3RuntimeState
    values: dict[str, int | float]
    mass_kg: float
    fuel_expended_kg: float
    fuel_rate_kg_s: float
    specific_force_velocity_mps2: FloatVector
    lift_coefficient: float = 0.0
    drag_coefficient: float = 0.0
    lift_to_drag: float = 0.0
    thrust_n: float = 0.0
    specific_impulse_s: float = 0.0
    capture_area_factor: float = 0.0
    fuel_mass_kg: float = 0.0


####


def lower_ghame3_source_bundle(bundle: CadacSourceBundle) -> Ghame3SourceDefinition:
    """Lower one GHAME3 ``CRUISE3`` case into a typed point-mass definition."""

    vehicles = bundle.case.vehicles_named("CRUISE3")
    if len(vehicles) != 1 or len(bundle.case.vehicles) != 1:
        raise Ghame3SourceError(f"GHAME3 source lowering requires exactly one CRUISE3; found {len(vehicles)}")
    ####
    module_order = tuple(module.name.casefold() for module in bundle.case.modules if CadacModuleStage.EXECUTE in module.stages)
    unknown = tuple(name for name in module_order if name not in _GHAME3_MODULES)
    if unknown:
        raise Ghame3SourceError(f"GHAME3 point-mass runtime does not implement source modules: {unknown!r}")
    ####
    missing = tuple(name for name in _GHAME3_MODULES if name not in module_order)
    if missing:
        raise Ghame3SourceError(f"GHAME3 point-mass runtime requires source modules: {missing!r}")
    ####
    timing = bundle.case.timing_values
    if "int_step" not in timing:
        raise Ghame3SourceError("GHAME3 source case must declare TIMING int_step")
    ####
    vehicle = vehicles[0]
    mode = _integer(vehicle, "mprop", 0)
    _validate_events(mode, vehicle.events)
    return Ghame3SourceDefinition(
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
        reference_area_m2=_number(vehicle, "area"),
        propulsion=Ghame3PropulsionConfig(
            mode=mode,
            initial_mass_kg=_number(vehicle, "mass0"),
            initial_fuel_kg=_number(vehicle, "fmass0", 0.0),
            cowl_area_m2=_number(vehicle, "acowl"),
            throttle=_number(vehicle, "throttle", 0.0),
            idle_throttle=_number(vehicle, "thrtl_idle", 0.0),
            maximum_throttle=_number(vehicle, "thrtl_max", 1.0),
            dynamic_pressure_command_pa=_number(vehicle, "qhold", 0.0),
            autothrottle_time_constant_s=_number(vehicle, "tq", 1.0),
        ),
        aerodynamic_deck=bundle.deck_for("CRUISE3", CadacDeckKind.AERODYNAMIC),
        propulsion_deck=bundle.deck_for("CRUISE3", CadacDeckKind.PROPULSION),
        events=vehicle.events,
        source_artifacts=bundle.artifacts,
    )


####


def load_ghame3_source_definition(path: str | Path) -> Ghame3SourceDefinition:
    """Parse, fingerprint, and lower one GHAME3 source case."""

    return lower_ghame3_source_bundle(load_cadac_source_bundle(path))


####


def run_ghame3_source_compatibility(
    definition: Ghame3SourceDefinition,
    *,
    end_time_s: float | None = None,
    sample_step_s: float | None = None,
) -> Ghame3RunResult:
    """Execute the GHAME3 source module order as a point-mass Round3 vehicle."""

    dt_s = definition.integration_step_s
    requested_end = definition.end_time_s if end_time_s is None else float(end_time_s)
    if not math.isfinite(requested_end) or requested_end <= 0.0:
        raise ValueError("GHAME3 end_time_s must be positive and finite")
    ####
    cadence = (definition.plot_step_s or dt_s) if sample_step_s is None else float(sample_step_s)
    if not math.isfinite(cadence) or cadence <= 0.0:
        raise ValueError("GHAME3 sample_step_s must be positive and finite")
    ####
    runtime = _initialize_runtime(definition)
    event_cursor = CadacEventCursor.from_events(definition.events)
    samples: list[Ghame3Sample] = []
    event_trace: list[Ghame3EventTrace] = []
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
                Ghame3EventTrace(
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
                runtime.values["time"] = sim_time
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
            ####
        ####
        if sim_time + 0.5 * dt_s >= next_sample:
            samples.append(_sample(sim_time, runtime, environment))
            while next_sample <= sim_time + 0.5 * dt_s:
                next_sample += cadence
            ####
        ####
        if not _runtime_is_finite(runtime):
            terminated_reason = "nonfinite_state"
            break
        ####
        sim_time += dt_s
    ####
    return Ghame3RunResult(
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


def _initialize_runtime(definition: Ghame3SourceDefinition) -> _Ghame3Runtime:
    return _Ghame3Runtime(
        round3=cadac_round3_initialize(definition.initial_state),
        values={
            "time": 0.0,
            "mprop": definition.propulsion.mode,
            "qhold": definition.propulsion.dynamic_pressure_command_pa,
            "tq": definition.propulsion.autothrottle_time_constant_s,
            "alphax": definition.initial_alpha_deg,
            "phimvx": definition.initial_bank_deg,
            "throttle": definition.propulsion.throttle,
        },
        mass_kg=definition.propulsion.initial_mass_kg,
        fuel_expended_kg=0.0,
        fuel_rate_kg_s=0.0,
        specific_force_velocity_mps2=np.zeros(3, dtype=np.float64),
        fuel_mass_kg=definition.propulsion.initial_fuel_kg,
    )


####


def _aerodynamics(definition: Ghame3SourceDefinition, runtime: _Ghame3Runtime, mach: float) -> None:
    deck = definition.aerodynamic_deck
    alpha = float(runtime.values["alphax"])
    cd0 = deck.table("cd0_vs_mach").interpolate((mach,))
    cl0 = deck.table("cl0_vs_mach").interpolate((mach,))
    ckk = deck.table("ckk_vs_mach").interpolate((mach,))
    cla = deck.table("cla_vs_mach").interpolate((mach,))
    cla0 = deck.table("cla0_vs_mach").interpolate((mach,))
    runtime.lift_coefficient = cla0 + cla * alpha
    runtime.drag_coefficient = cd0 + ckk * (runtime.lift_coefficient - cl0) ** 2
    runtime.lift_to_drag = runtime.lift_coefficient / runtime.drag_coefficient


####


def _propulsion(
    definition: Ghame3SourceDefinition,
    runtime: _Ghame3Runtime,
    environment: CadacRound3Environment,
    dt_s: float,
) -> None:
    mode = int(runtime.values["mprop"])
    alpha = float(runtime.values["alphax"])
    throttle = float(runtime.values["throttle"])
    if mode not in _SUPPORTED_PROPULSION_MODES:
        raise Ghame3SourceError(f"GHAME3 runtime supports propulsion modes {sorted(_SUPPORTED_PROPULSION_MODES)}; got {mode}")
    ####
    thrust = 0.0
    spi = 0.0
    capture = 0.0
    if mode > 0:
        spi = definition.propulsion_deck.table("spi_vs_throttle_mach").interpolate((throttle, environment.mach))
        capture = definition.propulsion_deck.table("ca_vs_alpha_mach").interpolate((alpha, environment.mach))
        denominator = 0.029 * spi * _STANDARD_GRAVITY_MPS2 * environment.density_kg_m3 * runtime.round3.speed_mps * capture * definition.propulsion.cowl_area_m2
        if mode == 1:
            thrust = denominator * throttle
        elif mode == 2:
            if denominator != 0.0:
                required_thrust = definition.reference_area_m2 * runtime.drag_coefficient * float(runtime.values["qhold"]) / math.cos(alpha * RAD_PER_DEG)
                throttle_required = required_thrust / denominator
                gain_q = 2.0 * runtime.mass_kg / (environment.density_kg_m3 * runtime.round3.speed_mps * denominator * float(runtime.values["tq"]))
                throttle = gain_q * (float(runtime.values["qhold"]) - environment.dynamic_pressure_pa) + throttle_required
            ####
            if throttle < 0.0:
                throttle = definition.propulsion.idle_throttle
            ####
            if throttle > definition.propulsion.maximum_throttle:
                throttle = definition.propulsion.maximum_throttle
            ####
            spi = definition.propulsion_deck.table("spi_vs_throttle_mach").interpolate((throttle, environment.mach))
            thrust = (
                0.029
                * spi
                * throttle
                * _STANDARD_GRAVITY_MPS2
                * environment.density_kg_m3
                * runtime.round3.speed_mps
                * capture
                * definition.propulsion.cowl_area_m2
            )
        ####
        if spi != 0.0:
            fuel_rate_new = thrust / (spi * _STANDARD_GRAVITY_MPS2)
            runtime.fuel_expended_kg = _integrate(runtime.fuel_expended_kg, fuel_rate_new, runtime.fuel_rate_kg_s, dt_s)
            runtime.fuel_rate_kg_s = fuel_rate_new
        ####
        runtime.mass_kg = definition.propulsion.initial_mass_kg - runtime.fuel_expended_kg
        runtime.fuel_mass_kg = definition.propulsion.initial_fuel_kg - runtime.fuel_expended_kg
        if runtime.fuel_mass_kg <= 0.0:
            mode = 0
        ####
    ####
    if mode == 0:
        runtime.fuel_rate_kg_s = 0.0
        thrust = 0.0
    ####
    runtime.values["mprop"] = mode
    runtime.values["throttle"] = throttle
    runtime.thrust_n = max(0.0, thrust)
    runtime.specific_impulse_s = max(0.0, spi)
    runtime.capture_area_factor = max(0.0, capture)


####


def _forces(definition: Ghame3SourceDefinition, runtime: _Ghame3Runtime, dynamic_pressure_pa: float) -> None:
    alpha = float(runtime.values["alphax"]) * RAD_PER_DEG
    bank = float(runtime.values["phimvx"]) * RAD_PER_DEG
    normal = dynamic_pressure_pa * definition.reference_area_m2 * runtime.lift_coefficient + runtime.thrust_n * math.sin(alpha)
    runtime.specific_force_velocity_mps2 = np.asarray(
        (
            (-dynamic_pressure_pa * definition.reference_area_m2 * runtime.drag_coefficient + runtime.thrust_n * math.cos(alpha)) / runtime.mass_kg,
            math.sin(bank) * normal / runtime.mass_kg,
            -math.cos(bank) * normal / runtime.mass_kg,
        ),
        dtype=np.float64,
    )


####


def _sample(time_s: float, runtime: _Ghame3Runtime, environment: CadacRound3Environment) -> Ghame3Sample:
    return Ghame3Sample(
        time_s=time_s,
        longitude_deg=runtime.round3.longitude_rad * DEG_PER_RAD,
        latitude_deg=runtime.round3.latitude_rad * DEG_PER_RAD,
        altitude_m=runtime.round3.altitude_m,
        speed_mps=runtime.round3.speed_mps,
        heading_deg=runtime.round3.heading_rad * DEG_PER_RAD,
        flight_path_deg=runtime.round3.flight_path_rad * DEG_PER_RAD,
        velocity_geographic_mps=_tuple3(runtime.round3.vbeg_mps),
        specific_force_velocity_mps2=_tuple3(runtime.specific_force_velocity_mps2),
        dynamic_pressure_pa=environment.dynamic_pressure_pa,
        mach=environment.mach,
        alpha_deg=float(runtime.values["alphax"]),
        bank_deg=float(runtime.values["phimvx"]),
        lift_to_drag=runtime.lift_to_drag,
        propulsion_mode=int(runtime.values["mprop"]),
        throttle=float(runtime.values["throttle"]),
        thrust_n=runtime.thrust_n,
        specific_impulse_s=runtime.specific_impulse_s,
        capture_area_factor=runtime.capture_area_factor,
        mass_kg=runtime.mass_kg,
        fuel_mass_kg=runtime.fuel_mass_kg,
    )


####


def _validate_events(initial_mode: int, events: tuple[CadacEventBlock, ...]) -> None:
    if initial_mode not in _SUPPORTED_PROPULSION_MODES:
        raise Ghame3SourceError(f"GHAME3 runtime supports propulsion modes {sorted(_SUPPORTED_PROPULSION_MODES)}; got {initial_mode}")
    ####
    for event in events:
        if event.condition.variable.casefold() not in _SUPPORTED_EVENT_VARIABLES:
            raise Ghame3SourceError(f"GHAME3 event at source line {event.source_line} watches unsupported runtime variable {event.condition.variable!r}")
        ####
        for assignment in event.assignments:
            key = assignment.name.casefold()
            if key not in _SUPPORTED_EVENT_VARIABLES:
                raise Ghame3SourceError(f"GHAME3 event at source line {event.source_line} mutates unsupported runtime variable {assignment.name!r}")
            ####
            if key == "mprop":
                value = int(assignment.value) if isinstance(assignment.value, (int, float)) else -999_999
                if value not in _SUPPORTED_PROPULSION_MODES:
                    raise Ghame3SourceError(f"GHAME3 event at source line {event.source_line} selects unsupported propulsion mode {assignment.value!r}")
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
            raise Ghame3SourceError(f"GHAME3 source vehicle is missing required parameter {name!r}") from None
        ####
        return float(default)
    ####
    if not isinstance(value, (int, float)):
        raise Ghame3SourceError(f"GHAME3 parameter {name!r} must be numeric")
    ####
    return float(value)


####


def _integer(vehicle: CadacVehicleBlock, name: str, default: int | None = None) -> int:
    value = _number(vehicle, name, None if default is None else float(default))
    if not value.is_integer():
        raise Ghame3SourceError(f"GHAME3 parameter {name!r} must be integer-valued")
    ####
    return int(value)


####


def _integrate(value: float, new_rate: float, old_rate: float, dt_s: float) -> float:
    return cadac_stored_derivative_step((value,), (new_rate,), (old_rate,), dt_s)[0]


####


def _tuple3(vector: FloatVector) -> tuple[float, float, float]:
    return (float(vector[0]), float(vector[1]), float(vector[2]))


####


def _runtime_is_finite(runtime: _Ghame3Runtime) -> bool:
    scalars = (
        runtime.round3.altitude_m,
        runtime.round3.speed_mps,
        runtime.mass_kg,
        runtime.thrust_n,
        runtime.specific_impulse_s,
    )
    arrays = (runtime.round3.vbeg_mps, runtime.round3.sbii_m, runtime.round3.vbii_mps, runtime.specific_force_velocity_mps2)
    return all(math.isfinite(value) for value in scalars) and all(np.all(np.isfinite(array)) for array in arrays)


####


__all__ = [
    "Ghame3EventTrace",
    "Ghame3PropulsionConfig",
    "Ghame3RunResult",
    "Ghame3Sample",
    "Ghame3SourceDefinition",
    "Ghame3SourceError",
    "load_ghame3_source_definition",
    "lower_ghame3_source_bundle",
    "run_ghame3_source_compatibility",
]
