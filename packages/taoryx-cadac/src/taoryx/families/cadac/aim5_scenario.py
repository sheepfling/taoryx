"""Multi-actor CADAC AIM5 compatibility scenario built on the AIM5 vertical slice."""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path
from typing import Literal

from pydantic import Field, model_validator

from .aim5 import (
    _AIM5_MODULES,
    _DEG_PER_RAD,
    Aim5EventTrace,
    Aim5ExecutionSemantics,
    Aim5Intercept,
    Aim5MissileConfig,
    Aim5ModuleTrace,
    Aim5Sample,
    Aim5SourceError,
    Aim5TargetConfig,
    _aim_aerodynamics,
    _aim_control,
    _aim_forces,
    _aim_guidance,
    _aim_intercept,
    _aim_propulsion,
    _aim_seeker,
    _AircraftState,
    _environment,
    _evaluate_aim_event,
    _evaluate_target_event,
    _event_trace,
    _initialize_missile,
    _initialize_target,
    _lower_missile,
    _lower_target,
    _module_trace_aim,
    _module_trace_target,
    _newton,
    _packet_for_target,
    _sample,
    _target_control,
    _target_forces,
    _target_guidance,
)
from .bundle import CadacSourceArtifact, CadacSourceBundle, load_cadac_source_bundle
from .deck import CadacDeck
from .events import CadacEventCursor
from .input_ast import CadacDeckKind, CadacEventBlock, CadacModel, CadacModuleStage


class Aim5MissileActorDefinition(CadacModel):
    """One source-ordered AIM5 actor and its vehicle-local resources."""

    object_id: str = Field(pattern=r"^m[1-9][0-9]*$")
    source_role: str = Field(min_length=1)
    source_line: int = Field(ge=1)
    config: Aim5MissileConfig
    aerodynamic_deck: CadacDeck
    propulsion_deck: CadacDeck
    events: tuple[CadacEventBlock, ...] = ()


####


class Aim5TargetActorDefinition(CadacModel):
    """One source-ordered AIRCRAFT3 actor."""

    object_id: str = Field(pattern=r"^a[1-9][0-9]*$")
    source_role: str = Field(min_length=1)
    source_line: int = Field(ge=1)
    config: Aim5TargetConfig
    events: tuple[CadacEventBlock, ...] = ()


####


class Aim5ScenarioVehicleRef(CadacModel):
    """One entry in the exact source vehicle loop order."""

    object_id: str = Field(min_length=2)
    model_name: Literal["AIM5", "AIRCRAFT3"]
    actor_index: int = Field(ge=0)


####


class Aim5ScenarioSourceDefinition(CadacModel):
    """Prepared multiple-engagement AIM5 case preserving source bus ordering."""

    schema_id: str = "taoryx.cadac.aim5-scenario-source/v0alpha1"
    source_name: str = Field(min_length=1)
    integration_step_s: float = Field(gt=0.0)
    plot_step_s: float | None = Field(default=None, gt=0.0)
    end_time_s: float = Field(gt=0.0)
    module_order: tuple[str, ...]
    vehicle_order: tuple[Aim5ScenarioVehicleRef, ...] = Field(min_length=2)
    missiles: tuple[Aim5MissileActorDefinition, ...] = Field(min_length=1)
    targets: tuple[Aim5TargetActorDefinition, ...] = Field(min_length=1)
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)

    @model_validator(mode="after")
    def validate_actor_graph(self) -> "Aim5ScenarioSourceDefinition":
        object_ids = tuple(item.object_id for item in self.vehicle_order)
        if len(object_ids) != len(set(object_ids)):
            raise ValueError("AIM5 scenario contains duplicate vehicle object IDs")
        ####
        expected = {item.object_id for item in self.missiles} | {item.object_id for item in self.targets}
        if set(object_ids) != expected:
            raise ValueError("AIM5 scenario vehicle order does not cover every actor exactly once")
        ####
        for missile in self.missiles:
            if missile.config.target_number > len(self.targets):
                raise ValueError(f"{missile.object_id} targets AIRCRAFT3 #{missile.config.target_number}, but only {len(self.targets)} targets exist")
            ####
        ####
        return self

    ####


####


class Aim5EngagementRun(CadacModel):
    """Trajectory and intercept state for one missile-to-target assignment."""

    missile_object_id: str = Field(pattern=r"^m[1-9][0-9]*$")
    target_object_id: str = Field(pattern=r"^a[1-9][0-9]*$")
    target_number: int = Field(ge=1)
    missile_alive: bool
    target_alive: bool
    intercept: Aim5Intercept | None = None
    samples: tuple[Aim5Sample, ...]


####


class Aim5TargetSample(CadacModel):
    """Independent AIRCRAFT3 truth sample emitted by the source scheduler."""

    time_s: float = Field(ge=0.0)
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]
    speed_mps: float = Field(ge=0.0)
    heading_deg: float
    flight_path_deg: float
    altitude_m: float
    alive: bool


####


class Aim5TargetRun(CadacModel):
    """Independent target trajectory retained beside missile engagement histories."""

    target_object_id: str = Field(pattern=r"^a[1-9][0-9]*$")
    alive: bool
    samples: tuple[Aim5TargetSample, ...] = Field(min_length=1)


####


class Aim5ScenarioRunResult(CadacModel):
    """Deterministic result for a source-ordered multiple AIM5 engagement."""

    schema_id: str = "taoryx.cadac.aim5-scenario-run/v0alpha1"
    source_name: str
    integration_step_s: float
    requested_end_time_s: float
    executed_steps: int = Field(ge=0)
    terminated_reason: Literal["all_missiles_terminated", "end_time"]
    execution_semantics: Aim5ExecutionSemantics
    source_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)
    engagements: tuple[Aim5EngagementRun, ...] = Field(min_length=1)
    targets: tuple[Aim5TargetRun, ...] = Field(min_length=1)
    module_trace: tuple[Aim5ModuleTrace, ...] = ()
    event_trace: tuple[Aim5EventTrace, ...] = ()


####


class Aim5ScenarioSession:
    """Persistent source-ordered AIM5/AIRCRAFT3 execution state.

    The session is deliberately narrower than the historical batch product:
    it owns the exact mutable actor, event-cursor, and communication-bus
    state needed for accepted source timesteps, while leaving sampling and
    result projection to its caller.  One call to :meth:`advance` holds the
    source-owned controller through an integral number of source timesteps;
    it never reinitializes hidden state between calls.
    """

    def __init__(self, definition: Aim5ScenarioSourceDefinition) -> None:
        self.definition = definition
        self.dt = definition.integration_step_s
        self.reset()
        ####

    def reset(self) -> None:
        """Reconstruct every mutable source state from the immutable definition."""

        self.missiles = [_initialize_missile(item.config) for item in self.definition.missiles]
        self.targets = [_initialize_target(item.config) for item in self.definition.targets]
        self.missile_alive = [True for _ in self.missiles]
        self.target_alive = [True for _ in self.targets]
        self.target_bus = [_packet_for_target(target, alive=True) for target in self.targets]
        self.missile_events = [CadacEventCursor.from_events(item.events) for item in self.definition.missiles]
        self.target_events = [CadacEventCursor.from_events(item.events) for item in self.definition.targets]
        self.intercepts: list[Aim5Intercept | None] = [None for _ in self.missiles]
        self.event_trace: list[Aim5EventTrace] = []
        self.sim_time = 0.0
        self.executed_steps = 0
        ####

    @property
    def completed(self) -> bool:
        """Return whether the terminal source condition has been reached."""

        return not any(self.missile_alive) or self.sim_time >= self.definition.end_time_s - 1.0e-12
        ####

    @property
    def primary_sample(self) -> Aim5Sample:
        """Project current accepted state for the installed single AIM5 actor."""

        aim = self.missiles[0]
        target_index = aim.config.target_number - 1
        return _sample(self.sim_time, aim, self.targets[target_index], self.target_bus[target_index])
        ####

    @property
    def primary_target_sample(self) -> Aim5TargetSample:
        """Project current accepted truth for the installed AIRCRAFT3 actor."""

        return _target_sample(self.sim_time, self.targets[0], alive=self.target_alive[0])
        ####

    def advance(self, duration_s: float) -> tuple[Aim5EventTrace, ...]:
        """Advance by an integral number of immutable source timesteps.

        CADAC's source controller and communication bus are updated at the
        source integration cadence.  Accepting arbitrary fractional holds
        would invent intermediate control/sensor ordering, so the session
        rejects them explicitly rather than silently rounding a user command.
        """

        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("AIM5 session duration_s must be positive and finite")
        steps = round(duration_s / self.dt)
        if steps <= 0 or not math.isclose(duration_s, steps * self.dt, rel_tol=0.0, abs_tol=max(1.0e-12, self.dt * 1.0e-9)):
            raise ValueError(f"AIM5 session duration_s must be an integral multiple of source step {self.dt:.17g} s")
        start_events = len(self.event_trace)
        for _ in range(steps):
            if self.completed:
                break
            ####
            self._advance_one()
        ####
        return tuple(self.event_trace[start_events:])
        ####

    def _advance_one(self) -> None:
        """Execute exactly one source vehicle-order pass and commit its state."""

        self.executed_steps += 1
        for vehicle in self.definition.vehicle_order:
            if vehicle.model_name == "AIM5":
                index = vehicle.actor_index
                if not self.missile_alive[index]:
                    continue
                ####
                aim = self.missiles[index]
                actor = self.definition.missiles[index]
                aim.flat.time_s = self.sim_time
                application = _evaluate_aim_event(self.missile_events[index], aim)
                if application is not None:
                    aim.flat.event_time_s = 0.0
                    self.event_trace.append(_event_trace(self.sim_time, "AIM5", application, object_id=actor.object_id))
                ####
                target_index = aim.config.target_number - 1
                for module in self.definition.module_order:
                    if module == "environment":
                        _environment(aim.flat)
                    elif module == "kinematics":
                        aim.flat.time_s = self.sim_time
                    elif module == "aerodynamics":
                        _aim_aerodynamics(aim, actor.aerodynamic_deck)
                    elif module == "propulsion":
                        _aim_propulsion(aim, actor.propulsion_deck)
                    elif module == "seeker":
                        _aim_seeker(aim, self.target_bus[target_index])
                    elif module == "guidance":
                        _aim_guidance(aim)
                    elif module == "control":
                        _aim_control(aim, self.dt)
                    elif module == "forces":
                        _aim_forces(aim)
                    elif module == "newton":
                        _newton(aim.flat, self.dt)
                    elif module == "intercept":
                        intercept = _aim_intercept(aim, self.target_bus[target_index], self.sim_time)
                        if intercept is not None:
                            self.intercepts[index] = intercept
                            self.missile_alive[index] = False
                            self.target_alive[target_index] = False
                            self.target_bus[target_index] = replace(self.target_bus[target_index], alive=False)
                        ####
                    ####
                ####
                aim.flat.event_time_s += self.dt
            else:
                index = vehicle.actor_index
                if not self.target_alive[index]:
                    continue
                ####
                target = self.targets[index]
                actor = self.definition.targets[index]
                target.flat.time_s = self.sim_time
                application = _evaluate_target_event(self.target_events[index], target)
                if application is not None:
                    target.flat.event_time_s = 0.0
                    self.event_trace.append(_event_trace(self.sim_time, "AIRCRAFT3", application, object_id=actor.object_id))
                ####
                for module in self.definition.module_order:
                    if module == "environment":
                        _environment(target.flat)
                    elif module == "kinematics":
                        target.flat.time_s = self.sim_time
                    elif module == "guidance":
                        _target_guidance(target)
                    elif module == "control":
                        _target_control(target, self.dt)
                    elif module == "forces":
                        _target_forces(target)
                    elif module == "newton":
                        _newton(target.flat, self.dt)
                    ####
                ####
                self.target_bus[index] = _packet_for_target(target, alive=self.target_alive[index])
                target.flat.event_time_s += self.dt
            ####
        ####
        self.sim_time = min(self.definition.end_time_s, self.sim_time + self.dt)
        ####

    ####


def lower_aim5_scenario_source_bundle(bundle: CadacSourceBundle) -> Aim5ScenarioSourceDefinition:
    """Lower every AIM5/AIRCRAFT3 actor in one source case without collapsing identities."""

    module_order = tuple(module.name.casefold() for module in bundle.case.modules if CadacModuleStage.EXECUTE in module.stages)
    unknown = tuple(name for name in module_order if name not in _AIM5_MODULES)
    if unknown:
        raise Aim5SourceError(f"AIM5 compatibility scenario does not implement source modules: {unknown!r}")
    ####
    missing = tuple(name for name in _AIM5_MODULES if name not in module_order)
    if missing:
        raise Aim5SourceError(f"AIM5 compatibility scenario requires source modules: {missing!r}")
    ####
    timing = bundle.case.timing_values
    try:
        integration_step_s = timing["int_step"]
    except KeyError as error:
        raise Aim5SourceError("AIM5 source case must declare TIMING int_step") from error
    ####

    missiles: list[Aim5MissileActorDefinition] = []
    targets: list[Aim5TargetActorDefinition] = []
    vehicle_order: list[Aim5ScenarioVehicleRef] = []
    for vehicle in bundle.case.vehicles:
        model = vehicle.model_name.casefold()
        if model == "aim5":
            index = len(missiles)
            object_id = f"m{index + 1}"
            try:
                aerodynamic_deck = bundle.deck_for("AIM5", CadacDeckKind.AERODYNAMIC, vehicle_role=vehicle.role)
                propulsion_deck = bundle.deck_for("AIM5", CadacDeckKind.PROPULSION, vehicle_role=vehicle.role)
            except KeyError as error:
                raise Aim5SourceError(f"{object_id} does not resolve one aerodynamic and propulsion deck") from error
            ####
            missiles.append(
                Aim5MissileActorDefinition(
                    object_id=object_id,
                    source_role=vehicle.role,
                    source_line=vehicle.source_line,
                    config=_lower_missile(vehicle),
                    aerodynamic_deck=aerodynamic_deck,
                    propulsion_deck=propulsion_deck,
                    events=vehicle.events,
                )
            )
            vehicle_order.append(Aim5ScenarioVehicleRef(object_id=object_id, model_name="AIM5", actor_index=index))
        elif model == "aircraft3":
            index = len(targets)
            object_id = f"a{index + 1}"
            targets.append(
                Aim5TargetActorDefinition(
                    object_id=object_id,
                    source_role=vehicle.role,
                    source_line=vehicle.source_line,
                    config=_lower_target(vehicle),
                    events=vehicle.events,
                )
            )
            vehicle_order.append(Aim5ScenarioVehicleRef(object_id=object_id, model_name="AIRCRAFT3", actor_index=index))
        else:
            raise Aim5SourceError(f"AIM5 compatibility scenario cannot execute vehicle model {vehicle.model_name!r}")
        ####
    ####
    if not missiles or not targets:
        raise Aim5SourceError("AIM5 compatibility scenario requires at least one AIM5 and one AIRCRAFT3")
    ####
    return Aim5ScenarioSourceDefinition(
        source_name=bundle.case.source_name,
        integration_step_s=integration_step_s,
        plot_step_s=timing.get("plot_step"),
        end_time_s=bundle.case.end_time_s,
        module_order=module_order,
        vehicle_order=tuple(vehicle_order),
        missiles=tuple(missiles),
        targets=tuple(targets),
        source_artifacts=bundle.artifacts,
    )


####


def load_aim5_scenario_source_definition(path: str | Path) -> Aim5ScenarioSourceDefinition:
    """Load a single- or multiple-engagement AIM5 source case."""

    return lower_aim5_scenario_source_bundle(load_cadac_source_bundle(path))


####


def run_aim5_scenario_source_compatibility(
    definition: Aim5ScenarioSourceDefinition,
    *,
    sample_step_s: float | None = None,
    trace_steps: int = 0,
) -> Aim5ScenarioRunResult:
    """Execute source-order multiple engagements with CADAC communication-bus lag."""

    dt = definition.integration_step_s
    requested_sample_step = sample_step_s if sample_step_s is not None else max(dt, 0.02)
    if not math.isfinite(requested_sample_step) or requested_sample_step <= 0.0:
        raise ValueError("sample_step_s must be positive and finite")
    ####
    if trace_steps < 0:
        raise ValueError("trace_steps must be nonnegative")
    ####

    missiles = [_initialize_missile(item.config) for item in definition.missiles]
    targets = [_initialize_target(item.config) for item in definition.targets]
    missile_alive = [True for _ in missiles]
    target_alive = [True for _ in targets]
    target_bus = [_packet_for_target(target, alive=True) for target in targets]
    missile_events = [CadacEventCursor.from_events(item.events) for item in definition.missiles]
    target_events = [CadacEventCursor.from_events(item.events) for item in definition.targets]
    intercepts: list[Aim5Intercept | None] = [None for _ in missiles]
    samples: list[list[Aim5Sample]] = [[] for _ in missiles]
    target_samples: list[list[Aim5TargetSample]] = [[] for _ in targets]
    next_sample_time = [0.0 for _ in missiles]
    next_target_sample_time = [0.0 for _ in targets]
    module_trace: list[Aim5ModuleTrace] = []
    event_trace: list[Aim5EventTrace] = []
    sim_time = 0.0
    steps = 0

    while sim_time <= definition.end_time_s + dt and any(missile_alive):
        steps += 1
        for vehicle in definition.vehicle_order:
            if vehicle.model_name == "AIM5":
                index = vehicle.actor_index
                if not missile_alive[index]:
                    continue
                ####
                aim = missiles[index]
                actor = definition.missiles[index]
                aim.flat.time_s = sim_time
                application = _evaluate_aim_event(missile_events[index], aim)
                if application is not None:
                    aim.flat.event_time_s = 0.0
                    event_trace.append(_event_trace(sim_time, "AIM5", application, object_id=actor.object_id))
                ####
                target_index = aim.config.target_number - 1
                for module in definition.module_order:
                    if module == "environment":
                        _environment(aim.flat)
                    elif module == "kinematics":
                        aim.flat.time_s = sim_time
                    elif module == "aerodynamics":
                        _aim_aerodynamics(aim, actor.aerodynamic_deck)
                    elif module == "propulsion":
                        _aim_propulsion(aim, actor.propulsion_deck)
                    elif module == "seeker":
                        _aim_seeker(aim, target_bus[target_index])
                    elif module == "guidance":
                        _aim_guidance(aim)
                    elif module == "control":
                        _aim_control(aim, dt)
                    elif module == "forces":
                        _aim_forces(aim)
                    elif module == "newton":
                        _newton(aim.flat, dt)
                    elif module == "intercept":
                        intercept = _aim_intercept(aim, target_bus[target_index], sim_time)
                        if intercept is not None:
                            intercepts[index] = intercept
                            missile_alive[index] = False
                            target_alive[target_index] = False
                            target_bus[target_index] = replace(target_bus[target_index], alive=False)
                            _append_target_sample_if_new(
                                target_samples[target_index],
                                sim_time,
                                targets[target_index],
                                alive=False,
                            )
                        ####
                    ####
                    if steps <= trace_steps:
                        module_trace.append(_module_trace_aim(sim_time, module, aim, object_id=actor.object_id))
                    ####
                ####
                if sim_time + 0.5 * dt >= next_sample_time[index] or intercepts[index] is not None:
                    samples[index].append(_sample(sim_time, aim, targets[target_index], target_bus[target_index]))
                    while next_sample_time[index] <= sim_time + 0.5 * dt:
                        next_sample_time[index] += requested_sample_step
                    ####
                ####
                aim.flat.event_time_s += dt
            else:
                index = vehicle.actor_index
                if not target_alive[index]:
                    continue
                ####
                target = targets[index]
                actor = definition.targets[index]
                target.flat.time_s = sim_time
                application = _evaluate_target_event(target_events[index], target)
                if application is not None:
                    target.flat.event_time_s = 0.0
                    event_trace.append(_event_trace(sim_time, "AIRCRAFT3", application, object_id=actor.object_id))
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
                        module_trace.append(_module_trace_target(sim_time, module, target, object_id=actor.object_id))
                    ####
                ####
                target_bus[index] = _packet_for_target(target, alive=target_alive[index])
                if sim_time + 0.5 * dt >= next_target_sample_time[index]:
                    _append_target_sample_if_new(
                        target_samples[index],
                        sim_time,
                        target,
                        alive=target_alive[index],
                    )
                    while next_target_sample_time[index] <= sim_time + 0.5 * dt:
                        next_target_sample_time[index] += requested_sample_step
                    ####
                ####
                target.flat.event_time_s += dt
            ####
        ####
        sim_time += dt
    ####

    for index, aim in enumerate(missiles):
        if intercepts[index] is not None:
            continue
        ####
        terminal_time = min(sim_time, definition.end_time_s)
        if samples[index] and samples[index][-1].time_s >= terminal_time - 0.5 * dt:
            continue
        ####
        target_index = aim.config.target_number - 1
        samples[index].append(_sample(terminal_time, aim, targets[target_index], target_bus[target_index]))
    ####

    terminal_time = min(sim_time, definition.end_time_s)
    for index, target in enumerate(targets):
        if not target_samples[index]:
            _append_target_sample_if_new(
                target_samples[index],
                0.0,
                target,
                alive=target_alive[index],
            )
        ####
        if target_samples[index][-1].time_s < terminal_time - 0.5 * dt and target_alive[index]:
            _append_target_sample_if_new(
                target_samples[index],
                terminal_time,
                target,
                alive=True,
            )
        ####
    ####

    target_runs = tuple(
        Aim5TargetRun(
            target_object_id=actor.object_id,
            alive=target_alive[index],
            samples=tuple(target_samples[index]),
        )
        for index, actor in enumerate(definition.targets)
    )

    engagements = tuple(
        Aim5EngagementRun(
            missile_object_id=actor.object_id,
            target_object_id=definition.targets[actor.config.target_number - 1].object_id,
            target_number=actor.config.target_number,
            missile_alive=missile_alive[index],
            target_alive=target_alive[actor.config.target_number - 1],
            intercept=intercepts[index],
            samples=tuple(samples[index]),
        )
        for index, actor in enumerate(definition.missiles)
    )
    return Aim5ScenarioRunResult(
        source_name=definition.source_name,
        integration_step_s=dt,
        requested_end_time_s=definition.end_time_s,
        executed_steps=steps,
        terminated_reason="all_missiles_terminated" if not any(missile_alive) else "end_time",
        execution_semantics=Aim5ExecutionSemantics(
            module_order=definition.module_order,
            vehicle_order=tuple(item.model_name for item in definition.vehicle_order),
        ),
        source_artifacts=definition.source_artifacts,
        engagements=engagements,
        targets=target_runs,
        module_trace=tuple(module_trace),
        event_trace=tuple(event_trace),
    )


####


def _target_sample(time_s: float, target: _AircraftState, *, alive: bool) -> Aim5TargetSample:
    """Project one internal AIRCRAFT3 state into a portable truth sample."""

    flat = target.flat
    return Aim5TargetSample(
        time_s=time_s,
        position_ned_m=tuple(float(value) for value in flat.position_ned_m),
        velocity_ned_mps=tuple(float(value) for value in flat.velocity_ned_mps),
        speed_mps=float(flat.speed_mps),
        heading_deg=float(flat.heading_rad * _DEG_PER_RAD),
        flight_path_deg=float(flat.flight_path_rad * _DEG_PER_RAD),
        altitude_m=float(flat.altitude_m),
        alive=alive,
    )


####


def _append_target_sample_if_new(
    samples: list[Aim5TargetSample],
    time_s: float,
    target: _AircraftState,
    *,
    alive: bool,
) -> None:
    """Append one target sample while preserving unique accepted timestamps."""

    sample = _target_sample(time_s, target, alive=alive)
    if samples and abs(samples[-1].time_s - time_s) <= 1.0e-12:
        samples[-1] = sample
        return
    ####
    samples.append(sample)


####


__all__ = [
    "Aim5EngagementRun",
    "Aim5MissileActorDefinition",
    "Aim5ScenarioSession",
    "Aim5ScenarioRunResult",
    "Aim5ScenarioSourceDefinition",
    "Aim5ScenarioVehicleRef",
    "Aim5TargetActorDefinition",
    "Aim5TargetRun",
    "Aim5TargetSample",
    "load_aim5_scenario_source_definition",
    "lower_aim5_scenario_source_bundle",
    "run_aim5_scenario_source_compatibility",
]
