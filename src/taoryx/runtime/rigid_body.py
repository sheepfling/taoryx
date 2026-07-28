"""Runtime adapters for the TAORYX rigid-body 6-DOF extension."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from enum import StrEnum

from taoryx.contracts import Frame, Vector3
from taoryx.modes import DynamicsMode
from taoryx.rigid_body import RIGID_BODY_STATE_NAMES, RigidBody6DofModel, RigidBody6DofState, RigidBodyForceMoment, ThermalLimits

from .common import RuntimeState, RuntimeVehicle
from .truth import rigid_body_truth_provider


class FlightPhase(StrEnum):
    """Phase labels for a staged ascent, coast, entry, and terminal mission."""

    STAGE_1_POWERED = "stage-1-powered"
    STAGE_1_SEPARATION = "stage-1-separation"
    STAGE_2_POWERED = "stage-2-powered"
    COAST_TO_APOGEE = "coast-to-apogee"
    ENTRY_THERMAL_CONTROL = "entry-thermal-control"
    TERMINAL_GUIDANCE = "terminal-guidance"
    IMPACT = "impact"
####


@dataclass(frozen=True, slots=True)
class PhaseTransition:
    """A named event boundary and the phase it activates."""

    source: FlightPhase
    target: FlightPhase
    event: str


@dataclass(slots=True)
class FlightPhaseMachine:
    """Validate event-driven transitions for a staged reentry mission."""

    phase: FlightPhase
    transitions: tuple[PhaseTransition, ...]

    def transition(self, event: str) -> FlightPhase:
        """Apply one named event or reject an out-of-order phase change."""

        match = next((item for item in self.transitions if item.source is self.phase and item.event == event), None)
        if match is None:
            raise ValueError(f"event {event!r} is invalid in phase {self.phase.value}")
        self.phase = match.target
        return self.phase
        ####
    ####


@dataclass(frozen=True, slots=True)
class ThermalControlCommand:
    """Bounded entry command produced by the thermal protection layer."""

    angle_of_attack_radians: float
    bank_radians: float
    limited: bool


@dataclass(frozen=True, slots=True)
class AttitudeControlOutput:
    """Bounded body moment and saturation state for one attitude command."""

    moment_body: Vector3
    saturated: bool


def bounded_attitude_moment(
    attitude_error: Vector3,
    body_rate: Vector3,
    *,
    attitude_gain: float,
    rate_damping: float,
    maximum_moment: float | None = None,
    maximum_body_rate: float | None = None,
) -> AttitudeControlOutput:
    """Apply proportional attitude and rate feedback with a moment limit."""

    if attitude_gain < 0.0 or rate_damping < 0.0:
        raise ValueError("attitude gain and rate damping must be nonnegative")
    if maximum_moment is not None and (not math.isfinite(maximum_moment) or maximum_moment <= 0.0):
        raise ValueError("maximum moment must be positive and finite")
    if maximum_body_rate is not None and (not math.isfinite(maximum_body_rate) or maximum_body_rate <= 0.0):
        raise ValueError("maximum body rate must be positive and finite")
    requested = attitude_error.scaled(attitude_gain) - body_rate.scaled(rate_damping)
    if maximum_body_rate is not None and body_rate.norm() > maximum_body_rate:
        requested = body_rate.scaled(-rate_damping)
        if maximum_moment is None or requested.norm() <= maximum_moment:
            return AttitudeControlOutput(requested, True)
    if maximum_moment is None or requested.norm() <= maximum_moment:
        return AttitudeControlOutput(requested, False)
    return AttitudeControlOutput(requested.scaled(maximum_moment / requested.norm()), True)
    ####
####


@dataclass(frozen=True, slots=True)
class ThermalEntryController:
    """Reduce commanded entry angle when heat-rate or heat-load margins close."""

    limits: ThermalLimits
    minimum_angle_of_attack_radians: float = 0.0
    maximum_angle_of_attack_radians: float = 0.35

    def command(self, requested_angle_of_attack_radians: float, heat_rate: float, heat_load: float) -> ThermalControlCommand:
        """Return a bounded command and identify whether thermal limiting acted."""

        requested = min(self.maximum_angle_of_attack_radians, max(self.minimum_angle_of_attack_radians, requested_angle_of_attack_radians))
        rate_fraction = heat_rate / self.limits.maximum_heat_rate
        load_fraction = heat_load / self.limits.maximum_heat_load
        limiting_fraction = max(rate_fraction, load_fraction)
        if limiting_fraction <= 0.8:
            return ThermalControlCommand(requested, 0.0, False)
        scale = max(0.0, min(1.0, (1.0 - limiting_fraction) / 0.2))
        return ThermalControlCommand(self.minimum_angle_of_attack_radians + scale * (requested - self.minimum_angle_of_attack_radians), 0.0, True)
        ####
    ####


RigidBodyLoadComponent = Callable[[RigidBody6DofState], RigidBodyForceMoment]


@dataclass(frozen=True, slots=True)
class RigidBodyLoadPipeline:
    """Compose propulsion, environment, aerodynamics, and control loads.

    Each component returns body-frame force and moment plus optional mass-flow
    and heat-rate contributions. Components are deliberately additive so a
    showcase can use a synthetic control input while the production path
    later supplies table-driven aerodynamic loads through the same seam.
    """

    components: tuple[RigidBodyLoadComponent, ...]

    def evaluate(self, state: RigidBody6DofState) -> RigidBodyForceMoment:
        """Sum all active load components for one rigid-body state."""

        force = Vector3(0.0, 0.0, 0.0)
        moment = Vector3(0.0, 0.0, 0.0)
        propellant_mass_rate = 0.0
        heat_rate = 0.0
        for component in self.components:
            load = component(state)
            force = force + load.force_body
            moment = moment + load.moment_body
            propellant_mass_rate += load.propellant_mass_rate
            heat_rate += load.heat_rate
        return RigidBodyForceMoment(force, moment, propellant_mass_rate, heat_rate)
        ####
    ####


def runtime_state(state: RigidBody6DofState) -> RuntimeState:
    """Pack a rigid-body state into the generic runtime contract."""

    values = state.to_values()
    named = dict(zip(RIGID_BODY_STATE_NAMES, values, strict=True))
    named["time"] = state.time
    return RuntimeState(state.time, values, frame=Frame.ECIC, value_names=RIGID_BODY_STATE_NAMES, named=named)
    ####


def rigid_body_vehicle(
    name: str,
    state: RigidBody6DofState,
    model: RigidBody6DofModel,
    *,
    step_size: float = 0.01,
    integrator: str = "rk4",
) -> RuntimeVehicle:
    """Create a generic runtime vehicle backed by the rigid-body model."""

    def observables(values: Mapping[str, float]) -> dict[str, float]:
        runtime_time = float(values.get("time", state.time))
        runtime_state = RigidBody6DofState.from_values(runtime_time, tuple(values[name] for name in RIGID_BODY_STATE_NAMES))
        return model.observables(runtime_state)
    ####

    return RuntimeVehicle(
        name,
        runtime_state(state),
        derivative=model.runtime_derivative(),
        step_size=step_size,
        integrator=integrator,
        dynamics_mode=DynamicsMode.RIGID_BODY_6DOF,
        environment_evaluator=observables,
        truth_provider=rigid_body_truth_provider(model),
    )
    ####
