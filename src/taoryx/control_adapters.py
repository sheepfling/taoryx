"""Vehicle-family adapters for the generic control contract."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from .contracts import Vector3
from .control import ControlCommand, ControlDemand, PlantEvaluation, VehicleObservation
from .rigid_body import RigidBody6DofState, RigidBodyForceMoment
from .rotorcraft import QuadRotorAllocation

RigidStateFactory = Callable[[VehicleObservation], RigidBody6DofState]
RigidLoadProvider = Callable[[RigidBody6DofState], RigidBodyForceMoment]
CommandedRigidLoadProvider = Callable[[RigidBody6DofState, ControlCommand], RigidBodyForceMoment]


@dataclass(frozen=True, slots=True)
class FunctionalPlantAdapter:
    """Adapt a point-mass or kinematic evaluator to ``VehiclePlant``."""

    evaluator: Callable[[VehicleObservation, ControlCommand], PlantEvaluation]

    def evaluate(self, observation: VehicleObservation, command: ControlCommand) -> PlantEvaluation:
        """Evaluate the supplied family-independent plant function."""

        return self.evaluator(observation, command)
        ####
    ####


@dataclass(frozen=True, slots=True)
class RigidBodyPlantAdapter:
    """Adapt an existing Newton--Euler load evaluator to ``VehiclePlant``.

    The state factory and load provider are where B747, X8, Hummingbird, and
    X-15 semantics enter.  The controller and evidence layers see only the
    common observation, command, force, moment, and mass-flow contract.
    """

    state_factory: RigidStateFactory
    load_provider: RigidLoadProvider
    commanded_load_provider: CommandedRigidLoadProvider | None = None

    def evaluate(self, observation: VehicleObservation, command: ControlCommand) -> PlantEvaluation:
        """Resolve a rigid-body state and return the canonical plant result."""

        state = self.state_factory(observation)
        load = (
            self.commanded_load_provider(state, command)
            if self.commanded_load_provider is not None
            else self.load_provider(state)
        )
        diagnostics = {
            "heat_rate": load.heat_rate,
            "has_aero_force": load.aero_force_body is not None,
            "has_propulsion_force": load.propulsion_force_body is not None,
        }
        return PlantEvaluation(load.force_body, load.moment_body, load.propellant_mass_rate, diagnostics)
        ####
    ####


@dataclass(frozen=True, slots=True)
class QuadRotorControlAllocator:
    """Map generic collective/moment demands into four rotor-speed commands."""

    allocation: QuadRotorAllocation

    def allocate(self, demand: ControlDemand, observation: VehicleObservation) -> ControlCommand:
        """Allocate ``collective_speed_rad_s`` and body moments in source axes."""

        del observation
        collective = demand.values.get("collective_speed_rad_s", demand.values.get("rotor_speed"))
        if collective is None:
            raise KeyError("rotor allocator requires collective_speed_rad_s or rotor_speed")
        moment = tuple(demand.values.get(name, 0.0) for name in ("moment_x_nm", "moment_y_nm", "moment_z_nm"))
        commands = self.allocation.allocate(float(collective), Vector3(*moment))
        values = {
            f"rotor-{index}-speed": speed
            for index, speed in enumerate(commands.values, start=1)
        }
        return ControlCommand(values, ("rotor-allocation",) if demand.saturated else (), demand.source)
        ####
    ####
