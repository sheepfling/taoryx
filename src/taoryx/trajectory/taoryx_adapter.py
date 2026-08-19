"""Taoryx provider adapter for the Alpha 2 point-mass session proof."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

from taoryx.modes import DynamicsMode
from taoryx.runtime import ControlSpec, InteractiveSession, RuntimeProblem, RuntimeState, RuntimeVehicle

from .authority import ControlArbitrator
from .contracts import ResolvedCase
from .providers import (
    CompiledCase,
    ControlFrame,
    ProviderCapabilities,
    ProviderSession,
    SessionState,
    StepResult,
    TrajectoryResult,
    TranslationEntry,
    TranslationReport,
    _case_number,
    reproject_standard_ecef_history,
)


def _runtime_problem(case: ResolvedCase) -> RuntimeProblem:
    """Construct the native runtime graph for the minimal point-mass adapter."""

    initial_speed = _case_number(case, "mission.initial_speed", 0.0)
    initial_altitude = _case_number(case, "mission.initial_altitude", 0.0)
    mass = _case_number(case, "vehicle.mass.initial", 1.0)
    thrust = _case_number(case, "vehicle.booster.thrust", 0.0)
    state = RuntimeState(
        0.0,
        (0.0, initial_speed),
        value_names=("position.downrange_m", "velocity.m_s"),
        named={"position.altitude_m": initial_altitude, "mass_kg": mass},
    )

    def derivative(current: RuntimeState) -> tuple[float, float]:
        throttle = float(current.named.get("command.throttle", 0.0))
        acceleration = throttle * thrust / mass
        speed = float(current.named.get("velocity.m_s", current.values[1]))
        return speed, acceleration
        ####

    vehicle = RuntimeVehicle(
        "vehicle",
        state,
        derivative=derivative,
        step_size=_case_number(case, "runtime.time_step", 0.05),
        integrator="rk4",
        dynamics_mode=DynamicsMode.POINT_MASS,
    )
    return RuntimeProblem({vehicle.name: vehicle}, final_time=_case_number(case, "mission.duration", 1.0))
    ####


class TaoryxPointMassAdapter:
    """Registered Taoryx provider over the canonical interactive runtime."""

    def __init__(self) -> None:
        self._capabilities = ProviderCapabilities(
            provider_id="taoryx.native",
            provider_kind="taoryx-adapter",
            families=("simple_aero",),
            fidelities=("point_mass_3dof",),
            supports_batch=True,
            supports_interactive=True,
            statuses={"batch": "native", "interactive": "native", "moments": "unsupported"},
        )
        ####

    @property
    def capabilities(self) -> ProviderCapabilities:
        """Return the adapter's negotiated capabilities."""

        return self._capabilities
        ####

    def compile(self, case: ResolvedCase) -> CompiledCase:
        """Compile the supported family into a native runtime graph."""

        if case.family not in self.capabilities.families:
            raise ValueError(f"Taoryx adapter does not support family {case.family!r}")
        if case.fidelity not in self.capabilities.fidelities:
            raise ValueError(f"Taoryx adapter does not support fidelity {case.fidelity!r}")
        report = TranslationReport(
            self.capabilities.provider_id,
            (
                TranslationEntry("family", case.family, "native"),
                TranslationEntry("fidelity", case.fidelity, "native"),
                TranslationEntry("command.throttle", "command.throttle", "native"),
                TranslationEntry("command.bank", "command.bank", "approximated", "retained in the fixed schema; vertical proof does not steer it"),
                TranslationEntry("rigid_body_moments", None, "unsupported", "point-mass adapter"),
            ),
        )
        return CompiledCase(self.capabilities.provider_id, case, report)
        ####

    def new_session(self, compiled: CompiledCase) -> ProviderSession:
        """Create a resettable native runtime session."""

        return _TaoryxSession(compiled)
        ####


class _TaoryxSession:
    """Normalized wrapper around :class:`InteractiveSession`."""

    def __init__(self, compiled: CompiledCase) -> None:
        self.compiled = compiled
        self._session: InteractiveSession
        self._history: list[SessionState]
        self._controls: list[Mapping[str, float]]
        self._requested_controls: list[Mapping[str, float]]
        self._resources: list[Mapping[str, float]]
        self._diagnostics: list[str]
        self._arbitrator = ControlArbitrator(compiled.case.controls)
        self._build()
        ####

    def _build(self) -> None:
        self._arbitrator.reset()
        self._session = InteractiveSession(
            _runtime_problem(self.compiled.case),
            controls=(
                ControlSpec("command.throttle", unit="dimensionless", default=0.0, lower=0.0, upper=1.0),
                ControlSpec("command.bank", unit="deg", default=0.0, lower=-90.0, upper=90.0),
            ),
        )
        self._history = [self._state_from_runtime()]
        self._controls = []
        self._requested_controls = []
        self._resources = []
        self._diagnostics = []
        ####

    def _state_from_runtime(self) -> SessionState:
        state = self._session.problem.vehicles["vehicle"].state
        return SessionState(
            state.time,
            {
                "position.downrange_m": float(state.values[0]),
                "position.altitude_m": float(state.named.get("position.altitude_m", 0.0)),
                "velocity.m_s": float(state.values[1]),
            },
        )
        ####

    def reset(self) -> SessionState:
        """Reset the native session and discard prior history."""

        self._build()
        return self._history[0]
        ####

    def step(self, duration_s: float, controls: ControlFrame | None = None) -> StepResult:
        """Advance one native runtime transition and normalize its result."""

        frame = controls or ControlFrame()
        before = self._session.time
        arbitration = self._arbitrator.apply(before, duration_s, frame)
        snapshot = self._session.step(duration_s, arbitration.values)
        state = self._state_from_runtime()
        self._history.append(state)
        self._history = list(reproject_standard_ecef_history(self._history))
        state = self._history[-1]
        self._controls.append(dict(arbitration.values))
        requested = {**frame.values, **frame.rates}
        self._requested_controls.append(requested)
        self._resources.append({})
        diagnostics = arbitration.diagnostics + snapshot.diagnostics
        self._diagnostics.extend(diagnostics)
        return StepResult(
            before,
            snapshot.time_end,
            state,
            dict(arbitration.values),
            snapshot.events,
            diagnostics,
            tuple(decision.to_dict() for decision in arbitration.decisions),
            requested_controls=requested,
            achieved_controls=dict(arbitration.values),
        )
        ####

    def run_to_completion(self, controls: Sequence[ControlFrame] = ()) -> TrajectoryResult:
        """Run the native session using the same public step transition."""

        self.reset()
        duration = _case_number(self.compiled.case, "mission.duration", 1.0)
        dt = _case_number(self.compiled.case, "runtime.time_step", duration)
        frames = tuple(controls)
        index = 0
        while self._session.time < duration - 1e-12:
            frame = frames[index] if index < len(frames) else ControlFrame()
            self.step(min(dt, duration - self._session.time), frame)
            index += 1
        return TrajectoryResult(
            self.compiled.provider_id,
            self.compiled.case.case_id,
            "completed",
            tuple(self._history),
            tuple(self._controls),
            diagnostics=tuple(self._diagnostics),
            requested_controls=tuple(self._requested_controls),
            resource_observations=tuple(self._resources),
        )
        ####


__all__ = ["TaoryxPointMassAdapter"]
####
