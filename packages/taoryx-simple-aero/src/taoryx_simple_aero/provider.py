"""Analytical point-mass reference provider extracted from the Taoryx host."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

from taoryx.trajectory.authority import ControlArbitrator
from taoryx.trajectory.contracts import ControlFrame, ControlSchema, ResolvedCase
from taoryx.trajectory.providers import (
    CompiledCase,
    ProviderCapabilities,
    ProviderSession,
    SessionState,
    StepResult,
    TrajectoryResult,
    TranslationEntry,
    TranslationReport,
)


def _case_number(case: ResolvedCase, parameter_id: str, fallback: float) -> float:
    """Read one numeric resolved parameter with an explicit fallback."""

    value = case.parameters.get(parameter_id)
    if value is None or not isinstance(value.value, (int, float)):
        return fallback
    return float(value.value)
    ####


class ReferencePointMassProvider:
    """Non-Taoryx constant-acceleration provider for host conformance tests."""

    def __init__(self) -> None:
        self._capabilities = ProviderCapabilities(
            provider_id="reference.point_mass",
            provider_kind="analytical",
            families=("simple_aero",),
            fidelities=("point_mass_3dof",),
            supports_batch=True,
            supports_interactive=True,
            statuses={"batch": "native", "interactive": "native", "moments": "unsupported"},
        )
        ####

    @property
    def capabilities(self) -> ProviderCapabilities:
        """Return the reference provider capabilities."""

        return self._capabilities
        ####

    def compile(self, case: ResolvedCase) -> CompiledCase:
        """Compile only the simple point-mass contract."""

        if case.family not in self.capabilities.families:
            raise ValueError(f"reference provider does not support family {case.family!r}")
        if case.fidelity not in self.capabilities.fidelities:
            raise ValueError(f"reference provider does not support fidelity {case.fidelity!r}")
        entries = (
            TranslationEntry("family", case.family, "native"),
            TranslationEntry("fidelity", case.fidelity, "native"),
            TranslationEntry("vehicle.mass.initial", "constant-mass", "approximated", "mass is fixed for the reference fixture"),
            TranslationEntry("rigid_body_moments", None, "unsupported", "point-mass reference provider"),
        )
        report = TranslationReport(self.capabilities.provider_id, entries)
        return CompiledCase(self.capabilities.provider_id, case, report)
        ####

    def new_session(self, compiled: CompiledCase) -> ProviderSession:
        """Create a reference session after retaining translation limits."""

        return _ReferenceSession(compiled)
        ####

    ####


class ReferencePointMassSession:
    """Public parameter-level session around the existing point-mass kernel."""

    def __init__(
        self,
        *,
        initial_speed_m_s: float,
        initial_altitude_m: float,
        mass_kg: float,
        thrust_n: float,
        controls: tuple[ControlSchema, ...],
        provider_id: str = "reference.point_mass",
        case_id: str = "simple-aero-session",
        duration_s: float = 1.0,
        time_step_s: float = 0.1,
    ) -> None:
        if mass_kg <= 0.0:
            raise ValueError("point-mass session mass_kg must be positive")
        if initial_speed_m_s < 0.0 or initial_altitude_m < 0.0 or thrust_n < 0.0:
            raise ValueError("point-mass session speed, altitude, and thrust must be nonnegative")
        if duration_s <= 0.0 or time_step_s <= 0.0:
            raise ValueError("point-mass session duration and time step must be positive")
        self._initial_speed_m_s = float(initial_speed_m_s)
        self._initial_altitude_m = float(initial_altitude_m)
        self._mass_kg = float(mass_kg)
        self._thrust_n = float(thrust_n)
        self._provider_id = provider_id
        self._case_id = case_id
        self._duration_s = float(duration_s)
        self._time_step_s = float(time_step_s)
        self._arbitrator = ControlArbitrator(controls)
        self._reset_values()
        ####

    def _reset_values(self) -> None:
        """Initialize mutable state without replacing the session object."""

        self._time = 0.0
        self._downrange = 0.0
        self._speed = self._initial_speed_m_s
        self._altitude = self._initial_altitude_m
        self._arbitrator.reset()
        self._history: list[SessionState] = [self._state()]
        self._controls: list[Mapping[str, float]] = []
        self._requested_controls: list[Mapping[str, float]] = []
        self._resources: list[Mapping[str, float]] = []
        self._diagnostics: list[str] = []
        ####

    def _state(self) -> SessionState:
        """Return the normalized current state."""

        return SessionState(
            self._time,
            {
                "position.downrange_m": self._downrange,
                "position.altitude_m": self._altitude,
                "velocity.m_s": self._speed,
            },
        )
        ####

    def reset(self) -> SessionState:
        """Reset the reference state and command history."""

        self._reset_values()
        return self._state()
        ####

    def observe(self) -> SessionState:
        """Return the accepted native point-mass boundary without advancing."""

        return self._state()
        ####

    def restore_state(
        self,
        *,
        time_s: float,
        downrange_m: float,
        altitude_m: float,
        speed_m_s: float,
    ) -> SessionState:
        """Restore a committed boundary for checkpoint-backed wrappers."""

        if min(time_s, downrange_m, altitude_m, speed_m_s) < 0.0:
            raise ValueError("point-mass restored state values must be nonnegative")
        self._time = float(time_s)
        self._downrange = float(downrange_m)
        self._altitude = float(altitude_m)
        self._speed = float(speed_m_s)
        self._arbitrator.reset()
        self._history = [self._state()]
        self._controls = []
        self._requested_controls = []
        self._resources = []
        self._diagnostics = []
        return self._state()
        ####

    def step(self, duration_s: float, controls: ControlFrame | None = None) -> StepResult:
        """Advance with exact constant-acceleration kinematics."""

        if duration_s <= 0.0:
            raise ValueError("provider step duration must be positive")
        frame = controls or ControlFrame()
        arbitration = self._arbitrator.apply(self._time, duration_s, frame)
        applied = arbitration.values
        throttle = float(applied.get("command.throttle", 0.0))
        acceleration = throttle * self._thrust_n / self._mass_kg
        start = self._time
        self._downrange += self._speed * duration_s + 0.5 * acceleration * duration_s * duration_s
        self._speed += acceleration * duration_s
        self._time += duration_s
        state = self._state()
        self._history.append(state)
        self._controls.append(dict(applied))
        requested = {**frame.values, **frame.rates}
        self._requested_controls.append(requested)
        self._resources.append({})
        self._diagnostics.extend(arbitration.diagnostics)
        return StepResult(
            start,
            self._time,
            state,
            dict(applied),
            diagnostics=arbitration.diagnostics,
            control_decisions=tuple(decision.to_dict() for decision in arbitration.decisions),
            requested_controls=requested,
            achieved_controls=dict(applied),
        )
        ####

    def run_to_completion(self, controls: Sequence[ControlFrame] = ()) -> TrajectoryResult:
        """Run for the configured duration using the supplied frames."""

        self.reset()
        duration = self._duration_s
        dt = self._time_step_s
        frames = tuple(controls)
        index = 0
        while self._time < duration - 1e-12:
            step = min(dt, duration - self._time)
            frame = frames[index] if index < len(frames) else ControlFrame()
            self.step(step, frame)
            index += 1
        return TrajectoryResult(
            self._provider_id,
            self._case_id,
            "completed",
            tuple(self._history),
            tuple(self._controls),
            diagnostics=tuple(self._diagnostics),
            requested_controls=tuple(self._requested_controls),
            resource_observations=tuple(self._resources),
        )
        ####

    ####


class _ReferenceSession(ReferencePointMassSession):
    """Compatibility adapter from the older compiled-case provider API."""

    def __init__(self, compiled: CompiledCase) -> None:
        self.compiled = compiled
        self.case = compiled.case
        super().__init__(
            initial_speed_m_s=_case_number(self.case, "mission.initial_speed", 0.0),
            initial_altitude_m=_case_number(self.case, "mission.initial_altitude", 0.0),
            mass_kg=_case_number(self.case, "vehicle.mass.initial", 1.0),
            thrust_n=_case_number(self.case, "vehicle.booster.thrust", 0.0),
            controls=self.case.controls,
            provider_id=compiled.provider_id,
            case_id=self.case.case_id,
            duration_s=_case_number(self.case, "mission.duration", 1.0),
            time_step_s=_case_number(self.case, "runtime.time_step", 1.0),
        )
        ####

    ####


__all__ = ["ReferencePointMassProvider", "ReferencePointMassSession"]
####
