"""Reusable phase-window checks for long vehicle validation scenarios."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Literal, Mapping, Sequence

Direction = Literal["increasing", "decreasing"]


@dataclass(frozen=True, slots=True)
class ContinuityViolation:
    """One state jump that is not covered by a declared discrete event."""

    channel: str
    left_time_s: float
    right_time_s: float
    delta: float
    tolerance: float
    event_time_s: float | None = None
####


@dataclass(frozen=True, slots=True)
class DeclaredDiscontinuity:
    """One event boundary and the channels it is allowed to change."""

    event_id: str
    time_s: float
    allowed_channels: tuple[str, ...] = ()
    reason: str = ""
####


def continuity_audit(
    samples: Sequence[Mapping[str, float]],
    channels: Mapping[str, float],
    *,
    event_times: Sequence[float] = (),
    declared_events: Sequence[DeclaredDiscontinuity] = (),
    event_time_tolerance: float = 1.0e-9,
) -> dict[str, object]:
    """Audit sampled state continuity against explicit jump tolerances.

    ``channels`` maps a telemetry channel to its permitted one-sample change.
    The tolerance is intentionally supplied by the scenario owner: a position
    sampled at 100 Hz and a position sampled at 1 Hz do not share a useful
    universal threshold.  A jump straddling a declared event time is reported
    separately and is not counted as an unexplained violation.

    This checks continuity of the recorded state, not physical smoothness of
    its derivative.  A finite force may change acceleration discontinuously,
    but position and velocity remain continuous unless the scenario declares
    an impulse or state reset.
    """

    if len(samples) < 2:
        raise ValueError("continuity audit requires at least two samples")
    times = tuple(float(sample["time_s"]) for sample in samples)
    if any(right <= left for left, right in zip(times, times[1:], strict=False)):
        raise ValueError("telemetry times must increase for continuity audit")
    declared = tuple(
        (*declared_events, *(DeclaredDiscontinuity(f"event-{index}", float(value), ("*",)) for index, value in enumerate(event_times)))
    )
    violations: list[ContinuityViolation] = []
    event_jumps: list[ContinuityViolation] = []
    maximums: dict[str, float] = {}
    for channel, tolerance in channels.items():
        if tolerance < 0.0:
            raise ValueError(f"continuity tolerance for {channel!r} must be nonnegative")
        values = require_channel(samples, channel)
        maximums[channel] = max(abs(right - left) for left, right in zip(values, values[1:], strict=False))
        for index, (left, right) in enumerate(zip(values, values[1:], strict=False)):
            delta = abs(right - left)
            if delta <= tolerance:
                continue
            left_time = times[index]
            right_time = times[index + 1]
            matching_event = next(
                (
                    event
                    for event in declared
                    if left_time - event_time_tolerance <= event.time_s <= right_time + event_time_tolerance
                    and ("*" in event.allowed_channels or channel in event.allowed_channels)
                ),
                None,
            )
            violation = ContinuityViolation(channel, left_time, right_time, delta, tolerance, matching_event.time_s if matching_event else None)
            (event_jumps if matching_event is not None else violations).append(violation)
    return {
        "passed": not violations,
        "samples": len(samples),
        "channels": dict(channels),
        "maximum_abs_delta": maximums,
        "unexplained_violations": [asdict(item) for item in violations],
        "declared_event_jumps": [asdict(item) for item in event_jumps],
        "declared_events": [asdict(item) for item in declared],
    }
####


@dataclass(frozen=True, slots=True)
class PhaseWindow:
    """Named interval used to scope a trajectory acceptance check."""

    name: str
    start_s: float
    end_s: float

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("phase name must not be empty")
        if self.start_s < 0.0 or self.end_s <= self.start_s:
            raise ValueError("phase window must have positive ordered bounds")
    ####

    def select(self, history: Sequence[Mapping[str, float]]) -> tuple[Mapping[str, float], ...]:
        """Return telemetry samples whose time lies within this phase."""

        samples = tuple(
            sample
            for sample in history
            if self.start_s <= float(sample.get("time_s", -1.0)) <= self.end_s
        )
        if not samples:
            raise AssertionError(f"phase {self.name!r} has no telemetry samples")
        return samples
    ####


def require_channel(samples: Sequence[Mapping[str, float]], channel: str) -> tuple[float, ...]:
    """Return one finite channel or raise an actionable validation error."""

    values: list[float] = []
    for sample in samples:
        if channel not in sample:
            raise AssertionError(f"required validation channel {channel!r} is missing")
        value = float(sample[channel])
        if value != value or value in (float("inf"), float("-inf")):
            raise AssertionError(f"validation channel {channel!r} is not finite")
        values.append(value)
    return tuple(values)
####


def require_bounded(
    samples: Sequence[Mapping[str, float]],
    channel: str,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
) -> None:
    """Require every value in a phase to remain inside an inclusive band."""

    values = require_channel(samples, channel)
    if minimum is not None and min(values) < minimum:
        raise AssertionError(f"{channel} fell below {minimum}: {min(values)}")
    if maximum is not None and max(values) > maximum:
        raise AssertionError(f"{channel} exceeded {maximum}: {max(values)}")
####


def require_monotonic(
    samples: Sequence[Mapping[str, float]],
    channel: str,
    direction: Direction,
    *,
    tolerance: float = 0.0,
) -> None:
    """Require a channel to move in one direction within a numeric tolerance."""

    values = require_channel(samples, channel)
    deltas = tuple(right - left for left, right in zip(values, values[1:], strict=False))
    violation = max((delta for delta in deltas if (delta < -tolerance if direction == "increasing" else delta > tolerance)), default=0.0)
    if violation:
        raise AssertionError(f"{channel} is not {direction} within tolerance {tolerance}")
####


def require_net_change(
    samples: Sequence[Mapping[str, float]],
    channel: str,
    direction: Direction,
    *,
    minimum: float = 0.0,
) -> None:
    """Require the net phase displacement to have the expected direction."""

    values = require_channel(samples, channel)
    change = values[-1] - values[0]
    signed_change = change if direction == "increasing" else -change
    if signed_change < minimum:
        raise AssertionError(f"{channel} net change was {change}, expected {direction} by {minimum}")
####


def require_change_of_sign(
    samples: Sequence[Mapping[str, float]],
    channel: str,
    *,
    minimum_before: float = 0.0,
    minimum_after: float = 0.0,
) -> None:
    """Require a signed response with nontrivial values on both sides."""

    values = require_channel(samples, channel)
    midpoint = len(values) // 2
    before = values[:midpoint]
    after = values[midpoint:]
    if not before or not after:
        raise AssertionError(f"{channel} did not exhibit the required sign change")
    forward = min(before) <= -minimum_before and max(after) >= minimum_after
    reverse = max(before) >= minimum_before and min(after) <= -minimum_after
    if not (forward or reverse):
        raise AssertionError(f"{channel} did not exhibit the required sign change")
####


def phase_slice(
    history: Sequence[Mapping[str, float]],
    start_s: float,
    end_s: float,
) -> tuple[Mapping[str, float], ...]:
    """Select a time interval without requiring a named phase object."""

    return PhaseWindow("phase", start_s, end_s).select(history)
####


def wrapped_angle_error(actual: float, target: float, *, period: float = 360.0) -> float:
    """Return the shortest signed angular error for a periodic coordinate."""

    if not period > 0.0:
        raise ValueError("angle period must be positive")
    return (actual - target + 0.5 * period) % period - 0.5 * period
####


def _time_values(samples: Sequence[Mapping[str, float]]) -> tuple[float, ...]:
    return require_channel(samples, "time_s")
####


def _trapezoidal_integral(times: Sequence[float], values: Sequence[float]) -> float:
    if len(times) != len(values) or len(times) < 2:
        raise AssertionError("at least two aligned samples are required for integration")
    return sum(
        0.5 * (right_time - left_time) * (right_value + left_value)
        for left_time, right_time, left_value, right_value in zip(
            times, times[1:], values, values[1:], strict=False
        )
    )
####


def dwell_in_band(
    samples: Sequence[Mapping[str, float]],
    channel: str,
    *,
    lower: float,
    upper: float,
) -> float:
    """Return the sampled time spent continuously inside an inclusive band."""

    if lower > upper:
        raise ValueError("band lower bound must not exceed upper bound")
    times = _time_values(samples)
    values = require_channel(samples, channel)
    return sum(
        right_time - left_time
        for left_time, right_time, left_value, right_value in zip(
            times, times[1:], values, values[1:], strict=False
        )
        if lower <= left_value <= upper and lower <= right_value <= upper
    )
####


def capture_time(
    samples: Sequence[Mapping[str, float]],
    channel: str,
    target: float,
    *,
    tolerance: float,
    start_s: float | None = None,
) -> float | None:
    """Return the first time after which a channel remains in its target band."""

    if tolerance < 0.0:
        raise ValueError("capture tolerance must be nonnegative")
    candidates = tuple(
        sample for sample in samples if start_s is None or float(sample["time_s"]) >= start_s
    )
    require_channel(candidates, channel)
    for index, sample in enumerate(candidates):
        if all(abs(float(later[channel]) - target) <= tolerance for later in candidates[index:]):
            return float(sample["time_s"])
    return None
####


def settling_time(
    samples: Sequence[Mapping[str, float]],
    channel: str,
    target: float,
    *,
    tolerance: float,
    start_s: float | None = None,
) -> float | None:
    """Alias for the stricter capture definition used in reports."""

    return capture_time(samples, channel, target, tolerance=tolerance, start_s=start_s)
####


def independent_force_closure(
    samples: Sequence[Mapping[str, float]],
    *,
    mass_channel: str = "mass_kg",
    velocity_channels: tuple[str, str, str] = ("xdt", "ydt", "zdt"),
    force_channels: tuple[str, str, str] = (
        "total_force_ecic_x_n",
        "total_force_ecic_y_n",
        "total_force_ecic_z_n",
    ),
    gravity_scale: float = 9.80665,
    event_times: Sequence[float] = (),
    event_time_tolerance_s: float = 1.0e-12,
) -> dict[str, float | list[float]]:
    """Compare finite-difference inertial acceleration with saved total force.

    This deliberately operates on emitted telemetry rather than the integrator
    RHS. It is therefore an independent diagnostic for a rigid-body run. The
    A centered five-point stencil is used on uniformly sampled histories to
    reduce truncation error without reusing the integrator RHS. Short or
    nonuniform histories use the three-point centered stencil; discontinuity
    phases should be sliced out by callers.
    """

    if len(samples) < 3:
        raise ValueError("independent closure requires at least three samples")
    require_channel(samples, "time_s")
    for channel in (mass_channel, *velocity_channels, *force_channels):
        require_channel(samples, channel)
    times = tuple(float(sample["time_s"]) for sample in samples)
    intervals = tuple(right - left for left, right in zip(times, times[1:], strict=False))
    if any(interval <= 0.0 for interval in intervals):
        raise ValueError("telemetry times must increase for independent closure")
    uniform = len(intervals) >= 4 and max(intervals) - min(intervals) <= 1.0e-9 * max(intervals)
    if event_time_tolerance_s < 0.0:
        raise ValueError("event time tolerance must be nonnegative")
    declared_events = tuple(float(value) for value in event_times)
    residuals: list[float] = []
    residual_times: list[float] = []
    excluded_event_times: list[float] = []
    indices = range(2, len(samples) - 2) if uniform else range(1, len(samples) - 1)
    for index in indices:
        stencil_start = times[index - 2] if uniform else times[index - 1]
        stencil_end = times[index + 2] if uniform else times[index + 1]
        crossing_event = next(
            (
                event_time
                for event_time in declared_events
                if stencil_start - event_time_tolerance_s
                <= event_time
                <= stencil_end + event_time_tolerance_s
            ),
            None,
        )
        if crossing_event is not None:
            excluded_event_times.append(crossing_event)
            continue
        center = samples[index]
        if uniform:
            step = intervals[0]
            acceleration = tuple(
                (
                    float(samples[index - 2][velocity])
                    - 8.0 * float(samples[index - 1][velocity])
                    + 8.0 * float(samples[index + 1][velocity])
                    - float(samples[index + 2][velocity])
                )
                / (12.0 * step)
                for velocity in velocity_channels
            )
        else:
            left = samples[index - 1]
            right = samples[index + 1]
            delta_t = times[index + 1] - times[index - 1]
            acceleration = tuple(
                (float(right[velocity]) - float(left[velocity])) / delta_t
                for velocity in velocity_channels
            )
        mass = float(center[mass_channel])
        force = tuple(float(center[channel]) for channel in force_channels)
        residual = sum((mass * a - f) ** 2 for a, f in zip(acceleration, force, strict=True)) ** 0.5
        scale = max(mass * gravity_scale, sum(value * value for value in force) ** 0.5, 1.0)
        residuals.append(residual / scale)
        residual_times.append(times[index])
    if not residuals:
        raise ValueError("independent force closure has no smooth interior samples")
    ordered = sorted(residuals)
    p99 = ordered[min(len(ordered) - 1, int(0.99 * len(ordered)))]
    return {
        "sample_count": float(len(residuals)),
        "maximum_normalized_residual": max(residuals),
        "p99_normalized_residual": p99,
        "median_normalized_residual": ordered[len(ordered) // 2],
        "maximum_time_s": residual_times[residuals.index(max(residuals))],
        "event_times": list(declared_events),
        "event_excluded_sample_count": float(len(excluded_event_times)),
    }
####


def independent_moment_closure(
    samples: Sequence[Mapping[str, float]],
    *,
    inertia_channels: tuple[str, str, str] = ("inertia_x_kg_m2", "inertia_y_kg_m2", "inertia_z_kg_m2"),
    rate_channels: tuple[str, str, str] = ("wx", "wy", "wz"),
    moment_channels: tuple[str, str, str] = (
        "total_moment_body_x_nm",
        "total_moment_body_y_nm",
        "total_moment_body_z_nm",
    ),
    event_times: Sequence[float] = (),
    event_time_tolerance_s: float = 1.0e-12,
) -> dict[str, float | list[float]]:
    """Recompute Euler rotational closure from saved telemetry.

    This deliberately does not use the integrator RHS or the runtime residual.
    It evaluates ``I * omega_dot + omega x (I omega) - M`` from emitted body
    rates and total moments. The current rigid-body contract uses a diagonal
    inertia tensor.
    """

    if len(samples) < 3:
        raise ValueError("independent moment closure requires at least three samples")
    require_channel(samples, "time_s")
    for channel in (*inertia_channels, *rate_channels, *moment_channels):
        require_channel(samples, channel)
    times = tuple(float(sample["time_s"]) for sample in samples)
    intervals = tuple(right - left for left, right in zip(times, times[1:], strict=False))
    if any(interval <= 0.0 for interval in intervals):
        raise ValueError("telemetry times must increase for independent moment closure")
    uniform = len(intervals) >= 4 and max(intervals) - min(intervals) <= 1.0e-9 * max(intervals)
    if event_time_tolerance_s < 0.0:
        raise ValueError("event time tolerance must be nonnegative")
    declared_events = tuple(float(value) for value in event_times)
    residuals: list[float] = []
    residual_times: list[float] = []
    excluded_event_times: list[float] = []
    indices = range(2, len(samples) - 2) if uniform else range(1, len(samples) - 1)
    for index in indices:
        stencil_start = times[index - 2] if uniform else times[index - 1]
        stencil_end = times[index + 2] if uniform else times[index + 1]
        crossing_event = next(
            (
                event_time
                for event_time in declared_events
                if stencil_start - event_time_tolerance_s
                <= event_time
                <= stencil_end + event_time_tolerance_s
            ),
            None,
        )
        if crossing_event is not None:
            excluded_event_times.append(crossing_event)
            continue
        center = samples[index]
        if uniform:
            step = intervals[0]
            rate_dot = tuple(
                (
                    float(samples[index - 2][rate])
                    - 8.0 * float(samples[index - 1][rate])
                    + 8.0 * float(samples[index + 1][rate])
                    - float(samples[index + 2][rate])
                )
                / (12.0 * step)
                for rate in rate_channels
            )
        else:
            left = samples[index - 1]
            right = samples[index + 1]
            delta_t = times[index + 1] - times[index - 1]
            rate_dot = tuple(
                (float(right[rate]) - float(left[rate])) / delta_t
                for rate in rate_channels
            )
        inertia = tuple(float(center[channel]) for channel in inertia_channels)
        omega = tuple(float(center[channel]) for channel in rate_channels)
        angular_momentum = tuple(value * rate for value, rate in zip(inertia, omega, strict=True))
        gyro = (
            omega[1] * angular_momentum[2] - omega[2] * angular_momentum[1],
            omega[2] * angular_momentum[0] - omega[0] * angular_momentum[2],
            omega[0] * angular_momentum[1] - omega[1] * angular_momentum[0],
        )
        moment = tuple(float(center[channel]) for channel in moment_channels)
        residual = sum(
            (inertia[axis] * rate_dot[axis] + gyro[axis] - moment[axis]) ** 2
            for axis in range(3)
        ) ** 0.5
        scale = max(sum(value * value for value in moment) ** 0.5, 1.0)
        residuals.append(residual / scale)
        residual_times.append(times[index])
    if not residuals:
        raise ValueError("independent moment closure has no interior samples")
    ordered = sorted(residuals)
    p99 = ordered[min(len(ordered) - 1, int(0.99 * len(ordered)))]
    return {
        "sample_count": float(len(residuals)),
        "maximum_normalized_residual": max(residuals),
        "p99_normalized_residual": p99,
        "median_normalized_residual": ordered[len(ordered) // 2],
        "maximum_time_s": residual_times[residuals.index(max(residuals))],
        "event_times": list(declared_events),
        "event_excluded_sample_count": float(len(excluded_event_times)),
    }
####


def specific_energy(
    samples: Sequence[Mapping[str, float]],
    *,
    altitude_channel: str = "altitude_m",
    speed_channel: str = "speed_m_s",
    gravity_m_s2: float = 9.80665,
) -> tuple[float, ...]:
    """Return mechanical specific energy ``g*h + V**2/2`` for each sample."""

    if gravity_m_s2 <= 0.0:
        raise ValueError("gravity must be positive")
    altitudes = require_channel(samples, altitude_channel)
    speeds = require_channel(samples, speed_channel)
    return tuple(gravity_m_s2 * altitude + 0.5 * speed * speed for altitude, speed in zip(altitudes, speeds, strict=True))
####


def energy_balance_residual(
    samples: Sequence[Mapping[str, float]],
    *,
    drag_force_channel: str = "drag_force_n",
    thrust_force_channel: str = "thrust_n",
    mass_channel: str = "mass_kg",
    altitude_channel: str = "altitude_m",
    speed_channel: str = "speed_m_s",
    gravity_m_s2: float = 9.80665,
) -> float:
    """Compare specific-energy change with integrated drag/thrust work per mass.

    Drag is expected as a nonnegative force opposing the velocity and thrust as
    a nonnegative force aligned with it.  The result is final minus initial
    specific energy minus the integrated ``(thrust - drag) * V / m`` work.
    """

    times = _time_values(samples)
    energies = specific_energy(
        samples,
        altitude_channel=altitude_channel,
        speed_channel=speed_channel,
        gravity_m_s2=gravity_m_s2,
    )
    drag = require_channel(samples, drag_force_channel)
    thrust = require_channel(samples, thrust_force_channel)
    mass = require_channel(samples, mass_channel)
    speed = require_channel(samples, speed_channel)
    specific_power = tuple((t - d) * v / m for t, d, v, m in zip(thrust, drag, speed, mass, strict=True))
    return energies[-1] - energies[0] - _trapezoidal_integral(times, specific_power)
####


def integral_mass_balance_error(
    samples: Sequence[Mapping[str, float]],
    *,
    mass_channel: str = "mass_kg",
    mass_rate_channel: str = "mass_rate_kg_s",
) -> float:
    """Return final mass minus initial mass plus integrated positive outflow."""

    times = _time_values(samples)
    mass = require_channel(samples, mass_channel)
    rate = require_channel(samples, mass_rate_channel)
    expected_final = mass[0] - _trapezoidal_integral(times, rate)
    return mass[-1] - expected_final
####


def actuator_saturation_fraction(
    samples: Sequence[Mapping[str, float]],
    channel: str = "actuator_saturated",
) -> float:
    """Return the fraction of telemetry intervals marked saturated."""

    values = require_channel(samples, channel)
    if not values:
        raise AssertionError("saturation channel is empty")
    return sum(value != 0.0 for value in values) / len(values)
####


def timestep_convergence_error(
    coarse: Sequence[Mapping[str, float]],
    fine: Sequence[Mapping[str, float]],
    channel: str,
) -> float:
    """Return the maximum absolute aligned-history difference for one channel."""

    coarse_times = _time_values(coarse)
    fine_times = _time_values(fine)
    if len(coarse_times) != len(fine_times) or any(abs(a - b) > 1.0e-9 for a, b in zip(coarse_times, fine_times, strict=True)):
        raise AssertionError("convergence histories must have aligned sample times")
    coarse_values = require_channel(coarse, channel)
    fine_values = require_channel(fine, channel)
    return max(abs(a - b) for a, b in zip(coarse_values, fine_values, strict=True))
####


__all__ = [
    "Direction",
    "PhaseWindow",
    "actuator_saturation_fraction",
    "capture_time",
    "dwell_in_band",
    "energy_balance_residual",
    "independent_force_closure",
    "independent_moment_closure",
    "integral_mass_balance_error",
    "phase_slice",
    "require_bounded",
    "require_change_of_sign",
    "require_channel",
    "require_monotonic",
    "require_net_change",
    "settling_time",
    "specific_energy",
    "timestep_convergence_error",
    "wrapped_angle_error",
]
