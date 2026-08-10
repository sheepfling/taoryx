from __future__ import annotations

import re
from dataclasses import dataclass
from math import cos, exp, hypot, pi, radians, sin

from .simple_aero_trajectories import SimpleAeroTrajectoryProblemSpec, SimpleAeroTrajectorySpec

_FIELD_PATTERN = re.compile(r"([A-Za-z_][A-Za-z0-9_\[\]]*)=([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)")
_WHEN_PATTERN = re.compile(r"\*when\s+(?:time|tseg)=([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)", re.IGNORECASE)


@dataclass(frozen=True, slots=True)
class SimpleAeroPlotSeries:
    """A synthetic 1D plot series for a SimpleAero trajectory artifact."""

    filename: str
    title: str
    axis_label: str
    value_label: str
    axis: tuple[float, ...]
    values: tuple[float, ...]


def build_problem_plot_series(problem: SimpleAeroTrajectoryProblemSpec) -> list[SimpleAeroPlotSeries]:
    """Build a deterministic, schematic plot pack for a SimpleAero problem."""

    series: list[SimpleAeroPlotSeries] = []
    for index, trajectory in enumerate(problem.trajectories, start=1):
        series.extend(_build_trajectory_plot_series(problem, trajectory, index))
    ####
    return series
####


def _build_trajectory_plot_series(
    problem: SimpleAeroTrajectoryProblemSpec,
    trajectory: SimpleAeroTrajectorySpec,
    trajectory_index: int,
) -> list[SimpleAeroPlotSeries]:
    kind = _classify_problem(problem, trajectory)
    role = _trajectory_role(trajectory)
    fields = _parse_fields(trajectory.initial_line)
    durations = tuple(_segment_duration(segment.body) for segment in trajectory.segments)
    duration = max(sum(durations), 1.0)
    samples = _sample_count(duration)
    time_axis = _sample_axis(duration, samples, start=fields.get("time", 0.0))
    profile = _sample_profile(problem, trajectory, kind, role, fields, time_axis, duration)

    label_prefix = f"trajectory-{trajectory_index}"
    plot_series: list[SimpleAeroPlotSeries] = []
    plot_series.append(
        SimpleAeroPlotSeries(
            filename=f"{label_prefix}-range-altitude.png",
            title=f"{label_prefix} range vs altitude",
            axis_label=_range_axis_label(problem, trajectory),
            value_label="altitude (m)",
            axis=profile["range_axis"],
            values=profile["altitude"],
        )
    )
    plot_series.append(
        SimpleAeroPlotSeries(
            filename=f"{label_prefix}-speed-time.png",
            title=f"{label_prefix} speed vs time",
            axis_label="time (s)",
            value_label="speed (m/s)",
            axis=time_axis,
            values=profile["speed"],
        )
    )
    plot_series.append(
        SimpleAeroPlotSeries(
            filename=f"{label_prefix}-mass-time.png",
            title=f"{label_prefix} mass vs time",
            axis_label="time (s)",
            value_label="mass (kg)",
            axis=time_axis,
            values=profile["mass"],
        )
    )
    if kind == "propnav":
        plot_series.append(
            SimpleAeroPlotSeries(
                filename=f"{label_prefix}-x-time.png",
                title=f"{label_prefix} x position vs time",
                axis_label="time (s)",
                value_label="x (m)",
                axis=time_axis,
                values=profile["x_position"],
            )
        )
        plot_series.append(
            SimpleAeroPlotSeries(
                filename=f"{label_prefix}-y-time.png",
                title=f"{label_prefix} y position vs time",
                axis_label="time (s)",
                value_label="y (m)",
                axis=time_axis,
                values=profile["y_position"],
            )
        )
    else:
        plot_series.append(
            SimpleAeroPlotSeries(
                filename=f"{label_prefix}-latitude-time.png",
                title=f"{label_prefix} latitude vs time",
                axis_label="time (s)",
                value_label="latitude (deg)",
                axis=time_axis,
                values=profile["latitude"],
            )
        )
        plot_series.append(
            SimpleAeroPlotSeries(
                filename=f"{label_prefix}-longitude-time.png",
                title=f"{label_prefix} longitude vs time",
                axis_label="time (s)",
                value_label="longitude (deg)",
                axis=time_axis,
                values=profile["longitude"],
            )
        )
    if kind != "propnav" or role == "interceptor":
        plot_series.append(
            SimpleAeroPlotSeries(
                filename=f"{label_prefix}-bank-time.png",
                title=f"{label_prefix} bank angle vs time",
                axis_label="time (s)",
                value_label="bank angle (deg)",
                axis=time_axis,
                values=profile["bank"],
            )
        )
        plot_series.append(
            SimpleAeroPlotSeries(
                filename=f"{label_prefix}-alpha-time.png",
                title=f"{label_prefix} angle of attack vs time",
                axis_label="time (s)",
                value_label="alpha (deg)",
                axis=time_axis,
                values=profile["alpha"],
            )
        )
    else:
        plot_series.append(
            SimpleAeroPlotSeries(
                filename=f"{label_prefix}-closing-range-time.png",
                title=f"{label_prefix} closing range vs time",
                axis_label="time (s)",
                value_label="range (m)",
                axis=time_axis,
                values=profile["closing_range"],
            )
        )
    plot_series.append(
        SimpleAeroPlotSeries(
            filename=f"{label_prefix}-gamma-time.png",
            title=f"{label_prefix} flight-path angle vs time",
            axis_label="time (s)",
            value_label="gamma (deg)",
            axis=time_axis,
            values=profile["gamma"],
        )
    )
    plot_series.append(
        SimpleAeroPlotSeries(
            filename=f"{label_prefix}-thrust-time.png",
            title=f"{label_prefix} thrust vs time",
            axis_label="time (s)",
            value_label="thrust (arb)",
            axis=time_axis,
            values=profile["thrust"],
        )
    )
    ####
    return plot_series
####


def _sample_profile(
    problem: SimpleAeroTrajectoryProblemSpec,
    trajectory: SimpleAeroTrajectorySpec,
    kind: str,
    role: str,
    fields: dict[str, float],
    time_axis: tuple[float, ...],
    duration: float,
) -> dict[str, tuple[float, ...]]:
    altitude: list[float] = []
    speed: list[float] = []
    mass: list[float] = []
    bank: list[float] = []
    alpha: list[float] = []
    gamma: list[float] = []
    thrust: list[float] = []
    range_axis: list[float] = []
    latitude: list[float] = []
    longitude: list[float] = []
    x_position: list[float] = []
    y_position: list[float] = []
    closing_range: list[float] = []

    start_altitude = fields.get("alt", fields.get("z", 0.0))
    start_speed = fields.get("vel", hypot(fields.get("xdt", 0.0), fields.get("ydt", 0.0)))
    start_mass = fields.get("wt", fields.get("mass", 1000.0))
    start_lat = fields.get("lat", 35.8766)
    start_lon = fields.get("long", 14.4425)
    start_x = fields.get("x", 0.0)
    start_y = fields.get("y", 0.0)
    start_psi = fields.get("psi", 90.0)
    start_gamma = fields.get("gama", 0.0)

    turn_sign = _turn_sign(problem, trajectory)
    range_scale = max(start_speed, 15.0) * max(duration, 1.0) * 8.0
    if kind == "propnav":
        range_scale = max(start_speed, 1.0) * max(duration, 1.0) * 10.0

    target_path: list[tuple[float, float]] | None = None
    if kind == "propnav" and role == "interceptor":
        target_path = _propnav_target_path(problem, time_axis)

    for time in time_axis:
        progress = time / duration if duration > 0 else 0.0
        if kind == "ballistic":
            altitude.append(start_altitude + 26000.0 * _smoothstep(min(progress / 0.35, 1.0)) - 30000.0 * _smoothstep(max(0.0, (progress - 0.25) / 0.75)))
            speed.append(start_speed + 220.0 * _smoothstep(min(progress / 0.18, 1.0)) - 90.0 * _smoothstep(max(0.0, (progress - 0.2) / 0.8)))
            bank.append(0.0)
            alpha.append(1.5 * sin(pi * progress))
            gamma.append(start_gamma + 6.0 * sin(pi * progress * 0.8))
            thrust.append(1.0 if progress < 0.18 else 0.0)
            mass.append(max(start_mass - start_mass * 0.12 * _smoothstep(min(progress / 0.18, 1.0)), start_mass * 0.65))
            range_axis.append(range_scale * (0.04 + 0.96 * _smoothstep(progress)))
            lon, lat = _geodetic_position(start_lon, start_lat, range_axis[-1], start_psi, turn_sign, kind)
            longitude.append(lon)
            latitude.append(lat)
        elif kind == "range_extension":
            altitude.append(start_altitude + 12000.0 * _smoothstep(min(progress / 0.25, 1.0)) - 18000.0 * _smoothstep(max(0.0, (progress - 0.22) / 0.78)))
            speed.append(start_speed + 70.0 * _smoothstep(min(progress / 0.15, 1.0)) - 35.0 * progress)
            bank.append(4.0 * sin(pi * progress))
            alpha.append(0.6 + 0.8 * sin(pi * progress * 0.6))
            gamma.append(start_gamma - 3.5 * progress)
            thrust.append(0.0)
            mass.append(start_mass)
            range_axis.append(range_scale * (0.02 + 0.98 * _smoothstep(progress)))
            lon, lat = _geodetic_position(start_lon, start_lat, range_axis[-1], start_psi, turn_sign, kind)
            longitude.append(lon)
            latitude.append(lat)
        elif kind == "phugoid":
            envelope = exp(-1.2 * progress)
            altitude.append(start_altitude + 6500.0 * envelope * sin(2.0 * pi * 3.0 * progress))
            speed.append(start_speed + 110.0 * envelope * cos(2.0 * pi * 3.0 * progress + pi / 4.0))
            bank.append(18.0 * sin(2.0 * pi * 1.3 * progress))
            alpha.append(4.5 * envelope * sin(2.0 * pi * 3.0 * progress + pi / 6.0))
            gamma.append(start_gamma + 6.0 * envelope * cos(2.0 * pi * 2.0 * progress))
            thrust.append(0.0)
            mass.append(start_mass)
            range_axis.append(range_scale * (0.05 + 0.95 * _smoothstep(progress)))
            lon, lat = _geodetic_position(start_lon, start_lat, range_axis[-1], start_psi + 4.0 * sin(2.0 * pi * progress), turn_sign, kind)
            longitude.append(lon)
            latitude.append(lat)
        elif kind == "skip":
            altitude.append(start_altitude + 14000.0 * abs(sin(2.0 * pi * 1.15 * progress)) * exp(-0.25 * progress))
            speed.append(start_speed + 45.0 * sin(2.0 * pi * 0.7 * progress + 0.35) - 20.0 * progress)
            bank.append(20.0 * sin(2.0 * pi * 2.0 * progress))
            alpha.append(2.4 * sin(2.0 * pi * 1.4 * progress))
            gamma.append(start_gamma - 4.0 * progress)
            thrust.append(0.0)
            mass.append(start_mass)
            range_axis.append(range_scale * (0.03 + 0.97 * _smoothstep(progress)))
            lon, lat = _geodetic_position(start_lon, start_lat, range_axis[-1], start_psi + 20.0 * sin(2.0 * pi * progress), turn_sign, kind)
            longitude.append(lon)
            latitude.append(lat)
        elif kind == "turn":
            altitude.append(start_altitude + 2200.0 * sin(pi * progress) - 6500.0 * _smoothstep(progress))
            speed.append(start_speed + 22.0 * _smoothstep(min(progress / 0.28, 1.0)) - 18.0 * progress)
            bank.append(turn_sign * (22.0 + 14.0 * sin(2.0 * pi * _turn_cycles(trajectory) * progress)))
            alpha.append(1.3 * sin(2.0 * pi * _turn_cycles(trajectory) * progress + pi / 5.0))
            gamma.append(start_gamma - 3.0 * progress)
            thrust.append(0.0)
            mass.append(start_mass)
            range_axis.append(range_scale * (0.02 + 0.98 * _smoothstep(progress)))
            lon, lat = _geodetic_position(start_lon, start_lat, range_axis[-1], start_psi + turn_sign * 30.0 * _smoothstep(progress), turn_sign, kind)
            longitude.append(lon)
            latitude.append(lat)
        elif kind == "propnav":
            target_x, target_y = (target_path or _propnav_target_path(problem, time_axis))[len(range_axis)]
            if role == "interceptor":
                closing = max(0.0, hypot(target_x - start_x, target_y - start_y) * (1.0 - 0.97 * _smoothstep(progress)))
                range_axis.append(range_scale * _smoothstep(progress))
                closing_range.append(closing)
                speed.append(start_speed + 25.0 * _smoothstep(min(progress / 0.2, 1.0)) - 8.0 * progress)
                altitude.append(start_altitude + 300.0 * sin(pi * progress))
                bank.append(14.0 * sin(pi * progress))
                alpha.append(2.0 * sin(2.0 * pi * progress))
                gamma.append(start_gamma + 2.0 * sin(pi * progress))
                thrust.append(1.0 if progress < 0.25 else 0.35)
                mass.append(max(start_mass - 0.6 * _smoothstep(min(progress / 0.25, 1.0)), start_mass * 0.7))
                x = start_x + (target_x - start_x) * (0.08 + 0.92 * _smoothstep(progress)) + 120.0 * sin(2.0 * pi * progress)
                y = start_y + (target_y - start_y) * (0.08 + 0.92 * _smoothstep(progress)) - 80.0 * sin(2.0 * pi * progress)
                x_position.append(x)
                y_position.append(y)
            else:
                speed.append(hypot(fields.get("xdt", 0.0), fields.get("ydt", 0.0)))
                altitude.append(start_altitude)
                bank.append(0.0)
                alpha.append(0.0)
                gamma.append(start_gamma)
                thrust.append(0.0)
                mass.append(start_mass)
                x = start_x + fields.get("xdt", 0.0) * time
                y = start_y + fields.get("ydt", 0.0) * time
                range_axis.append(range_scale * _smoothstep(progress))
                x_position.append(x)
                y_position.append(y)
                closing_range.append(hypot(x - fields.get("x", start_x), y - fields.get("y", start_y)))
        else:
            altitude.append(start_altitude + 1400.0 * sin(pi * progress) - 3000.0 * progress)
            speed.append(start_speed + 10.0 * sin(2.0 * pi * progress))
            bank.append(5.0 * sin(2.0 * pi * progress))
            alpha.append(0.7 * sin(2.0 * pi * progress))
            gamma.append(start_gamma - 2.0 * progress)
            thrust.append(0.0)
            mass.append(start_mass)
            range_axis.append(range_scale * (0.05 + 0.95 * _smoothstep(progress)))
            lon, lat = _geodetic_position(start_lon, start_lat, range_axis[-1], start_psi, turn_sign, kind)
            longitude.append(lon)
            latitude.append(lat)
    ####
    return {
        "altitude": tuple(altitude),
        "speed": tuple(speed),
        "mass": tuple(mass),
        "bank": tuple(bank),
        "alpha": tuple(alpha),
        "gamma": tuple(gamma),
        "thrust": tuple(thrust),
        "range_axis": tuple(range_axis),
        "latitude": tuple(latitude),
        "longitude": tuple(longitude),
        "x_position": tuple(x_position),
        "y_position": tuple(y_position),
        "closing_range": tuple(closing_range) if closing_range else tuple(range_axis),
    }
####


def _propnav_target_path(problem: SimpleAeroTrajectoryProblemSpec, time_axis: tuple[float, ...]) -> list[tuple[float, float]]:
    if len(problem.trajectories) < 2:
        return [(0.0, 0.0) for _ in time_axis]
    ####
    target = problem.trajectories[1]
    fields = _parse_fields(target.initial_line)
    start_x = fields.get("x", 0.0)
    start_y = fields.get("y", 0.0)
    x_rate = fields.get("xdt", 0.0)
    y_rate = fields.get("ydt", 0.0)
    return [(start_x + x_rate * time, start_y + y_rate * time) for time in time_axis]
####


def _geodetic_position(
    start_lon: float,
    start_lat: float,
    range_m: float,
    heading_deg: float,
    turn_sign: float,
    kind: str,
) -> tuple[float, float]:
    heading = radians(heading_deg)
    lateral = range_m * sin(heading)
    northing = range_m * cos(heading)
    meters_per_degree_lat = 111_320.0
    meters_per_degree_lon = meters_per_degree_lat * max(cos(radians(start_lat)), 0.2)
    if kind in {"skip", "phugoid"}:
        lateral *= 1.12
        northing *= 0.94
    if turn_sign < 0:
        lateral *= -1.0
    return start_lon + lateral / meters_per_degree_lon, start_lat + northing / meters_per_degree_lat
####


def _range_axis_label(_problem: SimpleAeroTrajectoryProblemSpec, _trajectory: SimpleAeroTrajectorySpec) -> str:
    return "range (m)"
####


def _sample_count(duration: float) -> int:
    return max(61, int(round(duration * 4.0)) + 1)
####


def _sample_axis(duration: float, samples: int, *, start: float = 0.0) -> tuple[float, ...]:
    if samples <= 1:
        return (start,)
    ####
    step = duration / float(samples - 1)
    return tuple(start + step * index for index in range(samples))
####


def _parse_fields(line: str) -> dict[str, float]:
    fields: dict[str, float] = {}
    for match in _FIELD_PATTERN.finditer(line):
        fields[match.group(1)] = float(match.group(2))
    ####
    return fields
####


def _segment_duration(body: tuple[str, ...]) -> float:
    for line in body:
        match = _WHEN_PATTERN.search(line)
        if match is not None:
            return max(float(match.group(1)), 1.0)
    ####
    return 30.0
####


def _classify_problem(problem: SimpleAeroTrajectoryProblemSpec, trajectory: SimpleAeroTrajectorySpec) -> str:
    text = " ".join(
        [
            problem.path,
            problem.problem_name,
            problem.title,
            trajectory.trajectory_line,
            trajectory.initial_line,
            trajectory.file_line,
            *(segment.title for segment in trajectory.segments),
            *(segment.phase for segment in trajectory.segments),
        ]
    ).casefold()
    if "propnav" in text or "final_pronav" in text:
        return "propnav"
    if "phugoid" in text:
        return "phugoid"
    if "skip" in text:
        return "skip"
    if "ballistic" in text:
        return "ballistic"
    if "range_extension" in text or "range extension" in text:
        return "range_extension"
    if any(keyword in text for keyword in ("slalom", "weave", "crossrange", "cbcr", "marv")):
        return "turn"
    return "generic"
####


def _trajectory_role(trajectory: SimpleAeroTrajectorySpec) -> str:
    text = trajectory.trajectory_line.casefold()
    if "interceptor" in text:
        return "interceptor"
    if "target" in text:
        return "target"
    return "primary"
####


def _turn_sign(problem: SimpleAeroTrajectoryProblemSpec, trajectory: SimpleAeroTrajectorySpec) -> float:
    text = " ".join([problem.path, problem.problem_name, problem.title, trajectory.trajectory_line]).casefold()
    if any(keyword in text for keyword in ("right", "negative", "clockwise")):
        return -1.0
    ####
    return 1.0
####


def _turn_cycles(trajectory: SimpleAeroTrajectorySpec) -> float:
    text = " ".join([trajectory.trajectory_line, trajectory.file_line]).casefold()
    if "slalom" in text or "weave" in text:
        return 2.5
    if "phugoid" in text:
        return 1.5
    return 1.0
####


def _smoothstep(value: float) -> float:
    clipped = min(max(value, 0.0), 1.0)
    return clipped * clipped * (3.0 - 2.0 * clipped)
####
