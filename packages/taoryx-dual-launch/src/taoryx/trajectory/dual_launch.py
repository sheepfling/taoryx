"""Resolved-case adapters for the Alpha 2 dual-launch glider proof.

The adapter emits ordinary TAORYX problem syntax.  Air release is a glider
case that starts at the common post-release contract.  Attached-booster launch
adds a powered segment and then changes only the active force/mass source at
the declared separation boundary.  No private runner or X-15-specific syntax
is required.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from .contracts import ResolvedCase

LaunchMode = Literal["air_release", "attached_booster"]


@dataclass(frozen=True, slots=True)
class DualLaunchProblem:
    """One generated launch-form problem and its shared mission projection."""

    launch_mode: LaunchMode
    path: Path
    adapter: str
    separation_time_s: float
    shared_guidance: Mapping[str, str]
    ####


def _number(case: ResolvedCase, parameter_id: str, fallback: float) -> float:
    """Read one canonical numeric case parameter."""

    value = case.parameters.get(parameter_id)
    if value is None or not isinstance(value.value, (int, float)):
        return fallback
    return float(value.value)
    ####


def _target_geometry(range_m: float, bearing_deg: float) -> tuple[float, float]:
    """Return a small-range spherical target latitude/longitude in degrees."""

    radius_m = 6_378_137.0
    angular_range = range_m / radius_m
    bearing = math.radians(bearing_deg)
    target_lat = math.degrees(math.asin(math.sin(angular_range) * math.cos(bearing)))
    target_lon = math.degrees(math.atan2(math.sin(bearing) * math.sin(angular_range), math.cos(angular_range)))
    return target_lat, target_lon
    ####


def _shared_guidance(case: ResolvedCase) -> dict[str, str]:
    """Describe the guidance/control projection shared by both launch forms."""

    return {
        "waypoint_guidance": "terminal proportional-navigation guidance from the common aim point",
        "command.bank": "common bank command; authority is resolved from the family control schema",
        "command.throttle": "common throttle command; booster launch owns it only before separation",
        "post_release_mode": "glider guidance and control binding is identical after separation",
        "target_range_m": f"{_number(case, 'mission.target_range', 2000.0):g}",
        "target_bearing_deg": f"{_number(case, 'mission.target_bearing', 90.0):g}",
    }
    ####


def _problem_text(case: ResolvedCase, launch_mode: LaunchMode) -> tuple[str, float]:
    """Render one ordinary point-mass problem and return its separation time."""

    vehicle_value = case.parameters.get("vehicle.id")
    vehicle_id = str(vehicle_value.value) if vehicle_value is not None else "dual-launch-glider"
    initial_altitude = _number(case, "mission.initial_altitude", 1000.0)
    release_altitude = _number(case, "mission.release_altitude", 1200.0)
    initial_speed = _number(case, "mission.initial_speed", 120.0)
    release_speed = _number(case, "mission.release_speed", 155.0)
    flight_path_angle = _number(case, "mission.initial_flight_path_angle", 5.0)
    heading = _number(case, "mission.initial_heading", 90.0)
    target_range = _number(case, "mission.target_range", 2000.0)
    target_bearing = _number(case, "mission.target_bearing", 90.0)
    target_altitude = _number(case, "mission.target_altitude", 1200.0)
    target_speed = _number(case, "mission.target_speed", 120.0)
    glider_mass = _number(case, "vehicle.mass.glider", 900.0)
    initial_mass = _number(case, "vehicle.mass.initial", 1000.0)
    thrust = _number(case, "vehicle.booster.thrust", 15_000.0)
    mass_flow = _number(case, "vehicle.booster.mass_flow", 50.0)
    drag = _number(case, "vehicle.aero.drag_coefficient", 0.02)
    lift_to_drag = _number(case, "vehicle.aero.lift_to_drag", 4.0)
    boost_duration = _number(case, "mission.boost_duration", 2.0)
    glide_duration = _number(case, "mission.glide_duration", 12.0)
    terminal_duration = _number(case, "mission.terminal_duration", 4.0)
    dt = _number(case, "runtime.time_step", 0.05)
    output_dt = _number(case, "runtime.output_interval", 0.1)
    target_lat, target_lon = _target_geometry(target_range, target_bearing)
    total_duration = glide_duration + terminal_duration + (boost_duration if launch_mode == "attached_booster" else 0.0)
    separation_time = 0.0 if launch_mode == "air_release" else boost_duration
    initial_alt = release_altitude if launch_mode == "air_release" else initial_altitude
    initial_vel = release_speed if launch_mode == "air_release" else initial_speed
    initial_weight = glider_mass if launch_mode == "air_release" else initial_mass
    launch_label = "air-release" if launch_mode == "air_release" else "attached-booster"
    common_header = f"""(alpha2-t6-dual-launch-{launch_mode})
*title Alpha 2 dual-launch glider — {launch_label} composition proof
*mode point-mass
*atmos standard
*earth wgs-84 omega=0
*runtime status vehicle={vehicle_id} family={case.family} launch-mode={launch_mode} separation-time-s={separation_time:g} release-altitude-m={release_altitude:g} release-speed-mps={release_speed:g}
*runtime status target range-m={target_range:g} bearing-deg={target_bearing:g} latitude-deg={target_lat:g} longitude-deg={target_lon:g} altitude-m={target_altitude:g} velocity-mps={target_speed:g}
*runtime event separation kind=separation time={separation_time:g} action=handoff frame=ecic impulse=none continuity=required
*runtime event waypoint-capture kind=waypoint time={total_duration:g} action=stop frame=geodetic tolerance-m=100
*runtime control command.bank unit=deg default=0 lower=-25 upper=25 authority-modes=autopilot,commanded,overlay,direct,mixed default-authority=autopilot cadence-s=0.1 hold=hold rate-limit-per-s=45
*runtime control command.throttle unit=dimensionless default=0 lower=0 upper=1 authority-modes=autopilot,commanded,overlay,direct,mixed default-authority=autopilot cadence-s=0.1 hold=failsafe rate-limit-per-s=1

"""
    lines = [
        common_header,
        f"*trajectory 1 {vehicle_id} start on 1",
        f"  *initial geodetic alt={initial_alt:g} long=0 lat=0 vel={initial_vel:g} gama={flight_path_angle:g} psi={heading:g} time=0 mass={initial_weight:g}",
        f"  *file {vehicle_id}.dat time alt vel mass",
    ]
    if launch_mode == "attached_booster":
        lines.extend(
            (
                "  *segment 1 attached-booster",
                f"    *integ dtprnt={output_dt:g} dt={dt:g}",
                f"    *aero ca={-drag:g} cn={drag * lift_to_drag:g}",
                f"    *prop thrust={thrust:g} mdot={mass_flow:g}",
                f"    # separation event: continuous state handoff after {boost_duration:g} s of attached thrust",
                f"    *when tseg={boost_duration:g} goto 2",
            )
        )
        next_segment = 2
    else:
        next_segment = 1
    lines.extend(
        (
            f"  *segment {next_segment} release-glide",
            f"    *integ dtprnt={output_dt:g} dt={dt:g}",
            f"    *aero ca={-drag:g} cn={drag * lift_to_drag:g}",
            "    *prop thrust=0 mdot=0",
            "    *fly alpha=0",
            f"    *when tseg={glide_duration:g} goto {next_segment + 1}",
            f"  *segment {next_segment + 1} terminal-guidance",
            f"    *integ dtprnt={output_dt:g} dt={dt:g}",
            f"    *aero ca={-drag:g} cn={drag * lift_to_drag:g}",
            "    *prop thrust=0 mdot=0",
            "    *fly propnav=2",
            "    *when relrng[2] < 100 stop",
            f"    *when tseg={terminal_duration:g} stop",
            "",
            "*trajectory 2 target start on 1",
            f"  *initial geodetic alt={target_altitude:g} long={target_lon:g} lat={target_lat:g} vel={target_speed:g} gama=0 psi={target_bearing:g} time=0 mass=1",
            "  *file target.dat time alt vel",
            "  *segment 1 target-track",
            f"    *integ dtprnt={output_dt:g} dt={dt:g}",
            f"    *when tseg={total_duration:g} stop",
            "*end",
        )
    )
    return "\n".join(lines) + "\n", separation_time
    ####


def render_dual_launch_problems(case: ResolvedCase, directory: str | Path) -> tuple[DualLaunchProblem, ...]:
    """Generate air-release and attached-booster problems from one case."""

    destination = Path(directory)
    destination.mkdir(parents=True, exist_ok=True)
    shared = _shared_guidance(case)
    problems: list[DualLaunchProblem] = []
    for launch_mode in ("air_release", "attached_booster"):
        text, separation_time = _problem_text(case, launch_mode)
        path = destination / f"{launch_mode}.prb"
        path.write_text(text, encoding="utf-8")
        problems.append(
            DualLaunchProblem(
                launch_mode=launch_mode,
                path=path,
                adapter="resolved-case-to-native-dual-launch-point-mass",
                separation_time_s=separation_time,
                shared_guidance=shared,
            )
        )
    return tuple(problems)
    ####


__all__ = ["DualLaunchProblem", "LaunchMode", "render_dual_launch_problems"]
