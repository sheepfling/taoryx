"""Resolved-case adapters for the Alpha 2 T5 fidelity-ladder proof.

The ladder deliberately has two different comparison claims.  Point-mass and
pseudo-6DOF are a reduction-parity pair: the pseudo bridge must preserve the
translation while adding an explicitly labelled attitude reconstruction.  The
rigid-body case is a separate free-body experiment using the same resolved
mission inputs, not a trajectory that is expected to overlay the reduction.
"""

from __future__ import annotations

import math
import re
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from taoryx.simple_aero_builder import build_fixed_ld_from_resolved_case

from .contracts import FidelityProfile, ResolvedCase


@dataclass(frozen=True, slots=True)
class FidelityProblem:
    """One generated problem and its explicit adapter/projection contract."""

    fidelity: FidelityProfile
    path: Path
    adapter: str
    state_projection: Mapping[str, str]
    control_projection: Mapping[str, str]
    claim: str
    ####


def _number(case: ResolvedCase, parameter_id: str, fallback: float) -> float:
    """Read a canonical numeric value from a resolved case."""

    value = case.parameters.get(parameter_id)
    if value is None or not isinstance(value.value, (int, float)):
        return fallback
    return float(value.value)
    ####


def _duration(case: ResolvedCase) -> float:
    """Resolve the common mission duration from the declared segment graph."""

    durations = {
        "mission.boost_duration": _number(case, "mission.boost_duration", 0.5),
        "mission.coast_duration": _number(case, "mission.coast_duration", 0.5),
        "mission.bank_duration": _number(case, "mission.bank_duration", 0.5),
        "mission.terminal_duration": _number(case, "mission.terminal_duration", 0.5),
    }
    total = 0.0
    for item in case.segment_graph.get("segments", []):
        if not isinstance(item, Mapping):
            continue
        if "duration_s" in item:
            total += float(item["duration_s"])
        else:
            total += durations.get(str(item.get("duration_parameter")), 0.0)
    return total or 2.0
    ####


def _pseudo_text(point_text: str) -> str:
    """Derive the pseudo tier from the point-mass problem without new syntax."""

    marker = "*mode point-mass"
    if marker not in point_text:
        raise ValueError("point-mass adapter requires a point-mass mode directive")
    return point_text.replace(
        marker,
        "*mode kinematic-6dof\n*runtime status attitude mode=lag roll-deg=0 pitch-deg=0 yaw-deg=0 lag-s=0.25 max-rate-deg-s=360",
        1,
    ).replace(
        "Reduced-order SimpleAero-style fixed-L/D 3-DOF trajectory",
        "Alpha 2 T5 pseudo-6DOF attitude reconstruction over the same translation",
        1,
    )
    ####


def _canonicalize_point_initial(case: ResolvedCase, point_text: str) -> str:
    """Make the three tiers share one explicit canonical ECIC initial state.

    The legacy-shaped builder intentionally emits geodetic syntax.  For a
    cross-fidelity experiment the adapter must own the conversion, so the
    generated ladder uses the canonical spherical reference radius explicitly
    at the boundary and records that fact in the evidence packet.
    """

    altitude = _number(case, "mission.initial_altitude", 1000.0)
    speed = _number(case, "mission.initial_speed", 60.0)
    radius = 6_378_137.0 + altitude
    replacement = f"  *initial ecic x={radius:g} y=0 z=0 xdt=0 ydt={speed:g} zdt=0 time=0 mass={_number(case, 'vehicle.mass.initial', 1000.0):g}"
    canonical = re.sub(r"^  \*initial geodetic .*?$", replacement, point_text, count=1, flags=re.MULTILINE)
    return re.sub(r"^  \*file .*?$", "", canonical, flags=re.MULTILINE)
    ####


def _rigid_text(case: ResolvedCase) -> str:
    """Render the native rigid-body adapter for the synthetic ladder family."""

    vehicle_id = str(case.parameters["vehicle.id"].value)
    altitude = _number(case, "mission.initial_altitude", 1000.0)
    speed = _number(case, "mission.initial_speed", 60.0)
    drag = _number(case, "vehicle.aero.drag_coefficient", 0.02)
    lift = drag * _number(case, "vehicle.aero.lift_to_drag", 4.0)
    mass = _number(case, "vehicle.mass.initial", 1000.0)
    reference_area = _number(case, "vehicle.geometry.reference_area", 1.0)
    reference_length = _number(case, "vehicle.geometry.reference_length", 1.0)
    inertia_x = _number(case, "vehicle.inertia.x", 100.0)
    inertia_y = _number(case, "vehicle.inertia.y", 120.0)
    inertia_z = _number(case, "vehicle.inertia.z", 140.0)
    max_alpha = _number(case, "vehicle.envelope.max_alpha", 20.0)
    max_beta = _number(case, "vehicle.envelope.max_beta", 20.0)
    moment_x = _number(case, "vehicle.aero.moment_coefficient.x", 0.0002)
    moment_y = _number(case, "vehicle.aero.moment_coefficient.y", 0.0004)
    moment_z = _number(case, "vehicle.aero.moment_coefficient.z", 0.0006)
    dt = _number(case, "runtime.time_step", 0.05)
    output_dt = _number(case, "runtime.output_interval", 0.1)
    duration = _duration(case)
    # Body +X is aligned with the initial ECIC +Y velocity by a +90 degree
    # yaw quaternion.  The moments are intentionally small but nonzero so the
    # free rigid-body evidence exercises rates without immediately leaving the
    # declared synthetic envelope.
    q = math.sqrt(0.5)
    graph = case.segment_graph
    segments = graph.get("segments", [])
    if not isinstance(segments, list) or not segments:
        segments = [{"id": "ladder", "duration_s": duration}]
    lines: list[str] = [
        f"({case.case_id}-rigid)",
        "*title Alpha 2 T5 free rigid-body 6-DOF projection",
        "*mode rigid-body-6dof",
        "*atmos standard",
        "*earth wgs-84 omega=0",
        "*runtime status vehicle reference-area="
        f"{reference_area:g} reference-length={reference_length:g} inertia-x={inertia_x:g} inertia-y={inertia_y:g} inertia-z={inertia_z:g} dry-mass-kg={mass:g} envelope-max-alpha-deg={max_alpha:g} envelope-max-beta-deg={max_beta:g}",
        "*runtime status actuator maximum-moment=50 maximum-body-rate-deg-s=180",
        "*runtime status thermal policy=none",
        f"*trajectory 1 {vehicle_id} start on 1",
        f"  *initial ecic x={6_378_137.0 + altitude:g} y=0 z=0 xdt=0 ydt={speed:g} zdt=0 qw={q:.16g} qx=0 qy=0 qz={q:.16g} wx=0 wy=0 wz=0 time=0 mass={mass:g} propellant_mass=0",
    ]
    elapsed = 0.0
    for index, item in enumerate(segments, start=1):
        segment_id = str(item.get("id", f"segment-{index}")) if isinstance(item, Mapping) else f"segment-{index}"
        if isinstance(item, Mapping) and "duration_s" in item:
            segment_duration = float(item["duration_s"])
        elif isinstance(item, Mapping):
            duration_map = {
                "mission.boost_duration": _number(case, "mission.boost_duration", 0.5),
                "mission.coast_duration": _number(case, "mission.coast_duration", 0.5),
                "mission.bank_duration": _number(case, "mission.bank_duration", 0.5),
                "mission.terminal_duration": _number(case, "mission.terminal_duration", 0.5),
            }
            segment_duration = duration_map.get(str(item.get("duration_parameter")), 0.5)
        else:
            segment_duration = 0.5
        target = "stop" if index == len(segments) else f"goto {index + 1}"
        elapsed += segment_duration
        lines.extend(
            (
                f"  *segment {index} {segment_id}",
                f"    *integ dtprnt={output_dt:g} dt={dt:g}",
                "    *prop thrust=0 mdot=0",
                f"    *aero cx={-drag:g} cy=0 cz={-lift:g} cmx={moment_x:g} cmy={moment_y:g} cmz={moment_z:g}",
                "    *fly alpha=0",
                f"    *when time>{elapsed:g} {target}",
            )
        )
    lines.append("*end")
    return "\n".join(lines) + "\n"
    ####


def render_fidelity_problems(case: ResolvedCase, directory: str | Path) -> tuple[FidelityProblem, ...]:
    """Generate the three native problem files from one immutable resolved case."""

    destination = Path(directory)
    destination.mkdir(parents=True, exist_ok=True)
    point_text = _canonicalize_point_initial(case, build_fixed_ld_from_resolved_case(case).problem_text)
    point_path = destination / "point_mass_3dof.prb"
    point_path.write_text(point_text, encoding="utf-8")
    pseudo_path = destination / "pseudo_6dof.prb"
    pseudo_path.write_text(_pseudo_text(point_text), encoding="utf-8")
    rigid_path = destination / "rigid_body_6dof.prb"
    rigid_path.write_text(_rigid_text(case), encoding="utf-8")
    state_projection = {
        "position_ecic_m": "point:x,y,z; pseudo:x,y,z; rigid:x,y,z",
        "velocity_ecic_m_s": "point:xdt,ydt,zdt; pseudo:xdt,ydt,zdt; rigid:vx,vy,vz",
        "mass_kg": "point:mass; pseudo:mass; rigid:mass",
    }
    control_projection = {
        "command.bank": "point:fly bankgd; pseudo:fly bankgd; rigid:attitude projection",
        "command.throttle": "point:prop throttle; pseudo:prop throttle; rigid:prop throttle",
    }
    return (
        FidelityProblem("point_mass_3dof", point_path, "resolved-case-to-native-point-mass", state_projection, control_projection, "point-mass source tier"),
        FidelityProblem("pseudo_6dof", pseudo_path, "point-mass-to-kinematic-attitude-bridge", state_projection, control_projection, "kinematic bridge only; not rigid-body evidence"),
        FidelityProblem("rigid_body_6dof", rigid_path, "resolved-case-to-native-rigid-body", state_projection, control_projection, "free rigid-body behavior; not historical TAOS compatibility"),
    )
    ####


def scale_problem_step(text: str, factor: float) -> str:
    """Return a deterministic step-refined problem text for convergence runs."""

    if factor <= 0.0:
        raise ValueError("step scale must be positive")
    return re.sub(
        r"(\bdt=)([0-9.eE+-]+)",
        lambda match: f"{match.group(1)}{float(match.group(2)) * factor:.16g}",
        text,
    )
    ####


def project_state(fidelity: FidelityProfile, named: Mapping[str, float]) -> dict[str, float]:
    """Project native telemetry into common Alpha2 comparison channels."""

    velocity_names = ("xdt", "ydt", "zdt") if fidelity != "rigid_body_6dof" else ("vx", "vy", "vz")
    position = tuple(float(named.get(name, 0.0)) for name in ("x", "y", "z"))
    velocity = tuple(float(named.get(name, 0.0)) for name in velocity_names)
    return {
        "time_s": float(named.get("time", 0.0)),
        "x_m": position[0],
        "y_m": position[1],
        "z_m": position[2],
        "vx_m_s": velocity[0],
        "vy_m_s": velocity[1],
        "vz_m_s": velocity[2],
        "speed_m_s": math.sqrt(sum(component * component for component in velocity)),
        "mass_kg": float(named.get("mass", named.get("wt", 0.0))),
    }
    ####


__all__ = ["FidelityProblem", "project_state", "render_fidelity_problems", "scale_problem_step"]
