"""Language-backed materialization for capability-scaled racetrack missions.

The reusable racetrack geometry becomes executable only after a native problem,
mission acceptance contract, and route catalog have all been materialized with
the *same* resolved values.  This module owns that bridge without importing a
qualification tool.  A caller may then run the generated inputs through the
normal runtime and independent evaluator.
"""

from __future__ import annotations

import copy
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .powered_fixed_wing_mission_compiler import CapabilityScaledRacetrack
from .racetrack_template import RACETRACK_FIDELITIES, RacetrackFidelity
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import compile_powered_fixed_wing_racetrack_from_composition, preflight_vehicle_composition

_ROOT = Path(__file__).resolve().parents[2]
_MISSION_CONFIG = _ROOT / "verification" / "family_qualification_missions.yaml"
_RACETRACK_CONFIG = _ROOT / "verification" / "racetrack_templates.yaml"

LANGUAGE_BACKED_RACETRACK_MISSIONS: dict[str, dict[RacetrackFidelity, str]] = {
    "skywalker_x8": {
        "point_mass_3dof": "x8-racetrack-altitude-turns-3dof-v1",
        "pseudo_6dof_kinematic_bridge": "x8-racetrack-altitude-turns-pseudo-6dof-v1",
        "rigid_body_6dof_direct_wrench": "x8-racetrack-altitude-turns-direct-wrench-v1",
        "rigid_body_6dof_surface_allocated": "x8-racetrack-altitude-turns-v1",
    },
    "b747": {
        "point_mass_3dof": "b747-racetrack-altitude-turns-3dof-v1",
        "pseudo_6dof_kinematic_bridge": "b747-racetrack-altitude-turns-pseudo-6dof-v1",
        "rigid_body_6dof_direct_wrench": "b747-racetrack-altitude-turns-6dof-v1",
    },
}


@dataclass(frozen=True, slots=True)
class LanguageBackedRacetrackInputs:
    """Disposable files whose route, objective windows, and horizon agree."""

    proposal: CapabilityScaledRacetrack
    source_mission_id: str
    materialized_mission_id: str
    problem: Path
    mission_config: Path
    racetrack_config: Path


def language_backed_racetrack_mission_id(vehicle_id: str, fidelity: RacetrackFidelity) -> str:
    """Return the canonical source mission for one language-backed route lane."""

    try:
        return LANGUAGE_BACKED_RACETRACK_MISSIONS[vehicle_id][fidelity]
    except KeyError as error:
        raise ValueError(f"{vehicle_id!r} has no language-backed racetrack mission at {fidelity!r}") from error
    ####


def materialize_language_backed_racetrack(
    proposal: CapabilityScaledRacetrack,
    source_mission_id: str,
    directory: Path,
) -> LanguageBackedRacetrackInputs:
    """Write exact disposable inputs without modifying a baseline problem.

    The problem route attributes, route catalog, truth-gate timing windows, and
    integration horizon are all derived from ``proposal``.  A materialization
    failure is therefore a failed integration boundary, never an excuse to run
    a nearby checked-in route.
    """

    if proposal.route.fidelity not in RACETRACK_FIDELITIES:
        raise ValueError(f"unsupported language-backed racetrack fidelity: {proposal.route.fidelity}")
    mission_catalog = yaml.safe_load(_MISSION_CONFIG.read_text(encoding="utf-8"))
    racetrack_catalog = yaml.safe_load(_RACETRACK_CONFIG.read_text(encoding="utf-8"))
    if not isinstance(mission_catalog, dict) or not isinstance(racetrack_catalog, dict):
        raise ValueError("candidate materialization requires mapping-based YAML catalogs")
    missions = mission_catalog.get("missions")
    bindings = racetrack_catalog.get("bindings")
    if not isinstance(missions, list) or not isinstance(bindings, dict):
        raise ValueError("candidate materialization catalogs have unexpected structure")
    source_mission = next(
        (item for item in missions if isinstance(item, dict) and item.get("id") == source_mission_id),
        None,
    )
    if source_mission is None:
        raise KeyError(f"unknown language-backed mission: {source_mission_id}")
    mission = copy.deepcopy(source_mission)
    _retime_mission(mission, proposal)
    mission["id"] = f"{source_mission_id}-candidate"
    source_problem = _ROOT / str(source_mission["problem"])
    source_problem_text = source_problem.read_text(encoding="utf-8")
    step_match = re.search(r"\*integ\b[^\n]*\bdt=([-+0-9.eE]+)", source_problem_text)
    if step_match is None or float(step_match.group(1)) <= 0.0:
        raise ValueError(f"{source_problem} has no positive integration dt for candidate max-step derivation")
    required_steps = math.ceil(proposal.route.horizon_s / float(step_match.group(1))) + 2
    mission["max_steps"] = max(int(mission["max_steps"]), required_steps)
    missions[missions.index(source_mission)] = mission
    bindings[proposal.route.binding_id] = _binding_values(proposal)
    directory.mkdir(parents=True, exist_ok=True)
    problem_path = directory / f"{mission['id']}.prb"
    problem_path.write_text(_replace_route_attributes(source_problem_text, proposal), encoding="utf-8")
    mission_path = directory / "family_qualification_missions.yaml"
    route_path = directory / "racetrack_templates.yaml"
    mission_path.write_text(yaml.safe_dump(mission_catalog, sort_keys=False), encoding="utf-8")
    route_path.write_text(yaml.safe_dump(racetrack_catalog, sort_keys=False), encoding="utf-8")
    return LanguageBackedRacetrackInputs(
        proposal=proposal,
        source_mission_id=source_mission_id,
        materialized_mission_id=str(mission["id"]),
        problem=problem_path,
        mission_config=mission_path,
        racetrack_config=route_path,
    )
    ####


def materialize_powered_fixed_wing_composition(
    composition: CompiledVehicleComposition,
    directory: Path,
) -> LanguageBackedRacetrackInputs:
    """Materialize one supported fixed-wing composition after route preflight."""

    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        diagnostics = "; ".join(preflight.diagnostics) or "no translation-ready geometry"
        raise ValueError(f"cannot materialize composition {composition.id!r}: {preflight.status}: {diagnostics}")
    proposal = compile_powered_fixed_wing_racetrack_from_composition(composition)
    return materialize_language_backed_racetrack(
        proposal,
        language_backed_racetrack_mission_id(composition.vehicle_id, proposal.route.fidelity),
        directory,
    )
    ####


def materialize_x8_composition(composition: CompiledVehicleComposition, directory: Path) -> LanguageBackedRacetrackInputs:
    """Compatibility alias for callers still naming the original X8 slice."""

    if composition.family_id != "skywalker_x8":
        raise ValueError("X8 compatibility materializer requires the Skywalker X8 family")
    return materialize_powered_fixed_wing_composition(composition, directory)
    ####


def _binding_values(proposal: CapabilityScaledRacetrack) -> dict[str, Any]:
    """Serialize only the catalog fields accepted by route resolution."""

    route = proposal.route
    return {
        "vehicle_id": route.vehicle_id,
        "fidelity": route.fidelity,
        "straight_length_m": route.straight_length_m,
        "turn_radius_m": route.turn_radius_m,
        "speed_m_s": route.speed_m_s,
        "low_altitude_m": route.low_altitude_m,
        "high_altitude_m": route.high_altitude_m,
        "climb_rate_m_s": route.climb_rate_m_s,
        "descent_rate_m_s": route.descent_rate_m_s,
        "left_turn_bank_deg": route.left_turn_bank_deg,
        "right_turn_bank_deg": route.right_turn_bank_deg,
        "simulation_margin_s": route.simulation_margin_s,
        "altitude_capture_gain_per_s": route.altitude_capture_gain_per_s,
        "altitude_capture_max_mps": route.altitude_capture_max_mps,
        "position_capture_gain": route.position_capture_gain,
        "position_capture_max_correction_mps": route.position_capture_max_correction_mps,
        "gate_corridor_m": route.gate_corridor_m,
        "gate_altitude_tolerance_m": route.gate_altitude_tolerance_m,
        "gate_speed_tolerance_mps": route.gate_speed_tolerance_mps,
        "position_capture_bank_gain_rad_per_m": route.position_capture_bank_gain_rad_per_m,
        "position_capture_max_bank_correction_deg": route.position_capture_max_bank_correction_deg,
        "turn_rate_command_scale": route.turn_rate_command_scale,
        "source_realization": "capability_scaled_candidate_materialization",
        "status": proposal.status,
    }
    ####


def _replace_route_attributes(problem_text: str, proposal: CapabilityScaledRacetrack) -> str:
    """Replace one route declaration with exact compiled runtime attributes."""

    lines = problem_text.splitlines()
    line_index = next((index for index, line in enumerate(lines) if line.lstrip().startswith("*runtime status route ")), None)
    if line_index is None or sum(line.lstrip().startswith("*runtime status route ") for line in lines) != 1:
        raise ValueError("candidate materialization requires exactly one runtime route declaration")
    attributes = proposal.route.route_attributes()
    tokens = lines[line_index].split()
    updated: list[str] = []
    seen: set[str] = set()
    for token in tokens:
        if "=" not in token:
            updated.append(token)
            continue
        key, _ = token.split("=", 1)
        if key in attributes:
            updated.append(f"{key}={attributes[key]}")
            seen.add(key)
        else:
            updated.append(token)
    missing = sorted(set(attributes).difference(seen))
    if missing:
        raise ValueError(f"runtime route declaration is missing required attributes: {', '.join(missing)}")
    lines[line_index] = " ".join(updated)
    stop_indices = [index for index, line in enumerate(lines) if re.match(r"\s*\*when time>[-+0-9.eE]+ stop\s*$", line)]
    if len(stop_indices) != 1:
        raise ValueError("candidate materialization requires exactly one time-based terminal stop")
    indent = lines[stop_indices[0]][: len(lines[stop_indices[0]]) - len(lines[stop_indices[0]].lstrip())]
    lines[stop_indices[0]] = f"{indent}*when time>{proposal.route.horizon_s:.15g} stop"
    return "\n".join(lines) + "\n"
    ####


def _phase_window(proposal: CapabilityScaledRacetrack, phase: str) -> tuple[float, float]:
    """Return a conservative truth-evaluation window for one route phase."""

    windows = {window.name: window for window in proposal.route.phase_windows}
    window = windows[phase]
    buffer_s = max(12.0, 0.05 * (window.end_s - window.start_s))
    return max(0.0, window.start_s - buffer_s), min(proposal.route.horizon_s, window.end_s + buffer_s)
    ####


def _retime_mission(mission: dict[str, Any], proposal: CapabilityScaledRacetrack) -> None:
    """Derive gate/event windows from candidate phase timing, never old guesses."""

    gates = {gate.id: gate for gate in proposal.route.gates}
    phases = {window.name for window in proposal.route.phase_windows}
    for item in [*mission.get("objectives", ()), mission.get("terminal", {})]:
        if not isinstance(item, dict):
            continue
        phase = item.get("racetrack_phase")
        if isinstance(phase, str) and phase in phases:
            # A response objective is meaningful only while the semantic
            # phase that commands it is active.  Unlike a spatial gate, it
            # must not borrow the conservative pre/post crossing buffer or a
            # level-flight sample can satisfy an upcoming descent objective.
            window = next(window for window in proposal.route.phase_windows if window.name == phase)
            item["window_start_s"], item["window_end_s"] = window.start_s, window.end_s
        gate_id = item.get("racetrack_gate_id")
        if not isinstance(gate_id, str) or gate_id not in gates:
            continue
        start, end = _phase_window(proposal, gates[gate_id].phase)
        if gate_id == "terminal-start-finish-gate":
            end = proposal.route.horizon_s
        item["window_start_s"] = start
        item["window_end_s"] = end
    event_phases = {
        "left": "left-turn",
        "return": "right-turn",
        "right": "right-turn",
        "bank-positive": "right-turn",
        "bank-negative": "left-turn",
        "pitch-positive": "outbound-climb",
        "pitch-negative": "inbound-descent",
    }
    for event in mission.get("truth_events", ()):
        if not isinstance(event, dict):
            continue
        event_id = str(event.get("id", ""))
        phase = next((value for key, value in event_phases.items() if key in event_id), None)
        if phase is not None:
            event["window_start_s"], event["window_end_s"] = _phase_window(proposal, phase)
    mission["racetrack_binding"] = proposal.route.binding_id
    mission["timing_estimate"] = {
        "straight_length_m": proposal.route.straight_length_m,
        "turn_radius_m": proposal.route.turn_radius_m,
        "speed_m_s": proposal.route.speed_m_s,
        "altitude_delta_m": proposal.route.high_altitude_m - proposal.route.low_altitude_m,
        "climb_rate_m_s": proposal.route.climb_rate_m_s,
        "descent_rate_m_s": proposal.route.descent_rate_m_s,
    }
    ####


__all__ = [
    "LANGUAGE_BACKED_RACETRACK_MISSIONS",
    "LanguageBackedRacetrackInputs",
    "language_backed_racetrack_mission_id",
    "materialize_language_backed_racetrack",
    "materialize_powered_fixed_wing_composition",
    "materialize_x8_composition",
]
