#!/usr/bin/env python3
"""Run the shared fixed R1 matrix for the X8 and B747 reduced tiers.

The matrix uses the same mission packet builder and independent truth
evaluator as the nominal cases.  It perturbs only the initial state in a
temporary problem copy; the source problem, tables, route, and evaluator are
otherwise unchanged.  A failed perturbation is retained as boundary evidence
and is never converted into a nominal failure or a release reliability claim.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import tempfile
from pathlib import Path
from typing import Any

import yaml

try:
    from build_family_qualification_packet import build
except ModuleNotFoundError:  # pragma: no cover - package execution path
    from tools.build_family_qualification_packet import build

ROOT = Path(__file__).resolve().parents[1]
MISSION_CONFIG = ROOT / "verification/family_qualification_missions.yaml"
DEFAULT_OUTPUT = ROOT / "verification/alpha3_airbreathing_r1"

MISSION_IDS = {
    "skywalker_x8": {
        "point_mass_3dof": "x8-racetrack-altitude-turns-3dof-v1",
        "pseudo_6dof": "x8-racetrack-altitude-turns-pseudo-6dof-v1",
    },
    "b747": {
        "point_mass_3dof": "b747-racetrack-altitude-turns-3dof-v1",
        "pseudo_6dof": "b747-racetrack-altitude-turns-pseudo-6dof-v1",
    },
}

CASE_SPECS: dict[str, dict[str, float]] = {
    "initial_altitude_plus": {"initial_altitude_offset_m": 50.0},
    "initial_speed_plus": {"initial_speed_scale": 1.05},
    "initial_cross_velocity_plus": {"initial_cross_velocity_m_s": 2.0},
    "initial_mass_plus": {"initial_mass_scale": 1.02},
}


def _mission_table() -> dict[str, dict[str, Any]]:
    payload = yaml.safe_load(MISSION_CONFIG.read_text(encoding="utf-8"))
    assert isinstance(payload, dict)
    missions = payload.get("missions")
    assert isinstance(missions, list)
    return {str(item["id"]): item for item in missions if isinstance(item, dict)}
    ####


def _replace_number(line: str, name: str, value: float) -> str:
    pattern = rf"(\b{re.escape(name)}=)([-+0-9.eE]+)"
    updated, count = re.subn(pattern, rf"\g<1>{value:.15g}", line, count=1)
    if count != 1:
        raise ValueError(f"initial state line does not contain {name}")
    return updated
    ####


def _mutate_problem(source: Path, destination: Path, perturbation: dict[str, float]) -> None:
    lines = source.read_text(encoding="utf-8").splitlines()
    initial_index = next(
        (index for index, line in enumerate(lines) if "*initial ecic" in line or "*initial geodetic" in line),
        None,
    )
    if initial_index is None:
        raise ValueError(f"{source} has no initial ECIC or geodetic state")
    line = lines[initial_index]
    values = {
        "initial_altitude_offset_m": lambda current: current + perturbation["initial_altitude_offset_m"],
        "initial_speed_scale": lambda current: current * perturbation["initial_speed_scale"],
        "initial_cross_velocity_m_s": lambda current: current + perturbation["initial_cross_velocity_m_s"],
        "initial_mass_scale": lambda current: current * perturbation["initial_mass_scale"],
    }
    is_geodetic = "*initial geodetic" in line
    altitude_name = "alt" if is_geodetic else "x"
    speed_name = "vel" if is_geodetic else "ydt"
    if "initial_altitude_offset_m" in perturbation:
        match = re.search(rf"\b{altitude_name}=([-+0-9.eE]+)", line)
        assert match is not None
        line = _replace_number(line, altitude_name, values["initial_altitude_offset_m"](float(match.group(1))))
    if "initial_speed_scale" in perturbation:
        match = re.search(rf"\b{speed_name}=([-+0-9.eE]+)", line)
        assert match is not None
        line = _replace_number(line, speed_name, values["initial_speed_scale"](float(match.group(1))))
    if "initial_cross_velocity_m_s" in perturbation:
        if is_geodetic:
            match = re.search(r"\bvel=([-+0-9.eE]+)", line)
            psi_match = re.search(r"\bpsi=([-+0-9.eE]+)", line)
            assert match is not None and psi_match is not None
            delta_deg = math.degrees(math.atan2(perturbation["initial_cross_velocity_m_s"], float(match.group(1))))
            line = _replace_number(line, "psi", float(psi_match.group(1)) + delta_deg)
        else:
            match = re.search(r"\bxdt=([-+0-9.eE]+)", line)
            assert match is not None
            line = _replace_number(line, "xdt", values["initial_cross_velocity_m_s"](float(match.group(1))))
    if "initial_mass_scale" in perturbation:
        match = re.search(r"\bmass=([-+0-9.eE]+)", line)
        assert match is not None
        line = _replace_number(line, "mass", values["initial_mass_scale"](float(match.group(1))))
    lines[initial_index] = line
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")
    ####


def _first_failure(summary: dict[str, Any]) -> dict[str, object] | None:
    evaluation = summary.get("truth_evaluation", {})
    results = evaluation.get("results", []) if isinstance(evaluation, dict) else []
    for item in results:
        if isinstance(item, dict) and item.get("status") != "pass":
            return {"id": item.get("id"), "status": item.get("status"), "message": item.get("message")}
    if not bool(summary.get("mission_pass")):
        envelope = summary.get("envelope_report", {})
        return {
            "id": "hard_gates",
            "status": "fail",
            "message": "independent envelope or numerical gate failed",
            "envelope_pass": envelope.get("pass") if isinstance(envelope, dict) else None,
        }
    return None
    ####


def run_matrix(output: Path, *, families: tuple[str, ...] = tuple(MISSION_IDS)) -> dict[str, object]:
    """Execute all selected family/tier/case combinations."""

    missions = _mission_table()
    records: list[dict[str, object]] = []
    with tempfile.TemporaryDirectory(prefix="taoryx-airbreathing-r1-") as temporary:
        temporary_root = Path(temporary)
        for family_id in families:
            for fidelity, mission_id in MISSION_IDS[family_id].items():
                mission = missions[mission_id]
                source = ROOT / str(mission["problem"])
                for case_id, perturbation in CASE_SPECS.items():
                    mutated = temporary_root / f"{family_id}-{fidelity}-{case_id}.prb"
                    _mutate_problem(source, mutated, perturbation)
                    case_output = output / f"{family_id}-{fidelity}-{case_id}"
                    command = f"PYTHONPATH=src python3 tools/validate_airbreathing_r1_matrix.py --family {family_id}"
                    build(case_output, mission_id, problem_override=mutated, reproduction_command=command)
                    summary_path = case_output / mission_id / "summary.json"
                    summary = json.loads(summary_path.read_text(encoding="utf-8"))
                    packet_path = summary_path.parent
                    packet_reference = (
                        packet_path.relative_to(ROOT).as_posix()
                        if packet_path.is_relative_to(ROOT)
                        else str(packet_path)
                    )
                    records.append(
                        {
                            "family_id": family_id,
                            "fidelity": fidelity,
                            "case_id": case_id,
                            "perturbation": perturbation,
                            "mission_pass": bool(summary.get("mission_pass")),
                            "numerical_valid": bool(summary.get("numerical_valid")),
                            "required_objectives": summary.get("truth_evaluation", {}).get("required_objectives"),
                            "required_passed": summary.get("truth_evaluation", {}).get("required_passed"),
                            "first_failure": _first_failure(summary),
                            "packet": packet_reference,
                        }
                    )
    passed = sum(bool(record["mission_pass"]) for record in records)
    report: dict[str, object] = {
        "schema": "taoryx.airbreathing-r1-matrix/v1alpha1",
        "status": "R1_fixed_matrix_complete",
        "claim": "Fixed initial-state perturbation evidence for the X8 and B747 point-mass and pseudo-6DOF racetrack realizations.",
        "claim_boundary": "This is a deterministic development R1 matrix, not a statistical reliability claim, wind robustness claim, physical-surface qualification, or full-envelope aircraft qualification.",
        "case_count": len(records),
        "passed_case_count": passed,
        "failed_case_count": len(records) - passed,
        "families": list(families),
        "fidelities": ["point_mass_3dof", "pseudo_6dof"],
        "cases": records,
        "perturbation_contract": {
            "source_problem_unchanged_except_initial_state": True,
            "temporary_problem_policy": "mutated copies are retained inside each packet input manifest; source files are not modified",
            "dimensions": list(CASE_SPECS),
            "failure_policy": "retain and classify each failed objective; never convert boundary failure to pass",
        },
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(
        "PYTHONPATH=src python3 tools/validate_airbreathing_r1_matrix.py\n",
        encoding="utf-8",
    )
    return report
    ####


def main() -> int:
    """Run and write the air-breathing R1 matrix."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--family", choices=("all", *MISSION_IDS), default="all")
    arguments = parser.parse_args()
    families = tuple(MISSION_IDS) if arguments.family == "all" else (arguments.family,)
    report = run_matrix(arguments.output, families=families)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
