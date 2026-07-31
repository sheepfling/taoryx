"""Execute candidate shared parity windows and write a conservative report."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import tempfile
import zipfile
from pathlib import Path
from typing import Any

import yaml

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files
from taoryx.validation import continuity_audit

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "verification/fidelity_parity.yaml"
####


def _bridge_problem(source: Path, destination: Path) -> Path:
    lines = source.read_text(encoding="utf-8").splitlines()
    title_index = next(index for index, line in enumerate(lines) if line.startswith("*title"))
    lines[title_index + 1:title_index + 1] = [
        "*mode kinematic-6dof",
        "*runtime status attitude mode=lag roll-deg=0 pitch-deg=0 yaw-deg=0 lag-s=0.25 max-rate-deg-s=360",
    ]
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return destination
####


def _scaled_problem(source: Path, destination: Path, factor: float) -> Path:
    """Render a deterministic step-refined copy without changing events."""

    text = source.read_text(encoding="utf-8")
    text = re.sub(
        r"(\bdt=)([0-9.eE+-]+)",
        lambda match: f"{match.group(1)}{float(match.group(2)) * factor:.16g}",
        text,
    )
    destination.write_text(text, encoding="utf-8")
    return destination
####


def _timed_problem(source: Path, destination: Path, duration_s: float) -> Path:
    """Render a metadata-selected long reduction window."""

    if duration_s <= 0.0:
        raise ValueError("reduction duration must be positive")
    text = source.read_text(encoding="utf-8")
    text, replacements = re.subn(
        r"\*when time>[0-9.eE+-]+ stop",
        f"*when time>{duration_s:.16g} stop",
        text,
    )
    if replacements != 1:
        raise ValueError(f"expected one time stop condition in {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(text, encoding="utf-8")
    return destination
####


def _canonical_history(report: Any, tier: str, state_mapping: dict[str, Any]) -> list[dict[str, float]]:
    """Extract the shared translational state in canonical SI units."""

    if not report.results:
        return []
    history = report.results[0].states["1"]
    mapping = {name: spec[tier] for name, spec in state_mapping.items()}
    scales = {name: float(state_mapping[name][f"{tier}_scale"]) for name in state_mapping}
    def value(state: Any, canonical: str, native: str) -> float:
        raw = state.named.get(native)
        if raw is None:
            raw = state.named[canonical]
        return float(raw)
    ####

    return [
        {
            "time_s": float(state.time),
            "altitude_m": value(state, "altitude_m", mapping["altitude_m"]) * scales["altitude_m"],
            "speed_m_s": value(state, "speed_m_s", mapping["speed_m_s"]) * scales["speed_m_s"],
            "mass_kg": value(state, "mass_kg", mapping["mass_kg"]) * scales["mass_kg"],
        }
        for state in history
    ]
####


def _run(problem: Path, tables: tuple[Path, ...], output: Path, tier: str, state_mapping: dict[str, Any], *, max_steps: int, integrator: str | None) -> dict[str, Any]:
    report = run_files(problem, tables, output_dir=output, max_steps=max_steps, integrator=integrator, profile=GrammarProfile.TAORYX)
    try:
        problem_name = str(problem.relative_to(ROOT))
    except ValueError:
        problem_name = str(problem)
    result: dict[str, Any] = {
        "problem": problem_name,
        "exit_code": report.exit_code,
        "completed": bool(report.results and all(item.completed for item in report.results)),
        "diagnostics": [{"code": item.code, "message": item.message} for item in report.diagnostics],
        "durations_s": [],
        "sample_counts": [],
    }
    for artifact in report.artifacts:
        vehicle = next(iter(artifact.vehicles.values()))
        result["durations_s"].append(float(vehicle.times[-1] - vehicle.times[0]))
        result["sample_counts"].append(len(vehicle.times))
    result["canonical_history"] = _canonical_history(report, tier, state_mapping)
    result["continuity"] = {"passed": True} if len(result["canonical_history"]) < 2 else None
    return result
####


def _final_difference(base: list[dict[str, float]], refined: list[dict[str, float]]) -> dict[str, float] | None:
    """Compare final canonical states from two step sizes."""

    if not base or not refined:
        return None
    left = base[-1]
    right = refined[-1]
    return {
        channel: abs(left[channel] - right[channel])
        for channel in ("altitude_m", "speed_m_s", "mass_kg")
    }
####


def _history_difference(
    left_history: list[dict[str, float]],
    right_history: list[dict[str, float]],
) -> dict[str, float | None]:
    """Return maximum canonical difference over the aligned common window."""

    paired = list(zip(left_history, right_history, strict=False))
    return {
        channel: max(abs(left[channel] - right[channel]) for left, right in paired) if paired else None
        for channel in ("altitude_m", "speed_m_s", "mass_kg")
    }
    ####


def run(catalog_path: Path = CATALOG, output: Path | None = None, *, family_ids: tuple[str, ...] | None = None, max_steps: int = 10000, integrator: str | None = None) -> Path:
    payload = yaml.safe_load(catalog_path.read_text(encoding="utf-8"))
    destination = output or ROOT / "artifacts/verification/fidelity_parity_v1"
    destination.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, Any]] = []
    with tempfile.TemporaryDirectory(prefix="taoryx-parity-") as temporary:
        temporary_root = Path(temporary)
        for family in payload["families"]:
            family_id = str(family["id"])
            if family_ids is not None and family_id not in family_ids:
                continue
            tables = tuple(ROOT / str(path) for path in family["source_tables"])
            record: dict[str, Any] = {
                "id": family_id,
                "scenario_id": family["scenario_id"],
                "catalog_status": family["status"],
                "reduction_kind": family["reduction_kind"],
                "bridge_kind": "3dof+attitude-reconstruction",
                "parity_tolerances": family["parity_tolerances"],
                "continuity_tolerances": family["continuity_tolerances"],
                "unit_profiles": {"point_mass": family["point_output_units"], "rigid_body": family["rigid_output_units"]},
                "execution_policy": {"max_steps": max_steps, "integrator": integrator or "problem_default"},
            }
            if family["status"] != "candidate":
                record["status"] = "blocked"
                record["blocker"] = family.get("blocker")
                records.append(record)
                continue
            point_source = ROOT / str(family["point_mass_problem"])
            rigid = ROOT / str(family["constrained_rigid_problem"])
            reduction_duration_s = float(family.get("reduction_duration_s", family["duration_s"]))
            generated_input_dir = destination / family_id / "generated-inputs"
            point = _timed_problem(
                point_source,
                generated_input_dir / f"{family_id}-reduction-{reduction_duration_s:g}s.prb",
                reduction_duration_s,
            )
            bridge = _bridge_problem(
                point,
                generated_input_dir / f"{family_id}-bridge-{reduction_duration_s:g}s.prb",
            )
            record["runs"] = {
                "3dof": _run(point, tables, destination / family_id / "3dof", "point", family["state_mapping"], max_steps=max_steps, integrator=integrator),
                "bridge": _run(bridge, tables, destination / family_id / "bridge", "point", family["state_mapping"], max_steps=max_steps, integrator=integrator),
                "6dof": _run(rigid, tables, destination / family_id / "6dof", "rigid", family["state_mapping"], max_steps=max_steps, integrator=integrator),
            }
            record["reduction_duration_s"] = reduction_duration_s
            record["generated_inputs"] = [
                str(path.relative_to(destination)) for path in (point, bridge)
            ]
            for tier in ("3dof", "bridge", "6dof"):
                history = record["runs"][tier]["canonical_history"]
                record["runs"][tier]["continuity"] = continuity_audit(history, family["continuity_tolerances"]) if len(history) >= 2 else {"passed": False, "reason": "insufficient samples"}
            point_history = record["runs"]["3dof"]["canonical_history"]
            bridge_history = record["runs"]["bridge"]["canonical_history"]
            rigid_history = record["runs"]["6dof"]["canonical_history"]
            differences = _history_difference(point_history, bridge_history)
            rigid_window_differences = _history_difference(point_history, rigid_history)
            record["initial_state_si"] = point_history[0] if point_history else None
            record["initial_state_difference"] = {
                channel: abs(point_history[0][channel] - rigid_history[0][channel])
                for channel in ("altitude_m", "speed_m_s", "mass_kg")
            } if point_history and rigid_history else None
            record["history_max_difference"] = differences
            record["rigid_window_max_difference"] = rigid_window_differences
            continuity_passed = all(record["runs"][tier]["continuity"].get("passed", False) for tier in ("3dof", "bridge", "6dof"))
            tolerance = family["parity_tolerances"]
            difference_values_list: list[float] = []
            for channel in ("altitude_m", "speed_m_s", "mass_kg"):
                value = differences.get(channel)
                if value is not None:
                    difference_values_list.append(float(value))
            difference_values = tuple(difference_values_list)
            parity_passed = (
                differences
                and continuity_passed
                and len(difference_values) == 3
                and all(
                    value <= float(tolerance[channel])
                    for value, channel in zip(
                        difference_values,
                        ("altitude_m", "speed_m_s", "mass_kg"),
                        strict=True,
                    )
                )
            )
            record["parity_gate"] = "pass_translational_window" if parity_passed else "blocked_translational_or_continuity_mismatch"
            refined_point = _scaled_problem(point, temporary_root / f"{family_id}-point-half.prb", 0.5)
            refined_bridge = _scaled_problem(bridge, temporary_root / f"{family_id}-bridge-half.prb", 0.5)
            refined_rigid = _scaled_problem(rigid, temporary_root / f"{family_id}-rigid-half.prb", 0.5)
            refined_point_run = _run(refined_point, tables, destination / family_id / "3dof-half", "point", family["state_mapping"], max_steps=max_steps, integrator=integrator)
            refined_bridge_run = _run(refined_bridge, tables, destination / family_id / "bridge-half", "point", family["state_mapping"], max_steps=max_steps, integrator=integrator)
            refined_rigid_run = _run(refined_rigid, tables, destination / family_id / "6dof-half", "rigid", family["state_mapping"], max_steps=max_steps, integrator=integrator)
            record["convergence"] = {
                "base_step": {"3dof": record["runs"]["3dof"]["sample_counts"], "bridge": record["runs"]["bridge"]["sample_counts"], "6dof": record["runs"]["6dof"]["sample_counts"]},
                "half_step": {"3dof": refined_point_run["sample_counts"], "bridge": refined_bridge_run["sample_counts"], "6dof": refined_rigid_run["sample_counts"]},
                "final_difference": {
                    "3dof": _final_difference(record["runs"]["3dof"]["canonical_history"], refined_point_run["canonical_history"]),
                    "bridge": _final_difference(record["runs"]["bridge"]["canonical_history"], refined_bridge_run["canonical_history"]),
                    "6dof": _final_difference(record["runs"]["6dof"]["canonical_history"], refined_rigid_run["canonical_history"]),
                },
            }
            record["status"] = "executed"
            records.append(record)
    report_path = destination / "report.json"
    report_path.write_text(json.dumps({"schema_version": 1, "families": records}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report_path
####


def package_report(report_path: Path, output: Path) -> Path:
    """Package only the scoped reduction-parity evidence and its inputs."""

    report_path = report_path.resolve()
    catalog = ROOT / "verification/fidelity_parity.yaml"
    contract = ROOT / "verification/generated/fidelity_parity_contract.json"
    payload = yaml.safe_load(catalog.read_text(encoding="utf-8"))
    files = [report_path, catalog]
    if contract.is_file():
        files.append(contract)
    for family in payload["families"]:
        files.extend(ROOT / str(family[key]) for key in ("point_mass_problem", "constrained_rigid_problem"))
        files.extend(ROOT / str(path) for path in family["source_tables"])
    generated_inputs = [
        report_path.parent / str(path)
        for family in json.loads(report_path.read_text(encoding="utf-8"))["families"]
        for path in family.get("generated_inputs", ())
    ]
    files.extend(generated_inputs)
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "claim": "metadata-generated long 3DOF-to-kinematic-bridge reduction parity plus rigid-body diagnostic windows",
        "claim_boundary": "not source validation, free-rigid-body parity, controller success, or mission completion",
        "files": {},
    }
    archive = output.with_suffix(".zip")
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as handle:
        for path in sorted(set(files)):
            name = "report.json" if path == report_path else str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path.relative_to(report_path.parent))
            handle.write(path, name)
            manifest["files"][name] = hashlib.sha256(path.read_bytes()).hexdigest()
        handle.writestr("manifest.json", json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    return archive
####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--families", help="comma-separated family IDs to run")
    parser.add_argument("--max-steps", type=int, default=10000)
    parser.add_argument("--integrator", choices=("euler", "rk4", "rkf45", "scipy-rk45", "scipy-dop853"))
    parser.add_argument("--zip", action="store_true", help="also write a scoped reduction-parity evidence ZIP")
    arguments = parser.parse_args()
    family_ids = tuple(item.strip() for item in arguments.families.split(",") if item.strip()) if arguments.families else None
    if arguments.max_steps <= 0:
        parser.error("--max-steps must be positive")
    report = run(output=arguments.output, family_ids=family_ids, max_steps=arguments.max_steps, integrator=arguments.integrator)
    print(package_report(report, report.parent / "fidelity-parity-evidence") if arguments.zip else report)
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
####
