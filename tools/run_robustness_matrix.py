"""Run the bounded, paired vehicle verification matrix."""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import re
import shutil
import uuid
from pathlib import Path
from typing import Any

import yaml

from taoryx.language import GrammarProfile
from taoryx.runtime.runner import RunReport, run_files
from taoryx.visualization import render_run_artifact_plots

ROOT = Path(__file__).resolve().parents[1]
MATRIX = ROOT / "verification/acceptance/robustness_matrix_v1.yaml"
FIXTURE_TABLES = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
FAMILY_ROOT = ROOT / "examples/mission_families"
_NUMBER = r"[-+]?\d+(?:\.\d*)?(?:[Ee][-+]?\d+)?"
####


def _load_matrix() -> dict[str, Any]:
    payload = yaml.safe_load(MATRIX.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        raise ValueError(f"unsupported robustness matrix: {MATRIX}")
    return payload
####


def _sha256(path: Path) -> str:
    """Return the content hash used to bind an artifact to its inputs."""

    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
####


def _portable_path(path: str | Path) -> str:
    """Represent generated paths relative to the repository root."""

    return str(Path(path).resolve().relative_to(ROOT.resolve()))
####


def _replace_parameter(text: str, name: str, value: float, *, occurrence: int | None = None) -> str:
    pattern = re.compile(rf"(\b{re.escape(name)}\s*=\s*)({_NUMBER})")
    count = 0

    def replace(match: re.Match[str]) -> str:
        nonlocal count
        count += 1
        if occurrence is not None and count != occurrence:
            return match.group(0)
        return f"{match.group(1)}{value:.12g}"
    ####

    return pattern.sub(replace, text)
####


def _variant_problem(source: Path, *, speed_scale: float, altitude_delta_m: float, mass_scale: float, dt_scale: float = 1.0) -> str:
    """Render a bounded variant by changing only canonical initial/integrator values."""

    text = source.read_text(encoding="utf-8")
    initial = re.search(r"^\s*\*initial\b.*$", text, re.MULTILINE)
    if initial is None:
        raise ValueError(f"problem has no *initial line: {source}")
    line = initial.group(0)
    is_point_mass = " vel=" in line
    if is_point_mass:
        velocity = float(re.search(rf"\bvel\s*=\s*({_NUMBER})", line).group(1))
        altitude = float(re.search(rf"\balt\s*=\s*({_NUMBER})", line).group(1))
        mass = float(re.search(rf"\bmass\s*=\s*({_NUMBER})", line).group(1))
        replacement = _replace_parameter(line, "vel", velocity * speed_scale)
        replacement = _replace_parameter(replacement, "alt", altitude + altitude_delta_m)
        replacement = _replace_parameter(replacement, "mass", mass * mass_scale)
    else:
        radius_coordinate = float(re.search(rf"\bx\s*=\s*({_NUMBER})", line).group(1))
        mass = float(re.search(rf"\bmass\s*=\s*({_NUMBER})", line).group(1))
        velocity_names = ("xdt", "ydt", "zdt")
        velocity_values = [float(re.search(rf"\b{name}\s*=\s*({_NUMBER})", line).group(1)) for name in velocity_names]
        replacement = line
        replacement = _replace_parameter(replacement, "x", radius_coordinate + altitude_delta_m)
        replacement = _replace_parameter(replacement, "mass", mass * mass_scale)
        for name, value in zip(velocity_names, velocity_values, strict=True):
            replacement = _replace_parameter(replacement, name, value * speed_scale)
    ####
    text = text.replace(line, replacement, 1)
    text = re.sub(r"(\bdt\s*=\s*)(" + _NUMBER + r")", lambda match: f"{match.group(1)}{float(match.group(2)) * dt_scale:.12g}", text, count=1)
    return text
####


def _within_envelope(family: dict[str, Any], speed: float, altitude: float) -> tuple[bool, str | None]:
    envelope = family["envelope"]
    speed_range = envelope.get("speed_m_s") or envelope.get("body_velocity_m_s")
    altitude_range = envelope.get("altitude_m")
    if speed_range is not None and not (float(speed_range[0]) <= speed <= float(speed_range[1])):
        return False, "speed_outside_declared_envelope"
    if altitude_range is not None and not (float(altitude_range[0]) <= altitude <= float(altitude_range[1])):
        return False, "altitude_outside_declared_envelope"
    return True, None
####


def _histories(report: RunReport) -> list[Any]:
    return [state for result in report.results for history in result.states.values() for state in history]
####


def _finite(history: list[Any]) -> bool:
    return all(math.isfinite(float(value)) for state in history for value in state.values)
####


def _metrics(report: RunReport, model: str) -> dict[str, Any]:
    history = _histories(report)
    initial = history[0] if history else None
    final = history[-1] if history else None
    quaternion_error = None
    if final is not None and all(name in final.named for name in ("qw", "qx", "qy", "qz")):
        quaternion_error = abs(sum(float(final.named[name]) ** 2 for name in ("qw", "qx", "qy", "qz")) - 1.0)
    def canonical_state(state: Any) -> dict[str, float]:
        if state is None:
            return {}
        named = state.named
        if model == "3dof":
            length_scale = 0.3048
            speed_scale = 0.3048
            mass_scale = 0.45359237
            return {
                "x_m": float(named.get("x", 0.0)) * length_scale,
                "y_m": float(named.get("y", 0.0)) * length_scale,
                "z_m": float(named.get("z", 0.0)) * length_scale,
                "vx_m_s": float(named.get("xdt", 0.0)) * speed_scale,
                "vy_m_s": float(named.get("ydt", 0.0)) * speed_scale,
                "vz_m_s": float(named.get("zdt", 0.0)) * speed_scale,
                "altitude_m": float(named.get("alt", 0.0)) * length_scale,
                "speed_m_s": float(named.get("vel", 0.0)) * speed_scale,
                "mass_kg": float(named.get("mass", 0.0)) * mass_scale,
            }
        return {
            "x_m": float(named.get("x", 0.0)),
            "y_m": float(named.get("y", 0.0)),
            "z_m": float(named.get("z", 0.0)),
            "vx_m_s": float(named.get("vx", 0.0)),
            "vy_m_s": float(named.get("vy", 0.0)),
            "vz_m_s": float(named.get("vz", 0.0)),
            "altitude_m": float(named.get("altitude_m", 0.0)),
            "speed_m_s": float(named.get("speed_m_s", 0.0)),
            "mass_kg": float(named.get("mass", 0.0)),
        }
    ####

    return {
        "model": model,
        "units_profile": "taos-fps-converted-to-si" if model == "3dof" else "si",
        "exit_code": report.exit_code,
        "completed": bool(report.results) and all(result.completed for result in report.results),
        "finite": _finite(history),
        "state_count": len(history),
        "final_time_s": None if final is None else float(final.time),
        "final": {} if final is None else {name: float(final.named[name]) for name in ("x", "y", "z", "vx", "vy", "vz", "mass", "altitude_m", "speed_m_s") if name in final.named},
        "quaternion_error": quaternion_error,
        "aero_active": None if final is None else float(final.named.get("aero_active", 0.0)),
        "actuator_saturation_max": max((float(state.named.get("attitude_controller_saturated", 0.0)) for state in history), default=0.0),
        "diagnostics": [{"code": item.code, "message": item.message} for item in report.diagnostics],
        "initial_canonical_si": canonical_state(initial),
        "final_canonical_si": canonical_state(final),
    }
####


def _render_report_plots(report: RunReport, directory: Path) -> list[str]:
    channels = (
        "position.altitude.geodetic",
        "kinematics.speed",
        "mass.total",
        "aero.airspeed",
        "aero.dynamic_pressure",
        "aero.force.body.x",
        "aero.force.body.y",
        "aero.force.body.z",
        "aero.moment.body.x",
        "aero.moment.body.y",
        "aero.moment.body.z",
        "thermal.heat-rate",
        "guidance.pro-nav-acceleration",
        "guidance.pro-nav-achieved-aero-acceleration",
        "attitude.roll",
        "attitude.pitch",
        "attitude.yaw",
    )
    rendered: list[str] = []
    for index, artifact in enumerate(report.artifacts):
        rendered.extend(str(path) for path in render_run_artifact_plots(artifact, directory / f"vehicle-{index + 1}", vehicle_id="1", channels=channels))
    return rendered
####


def _classify(pair: dict[str, Any], *, six: dict[str, Any], envelope_ok: bool) -> str | None:
    if not envelope_ok:
        return "envelope"
    if pair["three_dof"]["exit_code"] != 0 or six["exit_code"] != 0:
        return "parse" if any(item["code"].endswith("ingest-failed") or "missing-runtime-table" in item["code"] for item in six["diagnostics"]) else "runtime"
    if not pair["three_dof"]["completed"] or not six["completed"]:
        return "incomplete"
    if not six["finite"]:
        return "nonfinite"
    if six["final"].get("mass", 1.0) <= 0.0:
        return "mass"
    if six["quaternion_error"] is not None and six["quaternion_error"] > 1.0e-9:
        return "quaternion"
    if six["aero_active"] != 1.0:
        return "aero_inactive"
    if six["actuator_saturation_max"] > 0.0:
        return "actuator"
    if six["diagnostics"]:
        return "diagnostic"
    return None
####


def _runtime_rejected_outside_envelope(six: dict[str, Any]) -> bool:
    """Recognize a deliberate no-extrapolation stop from a coefficient table."""

    return six["exit_code"] != 0 and any("outside its declared envelope" in item["message"] for item in six["diagnostics"])
####


def _is_envelope_rejection(case: dict[str, Any]) -> bool:
    """Return whether a case was intentionally excluded by an envelope gate."""

    return str(case.get("status", "")).endswith("out_of_envelope") or str(case.get("status", "")).endswith("outside_declared_envelope")
####


def _run_case(family: dict[str, Any], variant: dict[str, float], output_root: Path, *, render_plots: bool = False) -> dict[str, Any]:
    directory = FAMILY_ROOT / family["directory"]
    problem_3dof = directory / family["problem_3dof"]
    problem_6dof = directory / family["problem_6dof"]
    source_text = problem_3dof.read_text(encoding="utf-8")
    initial = re.search(r"^\s*\*initial\b.*$", source_text, re.MULTILINE).group(0)
    speed = float(re.search(rf"\bvel\s*=\s*({_NUMBER})", initial).group(1)) * variant["speed_scale"]
    altitude = float(re.search(rf"\balt\s*=\s*({_NUMBER})", initial).group(1)) + variant["altitude_delta_m"]
    envelope_ok, envelope_reason = _within_envelope(family, speed, altitude)
    run_uuid = str(uuid.uuid4())
    record: dict[str, Any] = {
        "run_uuid": run_uuid,
        "variant": variant,
        "envelope_ok": envelope_ok,
        "envelope_reason": envelope_reason,
        "source_problem_sha256": _sha256(problem_3dof),
        "source_6dof_problem_sha256": _sha256(problem_6dof),
    }
    if not envelope_ok:
        record["status"] = "preflight_out_of_envelope"
        return record
    rendered_3 = _variant_problem(problem_3dof, **variant)
    rendered_6 = _variant_problem(problem_6dof, **variant)
    variant_dir = output_root / family["id"] / ("s" + str(variant["speed_scale"]).replace(".", "" ) + "_a" + str(variant["altitude_delta_m"]).replace(".", "m") + "_m" + str(variant["mass_scale"]).replace(".", ""))
    if variant_dir.exists():
        shutil.rmtree(variant_dir)
    variant_dir.mkdir(parents=True, exist_ok=True)
    problem_3_variant = variant_dir / problem_3dof.name
    problem_6_variant = variant_dir / problem_6dof.name
    problem_3_variant.write_text(rendered_3, encoding="utf-8")
    problem_6_variant.write_text(rendered_6, encoding="utf-8")
    tables = tuple(FIXTURE_TABLES / name for name in family["tables_6dof"])
    (variant_dir / "case_manifest.json").write_text(
        json.dumps(
            {
                **record,
                "resolved_problem_sha256": hashlib.sha256(rendered_3.encode()).hexdigest(),
                "resolved_6dof_problem_sha256": hashlib.sha256(rendered_6.encode()).hexdigest(),
                "table_sha256": {name: _sha256(path) for name, path in zip(family["tables_6dof"], tables, strict=True)},
                "paths": {"problem_3dof": problem_3_variant.name, "problem_6dof": problem_6_variant.name},
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    max_steps = int(family.get("max_steps", 200))
    baseline = run_files(problem_3_variant, output_dir=variant_dir / "3dof", max_steps=max_steps, profile=GrammarProfile.TAORYX)
    six = run_files(problem_6_variant, tables, output_dir=variant_dir / "6dof", max_steps=max_steps, integrator="euler", profile=GrammarProfile.TAORYX)
    if render_plots:
        record["plots"] = {
            model: [_portable_path(path) for path in paths]
            for model, paths in (
                ("3dof", _render_report_plots(baseline, variant_dir / "plots" / "3dof")),
                ("6dof", _render_report_plots(six, variant_dir / "plots" / "6dof")),
            )
        }
    record["three_dof"] = _metrics(baseline, "3dof")
    record["six_dof"] = _metrics(six, "6dof")
    (variant_dir / "initial_condition_audit.json").write_text(
        json.dumps(
            {
                "run_uuid": run_uuid,
                "three_dof": record["three_dof"]["initial_canonical_si"],
                "six_dof": record["six_dof"]["initial_canonical_si"],
                "units_profiles": {
                    "three_dof": record["three_dof"]["units_profile"],
                    "six_dof": record["six_dof"]["units_profile"],
                },
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    if _runtime_rejected_outside_envelope(record["six_dof"]):
        record["failure_class"] = "envelope"
        record["status"] = "initial_table_query_out_of_envelope"
        return record
    record["failure_class"] = _classify(record, six=record["six_dof"], envelope_ok=True)
    record["status"] = "pass" if record["failure_class"] is None else "fail"
    return record
####


def _variants(family: dict[str, Any]) -> list[dict[str, float]]:
    axes = family["variations"]
    keys = ("speed_scale", "altitude_delta_m", "mass_scale")
    return [dict(zip(keys, values, strict=True)) for values in itertools.product(*(map(float, axes[key]) for key in keys))]
####


def _nominal_case(cases: list[dict[str, Any]]) -> dict[str, Any] | None:
    for case in cases:
        variant = case.get("variant", {})
        if variant == {"speed_scale": 1.0, "altitude_delta_m": 0.0, "mass_scale": 1.0}:
            return case
    return None
####


def _worst_case(cases: list[dict[str, Any]]) -> dict[str, Any]:
    evaluated = [case for case in cases if not _is_envelope_rejection(case)]
    if not evaluated:
        return {"case": None}
    worst = max(evaluated, key=lambda case: (case.get("six_dof", {}).get("quaternion_error") or 0.0, -(case.get("six_dof", {}).get("final_time_s") or 0.0)))
    return {
        "case": worst["variant"],
        "failure_class": worst.get("failure_class"),
        "quaternion_error": worst.get("six_dof", {}).get("quaternion_error"),
        "final_time_s": worst.get("six_dof", {}).get("final_time_s"),
    }
####


def _convergence(family: dict[str, Any], output_root: Path) -> dict[str, Any]:
    directory = FAMILY_ROOT / family["directory"]
    source = directory / family["problem_6dof"]
    half = _variant_problem(source, speed_scale=1.0, altitude_delta_m=0.0, mass_scale=1.0, dt_scale=0.5)
    path = output_root / family["id"] / "convergence_6dof.prb"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(half, encoding="utf-8")
    tables = tuple(FIXTURE_TABLES / name for name in family["tables_6dof"])
    integrator = str(_load_matrix()["gates"]["convergence"].get("integrator", "euler"))
    max_steps = int(family.get("max_steps", 200))
    base = run_files(source, tables, output_dir=path.parent / "base", max_steps=max_steps, integrator=integrator, profile=GrammarProfile.TAORYX)
    refined = run_files(path, tables, output_dir=path.parent / "half_step", max_steps=max_steps * 2, integrator=integrator, profile=GrammarProfile.TAORYX)
    base_final = _histories(base)[-1] if _histories(base) else None
    refined_final = _histories(refined)[-1] if _histories(refined) else None
    differences = {name: None for name in ("position_m", "velocity_m_s", "attitude_rad")}
    if base_final is not None and refined_final is not None:
        differences["position_m"] = math.sqrt(sum((float(base_final.named.get(name, 0.0)) - float(refined_final.named.get(name, 0.0))) ** 2 for name in ("x", "y", "z")))
        differences["velocity_m_s"] = math.sqrt(sum((float(base_final.named.get(name, 0.0)) - float(refined_final.named.get(name, 0.0))) ** 2 for name in ("vx", "vy", "vz")))
        differences["attitude_rad"] = math.sqrt(sum((float(base_final.named.get(name, 0.0)) - float(refined_final.named.get(name, 0.0))) ** 2 for name in ("qw", "qx", "qy", "qz")))
    tolerances = _load_matrix()["gates"]["convergence"]["tolerances"]
    passed = all(differences[name] is not None and differences[name] <= float(tolerances[name]) for name in differences)
    return {"status": "pass" if passed else "fail", "integrator": integrator, "differences": differences, "tolerances": tolerances}
####


def _markdown(payload: dict[str, Any]) -> str:
    lines = ["# Robustness matrix report", "", f"Status: **{payload['status']}**", "", "Claims: engineering validity unproven; historical TAOS 96 compatibility unproven.", "", "| Family | Nominal | Evaluated | Pass | Pass rate | Convergence |", "|---|---|---:|---:|---:|---|"]
    for family in payload["families"]:
        lines.append(f"| {family['id']} | {'pass' if family['nominal_pass'] else 'fail'} | {family['evaluated']} | {family['passed']} | {family['pass_rate']:.3f} | {family['convergence']['status']} |")
    lines.extend(["", "CA-HI: **evidence-only**; no endpoint claim or independent route oracle was declared.", "", "Failure classes are recorded in the JSON report; out-of-envelope variants are not counted in the bounded pass rate."])
    return "\n".join(lines) + "\n"
####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "artifacts/verification/robustness_matrix_v1")
    parser.add_argument("--plots", action="store_true", help="render Matplotlib PNG artifacts for every evaluated pair")
    args = parser.parse_args()
    matrix = _load_matrix()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    family_reports: list[dict[str, Any]] = []
    for family in matrix["families"]:
        if family.get("status") == "evidence_only":
            continue
        family_output = args.output_dir / "cases" / family["id"]
        if family_output.exists():
            shutil.rmtree(family_output)
        cases = [_run_case(family, variant, args.output_dir / "cases", render_plots=args.plots) for variant in _variants(family)]
        evaluated = [case for case in cases if not _is_envelope_rejection(case)]
        passed = sum(case.get("status") == "pass" for case in evaluated)
        convergence = _convergence(family, args.output_dir / "convergence")
        nominal = _nominal_case(cases)
        nominal_pass = nominal is not None and nominal.get("status") == "pass"
        family_reports.append({"id": family["id"], "vehicle": family["vehicle"], "cases": cases, "evaluated": len(evaluated), "passed": passed, "pass_rate": passed / len(evaluated) if evaluated else 0.0, "nominal_pass": nominal_pass, "convergence": convergence, "worst_case": _worst_case(cases)})
    robustness_pass = all(report["evaluated"] >= 9 and report["pass_rate"] >= 0.95 for report in family_reports)
    nominal_pass = all(report["nominal_pass"] for report in family_reports)
    convergence_pass = all(report["convergence"]["status"] == "pass" for report in family_reports)
    payload = {"matrix": matrix["id"], "status": "pass" if nominal_pass and robustness_pass and convergence_pass else "fail", "gates": {"nominal": nominal_pass, "bounded_robustness": robustness_pass, "convergence": convergence_pass}, "claim_boundary": matrix["claim_boundary"], "families": family_reports, "cahi": {"status": "evidence_only", "reason": "endpoint requirement and independent route oracle not declared"}}
    (args.output_dir / "report.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (args.output_dir / "report.md").write_text(_markdown(payload), encoding="utf-8")
    print(f"Wrote {args.output_dir / 'report.json'}")
    print(f"Overall matrix status: {payload['status']}")
    return 0 if payload["status"] == "pass" else 1
####


if __name__ == "__main__":
    raise SystemExit(main())
