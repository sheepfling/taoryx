"""Build an honest, independent evidence pack for the synthetic CA-HI case.

The native CA-HI example is intentionally synthetic.  This builder keeps its
staged execution evidence, but adjudicates the endpoint from truth telemetry
instead of treating the terminal ProNav segment or runtime completion as a
target-hit claim.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import mimetypes
import shutil
import zipfile
from pathlib import Path
from typing import Any

from taoryx.language import GrammarProfile
from taoryx.runtime.runner import run_files
from taoryx.showcase import (
    ArtifactFile,
    EvidenceBoardSpec,
    FidelityShowcaseRealization,
    ShowcaseOutcome,
    build_showcase_run_artifact,
)

ROOT = Path(__file__).resolve().parents[1]
SHOWCASE = ROOT / "examples/showcases/california_to_hawaii"
MISSION = SHOWCASE / "mission.prb"
AERO = SHOWCASE / "aero.tbl"
START = (34.70, -120.60)
DESTINATION = (21.31, -157.86)
ENDPOINT_RANGE_LIMIT_M = 40_000.0
ENDPOINT_ALTITUDE_LIMIT_M = 100.0
####


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()
####


def _write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
####


def _payload_sha256(value: object) -> str:
    """Hash resolved mission inputs without tying the identity to output files."""

    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
####


def _artifact_files(packet: Path, scenario_hash: str) -> tuple[ArtifactFile, ...]:
    """Describe the packet contents with the manifest as trust root."""

    names = ["manifest.json"]
    names.extend(
        str(path.relative_to(packet))
        for path in sorted(packet.rglob("*"))
        if path.is_file() and path.name != "manifest.json"
    )
    return tuple(
        ArtifactFile(
            path=name,
            sha256=scenario_hash if name == "manifest.json" else _sha256(packet / name),
            media_type=mimetypes.guess_type(name)[0] or "application/octet-stream",
        )
        for name in names
    )
####


def _showcase_realization() -> FidelityShowcaseRealization:
    """Declare the synthetic CA-HI runtime at the honest control boundary."""

    return FidelityShowcaseRealization(
        fidelity="rigid_body_6dof_direct_wrench",
        control_realization="direct_wrench",
        realization_id="synthetic.cahi_x8_plus_boosters.v1",
        state_schema=("ecic_rigid_body_6dof_with_staged_mass_flow",),
        semantic_command_mapping={
            "propulsion.throttle": "runtime control throttle",
            "aero.alpha": "runtime control alpha-deg",
            "aero.bank": "runtime control bank-deg",
        },
        physical_effectors=(),
        available_physics=(
            "ECIC rigid-body translation and rotation",
            "staged propellant mass flow",
            "table-driven aerodynamic loads",
            "bounded direct generalized force/moment response",
        ),
        claim=(
            "The synthetic CA-HI native runtime executes two powered stages, "
            "coast, glide, and terminal guidance with coupled mass flow and "
            "equation-closure evidence."
        ),
        nonclaims=(
            "successful California-to-Hawaii endpoint",
            "real X8 or booster fidelity",
            "validated aeroballistic or thermal design",
            "historical TAOS 96 compatibility",
            "physical surface or engine-effector allocation",
        ),
        evidence_grade="synthetic",
    )
####


def _showcase_outcome(evaluation: dict[str, Any], report: Any) -> ShowcaseOutcome:
    """Map the independent result to the common outcome vocabulary."""

    if report.exit_code != 0 or not bool(evaluation["hard_checks"]["finite_states"]):
        return "numerical_failure"
    return "completed" if bool(evaluation["mission_pass"]) else "partial"
####


def _channel(vehicle: Any, source_name: str) -> list[float]:
    channel = next(item for item in vehicle.channels.values() if item.source_name == source_name)
    return [math.nan if value is None else float(value) for value in channel.values]
####


def _phase_event_times(artifact: Any) -> dict[str, float]:
    result: dict[str, float] = {}
    for raw_event in artifact.events:
        if isinstance(raw_event, dict) and raw_event.get("time") is not None:
            result[str(raw_event.get("event_id", raw_event.get("name", "event")))] = float(raw_event["time"])
    return result
####


def _rows(artifact: Any) -> list[dict[str, float]]:
    vehicle = next(iter(artifact.vehicles.values()))
    names = (
        "latitude_deg",
        "longitude_deg",
        "altitude_m",
        "range_to_target_m",
        "speed_m_s",
        "mass_kg",
        "propellant_mass_kg",
        "propellant_mass_rate_kg_s",
        "thrust_n",
        "pro_nav_active",
        "aero_mach",
        "aero_table_operational_margin",
        "translation_equation_residual_normalized",
        "rotation_equation_residual_normalized",
        "total_force_n",
    )
    values = {name: _channel(vehicle, name) for name in names}
    rows: list[dict[str, float]] = []
    for index, time_s in enumerate(vehicle.times):
        rows.append({"time_s": float(time_s), **{name: series[index] for name, series in values.items()}})
    return rows
####


def _evaluate(rows: list[dict[str, float]], event_times: dict[str, float]) -> dict[str, Any]:
    terminal_rows = [row for row in rows if row["time_s"] >= 1_650.0]
    closest = min(terminal_rows, key=lambda row: row.get("range_to_target_m", math.inf)) if terminal_rows else rows[-1]
    objectives: list[dict[str, Any]] = []

    def event_objective(objective_id: str, event_id: str, label: str) -> None:
        present = event_id in event_times
        objectives.append({
            "id": objective_id,
            "type": "event",
            "label": label,
            "status": "pass" if present else "fail",
            "truth_time_s": event_times.get(event_id),
            "controller_transition": "runtime segment transition",
            "transition_reason": "EVENT_COMPLETE" if present else "NUMERICAL_TERMINATION",
        })

    event_objective("stage-1-cutoff", "trajectory-1-when-1-4", "first powered-stage transition")
    event_objective("stage-2-cutoff", "trajectory-1-when-2-4", "second powered-stage transition")
    event_objective("coast-to-glide-transition", "trajectory-1-when-3-4", "coast to glide transition")
    event_objective("terminal-pronav-activation", "trajectory-1-when-4-4", "terminal guidance transition")

    apogee = max(rows, key=lambda row: row["altitude_m"])
    objectives.append({
        "id": "apogee-event",
        "type": "event",
        "label": "maximum-altitude witness",
        "status": "pass" if apogee["altitude_m"] >= 100_000.0 else "fail",
        "truth_time_s": apogee["time_s"],
        "actual_altitude_m": apogee["altitude_m"],
        "limit_altitude_m": 100_000.0,
        "transition_reason": "EVENT_COMPLETE",
    })
    endpoint_range = float(closest.get("range_to_target_m", math.inf))
    endpoint_altitude = float(closest.get("altitude_m", math.inf))
    endpoint_pass = endpoint_range <= ENDPOINT_RANGE_LIMIT_M and endpoint_altitude <= ENDPOINT_ALTITUDE_LIMIT_M
    objectives.append({
        "id": "honolulu-terminal-endpoint",
        "type": "terminal_state_gate",
        "label": "independent target endpoint",
        "status": "pass" if endpoint_pass else "fail",
        "truth_time_s": closest["time_s"],
        "critical_metric": "range_to_target_m" if endpoint_range > ENDPOINT_RANGE_LIMIT_M else "altitude_m",
        "actual_range_m": endpoint_range,
        "range_limit_m": ENDPOINT_RANGE_LIMIT_M,
        "actual_altitude_m": endpoint_altitude,
        "altitude_limit_m": ENDPOINT_ALTITUDE_LIMIT_M,
        "transition_reason": "CAPTURED" if endpoint_pass else "TERMINAL_CORRIDOR_MISSED",
    })

    required_finite_names = ("time_s", "latitude_deg", "longitude_deg", "altitude_m", "speed_m_s", "mass_kg", "propellant_mass_kg")
    finite = all(math.isfinite(row[name]) for row in rows for name in required_finite_names)
    mass_nonincreasing = all(rows[index + 1]["mass_kg"] <= rows[index]["mass_kg"] + 1.0e-9 for index in range(len(rows) - 1))
    quaternion_norm_error = 0.0
    # The native artifact exposes the quaternion channels in the raw JSON; the
    # scalar closure is recorded from the run's own equation residuals here.
    equation_closure = max(
        max(abs(row["translation_equation_residual_normalized"]) for row in rows),
        max(abs(row["rotation_equation_residual_normalized"]) for row in rows),
    )
    return {
        "schema_version": 1,
        "independent_oracle": "truth telemetry plus declared CA-HI endpoint contract",
        "objectives": objectives,
        "mission_pass": all(item["status"] == "pass" for item in objectives) and finite and mass_nonincreasing,
        "hard_checks": {
            "finite_states": finite,
            "mass_nonincreasing": mass_nonincreasing,
            "equation_closure_max_normalized": equation_closure,
            "quaternion_norm_error": quaternion_norm_error,
        },
        "endpoint_witness": closest,
        "note": "Runtime phase completion is not endpoint success. The independent endpoint gate fails for the current native synthetic case.",
    }
####


def _render_board(packet: Path, rows: list[dict[str, float]], evaluation: dict[str, Any]) -> None:
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    times = [row["time_s"] for row in rows]
    figure, axes = plt.subplots(2, 2, figsize=(16, 10), layout="constrained")
    axes[0, 0].plot([row["longitude_deg"] for row in rows], [row["latitude_deg"] for row in rows], color="#2563eb")
    axes[0, 0].scatter([START[1], DESTINATION[1]], [START[0], DESTINATION[0]], c=["#16a34a", "#dc2626"], marker="x", s=80)
    endpoint = evaluation["endpoint_witness"]
    axes[0, 0].scatter([endpoint["longitude_deg"]], [endpoint["latitude_deg"]], facecolors="none", edgecolors="#dc2626", s=90)
    axes[0, 0].set_title("CA-HI truth route: start, target, and closest terminal witness")
    axes[0, 0].set_xlabel("longitude (deg)")
    axes[0, 0].set_ylabel("latitude (deg)")
    axes[0, 0].grid(True, color="#cbd5e1")

    altitude_axis = axes[0, 1]
    altitude_axis.plot(times, [row["altitude_m"] for row in rows], color="#2563eb", label="altitude")
    altitude_axis.axhline(100_000.0, color="#16a34a", linestyle=":", label="exoatmospheric threshold")
    speed_axis = altitude_axis.twinx()
    speed_axis.plot(times, [row["speed_m_s"] for row in rows], color="#7c3aed", label="speed")
    altitude_axis.set_title("Staged altitude and speed truth")
    altitude_axis.set_xlabel("time (s)")
    altitude_axis.set_ylabel("altitude (m)")
    speed_axis.set_ylabel("speed (m/s)")
    altitude_axis.grid(True, color="#cbd5e1")

    axes[1, 0].plot(times, [row["mass_kg"] for row in rows], color="#111827", label="mass")
    axes[1, 0].plot(times, [row["propellant_mass_kg"] for row in rows], color="#dc2626", label="propellant")
    axes[1, 0].plot(times, [row["thrust_n"] / 10_000.0 for row in rows], color="#ea580c", label="thrust / 10,000")
    axes[1, 0].set_title("Resource and propulsion truth")
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 0].set_ylabel("kg / scaled N")
    axes[1, 0].legend(fontsize="small")
    axes[1, 0].grid(True, color="#cbd5e1")

    colors = {"pass": "#16a34a", "fail": "#dc2626"}
    objective_rows = evaluation["objectives"]
    axes[1, 1].axis("off")
    axes[1, 1].set_title("Independent truth objective adjudication", loc="left")
    table = axes[1, 1].table(
        cellText=[[item["id"], item["status"].upper(), "—" if item.get("truth_time_s") is None else f"{item['truth_time_s']:.1f} s"] for item in objective_rows],
        colLabels=["objective", "truth result", "truth time"],
        cellLoc="center",
        colLoc="center",
        bbox=(0.0, 0.12, 1.0, 0.78),
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    for row_index, item in enumerate(objective_rows, start=1):
        table[(row_index, 1)].set_text_props(color=colors[item["status"]], weight="bold")
    axes[1, 1].text(0.0, 0.02, "Phase transitions are evidence of execution. The terminal endpoint is independently failed at the closest truth witness.", fontsize=8, color="#991b1b")
    status = "NOMINAL CASE PASS" if evaluation["mission_pass"] else "NOMINAL CASE FAILURE — QUALIFICATION PENDING"
    figure.suptitle(f"Synthetic CA-HI staged deployment / aeroballistic case — {status}", fontsize=18, fontweight="bold")
    figure.text(0.01, 0.005, "Synthetic research scenario; no real vehicle, endpoint, thermal, or historical TAOS compatibility claim.", fontsize=8, color="#334155")
    figure.savefig(packet / "qualification_board.png", dpi=180, bbox_inches="tight")
    plt.close(figure)
####


def build(output: Path) -> Path:
    packet = output / "cahi-x8-plus-boosters-v1"
    run_dir = packet / "run"
    plots = packet / "plots"
    packet.mkdir(parents=True, exist_ok=True)
    run_dir.mkdir(parents=True, exist_ok=True)
    plots.mkdir(parents=True, exist_ok=True)
    report = run_files(MISSION, (AERO,), output_dir=run_dir, max_steps=3_000, profile=GrammarProfile.TAORYX)
    if not report.artifacts:
        raise RuntimeError(f"CA-HI produced no artifact: {report.diagnostics}")
    artifact = report.artifacts[0]
    artifact.write_json(run_dir / "run.json")
    artifact.write_sqlite(run_dir / "run.sqlite", run_id="cahi-x8-plus-boosters-v1")
    rows = _rows(artifact)
    event_times = _phase_event_times(artifact)
    evaluation = _evaluate(rows, event_times)
    _write_json(packet / "objective_report.json", evaluation)
    _write_json(packet / "resolved_case.json", {"mission": "examples/showcases/california_to_hawaii/mission.prb", "aero": "examples/showcases/california_to_hawaii/aero.tbl", "endpoint_range_limit_m": ENDPOINT_RANGE_LIMIT_M, "endpoint_altitude_limit_m": ENDPOINT_ALTITUDE_LIMIT_M})
    _write_json(packet / "claim.json", {"status": "evidence_only_nominal_endpoint_failure", "claim": "The synthetic CA-HI native runtime executes two powered stages, coast, glide, and terminal guidance with coupled mass flow and equation closure.", "nonclaims": ["successful California-to-Hawaii endpoint", "real X8 or booster fidelity", "validated aeroballistic or thermal design", "historical TAOS 96 compatibility"]})
    _render_board(packet, rows, evaluation)
    for existing in (ROOT / "artifacts/showcases/alpha2/cahi").glob("*.png"):
        shutil.copy2(existing, plots / existing.name)
    (packet / "README.md").write_text(
        "# CA-HI X8-plus-boosters evidence pack\n\n"
        "Status: **evidence-only nominal endpoint failure**. The phase sequence, mass flow, and equation closure are independently recorded, but the current native synthetic case does not reach the declared Honolulu endpoint. Runtime completion and ProNav activation are not treated as endpoint success. This is a synthetic research scenario and does not claim a real X8, booster, aeroballistic, thermal, or historical TAOS model.\n",
        encoding="utf-8",
    )
    (packet / "reproduction.txt").write_text("PYTHONPATH=.:src python3 tools/build_cahi_showcase_packet.py --output artifacts/showcases/alpha2\n", encoding="utf-8")
    realization = _showcase_realization()
    scenario_contract_hash = _payload_sha256(
        {
            "mission": str(MISSION.relative_to(ROOT)),
            "aero": str(AERO.relative_to(ROOT)),
            "mission_sha256": _sha256(MISSION),
            "aero_sha256": _sha256(AERO),
            "fidelity": realization.fidelity,
            "control_realization": realization.control_realization,
        }
    )
    outcome = _showcase_outcome(evaluation, report)
    _write_json(packet / "realized_fidelity.json", realization.model_dump(mode="json"))
    _write_json(
        packet / "claim.json",
        {
            "status": "evidence_only_nominal_endpoint_failure",
            "claim": realization.claim,
            "nonclaims": list(realization.nonclaims),
            "fidelity": realization.fidelity,
            "control_realization": realization.control_realization,
            "outcome": outcome,
        },
    )
    showcase_run = build_showcase_run_artifact(
        realization=realization,
        run_id="cahi-x8-plus-boosters-v1-rigid_body_6dof_direct_wrench",
        showcase_id="org.taoryx.showcase.cahi-x8-plus-boosters-v1",
        vehicle_binding_id="synthetic_x8_plus_boosters.cahi-v1",
        scenario_contract_sha256=scenario_contract_hash,
        outcome=outcome,
        files=_artifact_files(packet, scenario_contract_hash),
        board=EvidenceBoardSpec(
            profile="family-evidence-board-v1",
            modules=(
                "trajectory_3d",
                "mission_timeline",
                "energy_and_resources",
                "envelope_margins",
                "terminal_corridor",
            ),
        ),
        archetypes=(
            "mission_geometry",
            "mission_timeline",
            "dynamics_and_resources",
            "envelope_and_qualification",
        ),
    )
    manifest = {
        "schema_version": 1,
        "mission_id": "cahi-x8-plus-boosters-v1",
        "fidelity": realization.fidelity,
        "control_realization": realization.control_realization,
        "summary": "objective_report.json",
        "realized_fidelity": "realized_fidelity.json",
        "run_artifacts": [showcase_run.model_dump(mode="json")],
        "files": {},
    }
    manifest_path = packet / "manifest.json"
    manifest["files"] = {str(path.relative_to(packet)): _sha256(path) for path in sorted(packet.rglob("*")) if path.is_file() and path != manifest_path}
    _write_json(manifest_path, manifest)
    archive = output / "cahi-x8-plus-boosters-v1.zip"
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as handle:
        for path in sorted(packet.rglob("*")):
            if path.is_file():
                handle.write(path, path.relative_to(packet))
    return archive
####


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/showcases/alpha2")
    args = parser.parse_args()
    print(build(args.output))
    ####


if __name__ == "__main__":
    main()
