"""Build deterministic evidence-board packs for promoted DaveML families."""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import zipfile
from pathlib import Path
from typing import Any

import yaml

from taoryx.showcase import (
    ArtifactFile,
    EvidenceBoardSpec,
    ObjectLineage,
    ObjectLineageEvent,
    ObjectLineageNode,
    ShowcaseRunArtifact,
)

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CATALOG = ROOT / "verification/daveml_showcase_catalog.yaml"
DEFAULT_OUTPUT = ROOT / "artifacts/showcases/daveml-families"
NESC_EVIDENCE_ZIP = ROOT / "INBOX/taoryx-daveml-nesc-model-catalog-v1.0/qualified/nesc-two-stage-rocket/nesc-two-stage-rocket-v0.9-evidence.zip"
NESC_PREFIX = "taoryx-nesc-two-stage-rocket-v0.9/"


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256(path: Path) -> str:
    return _sha256_bytes(path.read_bytes())


def _json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _load_catalog(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or not isinstance(payload.get("boards"), list):
        raise ValueError(f"invalid DaveML showcase catalog: {path}")
    return payload


def _evidence(board: dict[str, Any]) -> list[dict[str, str]]:
    records: list[dict[str, str]] = []
    for label in board["evidence"]:
        path = ROOT / str(label)
        if not path.is_file():
            raise FileNotFoundError(f"missing showcase evidence: {label}")
        records.append({"path": str(label), "sha256": _sha256(path)})
    return records


def _events(board_id: str, source: dict[str, Any]) -> list[dict[str, Any]]:
    if not board_id.startswith("nesc-"):
        return [{"id": event, "time_s": float(index), "source": "qualified_evidence"} for index, event in enumerate(source["events"])]
    with zipfile.ZipFile(NESC_EVIDENCE_ZIP) as archive:
        rows = list(csv.DictReader(io.StringIO(archive.read(NESC_PREFIX + "tables/mission-events.csv").decode("utf-8"))))
    event_rows = [{"id": row["event"], "time_s": float(row["time_s"]), "phase_after_event": row["phase_after_event"], "source": "source_mission_events"} for row in rows]
    event_rows.append({"id": "orbit_coast", "time_s": 200.0, "phase_after_event": "orbit_coast", "source": "qualified_evidence"})
    if board_id == "nesc-synthetic-passive-child-deployment-v1":
        event_rows.insert(
            2,
            {
                "id": "synthetic_child_separation",
                "time_s": float(rows[1]["time_s"]),
                "phase_after_event": "synthetic_passive_child",
                "source": "taoryx_synthetic_deployment_contract",
            },
        )
        event_rows.append({"id": "terminal_checkpoint", "time_s": 200.0, "phase_after_event": "terminal", "source": "qualified_evidence"})
    return event_rows


def _nesc_rows() -> list[dict[str, Any]]:
    with zipfile.ZipFile(NESC_EVIDENCE_ZIP) as archive:
        raw = archive.read(NESC_PREFIX + "validation/scenario17-trajectory.csv").decode("utf-8")
    rows: list[dict[str, Any]] = []
    for row in csv.DictReader(io.StringIO(raw)):
        parsed: dict[str, Any] = {}
        for key, value in row.items():
            try:
                parsed[key] = float(value) if value else None
            except ValueError:
                parsed[key] = value
        rows.append(parsed)
    return rows


def _nesc_lineage(deployment: bool) -> ObjectLineage:
    nodes = [
        ObjectLineageNode(id="nesc-stack", role="two_stage_source_stack", active_from_s=0.0, active_to_s=200.0, terminal_disposition="terminal_checkpoint"),
        ObjectLineageNode(id="nesc-stage-1", role="source_stage_1", parent_id="nesc-stack", active_from_s=0.0, active_to_s=37.38045176470588, death_event_id="stage1_burnout", terminal_disposition="burnout_and_jettison"),
        ObjectLineageNode(id="nesc-stage-2", role="source_stage_2", parent_id="nesc-stack", active_from_s=134.1704517647059, active_to_s=195.3636357647059, death_event_id="stage2_burnout", terminal_disposition="burnout"),
    ]
    events = [
        ObjectLineageEvent(id="liftoff", event_type="release", object_id="nesc-stack", time_s=0.0, reason="source Scenario 17 ignition"),
        ObjectLineageEvent(id="stage1_burnout", event_type="terminal", object_id="nesc-stage-1", parent_object_id="nesc-stack", time_s=37.38045176470588, reason="source stage-1 burnout and jettison"),
        ObjectLineageEvent(id="stage2_ignition", event_type="release", object_id="nesc-stage-2", parent_object_id="nesc-stack", time_s=134.1704517647059, reason="source stage-2 ignition"),
        ObjectLineageEvent(id="stage2_burnout", event_type="terminal", object_id="nesc-stage-2", parent_object_id="nesc-stack", time_s=195.3636357647059, reason="source stage-2 burnout"),
        ObjectLineageEvent(id="terminal", event_type="terminal", object_id="nesc-stack", time_s=200.0, reason="source retained checkpoint"),
    ]
    if deployment:
        nodes.append(ObjectLineageNode(id="nesc-synthetic-cylinder", role="synthetic_passive_cylinder", parent_id="nesc-stack", spawn_event_id="synthetic_child_separation", active_from_s=37.38045176470588, active_to_s=200.0, death_event_id="synthetic_child_terminal", terminal_disposition="synthetic_ballistic_witness"))
        events.extend(
            [
                ObjectLineageEvent(id="synthetic_child_separation", event_type="separation", object_id="nesc-synthetic-cylinder", parent_object_id="nesc-stack", time_s=37.38045176470588, reason="synthetic passive cylinder deployment witness"),
                ObjectLineageEvent(id="synthetic_child_terminal", event_type="terminal", object_id="nesc-synthetic-cylinder", parent_object_id="nesc-stack", time_s=200.0, reason="synthetic child witness horizon"),
            ]
        )
    return ObjectLineage(nodes=tuple(nodes), events=tuple(events))


def _generic_metrics(board_id: str) -> tuple[dict[str, float], list[dict[str, Any]]]:
    if board_id == "f16-maneuver-trim-evidence-v1":
        trim = _json(ROOT / "verification/daveml_f16_equilibrium_trim_evidence.json")
        scenario = _json(ROOT / "verification/daveml_f16_scenario_evidence.json")
        tuning = _json(ROOT / "verification/daveml_f16_tuning_evidence.json")
        return (
            {
                "trim_alpha_deg": float(trim["state"]["alpha_deg"]),
                "trim_max_residual": float(trim["max_residual"]),
                "maneuver_score": float(scenario["objective_report"]["score"]),
                "maneuver_duration_s": float(scenario["scenario_contract"]["duration_s"]),
                "lqr_max_real_pole": float(tuning["maximum_real_pole"]),
                "lqr_condition_number": float(tuning["condition_number"]),
                "lqr_controllable": 1.0 if tuning["controllable"] else 0.0,
            },
            [{"time_s": 0.0, "phase": "equilibrium_trim", "alpha_deg": float(trim["state"]["alpha_deg"]), "residual": float(trim["max_residual"])}, {"time_s": float(scenario["scenario_contract"]["duration_s"]), "phase": "maneuver_probe", "objective_score": float(scenario["objective_report"]["score"])}],
        )
    if board_id == "hl20-glide-entry-evidence-v1":
        trim = _json(ROOT / "verification/daveml_hl20_trim_evidence.json")
        scenario = _json(ROOT / "verification/daveml_hl20_scenario_evidence.json")
        return (
            {
                "trim_alpha_deg": float(trim["trim"]["state"]["alpha_deg"]),
                "trim_max_residual": float(trim["trim"]["max_residual"]),
                "glide_score": float(scenario["objective_report"]["score"]),
                "glide_duration_s": float(scenario["scenario_contract"]["duration_s"]),
                "controller_claim": 0.0,
            },
            [{"time_s": 0.0, "phase": "glide_trim", "alpha_deg": float(trim["trim"]["state"]["alpha_deg"]), "residual": float(trim["trim"]["max_residual"])}, {"time_s": float(scenario["scenario_contract"]["duration_s"]), "phase": "glide_objective_gate", "objective_score": float(scenario["objective_report"]["score"])}],
        )
    if board_id == "a320-derived-versus-surrogate-v1":
        derived = _json(ROOT / "verification/daveml_a320_openap_integration.json")
        surrogate = _json(ROOT / "verification/daveml_a320_pseudo6dof_integration.json")
        runtime = surrogate["runtime_qualification"]
        return (
            {
                "derived_exact_trim_residual": float(max(abs(float(value)) for value in derived["trim"]["residuals"].values())),
                "derived_exact_objective_score": float(derived["objectives"]["score"]),
                "surrogate_cruise_trim_residual": float(runtime["trim"]["cruise"]["max_residual"]),
                "surrogate_turn_beta_rad": float(runtime["coordinated_turn"]["max_abs_beta_rad"]),
                "surrogate_control_pulse_count": float(len(runtime["control_pulses"])),
            },
            [{"time_s": 0.0, "product": "derived_exact", "trim_residual": float(max(abs(float(value)) for value in derived["trim"]["residuals"].values()))}, {"time_s": 1.0, "product": "surrogate_composite", "trim_residual": float(runtime["trim"]["cruise"]["max_residual"])}, {"time_s": 20.0, "product": "surrogate_composite", "event": "coordinated_turn"}],
        )
    raise ValueError(f"no generic metrics for {board_id}")


def _pack_data(board: dict[str, Any]) -> tuple[dict[str, float], list[dict[str, Any]], list[dict[str, Any]], ObjectLineage | None, dict[str, Any]]:
    board_id = str(board["id"])
    if board_id.startswith("nesc-"):
        rows = _nesc_rows()
        metrics = {
            "duration_s": float(rows[-1]["time_s"]),
            "maximum_altitude_m": max(float(row["altitude_m"]) for row in rows),
            "maximum_mach": max(float(row["mach"]) for row in rows),
            "stage1_burnout_s": 37.38045176470588,
            "stage2_ignition_s": 134.1704517647059,
            "stage2_burnout_s": 195.3636357647059,
        }
        deployment = board_id == "nesc-synthetic-passive-child-deployment-v1"
        if deployment:
            separation = metrics["stage1_burnout_s"]
            for row in rows:
                if float(row["time_s"]) < separation:
                    continue
                dt = float(row["time_s"]) - separation
                position = [float(row[f"position_eci_{axis}_m"]) for axis in ("x", "y", "z")]
                velocity = [float(row[f"velocity_eci_{axis}_mps"]) for axis in ("x", "y", "z")]
                radius = math.sqrt(sum(value * value for value in position))
                radial_velocity = sum(pos * vel for pos, vel in zip(position, velocity)) / radius
                row["synthetic_child_altitude_m"] = max(0.0, float(row["altitude_m"]) + radial_velocity * dt - 0.5 * 9.80665 * dt * dt)
                row["synthetic_child_mass_kg"] = 19000.0
                row["synthetic_child_projected_area_m2"] = 7.0
        return metrics, rows, _events(board_id, board), _nesc_lineage(deployment), {"telemetry_kind": "source_trajectory_with_optional_synthetic_child", "synthetic_child": board.get("synthetic_child") if deployment else None}
    metrics, samples = _generic_metrics(board_id)
    lineage = None
    return metrics, samples, _events(board_id, board), lineage, {"telemetry_kind": "qualified_evidence_summary", "synthetic_child": None}


def _render_board(board: dict[str, Any], metrics: dict[str, float], samples: list[dict[str, Any]], events: list[dict[str, Any]], destination: Path) -> None:
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 2, figsize=(14, 8), layout="constrained")
    figure.suptitle(str(board["claim"]), fontsize=14, fontweight="bold", x=0.02, ha="left")
    axis = axes[0][0]
    axis.axis("off")
    axis.set_title("Qualification metrics", loc="left", fontweight="bold")
    rows = [[key.replace("_", " "), f"{value:.6g}"] for key, value in list(metrics.items())[:8]]
    table = axis.table(cellText=rows, colLabels=["Metric", "Value"], loc="center", cellLoc="left")
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 1.35)
    axis = axes[0][1]
    axis.set_title("Family-local event timeline", loc="left", fontweight="bold")
    times = [float(event["time_s"]) for event in events]
    for index, event in enumerate(events):
        axis.scatter([times[index]], [0], color="#2563eb", zorder=3)
        axis.text(times[index], 0.05 + 0.08 * (index % 2), str(event["id"]), rotation=45, ha="left", va="bottom", fontsize=8)
    axis.set_yticks([])
    axis.set_xlabel("time (s)")
    axis.grid(True, axis="x", color="#cbd5e1", linewidth=0.7)
    axis = axes[1][0]
    axis.set_title("Evidence trace", loc="left", fontweight="bold")
    if samples and "altitude_m" in samples[0]:
        axis.plot([float(row["time_s"]) for row in samples], [float(row["altitude_m"]) for row in samples], label="parent altitude (m)", color="#2563eb")
        if "synthetic_child_altitude_m" in samples[0]:
            child = [row for row in samples if "synthetic_child_altitude_m" in row]
            axis.plot([float(row["time_s"]) for row in child], [float(row["synthetic_child_altitude_m"]) for row in child], label="synthetic child altitude (m)", color="#d97706", linestyle="--")
        axis.set_xlabel("time (s)")
        axis.legend(fontsize="small")
    else:
        labels = list(metrics.keys())
        values = list(metrics.values())
        axis.bar(range(len(labels)), values, color="#2563eb")
        axis.set_xticks(range(len(labels)), [label.replace("_", " ") for label in labels], rotation=55, ha="right", fontsize=8)
        axis.set_ylabel("value")
    axis.grid(True, axis="y", color="#cbd5e1", linewidth=0.7)
    axis = axes[1][1]
    axis.axis("off")
    axis.set_title("Claim boundary", loc="left", fontweight="bold")
    text = "\n".join(["Qualification class: " + str(board["qualification_class"]), "Evidence grade: " + str(board["evidence_grade"]), "", "Nonclaims:", *["- " + str(item) for item in board["nonclaims"]]])
    axis.text(0.02, 0.95, text, va="top", fontsize=10, wrap=True)
    figure.savefig(destination, dpi=160, bbox_inches="tight")
    plt.close(figure)


def build_board(board: dict[str, Any], output_root: Path) -> dict[str, Any]:
    destination = output_root / str(board["id"])
    destination.mkdir(parents=True, exist_ok=True)
    evidence = _evidence(board)
    metrics, samples, events, lineage, telemetry_meta = _pack_data(board)
    actual_event_ids = {str(event["id"]) for event in events}
    missing_events = sorted(set(str(event) for event in board["events"]).difference(actual_event_ids))
    if missing_events:
        raise ValueError(f"{board['id']} is missing required evidence events: {missing_events}")
    (destination / "evidence-summary.json").write_text(json.dumps({"board": board, "metrics": metrics, "events": events, "evidence": evidence}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (destination / "telemetry.json").write_text(json.dumps({"metadata": telemetry_meta, "samples": samples}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if lineage is not None:
        (destination / "object-lineage.json").write_text(json.dumps(lineage.model_dump(mode="json"), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _render_board(board, metrics, samples, events, destination / "evidence-board.png")
    files = ["manifest.json", "evidence-summary.json", "telemetry.json", "evidence-board.png"]
    if lineage is not None:
        files.append("object-lineage.json")
    core = {
        "schema_version": "taoryx.daveml-family-showcase/v1",
        "board_id": board["id"],
        "family_id": board["family_id"],
        "qualification_class": board["qualification_class"],
        "fidelity_profiles": board["fidelity_profiles"],
        "claim": board["claim"],
        "nonclaims": board["nonclaims"],
        "archetypes": board["archetypes"],
        "evidence": evidence,
        "metrics": metrics,
        "events": events,
        "artifact_files": files,
    }
    contract_hash = _sha256_bytes(json.dumps(core, sort_keys=True, separators=(",", ":")).encode("utf-8"))
    run_artifacts: list[dict[str, Any]] = []
    for fidelity in board["fidelity_profiles"]:
        run = ShowcaseRunArtifact(
            run_id=f"{board['id']}-{fidelity}",
            showcase_id=str(board["id"]),
            vehicle_binding_id=f"{board['family_id']}-showcase-v1",
            fidelity=fidelity,
            scenario_contract_sha256=contract_hash,
            outcome="completed",
            claim=str(board["claim"]),
            nonclaims=tuple(str(item) for item in board["nonclaims"]),
            files=tuple(ArtifactFile(path=name, sha256=contract_hash if name == "manifest.json" else _sha256(destination / name), media_type="image/png" if name.endswith(".png") else "application/json") for name in files),
            board=EvidenceBoardSpec(profile="daveml-family-evidence-board-v1", modules=tuple(str(item) for item in board["archetypes"])),
            archetypes=tuple(str(item) for item in board["archetypes"]),
            object_lineage=lineage,
        )
        run_artifacts.append(run.model_dump(mode="json"))
    manifest = {**core, "artifact_contract_hash": contract_hash, "manifest_hash_basis": "canonical core without generated hashes", "run_artifacts": run_artifacts, "object_lineage": lineage.model_dump(mode="json") if lineage is not None else None}
    (destination / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return {"board_id": board["id"], "family_id": board["family_id"], "status": "verified", "artifact_contract_hash": contract_hash, "path": str(destination.resolve().relative_to(ROOT))}


def build(catalog_path: str | Path = DEFAULT_CATALOG, output_root: str | Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    catalog = _load_catalog(Path(catalog_path))
    destination = Path(output_root)
    destination.mkdir(parents=True, exist_ok=True)
    reports = [build_board(board, destination) for board in catalog["boards"]]
    report = {"schema_version": "taoryx.daveml-family-showcase-report/v1", "status": "verified", "catalog_id": catalog["catalog_id"], "boards": reports, "claim_boundary": catalog["claim_boundary"]}
    (destination / "catalog-report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    print(json.dumps(build(arguments.catalog, arguments.output_dir), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
