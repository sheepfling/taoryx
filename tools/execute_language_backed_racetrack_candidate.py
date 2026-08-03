"""Execute a capability-scaled racetrack through language-backed packets.

X8 and B747 qualification uses a `.prb` scenario plus a racetrack catalog,
rather than a Python runner accepting a route object directly.  This tool
materializes disposable copies of those inputs for one compiled candidate,
then calls the normal family-packet builder and emits evidence that is bound
to the candidate fingerprint.  It never edits a checked-in baseline problem,
catalog, or acceptance configuration.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import mimetypes
import re
import tempfile
import zipfile
from pathlib import Path
from typing import Any, cast

import yaml

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.ingest import FileKind, ingest_file
from taoryx.language.models import ProblemDocument, TableDocument
from taoryx.language_backed_racetrack import materialize_language_backed_racetrack, materialize_powered_fixed_wing_composition
from taoryx.mission_objectives import TruthObjectiveSpec, evaluate_truth_objectives
from taoryx.mission_promotion import assess_mission_promotion, mission_proposal_fingerprint
from taoryx.powered_fixed_wing_mission_compiler import (
    CapabilityScaledRacetrack,
    compile_powered_fixed_wing_racetrack,
    resolve_powered_fixed_wing_mission_profile,
)
from taoryx.racetrack_template import RACETRACK_FIDELITIES, RacetrackFidelity, load_racetrack_template_catalog
from taoryx.runtime.interactive import InteractiveSession
from taoryx.runtime.lowering import lower_problem_document, problem_unit_settings
from taoryx.runtime.runner import run_files
from taoryx.runtime.table_binding import bind_runtime_tables
from taoryx.vehicle_composition import CompiledVehicleComposition, load_compiled_vehicle_composition
from taoryx.vehicle_execution_preflight import preflight_vehicle_composition

try:
    from build_family_qualification_packet import _local_rows, build
except ModuleNotFoundError:  # pragma: no cover - direct script invocation
    from tools.build_family_qualification_packet import _local_rows, build

ROOT = Path(__file__).resolve().parents[1]
PROFILES = ROOT / "verification/powered_fixed_wing_mission_profiles.yaml"

LANGUAGE_BACKED_MISSIONS: dict[str, dict[RacetrackFidelity, str]] = {
    "x8-cruise": {
        "point_mass_3dof": "x8-racetrack-altitude-turns-3dof-v1",
        "pseudo_6dof_kinematic_bridge": "x8-racetrack-altitude-turns-pseudo-6dof-v1",
        "rigid_body_6dof_direct_wrench": "x8-racetrack-altitude-turns-direct-wrench-v1",
        "rigid_body_6dof_surface_allocated": "x8-racetrack-altitude-turns-v1",
    },
    "b747-cruise": {
        "point_mass_3dof": "b747-racetrack-altitude-turns-3dof-v1",
        "pseudo_6dof_kinematic_bridge": "b747-racetrack-altitude-turns-pseudo-6dof-v1",
        "rigid_body_6dof_direct_wrench": "b747-racetrack-altitude-turns-6dof-v1",
    },
}


def materialize_candidate_inputs(
    proposal: CapabilityScaledRacetrack,
    mission_id: str,
    directory: Path,
) -> tuple[Path, Path, Path]:
    """Compatibility wrapper around the source-owned materialization seam."""

    materialized = materialize_language_backed_racetrack(proposal, mission_id, directory)
    return materialized.problem, materialized.mission_config, materialized.racetrack_config
    ####


def execute_candidate(profile_id: str, fidelity: RacetrackFidelity, output: Path) -> dict[str, Any]:
    """Compile, materialize, execute, and normalize one candidate packet."""

    if profile_id not in LANGUAGE_BACKED_MISSIONS or fidelity not in LANGUAGE_BACKED_MISSIONS[profile_id]:
        raise ValueError(f"{profile_id!r} has no language-backed mission at {fidelity!r}")
    proposal = compile_powered_fixed_wing_racetrack(
        *resolve_powered_fixed_wing_mission_profile(PROFILES, profile_id, fidelity),
        binding_id=f"{profile_id}-{fidelity}-candidate",
        fidelity=fidelity,
    )
    source_mission_id = LANGUAGE_BACKED_MISSIONS[profile_id][fidelity]
    with tempfile.TemporaryDirectory(prefix="taoryx-racetrack-candidate-") as temporary:
        temporary_root = Path(temporary)
        problem, mission_config, racetrack_config = materialize_candidate_inputs(proposal, source_mission_id, temporary_root)
        candidate_mission_id = f"{source_mission_id}-candidate"
        return _execute_materialized_candidate(
            output=output,
            proposal=proposal,
            candidate_mission_id=candidate_mission_id,
            problem=problem,
            mission_config=mission_config,
            racetrack_config=racetrack_config,
            reproduction_command=(
                "PYTHONPATH=src python3 tools/execute_language_backed_racetrack_candidate.py "
                f"--profile {profile_id} --fidelity {fidelity} --output {output}"
            ),
        )
    ####


def execute_composition(composition: CompiledVehicleComposition, output: Path) -> dict[str, Any]:
    """Run the exact source inputs materialized from one compiled composition.

    This is the P1 bridge from the public composition contract to the existing
    language-backed runtime and independent evaluator.  It fails before any
    integration if semantic preflight cannot prove the native route geometry.
    The qualification packet remains tool-owned for now; moving its renderer
    does not change the runtime/evaluator evidence path exercised here.
    """

    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        diagnostics = "; ".join(preflight.diagnostics) or "no translation-ready geometry"
        raise ValueError(f"cannot execute composition {composition.id!r}: {preflight.status}: {diagnostics}")
    with tempfile.TemporaryDirectory(prefix="taoryx-racetrack-composition-") as temporary:
        materialized = materialize_powered_fixed_wing_composition(composition, Path(temporary))
        execution = _execute_materialized_candidate(
            output=output,
            proposal=materialized.proposal,
            candidate_mission_id=materialized.materialized_mission_id,
            problem=materialized.problem,
            mission_config=materialized.mission_config,
            racetrack_config=materialized.racetrack_config,
            reproduction_command=(
                "PYTHONPATH=src python3 tools/execute_language_backed_racetrack_candidate.py "
                f"--composition <compiled-composition.json> --output {output}"
            ),
        evidence_metadata={
                "composition_id": composition.id,
                "composition_identity_sha256": composition.identity_sha256,
                "translation_preflight_status": preflight.status,
        },
    )
        packet = Path(str(execution["packet"]))
        inputs = packet / "inputs"
        (inputs / "compiled_vehicle_composition.json").write_text(
            json.dumps(composition.model_dump(mode="json", by_alias=True), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        (inputs / "vehicle_execution_preflight.json").write_text(
            json.dumps(preflight.as_dict(), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        _refresh_packet_contract(packet)
    return execution
    ####


def _execute_materialized_candidate(
    *,
    output: Path,
    proposal: CapabilityScaledRacetrack,
    candidate_mission_id: str,
    problem: Path,
    mission_config: Path,
    racetrack_config: Path,
    reproduction_command: str,
    evidence_metadata: dict[str, object] | None = None,
) -> dict[str, Any]:
    """Run and assess one exact, already-materialized candidate input set."""

    fingerprint = mission_proposal_fingerprint(proposal)
    metadata = {
        "mission_proposal_fingerprint": fingerprint,
        "candidate_binding_id": proposal.route.binding_id,
        "candidate_fidelity": proposal.route.fidelity,
        **({} if evidence_metadata is None else evidence_metadata),
    }
    _ = build(
        output,
        candidate_mission_id,
        problem_override=problem,
        mission_config=mission_config,
        racetrack_config=racetrack_config,
        evidence_metadata=metadata,
        reproduction_command=reproduction_command,
    )
    packet = output / candidate_mission_id
    inputs = packet / "inputs"
    for path in (mission_config, racetrack_config):
        (inputs / path.name).write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    summary = json.loads((packet / "summary.json").read_text(encoding="utf-8"))
    convergence = _semantic_gate_convergence(
        packet=packet,
        problem=problem,
        tables=tuple(Path(item) for item in _mission_tables(mission_config, candidate_mission_id)),
        mission_config=mission_config,
        racetrack_config=racetrack_config,
        candidate_mission_id=candidate_mission_id,
        base_summary=summary,
    )
    parity = _batch_step_parity(
        packet=packet,
        problem=problem,
        tables=tuple(Path(item) for item in _mission_tables(mission_config, candidate_mission_id)),
        max_steps=int(summary["run"]["sample_count"]) + 4,
    )
    (packet / "convergence_report.json").write_text(json.dumps(convergence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (packet / "batch_step_parity.json").write_text(json.dumps(parity, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    execution = {
        "schema_version": "taoryx.language-backed-candidate-execution/v1",
        "mission_proposal_fingerprint": fingerprint,
        "binding_id": proposal.route.binding_id,
        "fidelity": proposal.route.fidelity,
        "packet": str(packet),
        "evaluation": {
            "mission_pass": bool(summary["mission_pass"]),
            "required_objectives": summary["truth_evaluation"]["required_objectives"],
            "required_passed": summary["truth_evaluation"]["required_passed"],
            "hard_gates_passed": summary["truth_evaluation"]["hard_gates_passed"],
        },
        "runtime": {
            "numerical_valid": bool(summary["numerical_valid"]),
            "semantic_gate_convergence_pass": convergence["status"] == "pass",
            "batch_step_parity_pass": parity["status"] == "pass",
        },
    }
    (packet / "candidate_execution.json").write_text(json.dumps(execution, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (packet / "candidate_proposal.json").write_text(json.dumps(proposal.manifest(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    assessment = assess_mission_promotion(proposal, execution).as_dict()
    (packet / "promotion_assessment.json").write_text(json.dumps(assessment, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _refresh_packet_contract(packet)
    return {**execution, "promotion": assessment}
    ####


def _mission_tables(mission_config: Path, mission_id: str) -> tuple[str, ...]:
    """Return the immutable table inputs declared for one materialized mission."""

    payload = yaml.safe_load(mission_config.read_text(encoding="utf-8"))
    missions = payload.get("missions", ()) if isinstance(payload, dict) else ()
    mission = next((item for item in missions if isinstance(item, dict) and item.get("id") == mission_id), None)
    if mission is None:
        raise KeyError(f"materialized mission {mission_id!r} is absent from {mission_config}")
    return tuple(str(ROOT / item) for item in mission.get("tables", ()))
    ####


def _semantic_gate_convergence(
    *,
    packet: Path,
    problem: Path,
    tables: tuple[Path, ...],
    mission_config: Path,
    racetrack_config: Path,
    candidate_mission_id: str,
    base_summary: dict[str, Any],
) -> dict[str, object]:
    """Compare exact candidate truth objectives after one fixed-step refinement."""

    base_step = _problem_step(problem)
    with tempfile.TemporaryDirectory(prefix="taoryx-racetrack-convergence-") as temporary:
        root = Path(temporary)
        fine_problem = _scaled_problem(problem, root / "fine.prb", 0.5)
        fine_summary = _run_materialized_summary(
            problem=fine_problem,
            tables=tables,
            mission_config=mission_config,
            racetrack_config=racetrack_config,
            candidate_mission_id=candidate_mission_id,
            max_steps=_materialized_max_steps(mission_config, candidate_mission_id) * 2 + 4,
        )
    base_results = {str(item["id"]): item for item in base_summary["truth_evaluation"]["results"]}
    fine_results = {str(item["id"]): item for item in fine_summary["truth_evaluation"]["results"]}
    objective_ids = tuple(base_results)
    comparisons: list[dict[str, object]] = []
    maximum_time_delta_s = 0.0
    maximum_margin_delta = 0.0
    status_parity = set(base_results) == set(fine_results)
    for objective_id in objective_ids:
        base = base_results[objective_id]
        fine = fine_results.get(objective_id, {})
        base_time, fine_time = base.get("truth_time_s"), fine.get("truth_time_s")
        time_delta = None if base_time is None or fine_time is None else float(fine_time) - float(base_time)
        base_margin, fine_margin = base.get("margin"), fine.get("margin")
        margin_delta = None if base_margin is None or fine_margin is None else float(fine_margin) - float(base_margin)
        if time_delta is not None:
            maximum_time_delta_s = max(maximum_time_delta_s, abs(time_delta))
        if margin_delta is not None:
            maximum_margin_delta = max(maximum_margin_delta, abs(margin_delta))
        status_parity = status_parity and base.get("status") == fine.get("status")
        comparisons.append(
            {
                "id": objective_id,
                "base_status": base.get("status"),
                "fine_status": fine.get("status"),
                "base_truth_time_s": base_time,
                "fine_truth_time_s": fine_time,
                "truth_time_delta_s": time_delta,
                "base_margin": base_margin,
                "fine_margin": fine_margin,
                "margin_delta": margin_delta,
            }
        )
    mission_status_parity = bool(base_summary["mission_pass"]) == bool(fine_summary["mission_pass"])
    time_limit_s = max(0.5, 4.0 * base_step)
    margin_limit = 0.5
    passed = (
        bool(base_summary["numerical_valid"])
        and bool(fine_summary["numerical_valid"])
        and mission_status_parity
        and status_parity
        and maximum_time_delta_s <= time_limit_s
        and maximum_margin_delta <= margin_limit
    )
    return {
        "schema_version": "taoryx.semantic-gate-convergence/v1",
        "status": "pass" if passed else "fail",
        "base_step_s": base_step,
        "comparison_step_s": base_step * 0.5,
        "mission_status_parity": mission_status_parity,
        "objective_status_parity": status_parity,
        "maximum_truth_time_delta_s": maximum_time_delta_s,
        "maximum_margin_delta": maximum_margin_delta,
        "acceptance": {"maximum_truth_time_delta_s": time_limit_s, "maximum_margin_delta": margin_limit},
        "objectives": comparisons,
        "claim_boundary": "Two fixed-step truth-objective runs; not global numerical convergence or an integrator comparison.",
    }
    ####


def _batch_step_parity(*, packet: Path, problem: Path, tables: tuple[Path, ...], max_steps: int) -> dict[str, object]:
    """Replay the exact static scenario through the external accepted-step seam."""

    table_documents: list[TableDocument] = []
    for table in tables:
        ingested = ingest_file(table)
        if ingested.kind is not FileKind.TABLE:
            raise ValueError(f"expected table input, received {table}")
        table_documents.append(cast(TableDocument, ingested.document))
    available_tables, runtime_tables = bind_runtime_tables(table_documents, {})
    ingested_problem = ingest_file(problem, available_tables=available_tables, profile=GrammarProfile.TAORYX)
    if ingested_problem.kind is not FileKind.PROBLEM:
        raise ValueError(f"expected problem input, received {problem}")
    document = cast(ProblemDocument, ingested_problem.document)
    unit_settings, _ = problem_unit_settings(document)
    available_tables, runtime_tables = bind_runtime_tables(table_documents, unit_settings)
    lowered = lower_problem_document(document, runtime_tables)
    if lowered.unsupported_features or len(lowered.cases) != 1:
        raise ValueError("batch/step parity currently requires one fully lowerable runtime case")
    for vehicle in lowered.cases[0].problem.vehicles.values():
        vehicle.integrator = "rk4"
    session = InteractiveSession(lowered.cases[0].problem)
    for _ in range(max_steps):
        active = session.problem.active_vehicles()
        if not active:
            break
        session.step(min(vehicle.step_size for vehicle in active))
    else:
        raise RuntimeError(f"interactive replay exceeded {max_steps} accepted steps")
    vehicle = next(iter(session.problem.vehicles.values()))
    step_rows = _local_rows(tuple(vehicle.history))
    with (packet / "truth_telemetry.csv").open(encoding="utf-8", newline="") as stream:
        batch_rows = tuple(dict(row) for row in csv.DictReader(stream))
    channels = (
        "north_m",
        "east_m",
        "altitude_m",
        "speed_m_s",
        "kinematic_roll_deg",
        "kinematic_pitch_deg",
        "kinematic_yaw_deg",
    )
    batch_by_time = {round(float(row["time_s"]), 9): row for row in batch_rows}
    step_by_time = {round(float(row["time_s"]), 9): row for row in step_rows}
    shared_times = tuple(sorted(set(batch_by_time).intersection(step_by_time)))
    differences: dict[str, float] = {}
    for channel in channels:
        channel_times = tuple(
            time_s
            for time_s in shared_times
            if channel in batch_by_time[time_s] and channel in step_by_time[time_s]
        )
        if not channel_times:
            continue
        differences[channel] = max(
            abs(float(batch_by_time[time_s][channel]) - float(step_by_time[time_s][channel]))
            for time_s in channel_times
        )
    tolerance = 1.0e-9
    expected_shared = min(len(batch_rows), len(step_rows)) - 1
    passed = bool(shared_times) and len(shared_times) >= expected_shared and all(value <= tolerance for value in differences.values())
    return {
        "schema_version": "taoryx.batch-step-parity/v1",
        "status": "pass" if passed else "fail",
        "mode": "same-runtime-external-accepted-steps",
        "batch_samples": len(batch_rows),
        "step_samples": len(step_rows),
        "shared_timestamp_samples": len(shared_times),
        "terminal_event_refinement_difference_samples": len(batch_rows) + len(step_rows) - 2 * len(shared_times),
        "channels": differences,
        "tolerance": tolerance,
        "claim_boundary": "Parity covers common accepted fixed-step samples. The final stop-event refinement may add one batch-only endpoint.",
    }
    ####


def _problem_step(problem: Path) -> float:
    """Extract the one declared fixed integration step for a candidate witness."""

    match = re.search(r"\*integ\b[^\n]*\bdt=([-+0-9.eE]+)", problem.read_text(encoding="utf-8"))
    if match is None:
        raise ValueError(f"{problem} has no fixed integration step")
    step = float(match.group(1))
    if not math.isfinite(step) or step <= 0.0:
        raise ValueError(f"{problem} has an invalid fixed integration step")
    return step
    ####


def _scaled_problem(problem: Path, destination: Path, factor: float) -> Path:
    """Materialize a refinement input without changing the candidate source."""

    if factor <= 0.0:
        raise ValueError("step refinement factor must be positive")
    text = re.sub(
        r"(\bdt=)([-+0-9.eE]+)",
        lambda match: f"{match.group(1)}{float(match.group(2)) * factor:.16g}",
        problem.read_text(encoding="utf-8"),
    )
    destination.write_text(text, encoding="utf-8")
    return destination
    ####


def _materialized_mission(mission_config: Path, mission_id: str) -> dict[str, Any]:
    """Load the exact materialized mission contract selected for execution."""

    payload = yaml.safe_load(mission_config.read_text(encoding="utf-8"))
    missions = payload.get("missions", ()) if isinstance(payload, dict) else ()
    mission = next((item for item in missions if isinstance(item, dict) and item.get("id") == mission_id), None)
    if mission is None:
        raise KeyError(f"materialized mission {mission_id!r} is absent from {mission_config}")
    return mission
    ####


def _materialized_max_steps(mission_config: Path, mission_id: str) -> int:
    """Return the declared batch step budget for the resolved mission."""

    return int(_materialized_mission(mission_config, mission_id)["max_steps"])
    ####


def _run_materialized_summary(
    *,
    problem: Path,
    tables: tuple[Path, ...],
    mission_config: Path,
    racetrack_config: Path,
    candidate_mission_id: str,
    max_steps: int,
) -> dict[str, Any]:
    """Run the normal plant and re-evaluate truth objectives without rendering.

    Fixed-step convergence needs a second numerical witness, not a second
    visual packet.  This reuses the production file runner and the same
    materialized mission data, then applies the independent evaluator directly
    to its accepted truth samples.
    """

    mission = _materialized_mission(mission_config, candidate_mission_id)
    report = run_files(
        problem,
        tables,
        output_dir=problem.parent / "runtime",
        max_steps=max_steps,
        integrator=str(mission.get("integrator", "rk4")),
        profile=GrammarProfile.TAORYX,
    )
    states = tuple(next(iter(report.results[0].states.values()), ())) if report.results else ()
    rows = _local_rows(states)
    route = load_racetrack_template_catalog(racetrack_config).get(str(mission["racetrack_binding"]))
    objectives: list[TruthObjectiveSpec] = []
    for item in [*mission.get("objectives", ()), mission.get("terminal", {})]:
        if not isinstance(item, dict):
            continue
        resolved = dict(item)
        resolved.pop("racetrack_phase", None)
        gate_id = resolved.pop("racetrack_gate_id", None)
        if gate_id is not None:
            gate = next(candidate for candidate in route.gates if candidate.id == gate_id)
            resolved["target"] = gate.target(route.speed_m_s)
            resolved["gate_normal"] = gate.gate_normal()
        if resolved.get("id") is None:
            resolved["id"] = "terminal-contract"
        objectives.append(TruthObjectiveSpec(**resolved))
    envelope_pass = all(
        all(
            float(specification.get("minimum", -math.inf))
            <= float(row.get(str(specification["channel"]), math.nan))
            <= float(specification.get("maximum", math.inf))
            for row in rows
        )
        for specification in mission.get("envelope", ())
        if isinstance(specification, dict)
    )
    numerical_valid = report.exit_code == 0 and all(result.completed for result in report.results)
    evaluation = evaluate_truth_objectives(
        objectives,
        rows,
        hard_gates_passed=numerical_valid and envelope_pass,
    )
    return {
        "numerical_valid": numerical_valid,
        "mission_pass": bool(evaluation["mission_pass"]),
        "truth_evaluation": evaluation,
    }
    ####


def _refresh_packet_contract(packet: Path) -> None:
    """Refresh the packet manifest after composition-specific addenda exist.

    The reusable family builder writes its manifest before this compatibility
    driver adds generated route catalogs, candidate evidence, and optional
    compiled-composition provenance.  Re-hash the complete packet and rebuild
    its archive so the final artifact contract does not silently omit those
    files.
    """

    manifest_path = packet / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise ValueError(f"{manifest_path} must contain a mapping")
    paths = tuple(path for path in sorted(packet.rglob("*")) if path.is_file() and path != manifest_path)
    hashes = {str(path.relative_to(packet)): _sha256(path) for path in paths}
    manifest["files"] = hashes
    run_artifacts = manifest.get("run_artifacts")
    if isinstance(run_artifacts, list):
        for run_artifact in run_artifacts:
            if not isinstance(run_artifact, dict):
                continue
            existing = run_artifact.get("files")
            records = {item.get("path"): item for item in existing if isinstance(item, dict)} if isinstance(existing, list) else {}
            updated: list[dict[str, object]] = []
            scenario_hash = str(run_artifact.get("scenario_contract_sha256", ""))
            updated.append(
                {
                    "path": "manifest.json",
                    "sha256": scenario_hash,
                    "media_type": "application/json",
                    "required": True,
                }
            )
            for relative_path, digest in hashes.items():
                prior = records.get(relative_path)
                updated.append(
                    {
                        "path": relative_path,
                        "sha256": digest,
                        "media_type": (
                            prior.get("media_type")
                            if isinstance(prior, dict) and isinstance(prior.get("media_type"), str)
                            else mimetypes.guess_type(relative_path)[0] or "application/octet-stream"
                        ),
                        "required": True,
                    }
                )
            run_artifact["files"] = updated
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    archive = packet.parent / f"{packet.name}.zip"
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as handle:
        for path in sorted(packet.rglob("*")):
            if path.is_file():
                handle.write(path, path.relative_to(packet))
    ####


def _sha256(path: Path) -> str:
    """Return the stable content hash required by the artifact contract."""

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
    ####


def main() -> int:
    """Run the language-backed candidate adapter from the command line."""

    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--profile", choices=tuple(LANGUAGE_BACKED_MISSIONS))
    selection.add_argument("--composition", type=Path, help="compiled X8 vehicle composition JSON")
    parser.add_argument("--fidelity", choices=RACETRACK_FIDELITIES)
    parser.add_argument("--output", type=Path, default=ROOT / "verification/generated/language_backed_candidates")
    args = parser.parse_args()
    if args.profile is not None and args.fidelity is None:
        parser.error("--fidelity is required with --profile")
    execution = (
        execute_candidate(args.profile, args.fidelity, args.output)
        if args.profile is not None
        else execute_composition(load_compiled_vehicle_composition(args.composition), args.output)
    )
    print(json.dumps(execution, indent=2, sort_keys=True))
    return 0 if bool(execution["evaluation"]["mission_pass"]) else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
