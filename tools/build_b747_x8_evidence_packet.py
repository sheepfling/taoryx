"""Build a clean, hash-bound evidence packet for the B747 and X8 tranche."""

from __future__ import annotations

import argparse
import hashlib
import json
import mimetypes
import shutil
import uuid
import zipfile
from pathlib import Path
from typing import Any

from taoryx.fidelity_contracts import control_realization_for
from taoryx.language import GrammarProfile
from taoryx.runtime.runner import run_files
from taoryx.showcase import (
    ArtifactFile,
    EvidenceBoardSpec,
    FidelityShowcaseRealization,
    ShowcaseOutcome,
    build_showcase_run_artifact,
)
from taoryx.validation import independent_force_closure, independent_moment_closure
from taoryx.visualization import render_run_artifact_plots

ROOT = Path(__file__).resolve().parents[1]
TABLE_ROOT = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
CASES: tuple[dict[str, Any], ...] = (
    {
        "id": "b747-trim-6dof",
        "vehicle": "B747-100",
        "problem": "examples/mission_families/slower_b747/SV01_long_trim_hold_6dof.prb",
        "tables": ["b747_nominal_elevator_6axis.tbl"],
        "max_steps": 4000,
    },
    {
        "id": "b747-trim-120s-generated-6dof",
        "vehicle": "B747-100",
        "problem": "examples/mission_families/slower_b747/SV01_long_trim_hold_120_6dof.prb",
        "tables": ["b747_nominal_elevator_6axis.tbl"],
        "max_steps": 7000,
    },
    {
        "id": "b747-descent-recovery-120s-generated-6dof",
        "vehicle": "B747-100",
        "problem": "examples/mission_families/slower_b747/SV01_descent_recovery_120_6dof.prb",
        "tables": ["b747_nominal_elevator_6axis.tbl"],
        "max_steps": 7000,
    },
    {
        "id": "b747-approach-go-around-6dof",
        "vehicle": "B747-100",
        "problem": "examples/mission_families/slower_b747/SV01_approach_go_around_long_6dof.prb",
        "tables": ["b747_nominal_elevator_6axis.tbl"],
        "max_steps": 4000,
    },
    {
        "id": "b747-source-deck-reduction-3dof",
        "vehicle": "B747-100",
        "problem": "examples/mission_families/slower_b747/SV01_powered_trim_reduction_3dof.prb",
        "tables": ["b747_nominal_elevator_6axis.tbl", "b747_jt9d_thrust.tbl"],
        "max_steps": 100,
    },
    {
        "id": "b747-bounded-route-controller-6dof",
        "vehicle": "B747-100",
        "problem": "examples/mission_families/slower_b747/SV01_bounded_route_controller_6dof.prb",
        "tables": ["b747_nominal_elevator_6axis.tbl"],
        "max_steps": 600,
    },
    {
        "id": "x8-powered-validation-6dof",
        "vehicle": "Skywalker X8",
        "problem": "examples/mission_families/slower_x8/SV03_long_validation_6dof.prb",
        "tables": [
            "skywalker_x8_static_6axis.tbl",
            "skywalker_x8_collective_elevon_6axis.tbl",
            "skywalker_x8_differential_elevon_6axis.tbl",
            "skywalker_x8_thrust.tbl",
        ],
        "max_steps": 13000,
    },
    {
        "id": "x8-source-composed-trim-hold-30s",
        "vehicle": "Skywalker X8",
        "problem": "examples/mission_families/slower_x8/SV03_source_trim_hold_30_6dof.prb",
        "tables": [
            "skywalker_x8_static_6axis.tbl",
            "skywalker_x8_collective_elevon_6axis.tbl",
            "skywalker_x8_differential_elevon_6axis.tbl",
            "skywalker_x8_thrust.tbl",
        ],
        "max_steps": 7000,
    },
    {
        "id": "x8-controller-recovery-10s-generated-6dof",
        "vehicle": "Skywalker X8",
        "problem": "examples/generated/vehicles/skywalker_x8_controller_recovery_10_6dof.prb",
        "tables": [
            "skywalker_x8_static_6axis.tbl",
            "skywalker_x8_collective_elevon_6axis.tbl",
            "skywalker_x8_differential_elevon_6axis.tbl",
            "skywalker_x8_thrust.tbl",
        ],
        "max_steps": 3000,
    },
    {
        "id": "x8-level-settling-corridor-6dof",
        "vehicle": "Skywalker X8",
        "problem": "examples/mission_families/slower_x8/SV03_long_level_settling_6dof.prb",
        "tables": [
            "skywalker_x8_static_6axis.tbl",
            "skywalker_x8_collective_elevon_6axis.tbl",
            "skywalker_x8_differential_elevon_6axis.tbl",
            "skywalker_x8_thrust.tbl",
        ],
        "max_steps": 25000,
    },
    {
        "id": "x8-four-leg-route-6dof",
        "vehicle": "Skywalker X8",
        "problem": "examples/mission_families/slower_x8/SV03_long_rectangle_route_6dof.prb",
        "tables": [
            "skywalker_x8_static_6axis.tbl",
            "skywalker_x8_collective_elevon_6axis.tbl",
            "skywalker_x8_differential_elevon_6axis.tbl",
            "skywalker_x8_thrust.tbl",
        ],
        "max_steps": 7000,
    },
    {
        "id": "x8-basic-waypoint-capture-6dof",
        "vehicle": "Skywalker X8",
        "problem": "examples/mission_families/slower_x8/SV03_basic_waypoint_altitude_6dof.prb",
        "tables": [
            "skywalker_x8_static_6axis.tbl",
            "skywalker_x8_collective_elevon_6axis.tbl",
            "skywalker_x8_differential_elevon_6axis.tbl",
            "skywalker_x8_thrust.tbl",
        ],
        "max_steps": 5000,
    },
    {
        "id": "x8-source-deck-reduction-3dof",
        "vehicle": "Skywalker X8",
        "problem": "examples/generated/vehicles/skywalker_x8_source_trim_reduction_3dof.prb",
        "tables": [
            "skywalker_x8_static_6axis.tbl",
            "skywalker_x8_collective_elevon_6axis.tbl",
            "skywalker_x8_differential_elevon_6axis.tbl",
            "skywalker_x8_thrust.tbl",
        ],
        "max_steps": 20,
    },
)
PLOT_CHANNELS = (
    "position.altitude.geodetic",
    "kinematics.speed",
    "aero.airspeed",
    "aero.dynamic_pressure",
    "aero.force.body.x",
    "aero.force.body.y",
    "aero.force.body.z",
    "aero.moment.body.x",
    "aero.moment.body.y",
    "aero.moment.body.z",
    "attitude.roll",
    "attitude.pitch",
    "attitude.yaw",
    "guidance.pro-nav-acceleration",
    "route.target-error",
    "taos.route_corner_0_error_m",
    "taos.route_corner_1_error_m",
    "taos.route_corner_2_error_m",
    "taos.route_corner_3_error_m",
)
SHOWCASE_MODULES = (
    "trajectory_3d",
    "mission_timeline",
    "energy_and_resources",
    "attitude_and_rates",
    "semantic_controls",
    "physical_effectors",
    "envelope_margins",
    "terminal_corridor",
)
####


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
####


def _payload_sha256(value: object) -> str:
    """Hash resolved case inputs without depending on generated outputs."""

    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
####


def _artifact_files(packet: Path, scenario_hash: str) -> tuple[ArtifactFile, ...]:
    """Describe one legacy packet with the common artifact boundary."""

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


def _case_realization(case: dict[str, Any]) -> FidelityShowcaseRealization:
    """Resolve each historical case to an explicit canonical tier."""

    is_3dof = str(case["id"]).endswith("3dof")
    tier = "point_mass_3dof" if is_3dof else "rigid_body_6dof_surface_allocated"
    controls = ("force_model",) if is_3dof else ("surface_allocated",)
    effectors: tuple[str, ...] = ()
    if not is_3dof:
        effectors = (
            ("elevator_deg",)
            if str(case["vehicle"]) == "B747-100"
            else ("collective_elevon_deg", "differential_elevon_deg", "throttle_fraction")
        )
    vehicle_slug = "b747" if str(case["vehicle"]) == "B747-100" else "skywalker_x8"
    control = control_realization_for(tier)
    assert control == controls[0]
    return FidelityShowcaseRealization(
        fidelity=tier,
        control_realization=control,
        realization_id=f"{vehicle_slug}.legacy_evidence.{case['id']}.v1",
        state_schema=("geodetic_point_mass_3dof",) if is_3dof else ("ecic_rigid_body_6dof",),
        semantic_command_mapping={
            "vehicle.controls": "declared runtime controls in source problem",
            "vehicle.propulsion": "declared propulsion schedule in source problem",
        },
        physical_effectors=effectors,
        available_physics=(
            "source-bounded aerodynamic loads",
            "declared propulsion and mass properties",
            "point-mass guidance" if is_3dof else "rigid-body translation and rotation",
            "bounded declared surface-control path" if not is_3dof else "no physical effector claim",
        ),
        claim=(
            f"The {case['vehicle']} {case['id']} case is a source-bounded "
            "research-surrogate execution record with explicit fidelity and "
            "control-path provenance."
        ),
        nonclaims=(
            "family-wide qualification",
            "flight validation",
            "unmodeled actuator, propulsion, or aerodynamic behavior",
            "direct-wrench results being equivalent to physical-effector validation",
        ),
        evidence_grade="mixed",
    )
####


def _case_outcome(case: dict[str, Any], report: Any) -> ShowcaseOutcome:
    """Map runtime completion to an honest artifact outcome."""

    if report.exit_code != 0 or not report.results:
        return "numerical_failure"
    return "completed" if all(result.completed for result in report.results) else "partial"
####


def _files(root: Path) -> tuple[Path, ...]:
    return tuple(path for path in sorted(root.rglob("*")) if path.is_file())
####


def _event_times(report: Any) -> tuple[float, ...]:
    """Return declared runtime event times for discontinuity-aware closure."""

    values: list[float] = []
    for artifact in report.artifacts:
        for raw_event in artifact.events:
            event: Any = raw_event
            if isinstance(event, dict) and "time" in event:
                values.append(float(event["time"]))
    return tuple(values)
####


def _portable(path: Path, packet: Path) -> str:
    return str(path.relative_to(packet))
####


def _closure_report(report: Any) -> dict[str, Any] | None:
    """Build independent closure evidence from a rigid-body run report."""

    if not report.results or not report.results[0].states:
        return None
    states = next(iter(report.results[0].states.values()))
    history = tuple({"time_s": state.time, **dict(state.named)} for state in states)
    required_force = {
        "mass_kg",
        "xdt",
        "ydt",
        "zdt",
        "total_force_ecic_x_n",
        "total_force_ecic_y_n",
        "total_force_ecic_z_n",
    }
    required_moment = {
        "wx",
        "wy",
        "wz",
        "total_moment_body_x_nm",
        "total_moment_body_y_nm",
        "total_moment_body_z_nm",
    }
    if not required_force.issubset(history[0]) or not required_moment.issubset(history[0]):
        return None
    metadata = report.metadata[0] if report.metadata else {}
    vehicle = metadata.get("vehicle", {}) if isinstance(metadata, dict) else {}
    inertia = tuple(float(vehicle[name]) for name in ("inertia-x", "inertia-y", "inertia-z") if name in vehicle)
    if len(inertia) != 3:
        return None
    enriched = tuple(
        {
            **sample,
            "inertia_x_kg_m2": inertia[0],
            "inertia_y_kg_m2": inertia[1],
            "inertia_z_kg_m2": inertia[2],
        }
        for sample in history
    )
    event_times = _event_times(report)
    return {
        "independent_translation": independent_force_closure(enriched, event_times=event_times),
        "independent_rotation": independent_moment_closure(enriched, event_times=event_times),
    }


def _route_corner_report(report: Any) -> dict[str, Any] | None:
    """Extract fixed-corner errors for a route review without lossy plots."""

    capture_gate_m = 25.0

    if not report.results or not report.results[0].states:
        return None
    states = next(iter(report.results[0].states.values()))
    samples = tuple(
        {
            "time_s": state.time,
            "leg_index": state.named.get("route_leg_index"),
            **{
                f"corner_{index}_error_m": state.named.get(f"route_corner_{index}_error_m")
                for index in range(4)
            },
        }
        for state in states
        if all(f"route_corner_{index}_error_m" in state.named for index in range(4))
    )
    if not samples:
        return None
    boundary_samples = tuple(
        min(
            samples,
            key=lambda sample: abs(float(sample["time_s"]) - boundary),
        )
        for boundary in (30.0, 60.0, 90.0, 120.0)
    )
    boundary_errors = {
        f"corner_{index}": max(
            float(sample[f"corner_{index}_error_m"]) for sample in boundary_samples
        )
        for index in range(4)
    }
    return {
        "sample_count": len(samples),
        "corner_boundary_times_s": [30.0, 60.0, 90.0, 120.0],
        "corner_boundary_samples": boundary_samples,
        "capture_gate_m": capture_gate_m,
        "capture_status": "pass" if max(boundary_errors.values()) <= capture_gate_m else "blocked",
        "boundary_maximum_error_m": boundary_errors,
        "maximum_error_m": {
            f"corner_{index}": max(float(sample[f"corner_{index}_error_m"]) for sample in samples)
            for index in range(4)
        },
    }
    ####
####


def build(
    output: Path,
    *,
    plots: bool = True,
    case_ids: tuple[str, ...] | None = None,
) -> Path:
    """Run the declared cases and return the resulting zip packet."""

    import sys

    sys.path.insert(0, str(ROOT))
    from tests.e2e.test_golden_source_differential import source_differential_reports

    run_id = str(uuid.uuid4())
    packet = output / run_id
    packet.mkdir(parents=True, exist_ok=False)
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "run_id": run_id,
        "claim_boundary": "source-bounded research-surrogate evidence; not flight qualification",
        "metadata_inputs": ["claims.md", "controller_scenarios.yaml", "problem_generation.yaml"],
        "selected_cases": list(case_ids) if case_ids is not None else "all",
        "cases": [],
    }
    for metadata_name in ("verification/claims.md", "verification/controller_scenarios.yaml", "verification/problem_generation.yaml"):
        source = ROOT / metadata_name
        shutil.copy2(source, packet / source.name)
    (packet / "source-differential-report.json").write_text(
        json.dumps(source_differential_reports(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    case_records: list[tuple[dict[str, Any], Any]] = []
    selected_cases = CASES if case_ids is None else tuple(case for case in CASES if case["id"] in case_ids)
    unknown_cases = set(case_ids or ()) - {str(case["id"]) for case in CASES}
    if unknown_cases:
        raise ValueError(f"unknown evidence case ids: {sorted(unknown_cases)}")
    if not selected_cases:
        raise ValueError("case selection produced no evidence cases")
    for case in selected_cases:
        case_dir = packet / "cases" / str(case["id"])
        case_dir.mkdir(parents=True, exist_ok=True)
        problem = ROOT / str(case["problem"])
        input_dir = case_dir / "inputs"
        input_dir.mkdir()
        problem_copy = input_dir / problem.name
        shutil.copy2(problem, problem_copy)
        table_paths: list[Path] = []
        for table_name in case["tables"]:
            source = TABLE_ROOT / str(table_name)
            destination = input_dir / source.name
            shutil.copy2(source, destination)
            table_paths.append(source)
        report = run_files(
            problem,
            tuple(table_paths),
            output_dir=case_dir / "run",
            max_steps=int(case["max_steps"]),
            integrator="rk4",
            profile=GrammarProfile.TAORYX,
        )
        case_records.append((case, report))
        (case_dir / "run-report.json").write_text(
            json.dumps(
                {
                    "case": case,
                    "exit_code": report.exit_code,
                    "diagnostics": [{"code": item.code, "message": item.message} for item in report.diagnostics],
                    "results": [
                        {"completed": result.completed, "state_counts": {name: len(states) for name, states in result.states.items()}}
                        for result in report.results
                    ],
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        closure = _closure_report(report)
        if closure is not None:
            (case_dir / "closure-report.json").write_text(
                json.dumps(closure, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        route_corners = _route_corner_report(report)
        if route_corners is not None:
            (case_dir / "route-corner-report.json").write_text(
                json.dumps(route_corners, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        if plots:
            for artifact in report.artifacts:
                render_run_artifact_plots(artifact, case_dir / "plots", vehicle_id="1", channels=PLOT_CHANNELS)
        manifest["cases"].append(
            {
                "id": case["id"],
                "vehicle": case["vehicle"],
                "problem": str(Path("cases") / str(case["id"]) / "inputs" / problem.name),
                "tables": [str(Path("cases") / str(case["id"]) / "inputs" / path.name) for path in table_paths],
                "runtime_exit_code": report.exit_code,
            }
        )
    run_artifacts = []
    for case, report in case_records:
        realization = _case_realization(case)
        scenario_contract_hash = _payload_sha256(
            {
                "case": case,
                "problem_sha256": _sha256(ROOT / str(case["problem"])),
                "tables_sha256": [
                    _sha256(TABLE_ROOT / str(table_name)) for table_name in case["tables"]
                ],
                "fidelity": realization.fidelity,
                "control_realization": realization.control_realization,
            }
        )
        showcase_run = build_showcase_run_artifact(
            realization=realization,
            run_id=f"{run_id}-{case['id']}",
            showcase_id=f"org.taoryx.showcase.b747-x8-evidence.{case['id']}",
            vehicle_binding_id=(
                "b747-100.source-bounded-v1"
                if str(case["vehicle"]) == "B747-100"
                else "skywalker-x8.source-bounded-v1"
            ),
            scenario_contract_sha256=scenario_contract_hash,
            outcome=_case_outcome(case, report),
            files=_artifact_files(packet, scenario_contract_hash),
            board=EvidenceBoardSpec(profile="family-evidence-board-v1", modules=SHOWCASE_MODULES),
            archetypes=(
                "mission_geometry",
                "mission_timeline",
                "dynamics_and_resources",
                "envelope_and_qualification",
            ),
        )
        run_artifacts.append(showcase_run.model_dump(mode="json"))
    manifest["run_artifacts"] = run_artifacts
    manifest_path = packet / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    # A manifest cannot contain its own final digest without recursive
    # self-reference.  The manifest is therefore the trust root and is
    # deliberately excluded from the file-hash map.
    files = tuple(path for path in _files(packet) if path != manifest_path)
    manifest["files"] = { _portable(path, packet): _sha256(path) for path in files }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    archive = output / f"b747-x8-evidence-{run_id}.zip"
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as handle:
        for path in _files(packet):
            handle.write(path, _portable(path, packet))
    return archive
####


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/verification/b747_x8_v1")
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--case", dest="case_ids", action="append", help="run one declared case; repeat to select several")
    arguments = parser.parse_args()
    arguments.output.mkdir(parents=True, exist_ok=True)
    selected = None if arguments.case_ids is None else tuple(arguments.case_ids)
    print(build(arguments.output, plots=not arguments.no_plots, case_ids=selected))
####


if __name__ == "__main__":
    main()
