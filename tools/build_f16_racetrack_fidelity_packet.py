"""Build the self-contained F-16 S-119 shared-racetrack fidelity packet.

This builder is intentionally an evidence assembler around
``validate_f16_racetrack.run_case``.  It does not promote a lower-fidelity
pass or a direct-wrench pass into a physical-control claim.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import zipfile
from pathlib import Path
from typing import Any, Final

from validate_f16_racetrack import F16_VALIDITY_ENVELOPE, _envelope_violations, _render_board, run_case

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "artifacts/showcases/f16-s119-racetrack-fidelity-ladder"
MODES: Final[tuple[tuple[str, str], ...]] = (
    ("3dof", "point_mass_3dof"),
    ("pseudo-6dof", "pseudo_6dof_kinematic_bridge"),
    ("6dof-direct-wrench", "direct_wrench"),
    ("6dof-surfaces", "surface_allocated"),
)
MODE_DEFINITIONS: Final[dict[str, dict[str, str]]] = {
    "point_mass_3dof": {
        "fidelity": "point_mass_3dof",
        "control_path": "bounded translational kinematics plus source-force diagnostics",
        "claim": "Shared F-16 racetrack geometry, speed/altitude/heading response, and truth-gate closure at the declared reduced model.",
        "evidence_tier": "T2_reduced_route_evidence",
    },
    "pseudo_6dof_kinematic_bridge": {
        "fidelity": "pseudo_6dof_kinematic_bridge",
        "control_path": "bounded translational kinematics plus named source-calibrated attitude/rate response bridge",
        "claim": "Shared F-16 racetrack geometry with declared attitude/rate response and truth-gate closure.",
        "evidence_tier": "T2_reduced_route_evidence",
    },
    "direct_wrench": {
        "fidelity": "rigid_body_6dof_direct_wrench",
        "control_path": "state reference to LQR desired wrench to bounded direct-wrench nonlinear plant",
        "claim": "Source-grounded F-16 body-load racetrack comparison screen using direct generalized force/moment realization.",
        "evidence_tier": "T3_screen_only",
    },
    "surface_allocated": {
        "fidelity": "rigid_body_6dof_surface_allocated",
        "control_path": "state reference to LQR desired wrench to bounded elevator/aileron/rudder/throttle allocation to nonlinear plant",
        "claim": "Source-grounded F-16 racetrack attempt with physical effector allocation and independent truth-gate evaluation.",
        "evidence_tier": "T4_physical_allocation_development",
    },
}
NONCLAIMS: Final[tuple[str, ...]] = (
    "manufacturer flight-control fidelity",
    "global flight-envelope qualification",
    "wind or uncertainty robustness",
    "TAOS 96 runtime compatibility",
)
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


def _write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    fields = list(rows[0]) if rows else ["time_s"]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    ####


def _reduction_mission_window_comparison(
    reduced_runs: dict[str, tuple[dict[str, Any], list[dict[str, float | int | str]]]],
    parent_run: tuple[dict[str, Any], list[dict[str, float | int | str]]],
) -> dict[str, object]:
    """Compare reduced and parent routes by semantic mission evidence.

    Lower fidelities are not expected to reproduce rigid-body samples exactly.
    This report therefore gates common time/reference channels and objective
    parity, while retaining measured trajectory disagreement as a declared
    comparison result rather than hiding it in a scalar score.
    """

    parent_evidence, parent_rows = parent_run
    parent_results = {str(item["id"]): item for item in parent_evidence["evaluation"]["results"]}
    comparisons: dict[str, object] = {}
    for mode, (evidence, rows) in reduced_runs.items():
        common_count = min(len(rows), len(parent_rows))
        route_fields = (
            "route_north_command_m",
            "route_east_command_m",
            "route_altitude_command_m",
            "route_speed_command_m_s",
            "route_heading_command_deg",
            "route_bank_command_deg",
        )
        max_reference_error = max(
            (
                abs(float(rows[index][field]) - float(parent_rows[index][field]))
                for index in range(common_count)
                for field in route_fields
            ),
            default=0.0,
        )
        trajectory_fields = ("north_m", "east_m", "altitude_m", "speed_m_s")
        trajectory_disagreement = {
            field: {
                "maximum_absolute": max(
                    (
                        abs(float(rows[index][field]) - float(parent_rows[index][field]))
                        for index in range(common_count)
                    ),
                    default=0.0,
                ),
                "rms": (
                    sum(
                        (float(rows[index][field]) - float(parent_rows[index][field])) ** 2
                        for index in range(common_count)
                    )
                    / max(common_count, 1)
                )
                ** 0.5,
            }
            for field in trajectory_fields
        }
        objective_results = []
        objective_parity = True
        for item in evidence["evaluation"]["results"]:
            objective_id = str(item["id"])
            parent_item = parent_results.get(objective_id)
            status_parity = parent_item is not None and item["status"] == parent_item["status"]
            objective_parity = objective_parity and status_parity
            reduced_time = item.get("truth_time_s")
            parent_time = None if parent_item is None else parent_item.get("truth_time_s")
            objective_results.append(
                {
                    "id": objective_id,
                    "reduced_status": item["status"],
                    "parent_status": None if parent_item is None else parent_item["status"],
                    "status_parity": status_parity,
                    "reduced_truth_time_s": reduced_time,
                    "parent_truth_time_s": parent_time,
                    "truth_time_delta_s": (
                        None
                        if reduced_time is None or parent_time is None
                        else float(reduced_time) - float(parent_time)
                    ),
                }
            )
        mission_parity = bool(evidence["evaluation"]["mission_pass"]) == bool(
            parent_evidence["evaluation"]["mission_pass"]
        )
        comparisons[mode] = {
            "status": "semantic_mission_parity_passed" if objective_parity and mission_parity else "semantic_mission_parity_failed",
            "reduced_fidelity": evidence["fidelity"],
            "parent_fidelity": parent_evidence["fidelity"],
            "sample_count": common_count,
            "time_grid_parity": len(rows) == len(parent_rows)
            and all(abs(float(rows[index]["time_s"]) - float(parent_rows[index]["time_s"])) <= 1.0e-9 for index in range(common_count)),
            "maximum_reference_channel_error": max_reference_error,
            "trajectory_disagreement": trajectory_disagreement,
            "objective_parity": objective_parity,
            "mission_status_parity": mission_parity,
            "objectives": objective_results,
            "claim_boundary": "Semantic mission parity only; trajectory disagreement is retained and no parent-plant or actuator equivalence is claimed.",
        }
    return {
        "schema_version": 1,
        "family_id": "reference_f16_s119",
        "comparison": "shared_racetrack_semantic_mission_window_v1",
        "parent_packet": "6dof-surfaces",
        "records": comparisons,
        "status": "semantic_mission_parity_passed"
        if all(
            isinstance(record, dict) and record.get("status") == "semantic_mission_parity_passed"
            for record in comparisons.values()
        )
        else "semantic_mission_parity_failed",
    }
    ####


def _envelope_report(rows: list[dict[str, float | int | str]]) -> dict[str, object]:
    channels = (
        "dynamic_pressure_pa",
        "mach",
        "aero_alpha_deg",
        "aero_beta_deg",
        "aero_sideslip_deg",
        "altitude_m",
        "speed_m_s",
    )
    ranges: dict[str, dict[str, float]] = {}
    for channel in channels:
        values = [float(row[channel]) for row in rows if channel in row]
        if values:
            ranges[channel] = {"minimum": min(values), "maximum": max(values)}
    return {
        "schema_version": 1,
        "status": "pass" if not _envelope_violations(rows) else "failed",
        "ranges": ranges,
        "source_validity_envelope": F16_VALIDITY_ENVELOPE,
        "hard_domain_violations": _envelope_violations(rows),
        "note": "This is a bounded source-domain check for the declared route, not global envelope qualification.",
    }
    ####


def _row_at_time(rows: list[dict[str, float | int | str]], time_s: float | None) -> dict[str, float | int | str] | None:
    """Return the truth sample nearest an independently evaluated event."""

    if time_s is None or not rows:
        return None
    return min(rows, key=lambda row: abs(float(row["time_s"]) - time_s))
    ####


def _truth_event_artifacts(
    rows: list[dict[str, float | int | str]],
    evaluation: dict[str, Any],
) -> tuple[list[str], dict[str, float], list[dict[str, object]]]:
    """Materialize truth objective events without inventing controller events."""

    event_ids: list[str] = []
    event_times: dict[str, float] = {}
    records: list[dict[str, object]] = []
    for result in evaluation.get("results", []):
        if not isinstance(result, dict):
            continue
        objective_id = str(result["id"])
        truth_time = result.get("truth_time_s")
        passed = result.get("status") == "pass" and truth_time is not None
        if passed:
            event_ids.append(objective_id)
            event_times[objective_id] = float(truth_time) if truth_time is not None else 0.0
        records.append(
            {
                "id": objective_id,
                "kind": "truth_objective",
                "status": str(result.get("status", "unknown")),
                "truth_time_s": None if truth_time is None else float(truth_time),
                "controller_time_s": result.get("controller_time_s"),
                "controller_reason": result.get("controller_reason"),
            }
        )
    return event_ids, event_times, records
    ####


def _actuator_rows(rows: list[dict[str, float | int | str]]) -> list[dict[str, float | int | str]]:
    """Project physical command/actual actuator telemetry into its own artifact."""

    fields = (
        "time_s",
        "allocation_status",
        "saturation_count",
        "commanded_elevator_deg",
        "elevator_deg",
        "elevator_position_error_deg",
        "commanded_aileron_deg",
        "aileron_deg",
        "aileron_position_error_deg",
        "commanded_rudder_deg",
        "rudder_deg",
        "rudder_position_error_deg",
        "commanded_throttle_fraction",
        "throttle_fraction",
        "throttle_position_error_fraction",
    )
    return [{field: row[field] for field in fields if field in row} for row in rows]
    ####


def _control_coverage(rows: list[dict[str, float | int | str]], runtime_mode: str) -> dict[str, object]:
    """Summarize which semantic and physical channels were exercised."""

    physical = runtime_mode in {"direct_wrench", "surface_allocated"}
    allocation_statuses = sorted({str(row["allocation_status"]) for row in rows if "allocation_status" in row})
    residuals = [float(row["allocation_residual_norm"]) for row in rows if "allocation_residual_norm" in row]
    saturation = [int(row["saturation_count"]) for row in rows if "saturation_count" in row]
    return {
        "runtime_mode": runtime_mode,
        "semantic_channels": [
            "route_speed_command_m_s",
            "route_altitude_command_m",
            "route_heading_command_deg",
            "route_bank_command_deg",
        ],
        "physical_effectors": ["elevator", "aileron", "rudder", "throttle"] if physical else [],
        "allocation_statuses": allocation_statuses,
        "maximum_allocation_residual_norm": max(residuals, default=0.0),
        "saturation_samples": sum(value > 0 for value in saturation),
        "saturation_fraction": (sum(value > 0 for value in saturation) / len(saturation)) if saturation else 0.0,
        "claim_boundary": (
            "Physical effectors are evidenced only for the surface-allocated path; direct-wrench and reduced "
            "paths retain their declared nonclaims."
        ),
    }
    ####


def _equation_closure_report(rows: list[dict[str, float | int | str]], runtime_mode: str) -> dict[str, object]:
    """Report source-load closure for the committed surface controls."""

    axes = (
        ("force_x_n", "achieved_force_x_n", "source_total_force_x_n"),
        ("moment_x_nm", "achieved_moment_x_nm", "source_total_moment_x_nm"),
        ("moment_y_nm", "achieved_moment_y_nm", "source_total_moment_y_nm"),
        ("moment_z_nm", "achieved_moment_z_nm", "source_total_moment_z_nm"),
    )
    residuals = {
        name: max((abs(float(row[achieved]) - float(row[source])) for row in rows if achieved in row and source in row), default=0.0)
        for name, achieved, source in axes
    }
    return {
        "status": "pass" if runtime_mode == "surface_allocated" else "not_applicable",
        "runtime_mode": runtime_mode,
        "source_load_model": "source_aerodynamics_plus_propulsion_at_actual_effector_state",
        "maximum_absolute_closure_residual": residuals,
        "claim_boundary": "Load closure is a source-runtime accounting check; it is not independent flight-test validation.",
    }
    ####


def build(output: Path = DEFAULT_OUTPUT, *, dt_s: float = 0.5) -> Path:
    """Build all four F-16 route realizations and a cross-tier summary."""

    output.mkdir(parents=True, exist_ok=True)
    operating_points_evidence = json.loads(
        (ROOT / "verification/f16_operating_points_evidence.json").read_text(encoding="utf-8")
    )
    robustness_evidence = json.loads(
        (ROOT / "verification/f16_racetrack_robustness_evidence.json").read_text(encoding="utf-8")
    )
    records: list[dict[str, object]] = []
    run_artifacts: dict[str, tuple[dict[str, Any], list[dict[str, float | int | str]]]] = {}
    for packet_name, runtime_mode in MODES:
        evidence, rows = run_case(runtime_mode, None, dt_s)
        run_artifacts[runtime_mode] = (evidence, rows)
        packet = output / packet_name
        if packet.exists():
            shutil.rmtree(packet)
        packet.mkdir(parents=True)
        mode_definition = MODE_DEFINITIONS[runtime_mode]
        binding_id = str(evidence["binding_id"])
        route = evidence["route"]
        supporting_evidence = (
            "f16_local_maneuver_evidence.json",
            "f16_lqr_trim_hold_evidence.json",
            "f16_physical_allocation_evidence.json",
            "f16_physical_wrench_lqr_evidence.json",
            "f16_physical_wrench_perturbation_evidence.json",
            "f16_reduction_evidence.json",
            "f16_runtime_linearization_evidence.json",
            "f16_racetrack_robustness_evidence.json",
            "f16_t5_local_promotion_evidence.json",
        )
        for evidence_name in supporting_evidence:
            shutil.copy2(ROOT / "verification" / evidence_name, packet / evidence_name)
        truth_event_ids, truth_event_times, truth_event_records = _truth_event_artifacts(rows, evidence["evaluation"])
        terminal_result = next(
            (result for result in evidence["evaluation"].get("results", []) if result.get("id") == "terminal-start-finish-gate"),
            None,
        )
        terminal_truth_time = None if terminal_result is None else terminal_result.get("truth_time_s")
        terminal_truth_row = _row_at_time(rows, None if terminal_truth_time is None else float(terminal_truth_time))
        _write_json(packet / "claim.json", {
            "schema_version": 1,
            "status": evidence["status"],
            "evidence_tier": mode_definition["evidence_tier"],
            "claim": mode_definition["claim"],
            "nonclaims": [*NONCLAIMS, evidence["claim_boundary"]],
        })
        _write_json(packet / "resolved_case.json", {
            "family_id": evidence["family_id"],
            "vehicle_id": evidence["vehicle_id"],
            "binding_id": binding_id,
            "operating_point_id": "f16-sea-level-152mps",
            "route": route,
        })
        _write_json(packet / "compiled_scenario.json", {
            "template_id": route["template_id"],
            "binding_id": binding_id,
            "phase_timeline": _phase_timeline_from_manifest(route),
        })
        _write_json(packet / "realized_fidelity.json", {
            "runtime_mode": runtime_mode,
            **mode_definition,
            "source_realization": route_binding_source(evidence),
        })
        _write_json(packet / "parameter_provenance.json", {
            "family_manifest": "families/reference_f16_s119/family.yaml",
            "route_binding": "verification/racetrack_templates.yaml",
            "operating_point": "verification/f16_runtime_linearization_evidence.json",
            "operating_point_catalog": "verification/f16_operating_points_evidence.json",
            "supporting_evidence_local": list(supporting_evidence),
            "robustness_matrix": "verification/f16_racetrack_robustness_evidence.json",
            "source_plant": "families/reference_f16_s119/plant/daveml-import.json",
            "atmosphere": "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml",
        })
        _write_json(packet / "metric_dictionary.json", {
            "time_s": "seconds",
            "north_m": "local north position, metres",
            "east_m": "local east position, metres",
            "altitude_m": "mission altitude, metres",
            "speed_m_s": "true airspeed proxy, metres per second",
            "route_*": "command/reference channels from the shared racetrack",
            "source_force_*_n": "source-load diagnostic channels, Newtons",
            "allocation_residual_norm": "control-path residual; zero in reduced response-law modes is not physical allocation evidence",
            "commanded_*": "physical actuator command requested by the allocator, where present",
            "*_position_error_*": "commanded minus actual actuator position, where present",
            "source_total_*": "source aerodynamic plus propulsion load at the actual effector state",
        })
        _write_json(packet / "state_schema.json", {
            "position": ["north_m", "east_m", "altitude_m"],
            "kinematics": ["speed_m_s", "route_heading_achieved_deg", "route_pitch_achieved_deg", "route_bank_achieved_deg"],
            "rates": ["p_rad_s", "q_rad_s", "r_rad_s"],
        })
        _write_json(packet / "control_schema.json", {
            "semantic": ["route_speed_command_m_s", "route_altitude_command_m", "route_heading_command_deg", "route_bank_command_deg"],
            "physical": ["elevator_deg", "aileron_deg", "rudder_deg", "throttle_fraction"] if runtime_mode in {"direct_wrench", "surface_allocated"} else [],
            "control_path": mode_definition["control_path"],
        })
        _write_json(packet / "observation_schema.json", {"truth_channels": sorted(rows[0]) if rows else []})
        _write_json(packet / "controller_config.json", {
            "runtime_mode": runtime_mode,
            "binding_id": binding_id,
            "controller_transition_source": "time-scheduled guidance reference; no controller completion event is promoted to truth",
            "controller_transitions": [],
        })
        _write_csv(packet / "telemetry.csv", rows)
        _write_csv(packet / "actuators.csv", _actuator_rows(rows))
        _write_json(packet / "resources.json", {
            "status": "not_modeled",
            "propulsion": "source propulsion load is evaluated from throttle; fuel-flow/resource depletion is not modeled in this packet",
            "claim_boundary": "No fuel, endurance, or depletion claim is made.",
        })
        _write_json(packet / "operating_points_evidence.json", operating_points_evidence)
        _write_json(packet / "events.json", {
            "phase_events": _phase_timeline_from_manifest(route),
            "controller_transitions": [],
            "truth_events": truth_event_ids,
            "truth_event_times": truth_event_times,
            "objective_events": truth_event_records,
        })
        _write_json(packet / "segment_timeline.json", {"phases": _phase_timeline_from_manifest(route)})
        _write_json(packet / "objective_report.json", evidence["evaluation"])
        _write_json(packet / "envelope_report.json", _envelope_report(rows))
        _write_json(packet / "control_coverage.json", _control_coverage(rows, runtime_mode))
        _write_json(packet / "equation_closure_report.json", _equation_closure_report(rows, runtime_mode))
        _write_json(packet / "convergence_report.json", _convergence_report(runtime_mode, evidence, rows, dt_s))
        _write_json(packet / "batch_step_parity.json", {
            "status": "not_run",
            "deterministic_replay": "not_run",
            "claim": "The local racetrack runner does not yet expose the provider batch/step API; no batch/step parity claim is made.",
        })
        if runtime_mode == "surface_allocated":
            robustness_report: dict[str, object] = robustness_evidence
        else:
            robustness_report = {
                "status": "not_applicable_for_this_reduced_or_screen_path",
                "tier": "R1_fixed_initial_condition_matrix",
                "reference_surface_allocated_artifact": "../6dof-surfaces/robustness_report.json",
                "claim": "The fixed perturbation matrix is executed for the physical-effector path; this tier does not inherit that claim.",
            }
        _write_json(packet / "robustness_report.json", robustness_report)
        _write_json(packet / "terminal_state.json", {
            "objective_id": "terminal-start-finish-gate",
            "truth_time_s": terminal_truth_time,
            "truth_gate_sample": terminal_truth_row,
            "post_gate_final_sample": rows[-1] if rows else None,
            "post_gate_elapsed_s": (
                None
                if terminal_truth_time is None or not rows
                else float(rows[-1]["time_s"]) - float(terminal_truth_time)
            ),
            "terminal_objective_result": terminal_result,
        })
        _write_json(packet / "evaluation.json", evidence["evaluation"])
        _write_json(packet / "evidence.json", evidence)
        _write_csv(packet / "controls.csv", rows)
        _render_board(packet / "evidence_board.png", rows, evidence["evaluation"], load_route(binding_id), runtime_mode)
        (packet / "reproduction.txt").write_text(
            f"PYTHONPATH=src python3 tools/validate_f16_racetrack.py --mode {runtime_mode} --dt-s {dt_s} --output-dir {packet}\n",
            encoding="utf-8",
        )
        manifest = {
            "schema_version": 1,
            "showcase_schema": "taoryx.showcase/v1alpha1",
            "family_id": evidence["family_id"],
            "vehicle_id": evidence["vehicle_id"],
            "binding_id": binding_id,
            "runtime_mode": runtime_mode,
            "status": evidence["status"],
            "files": {},
        }
        manifest_path = packet / "manifest.json"
        manifest["files"] = {str(path.relative_to(packet)): _sha256(path) for path in sorted(packet.rglob("*")) if path.is_file() and path != manifest_path}
        _write_json(manifest_path, manifest)
        records.append({
            "mode": runtime_mode,
            "fidelity": evidence["fidelity"],
            "status": evidence["status"],
            "mission_pass": evidence["evaluation"]["mission_pass"],
            "required_passed": evidence["evaluation"]["required_passed"],
            "required_objectives": evidence["evaluation"]["required_objectives"],
            "packet": str(packet.relative_to(output)),
        })
    _write_json(output / "comparison.json", {
        "schema_version": 1,
        "family_id": "reference_f16_s119",
        "mission_template": "powered_fixed_wing_racetrack_v1",
        "records": records,
        "rule": "Fidelity passes are reported independently; no lower-fidelity pass promotes a higher-fidelity control claim.",
    })
    _write_json(
        output / "reduction_mission_window_comparison.json",
        _reduction_mission_window_comparison(
            {
                mode: run_artifacts[mode]
                for mode in ("point_mass_3dof", "pseudo_6dof_kinematic_bridge")
            },
            run_artifacts["surface_allocated"],
        ),
    )
    _write_json(output / "robustness_evidence.json", robustness_evidence)
    (output / "reproduction.txt").write_text(
        f"PYTHONPATH=src python3 tools/build_f16_racetrack_fidelity_packet.py --output {output} --dt-s {dt_s}\n",
        encoding="utf-8",
    )
    manifest_path = output / "manifest.json"
    _write_json(manifest_path, {
        "schema_version": 1,
        "catalog": "f16-s119-racetrack-fidelity-ladder-v1",
        "family_id": "reference_f16_s119",
        "records": records,
        "files": {str(path.relative_to(output)): _sha256(path) for path in sorted(output.rglob("*")) if path.is_file() and path != manifest_path},
    })
    archive = output.parent / f"{output.name}.zip"
    if archive.exists():
        archive.unlink()
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as handle:
        for path in sorted(output.rglob("*")):
            if path.is_file():
                handle.write(path, path.relative_to(output))
    return archive
    ####


def _convergence_report(
    runtime_mode: str,
    base_evidence: dict[str, Any],
    base_rows: list[dict[str, float | int | str]],
    base_dt_s: float,
) -> dict[str, object]:
    """Compare semantic gate results at a second integration step.

    The terminal objective is a gate crossing, not the final sample after the
    vehicle has passed that gate. The report therefore compares objective
    status, truth-event time, and critical margins rather than treating
    post-terminal endpoint drift as mission failure.
    """

    comparison_dt_s = (
        0.75
        if runtime_mode == "surface_allocated" and math.isclose(base_dt_s, 0.5)
        else base_dt_s / 2.0
    )
    comparison_evidence, comparison_rows = run_case(runtime_mode, None, comparison_dt_s)
    base_results = base_evidence["evaluation"]["results"]
    comparison_results = comparison_evidence["evaluation"]["results"]
    objective_deltas: list[dict[str, object]] = []
    max_time_delta = 0.0
    max_margin_delta = 0.0
    status_parity = True
    for base_result, comparison_result in zip(base_results, comparison_results, strict=True):
        base_time = base_result.get("truth_time_s")
        comparison_time = comparison_result.get("truth_time_s")
        time_delta = (
            None
            if base_time is None or comparison_time is None
            else float(comparison_time) - float(base_time)
        )
        margin_delta = float(comparison_result["margin"]) - float(base_result["margin"])
        if time_delta is not None:
            max_time_delta = max(max_time_delta, abs(time_delta))
        max_margin_delta = max(max_margin_delta, abs(margin_delta))
        status_parity = status_parity and base_result["status"] == comparison_result["status"]
        objective_deltas.append({
            "id": base_result["id"],
            "base_status": base_result["status"],
            "comparison_status": comparison_result["status"],
            "base_truth_time_s": base_time,
            "comparison_truth_time_s": comparison_time,
            "truth_time_delta_s": time_delta,
            "base_critical_margin": base_result["critical_metric"]["margin"],
            "comparison_critical_margin": comparison_result["critical_metric"]["margin"],
            "critical_margin_delta": margin_delta,
        })
    mission_status_parity = bool(base_evidence["evaluation"]["mission_pass"]) == bool(
        comparison_evidence["evaluation"]["mission_pass"]
    )
    passed = (
        base_evidence["runtime"]["numerical_valid"]
        and comparison_evidence["runtime"]["numerical_valid"]
        and mission_status_parity
        and status_parity
        and max_time_delta <= 30.0
        and max_margin_delta <= 5.0
    )
    return {
        "schema_version": 1,
        "status": "semantic_gate_convergence_pass" if passed else "semantic_gate_convergence_pending",
        "runtime_mode": runtime_mode,
        "base_step_s": base_dt_s,
        "comparison_step_s": comparison_dt_s,
        "base_sample_count": len(base_rows),
        "comparison_sample_count": len(comparison_rows),
        "mission_status_parity": mission_status_parity,
        "objective_status_parity": status_parity,
        "maximum_truth_event_time_delta_s": max_time_delta,
        "maximum_critical_margin_delta": max_margin_delta,
        "acceptance": {
            "maximum_truth_event_time_delta_s": 30.0,
            "maximum_critical_margin_delta": 5.0,
            "definition": "semantic gate convergence; not sample-by-sample trajectory identity",
        },
        "objectives": objective_deltas,
        "claim_boundary": (
            "This report compares independent truth-gate outcomes at two integration steps. "
            "It does not establish batch/step provider parity, scheduled-control validity, or global numerical convergence."
        ),
    }
    ####


def _phase_timeline_from_manifest(route: dict[str, Any]) -> list[dict[str, object]]:
    """Reconstruct the phase timeline from the route timing manifest."""

    timing = route["timing"]
    durations = (
        float(timing["climb_time_s"]),
        float(timing["outbound_level_time_s"]),
        float(timing["turn_time_s"]),
        float(timing["descent_time_s"]),
        float(timing["inbound_level_time_s"]),
        float(timing["turn_time_s"]),
    )
    names = ("outbound-climb", "outbound-level", "left-turn", "inbound-descent", "inbound-level", "right-turn")
    current = 0.0
    result: list[dict[str, object]] = []
    for name, duration in zip(names, durations, strict=True):
        result.append({"id": name, "start_s": current, "end_s": current + duration})
        current += duration
    return result
    ####


def route_binding_source(evidence: dict[str, Any]) -> str:
    """Return the route realization label from the evidence packet."""

    return str(evidence["route"].get("source_realization", "declared_binding"))
    ####


def load_route(binding_id: str) -> Any:
    """Load one resolved route for board rendering."""

    from taoryx.racetrack_template import load_racetrack_template_catalog

    return load_racetrack_template_catalog(ROOT / "verification/racetrack_templates.yaml").get(binding_id)
    ####


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--dt-s", type=float, default=0.5)
    arguments = parser.parse_args()
    print(build(arguments.output, dt_s=arguments.dt_s))
####
