"""Contract tests for the staged four-family evidence packet."""

from __future__ import annotations

import hashlib
import json
import zipfile
from pathlib import Path
from types import SimpleNamespace

import pytest

from taoryx.showcase import validate_showcase_run_artifact_boundary
from tools import build_fidelity_ladder_packet as packet

pytestmark = pytest.mark.slow


def test_telemetry_rollup_includes_margin_saturation_and_control_rate_metrics() -> None:
    """The packet scorer retains trajectory-quality telemetry beyond final error."""

    states = [
        SimpleNamespace(
            time=0.0,
            named={
                "aero_table_min_normalized_margin": 0.30,
                "attitude_controller_saturated": 0.0,
                "elevator-deg": 0.0,
            },
        ),
        SimpleNamespace(
            time=0.5,
            named={
                "aero_table_min_normalized_margin": 0.10,
                "attitude_controller_saturated": 1.0,
                "elevator-deg": 5.0,
            },
        ),
        SimpleNamespace(
            time=1.0,
            named={
                "aero_table_min_normalized_margin": 0.20,
                "attitude_controller_saturated": 0.0,
                "elevator-deg": 3.0,
            },
        ),
    ]
    report = SimpleNamespace(results=[SimpleNamespace(states={"1": states})])

    metrics = packet._telemetry_metrics(report)

    assert metrics["table_margin_min_normalized"] == pytest.approx(0.10)
    assert metrics["control_saturation_fraction"] == pytest.approx(1.0 / 3.0)
    assert metrics["control_saturation_average"] == pytest.approx(1.0 / 3.0)
    assert metrics["control_saturation_max_abs"] == pytest.approx(1.0)
    assert metrics["control_derivative_abs_average"] == pytest.approx(7.0)
    assert metrics["control_derivative_abs_max"] == pytest.approx(10.0)
    ####


def test_packet_promotes_summary_into_neutral_evaluation_envelope() -> None:
    """Mission evidence carries claims, controls, resources, events, and gates."""

    summary = {
        "exit_code": 0,
        "results": [{"completed": True}],
        "scenario_contract": {"contract_sha256": "a" * 64},
        "telemetry_metrics": {
            "duration_s": 12.0,
            "final": {
                "bank_command_deg": 10.0,
                "bank_achieved_deg": 8.0,
                "elevator-deg": 1.0,
                "mass_kg": 3.0,
                "battery_soc": 0.8,
            },
            "trajectory_rollup": {"control_channels": ["elevator-deg"]},
        },
        "event_timeline": [{"name": "waypoint-capture", "time": 5.0}],
        "closure_evaluation": {
            "status": "pass",
            "checks": {"translation_p99": {"actual": 1.0e-8, "limit": 1.0e-4, "passed": True}},
        },
    }
    evaluation = packet._trajectory_evaluation(
        summary,
        scenario_id="demo:controller",
        claim_boundary="bounded evidence",
        objective_report={"status": "pass", "objectives": []},
        convergence_report={"status": "pass"},
    )

    assert evaluation["validity"] == "valid"
    assert evaluation["outcome"] == "completed"
    assert evaluation["scenario_contract_sha256"] == "a" * 64
    assert {item["source"] for item in evaluation["requested_controls"]} == {"requested"}
    assert {item["source"] for item in evaluation["achieved_controls"]} == {"achieved"}
    assert {item["source"] for item in evaluation["resources"]} == {"resource"}
    assert evaluation["events"][0]["id"] == "event:0:waypoint-capture"
    assert evaluation["closure"][0]["status"] == "pass"
    assert evaluation["convergence"][0]["status"] == "pass"


def test_packet_evaluation_blocks_missing_closure_evidence() -> None:
    """A completed run without independent closure remains unqualified evidence."""

    evaluation = packet._trajectory_evaluation(
        {"exit_code": 0, "results": [{"completed": True}]},
        scenario_id="demo:missing-closure",
        claim_boundary="diagnostic evidence",
        objective_report={"status": "pass", "objectives": []},
    )
    assert evaluation["validity"] == "not_run"
    assert evaluation["closure"][0]["status"] == "blocked"
    assert evaluation["convergence"][0]["status"] == "blocked"
    assert evaluation["gates"][0]["status"] == "pass"


def test_packet_contains_all_families_and_hashes_inputs(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """The packet remains auditable even when runtime execution is mocked."""

    def fake_run_case(problem: Path, tables: tuple[Path, ...], output: Path, max_steps: int, *, plots: bool = False) -> dict[str, object]:
        output.mkdir(parents=True, exist_ok=True)
        return {
            "exit_code": 0,
            "diagnostics": [],
            "results": [{"completed": True, "state_count": max_steps, "final_time_s": 1.0, "dynamics": "test"}],
            "plots": [] if not plots else ["plots/plot-manifest.json"],
        }

    monkeypatch.setattr(packet, "_run_case", fake_run_case)
    archive = packet.build(tmp_path)

    with zipfile.ZipFile(archive) as handle:
        names = set(handle.namelist())
        manifest = json.loads(handle.read("manifest.json"))
        assert {item["id"] for item in manifest["families"]} == {"b747", "skywalker_x8", "hummingbird", "x15"}
        assert manifest["tiers"] == ["point-mass-3dof", "kinematic-3-plus-3-dof", "rigid-body-6dof"]
        assert len(manifest["run_artifacts"]) == 12
        assert {item["fidelity"] for item in manifest["run_artifacts"]} == {
            "point_mass_3dof",
            "pseudo_6dof",
            "rigid_body_6dof_surface_allocated",
        }
        assert all(item["realization"]["control_realization"] for item in manifest["run_artifacts"])
        for run_artifact in manifest["run_artifacts"]:
            validate_showcase_run_artifact_boundary(run_artifact)
        assert set(manifest["closure_contract"]) == {"b747", "skywalker_x8", "hummingbird", "x15"}
        assert manifest["claim_inputs"] == [
            "verification/claims.md",
            "verification/controller_scenarios.yaml",
                "verification/family_validation_execution.yaml",
                "verification/vehicle_models.yaml",
                "verification/staged_completion_matrix.yaml",
                "verification/fidelity_parity.yaml",
                "verification/long_validation_trajectories.yaml",
                "verification/controller_missions.yaml",
                "verification/acceptance/README.md",
                "verification/acceptance/robustness_matrix_v1.yaml",
        ]
        assert "evidence/claims.md" in names
        assert "evidence/controller_scenarios.yaml" in names
        assert "evidence/family_validation_execution.yaml" in names
        assert "evidence/vehicle_models.yaml" in names
        assert "evidence/long_validation_trajectories.yaml" in names
        assert "evidence/controller_missions.yaml" in names
        assert "evidence/fidelity_parity.yaml" in names
        assert "evidence/source-differential-report.json" in names
        assert "controllers/skywalker-x8-longitudinal-recovery/summary.json" in names
        assert "controllers/hummingbird-waypoint-return-home-land/summary.json" in names
        assert {
            item["id"] for item in manifest["controller_missions"]
        } == {
            "b747-integrated-route-transition",
            "skywalker-x8-powered-rectangle",
            "hummingbird-elevated-rectangle",
            "x15-source-trim-propnav-transition",
        }
        assert any(
            item["id"] == "hummingbird-controller-blocked" and item["status"] == "blocked"
            for item in manifest["controller_cases"]
        )
        for family in packet._load_cases():
            problem_name = Path(str(family["point_mass_problem"])).name
            assert f"families/{family['id']}/inputs/{problem_name}" in names
        assert "families/x15/inputs/mission_3dof.prb" in names
        assert "families/x15/inputs/route_geometry_diagnostic_6dof.prb" in names
        assert "families/b747/evidence/validation_report.json" in names
        assert "families/x15/evidence/validation_report.json" in names
        assert manifest["families"][-1]["diagnostics"]["route-geometry"]["results"][0]["completed"] is True
        assert manifest["families"][-1]["diagnostics"]["short-range-propnav"]["expected_status"] == "blocked"
        assert all(family["source_validation"]["status"] == "PASS" for family in manifest["families"])
        for name, expected in manifest["files"].items():
            digest = hashlib.sha256(handle.read(name)).hexdigest()
            assert digest == expected
    ####


def test_packet_can_be_scoped_to_one_family_and_controller(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A family can be tuned without executing unrelated vehicle models."""

    monkeypatch.setattr(packet, "_run_case", lambda problem, tables, output, max_steps, *, plots=False: {
        "exit_code": 0,
        "diagnostics": [],
        "results": [{"completed": True, "state_count": 3, "final_time_s": 1.0, "dynamics": "test"}],
        "plots": [],
    })
    monkeypatch.setattr(
        "tests.e2e.test_golden_source_differential.source_differential_reports",
        lambda: {"b747": {"sample_count": 1}},
    )

    archive = packet.build(
        tmp_path,
        family_ids=frozenset({"b747"}),
        controller_ids=frozenset({"b747-bounded-route-controller"}),
    )
    with zipfile.ZipFile(archive) as handle:
        manifest = json.loads(handle.read("manifest.json"))
    assert [item["id"] for item in manifest["families"]] == ["b747"]
    assert [item["id"] for item in manifest["controller_cases"]] == ["b747-bounded-route-controller"]
    ####


def test_expectation_evaluation_reports_catalog_contract() -> None:
    """Controller packets expose pass/fail checks for their declared limits."""

    metrics = {
        "duration_s": 12.0,
        "final": {"range_to_target_m": 0.2, "altitude_m": 0.01, "motor_shutdown": 1.0},
        "max": {"altitude_m": 2.0, "speed_m_s": 0.8},
        "max_abs": {
            "aero_alpha_deg": 8.0,
            "aero_sideslip_deg": 1.5,
            "wx": 0.1,
            "wy": 0.1,
            "wz": 0.1,
            "pro_nav_active": 1.0,
            "attitude_controller_saturated": 0.0,
        },
    }
    result = packet._expectation_evaluation(
        {
            "min_duration_s": 10.0,
            "max_alpha_deg": 10.0,
            "max_beta_deg": 5.0,
            "max_body_rate_deg_s": 20.0,
            "max_altitude_m": 2.1,
            "max_speed_m_s": 1.0,
            "final_range_m": 1.0,
            "final_altitude_m": 0.1,
            "require_guidance": 1.0,
            "require_unsaturated": 1.0,
            "require_motor_shutdown": 1.0,
        },
        metrics,
    )
    assert result["status"] == "pass"
    assert result["check_count"] == 11

    failed = packet._expectation_evaluation({"max_speed_m_s": 0.5}, metrics)
    assert failed["status"] == "fail"
    assert failed["checks"][0]["actual"] == 0.8
    ####


def test_packet_helpers_capture_initial_state_and_events() -> None:
    """Evidence summaries retain the state/event boundary needed for review."""

    class State:
        time = 0.25
        named = {"x": 1.0, "mass_kg": 2.0, "qw": 1.0, "qz": 0.0}

    class Result:
        states = {"1": (State(),)}

    class Artifact:
        events = [{"time": 0.25, "name": "segment-transition", "kind": "segment_transition"}]

    class Report:
        results = [Result()]
        artifacts = [Artifact()]

    audit = packet._initial_condition_audit(Report())
    assert audit["status"] == "available"
    assert audit["channels"] == {"x": 1.0, "mass_kg": 2.0, "qw": 1.0, "qz": 0.0}
    assert packet._event_timeline(Report())[0]["name"] == "segment-transition"
    ####


def test_parity_convergence_is_projected_into_family_evidence(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """The packet can reuse the authoritative parity runner without rerunning it."""

    report_path = tmp_path / "artifacts/verification/fidelity_parity_v1/report.json"
    report_path.parent.mkdir(parents=True)
    report_path.write_text(
        json.dumps(
            {
                "families": [
                    {
                        "id": "b747",
                        "parity_gate": "pass_translational_window",
                        "rigid_window_max_difference": {"altitude_m": 0.1},
                        "runs": {"6dof": {"completed": True}},
                        "convergence": {"final_difference": {"6dof": {"altitude_m": 0.01}}},
                    }
                ]
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(packet, "ROOT", tmp_path)
    result = packet._parity_convergence_report("b747", {"source": "fidelity_parity", "window_s": 30})
    assert result["status"] == "pass"
    assert result["evidence_source"] == "evidence/fidelity-parity-report.json"
    assert result["convergence"]["final_difference"]["6dof"]["altitude_m"] == 0.01
    ####


def test_closure_evaluation_applies_family_contract() -> None:
    """Residual values are converted into an explicit packet gate."""

    summary = {
        "independent_closure": {
            "independent_translation": {"p99_normalized_residual": 1.0e-6},
            "independent_rotation": {"p99_normalized_residual": 1.0e-6},
        }
    }
    result = packet._closure_evaluation("x15", summary)
    assert result["status"] == "pass"
    assert result["checks"]["translation_p99"]["limit"] == 1.0e-3

    failed = packet._closure_evaluation(
        "x15",
        {
            "independent_closure": {
                "independent_translation": {"p99_normalized_residual": 1.0},
                "independent_rotation": {"p99_normalized_residual": 1.0},
            }
        },
    )
    assert failed["status"] == "fail"
    ####
