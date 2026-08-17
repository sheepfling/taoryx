"""File-first authoring and CLI-to-Composition vertical witnesses."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from taoryx_parametric_interceptors import (
    CONTROL_ALLOCATION_POLICIES,
    build_interceptor_authoring_report,
    interceptor_authoring_schema_bundle,
    load_interceptor_authoring_profile,
)
from taoryx_parametric_interceptors.__main__ import main

_EXAMPLES = Path("examples/parametric_interceptors")


def test_authoring_schema_and_template_advertise_control_allocation_choices(tmp_path: Path) -> None:
    bundle = interceptor_authoring_schema_bundle("flat")
    flat = bundle["formats"]["flat"]
    policy_schema = flat["properties"]["control_allocation_policy"]["anyOf"][0]
    assert tuple(policy_schema["enum"]) == CONTROL_ALLOCATION_POLICIES

    template_path = tmp_path / "allocation-template.yaml"
    assert main(["init", str(template_path), "--interceptor-id", "allocation-template"]) == 0
    assert "aerodynamic_first | thrust_vector_first | proportional" in template_path.read_text(encoding="utf-8")
    ####


def test_authoring_report_exposes_evidence_assumptions_and_composition_surface() -> None:
    source = _EXAMPLES / "generic_medium_sam.yaml"
    report = build_interceptor_authoring_report(source)

    assert report.schema_id == "taoryx.parametric-interceptor-authoring-report/v1"
    assert report.input_format == "flat"
    assert report.profile_fingerprint
    assert report.resolved_profile.fingerprint
    assert report.readiness.execution_status == "runnable_surrogate"
    assert report.readiness.execution_gate_reason_codes == ()
    assert report.readiness.performance_claim_status == "unqualified_surrogate"
    assert report.readiness.review_required is True
    assert report.origin_counts["simulation_assumption"] > 0
    assert report.origin_counts["archetype_assumption"] > 0
    assert "drag_coefficient" in report.readiness.assumption_parameter_ids
    assert report.composition.provider_id == "taoryx.parametric-interceptors.mission-composition"
    assert set(report.composition.fidelity_ids) == {
        "point_mass_3dof",
        "attitude_response_pseudo_6dof",
    }
    assert "runtime.environment_model_id" in report.composition.configuration_parameter_ids
    assert "runtime.gravity_model_id" in report.composition.configuration_parameter_ids
    assert "runtime.sensor_suite_id" in report.composition.configuration_parameter_ids
    assert report.composition.environment_model_ids == ("taoryx.environment.exponential-atmosphere.standard-earth-v1",)
    assert report.composition.gravity_model_ids == ("taoryx.equations.inverse-square-gravity.standard-earth-v1",)
    assert report.composition.sensor_suite_ids == ("taoryx.sensors.interceptor-navigation.standard-v1",)
    assert set(report.composition.mission_template_ids) == {
        "fixed_waypoint_intercept",
        "constant_velocity_target_intercept",
        "direct_lateral_acceleration_control",
    }
    assert set(report.composition.authority_ids) == {
        "waypoint_guidance",
        "live_waypoint_guidance",
        "target_track_guidance",
        "live_target_track_guidance",
        "direct_lateral_acceleration",
        "live_direct_lateral_acceleration",
    }
    assert "phase.id" in report.composition.output_channel_ids
    assert "sensor.imu.valid" in report.composition.output_channel_ids
    analysis = report.composition.controller_analysis
    assert analysis.fidelity_id == "attitude_response_pseudo_6dof"
    assert analysis.response_analysis_status == "available_local_unsaturated_frozen_command_support"
    assert analysis.response_analysis_contract == "taoryx.parametric-interceptors.pseudo6-response-analysis/v2"
    assert analysis.axes == ("roll", "pitch", "yaw")
    assert analysis.provider_operations == (
        "analyze_pseudo6_response",
        "analyze_pseudo6_response_at_operating_point",
        "compare_pseudo6_responses",
        "compare_pseudo6_responses_at_operating_point",
        "run_pseudo6_attitude_step",
    )
    assert any(command.startswith("taoryx-interceptor analyze-response ") for command in analysis.cli_commands)
    assert any(command.startswith("taoryx-interceptor analyze-operating-response ") for command in analysis.cli_commands)
    assert "do not qualify a physical controller" in analysis.claim_boundary
    assert any(command.startswith("taoryx-interceptor calibrate ") for command in report.next_commands)
    assert any(command.startswith("taoryx-interceptor compare-cases ") for command in report.next_commands)
    assert any(command.startswith("taoryx-interceptor fit ") for command in report.next_commands)
    ####


def test_catalogue_report_keeps_sparse_pac3_gaps_and_diagnostics_visible() -> None:
    source = _EXAMPLES / "pac3_mse_catalogue_seed.yaml"
    profile, detected = load_interceptor_authoring_profile(source)
    report = build_interceptor_authoring_report(source, assumption_case="conservative")

    assert detected == "catalogue"
    assert profile.catalogue_interceptor_id == "int:us-pac3-mse"
    assert report.input_format == "catalogue"
    assert report.resolved_profile.assumption_case.value == "conservative"
    assert report.readiness.evidence_status == "archetype_dominant"
    assert report.readiness.execution_status == "resolver_only"
    assert set(report.readiness.execution_gate_reason_codes) == set(report.readiness.required_diagnostics)
    assert all("--allow-unqualified" in command for command in report.next_commands[1:])
    assert set(report.readiness.evidence_gap_ids) >= {
        "launch_mass_kg",
        "length_m",
        "body_diameter_m",
    }
    assert set(report.readiness.required_diagnostics) == {
        "geometry_missing",
        "mass_properties_missing",
        "motor_pulse_timing_missing",
        "aerodynamic_model_missing",
        "control_authority_missing",
    }
    assert report.readiness.calibration_target_ids == ()
    assert set(report.readiness.missing_calibration_target_ids) == {
        "reported_max_speed_mps",
        "reported_max_range_m",
        "reported_max_altitude_m",
    }
    ####


def test_cli_template_inspect_and_run_use_one_file_to_composition_path(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    profile_path = tmp_path / "quick-sam.yaml"
    report_path = tmp_path / "quick-sam-report.json"
    result_path = tmp_path / "quick-sam-result.json"

    assert main(["init", str(profile_path), "--interceptor-id", "quick-sam"]) == 0
    initialized = json.loads(capsys.readouterr().out)
    assert initialized["status"] == "created"
    assert profile_path.exists()

    assert main(["inspect", str(profile_path), "--output", str(report_path)]) == 0
    assert capsys.readouterr().out == f"wrote {report_path}\n"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["resolved_profile"]["model_id"] == "quick-sam"
    assert report["readiness"]["performance_claim_status"] == "unqualified_surrogate"
    assert report["parameter_usage"]["reference_area_m2"]["usage_class"] == "dynamics_input"
    assert report["parameter_usage"]["length_m"]["usage_class"] == "evidence_only"
    assert sum(report["usage_counts"].values()) == len(report["resolved_profile"]["parameters"]) + 2

    assert (
        main(
            [
                "run",
                str(profile_path),
                "--fidelity",
                "point_mass_3dof",
                "--set",
                "runtime.duration_s=0.2",
                "--set",
                "runtime.time_step_s=0.1",
                "--set",
                "runtime.environment_model_id=taoryx.environment.exponential-atmosphere.standard-earth-v1",
                "--set",
                "runtime.gravity_model_id=taoryx.equations.inverse-square-gravity.standard-earth-v1",
                "--output-mode",
                "selected",
                "--telemetry-group",
                "guidance",
                "--maximum-samples",
                "5",
                "--output",
                str(result_path),
            ]
        )
        == 0
    )
    assert capsys.readouterr().out == f"wrote {result_path}\n"
    response = json.loads(result_path.read_text(encoding="utf-8"))
    vehicle = response["result"]["objects"][0]
    assert response["kind"] == "trajectory"
    assert response["result"]["provider_id"] == "taoryx.parametric-interceptors.mission-composition"
    assert vehicle["model_id"] == "quick-sam"
    assert len(vehicle["samples"]) == 3
    assert "guidance.available" in vehicle["samples"][-1]["values"]
    assert "propulsion.phase" not in vehicle["samples"][-1]["values"]
    ####


def test_cli_template_refuses_accidental_overwrite(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    profile_path = tmp_path / "existing.yaml"
    profile_path.write_text("keep: me\n", encoding="utf-8")

    assert main(["init", str(profile_path)]) == 2
    assert "refusing to overwrite" in capsys.readouterr().err
    assert profile_path.read_text(encoding="utf-8") == "keep: me\n"
    ####


def test_cli_catalogue_template_accepts_ontology_identity(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    profile_path = tmp_path / "ontology-sam.yaml"

    assert (
        main(
            [
                "init",
                str(profile_path),
                "--format",
                "catalogue",
                "--interceptor-id",
                "int:example:new-sam",
            ]
        )
        == 0
    )
    capsys.readouterr()
    profile, detected = load_interceptor_authoring_profile(profile_path)
    assert detected == "catalogue"
    assert profile.catalogue_interceptor_id == "int:example:new-sam"
    assert profile.model_id == "example-new-sam"
    assert {item.parameter_id for item in profile.evidence_gaps} >= {
        "launch_mass_kg",
        "length_m",
        "body_diameter_m",
    }
    ####


def test_cli_requires_explicit_acknowledgement_for_resolver_only_profile(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    source = _EXAMPLES / "pac3_mse_catalogue_seed.yaml"
    result_path = tmp_path / "pac3-unqualified.json"
    common = [
        "run",
        str(source),
        "--set",
        "runtime.duration_s=0.1",
        "--set",
        "runtime.time_step_s=0.1",
        "--maximum-samples",
        "3",
        "--output",
        str(result_path),
    ]

    assert main(common) == 2
    assert "resolver-only" in capsys.readouterr().err
    assert not result_path.exists()

    assert main([*common, "--allow-unqualified"]) == 0
    assert capsys.readouterr().out == f"wrote {result_path}\n"
    response = json.loads(result_path.read_text(encoding="utf-8"))
    assert response["result"]["primary_model_id"] == "pac3-mse"
    ####


def test_cli_selects_constant_velocity_target_mission(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    result_path = tmp_path / "target-track.json"
    assert (
        main(
            [
                "run",
                str(_EXAMPLES / "generic_medium_sam.yaml"),
                "--mission-template",
                "constant_velocity_target_intercept",
                "--set",
                "navigation.target.position.north.command=1000",
                "--set",
                "navigation.target.position.altitude.command=100",
                "--set",
                "navigation.target.velocity.east.command=50",
                "--set",
                "runtime.duration_s=0.1",
                "--set",
                "runtime.time_step_s=0.1",
                "--output-mode",
                "selected",
                "--telemetry-group",
                "guidance",
                "--output",
                str(result_path),
            ]
        )
        == 0
    )
    assert capsys.readouterr().out == f"wrote {result_path}\n"
    payload = json.loads(result_path.read_text(encoding="utf-8"))
    values = payload["result"]["objects"][0]["samples"][0]["values"]
    assert values["guidance.objective.kind"] == "constant_velocity_target"
    assert values["guidance.target.velocity.east.accepted"] == 50.0
    ####


def test_cli_selects_direct_lateral_acceleration_mission(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    result_path = tmp_path / "direct-acceleration.json"
    assert (
        main(
            [
                "run",
                str(_EXAMPLES / "generic_medium_sam.yaml"),
                "--mission-template",
                "direct_lateral_acceleration_control",
                "--set",
                "launch.altitude_m=100",
                "--set",
                "launch.speed_mps=100",
                "--set",
                "launch.flight_path_deg=0",
                "--set",
                "control.lateral_acceleration.local.east.command=10",
                "--set",
                "runtime.duration_s=0.1",
                "--set",
                "runtime.time_step_s=0.1",
                "--output-mode",
                "selected",
                "--telemetry-group",
                "guidance",
                "--output",
                str(result_path),
            ]
        )
        == 0
    )
    assert capsys.readouterr().out == f"wrote {result_path}\n"
    payload = json.loads(result_path.read_text(encoding="utf-8"))
    values = payload["result"]["objects"][0]["samples"][0]["values"]
    assert values["guidance.objective.kind"] == "direct_lateral_acceleration"
    assert values["guidance.law.id"] == "external_lateral_acceleration"
    assert values["guidance.lateral_acceleration.commanded.local.east"] == 10.0
    ####


def test_cli_calibration_binds_runtime_dependencies(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    result_path = tmp_path / "calibration-result.json"
    assert (
        main(
            [
                "calibrate",
                str(_EXAMPLES / "generic_medium_sam.yaml"),
                str(_EXAMPLES / "generic_medium_sam_calibration.yaml"),
                "--output",
                str(result_path),
            ]
        )
        == 0
    )
    assert capsys.readouterr().out == f"wrote {result_path}\n"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    assert result["contract"] == "taoryx.parametric-interceptors.calibration-result/v1"
    assert result["model_id"] == "generic-medium-sam-example"
    assert result["sensor_suite_id"] == "taoryx.sensors.interceptor-navigation.standard-v1"
    assert result["sensor_suite_version"] == "1.1.0"
    assert len(result["sensor_suite_fingerprint"]) == 64
    assert result["navigation_sensor_provider_kind"] == "translation-acceleration"
    assert result["target_track_sensor_provider_kind"] == "relative-state-track"
    ####


def test_cli_compares_selected_assumption_cases_without_mutating_evidence(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    result_path = tmp_path / "assumption-cases.json"
    assert (
        main(
            [
                "compare-cases",
                str(_EXAMPLES / "generic_medium_sam.yaml"),
                str(_EXAMPLES / "generic_medium_sam_calibration.yaml"),
                "--case",
                "conservative",
                "--case",
                "nominal",
                "--output",
                str(result_path),
            ]
        )
        == 0
    )
    assert capsys.readouterr().out == f"wrote {result_path}\n"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    assert result["contract"] == "taoryx.parametric-interceptors.assumption-case-comparison/v1"
    assert result["schema_version"] == 1
    assert len(result["source_profile_fingerprint"]) == 64
    assert {item["assumption_case"] for item in result["results"]} == {
        "conservative",
        "nominal",
    }
    assert len({item["profile_fingerprint"] for item in result["results"]}) == 2
    assert {item["assumption_case"] for item in result["ranking"]} == {
        "conservative",
        "nominal",
    }
    assert result["best_case"] == result["ranking"][0]["assumption_case"]
    assert "not parameter estimation" in result["claim_boundary"]
    assert all(item["evaluation"]["qualification"] == "unqualified" for item in result["results"])
    ####


def test_cli_case_comparison_requires_sparse_profile_acknowledgement(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    result_path = tmp_path / "pac3-case-screen.json"
    command = [
        "compare-cases",
        str(_EXAMPLES / "pac3_mse_catalogue_seed.yaml"),
        str(_EXAMPLES / "generic_medium_sam_calibration.yaml"),
        "--case",
        "nominal",
        "--output",
        str(result_path),
    ]

    assert main(command) == 2
    assert "resolver-only" in capsys.readouterr().err
    assert not result_path.exists()

    assert main([*command, "--allow-unqualified"]) == 0
    assert capsys.readouterr().out == f"wrote {result_path}\n"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    assert result["model_id"] == "pac3-mse"
    assert [item["assumption_case"] for item in result["results"]] == ["nominal"]
    assert result["results"][0]["evaluation"]["qualification"] == "unqualified"
    ####


def test_cli_fit_writes_receipt_and_runnable_fitted_profile(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    campaign_path = tmp_path / "quick-fit.yaml"
    receipt_path = tmp_path / "quick-fit-receipt.json"
    fitted_path = tmp_path / "quick-fit-profile.yaml"
    campaign_path.write_text(
        """campaign_id: quick-cli-fit-v1
max_iterations: 1
acceptance_normalized_error: 10.0
variables:
  - parameter_id: thrust_scale
    lower_bound: 0.8
    upper_bound: 1.2
scenarios:
  - scenario_id: quick-cli-fit-screen
    scenario_basis: Synthetic CLI workflow witness.
    mission:
      duration_s: 0.1
      time_step_s: 0.1
    targets:
      - metric_id: peak_speed_mps
        target: 20.0
        tolerance: 1000.0
        unit: m/s
        source_basis: Synthetic CLI workflow witness.
""",
        encoding="utf-8",
    )

    assert (
        main(
            [
                "fit",
                str(_EXAMPLES / "generic_medium_sam.yaml"),
                str(campaign_path),
                "--fitted-profile-output",
                str(fitted_path),
                "--output",
                str(receipt_path),
            ]
        )
        == 0
    )
    assert capsys.readouterr().out == f"wrote {fitted_path}\nwrote {receipt_path}\n"
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    assert receipt["contract"] == "taoryx.parametric-interceptors.fit-receipt/v1"
    assert receipt["accepted"] is True
    fitted, detected = load_interceptor_authoring_profile(fitted_path)
    assert detected == "flat"
    assert fitted.thrust_scale is not None
    assert fitted.thrust_scale.origin.value == "calibrated"
    ####


def test_cli_analyzes_and_compares_response_laws(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    request = _EXAMPLES / "generic_medium_sam_response_analysis.yaml"
    analysis_path = tmp_path / "response-analysis.json"
    comparison_path = tmp_path / "response-comparison.json"

    assert (
        main(
            [
                "analyze-response",
                str(_EXAMPLES / "generic_medium_sam.yaml"),
                str(request),
                "--output",
                str(analysis_path),
            ]
        )
        == 0
    )
    assert capsys.readouterr().out == f"wrote {analysis_path}\n"
    analysis = json.loads(analysis_path.read_text(encoding="utf-8"))
    assert analysis["contract"] == "taoryx.parametric-interceptors.pseudo6-response-analysis/v2"
    assert analysis["command_support_fraction"] == 0.6
    assert analysis["analysis_axis"] == "roll"

    assert (
        main(
            [
                "compare-response",
                str(_EXAMPLES / "generic_medium_sam.yaml"),
                str(_EXAMPLES / "aim9x_block2_catalogue_seed.yaml"),
                str(request),
                "--output",
                str(comparison_path),
            ]
        )
        == 0
    )
    assert capsys.readouterr().out == f"wrote {comparison_path}\n"
    comparison = json.loads(comparison_path.read_text(encoding="utf-8"))
    assert comparison["contract"] == "taoryx.parametric-interceptors.pseudo6-response-comparison/v2"
    assert comparison["overall_winner_model_id"] is None
    assert comparison["physical_controller_qualification_claim"] is False
    ####


def test_cli_analyzes_response_at_a_typed_force_operating_point(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    output = tmp_path / "operating-point-response.json"

    assert (
        main(
            [
                "analyze-operating-response",
                str(_EXAMPLES / "generic_medium_sam.yaml"),
                str(_EXAMPLES / "generic_medium_sam_operating_point_response.yaml"),
                "--output",
                str(output),
            ]
        )
        == 0
    )
    assert capsys.readouterr().out == f"wrote {output}\n"
    result = json.loads(output.read_text(encoding="utf-8"))
    assert result["contract"] == ("taoryx.parametric-interceptors.pseudo6-operating-point-response-analysis/v1")
    resolution = result["operating_point_resolution"]
    assert resolution["operating_point"]["operating_point_id"] == "boost-turn-local-point"
    assert 0.0 <= resolution["response_support_fraction"] <= 1.0
    assert result["response_analysis"]["command_support_fraction"] == pytest.approx(resolution["response_support_fraction"])
    assert result["response_analysis"]["physical_controller_qualification_claim"] is False
    ####


def test_cli_compares_two_models_at_one_typed_force_operating_point(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    output = tmp_path / "operating-point-comparison.json"

    assert (
        main(
            [
                "compare-operating-response",
                str(_EXAMPLES / "generic_medium_sam.yaml"),
                str(_EXAMPLES / "aim9x_block2_catalogue_seed.yaml"),
                str(_EXAMPLES / "generic_medium_sam_operating_point_response.yaml"),
                "--output",
                str(output),
            ]
        )
        == 0
    )
    assert capsys.readouterr().out == f"wrote {output}\n"
    result = json.loads(output.read_text(encoding="utf-8"))
    assert result["contract"] == ("taoryx.parametric-interceptors.pseudo6-operating-point-response-comparison/v1")
    assert result["operating_point"]["operating_point_id"] == "boost-turn-local-point"
    assert result["baseline"]["response_analysis"]["model_id"] == "generic-medium-sam-example"
    assert result["candidate"]["response_analysis"]["model_id"] == "aim9x-block2"
    assert result["overall_winner_model_id"] is None
    assert result["physical_controller_qualification_claim"] is False
    ####
