"""Fast vertical Composition proof for the X8 source-table vehicle paths."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, cast

import pytest
from taoryx.source_table_fixed_wing import (
    build_x8_source_surface_physical_lqi_design,
    build_x8_source_table_plant,
)

from taoryx.composition_episode import ActionFrame, LanguageBackedCompositionEpisode, open_vehicle_composition_episode
from taoryx.composition_result_catalog import index_composition_results
from taoryx.language.diagnostics import Diagnostic, Severity
from taoryx.language_backed_execution import execute_powered_fixed_wing_composition
from taoryx.language_backed_racetrack import materialize_powered_fixed_wing_composition
from taoryx.model_authoring import ModelAuthoringError, build_model_authoring_plan, resolve_model_authoring_selection
from taoryx.physical_lqr import validate_nonlinear_wrench_lqi
from taoryx.plugins import PluginCatalog, discover_plugins
from taoryx.runtime.cli import main
from taoryx.runtime.runner import RunReport
from taoryx.vehicle_batch_execution import execute_vehicle_composition_batch
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request, resolve_vehicle_composition_interface_contract
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog
from taoryx.vehicle_execution_preflight import preflight_vehicle_composition

ROOT = Path(__file__).resolve().parents[2]
PROVIDER_ID = "taoryx.x8.mission-composition"
MODEL_ID = "skywalker_x8"
MISSION_ID = "powered_fixed_wing_racetrack_v1"
FIDELITY = "point_mass_3dof"
COMPOSITION = "x8_racetrack_capability_3dof_compose.yaml"
DIRECT_WRENCH_COMPOSITION = "x8_racetrack_direct_wrench_compose.yaml"
SURFACE_CAMPAIGN_ID = "x8-source-surface-local-lqi-v1"
GUIDANCE_CAMPAIGN_ID = "x8-language-backed-guidance-local-lqi-v1"
PSEUDO_GUIDANCE_CAMPAIGN_ID = "x8-language-backed-pseudo-guidance-local-lqi-v1"
LOCAL_SURFACE_MISSION_ID = "x8_local_physical_surface_lqr_screen_v1"
LOCAL_SURFACE_COMPOSITION = "x8_local_physical_surface_lqr_screen_compose.yaml"
LOCAL_SURFACE_LQI_MISSION_ID = "x8_local_physical_surface_lqi_screen_v1"
LOCAL_SURFACE_LQI_COMPOSITION = "x8_local_physical_surface_lqi_screen_compose.yaml"
LOCAL_SURFACE_LQI_LONG_RECOVERY_MISSION_ID = "x8_local_physical_surface_lqi_long_recovery_screen_v1"
LOCAL_SURFACE_LQI_LONG_RECOVERY_COMPOSITION = "x8_local_physical_surface_lqi_long_recovery_screen_compose.yaml"


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover the installed plug-in distributions once for this slice."""

    return discover_plugins(include_external=False, selected=("taoryx.source-table-fixed-wing",))
    ####


def _x8_composition():
    """Compile the documented source-table racetrack through Composition."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _x8_pseudo_composition():
    """Compile the profile-backed pseudo-6DOF racetrack."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_compose.yaml")
    return compile_vehicle_composition(request)
    ####


def _x8_direct_wrench_composition():
    """Compile the source-backed autonomous rigid-body direct-wrench route."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / DIRECT_WRENCH_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _x8_local_surface_composition():
    """Compile the exact source-table physical LQR recovery screen."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / LOCAL_SURFACE_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _x8_local_surface_lqi_composition():
    """Compile the exact source-table physical LQI recovery screen."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / LOCAL_SURFACE_LQI_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _x8_local_surface_lqi_long_recovery_composition():
    """Compile the exact source-powered extended LQI recovery screen."""

    request = load_vehicle_composition_request(
        ROOT / "examples/vehicle_composition" / LOCAL_SURFACE_LQI_LONG_RECOVERY_COMPOSITION
    )
    return compile_vehicle_composition(request)
    ####


def test_x8_advertisement_builds_a_complete_racetrack_authoring_plan(
    plugins: PluginCatalog,
) -> None:
    """The public plan names source data, controls, and usable route segments."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity=FIDELITY,
        realization_id=FIDELITY,
        mission_template_id=MISSION_ID,
    )

    assert plan["status"] == "ready_to_author"
    selection = cast(dict[str, object], plan["selection"])
    assert selection["physical_family"] == "powered_fixed_wing"
    assert selection["fidelity"] == FIDELITY
    data_contract = cast(dict[str, object], plan["data_contract"])
    assert data_contract["properties"]
    assert data_contract["source_refs"]
    channels = cast(list[dict[str, Any]], cast(dict[str, object], plan["controller_automation"])["channels"])
    advertised = {item["id"]: item for item in channels}
    assert set(advertised) == {
        "guidance.override.enabled",
        "guidance.speed.command",
        "guidance.flight_path_angle.command",
        "guidance.heading.command",
        "control.lateral.bridge.command",
        "control.longitudinal.bridge.command",
        "propulsion.command.fraction",
    }
    assert advertised["guidance.speed.command"]["provider_binding"]["tuning_eligible"] is True
    assert advertised["propulsion.command.fraction"]["provider_binding"]["tuning_eligible"] is False
    assert cast(dict[str, object], plan["segment_automation"])["instances"]
    ####


def test_x8_lqi_screen_advertises_its_exact_controller_and_batch_endpoint(
    plugins: PluginCatalog,
) -> None:
    """The common authoring plan keeps the LQI mission separate from the LQR screen."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity="rigid_body_6dof_surface_allocated",
        realization_id="rigid_body_6dof_surface_allocated",
        mission_template_id=LOCAL_SURFACE_LQI_MISSION_ID,
    )

    controller = cast(dict[str, object], plan["controller_automation"])
    execution = cast(dict[str, object], plan["execution_advertisement"])
    assert controller["status"] == "provider_managed_with_campaign"
    assert [item["id"] for item in cast(list[dict[str, object]], controller["campaigns"])] == [SURFACE_CAMPAIGN_ID]
    local_screen = cast(dict[str, object], controller["local_controller_screen"])
    assert local_screen["control_realization"] == "source_table_surface_lqi_allocation"
    assert cast(dict[str, object], local_screen["controller"])["integral_output_names"] == [
        "roll_error_rad",
        "pitch_error_rad",
    ]
    assert [(item["id"], item["lower"], item["upper"]) for item in cast(list[dict[str, object]], local_screen["effector_controls"])] == [
        ("effector.elevon.collective.position", -20.0, 20.0),
        ("effector.elevon.differential.position", -20.0, 20.0),
        ("effector.throttle.position", 0.0, 1.0),
    ]
    assert execution["status"] == "runnable"
    assert execution["endpoint_maturity"] == "batch_and_step_ready"
    assert execution["available_operations"] == ["validate", "batch", "step"]
    ####


def test_x8_composition_executes_the_documented_source_table_racetrack(tmp_path: Path) -> None:
    """The advertised batch path reaches the exact language-backed executor."""

    composition = _x8_composition()
    result = execute_powered_fixed_wing_composition(composition, tmp_path / "x8-racetrack")

    assert result.mission_pass is True
    assert result.numerical_valid is True
    assert result.preflight.status == "translation_ready"
    assert result.truth_evaluation["required_passed"] == 4
    assert result.semantic_action_trace is not None
    assert (result.output_dir / "truth_telemetry.csv").is_file()
    assert (result.output_dir / "semantic_action_trace.json").is_file()
    ####


def test_x8_composition_episode_accepts_the_advertised_native_bridge(tmp_path: Path) -> None:
    """The matching episode remains bounded, checkpointable, and source-owned."""

    episode = open_vehicle_composition_episode(_x8_composition())
    contract = episode.interface_contract
    result = episode.step_frame(
        ActionFrame(
            contract.id,
            contract.fingerprint,
            "native_control_bridge",
            {
                "propulsion.command.fraction": 0.6,
                "control.longitudinal.bridge.command": -2.0,
                "control.lateral.bridge.command": 1.0,
            },
            0.1,
        )
    )

    assert isinstance(episode, LanguageBackedCompositionEpisode)
    assert result.status_frame is not None
    assert result.applied_semantic_action is not None
    checkpoint = episode.save_checkpoint(tmp_path / "x8.checkpoint.json")
    expected = episode.observe().as_dict()
    episode.reset()
    episode.load_checkpoint(checkpoint)
    assert episode.observe().as_dict() == expected
    episode.close()
    ####


def test_x8_composition_episode_executes_the_explicit_kinematic_guidance_authority() -> None:
    """External guidance replaces native route targets without surface promotion."""

    episode = open_vehicle_composition_episode(_x8_composition())
    contract = episode.interface_contract
    response = episode.step_frame(
        ActionFrame(
            contract.id,
            contract.fingerprint,
            "kinematic_guidance",
            {
                "guidance.override.enabled": True,
                "guidance.speed.command": 10.0,
                "guidance.flight_path_angle.command": 5.0,
                "guidance.heading.command": 110.0,
            },
            0.1,
        )
    )

    assert response.applied_semantic_action == {
        "guidance.override.enabled": True,
        "guidance.speed.command": pytest.approx(10.0),
        "guidance.flight_path_angle.command": pytest.approx(5.0),
        "guidance.heading.command": pytest.approx(110.0),
    }
    assert response.status_frame is not None
    assert response.status_frame.values["guidance.override.active"] is True
    assert response.status_frame.values["velocity.speed"] < 17.9
    assert response.status_frame.values["flight.path_angle"] > 0.0
    assert response.status_frame.values["flight.heading"] > 90.0
    episode.close()
    ####


def test_x8_direct_wrench_runtime_failure_still_writes_a_normalized_result(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    """A source failure before control-ledger output remains inspectable."""

    runtime_failure = RunReport(
        problem="x8-direct-wrench-source.prb",
        tables=(),
        cases=0,
        results=(),
        diagnostics=(Diagnostic(severity=Severity.ERROR, code="source-runtime-failure", message="forced source failure"),),
        outputs=(),
    )
    monkeypatch.setattr("taoryx.language_backed_execution.run_files", lambda *_args, **_kwargs: runtime_failure)

    result = execute_powered_fixed_wing_composition(
        _x8_direct_wrench_composition(),
        tmp_path / "x8-direct-wrench-failure",
    )

    assert result.runtime.exit_code == 2
    assert result.numerical_valid is False
    assert result.semantic_action_trace is None
    assert result.control_provenance["status"] == "not_available"
    assert result.control_provenance["runtime_diagnostics"][0]["code"] == "source-runtime-failure"  # type: ignore[index]
    assert result.status_trace is None
    assert (result.output_dir / "execution.json").is_file()
    assert (result.output_dir / "evaluation.json").is_file()
    assert not (result.output_dir / "status_trace.json").exists()
    assert not (result.output_dir / "resource_ledger.json").exists()
    ####


def test_x8_direct_wrench_composition_runs_through_the_public_batch_path(tmp_path: Path) -> None:
    """The exact source direct packet retains its autonomous controller."""

    composition = _x8_direct_wrench_composition()
    preflight = preflight_vehicle_composition(composition)
    materialized = materialize_powered_fixed_wing_composition(composition, tmp_path / "x8-direct-wrench-inputs")
    problem = materialized.problem.read_text(encoding="utf-8")

    assert preflight.status == "translation_ready"
    assert materialized.proposal.route.turn_radius_m == 250.0
    assert materialized.proposal.route.altitude_capture_gain_per_s == 0.1
    assert materialized.proposal.route.position_capture_gain == 0.01
    assert "racetrack-turn-radius-m=250" in problem
    assert "racetrack-altitude-capture-gain-per-s=0.1" in problem
    assert "position-capture-gain=0.01" in problem
    assert "*when time>169.966275239938 stop" in problem
    batch = execute_vehicle_composition_batch(
        composition,
        tmp_path / "x8-direct-wrench-interval",
        max_steps=3,
    )
    payload = batch.as_dict()
    action_trace = cast(dict[str, object], payload["semantic_action_trace"])

    assert batch.binding.factory_id == "language_backed_powered_fixed_wing.v1"
    assert payload["preflight"]["status"] == "translation_ready"
    assert payload["runtime"]["exit_code"] == 1  # Focused interval, not a complete racetrack.
    assert payload["runtime"]["execution_limit_reason"] == "max_steps"
    assert action_trace["requested_action_channel_count"] == 0
    assert action_trace["sample_count"] == 3
    evaluation = json.loads((batch.output_dir / "evaluation.json").read_text(encoding="utf-8"))
    assert evaluation["validity"] == "valid"
    assert evaluation["outcome"] == "time_limited"
    assert (batch.output_dir / "truth_telemetry.csv").is_file()
    assert (batch.output_dir / "status_trace.json").is_file()
    assert (batch.output_dir / "resource_ledger.json").is_file()
    ####


def test_x8_public_batch_cli_records_a_capped_prefix_as_max_steps(tmp_path: Path) -> None:
    """The CLI-owned manifest preserves the reason for a bounded source smoke."""

    composition = _x8_direct_wrench_composition()
    composition_path = tmp_path / "x8-direct-wrench.composition.json"
    composition.write_json(composition_path)
    output_dir = tmp_path / "x8-direct-wrench-cli"

    assert main(
        [
            "vehicle",
            "run",
            str(composition_path),
            "--output-dir",
            str(output_dir),
            "--max-steps",
            "3",
        ]
    ) == 1

    execution = json.loads((output_dir / "execution.json").read_text(encoding="utf-8"))
    evaluation = json.loads((output_dir / "evaluation.json").read_text(encoding="utf-8"))
    run_manifest = json.loads((output_dir / "run-manifest.json").read_text(encoding="utf-8"))
    assert execution["schema"] == "taoryx.vehicle-execution-packet/v1alpha1"
    assert execution["provider_execution_schema"] == "taoryx.language-backed-composition-execution/v1alpha1"
    assert execution["execution_limit_reason"] == "max_steps"
    assert execution["outcome"] == {
        "scope": "mission",
        "disposition": "failed",
        "provider_status": "mission_failed",
    }
    assert evaluation["outcome"] == "time_limited"
    assert run_manifest["status"] == "incomplete"
    assert run_manifest["termination"]["reason"] == "max_steps"
    assert run_manifest["integration"]["execution_packet"]["packet_identity_sha256"] == execution["packet_identity_sha256"]
    catalog = index_composition_results(output_dir)
    assert catalog["status"] == "pass"
    assert catalog["records"][0]["run_manifest_evidence"]["status"] == "verified"
    ####


def test_x8_pseudo_episode_exercises_profile_backed_bank_guidance() -> None:
    """The pseudo tier exposes measured sidecar attitude without actuator claims."""

    episode = open_vehicle_composition_episode(_x8_pseudo_composition())
    contract = episode.interface_contract
    response = episode.step_frame(
        ActionFrame(
            contract.id,
            contract.fingerprint,
            "kinematic_guidance",
            {
                "guidance.override.enabled": True,
                "guidance.speed.command": 10.0,
                "guidance.flight_path_angle.command": 3.0,
                "guidance.heading.command": 110.0,
                "guidance.bank.command": 12.0,
            },
            0.1,
        )
    )

    assert response.status_frame is not None
    assert response.status_frame.values["guidance.override.active"] is True
    attitude = cast(list[float], response.status_frame.values["attitude.euler"])
    body_rate = cast(list[float], response.status_frame.values["body_rate"])
    assert attitude[0] > 0.0
    assert attitude[1] > 7.8095001441
    assert body_rate[0] > 0.0
    episode.close()
    ####


def test_x8_language_guidance_lqi_campaign_runs_through_the_common_host(
    plugins: PluginCatalog,
) -> None:
    """The campaign is registered on the exact point-mass guidance realization."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity=FIDELITY,
        realization_id=FIDELITY,
        mission_template_id=MISSION_ID,
    )

    controller = cast(dict[str, object], plan["controller_automation"])
    assert controller["status"] == "campaign_registered"
    assert [item["id"] for item in cast(list[dict[str, object]], controller["campaigns"])] == [GUIDANCE_CAMPAIGN_ID]


def test_x8_pseudo_guidance_lqi_campaign_is_selectable_through_the_common_host(
    plugins: PluginCatalog,
) -> None:
    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity="pseudo_6dof",
        realization_id="pseudo_6dof",
        mission_template_id=MISSION_ID,
    )

    controller = cast(dict[str, object], plan["controller_automation"])
    assert controller["status"] == "campaign_registered"
    assert [item["id"] for item in cast(list[dict[str, object]], controller["campaigns"])] == [PSEUDO_GUIDANCE_CAMPAIGN_ID]
    ####


def test_x8_source_surface_lqi_campaign_runs_through_the_common_host(
    plugins: PluginCatalog,
    tmp_path: Path,
) -> None:
    """A registered local candidate does not need a runnable full racetrack."""

    providers = plugins.build_mission_composition_provider_registry()
    # Fidelity applicability fails before realization applicability now that
    # model authoring validates the public mission contract first.  Both
    # checks are fail-closed; this probe only requires that a full racetrack
    # cannot be selected for the local surface tier.
    with pytest.raises(ModelAuthoringError, match="mission-fidelity-mismatch|realization-mission-mismatch"):
        resolve_model_authoring_selection(
            providers,
            PROVIDER_ID,
            MODEL_ID,
            fidelity="rigid_body_6dof_surface_allocated",
            realization_id="rigid_body_6dof_surface_allocated",
            mission_template_id=MISSION_ID,
        )

    output = tmp_path / "x8-source-surface-lqi.json"
    assert (
        main(
            [
                "model",
                "tune",
                PROVIDER_ID,
                MODEL_ID,
                "--campaign",
                SURFACE_CAMPAIGN_ID,
                "--no-cache",
                "--output",
                str(output),
            ]
        )
        == 0
    )
    payload = json.loads(output.read_text(encoding="utf-8"))
    selection = cast(dict[str, object], payload["selection"])
    sources = cast(dict[str, str], selection["sources"])
    report = cast(dict[str, object], payload["report"])
    nodes = cast(list[dict[str, object]], report["nodes"])
    lqr = cast(dict[str, object], nodes[0]["lqr"])

    assert report["status"] == "candidate_ready"
    assert sources.get("realization_availability") in {None, "blocked_local_design_allowed_for_registered_campaign"}
    assert lqr["method"] == "lqi"
    assert lqr["integral_output_names"] == ["roll_error_rad", "pitch_error_rad"]
    candidates = cast(list[dict[str, object]], lqr["candidates"])
    assert any(
        cast(dict[str, object], candidate["weights"])["integral_q_diagonal"] == [0.15, 0.15]
        for candidate in candidates
    )
    ####


def test_x8_physical_lqi_design_passes_the_bounded_source_local_baseline() -> None:
    """The public local candidate follows the actual elevon allocation path.

    This only checks the declared roll/pitch local recovery.  The X8's
    differential-elevon coordinate has coupled lateral influence, but no
    independent yaw-moment channel. The declared external pitch-moment seam
    is used only by the separate fixed-offset evidence screen, not tuning.
    """

    plant = build_x8_source_table_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    assert trim.success
    design = build_x8_source_surface_physical_lqi_design()
    initial_state = dict(trim.state)
    initial_state.update(
        {
            "roll_error_rad": math.radians(5.0),
            "pitch_error_rad": math.radians(-3.0),
            "p_rad_s": math.radians(4.0),
            "q_rad_s": math.radians(-3.0),
        }
    )
    validation = validate_nonlinear_wrench_lqi(
        plant,
        trim,
        design,
        initial_state=initial_state,
        duration_s=8.0,
        dt_s=0.01,
        integral_lower={name: -0.5 for name in design.result.output_names},
        integral_upper={name: 0.5 for name in design.result.output_names},
    )

    final_error_fraction = validation.final_normalized_feedback_error_norm / validation.initial_normalized_feedback_error_norm
    assert design.result.hurwitz
    assert design.result.output_names == ("roll_error_rad", "pitch_error_rad")
    assert design.integral_q_diagonal == (0.15, 0.15)
    assert plant.external_moment_environment_keys == {"moment_y_nm": "external_pitch_moment_bias_nm"}
    assert validation.integrators_exercised
    assert final_error_fraction <= 0.05
    assert validation.final_controlled_actual_residual <= 0.005
    assert validation.saturation_fraction <= 0.05
    assert validation.maximum_continuous_saturation_duration_s <= 0.25
    assert set(validation.allocation_statuses) <= {"feasible", "feasible_near_limit", "partially_achievable"}
    ####


def test_x8_local_source_table_surface_lqr_screen_runs_through_public_composition(
    tmp_path: Path,
) -> None:
    """The source-table physical tier is a bounded local screen, not a racetrack claim."""

    composition = _x8_local_surface_composition()
    batch = execute_vehicle_composition_batch(composition, tmp_path / "x8-local-surface-lqr")
    payload = batch.as_dict()
    control_screen = cast(dict[str, object], payload["control_screen"])
    runtime = cast(dict[str, object], payload["runtime"])
    preflight = cast(dict[str, object], payload["preflight"])
    derived_mission = cast(dict[str, object], preflight["derived_mission"])
    capability = cast(dict[str, object], derived_mission["capability"])
    lqi_candidate = cast(dict[str, object], capability["offset_free_tuning_candidate"])

    assert batch.passed is True
    assert payload["screen_pass"] is True
    assert control_screen["mission_pass"] is True
    assert runtime["controller_method"] == "lqr"
    assert runtime["physical_effector_allocation"] is True
    assert runtime["controlled_wrench_axes"] == ["moment_x_nm", "moment_y_nm"]
    assert runtime["unallocated_wrench_axes"] == ["moment_z_nm"]
    assert lqi_candidate["campaign_id"] == SURFACE_CAMPAIGN_ID
    assert lqi_candidate["method"] == "lqi"
    assert lqi_candidate["controller_id"] == "skywalker-x8-source-trim-roll-pitch-wrench-lqi-v1"
    assert lqi_candidate["physical_allocation_baseline"] == "focused_bounded_source_local_recovery_passed"
    assert lqi_candidate["persistent_disturbance_status"] == "executed_by_paired_standard_lqi_screen"
    assert lqi_candidate["physical_screen_status"] == "not_executed_by_this_lqr_screen"
    lqi_execution = cast(dict[str, object], lqi_candidate["physical_screen_execution"])
    assert lqi_execution == {
        "status": "executed_by_paired_lqi_screen",
        "mission_id": "x8_local_physical_surface_lqi_screen_v1",
        "capability_adapter_id": "taoryx.x8_local_physical_surface_lqi_screen.capability.v1",
        "operations": ["validate", "batch"],
        "control_realization": "source_table_coordinate_physical_wrench_lqi_allocation",
    }
    status_trace = json.loads((batch.output_dir / "status_trace.json").read_text(encoding="utf-8"))
    assert "resources.mass.total" in status_trace["channels"]
    assert all(sample["values"]["resources.mass.total"] == pytest.approx(runtime["mass_kg"]) for sample in status_trace["samples"])
    assert (batch.output_dir / "status_trace.json").is_file()
    assert (batch.output_dir / "semantic_action_trace.json").is_file()
    assert (batch.output_dir / "resource_ledger.json").is_file()
    ####


@pytest.mark.slow
def test_x8_local_source_table_surface_lqi_screen_runs_through_public_composition(
    tmp_path: Path,
) -> None:
    """The physical LQI profile emits its bounded pitch-offset evidence packet."""

    composition = _x8_local_surface_lqi_composition()
    batch = execute_vehicle_composition_batch(composition, tmp_path / "x8-local-surface-lqi")
    payload = batch.as_dict()
    control_screen = cast(dict[str, object], payload["control_screen"])
    runtime = cast(dict[str, object], payload["runtime"])
    preflight = cast(dict[str, object], payload["preflight"])
    derived_mission = cast(dict[str, object], preflight["derived_mission"])
    capability = cast(dict[str, object], derived_mission["capability"])
    lqi_candidate = cast(dict[str, object], capability["offset_free_tuning_candidate"])

    assert batch.passed is True
    assert payload["screen_pass"] is True
    assert control_screen["mission_pass"] is True
    assert runtime["controller_method"] == "lqi"
    assert runtime["control_realization"] == "source_table_coordinate_physical_wrench_lqi_allocation"
    assert runtime["physical_effector_allocation"] is True
    assert runtime["integral_q_diagonal"] == [0.15, 0.15]
    offset_runtime = cast(dict[str, object], runtime["matched_pitch_offset_screen"])
    assert offset_runtime["status"] == "applied"
    assert offset_runtime["pass"] is True
    assert lqi_candidate["physical_screen_status"] == "executed_by_this_lqi_screen"
    assert cast(dict[str, object], lqi_candidate["physical_screen_execution"])["status"] == "executed_by_this_lqi_screen"
    assert lqi_candidate["persistent_disturbance_status"] == "executed_by_this_screen"
    automation = cast(dict[str, object], lqi_candidate["controller_automation"])
    assert cast(dict[str, object], automation["integral_priority_grid"])["multipliers"] == [1.0]
    assert cast(dict[str, object], automation["physical_wrench_profile"])["integral_q_diagonal"] == [0.15, 0.15]
    assert (batch.output_dir / "nonlinear_validation.json").is_file()
    assert (batch.output_dir / "robustness_report.json").is_file()
    assert (batch.output_dir / "status_trace.json").is_file()
    nonlinear_validation = cast(dict[str, object], json.loads((batch.output_dir / "nonlinear_validation.json").read_text()))
    robustness = cast(dict[str, object], json.loads((batch.output_dir / "robustness_report.json").read_text()))
    metrics = cast(dict[str, object], nonlinear_validation["metrics"])
    assert nonlinear_validation["schema"] == "taoryx.physical-lqi-validation/v1alpha1"
    assert metrics["integrators_exercised"] is True
    assert robustness["schema"] == "taoryx.endpoint-robustness-screen/v1alpha1"
    assert robustness["release_evidence_schema"] == "taoryx.claim-bound-release-evidence/v1alpha1"
    assert robustness["release_evidence_kind"] == "robustness"
    assert cast(dict[str, object], robustness["release_evidence_subject"])["composition_identity_sha256"] == composition.identity_sha256
    assert robustness["id"] == "x8-local-lqi-matched-pitch-wrench-offset"
    assert robustness["kind"] == "constant_offset"
    assert robustness["pass"] is True
    cases = cast(list[dict[str, object]], robustness["cases"])
    assert [case["id"] for case in cases] == ["nominal", "positive-pitch-offset", "negative-pitch-offset"]
    assert [cast(dict[str, float], case["parameters"])["pitch_wrench_bias_fraction"] for case in cases] == [0.0, 0.05, -0.05]
    assert all(case["status"] == "pass" for case in cases)
    assert all(cast(dict[str, float], case["metrics"])["final_feedback_error_fraction"] <= 0.05 for case in cases)
    assert all(cast(dict[str, float], case["metrics"])["saturation_fraction"] <= 0.05 for case in cases)
    ####


@pytest.mark.slow
def test_x8_physical_lqi_screen_applies_the_exact_common_tuning_candidate(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """The selected LQI candidate uses the screen's state-to-wrench contract."""

    registration = plugins.build_controller_tuning_campaign_registry().registration(SURFACE_CAMPAIGN_ID)
    contexts = registration.application_contexts(registration.run_cached(tmp_path / "tuning-cache"))

    assert len(contexts) == 1
    batch = execute_vehicle_composition_batch(
        _x8_local_surface_lqi_composition(),
        tmp_path / "x8-local-surface-lqi-tuned",
        tuning_context=contexts[0],
    )
    runtime = cast(dict[str, object], batch.as_dict()["runtime"])
    binding = cast(dict[str, object], runtime["tuning_binding"])

    assert batch.passed is True
    assert binding["campaign_id"] == SURFACE_CAMPAIGN_ID
    assert binding["candidate_profile_id"] == contexts[0].candidate_profile_id
    assert binding["applied_gain_fingerprint_sha256"] == contexts[0].resolved_gain_fingerprint_sha256
    ####


def test_x8_source_powered_trim_and_extended_lqi_recovery_runs_through_public_composition(
    tmp_path: Path,
) -> None:
    """The promoted screen proves a fresh throttle-bearing trim and a longer local recovery."""

    composition = _x8_local_surface_lqi_long_recovery_composition()
    batch = execute_vehicle_composition_batch(composition, tmp_path / "x8-local-surface-lqi-long-recovery")
    payload = batch.as_dict()
    control_screen = cast(dict[str, object], payload["control_screen"])
    runtime = cast(dict[str, object], payload["runtime"])
    preflight = cast(dict[str, object], payload["preflight"])
    derived_mission = cast(dict[str, object], preflight["derived_mission"])
    capability = cast(dict[str, object], derived_mission["capability"])
    capability_trim = cast(dict[str, object], capability["source_powered_trim"])
    runtime_trim = cast(dict[str, object], runtime["source_powered_trim"])
    extended = cast(dict[str, object], runtime["extended_recovery"])

    assert batch.passed is True
    assert payload["screen_pass"] is True
    assert control_screen["mission_pass"] is True
    assert runtime["duration_s"] == pytest.approx(20.0)
    assert runtime["dt_s"] == pytest.approx(0.02)
    assert runtime_trim["status"] == "freshly_solved_for_this_execution"
    assert runtime_trim["success"] is True
    assert cast(dict[str, float], runtime_trim["controls"])["throttle"] > 0.0
    assert capability_trim["status"] == "freshly_solved_for_this_estimate"
    assert capability_trim["success"] is True
    assert extended == {"status": "executed_by_this_screen", "duration_s": 20.0, "fixed_cadence_s": 0.02}
    assert {
        item["id"]: item["passed"]
        for item in cast(list[dict[str, object]], preflight["checks"])
    }["x8.extended_physical_lqi_recovery_duration_s"] is True
    assert {item["id"] for item in cast(list[dict[str, object]], control_screen["results"])} >= {
        "source_powered_trim",
        "extended_recovery_duration",
    }
    assert (batch.output_dir / "truth_telemetry.csv").is_file()
    assert (batch.output_dir / "semantic_action_trace.json").is_file()
    ####


def test_x8_local_surface_preflight_advertises_coupled_lateral_authority_without_yaw_promotion() -> None:
    """The exact lateral candidate is discoverable, but still not a route claim."""

    preflight = preflight_vehicle_composition(_x8_local_surface_composition())
    capability = cast(dict[str, object], cast(dict[str, object], preflight.derived_mission)["capability"])
    lateral = cast(dict[str, object], capability["coupled_lateral_authority"])
    linear_preflight = cast(dict[str, object], lateral["linear_preflight"])

    assert preflight.status == "translation_ready"
    assert lateral["status"] == "passed"
    assert lateral["control_coordinate"] == "differential-elevon-deg"
    assert lateral["independent_yaw_wrench_axis"] is False
    assert lateral["nonlinear_recovery_status"] == "not_qualified"
    assert linear_preflight["metrics"]["controllability_rank"] == pytest.approx(5.0)
    assert linear_preflight["metrics"]["controllability_condition_number"] < 1.0e3
    assert {
        check.id: check.passed
        for check in preflight.checks
    }["x8.coupled_lateral_authority_diagnosed"] is True
    ####


def test_x8_catalog_advertises_the_exercised_batch_and_episode_endpoints() -> None:
    """The source-table racetrack has exact batch and episode bindings."""

    kit = load_resolved_vehicle_composition_catalog().vehicle(MODEL_ID).authoring_kit_dict(MISSION_ID, FIDELITY)
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "language_backed_powered_fixed_wing.v1"),
        ("episode", "language_backed_interactive.v1"),
    }
    ####


def test_x8_direct_wrench_catalog_advertises_only_the_evidenced_batch_endpoint() -> None:
    """The autonomous source lane is batch-runnable, not externally interactive."""

    kit = (
        load_resolved_vehicle_composition_catalog()
        .vehicle(MODEL_ID)
        .authoring_kit_dict(
            MISSION_ID,
            "rigid_body_6dof_direct_wrench",
        )
    )
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])
    interface = resolve_vehicle_composition_interface_contract(_x8_direct_wrench_composition())

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "language_backed_powered_fixed_wing.v1"),
    }
    assert {item.id for item in interface.action_channels} == {"wrench.force.command", "wrench.moment.command"}
    assert all(item.availability == "planned" for item in interface.action_channels)
    ####


def test_x8_local_surface_catalog_advertises_actual_source_coordinate_effectors() -> None:
    """Actual values are advertised as source coordinates, never as servo wiring."""

    kit = (
        load_resolved_vehicle_composition_catalog()
        .vehicle(MODEL_ID)
        .authoring_kit_dict(
            LOCAL_SURFACE_MISSION_ID,
            "rigid_body_6dof_surface_allocated",
        )
    )
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])
    interface = resolve_vehicle_composition_interface_contract(_x8_local_surface_composition())

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "x8_local_physical_surface_lqr_screen.v1"),
    }
    assert {item.id for item in interface.effector_channels if item.availability == "available_in_batch"} == {
        "effector.throttle.position",
        "effector.elevon.collective.position",
        "effector.elevon.differential.position",
    }
    ####


def test_x8_local_surface_lqi_catalog_advertises_the_exact_offset_free_batch_endpoint() -> None:
    """LQI uses the same physical coordinates but never aliases the LQR execution binding."""

    kit = (
        load_resolved_vehicle_composition_catalog()
        .vehicle(MODEL_ID)
        .authoring_kit_dict(
            LOCAL_SURFACE_LQI_MISSION_ID,
            "rigid_body_6dof_surface_allocated",
        )
    )
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])
    interface = resolve_vehicle_composition_interface_contract(_x8_local_surface_lqi_composition())

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "x8_local_physical_surface_lqi_screen.v1"),
    }
    assert {item.id for item in interface.effector_channels if item.availability == "available_in_batch"} == {
        "effector.throttle.position",
        "effector.elevon.collective.position",
        "effector.elevon.differential.position",
    }
    ####


def test_x8_extended_lqi_catalog_advertises_the_exact_source_powered_batch_endpoint() -> None:
    """The extended screen is a separate public endpoint with actual source-coordinate effectors."""

    kit = (
        load_resolved_vehicle_composition_catalog()
        .vehicle(MODEL_ID)
        .authoring_kit_dict(
            LOCAL_SURFACE_LQI_LONG_RECOVERY_MISSION_ID,
            "rigid_body_6dof_surface_allocated",
        )
    )
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])
    interface = resolve_vehicle_composition_interface_contract(_x8_local_surface_lqi_long_recovery_composition())

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "x8_local_physical_surface_lqi_long_recovery_screen.v1"),
    }
    assert {item.id for item in interface.effector_channels if item.availability == "available_in_batch"} == {
        "effector.throttle.position",
        "effector.elevon.collective.position",
        "effector.elevon.differential.position",
    }
    ####
