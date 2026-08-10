"""Fast vertical Composition proof for the B747 source-table vehicle paths."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, cast

import pytest
from taoryx.source_table_fixed_wing import (
    build_b747_condition3_source_surface_physical_lqi_design,
    build_b747_condition3_source_table_plant,
)

from taoryx.composition_episode import ActionFrame, LanguageBackedCompositionEpisode, open_vehicle_composition_episode
from taoryx.language_backed_execution import execute_powered_fixed_wing_composition
from taoryx.language_backed_racetrack import materialize_powered_fixed_wing_composition
from taoryx.model_authoring import ModelAuthoringError, build_model_authoring_plan, resolve_model_authoring_selection
from taoryx.physical_lqr import validate_nonlinear_wrench_lqi
from taoryx.plugins import PluginCatalog, discover_plugins
from taoryx.runtime.cli import main
from taoryx.vehicle_batch_execution import execute_vehicle_composition_batch
from taoryx.vehicle_composition import (
    compile_vehicle_composition,
    load_vehicle_composition_request,
    resolve_vehicle_composition_interface_contract,
)
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog
from taoryx.vehicle_execution_preflight import preflight_vehicle_composition

ROOT = Path(__file__).resolve().parents[2]
PROVIDER_ID = "taoryx.registry.mission-composition"
MODEL_ID = "b747"
MISSION_ID = "powered_fixed_wing_racetrack_v1"
FIDELITY = "point_mass_3dof"
COMPOSITION = "b747_racetrack_capability_3dof_compose.yaml"
DIRECT_WRENCH_COMPOSITION = "b747_racetrack_direct_wrench_compose.yaml"
SURFACE_CAMPAIGN_ID = "b747-source-surface-local-lqi-v1"
GUIDANCE_CAMPAIGN_ID = "b747-language-backed-guidance-local-lqi-v1"
PSEUDO_GUIDANCE_CAMPAIGN_ID = "b747-language-backed-pseudo-guidance-local-lqi-v1"
LOCAL_SURFACE_MISSION_ID = "b747_condition3_local_physical_surface_lqr_screen_v1"
LOCAL_SURFACE_COMPOSITION = "b747_condition3_local_physical_surface_lqr_screen_compose.yaml"
LOCAL_SURFACE_LQI_MISSION_ID = "b747_condition3_local_physical_surface_lqi_screen_v1"
LOCAL_SURFACE_LQI_COMPOSITION = "b747_condition3_local_physical_surface_lqi_screen_compose.yaml"


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover the installed plug-in distributions once for this slice."""

    return discover_plugins(include_external=False)
    ####


def _b747_composition():
    """Compile the documented source-table transport racetrack."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _b747_pseudo_composition():
    """Compile the profile-backed pseudo-6DOF transport racetrack."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/b747_racetrack_capability_pseudo6dof_compose.yaml")
    return compile_vehicle_composition(request)
    ####


def _b747_direct_wrench_composition():
    """Compile the source-evidenced B747 direct-wrench racetrack."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / DIRECT_WRENCH_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _b747_local_surface_composition():
    """Compile the exact NASA condition-3 physical LQR recovery screen."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / LOCAL_SURFACE_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _b747_local_surface_lqi_composition():
    """Compile the exact B747 condition-3 physical LQI recovery screen."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / LOCAL_SURFACE_LQI_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def test_b747_advertisement_builds_a_complete_racetrack_authoring_plan(
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
    assert {item["id"] for item in channels} == {
        "control.longitudinal.bridge.command",
        "guidance.override.enabled",
        "guidance.speed.command",
        "guidance.flight_path_angle.command",
        "guidance.heading.command",
        "propulsion.command.fraction",
    }
    assert cast(dict[str, object], plan["segment_automation"])["instances"]
    controller = cast(dict[str, object], plan["controller_automation"])
    assert controller["status"] == "campaign_registered"
    assert [item["id"] for item in cast(list[dict[str, object]], controller["campaigns"])] == [GUIDANCE_CAMPAIGN_ID]
    ####


def test_b747_lqi_screen_advertises_its_exact_controller_and_batch_endpoint(
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
        "yaw_error_rad",
    ]
    assert [(item["id"], item["lower"], item["upper"]) for item in cast(list[dict[str, object]], local_screen["effector_controls"])] == [
        ("effector.elevator.position", -10.0, 10.0),
        ("effector.aileron.position", -10.0, 10.0),
        ("effector.rudder.position", -15.0, 15.0),
        ("effector.throttle.position", 0.0, 1.0),
    ]
    assert execution["status"] == "runnable"
    assert execution["endpoint_maturity"] == "batch_ready"
    assert execution["available_operations"] == ["validate", "batch"]
    ####


def test_b747_composition_executes_a_source_table_batch_interval(tmp_path: Path) -> None:
    """The focused proof reaches the exact batch executor without a long route run.

    Full B747 route-objective coverage remains in the slower catalogue/runtime
    tests. This vertical slice holds three committed source-runtime intervals
    so it can quickly prove composition, lowering, telemetry, and actions.
    """

    composition = _b747_composition()
    result = execute_powered_fixed_wing_composition(composition, tmp_path / "b747-racetrack-interval", max_steps=3)

    assert result.preflight.status == "translation_ready"
    assert result.semantic_action_trace is not None
    assert len(cast(list[dict[str, object]], result.semantic_action_trace["samples"])) == 3
    assert result.control_provenance["detail"] == "intervals"
    assert (result.output_dir / "truth_telemetry.csv").is_file()
    assert (result.output_dir / "semantic_action_trace.json").is_file()
    ####


def test_b747_direct_wrench_composition_runs_through_the_public_batch_path(tmp_path: Path) -> None:
    """The source direct packet retains its controller through public batch execution."""

    composition = _b747_direct_wrench_composition()
    preflight = preflight_vehicle_composition(composition)
    materialized = materialize_powered_fixed_wing_composition(composition, tmp_path / "b747-direct-wrench-inputs")
    problem = materialized.problem.read_text(encoding="utf-8")

    assert preflight.status == "translation_ready"
    assert materialized.proposal.route.turn_radius_m == 8300.0
    assert materialized.proposal.route.altitude_capture_gain_per_s == 1.0
    assert materialized.proposal.route.position_capture_gain == 0.01
    assert "racetrack-turn-radius-m=8300" in problem
    assert "racetrack-altitude-capture-gain-per-s=1" in problem
    assert "position-capture-gain=0.01" in problem
    assert "*when time>664.578026467912 stop" in problem
    batch = execute_vehicle_composition_batch(
        composition,
        tmp_path / "b747-direct-wrench-interval",
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


def test_b747_composition_episode_accepts_the_advertised_native_bridge(tmp_path: Path) -> None:
    """The matching episode remains bounded, checkpointable, and source-owned."""

    episode = open_vehicle_composition_episode(_b747_composition())
    contract = episode.interface_contract
    result = episode.step_frame(
        ActionFrame(
            contract.id,
            contract.fingerprint,
            "native_control_bridge",
            {
                "propulsion.command.fraction": 0.6,
                "control.longitudinal.bridge.command": -2.0,
            },
            0.1,
        )
    )

    assert isinstance(episode, LanguageBackedCompositionEpisode)
    assert result.status_frame is not None
    assert result.applied_semantic_action is not None
    checkpoint = episode.save_checkpoint(tmp_path / "b747.checkpoint.json")
    expected = episode.observe().as_dict()
    episode.reset()
    episode.load_checkpoint(checkpoint)
    assert episode.observe().as_dict() == expected
    episode.close()
    ####


def test_b747_composition_episode_executes_the_explicit_kinematic_guidance_authority() -> None:
    """External guidance changes native lower-tier states without surface promotion."""

    episode = open_vehicle_composition_episode(_b747_composition())
    contract = episode.interface_contract
    response = episode.step_frame(
        ActionFrame(
            contract.id,
            contract.fingerprint,
            "kinematic_guidance",
            {
                "guidance.override.enabled": True,
                "guidance.speed.command": 120.0,
                "guidance.flight_path_angle.command": 3.0,
                "guidance.heading.command": 110.0,
            },
            0.1,
        )
    )

    assert response.applied_semantic_action == {
        "guidance.override.enabled": True,
        "guidance.speed.command": pytest.approx(120.0),
        "guidance.flight_path_angle.command": pytest.approx(3.0),
        "guidance.heading.command": pytest.approx(110.0),
    }
    assert response.status_frame is not None
    assert response.status_frame.values["guidance.override.active"] is True
    assert response.status_frame.values["velocity.speed"] < 153.0
    assert response.status_frame.values["flight.path_angle"] > 0.0
    assert response.status_frame.values["flight.heading"] > 90.0
    episode.close()
    ####


def test_b747_pseudo_episode_exercises_profile_backed_bank_guidance() -> None:
    """The pseudo transport status contains the driven, bounded sidecar response."""

    episode = open_vehicle_composition_episode(_b747_pseudo_composition())
    contract = episode.interface_contract
    response = episode.step_frame(
        ActionFrame(
            contract.id,
            contract.fingerprint,
            "kinematic_guidance",
            {
                "guidance.override.enabled": True,
                "guidance.speed.command": 120.0,
                "guidance.flight_path_angle.command": 3.0,
                "guidance.heading.command": 110.0,
                "guidance.bank.command": 8.0,
            },
            0.1,
        )
    )

    assert response.status_frame is not None
    assert response.status_frame.values["guidance.override.active"] is True
    attitude = cast(list[float], response.status_frame.values["attitude.euler"])
    body_rate = cast(list[float], response.status_frame.values["body_rate"])
    assert attitude[0] > 0.0
    assert attitude[1] > 3.1
    assert body_rate[0] > 0.0
    episode.close()
    ####


def test_b747_language_guidance_lqi_campaign_runs_through_the_common_host(
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
    ####


def test_b747_pseudo_guidance_lqi_campaign_is_selectable_through_the_common_host(
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


def test_b747_source_surface_lqi_campaign_runs_through_the_common_host(
    plugins: PluginCatalog,
    tmp_path: Path,
) -> None:
    """A registered local candidate does not need a runnable full racetrack."""

    providers = plugins.build_mission_composition_provider_registry()
    # Public mission/fidelity applicability is validated before realization
    # applicability.  Either fail-closed diagnostic proves that the local
    # surface tier cannot be misrepresented as the full transport racetrack.
    with pytest.raises(ModelAuthoringError, match="mission-fidelity-mismatch|realization-mission-mismatch"):
        resolve_model_authoring_selection(
            providers,
            PROVIDER_ID,
            MODEL_ID,
            fidelity="rigid_body_6dof_surface_allocated",
            realization_id="rigid_body_6dof_surface_allocated",
            mission_template_id=MISSION_ID,
        )

    output = tmp_path / "b747-source-surface-lqi.json"
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
    assert lqr["integral_output_names"] == ["roll_error_rad", "pitch_error_rad", "yaw_error_rad"]
    ####


def test_b747_physical_lqi_design_passes_the_bounded_source_local_baseline() -> None:
    """The named LQI design uses the real B747 allocator, not injected control.

    This is deliberately a local recovery baseline.  The separately exercised
    standard LQI screen admits only one fixed matched pitch-moment bias through
    an explicit derivative seam; neither check implies wind, mass, or transport
    offset-disturbance qualification.
    """

    plant = build_b747_condition3_source_table_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    assert trim.success
    design = build_b747_condition3_source_surface_physical_lqi_design()
    initial_state = dict(trim.state)
    initial_state.update(
        {
            "roll_error_rad": math.radians(1.0),
            "pitch_error_rad": math.radians(-0.5),
            "yaw_error_rad": math.radians(1.0),
            "p_rad_s": math.radians(0.5),
            "q_rad_s": math.radians(-0.5),
            "r_rad_s": math.radians(0.5),
        }
    )
    validation = validate_nonlinear_wrench_lqi(
        plant,
        trim,
        design,
        initial_state=initial_state,
        duration_s=80.0,
        dt_s=0.05,
        integral_lower={name: -0.5 for name in design.result.output_names},
        integral_upper={name: 0.5 for name in design.result.output_names},
    )

    final_error_fraction = validation.final_normalized_feedback_error_norm / validation.initial_normalized_feedback_error_norm
    assert design.result.hurwitz
    assert design.result.output_names == ("roll_error_rad", "pitch_error_rad", "yaw_error_rad")
    assert design.integral_q_diagonal == (0.025, 0.025, 0.025)
    assert validation.integrators_exercised
    assert final_error_fraction <= 0.10
    assert validation.final_controlled_actual_residual <= 10.0
    assert validation.saturation_fraction <= 0.05
    assert validation.maximum_continuous_saturation_duration_s <= 0.50
    assert set(validation.allocation_statuses) <= {"feasible", "feasible_near_limit", "partially_achievable"}
    ####


def test_b747_condition3_source_table_surface_lqr_screen_runs_through_public_composition(
    tmp_path: Path,
) -> None:
    """The physical tier is a local source-table screen, not a transport route claim."""

    composition = _b747_local_surface_composition()
    batch = execute_vehicle_composition_batch(composition, tmp_path / "b747-condition3-local-surface-lqr")
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
    assert runtime["controlled_wrench_axes"] == ["moment_x_nm", "moment_y_nm", "moment_z_nm"]
    assert runtime["effector_dynamics"] == "ideal bounded source-table coordinates; no source servo rate or lag data"
    assert lqi_candidate["campaign_id"] == SURFACE_CAMPAIGN_ID
    assert lqi_candidate["method"] == "lqi"
    assert lqi_candidate["controller_id"] == "b747-condition3-source-surface-wrench-lqi-v1"
    assert lqi_candidate["physical_allocation_baseline"] == "focused_bounded_source_local_recovery_passed"
    assert lqi_candidate["persistent_disturbance_status"] == "executed_by_paired_standard_lqi_screen"
    assert lqi_candidate["physical_screen_status"] == "not_executed_by_this_lqr_screen"
    lqi_execution = cast(dict[str, object], lqi_candidate["physical_screen_execution"])
    assert lqi_execution == {
        "status": "executed_by_paired_lqi_screen",
        "mission_id": "b747_condition3_local_physical_surface_lqi_screen_v1",
        "capability_adapter_id": "taoryx.b747_condition3_local_physical_surface_lqi_screen.capability.v1",
        "operations": ["validate", "batch"],
        "control_realization": "source_table_physical_wrench_lqi_allocation",
    }
    status_trace = json.loads((batch.output_dir / "status_trace.json").read_text(encoding="utf-8"))
    assert "resources.mass.total" in status_trace["channels"]
    assert all(sample["values"]["resources.mass.total"] == pytest.approx(runtime["mass_kg"]) for sample in status_trace["samples"])
    assert (batch.output_dir / "status_trace.json").is_file()
    assert (batch.output_dir / "semantic_action_trace.json").is_file()
    assert (batch.output_dir / "resource_ledger.json").is_file()
    ####


def test_b747_condition3_source_table_surface_lqi_screen_runs_through_public_composition(
    tmp_path: Path,
) -> None:
    """The condition-3 LQI endpoint emits its bounded pitch-offset evidence."""

    composition = _b747_local_surface_lqi_composition()
    batch = execute_vehicle_composition_batch(composition, tmp_path / "b747-condition3-local-surface-lqi")
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
    assert runtime["control_realization"] == "source_table_physical_wrench_lqi_allocation"
    assert runtime["physical_effector_allocation"] is True
    assert runtime["integral_q_diagonal"] == [0.025, 0.025, 0.025]
    offset_runtime = cast(dict[str, object], runtime["matched_pitch_offset_screen"])
    assert offset_runtime["status"] == "applied"
    assert offset_runtime["pass"] is True
    assert lqi_candidate["physical_screen_status"] == "executed_by_this_lqi_screen"
    assert cast(dict[str, object], lqi_candidate["physical_screen_execution"])["status"] == "executed_by_this_lqi_screen"
    assert lqi_candidate["persistent_disturbance_status"] == "executed_by_this_screen"
    automation = cast(dict[str, object], lqi_candidate["controller_automation"])
    assert cast(dict[str, object], automation["integral_priority_grid"])["multipliers"] == [0.1, 1.0, 10.0, 100.0]
    assert cast(dict[str, object], automation["physical_wrench_profile"])["integral_q_diagonal"] == [0.025, 0.025, 0.025]
    nonlinear_validation = cast(dict[str, object], json.loads((batch.output_dir / "nonlinear_validation.json").read_text()))
    robustness = cast(dict[str, object], json.loads((batch.output_dir / "robustness_report.json").read_text()))
    metrics = cast(dict[str, object], nonlinear_validation["metrics"])
    assert nonlinear_validation["schema"] == "taoryx.physical-lqi-validation/v1alpha1"
    assert metrics["integrators_exercised"] is True
    assert robustness["schema"] == "taoryx.endpoint-robustness-screen/v1alpha1"
    assert robustness["id"] == "b747-local-lqi-matched-pitch-wrench-offset"
    assert robustness["kind"] == "constant_offset"
    assert robustness["pass"] is True
    cases = cast(list[dict[str, object]], robustness["cases"])
    assert [case["id"] for case in cases] == ["nominal", "positive-pitch-offset", "negative-pitch-offset"]
    assert [cast(dict[str, float], case["parameters"])["pitch_wrench_bias_fraction"] for case in cases] == [0.0, 0.05, -0.05]
    assert all(case["status"] == "pass" for case in cases)
    assert all(cast(dict[str, float], case["metrics"])["final_feedback_error_fraction"] <= 0.10 for case in cases)
    assert all(cast(dict[str, float], case["metrics"])["saturation_fraction"] <= 0.05 for case in cases)
    assert (batch.output_dir / "robustness_report.json").is_file()
    assert (batch.output_dir / "status_trace.json").is_file()
    ####


def test_b747_catalog_advertises_the_exercised_batch_and_episode_endpoints() -> None:
    """The source-table transport racetrack has exact executable bindings."""

    kit = load_resolved_vehicle_composition_catalog().vehicle(MODEL_ID).authoring_kit_dict(MISSION_ID, FIDELITY)
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "language_backed_powered_fixed_wing.v1"),
        ("episode", "language_backed_interactive.v1"),
    }
    ####


def test_b747_direct_wrench_catalog_advertises_only_the_evidenced_batch_endpoint() -> None:
    """The autonomous source lane is batch-runnable, not externally interactive."""

    kit = (
        load_resolved_vehicle_composition_catalog()
        .vehicle(MODEL_ID)
        .authoring_kit_dict(MISSION_ID, "rigid_body_6dof_direct_wrench")
    )
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])
    interface = resolve_vehicle_composition_interface_contract(_b747_direct_wrench_composition())

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "language_backed_powered_fixed_wing.v1"),
    }
    assert {item.id for item in interface.action_channels} == {"wrench.force.command", "wrench.moment.command"}
    assert all(item.availability == "planned" for item in interface.action_channels)
    ####


def test_b747_condition3_surface_catalog_advertises_actual_source_effectors() -> None:
    """Actual physical coordinates are batch-visible without inventing actuator dynamics."""

    kit = (
        load_resolved_vehicle_composition_catalog()
        .vehicle(MODEL_ID)
        .authoring_kit_dict(
            LOCAL_SURFACE_MISSION_ID,
            "rigid_body_6dof_surface_allocated",
        )
    )
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])
    interface = resolve_vehicle_composition_interface_contract(_b747_local_surface_composition())

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "b747_condition3_local_physical_surface_lqr_screen.v1"),
    }
    assert {item.id for item in interface.effector_channels if item.availability == "available_in_batch"} == {
        "effector.throttle.position",
        "effector.elevator.position",
        "effector.aileron.position",
        "effector.rudder.position",
    }
    ####


def test_b747_condition3_surface_lqi_catalog_advertises_the_exact_offset_free_batch_endpoint() -> None:
    """LQI uses the same source coordinates but does not borrow the LQR binding."""

    kit = (
        load_resolved_vehicle_composition_catalog()
        .vehicle(MODEL_ID)
        .authoring_kit_dict(
            LOCAL_SURFACE_LQI_MISSION_ID,
            "rigid_body_6dof_surface_allocated",
        )
    )
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])
    interface = resolve_vehicle_composition_interface_contract(_b747_local_surface_lqi_composition())

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "b747_condition3_local_physical_surface_lqi_screen.v1"),
    }
    assert {item.id for item in interface.effector_channels if item.availability == "available_in_batch"} == {
        "effector.throttle.position",
        "effector.elevator.position",
        "effector.aileron.position",
        "effector.rudder.position",
    }
    ####
