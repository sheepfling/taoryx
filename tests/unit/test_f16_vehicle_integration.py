"""F-16 Composition and controller evidence outside the fast package gate."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, cast

import pytest
from taoryx.f16_reduced_execution import execute_f16_reduced_composition
from taoryx.source_f16 import (
    build_f16_local_physical_wrench_lqi_design,
    build_f16_source_physical_plant,
    build_f16_source_physical_schedule_lqi_nodes,
    build_f16_source_physical_schedule_nodes,
    f16_source_control_bounds,
    run_f16_source_physical_lqr_schedule_transition_cases,
)
from taoryx_f16.resources import model_resource_root

from taoryx.composition_episode import (
    ActionFrame,
    ReducedFixedWingCompositionEpisode,
    open_vehicle_composition_episode,
)
from taoryx.generic_tuning import validate_nonlinear_native_coordinate_lqi
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.physical_lqr import (
    apply_tuning_context_to_physical_wrench_lqi_design,
    apply_tuning_context_to_physical_wrench_lqr_design,
    validate_nonlinear_wrench_lqi,
)
from taoryx.plugins import PluginCatalog, discover_plugins
from taoryx.tuning_application import TuningApplicationContextSet
from taoryx.vehicle_batch_execution import execute_vehicle_composition_batch
from taoryx.vehicle_composition import (
    compile_vehicle_composition,
    load_vehicle_composition_request,
    resolve_vehicle_composition_interface_contract,
)
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog

RESOURCE_ROOT = model_resource_root()
PROVIDER_ID = "taoryx.f16.mission-composition"
MODEL_ID = "f16_s119"
MISSION_ID = "powered_fixed_wing_racetrack_v1"
PSEUDO_COMPOSITION = "f16_racetrack_capability_pseudo6dof_compose.yaml"
PHYSICAL_SCREEN_COMPOSITIONS = (
    (
        "f16_local_physical_direct_wrench_screen_compose.yaml",
        "rigid_body_6dof_direct_wrench",
    ),
    (
        "f16_local_physical_surface_screen_compose.yaml",
        "rigid_body_6dof_surface_allocated",
    ),
)
LQI_SURFACE_SCREEN_COMPOSITION = "f16_local_physical_surface_lqi_screen_compose.yaml"
LQI_SURFACE_SCREEN_MISSION_ID = "f16_local_physical_surface_lqi_screen_v1"
SCHEDULE_INTERIOR_SCREEN_COMPOSITION = "f16_local_physical_surface_lqr_schedule_interior_screen_compose.yaml"
SCHEDULE_INTERIOR_SCREEN_MISSION_ID = "f16_local_physical_surface_lqr_schedule_interior_screen_v1"
LQI_SCHEDULE_INTERIOR_SCREEN_COMPOSITION = "f16_local_physical_surface_lqi_schedule_interior_screen_compose.yaml"
LQI_SCHEDULE_INTERIOR_SCREEN_MISSION_ID = "f16_local_physical_surface_lqi_schedule_interior_screen_v1"
SCHEDULE_TRANSITION_SCREEN_COMPOSITION = "f16_local_physical_surface_lqr_schedule_transition_screen_compose.yaml"
SCHEDULE_TRANSITION_SCREEN_MISSION_ID = "f16_local_physical_surface_lqr_schedule_transition_screen_v1"


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover the installed distributions once for the focused slice."""

    return discover_plugins(include_external=False, selected=("taoryx.f16",))
    ####


def _pseudo_composition():
    """Compile the documented F-16 pseudo-6DOF request through Composition."""

    request = load_vehicle_composition_request(RESOURCE_ROOT / "examples/vehicle_composition" / PSEUDO_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def test_f16_advertisement_builds_a_complete_pseudo6dof_authoring_plan(
    plugins: PluginCatalog,
) -> None:
    """The plug-in advertises data, controls, navigation, and its shared tuner."""

    providers = plugins.build_mission_composition_provider_registry()
    campaigns = plugins.build_controller_tuning_campaign_registry()
    plan = build_model_authoring_plan(
        providers,
        campaigns,
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity="pseudo_6dof",
        realization_id="pseudo_6dof",
        mission_template_id=MISSION_ID,
    )

    assert plan["schema"] == "taoryx.model-authoring-plan/v1"
    assert plan["status"] == "ready_to_author"
    selection = cast(dict[str, object], plan["selection"])
    assert isinstance(selection["model_metadata_fingerprint"], str)
    assert len(cast(str, selection["model_metadata_fingerprint"])) == 64
    assert {key: value for key, value in selection.items() if key != "model_metadata_fingerprint"} == {
        "provider_id": PROVIDER_ID,
        "model_id": MODEL_ID,
        "model_version": "1.0.0+composition-v1",
        "family_id": MODEL_ID,
        "physical_family": "powered_fixed_wing",
        "fidelity": "pseudo_6dof",
        "realization_id": "pseudo_6dof",
        "mission_template_id": MISSION_ID,
        "sources": {
            "fidelity": "caller",
            "realization": "caller",
            "mission": "caller",
        },
    }
    data_contract = cast(dict[str, object], plan["data_contract"])
    assert data_contract["properties"]
    assert data_contract["source_refs"]
    controller = cast(dict[str, object], plan["controller_automation"])
    advertised_channels = cast(list[dict[str, Any]], controller["channels"])
    assert {item["id"] for item in advertised_channels} == {
        "guidance.speed.command",
        "guidance.flight_path_angle.command",
        "guidance.heading.command",
        "guidance.bank.command",
    }
    assert controller["default_authority_id"] == "kinematic_guidance"
    assert controller["channel_projection"] == "default_authority"
    available_channels = cast(list[dict[str, Any]], controller["available_channels"])
    assert {item["id"] for item in available_channels} > {item["id"] for item in advertised_channels}
    assert {item["id"] for item in cast(list[dict[str, object]], controller["authorities"])} == {
        "kinematic_guidance",
        "reduced_pilot_command",
        "body_rate_command",
        "live_waypoint_guidance",
    }
    advertised_campaigns = cast(list[dict[str, object]], controller["campaigns"])
    assert [item["id"] for item in advertised_campaigns] == ["f16-pseudo-source-trim-attitude-v1"]
    navigation = cast(dict[str, object], plan["navigation_automation"])
    assert cast(dict[str, object], navigation["mission_template"])["id"] == MISSION_ID
    segment_automation = cast(dict[str, object], plan["segment_automation"])
    assert segment_automation["instances"]
    ####


def test_f16_composition_compiles_and_executes_the_real_reduced_racetrack(
    tmp_path: Path,
) -> None:
    """A documented F-16 request reaches the source-backed batch executor."""

    composition = _pseudo_composition()
    result = execute_f16_reduced_composition(composition, tmp_path / "f16-racetrack")

    assert composition.family_id == MODEL_ID
    assert composition.fidelity == "pseudo_6dof"
    assert composition.control_realization == "response_law"
    assert result.mission_pass is True
    assert result.runtime["adapter_id"] == "taoryx.fixed_wing.daveml.v1"
    assert result.truth_evaluation["required_passed"] == 4
    assert (result.output_dir / "composition.json").is_file()
    assert (result.output_dir / "trim.json").is_file()
    assert (result.output_dir / "semantic_action_trace.json").is_file()
    ####


def test_f16_composition_episode_accepts_the_advertised_guidance_contract(
    tmp_path: Path,
) -> None:
    """The same composition exposes a usable interactive episode, not just batch output."""

    episode = open_vehicle_composition_episode(_pseudo_composition())
    contract = episode.interface_contract
    frame = ActionFrame(
        contract.id,
        contract.fingerprint,
        "kinematic_guidance",
        {
            "guidance.speed.command": 155.0,
            "guidance.flight_path_angle.command": 2.0,
            "guidance.heading.command": 0.0,
            "guidance.bank.command": 8.0,
        },
        1.0,
    )
    result = episode.step_frame(frame)

    assert isinstance(episode, ReducedFixedWingCompositionEpisode)
    assert result.status_frame is not None
    assert result.status_frame.values["control.realization"] == "response_law"
    assert result.status_frame.values["position.east"] > 0.0
    checkpoint = episode.save_checkpoint(tmp_path / "f16.checkpoint.json")
    expected = episode.observe().as_dict()
    episode.reset()
    episode.load_checkpoint(checkpoint)
    assert episode.observe().as_dict() == expected
    episode.close()
    ####


def test_f16_declared_controller_campaigns_run_through_the_common_host(
    plugins: PluginCatalog,
) -> None:
    """Both F-16 reductions supply data; the host supplies the tuning runner."""

    registry = plugins.build_controller_tuning_campaign_registry()
    expected_methods = {
        "f16-point-source-trim-translation-v1": ("lqr", False),
        "f16-pseudo-source-trim-attitude-v1": ("lqi", True),
        "f16-source-surface-local-lqr-v1": ("lqr", False),
        "f16-source-surface-schedule-lqr-v1": ("lqr", False),
        "f16-source-surface-local-lqi-v1": ("lqi", True),
        "f16-source-surface-schedule-lqi-v1": ("lqi", True),
    }

    for campaign_id, (method, expects_integral_outputs) in expected_methods.items():
        report = registry.registration(campaign_id).run()
        assert report.status == "candidate_ready"
        assert report.nodes
        assert all(node.lqr is not None and node.lqr.method == method for node in report.nodes)
        assert all(bool(node.lqr is not None and node.lqr.integral_output_names) == expects_integral_outputs for node in report.nodes)
    ####


def test_f16_pseudo_lqi_candidate_recovers_through_bounded_source_coordinates(
    plugins: PluginCatalog,
) -> None:
    """The source-calibrated response law retains a bounded nonlinear LQI proof.

    The source actuator overlay supplies coordinate travel bounds only. This
    remains a pseudo-6DOF response-law result, not evidence that the reduced
    model realizes physical F-16 actuator dynamics or allocation.
    """

    registration = plugins.build_controller_tuning_campaign_registry().registration("f16-pseudo-source-trim-attitude-v1")
    adapter, _ = registration.build()
    assert adapter.plant is not None
    report = registration.run()
    node = report.nodes[0]
    candidate = node.lqr.best if node.lqr is not None else None
    assert node.trim is not None
    assert candidate is not None
    assert candidate.lqi is not None
    initial_state = dict(node.trim.state)
    initial_state.update(
        {
            "roll_rad": 0.05,
            "pitch_rad": -0.03,
            "yaw_rad": 0.04,
            "p_rad_s": 0.02,
            "q_rad_s": -0.01,
            "r_rad_s": 0.02,
        }
    )
    bounds = f16_source_control_bounds()
    validation = validate_nonlinear_native_coordinate_lqi(
        adapter.plant,
        node.trim,
        candidate,
        initial_state=initial_state,
        duration_s=15.0,
        dt_s=0.02,
        assessment_state_names=("roll_rad", "pitch_rad", "yaw_rad", "p_rad_s", "q_rad_s", "r_rad_s"),
        control_lower={name: bounds[name][0] for name in candidate.control_names},
        control_upper={name: bounds[name][1] for name in candidate.control_names},
        integral_lower={name: -0.5 for name in candidate.lqi.output_names},
        integral_upper={name: 0.5 for name in candidate.lqi.output_names},
    )

    assert validation.integrators_exercised
    assert validation.final_normalized_feedback_error_norm < validation.initial_normalized_feedback_error_norm * 0.05
    assert validation.control_saturation_fraction == 0.0
    assert validation.saturated_controls == ()
    payload = validation.as_dict()
    assert payload["control_realization"] == "native_named_coordinates"
    assert payload["candidate"]["integral_output_names"] == ["roll_rad", "pitch_rad", "yaw_rad"]
    ####


def test_f16_physical_lqi_design_passes_the_bounded_source_local_baseline() -> None:
    """The source-local LQI passes through actual allocated effectors.

    This is a small fixed-altitude source-trim velocity recovery. The
    engineering actuator overlay supplies position, rate, and lag limits, but
    the source seam provides no declared wind or mass perturbation input; this
    test is not a persistent-disturbance or scheduled-envelope claim.
    """

    plant = build_f16_source_physical_plant()
    trim = plant.trim_result
    design = build_f16_local_physical_wrench_lqi_design()
    initial_state = dict(trim.state)
    initial_state.update(
        {
            "u_m_s": initial_state["u_m_s"] + 0.5,
            "v_m_s": 0.05,
            "w_m_s": initial_state["w_m_s"] - 0.05,
            "p_rad_s": 0.001,
            "q_rad_s": -0.001,
            "r_rad_s": 0.001,
        }
    )
    validation = validate_nonlinear_wrench_lqi(
        plant,
        trim,
        design,
        initial_state=initial_state,
        duration_s=5.0,
        dt_s=0.02,
        integral_lower={name: -1.0 for name in design.result.output_names},
        integral_upper={name: 1.0 for name in design.result.output_names},
    )

    final_error_fraction = validation.final_normalized_feedback_error_norm / validation.initial_normalized_feedback_error_norm
    assert design.result.hurwitz
    assert design.result.output_names == ("u_m_s", "v_m_s", "w_m_s")
    assert design.integral_q_diagonal == (0.03, 0.03, 0.03)
    assert validation.integrators_exercised
    assert final_error_fraction <= 0.05
    assert validation.final_controlled_actual_residual <= 20.0
    assert validation.saturation_fraction <= 0.05
    assert validation.maximum_continuous_saturation_duration_s <= 0.25
    assert set(validation.allocation_statuses) <= {"feasible", "feasible_near_limit", "partially_achievable"}
    ####


def test_f16_physical_schedule_nodes_are_owned_by_the_plugin_runtime() -> None:
    """The composed schedule screen does not recover controller nodes from a tool."""

    nodes = build_f16_source_physical_schedule_nodes()

    assert [(node.point_id, node.altitude_m, node.true_airspeed_m_s) for node in nodes] == [
        ("f16-sea-level-152mps", 0.0, 152.4),
        ("f16-3km-152mps", 3000.0, 152.4),
        ("f16-6km-152mps", 6000.0, 152.4),
        ("f16-9km-152mps", 9000.0, 152.4),
    ]
    assert all(node.design.result.hurwitz for node in nodes)
    assert all(node.design.projection.source_linearization.provenance.derivative_consistent for node in nodes)
    ####


def test_f16_physical_schedule_lqi_nodes_are_owned_by_the_plugin_runtime() -> None:
    """Each source schedule node supplies an offset-free velocity design."""

    nodes = build_f16_source_physical_schedule_lqi_nodes()

    assert [node.point_id for node in nodes] == [
        "f16-sea-level-152mps",
        "f16-3km-152mps",
        "f16-6km-152mps",
        "f16-9km-152mps",
    ]
    assert all(node.design.result.hurwitz for node in nodes)
    assert all(node.design.result.output_names == ("u_m_s", "v_m_s", "w_m_s") for node in nodes)
    ####


def test_f16_physical_schedule_interior_screen_runs_through_public_composition(
    tmp_path: Path,
) -> None:
    """Four F-16 physical nodes expose exact held-node interior evidence."""

    composition = compile_vehicle_composition(
        load_vehicle_composition_request(RESOURCE_ROOT / "examples/vehicle_composition" / SCHEDULE_INTERIOR_SCREEN_COMPOSITION)
    )
    batch = execute_vehicle_composition_batch(composition, tmp_path / "f16-surface-lqr-schedule-interior")
    payload = batch.as_dict()
    compact_report = cast(dict[str, object], payload["schedule_interior"])
    full_report = cast(dict[str, object], json.loads((batch.output_dir / "schedule_interior_report.json").read_text()))
    preflight = cast(dict[str, object], payload["preflight"])
    capability = cast(dict[str, object], cast(dict[str, object], preflight["derived_mission"])["capability"])
    status_trace = cast(dict[str, object], json.loads((batch.output_dir / "status_trace.json").read_text()))
    samples = cast(list[dict[str, object]], status_trace["samples"])

    assert batch.passed is True
    assert payload["status"] == "development_schedule_interior_screen_pass"
    assert compact_report["artifact"] == "schedule_interior_report.json"
    assert compact_report["summary"] == {
        "node_count": 4,
        "case_count": 8,
        "passed_case_count": 8,
        "failed_case_count": 0,
    }
    assert full_report["controller_selection"] == "discrete_source_node_held_for_each_recovery"
    assert [node["point_id"] for node in cast(list[dict[str, object]], full_report["nodes"])] == [
        "f16-sea-level-152mps",
        "f16-3km-152mps",
        "f16-6km-152mps",
        "f16-9km-152mps",
    ]
    assert capability["persistent_disturbance_status"] == "not_executable_without_a_declared_source_wind_or_mass_derivative_environment"
    assert cast(dict[str, object], capability["physical_screen_execution"])["operations"] == ["validate", "batch"]
    assert {cast(dict[str, object], sample["values"])["control.schedule.node_id"] for sample in samples} == {
        "f16-sea-level-152mps",
        "f16-3km-152mps",
        "f16-6km-152mps",
        "f16-9km-152mps",
    }
    assert {cast(dict[str, object], sample["values"])["control.schedule.selection"] for sample in samples} == {"discrete_source_node_held_for_each_recovery"}
    ####


def test_f16_physical_lqi_schedule_interior_screen_runs_through_public_composition(
    tmp_path: Path,
) -> None:
    """The four source nodes retain a separate, source-feasible LQI interior."""

    composition = compile_vehicle_composition(
        load_vehicle_composition_request(RESOURCE_ROOT / "examples/vehicle_composition" / LQI_SCHEDULE_INTERIOR_SCREEN_COMPOSITION)
    )
    batch = execute_vehicle_composition_batch(composition, tmp_path / "f16-surface-lqi-schedule-interior")
    payload = batch.as_dict()
    compact_report = cast(dict[str, object], payload["schedule_interior"])
    full_report = cast(dict[str, object], json.loads((batch.output_dir / "schedule_interior_report.json").read_text()))
    runtime = cast(dict[str, object], payload["runtime"])
    preflight = cast(dict[str, object], payload["preflight"])
    capability = cast(dict[str, object], cast(dict[str, object], preflight["derived_mission"])["capability"])
    nodes = cast(list[dict[str, object]], full_report["nodes"])

    assert batch.passed is True
    assert payload["status"] == "development_schedule_interior_screen_pass"
    assert runtime["controller_method"] == "lqi"
    assert compact_report["summary"] == {
        "node_count": 4,
        "case_count": 8,
        "passed_case_count": 8,
        "failed_case_count": 0,
    }
    assert full_report["controller_method"] == "lqi"
    assert full_report["interior_cases"] == {
        "alpha_plus_0_25mps": {"w_m_s": 0.25},
        "alpha_minus_0_25mps": {"w_m_s": -0.25},
    }
    assert all(cast(dict[str, object], case["result"])["saturation_fraction"] == 0.0 for node in nodes for case in cast(list[dict[str, object]], node["cases"]))
    assert capability["controller_method"] == "lqi"
    assert cast(dict[str, object], capability["physical_screen_execution"])["capability_adapter_id"] == (
        "taoryx.f16_local_physical_surface_lqi_schedule_interior_screen.capability.v1"
    )
    ####


@pytest.mark.slow
def test_f16_physical_schedule_transition_screen_runs_through_public_composition(
    tmp_path: Path,
) -> None:
    """The common runner executes bounded source-node schedule transitions."""

    composition = compile_vehicle_composition(
        load_vehicle_composition_request(RESOURCE_ROOT / "examples/vehicle_composition" / SCHEDULE_TRANSITION_SCREEN_COMPOSITION)
    )
    batch = execute_vehicle_composition_batch(composition, tmp_path / "f16-surface-lqr-schedule-transition")
    payload = batch.as_dict()
    compact_report = cast(dict[str, object], payload["schedule_transition"])
    full_report = cast(dict[str, object], json.loads((batch.output_dir / "schedule_transition_report.json").read_text()))
    robustness = cast(dict[str, object], json.loads((batch.output_dir / "robustness_report.json").read_text()))
    runtime = cast(dict[str, object], payload["runtime"])
    preflight = cast(dict[str, object], payload["preflight"])
    capability = cast(dict[str, object], cast(dict[str, object], preflight["derived_mission"])["capability"])
    status_trace = cast(dict[str, object], json.loads((batch.output_dir / "status_trace.json").read_text()))
    samples = cast(list[dict[str, object]], status_trace["samples"])

    assert batch.passed is True
    assert payload["status"] == "development_schedule_transition_screen_pass"
    assert compact_report["artifact"] == "schedule_transition_report.json"
    assert compact_report["summary"] == {
        "case_count": 4,
        "passed_case_count": 4,
        "failed_case_count": 0,
        "all_allocation_statuses": ["feasible"],
        "maximum_controlled_allocation_residual": full_report["summary"]["maximum_controlled_allocation_residual"],
    }
    assert full_report["direct_body_moment_injection"] is False
    assert all(cast(dict[str, object], case)["saturation_steps"] == 0 for case in cast(dict[str, object], full_report["cases"]).values())
    assert runtime["controller_selection"] == "linearly_interpolated_source_lqr_schedule_by_altitude_coordinate"
    persistent_disturbance = cast(dict[str, object], runtime["persistent_disturbance_screen"])
    assert persistent_disturbance["status"] == "applied"
    assert persistent_disturbance["pass"] is True
    assert robustness["schema"] == "taoryx.endpoint-robustness-screen/v1alpha1"
    assert robustness["release_evidence_schema"] == "taoryx.claim-bound-release-evidence/v1alpha1"
    assert robustness["release_evidence_kind"] == "robustness"
    assert cast(dict[str, object], robustness["release_evidence_subject"])["composition_identity_sha256"] == composition.identity_sha256
    assert robustness["id"] == "f16-schedule-matched-pitch-wrench-offset"
    assert robustness["kind"] == "constant_offset"
    assert robustness["pass"] is True
    robustness_cases = cast(list[dict[str, object]], robustness["cases"])
    assert [case["id"] for case in robustness_cases] == ["nominal", "positive-pitch-offset", "negative-pitch-offset"]
    assert [cast(dict[str, float], case["parameters"])["pitch_wrench_bias_fraction"] for case in robustness_cases] == [0.0, 0.05, -0.05]
    assert all(case["status"] == "pass" for case in robustness_cases)
    assert all(cast(dict[str, float], case["metrics"])["maximum_final_normalized_error"] <= 0.25 for case in robustness_cases)
    assert all(cast(dict[str, float], case["metrics"])["saturation_fraction"] == 0.0 for case in robustness_cases)
    assert cast(dict[str, object], capability["transition_execution"])["operations"] == ["validate", "batch"]
    assert capability["persistent_disturbance_status"] == "available_as_declared_matched_external_pitch_moment_screen"
    assert {cast(dict[str, object], sample["values"])["control.schedule.selection"] for sample in samples} == {
        "linearly_interpolated_source_lqr_schedule_by_altitude_coordinate"
    }
    assert {cast(dict[str, object], sample["values"])["control.controller.method"] for sample in samples} == {"lqr"}
    ####


def test_f16_physical_surface_lqi_screen_runs_through_public_composition(
    tmp_path: Path,
) -> None:
    """The source-local LQI evidence is a distinct public Composition endpoint."""

    composition = compile_vehicle_composition(load_vehicle_composition_request(RESOURCE_ROOT / "examples/vehicle_composition" / LQI_SURFACE_SCREEN_COMPOSITION))
    batch = execute_vehicle_composition_batch(composition, tmp_path / "f16-surface-lqi")
    payload = batch.as_dict()
    runtime = cast(dict[str, object], payload["runtime"])
    control_screen = cast(dict[str, object], payload["control_screen"])
    preflight = cast(dict[str, object], payload["preflight"])
    capability = cast(dict[str, object], cast(dict[str, object], preflight["derived_mission"])["capability"])
    nonlinear = cast(dict[str, object], json.loads((batch.output_dir / "nonlinear_validation.json").read_text()))
    metrics = cast(dict[str, object], nonlinear["metrics"])

    assert batch.passed is True
    assert payload["screen_pass"] is True
    assert control_screen["mission_pass"] is True
    assert runtime["controller_method"] == "lqi"
    assert runtime["physical_effector_allocation"] is True
    assert runtime["navigation_state"] == "fixed_local_origin_and_trim_attitude"
    assert capability["physical_screen_status"] == "executed_by_this_lqi_screen"
    assert cast(dict[str, object], capability["physical_screen_execution"]) == {
        "status": "executed_by_this_lqi_screen",
        "mission_id": "f16_local_physical_surface_lqi_screen_v1",
        "capability_adapter_id": "taoryx.f16_local_physical_surface_lqi_screen.capability.v1",
        "operations": ["validate", "batch"],
        "control_realization": "surface_allocated",
    }
    assert capability["persistent_disturbance_status"] == "not_executable_without_a_declared_source_wind_or_mass_derivative_environment"
    assert nonlinear["schema"] == "taoryx.physical-lqi-validation/v1alpha1"
    assert metrics["integrators_exercised"] is True
    assert (batch.output_dir / "status_trace.json").is_file()
    assert (batch.output_dir / "semantic_action_trace.json").is_file()
    ####


def test_f16_source_surface_lqi_candidate_matches_the_physical_wrench_runtime(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """The F-16 campaign and local LQI screen use one ordered wrench contract."""

    registration = plugins.build_controller_tuning_campaign_registry().registration("f16-source-surface-local-lqi-v1")
    context = registration.application_contexts(registration.run_cached(tmp_path / "tuning-cache"))[0]
    applied, binding = apply_tuning_context_to_physical_wrench_lqi_design(
        build_f16_local_physical_wrench_lqi_design(),
        context,
    )

    assert applied.projection.state_names == context.state_names
    assert applied.projection.wrench_names == context.control_names
    assert binding.campaign_id == registration.id
    ####


def test_f16_physical_surface_lqi_screen_applies_the_exact_common_tuning_candidate(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """The public F-16 batch emits a binding only after applying its selected gains."""

    registration = plugins.build_controller_tuning_campaign_registry().registration("f16-source-surface-local-lqi-v1")
    context = registration.application_contexts(registration.run_cached(tmp_path / "tuning-cache"))[0]
    composition = compile_vehicle_composition(load_vehicle_composition_request(RESOURCE_ROOT / "examples/vehicle_composition" / LQI_SURFACE_SCREEN_COMPOSITION))

    batch = execute_vehicle_composition_batch(
        composition,
        tmp_path / "f16-surface-lqi-tuned",
        tuning_context=context,
    )
    runtime = cast(dict[str, object], batch.as_dict()["runtime"])
    binding = cast(dict[str, object], runtime["tuning_binding"])

    assert batch.passed is True
    assert binding["campaign_id"] == registration.id
    assert binding["controller_method"] == "lqi"
    ####


def test_f16_held_node_lqi_schedule_applies_every_exact_campaign_candidate(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """The schedule cannot silently substitute one source-node gain for another."""

    registration = plugins.build_controller_tuning_campaign_registry().registration("f16-source-surface-schedule-lqi-v1")
    context_set = TuningApplicationContextSet(registration.application_contexts(registration.run_cached(tmp_path / "schedule-tuning-cache")))
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(RESOURCE_ROOT / "examples/vehicle_composition" / LQI_SCHEDULE_INTERIOR_SCREEN_COMPOSITION)
    )

    batch = execute_vehicle_composition_batch(
        composition,
        tmp_path / "f16-lqi-schedule-tuned",
        tuning_context_set=context_set,
    )
    runtime = cast(dict[str, object], batch.as_dict()["runtime"])
    bindings = cast(list[dict[str, object]], runtime["tuning_bindings"])

    assert batch.passed is True
    assert [binding["node_id"] for binding in bindings] == list(context_set.node_ids)
    assert [binding["candidate_profile_id"] for binding in bindings] == [context.candidate_profile_id for context in context_set.contexts]
    assert all(binding["campaign_id"] == registration.id for binding in bindings)
    ####


def test_f16_held_node_lqr_schedule_applies_every_exact_campaign_candidate(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """A held LQR schedule uses every node-indexed candidate, not one default gain."""

    registration = plugins.build_controller_tuning_campaign_registry().registration("f16-source-surface-schedule-lqr-v1")
    context_set = TuningApplicationContextSet(registration.application_contexts(registration.run_cached(tmp_path / "schedule-tuning-cache")))
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(RESOURCE_ROOT / "examples/vehicle_composition" / SCHEDULE_INTERIOR_SCREEN_COMPOSITION)
    )

    batch = execute_vehicle_composition_batch(
        composition,
        tmp_path / "f16-lqr-schedule-tuned",
        tuning_context_set=context_set,
    )
    runtime = cast(dict[str, object], batch.as_dict()["runtime"])
    bindings = cast(list[dict[str, object]], runtime["tuning_bindings"])

    assert batch.passed is True
    assert [binding["node_id"] for binding in bindings] == list(context_set.node_ids)
    assert [binding["candidate_profile_id"] for binding in bindings] == [context.candidate_profile_id for context in context_set.contexts]
    assert all(binding["campaign_id"] == registration.id for binding in bindings)
    applied, receipt = apply_tuning_context_to_physical_wrench_lqr_design(
        build_f16_source_physical_schedule_nodes()[0].design,
        context_set.contexts[0],
    )
    assert applied.result.hurwitz is True
    assert receipt.node_id == context_set.contexts[0].node_id
    ####


def test_f16_lqr_transition_applies_every_exact_candidate_before_interpolation(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """The transition runner cannot interpolate one unbound default schedule."""

    registration = plugins.build_controller_tuning_campaign_registry().registration("f16-source-surface-schedule-lqr-v1")
    context_set = TuningApplicationContextSet(registration.application_contexts(registration.run_cached(tmp_path / "transition-tuning-cache")))

    report = run_f16_source_physical_lqr_schedule_transition_cases(
        duration_s=0.05,
        dt_s=0.05,
        sample_stride_steps=1,
        tuning_context_set=context_set,
    )
    bindings = cast(list[dict[str, object]], report["tuning_bindings"])

    assert report["controller_selection"] == "linearly_interpolated_source_lqr_schedule_by_altitude_coordinate"
    assert [binding["node_id"] for binding in bindings] == list(context_set.node_ids)
    assert all(binding["campaign_id"] == registration.id for binding in bindings)
    ####


def test_f16_physical_surface_lqi_screen_is_selectable_through_the_common_plan(
    plugins: PluginCatalog,
) -> None:
    """The plan makes the LQI campaign and exact batch binding discoverable together."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity="rigid_body_6dof_surface_allocated",
        realization_id="rigid_body_6dof_surface_allocated",
        mission_template_id=LQI_SURFACE_SCREEN_MISSION_ID,
    )

    controller = cast(dict[str, object], plan["controller_automation"])
    execution = cast(dict[str, object], plan["execution_advertisement"])
    assert controller["status"] == "provider_managed_with_campaign"
    assert [item["id"] for item in cast(list[dict[str, object]], controller["campaigns"])] == [
        "f16-source-surface-local-lqi-v1",
    ]
    local_screen = cast(dict[str, object], controller["local_controller_screen"])
    assert local_screen["control_realization"] == "surface_allocated"
    assert cast(dict[str, object], local_screen["controller"])["integral_output_names"] == ["u_m_s", "v_m_s", "w_m_s"]
    assert [(item["id"], item["lower"], item["upper"]) for item in cast(list[dict[str, object]], local_screen["effector_controls"])] == [
        ("effector.elevator.position", -25.0, 25.0),
        ("effector.aileron.position", -21.0, 21.0),
        ("effector.rudder.position", -30.0, 30.0),
        ("effector.throttle.position", 0.0, 1.0),
    ]
    assert execution["status"] == "runnable"
    assert execution["endpoint_maturity"] == "batch_ready"
    assert execution["available_operations"] == ["validate", "batch"]
    ####


def test_f16_physical_schedule_interior_screen_is_selectable_through_the_common_plan(
    plugins: PluginCatalog,
) -> None:
    """The model-authoring plan exposes the held-node choice rather than a false scheduler."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity="rigid_body_6dof_surface_allocated",
        realization_id="rigid_body_6dof_surface_allocated",
        mission_template_id=SCHEDULE_INTERIOR_SCREEN_MISSION_ID,
    )

    controller = cast(dict[str, object], plan["controller_automation"])
    execution = cast(dict[str, object], plan["execution_advertisement"])
    local_screen = cast(dict[str, object], controller["local_controller_screen"])
    details = cast(dict[str, object], local_screen["controller"])

    assert [item["id"] for item in cast(list[dict[str, object]], controller["campaigns"])] == ["f16-source-surface-schedule-lqr-v1"]
    assert details["method"] == "lqr"
    assert details["selection"] == "discrete_source_node_held_for_each_recovery"
    assert execution["available_operations"] == ["validate", "batch"]
    ####


def test_f16_physical_schedule_transition_screen_is_selectable_through_the_common_plan(
    plugins: PluginCatalog,
) -> None:
    """The common authoring flow selects the reusable scheduled LQR campaign."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity="rigid_body_6dof_surface_allocated",
        realization_id="rigid_body_6dof_surface_allocated",
        mission_template_id=SCHEDULE_TRANSITION_SCREEN_MISSION_ID,
    )

    controller = cast(dict[str, object], plan["controller_automation"])
    execution = cast(dict[str, object], plan["execution_advertisement"])
    local_screen = cast(dict[str, object], controller["local_controller_screen"])
    details = cast(dict[str, object], local_screen["controller"])

    assert [item["id"] for item in cast(list[dict[str, object]], controller["campaigns"])] == ["f16-source-surface-schedule-lqr-v1"]
    assert details["method"] == "lqr"
    assert details["selection"] == "linearly_interpolated_source_lqr_schedule_by_altitude_coordinate"
    assert details["screen_duration_s"] == 240.0
    assert execution["available_operations"] == ["validate", "batch"]
    ####


def test_f16_physical_lqi_schedule_interior_screen_is_selectable_through_the_common_plan(
    plugins: PluginCatalog,
) -> None:
    """The plan exposes the held-node LQI campaign without claiming interpolation."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity="rigid_body_6dof_surface_allocated",
        realization_id="rigid_body_6dof_surface_allocated",
        mission_template_id=LQI_SCHEDULE_INTERIOR_SCREEN_MISSION_ID,
    )

    controller = cast(dict[str, object], plan["controller_automation"])
    execution = cast(dict[str, object], plan["execution_advertisement"])
    local_screen = cast(dict[str, object], controller["local_controller_screen"])
    details = cast(dict[str, object], local_screen["controller"])

    assert [item["id"] for item in cast(list[dict[str, object]], controller["campaigns"])] == ["f16-source-surface-schedule-lqi-v1"]
    assert details["method"] == "lqi"
    assert details["integral_output_names"] == ["u_m_s", "v_m_s", "w_m_s"]
    assert details["selection"] == "discrete_source_node_held_for_each_recovery"
    assert execution["available_operations"] == ["validate", "batch"]
    ####


def test_f16_physical_surface_lqi_catalog_advertises_actual_overlay_effectors() -> None:
    """The LQI mission is batch-runnable without borrowing the LQR endpoint name."""

    kit = load_resolved_vehicle_composition_catalog().vehicle(MODEL_ID).authoring_kit_dict(LQI_SURFACE_SCREEN_MISSION_ID, "rigid_body_6dof_surface_allocated")
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])
    composition = compile_vehicle_composition(load_vehicle_composition_request(RESOURCE_ROOT / "examples/vehicle_composition" / LQI_SURFACE_SCREEN_COMPOSITION))
    interface = resolve_vehicle_composition_interface_contract(composition)

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "f16_local_physical_surface_lqi_screen.v1"),
    }
    assert {item.id for item in interface.effector_channels if item.availability == "available_in_batch"} == {
        "effector.elevator.position",
        "effector.aileron.position",
        "effector.rudder.position",
        "effector.throttle.position",
    }
    ####


def test_f16_physical_surface_campaign_is_selectable_through_the_common_host(
    plugins: PluginCatalog,
) -> None:
    """The physical source-effector screen exposes selectable LQR and LQI paths."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity="rigid_body_6dof_surface_allocated",
        realization_id="rigid_body_6dof_surface_allocated",
        mission_template_id="f16_local_physical_control_screen_v1",
    )

    selection = cast(dict[str, object], plan["selection"])
    controller = cast(dict[str, object], plan["controller_automation"])
    campaigns = cast(list[dict[str, object]], controller["campaigns"])
    assert selection["fidelity"] == "rigid_body_6dof_surface_allocated"
    assert selection["realization_id"] == "rigid_body_6dof_surface_allocated"
    assert controller["status"] == "provider_managed_with_campaign"
    # Campaign applicability is part of the executable contract: the local
    # source-trim screen must not advertise node-schedule controllers that
    # require one of the dedicated schedule mission templates.  Those
    # templates exercise their respective schedule campaigns above.
    assert [item["id"] for item in campaigns] == [
        "f16-source-surface-local-lqr-v1",
        "f16-source-surface-local-lqi-v1",
    ]
    assert cast(dict[str, object], cast(dict[str, object], controller["local_controller_screen"])["controller"])["method"] == "lqr"
    ####


def test_f16_direct_wrench_screen_advertises_its_retained_source_lqr_profile(
    plugins: PluginCatalog,
) -> None:
    """The direct comparator is discoverable without mislabeling it as tunable."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        local_controller_screens=plugins.build_local_controller_screen_advertisement_registry(),
        fidelity="rigid_body_6dof_direct_wrench",
        realization_id="rigid_body_6dof_direct_wrench",
        mission_template_id="f16_local_physical_control_screen_v1",
    )

    controller = cast(dict[str, object], plan["controller_automation"])
    local_screen = cast(dict[str, object], controller["local_controller_screen"])
    details = cast(dict[str, object], local_screen["controller"])
    authority = cast(dict[str, object], local_screen["control_authority"])
    controls = cast(list[dict[str, object]], local_screen["direct_wrench_controls"])

    assert controller["status"] == "provider_managed"
    assert controller["campaigns"] == []
    assert local_screen["control_realization"] == "direct_wrench"
    assert local_screen["physical_effector_allocation"] is False
    assert details == {
        "method": "lqr",
        "controller_id": "f16.local_physical_wrench_lqr.v1",
        "campaign_id": None,
        "fixed_cadence_s": 0.2,
        "screen_duration_s": 1.0,
    }
    assert authority == {
        "availability": "internally_generated_batch_only",
        "external_override": False,
        "hard_limits": "not_declared",
    }
    assert [(item["channel_id"], item["component"], item["native_control_id"], item["normalization_scale"], item["hard_bounds"]) for item in controls] == [
        ("wrench.force.command", 0, "total_force_x_n", 5000.0, None),
        ("wrench.moment.command", 0, "total_moment_x_nm", 10000.0, None),
        ("wrench.moment.command", 1, "total_moment_y_nm", 10000.0, None),
        ("wrench.moment.command", 2, "total_moment_z_nm", 10000.0, None),
    ]
    ####


def test_f16_catalog_advertises_the_same_runnable_endpoints_as_the_slice() -> None:
    """The user-facing catalog does not overstate the verified vertical path."""

    kit = (
        load_resolved_vehicle_composition_catalog()
        .vehicle(MODEL_ID)
        .authoring_kit_dict(
            MISSION_ID,
            "pseudo_6dof",
        )
    )
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "reduced_fixed_wing_f16_source.v1"),
        ("episode", "reduced_fixed_wing_f16_episode.v1"),
    }
    ####


@pytest.mark.parametrize(("filename", "fidelity"), PHYSICAL_SCREEN_COMPOSITIONS)
def test_f16_physical_control_screens_run_through_the_composition_factory(
    tmp_path: Path,
    filename: str,
    fidelity: str,
) -> None:
    """Both physical F-16 control paths are public, bounded Composition screens."""

    composition = compile_vehicle_composition(load_vehicle_composition_request(RESOURCE_ROOT / "examples/vehicle_composition" / filename))
    batch = execute_vehicle_composition_batch(composition, tmp_path / fidelity)
    payload = batch.as_dict()

    assert batch.passed is True
    assert payload["status"] == "development_local_screen_pass"
    assert payload["screen_pass"] is True
    assert payload["plan"]["mode"] == ("direct_wrench" if fidelity == "rigid_body_6dof_direct_wrench" else "surface_allocated")
    assert payload["control_screen"]["mission_pass"] is True
    preflight = cast(dict[str, object], payload["preflight"])
    capability = cast(dict[str, object], preflight["capability_estimate"])
    assert capability["schema"] == "taoryx.concrete-capability-preflight/v1alpha1"
    assert capability["composition_identity_sha256"] == composition.identity_sha256
    assert capability["adapter_id"] == "taoryx.f16_local_physical_control_screen.capability.v1"
    status_trace = cast(dict[str, object], json.loads((batch.output_dir / "status_trace.json").read_text(encoding="utf-8")))
    samples = cast(list[dict[str, object]], status_trace["samples"])
    for sample in samples:
        values = cast(dict[str, object], sample["values"])
        requested_force = cast(list[float], values["control.wrench.requested.force"])
        achieved_force = cast(list[float], values["control.wrench.achieved.force"])
        residual_force = cast(list[float], values["control.wrench.residual.force"])
        requested_moment = cast(list[float], values["control.wrench.requested.moment"])
        achieved_moment = cast(list[float], values["control.wrench.achieved.moment"])
        residual_moment = cast(list[float], values["control.wrench.residual.moment"])
        assert residual_force == pytest.approx([requested - achieved for requested, achieved in zip(requested_force, achieved_force, strict=True)])
        assert residual_moment == pytest.approx([requested - achieved for requested, achieved in zip(requested_moment, achieved_moment, strict=True)])
    if fidelity == "rigid_body_6dof_surface_allocated":
        trace = cast(dict[str, object], payload["control_trace"])
        derived_mission = cast(dict[str, object], preflight["derived_mission"])
        lqi_candidate = cast(dict[str, object], cast(dict[str, object], derived_mission["capability"])["offset_free_tuning_candidate"])
        assert trace["achieved_effector_channel_count"] == 4
        assert lqi_candidate["campaign_id"] == "f16-source-surface-local-lqi-v1"
        assert lqi_candidate["controller_id"] == "f16.source_local_velocity_wrench_lqi.v1"
        assert lqi_candidate["physical_allocation_baseline"] == "focused_bounded_source_local_recovery_passed"
        assert lqi_candidate["persistent_disturbance_status"] == "not_executable_without_a_declared_source_wind_or_mass_derivative_environment"
    ####


def test_f16_physical_control_advertisement_distinguishes_local_readiness_from_qualification() -> None:
    """The catalogue exposes both physical tiers without calling either a flight mission."""

    vehicle = load_resolved_vehicle_composition_catalog().vehicle(MODEL_ID)
    for fidelity in ("rigid_body_6dof_direct_wrench", "rigid_body_6dof_surface_allocated"):
        kit = vehicle.authoring_kit_dict("f16_local_physical_control_screen_v1", fidelity)
        selection = cast(dict[str, object], kit["selection"])
        maturity = cast(dict[str, object], selection["operational_maturity"])
        endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])

        assert maturity["scope"] == "local_controller_screen"
        assert maturity["endpoint_maturity"] == "batch_ready"
        assert {(item["operation"], item["factory_id"]) for item in endpoints} == {
            ("batch", "f16_local_physical_control_screen.v1"),
        }
    ####
