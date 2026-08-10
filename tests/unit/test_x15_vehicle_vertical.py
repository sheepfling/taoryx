"""Fast vertical Composition proof for the runnable X-15 direct-wrench screen."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any, cast

import pytest
from taoryx.x15_adapter import (
    X15_INERTIA_BODY_KG_M2,
    X15_SOURCE_SURFACE_NAMES,
    build_x15_source_direct_wrench_plant,
    build_x15_source_surface_local_plant,
)

from taoryx.composition_episode import ActionFrame, open_vehicle_composition_episode
from taoryx.generic_tuning import validate_nonlinear_native_coordinate_lqi
from taoryx.local_direct_wrench_composition_execution import execute_local_direct_wrench_composition
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import PluginCatalog, discover_plugins
from taoryx.vehicle_batch_execution import execute_vehicle_composition_batch
from taoryx.vehicle_composition import (
    compile_vehicle_composition,
    load_vehicle_composition_request,
)
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog

ROOT = Path(__file__).resolve().parents[2]
PROVIDER_ID = "taoryx.registry.mission-composition"
MODEL_ID = "x15"
MISSION_ID = "x15_local_direct_wrench_screen_v1"
FIDELITY = "rigid_body_6dof_direct_wrench"
COMPOSITION = "x15_local_direct_wrench_screen_compose.yaml"
LQI_MISSION_ID = "x15_local_direct_wrench_lqi_screen_v1"
LQI_COMPOSITION = "x15_local_direct_wrench_lqi_screen_compose.yaml"
SURFACE_AUTHORITY_MISSION_ID = "x15_source_surface_authority_screen_v1"
SURFACE_AUTHORITY_FIDELITY = "rigid_body_6dof_surface_allocated"
SURFACE_AUTHORITY_COMPOSITION = "x15_source_surface_authority_screen_compose.yaml"
SURFACE_LQI_MISSION_ID = "x15_source_surface_attitude_rate_lqi_screen_v1"
SURFACE_LQI_COMPOSITION = "x15_source_surface_attitude_rate_lqi_screen_compose.yaml"


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover the installed distributions once for the focused slice."""

    return discover_plugins(include_external=False)
    ####


def _direct_wrench_composition():
    """Compile the documented X-15 local controller-screen request."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _direct_wrench_lqi_composition():
    """Compile the exact X-15 source-release LQI request."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / LQI_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _surface_authority_composition():
    """Compile the exact public X-15 three-surface source-authority request."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / SURFACE_AUTHORITY_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _surface_lqi_composition():
    """Compile the exact X-15 physical source-surface LQI request."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / SURFACE_LQI_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def test_x15_advertisement_builds_a_complete_direct_wrench_authoring_plan(
    plugins: PluginCatalog,
) -> None:
    """The plug-in advertises source data, bounded wrench controls, and its tuner."""

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

    assert plan["schema"] == "taoryx.model-authoring-plan/v1"
    assert plan["status"] == "ready_to_author"
    selection = cast(dict[str, object], plan["selection"])
    assert selection["model_id"] == MODEL_ID
    assert selection["physical_family"] == "powered_fixed_wing"
    assert selection["fidelity"] == FIDELITY
    assert selection["mission_template_id"] == MISSION_ID
    data_contract = cast(dict[str, object], plan["data_contract"])
    assert data_contract["properties"]
    assert data_contract["source_refs"]
    controller = cast(dict[str, object], plan["controller_automation"])
    channels = cast(list[dict[str, Any]], controller["channels"])
    assert {item["id"] for item in channels} == {
        "wrench.force.command",
        "wrench.moment.command",
    }
    campaigns = cast(list[dict[str, object]], controller["campaigns"])
    assert [item["id"] for item in campaigns] == ["x15-source-release-direct-wrench-v1"]
    assert cast(dict[str, object], plan["segment_automation"])["instances"]
    ####


def test_x15_composition_executes_the_source_local_direct_wrench_screen(
    tmp_path: Path,
) -> None:
    """The advertised batch endpoint is an actual bounded local-control screen."""

    result = execute_local_direct_wrench_composition(
        _direct_wrench_composition(),
        tmp_path / "x15-local-direct-wrench",
    )

    assert result.composition.family_id == MODEL_ID
    assert result.composition.fidelity == FIDELITY
    assert result.screen_pass is True
    assert result.preflight.status == "translation_ready"
    assert result.preflight.translator_id == "taoryx.x15_local_direct_wrench_screen.capability.v1"
    capability = result.preflight.capability_estimate
    assert capability is not None
    assert capability["schema"] == "taoryx.concrete-capability-preflight/v1alpha1"
    assert capability["composition_identity_sha256"] == result.composition.identity_sha256
    assert capability["adapter_id"] == result.preflight.translator_id
    derived_mission = cast(dict[str, object], result.preflight.as_dict()["derived_mission"])
    lqi_candidate = cast(dict[str, object], derived_mission["capability"])["offset_free_tuning_candidate"]
    assert cast(dict[str, object], lqi_candidate)["campaign_id"] == "x15-source-release-direct-wrench-lqi-v1"
    assert cast(dict[str, object], cast(dict[str, object], lqi_candidate)["controller_screen_execution"])["mission_id"] == LQI_MISSION_ID
    assert result.screen.observed_statuses == ("feasible",)
    assert (result.output_dir / "local_screen.json").is_file()
    assert (result.output_dir / "status_trace.json").is_file()
    assert (result.output_dir / "semantic_action_trace.json").is_file()
    assert "not prove X-15 flight trim" in result.claim_boundary
    ####


def test_x15_composition_episode_accepts_the_advertised_bounded_wrench_contract(
    tmp_path: Path,
) -> None:
    """The episode exposes the direct-wrench bridge without claiming effectors."""

    episode = open_vehicle_composition_episode(_direct_wrench_composition())
    contract = episode.interface_contract
    result = episode.step_frame(
        ActionFrame(
            contract.id,
            contract.fingerprint,
            "direct_wrench",
            {
                "wrench.force.command": [1.0e9, 0.0, 0.0],
                "wrench.moment.command": [0.0, 0.0, 0.0],
            },
            0.004,
        )
    )

    assert result.applied_semantic_action == {
        "wrench.force.command": [4.0e5, 0.0, 0.0],
        "wrench.moment.command": [0.0, 0.0, 0.0],
    }
    assert result.status_frame is not None
    assert result.status_frame.values["control.realization"] == "direct_wrench_screen"
    assert result.status_frame.values["control.wrench.saturated"] is True
    assert result.status_frame.values["control.physical_effector_allocation"] is False
    checkpoint = episode.save_checkpoint(tmp_path / "x15.checkpoint.json")
    expected = episode.observe().as_dict()
    episode.reset()
    episode.load_checkpoint(checkpoint)
    assert episode.observe().as_dict() == expected
    episode.close()
    ####


def test_x15_declared_controller_campaigns_run_through_the_common_host(
    plugins: PluginCatalog,
) -> None:
    """The X-15 plug-in supplies source-point data while core selects LQR or LQI."""

    registry = plugins.build_controller_tuning_campaign_registry()
    expected_methods = {
        "x15-source-release-direct-wrench-v1": ("lqr", False),
        "x15-source-release-direct-wrench-lqi-v1": ("lqi", True),
        "x15-source-surface-local-lqi-v1": ("lqi", True),
    }
    for campaign_id, (method, expects_integral_outputs) in expected_methods.items():
        report = registry.registration(campaign_id).run()
        assert report.status == "candidate_ready"
        assert report.nodes
        assert all(node.lqr is not None and node.lqr.method == method for node in report.nodes)
        assert all(bool(node.lqr is not None and node.lqr.integral_output_names) == expects_integral_outputs for node in report.nodes)
        if campaign_id == "x15-source-surface-local-lqi-v1":
            candidates = report.nodes[0].lqr.candidates if report.nodes[0].lqr is not None else ()
            assert any(candidate.integral_q_diagonal == (100_000.0, 100_000.0, 100_000.0) for candidate in candidates)
    ####


def test_x15_direct_wrench_lqi_candidate_recovers_through_declared_native_controls(
    plugins: PluginCatalog,
) -> None:
    """The LQI candidate uses bounded direct-wrench coordinates, not effectors."""

    report = plugins.build_controller_tuning_campaign_registry().registration(
        "x15-source-release-direct-wrench-lqi-v1"
    ).run()
    node = report.nodes[0]
    candidate = node.lqr.best if node.lqr is not None else None
    assert node.trim is not None
    assert candidate is not None
    assert candidate.lqi is not None
    plant = build_x15_source_direct_wrench_plant()
    initial_state = dict(node.trim.state)
    initial_state["u_m_s"] += 0.5
    validation = validate_nonlinear_native_coordinate_lqi(
        plant,
        node.trim,
        candidate,
        initial_state=initial_state,
        duration_s=3.5,
        dt_s=0.01,
        assessment_state_names=("u_m_s",),
        control_lower=plant.limits.lower,
        control_upper=plant.limits.upper,
        integral_lower={name: -1.0 for name in candidate.lqi.output_names},
        integral_upper={name: 1.0 for name in candidate.lqi.output_names},
    )

    assert validation.integrators_exercised
    assert validation.final_normalized_feedback_error_norm < validation.initial_normalized_feedback_error_norm * 0.05
    assert validation.control_saturation_fraction == 0.0
    assert validation.saturated_controls == ()
    payload = validation.as_dict()
    assert payload["control_realization"] == "native_named_coordinates"
    assert payload["candidate"]["integral_output_names"] == ["u_m_s"]
    ####


def test_x15_lqi_screen_runs_through_the_public_composition_path(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """The advertised source-release LQI candidate is a batch-ready exact mission."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity=FIDELITY,
        realization_id=FIDELITY,
        mission_template_id=LQI_MISSION_ID,
    )
    campaigns = cast(list[dict[str, object]], cast(dict[str, object], plan["controller_automation"])["campaigns"])
    local_advertisement = cast(
        dict[str, object],
        cast(dict[str, object], plan["controller_automation"])["local_controller_screen"],
    )
    execution = cast(dict[str, object], plan["execution_advertisement"])
    assert [item["id"] for item in campaigns] == ["x15-source-release-direct-wrench-lqi-v1"]
    assert local_advertisement["control_realization"] == "direct_wrench"
    assert local_advertisement["operations"] == ["batch"]
    assert cast(dict[str, object], local_advertisement["controller"])["method"] == "lqi"
    assert cast(dict[str, object], local_advertisement["controller"])["campaign_id"] == "x15-source-release-direct-wrench-lqi-v1"
    assert [item["id"] for item in cast(list[dict[str, object]], local_advertisement["direct_wrench_controls"])] == [
        "force_x_n",
        "force_y_n",
        "force_z_n",
        "moment_x_nm",
        "moment_y_nm",
        "moment_z_nm",
    ]
    assert execution["status"] == "runnable"
    assert execution["endpoint_maturity"] == "batch_ready"
    assert execution["available_operations"] == ["validate", "batch"]

    batch = execute_vehicle_composition_batch(
        _direct_wrench_lqi_composition(),
        tmp_path / "x15-local-direct-wrench-lqi",
    )
    payload = batch.as_dict()
    control_screen = cast(dict[str, object], payload["control_screen"])
    preflight = cast(dict[str, object], payload["preflight"])
    capability = cast(dict[str, object], cast(dict[str, object], preflight["derived_mission"])["capability"])
    local_screen = cast(dict[str, object], json.loads((batch.output_dir / "local_screen.json").read_text()))

    assert batch.passed is True
    assert control_screen["controller_method"] == "lqi"
    assert control_screen["integrators_exercised"] is True
    assert capability["controller_tuning_campaign_id"] == "x15-source-release-direct-wrench-lqi-v1"
    assert cast(dict[str, object], capability["controller_screen_execution"])["status"] == "executed_by_this_lqi_screen"
    assert cast(dict[str, object], local_screen["evaluation"])["assessment_state_names"] == ["u_m_s"]
    limits = cast(dict[str, object], local_screen["limits"])
    advertised_limits = {
        item["id"]: (item["lower"], item["upper"])
        for item in cast(list[dict[str, object]], local_advertisement["direct_wrench_controls"])
    }
    assert advertised_limits == {
        name: (value, cast(dict[str, float], limits["upper"])[name])
        for name, value in cast(dict[str, float], limits["lower"]).items()
    }
    assert (batch.output_dir / "status_trace.json").is_file()
    assert (batch.output_dir / "semantic_action_trace.json").is_file()
    ####


def test_x15_public_adapter_exercises_the_same_source_local_operations(
    plugins: PluginCatalog,
) -> None:
    """The advertised direct-wrench adapter is operation-probed at its pinned source point."""

    check = plugins.build_family_adapter_registry().check(MODEL_ID, FIDELITY)

    assert check.status == "pass"
    assert check.probe is not None
    operations = {item.operation: item.status for item in check.probe.operations}
    assert operations["state_derivative"] == "pass"
    assert operations["trim"] == "pass"
    assert operations["linearize"] == "pass"
    assert operations["effectiveness"] == "not_applicable"
    assert operations["allocate"] == "not_applicable"
    ####


def test_x15_catalog_advertises_the_same_runnable_endpoints_as_the_slice() -> None:
    """The public catalog names the local screen and its bounded interactive peer."""

    kit = load_resolved_vehicle_composition_catalog().vehicle(MODEL_ID).authoring_kit_dict(
        MISSION_ID,
        FIDELITY,
    )
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "local_direct_wrench_screen.v1"),
        ("episode", "x15_local_direct_wrench_episode.v1"),
    }
    ####


def test_x15_lqi_catalog_advertises_only_its_exercised_batch_endpoint() -> None:
    """The source-release LQI screen does not borrow the manual bridge episode."""

    kit = load_resolved_vehicle_composition_catalog().vehicle(MODEL_ID).authoring_kit_dict(
        LQI_MISSION_ID,
        FIDELITY,
    )
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "local_direct_wrench_screen.v1"),
    }
    ####


def test_x15_source_surface_authority_screen_is_public_physical_table_allocation(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """Composition advertises and executes the exact three-source-surface screen."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity=SURFACE_AUTHORITY_FIDELITY,
        realization_id=SURFACE_AUTHORITY_FIDELITY,
        mission_template_id=SURFACE_AUTHORITY_MISSION_ID,
    )
    channels = cast(list[dict[str, object]], cast(dict[str, object], plan["controller_automation"])["channels"])
    assert [(item["id"], item["availability"]) for item in channels] == [
        ("effector.surface.symmetric_stabilator.position", "available_in_batch"),
        ("effector.surface.differential_stabilator.position", "available_in_batch"),
        ("effector.surface.rudder.position", "available_in_batch"),
    ]

    batch = execute_vehicle_composition_batch(
        _surface_authority_composition(),
        tmp_path / "x15-source-surface-authority",
    )
    payload = batch.as_dict()
    runtime = cast(dict[str, object], payload["runtime"])
    capability = cast(dict[str, object], cast(dict[str, object], payload["preflight"])["derived_mission"])["capability"]
    trace = json.loads((batch.output_dir / "semantic_action_trace.json").read_text())

    assert batch.passed is True
    assert runtime["physical_effector_allocation"] is True
    assert runtime["effector_names"] == list(X15_SOURCE_SURFACE_NAMES)
    assert runtime["source_effectiveness_rank"] == 3
    assert cast(dict[str, object], runtime["full_state_trim"])["status"] == "not_available"
    assert cast(dict[str, object], capability)["controlled_wrench_axes"] == ["moment_x_nm", "moment_y_nm", "moment_z_nm"]
    assert set(cast(dict[str, object], trace)["achieved_effector_channels"]) == {
        "effector.surface.symmetric_stabilator.position",
        "effector.surface.differential_stabilator.position",
        "effector.surface.rudder.position",
    }
    assert (batch.output_dir / "surface_authority.json").is_file()
    ####


def test_x15_source_surface_lqi_screen_runs_through_the_public_composition_path(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """The surface-allocated X-15 path proves its local feedback scope end to end."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity=SURFACE_AUTHORITY_FIDELITY,
        realization_id=SURFACE_AUTHORITY_FIDELITY,
        mission_template_id=SURFACE_LQI_MISSION_ID,
    )
    controller = cast(dict[str, object], plan["controller_automation"])
    campaigns = cast(list[dict[str, object]], controller["campaigns"])
    local_advertisement = cast(dict[str, object], controller["local_controller_screen"])
    execution = cast(dict[str, object], plan["execution_advertisement"])

    assert [item["id"] for item in campaigns] == ["x15-source-surface-local-lqi-v1"]
    assert local_advertisement["control_realization"] == "source_surface_physical_wrench_lqi_allocation"
    assert cast(dict[str, object], local_advertisement["controller"])["method"] == "lqi"
    assert execution["status"] == "runnable"

    batch = execute_vehicle_composition_batch(
        _surface_lqi_composition(),
        tmp_path / "x15-source-surface-lqi",
    )
    payload = batch.as_dict()
    runtime = cast(dict[str, object], payload["runtime"])
    control_screen = cast(dict[str, object], payload["control_screen"])
    trace = json.loads((batch.output_dir / "semantic_action_trace.json").read_text())
    robustness = json.loads((batch.output_dir / "robustness_report.json").read_text())

    assert batch.passed is True
    assert runtime["physical_effector_allocation"] is True
    assert runtime["effector_names"] == list(X15_SOURCE_SURFACE_NAMES)
    assert runtime["controller_method"] == "lqi"
    assert runtime["integral_q_diagonal"] == [100_000.0, 100_000.0, 100_000.0]
    physical_profile = cast(dict[str, object], runtime["physical_wrench_profile"])
    assert physical_profile["status"] == "applied"
    assert cast(dict[str, object], physical_profile["matched_pitch_wrench_offset_screen"])["pass"] is True
    assert cast(dict[str, object], runtime["full_state_trim"])["status"] == "not_available"
    assert cast(dict[str, object], runtime["local_moment_balance_trim"])["status"] == "verified"
    assert control_screen["controller_method"] == "lqi"
    metrics = cast(dict[str, object], control_screen["metrics"])
    assert metrics["saturation_fraction"] == 0.0
    assert runtime["integrators_exercised"] is True
    assert runtime["control_saturation_fraction"] == 0.0
    assert set(cast(dict[str, object], trace)["achieved_effector_channels"]) == {
        "effector.surface.symmetric_stabilator.position",
        "effector.surface.differential_stabilator.position",
        "effector.surface.rudder.position",
    }
    with (batch.output_dir / "truth_telemetry.csv").open(encoding="utf-8", newline="") as stream:
        row = next(csv.DictReader(stream))
    assert all(name in row and math.isfinite(float(row[name])) for name in ("u_m_s", "v_m_s", "w_m_s"))
    assert (batch.output_dir / "local_surface_lqi_screen.json").is_file()
    assert robustness["schema"] == "taoryx.endpoint-robustness-screen/v1alpha1"
    assert robustness["id"] == "x15-local-lqi-matched-pitch-wrench-offset"
    assert robustness["kind"] == "constant_offset"
    assert robustness["pass"] is True
    cases = cast(list[dict[str, object]], robustness["cases"])
    assert [case["id"] for case in cases] == ["nominal", "positive-pitch-offset", "negative-pitch-offset"]
    assert [cast(dict[str, object], case["parameters"])["pitch_wrench_bias_fraction"] for case in cases] == [0.0, 0.05, -0.05]
    for case in cases:
        assert case["status"] == "pass"
        robustness_metrics = cast(dict[str, object], case["metrics"])
        assert float(robustness_metrics["final_feedback_error_fraction"]) <= 0.2
        assert float(robustness_metrics["saturation_fraction"]) == 0.0
    ####


def test_x15_source_surface_plant_exposes_a_declared_external_pitch_moment_disturbance_seam() -> None:
    """An offset screen perturbs source dynamics, never its LQI request path."""

    plant = build_x15_source_surface_local_plant()
    trim = plant.trim({}, {})
    nominal = plant.state_derivative(trim.state, trim.controls, {})
    biased = plant.state_derivative(
        trim.state,
        trim.controls,
        {"external_pitch_moment_bias_nm": 50_000.0},
    )

    assert biased["q_rad_s"] - nominal["q_rad_s"] == pytest.approx(
        50_000.0 / X15_INERTIA_BODY_KG_M2.y
    )
    with pytest.raises(ValueError, match="external_pitch_moment_bias_nm"):
        plant.state_derivative(trim.state, trim.controls, {"external_pitch_moment_bias_nm": "bad"})
    ####


def test_x15_source_surface_evaluator_changes_all_declared_controls_and_restores_source_defaults() -> None:
    """The screen uses real table coordinates, not a synthesized direct wrench."""

    plant = build_x15_source_direct_wrench_plant()
    state = dict(plant.reference_state)
    _, baseline = plant.source_surface_loads(state, {name: 0.0 for name in X15_SOURCE_SURFACE_NAMES})
    probes = {
        "symmetric_stabilator": {"moment_y_nm"},
        "differential_stabilator": {"moment_x_nm", "moment_z_nm"},
        "rudder": {"moment_x_nm", "moment_z_nm"},
    }
    for surface, affected_axes in probes.items():
        positions = {name: 0.0 for name in X15_SOURCE_SURFACE_NAMES}
        positions[surface] = 1.0
        _, perturbed = plant.source_surface_loads(state, positions)
        assert all(abs(perturbed[axis] - baseline[axis]) > 1.0 for axis in affected_axes)
    assert plant.vehicle.control_values["symmetric-stabilator-deg"] == 0.0
    assert plant.vehicle.control_values["differential-stabilator-deg"] == 0.0
    assert plant.vehicle.control_values["rudder-deg"] == 0.0
    ####


def test_x15_surface_authority_catalog_advertises_only_its_exercised_batch_endpoint() -> None:
    """The authority probe does not borrow the direct-wrench episode."""

    kit = load_resolved_vehicle_composition_catalog().vehicle(MODEL_ID).authoring_kit_dict(
        SURFACE_AUTHORITY_MISSION_ID,
        SURFACE_AUTHORITY_FIDELITY,
    )
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])

    assert {(item["operation"], item["factory_id"], item["execution_mode"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "x15_source_surface_authority_screen.v1", "source_surface_authority_screen"),
    }
    ####
