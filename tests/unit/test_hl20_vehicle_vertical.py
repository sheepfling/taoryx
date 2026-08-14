"""Fast vertical Composition proof for the runnable HL-20 direct-wrench screen."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any, cast

import pytest
from taoryx.hl20_adapter import (
    build_hl20_source_direct_wrench_tuning_plant,
    build_hl20_source_surface_physical_lqi_design,
)
from taoryx_hl20.resources import model_resource_root

from taoryx.composition_episode import ActionFrame, open_vehicle_composition_episode
from taoryx.generic_tuning import validate_nonlinear_native_coordinate_lqi
from taoryx.local_direct_wrench_composition_execution import execute_local_direct_wrench_composition
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.physical_lqr import apply_tuning_context_to_physical_wrench_lqi_design
from taoryx.plugins import PluginCatalog, discover_plugins, plugin_catalog_scope
from taoryx.vehicle_batch_execution import execute_vehicle_composition_batch
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog, load_vehicle_composition_registry
from taoryx.vehicle_trim_adapters import solve_vehicle_trim_evidence

PROVIDER_ID = "taoryx.hl20.mission-composition"
MODEL_ID = "hl20_mod_k"
MISSION_ID = "hl20_local_direct_wrench_screen_v1"
FIDELITY = "rigid_body_6dof_direct_wrench"
COMPOSITION = "hl20_local_direct_wrench_screen_compose.yaml"
LQI_MISSION_ID = "hl20_local_direct_wrench_lqi_screen_v1"
LQI_COMPOSITION = "hl20_local_direct_wrench_lqi_screen_compose.yaml"
SURFACE_AUTHORITY_MISSION_ID = "hl20_source_surface_pitch_authority_screen_v1"
SURFACE_AUTHORITY_FIDELITY = "rigid_body_6dof_surface_allocated"
SURFACE_AUTHORITY_COMPOSITION = "hl20_source_surface_pitch_authority_screen_compose.yaml"
SURFACE_LQI_MISSION_ID = "hl20_source_surface_attitude_rate_lqi_screen_v1"
SURFACE_LQI_COMPOSITION = "hl20_source_surface_attitude_rate_lqi_screen_compose.yaml"
HL20_SOURCE_SURFACES = {
    "upper_left_body_flap",
    "lower_left_body_flap",
    "upper_right_body_flap",
    "lower_right_body_flap",
    "left_wing_flap",
    "right_wing_flap",
    "rudder",
}


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover only the HL-20 plug-in for the family-owned vertical slice."""

    return discover_plugins(include_external=False, selected=("taoryx.hl20",))
    ####


@pytest.fixture(autouse=True)
def _hl20_plugin_scope(plugins: PluginCatalog):
    """Keep implicit runtime lookups inside the selected HL-20 plug-in scope."""

    with plugin_catalog_scope(plugins):
        yield
    ####


def _hl20_catalog():
    """Load the HL-20-owned catalog fragment rather than the checkout aggregate."""

    from taoryx.family_manifest import load_unified_family_manifest_catalog
    from taoryx.horizontal_fidelity import load_horizontal_registry
    from taoryx.trajectory.pseudo6dof_profiles import load_pseudo6dof_catalog

    root = model_resource_root()
    pseudo = load_pseudo6dof_catalog(root / "verification/pseudo6dof_profiles.yaml")
    horizontal = load_horizontal_registry(root / "verification/horizontal_fidelity_registry.yaml")
    manifests = load_unified_family_manifest_catalog(
        horizontal=horizontal,
        pseudo=pseudo,
        root=root,
        validate_source_imports=False,
    )
    registry = load_vehicle_composition_registry(root / "verification/vehicle_composition_registry.yaml")
    return load_resolved_vehicle_composition_catalog(registry=registry, manifests=manifests)
    ####


def _direct_wrench_composition():
    """Compile the documented HL-20 local controller-screen request."""

    request = load_vehicle_composition_request(model_resource_root() / "examples/vehicle_composition" / COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _direct_wrench_lqi_composition():
    """Compile the exact HL-20 subsonic LQI request."""

    request = load_vehicle_composition_request(model_resource_root() / "examples/vehicle_composition" / LQI_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _surface_authority_composition():
    """Compile the exact public seven-surface source-authority request."""

    request = load_vehicle_composition_request(model_resource_root() / "examples/vehicle_composition" / SURFACE_AUTHORITY_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def _surface_lqi_composition():
    """Compile the exact public HL-20 physical source-surface LQI request."""

    request = load_vehicle_composition_request(model_resource_root() / "examples/vehicle_composition" / SURFACE_LQI_COMPOSITION)
    return compile_vehicle_composition(request)
    ####


def test_hl20_focused_provider_excludes_optional_reachability_overlays(
    plugins: PluginCatalog,
) -> None:
    """Local HL-20 discovery never claims reachability-owned missions or inputs."""

    provider = plugins.build_mission_composition_provider_registry().provider(PROVIDER_ID)
    model = provider.model(MODEL_ID)

    assert [item.id for item in model.mission_templates] == [
        SURFACE_AUTHORITY_MISSION_ID,
        SURFACE_LQI_MISSION_ID,
        MISSION_ID,
        LQI_MISSION_ID,
    ]
    advertised_missions = {
        mission_id
        for realization in model.realizations
        for mission_id in realization.mission_template_ids
    }
    assert "hl20_source_booster_release_replay_v1" not in advertised_missions
    assert "lifting_body_glide_energy_management_v1" not in advertised_missions
    assert set(model.capabilities.initialization_modes) == {
        "source_subsonic_local_point",
        "source_mach1_pitch_trim_anchor",
    }
    assert set(model.capabilities.segment_types) == {
        "local_wrench_recovery_screen",
        "local_wrench_lqi_recovery_screen",
        "source_surface_pitch_authority_allocation_screen",
        "source_surface_attitude_rate_lqi_recovery_screen",
    }
    ####


def test_hl20_trim_evidence_uses_the_family_owned_source_binding(
    plugins: PluginCatalog,
) -> None:
    """The shared trim API resolves HL-20's recipe and DAVE-ML binding locally."""

    report = solve_vehicle_trim_evidence("reference_hl20_mod_k", plugins=plugins)

    assert report.status == "verified"
    assert report.adapter == "taoryx.adapters.reference_hl20_mod_k.daveml_pitch_channel"
    assert [point.point_id for point in report.points] == ["hl20-mach1-pitch-trim"]
    assert report.points[0].max_residual < 1.0e-10
    ####


def test_hl20_advertisement_builds_a_complete_direct_wrench_authoring_plan(
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
    assert selection["physical_family"] == "lifting_body"
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
    assert [item["id"] for item in campaigns] == ["hl20-source-subsonic-direct-wrench-v1"]
    assert cast(dict[str, object], plan["segment_automation"])["instances"]
    ####


def test_hl20_composition_executes_the_source_local_direct_wrench_screen(
    tmp_path: Path,
) -> None:
    """The advertised batch endpoint is an actual bounded local-control screen."""

    result = execute_local_direct_wrench_composition(
        _direct_wrench_composition(),
        tmp_path / "hl20-local-direct-wrench",
    )

    assert result.composition.family_id == MODEL_ID
    assert result.composition.fidelity == FIDELITY
    assert result.screen_pass is True
    assert result.preflight.status == "translation_ready"
    assert result.preflight.translator_id == "taoryx.hl20_local_direct_wrench_screen.capability.v1"
    capability = result.preflight.capability_estimate
    assert capability is not None
    derived_mission = cast(dict[str, object], result.preflight.as_dict()["derived_mission"])
    lqi_candidate = cast(dict[str, object], derived_mission["capability"])["offset_free_tuning_candidate"]
    assert cast(dict[str, object], lqi_candidate)["campaign_id"] == "hl20-source-subsonic-direct-wrench-lqi-v1"
    assert cast(dict[str, object], cast(dict[str, object], lqi_candidate)["controller_screen_execution"])["mission_id"] == LQI_MISSION_ID
    assert result.screen.observed_statuses == ("feasible",)
    mass_kg = result.screen.config.resource_values["mass_kg"]
    assert mass_kg > 0.0
    assert "resources.mass.total" in result.status_trace["channels"]
    assert all(sample["values"]["resources.mass.total"] == pytest.approx(mass_kg) for sample in result.status_trace["samples"])
    assert (result.output_dir / "local_screen.json").is_file()
    assert (result.output_dir / "status_trace.json").is_file()
    assert (result.output_dir / "semantic_action_trace.json").is_file()
    assert "high-energy performance" in result.claim_boundary
    assert "physical control-surface allocation" in result.claim_boundary
    ####


def test_hl20_composition_episode_accepts_the_advertised_bounded_wrench_contract(
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
        "wrench.force.command": [2.0e5, 0.0, 0.0],
        "wrench.moment.command": [0.0, 0.0, 0.0],
    }
    assert result.status_frame is not None
    assert result.status_frame.values["control.realization"] == "direct_wrench_screen"
    assert result.status_frame.values["control.wrench.saturated"] is True
    assert result.status_frame.values["control.physical_effector_allocation"] is False
    checkpoint = episode.save_checkpoint(tmp_path / "hl20.checkpoint.json")
    expected = episode.observe().as_dict()
    episode.reset()
    episode.load_checkpoint(checkpoint)
    assert episode.observe().as_dict() == expected
    episode.close()
    ####


def test_hl20_declared_controller_campaigns_run_through_the_common_host(
    plugins: PluginCatalog,
) -> None:
    """The plug-in supplies source-point data while core selects LQR or LQI."""

    registry = plugins.build_controller_tuning_campaign_registry()
    expected_methods = {
        "hl20-source-subsonic-direct-wrench-v1": ("lqr", False),
        "hl20-source-subsonic-direct-wrench-lqi-v1": ("lqi", True),
    }
    for campaign_id, (method, expects_integral_outputs) in expected_methods.items():
        report = registry.registration(campaign_id).run()
        assert report.status == "candidate_ready"
        assert report.nodes
        assert all(node.lqr is not None and node.lqr.method == method for node in report.nodes)
        assert all(bool(node.lqr is not None and node.lqr.integral_output_names) == expects_integral_outputs for node in report.nodes)
        assert all(node.authority_preflight is not None and node.authority_preflight.status == "passed" for node in report.nodes)
    ####


def test_hl20_direct_wrench_lqi_candidate_recovers_through_declared_native_controls(
    plugins: PluginCatalog,
) -> None:
    """The LQI candidate remains a bounded direct-wrench bridge result."""

    report = plugins.build_controller_tuning_campaign_registry().registration(
        "hl20-source-subsonic-direct-wrench-lqi-v1"
    ).run()
    node = report.nodes[0]
    candidate = node.lqr.best if node.lqr is not None else None
    assert node.trim is not None
    assert candidate is not None
    assert candidate.lqi is not None
    plant = build_hl20_source_direct_wrench_tuning_plant()
    initial_state = dict(node.trim.state)
    initial_state["u_m_s"] += 0.5
    validation = validate_nonlinear_native_coordinate_lqi(
        plant,
        node.trim,
        candidate,
        initial_state=initial_state,
        duration_s=4.0,
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


def test_hl20_lqi_screen_runs_through_the_public_composition_path(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """The advertised subsonic LQI candidate is a batch-ready exact mission."""

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
    execution = cast(dict[str, object], plan["execution_advertisement"])
    assert [item["id"] for item in campaigns] == ["hl20-source-subsonic-direct-wrench-lqi-v1"]
    assert execution["status"] == "runnable"
    assert execution["endpoint_maturity"] == "batch_ready"
    assert execution["available_operations"] == ["validate", "batch"]

    batch = execute_vehicle_composition_batch(
        _direct_wrench_lqi_composition(),
        tmp_path / "hl20-local-direct-wrench-lqi",
    )
    payload = batch.as_dict()
    control_screen = cast(dict[str, object], payload["control_screen"])
    preflight = cast(dict[str, object], payload["preflight"])
    capability = cast(dict[str, object], cast(dict[str, object], preflight["derived_mission"])["capability"])
    local_screen = cast(dict[str, object], json.loads((batch.output_dir / "local_screen.json").read_text()))

    assert batch.passed is True
    assert control_screen["controller_method"] == "lqi"
    assert control_screen["integrators_exercised"] is True
    assert capability["controller_tuning_campaign_id"] == "hl20-source-subsonic-direct-wrench-lqi-v1"
    assert cast(dict[str, object], capability["controller_screen_execution"])["status"] == "executed_by_this_lqi_screen"
    assert cast(dict[str, object], local_screen["evaluation"])["assessment_state_names"] == ["u_m_s"]
    assert (batch.output_dir / "status_trace.json").is_file()
    assert (batch.output_dir / "semantic_action_trace.json").is_file()
    ####


@pytest.mark.slow
def test_hl20_public_adapters_exercise_direct_and_surface_operations(
    plugins: PluginCatalog,
) -> None:
    """The surface overlay is operation-probed without mislabeling it as a flight controller."""

    registry = plugins.build_family_adapter_registry()
    direct = registry.check(MODEL_ID, FIDELITY)
    surface = registry.check(MODEL_ID, "rigid_body_6dof_surface_allocated")

    assert direct.status == "pass"
    assert surface.status == "pass"
    assert direct.probe is not None
    assert surface.probe is not None
    direct_operations = {item.operation: item.status for item in direct.probe.operations}
    surface_operations = {item.operation: item.status for item in surface.probe.operations}
    assert direct_operations["state_derivative"] == "pass"
    assert direct_operations["trim_fragment"] == "pass"
    assert direct_operations["effectiveness"] == "not_applicable"
    assert direct_operations["allocate"] == "not_applicable"
    assert surface_operations["state_derivative"] == "pass"
    assert surface_operations["trim_fragment"] == "pass"
    assert surface_operations["effectiveness"] == "pass"
    assert surface_operations["allocate"] == "pass"
    assert surface_operations["trim"] == "pass"
    assert surface_operations["linearize"] == "pass"
    ####


def test_hl20_surface_authority_screen_runs_through_public_composition_with_all_named_effectors(
    tmp_path: Path,
) -> None:
    """The source-surface tier has an exact batch route rather than only a probe API."""

    batch = execute_vehicle_composition_batch(
        _surface_authority_composition(),
        tmp_path / "hl20-source-surface-authority",
    )
    payload = batch.as_dict()
    control_screen = cast(dict[str, object], payload["control_screen"])
    preflight = cast(dict[str, object], payload["preflight"])
    runtime = cast(dict[str, object], payload["runtime"])
    status_trace = json.loads((batch.output_dir / "status_trace.json").read_text(encoding="utf-8"))
    control_trace = json.loads((batch.output_dir / "semantic_action_trace.json").read_text(encoding="utf-8"))

    assert batch.passed is True
    assert control_screen["mission_pass"] is True
    assert preflight["translator_id"] == "taoryx.hl20_source_surface_pitch_authority_screen.capability.v1"
    assert runtime["control_realization"] == "source_surface_pitch_authority_allocation"
    assert cast(dict[str, object], runtime["full_state_trim"])["status"] == "not_available"
    assert set(runtime["effector_names"]) == HL20_SOURCE_SURFACES
    assert control_trace["achieved_effector_channels"] == sorted(
        f"effector.surface.{name}.position" for name in HL20_SOURCE_SURFACES
    )
    assert status_trace["samples"][-1]["values"]["control.physical_effector_allocation"] is True
    assert status_trace["samples"][-1]["values"]["trim.full_state.status"] == "not_available"
    assert (batch.output_dir / "surface_authority.json").is_file()
    assert (batch.output_dir / "evaluation.json").is_file()
    ####


def test_hl20_surface_authority_authoring_plan_advertises_actual_surfaces_and_controller_boundary(
    plugins: PluginCatalog,
) -> None:
    """A plug-in author sees the real source controls without a fictional tuner."""

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
    controller = cast(dict[str, object], plan["controller_automation"])
    channels = cast(list[dict[str, object]], controller["channels"])
    execution = cast(dict[str, object], plan["execution_advertisement"])

    assert plan["status"] == "ready_to_author"
    assert {
        f"effector.surface.{name}.position" for name in HL20_SOURCE_SURFACES
    } <= {item["id"] for item in channels if item["channel_kind"] == "effector"}
    assert controller["campaigns"] == []
    assert execution["status"] == "runnable"
    assert execution["endpoint_maturity"] == "batch_ready"
    assert execution["available_operations"] == ["validate", "batch"]
    ####


@pytest.mark.slow
def test_hl20_source_surface_lqi_screen_runs_through_the_public_composition_path(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """The seven-surface feedback screen remains local and evidence-complete."""

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
    assert [item["id"] for item in campaigns] == ["hl20-source-surface-local-lqi-v1"]
    assert local_advertisement["control_realization"] == "source_surface_physical_wrench_lqi_allocation"
    assert cast(dict[str, object], local_advertisement["controller"])["method"] == "lqi"

    composition = _surface_lqi_composition()
    batch = execute_vehicle_composition_batch(composition, tmp_path / "hl20-source-surface-lqi")
    payload = batch.as_dict()
    runtime = cast(dict[str, object], payload["runtime"])
    control_screen = cast(dict[str, object], payload["control_screen"])
    trace = json.loads((batch.output_dir / "semantic_action_trace.json").read_text())
    robustness = cast(dict[str, object], json.loads((batch.output_dir / "robustness_report.json").read_text()))

    assert batch.passed is True
    assert runtime["physical_effector_allocation"] is True
    assert runtime["controller_method"] == "lqi"
    assert cast(dict[str, object], runtime["full_state_trim"])["status"] == "not_available"
    assert cast(dict[str, object], runtime["local_moment_balance_trim"])["status"] == "verified"
    assert runtime["integrators_exercised"] is True
    persistent_disturbance = cast(dict[str, object], runtime["persistent_disturbance_screen"])
    assert persistent_disturbance["status"] == "applied"
    assert persistent_disturbance["pass"] is True
    assert control_screen["controller_method"] == "lqi"
    assert set(cast(dict[str, object], trace)["achieved_effector_channels"]) == {
        f"effector.surface.{name}.position" for name in HL20_SOURCE_SURFACES
    }
    with (batch.output_dir / "truth_telemetry.csv").open(encoding="utf-8", newline="") as stream:
        row = next(csv.DictReader(stream))
    assert all(name in row and math.isfinite(float(row[name])) for name in ("u_m_s", "v_m_s", "w_m_s"))
    assert (batch.output_dir / "local_surface_lqi_screen.json").is_file()
    assert robustness["schema"] == "taoryx.endpoint-robustness-screen/v1alpha1"
    assert robustness["release_evidence_schema"] == "taoryx.claim-bound-release-evidence/v1alpha1"
    assert robustness["release_evidence_kind"] == "robustness"
    assert cast(dict[str, object], robustness["release_evidence_subject"])["composition_identity_sha256"] == composition.identity_sha256
    assert robustness["id"] == "hl20-local-lqi-matched-pitch-wrench-offset"
    assert robustness["kind"] == "constant_offset"
    assert robustness["pass"] is True
    robustness_cases = cast(list[dict[str, object]], robustness["cases"])
    assert [case["id"] for case in robustness_cases] == ["nominal", "positive-pitch-offset", "negative-pitch-offset"]
    assert [cast(dict[str, float], case["parameters"])["pitch_wrench_bias_fraction"] for case in robustness_cases] == [0.0, 0.05, -0.05]
    assert all(case["status"] == "pass" for case in robustness_cases)
    assert all(cast(dict[str, float], case["metrics"])["final_feedback_error_fraction"] <= 0.2 for case in robustness_cases)
    assert all(cast(dict[str, float], case["metrics"])["saturation_fraction"] == 0.0 for case in robustness_cases)
    ####


@pytest.mark.slow
def test_hl20_source_surface_lqi_candidate_matches_the_physical_wrench_runtime(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """The surface campaign emits gains in the screen's moment coordinates."""

    registration = plugins.build_controller_tuning_campaign_registry().registration("hl20-source-surface-local-lqi-v1")
    context = registration.application_contexts(registration.run_cached(tmp_path / "tuning-cache"))[0]
    applied, binding = apply_tuning_context_to_physical_wrench_lqi_design(
        build_hl20_source_surface_physical_lqi_design(),
        context,
    )

    assert applied.projection.state_names == context.state_names
    assert applied.projection.wrench_names == context.control_names
    assert binding.campaign_id == registration.id
    ####


@pytest.mark.slow
def test_hl20_source_surface_lqi_screen_applies_the_exact_common_tuning_candidate(
    tmp_path: Path,
    plugins: PluginCatalog,
) -> None:
    """The selected candidate is applied before the nonlinear seven-surface allocator."""

    registration = plugins.build_controller_tuning_campaign_registry().registration("hl20-source-surface-local-lqi-v1")
    context = registration.application_contexts(registration.run_cached(tmp_path / "tuning-cache"))[0]
    batch = execute_vehicle_composition_batch(
        _surface_lqi_composition(),
        tmp_path / "hl20-source-surface-lqi-tuned",
        tuning_context=context,
    )
    runtime = cast(dict[str, object], batch.as_dict()["runtime"])
    binding = cast(dict[str, object], runtime["tuning_binding"])

    assert batch.passed is True
    assert binding["campaign_id"] == registration.id
    assert binding["candidate_profile_id"] == context.candidate_profile_id
    assert binding["applied_gain_fingerprint_sha256"] == context.resolved_gain_fingerprint_sha256
    ####


def test_hl20_catalog_advertises_the_same_runnable_endpoints_as_the_slice() -> None:
    """The public catalog names the local screen and its bounded interactive peer."""

    kit = _hl20_catalog().vehicle(MODEL_ID).authoring_kit_dict(
        MISSION_ID,
        FIDELITY,
    )
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "hl20_local_direct_wrench_screen.v1"),
        ("episode", "hl20_local_direct_wrench_episode.v1"),
    }
    ####


def test_hl20_lqi_catalog_advertises_only_its_exercised_batch_endpoint() -> None:
    """The batch LQI screen does not borrow the manual direct-wrench episode."""

    kit = _hl20_catalog().vehicle(MODEL_ID).authoring_kit_dict(
        LQI_MISSION_ID,
        FIDELITY,
    )
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "hl20_local_direct_wrench_screen.v1"),
    }
    ####


def test_hl20_surface_authority_catalog_advertises_only_its_exercised_batch_endpoint() -> None:
    """The authority probe does not borrow a direct-wrench or controller episode."""

    kit = _hl20_catalog().vehicle(MODEL_ID).authoring_kit_dict(
        SURFACE_AUTHORITY_MISSION_ID,
        SURFACE_AUTHORITY_FIDELITY,
    )
    endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])

    assert {(item["operation"], item["factory_id"], item["execution_mode"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "hl20_source_surface_pitch_authority_screen.v1", "source_surface_authority_screen"),
    }
    ####
