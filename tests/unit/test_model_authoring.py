from __future__ import annotations

import json
from pathlib import Path

import pytest
from taoryx.trajectory.contract_probe_mission_composition import build_contract_probe_configuration
from taoryx.trajectory.dual_launch_mission_composition import build_dual_launch_example_configuration
from taoryx.trajectory.simple_aero_mission_composition import build_simple_aero_example_configuration

from taoryx.model_authoring import (
    ModelAuthoringError,
    author_configuration,
    build_model_authoring_plan,
    build_model_automation_assessment,
    build_model_automation_readiness_summary,
    compile_model_authoring_draft,
    custom_sequence,
    load_model_authoring_draft,
    resolve_model_authoring_selection,
    run_prepared_mission_composition,
    scaffold_model_authoring_draft,
    segment_occurrence,
    select_variant,
    sequence_template,
    write_model_authoring_draft,
)
from taoryx.plugins import PluginCatalog, discover_plugins
from taoryx.runtime.cli import main
from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection, RunnableMissionCompositionProvider


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover the checked-in distributions once for the authoring tests."""

    return discover_plugins(include_external=False)
    ####


def test_every_advertised_model_has_a_common_plan_and_plain_value_scaffold(
    plugins: PluginCatalog,
) -> None:
    providers = plugins.build_mission_composition_provider_registry()
    campaigns = plugins.build_controller_tuning_campaign_registry()
    adapters = plugins.build_family_adapter_registry()
    exercised: set[tuple[str, str]] = set()
    compiled_defaults: set[tuple[str, str]] = set()

    for provider in providers.providers:
        for model in provider.list_models():
            plan = build_model_authoring_plan(
                providers,
                campaigns,
                provider.metadata.id,
                model.id,
                family_adapters=adapters,
            )
            draft = scaffold_model_authoring_draft(
                providers,
                provider.metadata.id,
                model.id,
            )

            assert plan["schema"] == "taoryx.model-authoring-plan/v1"
            assert plan["status"] in {"ready_to_author", "selection_required"}
            assert plan["data_contract"]["configuration_schema_fingerprint"] == draft.schema_fingerprint  # type: ignore[index]
            maturity = plan["maturity_advertisement"]
            focused_endpoint = plan["focused_endpoint_verification"]
            if provider.metadata.id == "taoryx.registry.mission-composition":
                assert maturity["status"] == "declared"
                assert maturity["maturity"] in {"M4", "M5"}
            if focused_endpoint["status"] == "available":
                assert focused_endpoint["status"] == "available"
                assert focused_endpoint["endpoints"]
            else:
                assert focused_endpoint["status"] == "not_declared"
                assert focused_endpoint["endpoints"] == []
            assert draft.model_id == model.id
            assert draft.provider_id == provider.metadata.id
            assert isinstance(draft.values, dict)
            if not draft.unresolved_inputs:
                prepared = compile_model_authoring_draft(providers, draft)
                assert prepared.configuration.model_id == model.id
                compiled_defaults.add((provider.metadata.id, model.id))
            exercised.add((provider.metadata.id, model.id))

    assert exercised == {(provider.metadata.id, model.id) for provider in providers.providers for model in provider.list_models()}
    # This is an aggregate release check, so it must tolerate an independently
    # installed provider adding models while still proving every advertised
    # record was exercised. The default-compilable core set is a minimum, not
    # a closed inventory of plug-in-owned fixtures.
    assert {
        ("taoryx.registry.mission-composition", "x15"),
        ("taoryx.registry.mission-composition", "hl20_mod_k"),
        ("taoryx.registry.mission-composition", "f16_s119"),
        ("taoryx.registry.mission-composition", "hummingbird"),
        ("taoryx.hummingbird.mission-composition", "hummingbird"),
        ("taoryx.registry.mission-composition", "a320_openap_3dof"),
        ("taoryx.registry.mission-composition", "skywalker_x8"),
        ("taoryx.registry.mission-composition", "b747"),
    } <= compiled_defaults
    ####


def test_registry_workflow_models_execute_through_the_declared_provider_runner(
    plugins: PluginCatalog,
) -> None:
    """The authoring run path uses an explicit execution-capable provider contract."""

    providers = plugins.build_mission_composition_provider_registry()
    provider = providers.provider("taoryx.registry.mission-composition")
    assert isinstance(provider, RunnableMissionCompositionProvider)

    simple_aero = provider.validate_configuration(
        build_simple_aero_example_configuration(provider.get_model_schema("simple_aero"))
    )
    dual_launch = provider.validate_configuration(
        build_dual_launch_example_configuration("attached_booster", provider.get_model_schema("dual_launch_glider"))
    )
    for prepared in (simple_aero, dual_launch):
        response = run_prepared_mission_composition(
            providers,
            provider.metadata.id,
            prepared,
            output=MissionCompositionOutputSelection(mode="core", maximum_samples_per_object=3),
        )
        assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
        assert response.result.status == "completed"
        assert response.result.primary_model_id == prepared.configuration.model_id
    ####


def test_selection_reconciles_an_advertised_mission_with_its_only_fidelity(
    plugins: PluginCatalog,
) -> None:
    providers = plugins.build_mission_composition_provider_registry()

    selection = resolve_model_authoring_selection(
        providers,
        "taoryx.registry.mission-composition",
        "x15",
    )

    assert selection.fidelity == "rigid_body_6dof_surface_allocated"
    assert selection.realization is not None
    assert selection.realization.id == "rigid_body_6dof_surface_allocated"
    assert selection.mission is not None
    assert selection.mission.id == "x15_source_surface_authority_screen_v1"
    assert selection.sources["fidelity"] == "only_fidelity_compatible_with_advertised_mission"

    with pytest.raises(ModelAuthoringError, match="mission-fidelity-mismatch"):
        resolve_model_authoring_selection(
            providers,
            "taoryx.registry.mission-composition",
            "x15",
            fidelity="point_mass_3dof",
            mission_template_id="x15_local_direct_wrench_screen_v1",
        )
    ####


def test_selection_retains_an_explicit_reduced_fidelity_and_selects_its_runnable_mission(
    plugins: PluginCatalog,
) -> None:
    """A physical controller-screen default cannot override a caller's tier."""

    providers = plugins.build_mission_composition_provider_registry()

    selection = resolve_model_authoring_selection(
        providers,
        "taoryx.registry.mission-composition",
        "hummingbird",
        fidelity="pseudo_6dof",
    )

    assert selection.fidelity == "pseudo_6dof"
    assert selection.mission is not None
    assert selection.mission.id == "multirotor_pad_box_yaw_recovery_land_v1"
    assert selection.sources["mission"] == "first_available_fidelity_compatible_mission"
    assert selection.realization is not None
    assert selection.realization.id == "pseudo_6dof"
    ####


def test_plain_python_values_compile_through_the_exact_provider_schema(
    plugins: PluginCatalog,
) -> None:
    provider = plugins.build_mission_composition_provider_registry().provider("taoryx.registry.mission-composition")

    prepared = author_configuration(
        provider,
        configuration_id="programmatic-tumbling-body",
        model_id="tumbling_body",
        fidelity="point_mass_3dof",
        realization_id="point_mass_3dof",
        mission_template_id="tumbling_body_release_damping_impact_v1",
        values={
            "initialization": select_variant(
                "atmospheric_release",
                altitude_m=1000.0,
                speed_m_s=120.0,
                body_rates_rad_s=[0.1, 0.2, 0.3],
                area_policy="orientation_averaged_projected_area",
            ),
            "segments": sequence_template(
                "tumbling_body_release_damping_impact_v1",
                {"duration_s": 10.0},
                {"impact_plane_altitude_m": 0.0},
            ),
        },
    )

    assert prepared.configuration.configuration_id == "programmatic-tumbling-body"
    assert prepared.resolved["initialization"]["selected"] == "atmospheric_release"  # type: ignore[index]
    assert [item["selected"] for item in prepared.resolved["segments"]] == [  # type: ignore[index]
        "passive_coast",
        "atmospheric_descent",
    ]
    ####


def test_open_waypoint_sequences_have_concise_programmatic_occurrences(
    plugins: PluginCatalog,
) -> None:
    provider = plugins.build_mission_composition_provider_registry().provider("taoryx.reference.mission-composition")

    prepared = author_configuration(
        provider,
        configuration_id="two-waypoint-reference",
        model_id="reference_constant_velocity_waypoint_3dof",
        fidelity="point_mass_3dof",
        realization_id="analytical_point_mass",
        values={
            "initialization": select_variant(
                "initial_state",
                altitude_m=1000.0,
                speed_m_s=50.0,
                heading_deg=0.0,
            ),
            "segments": custom_sequence(
                segment_occurrence(
                    "waypoint_leg",
                    instance_id="north-leg",
                    duration_s=30.0,
                    waypoint_north_m=1000.0,
                    waypoint_east_m=0.0,
                    waypoint_altitude_m=1000.0,
                ),
                segment_occurrence(
                    "waypoint_leg",
                    instance_id="east-leg",
                    duration_s=30.0,
                    waypoint_north_m=1000.0,
                    waypoint_east_m=1000.0,
                    waypoint_altitude_m=1000.0,
                ),
            ),
        },
    )

    segments = prepared.resolved["segments"]
    assert [item["instance_id"] for item in segments] == ["north-leg", "east-leg"]  # type: ignore[union-attr]
    assert [item["value"]["waypoint_east_m"] for item in segments] == [0.0, 1000.0]  # type: ignore[union-attr]
    ####


def test_three_leading_composition_models_run_through_the_consumer_seam(
    plugins: PluginCatalog,
) -> None:
    """Keep the ballistic, waypoint, and full-contract fixtures runnable end to end."""

    providers = plugins.build_mission_composition_provider_registry()
    reference = providers.provider("taoryx.reference.mission-composition")
    ballistic = author_configuration(
        reference,
        configuration_id="consumer-ballistic",
        model_id="reference_ballistic_3dof",
        fidelity="point_mass_3dof",
        realization_id="analytical_point_mass",
        values={
            "initialization": select_variant(
                "launch_state",
                altitude_m=1000.0,
                speed_m_s=150.0,
                heading_deg=0.0,
                flight_path_angle_deg=20.0,
            ),
            "segments": custom_sequence(
                segment_occurrence("ballistic_coast", duration_s=2.0),
                segment_occurrence("ballistic_coast", duration_s=2.0),
            ),
        },
    )
    waypoint = author_configuration(
        reference,
        configuration_id="consumer-waypoint",
        model_id="reference_constant_velocity_waypoint_3dof",
        fidelity="point_mass_3dof",
        realization_id="analytical_point_mass",
        values={
            "initialization": select_variant(
                "initial_state",
                altitude_m=1000.0,
                speed_m_s=100.0,
                heading_deg=90.0,
            ),
            "segments": custom_sequence(
                segment_occurrence(
                    "waypoint_leg",
                    instance_id="outbound",
                    duration_s=5.0,
                    waypoint_north_m=0.0,
                    waypoint_east_m=500.0,
                    waypoint_altitude_m=1000.0,
                ),
                segment_occurrence(
                    "waypoint_leg",
                    instance_id="return",
                    duration_s=5.0,
                    waypoint_north_m=0.0,
                    waypoint_east_m=0.0,
                    waypoint_altitude_m=1000.0,
                ),
            ),
        },
    )
    probe = providers.provider("taoryx.debug.mission-composition-contract-probe")
    contract = probe.validate_configuration(build_contract_probe_configuration(probe))  # type: ignore[arg-type]

    ballistic_response = run_prepared_mission_composition(providers, reference.metadata.id, ballistic)
    waypoint_response = run_prepared_mission_composition(providers, reference.metadata.id, waypoint)
    contract_response = run_prepared_mission_composition(
        providers,
        probe.metadata.id,
        contract,
        output=MissionCompositionOutputSelection(mode="all"),
    )

    assert ballistic_response.kind == "trajectory"
    assert ballistic_response.result.primary_model_id == "reference_ballistic_3dof"
    assert ballistic_response.result.objects[0].samples[-1].time_s == pytest.approx(4.0)
    assert waypoint_response.kind == "trajectory"
    assert waypoint_response.result.primary_model_id == "reference_constant_velocity_waypoint_3dof"
    assert waypoint_response.result.objects[0].samples[-1].time_s == pytest.approx(10.0)
    assert contract_response.kind == "trajectory"
    assert contract_response.result.primary_model_id == "contract_probe_vehicle"
    assert len(contract_response.result.objects) == 3
    ####


def test_generated_draft_round_trips_and_rejects_unresolved_or_stale_input(
    plugins: PluginCatalog,
    tmp_path: Path,
) -> None:
    providers = plugins.build_mission_composition_provider_registry()
    draft = scaffold_model_authoring_draft(
        providers,
        "taoryx.registry.mission-composition",
        "tumbling_body",
    )
    path = write_model_authoring_draft(draft, tmp_path / "tumbling-body.yaml")

    assert load_model_authoring_draft(path) == draft
    assert draft.unresolved_inputs
    with pytest.raises(ModelAuthoringError, match="unresolved-inputs"):
        compile_model_authoring_draft(providers, draft)

    stale = draft.model_copy(
        update={
            "schema_fingerprint": "0" * 64,
            "values": {
                "initialization": select_variant(
                    "atmospheric_release",
                    altitude_m=1000.0,
                    speed_m_s=120.0,
                    body_rates_rad_s=[0.1, 0.2, 0.3],
                    area_policy="orientation_averaged_projected_area",
                ),
                "segments": sequence_template(
                    "tumbling_body_release_damping_impact_v1",
                    {"duration_s": 10.0},
                    {"impact_plane_altitude_m": 0.0},
                ),
            },
        }
    )
    with pytest.raises(ModelAuthoringError, match="schema-fingerprint-mismatch"):
        compile_model_authoring_draft(providers, stale)
    ####


def test_registered_reference_campaigns_have_a_well_formed_common_runner_contract(
    plugins: PluginCatalog,
) -> None:
    """Validate every declaration without running every numerical screen.

    The individual vehicle-vertical suites execute their family-owned
    campaigns.  Running the entire catalog here made an authoring/metadata
    change silently invoke every model's trim, linearization, and LQR design
    screen.  Keep this fast catalog test structural; the exhaustive common
    runner proof remains below as an explicit slow integration test.
    """

    registry = plugins.build_controller_tuning_campaign_registry()
    registry.validate_against(plugins.build_mission_composition_provider_registry())

    catalog = registry.public_dict()
    advertised_campaigns = catalog["campaigns"]
    assert catalog["schema"] == "taoryx.controller-tuning-campaign-catalog/v1"
    assert isinstance(advertised_campaigns, list) and advertised_campaigns
    assert [item["id"] for item in advertised_campaigns] == [item.id for item in registry.registrations]
    for registration in registry.registrations:
        advertisement = registration.public_dict()
        assert advertisement["id"] == registration.id
        assert advertisement["provider_id"] == registration.provider_id
        assert advertisement["model_id"] == registration.model_id
        assert advertisement["family_id"] == registration.family_id
        assert advertisement["fidelity"] == registration.fidelity
        assert advertisement["realization_ids"] == list(registration.realization_ids)
        assert advertisement["mission_template_ids"] == list(registration.mission_template_ids)
        assert advertisement["local_controller_screens"] == [dict(screen) for screen in registration.local_controller_screens]
        assert advertisement["claim_boundary"] == registration.claim_boundary
    ####


@pytest.mark.slow
@pytest.mark.integration
def test_registered_reference_campaigns_execute_through_the_common_runner(
    plugins: PluginCatalog,
) -> None:
    """Exercise every declared campaign through the host runner on opt-in."""

    registry = plugins.build_controller_tuning_campaign_registry()
    for registration in registry.registrations:
        report = registration.run()
        assert report.status == "candidate_ready"
        assert all(node.status == "candidate_ready" for node in report.nodes)
        declared_nodes = {node.node_id: node for node in report.campaign.nodes}
        assert all(node.lqr is not None for node in report.nodes)
        for node in report.nodes:
            declared = declared_nodes[node.node_id]
            assert node.lqr is not None
            assert node.lqr.method == declared.controller_method
            assert node.lqr.integral_output_names == declared.integral_output_names
    ####


def test_registered_campaign_cache_is_content_addressed(
    plugins: PluginCatalog,
    tmp_path: Path,
) -> None:
    registration = plugins.build_controller_tuning_campaign_registry().registration("hummingbird-pseudo-hover-attitude-v1")

    first = registration.run_cached(tmp_path, context_fingerprint=plugins.fingerprint)
    second = registration.run_cached(tmp_path, context_fingerprint=plugins.fingerprint)

    assert not first.cache_hit
    assert second.cache_hit
    assert first.cache_key == second.cache_key
    assert first.payload == second.payload
    assert second.cache_path is not None and second.cache_path.is_file()
    ####


def test_authoring_plan_advertises_explicit_tuner_selection_and_safe_cache_reuse(
    plugins: PluginCatalog,
) -> None:
    """Plug-in authors can discover selection and reuse rules before tuning."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        "taoryx.registry.mission-composition",
        "hummingbird",
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity="rigid_body_6dof_surface_allocated",
        realization_id="rigid_body_6dof_surface_allocated",
        mission_template_id="hummingbird_local_vertical_translation_lqi_screen_v1",
    )
    controller = plan["controller_automation"]
    assert isinstance(controller, dict)
    selection = controller["tuner_selection"]
    cache = controller["tuning_cache"]
    assert isinstance(selection, dict) and isinstance(cache, dict)
    assert selection["campaign_ids"] == ["hummingbird-source-rotor-vertical-lqi-v1"]
    assert selection["cli_argument"] == "--campaign <campaign-id>"
    assert cache["status"] == "available"
    assert cache["default_directory"] == "build/controller-cache"
    assert cache["cli_options"] == {"directory": "--cache-dir <path>", "bypass": "--no-cache"}
    assert cache["identity_inputs"] == [
        "installed_plugin_catalog_fingerprint",
        "campaign_registration_advertisement",
        "campaign_adapter_descriptor",
        "campaign_declaration",
    ]
    ####


def test_a320_named_surrogate_plan_and_lqi_campaign_are_selectable_and_cached(tmp_path: Path) -> None:
    """An agent can select and tune the named response-law product without adapter knowledge."""

    plan_path = tmp_path / "a320-jsbsim-surrogate-plan.json"
    first_tuning_path = tmp_path / "a320-jsbsim-surrogate-tuning-first.json"
    second_tuning_path = tmp_path / "a320-jsbsim-surrogate-tuning-second.json"
    cache_dir = tmp_path / "controller-cache"
    selection = [
        "taoryx.registry.mission-composition",
        "a320_openap_3dof",
        "--fidelity",
        "pseudo_6dof",
        "--realization",
        "jsbsim_surrogate_composite_pseudo6dof",
        "--mission",
        "powered_fixed_wing_racetrack_v1",
    ]

    assert main(["model", "plan", *selection, "--output", str(plan_path)]) == 0
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    assert plan["selection"]["realization_id"] == "jsbsim_surrogate_composite_pseudo6dof"
    assert plan["selection"]["sources"]["realization"] == "caller"
    campaigns = plan["controller_automation"]["campaigns"]
    assert [item["id"] for item in campaigns] == ["a320-pseudo-cruise-attitude-v1"]
    assert campaigns[0]["realization_ids"] == ["pseudo_6dof", "jsbsim_surrogate_composite_pseudo6dof"]

    tune_arguments = [
        "model",
        "tune",
        *selection,
        "--campaign",
        "a320-pseudo-cruise-attitude-v1",
        "--cache-dir",
        str(cache_dir),
    ]
    assert main([*tune_arguments, "--output", str(first_tuning_path)]) == 0
    assert main([*tune_arguments, "--output", str(second_tuning_path)]) == 0

    first = json.loads(first_tuning_path.read_text(encoding="utf-8"))
    second = json.loads(second_tuning_path.read_text(encoding="utf-8"))
    assert first["selection"]["realization_id"] == "jsbsim_surrogate_composite_pseudo6dof"
    assert first["report"]["status"] == "candidate_ready"
    assert first["report"]["nodes"][0]["lqr"]["method"] == "lqi"
    assert first["cache"]["hit"] is False
    assert second["cache"]["hit"] is True
    assert first["cache"]["key"] == second["cache"]["key"]
    ####


@pytest.mark.slow
@pytest.mark.matrix
@pytest.mark.integration
def test_model_automation_assessment_exercises_every_advertised_realization(
    plugins: PluginCatalog,
) -> None:
    """Exercise the all-model assessment only when its global matrix changes.

    ``model assess`` deliberately joins every installed provider, realization,
    and campaign-owned adapter advertisement.  That breadth is valuable for a
    catalog release, but it is not a unit-level dependency of an isolated
    vehicle plug-in change.
    """

    providers = plugins.build_mission_composition_provider_registry()
    assessment = build_model_automation_assessment(
        providers,
        plugins.build_controller_tuning_campaign_registry(),
        family_adapters=plugins.build_family_adapter_registry(),
        local_controller_screens=plugins.build_local_controller_screen_advertisement_registry(),
    )
    advertised_model_keys = {
        (provider.metadata.id, model.id)
        for provider in providers.providers
        for model in provider.list_models()
    }

    assert assessment["schema"] == "taoryx.model-automation-assessment/v1"
    assert assessment["rslqr"]["status"] == "deferred"  # type: ignore[index]
    advertisement_summary = assessment["advertisement_readiness_summary"]
    assert advertisement_summary["status"] == "complete"
    assert advertisement_summary["model_count"] == len(advertised_model_keys)
    assert advertisement_summary["complete_model_count"] == len(advertised_model_keys)
    assert advertisement_summary["incomplete_models"] == []
    readiness_summary = assessment["controller_readiness_summary"]
    assert readiness_summary["status"] == "complete"
    assert readiness_summary["external_control_gaps"] == []
    models = [
        model
        for provider in assessment["providers"]  # type: ignore[index]
        for model in provider["models"]  # type: ignore[index]
    ]
    assert {
        (str(provider["id"]), str(model["id"]))
        for provider in assessment["providers"]  # type: ignore[index]
        for model in provider["models"]  # type: ignore[index]
    } == advertised_model_keys
    assert all(model["advertisement"]["status"] == "complete" for model in models)  # type: ignore[index]
    assert all(model["advertisement"]["plan_exercise"]["status"] == "pass" for model in models)  # type: ignore[index]
    assert all(
        set(model["advertisement"]["plan_exercise"]["sections"])
        == {
            "data_contract",
            "controller_automation",
            "execution_advertisement",
            "maturity_advertisement",
            "focused_endpoint_verification",
            "navigation_automation",
            "mode_automation",
            "segment_automation",
        }
        for model in models
    )  # type: ignore[index]
    assert all(model["generic_authoring"]["status"] == "advertised" for model in models)  # type: ignore[index]
    externally_controllable_fidelity_count = sum(
        realization["status"] == "available"
        and realization["control"]["status"] == "available"
        and fidelity["controller_automation"]["status"] in {"campaign_registered", "campaign_registration_required"}
        for model in models
        for realization in model["realizations"]
        for fidelity in realization["fidelities"]
    )
    externally_tunable_fidelity_count = sum(
        realization["status"] == "available"
        and realization["control"]["status"] == "available"
        and fidelity["controller_automation"]["status"] == "campaign_registered"
        for model in models
        for realization in model["realizations"]
        for fidelity in realization["fidelities"]
    )
    assert readiness_summary["externally_controllable_fidelity_count"] == externally_controllable_fidelity_count
    assert readiness_summary["externally_tunable_fidelity_count"] == externally_tunable_fidelity_count
    for model in models:
        for realization in model["realizations"]:
            if realization["input_realization"] in {"uncontrolled", "source_replay"}:
                continue
            if realization["control"]["status"] != "internally_generated":
                continue
            for fidelity in realization["fidelities"]:
                automation = fidelity["controller_automation"]
                expected_status = "provider_managed_with_campaign" if automation["campaign_ids"] else "provider_managed"
                assert automation["status"] == expected_status

    hummingbird = next(item for item in models if item["id"] == "hummingbird")
    pseudo = next(item for item in hummingbird["realizations"] if item["id"] == "pseudo_6dof")
    tuning = next(item for item in pseudo["fidelities"] if item["id"] == "pseudo_6dof")
    assert pseudo["control"]["channel_count"] > 0
    assert tuning["controller_automation"]["status"] == "campaign_registered"
    assert tuning["controller_automation"]["campaign_ids"] == ["hummingbird-pseudo-hover-attitude-v1"]
    assert pseudo["lowering"]["status"] == "not_advertised"

    physical = next(item for item in hummingbird["realizations"] if item["id"] == "rigid_body_6dof_surface_allocated")
    physical_tuning = physical["fidelities"][0]["controller_automation"]
    assert physical_tuning["status"] == "provider_managed_with_campaign"
    assert physical_tuning["campaign_ids"] == [
        "hummingbird-source-rotor-local-lqi-v1",
        "hummingbird-source-rotor-vertical-lqi-v1",
    ]

    a320 = next(item for item in models if item["id"] == "a320_openap_3dof")
    a320_point = next(item for item in a320["realizations"] if item["id"] == "point_mass_3dof")
    assert a320_point["fidelities"][0]["controller_automation"]["status"] == "campaign_registered"
    assert a320_point["fidelities"][0]["controller_automation"]["campaign_ids"] == ["a320-point-cruise-performance-lqr-v1"]

    x8 = next(item for item in models if item["id"] == "skywalker_x8")
    x8_point = next(item for item in x8["realizations"] if item["id"] == "point_mass_3dof")
    x8_point_fidelity = x8_point["fidelities"][0]
    assert x8_point_fidelity["controller_automation"]["status"] == "campaign_registered"
    assert x8_point_fidelity["controller_automation"]["campaign_ids"] == ["x8-language-backed-guidance-local-lqi-v1"]
    assert x8_point_fidelity["family_adapter"]["fidelity_supported"] is False
    tuning_adapter = x8_point_fidelity["controller_automation"]["tuning_adapter"]
    assert tuning_adapter["status"] == "campaign_owned"
    assert tuning_adapter["campaign_ids"] == ["x8-language-backed-guidance-local-lqi-v1"]
    adapters = tuning_adapter["adapters"]
    assert len(adapters) == 1
    descriptor = adapters[0]["descriptor"]
    assert descriptor["adapter_id"] == "taoryx.fixed_wing.language_backed_guidance.v1"
    assert descriptor["tier"] == "point_mass_3dof"
    assert [channel["name"] for channel in descriptor["control_channels"]] == [
        "speed_command_m_s",
        "flight_path_angle_command_deg",
        "heading_command_deg",
    ]
    operations = {item["operation"]: item for item in adapters[0]["operations"]}
    assert operations["trim"]["usable"] is True
    assert operations["linearize"]["usable"] is True
    assert not any("family adapter does not support" in blocker for blocker in x8_point["blockers"])
    x8_pseudo = next(item for item in x8["realizations"] if item["id"] == "pseudo_6dof")
    assert x8_pseudo["fidelities"][0]["controller_automation"]["campaign_ids"] == ["x8-language-backed-pseudo-guidance-local-lqi-v1"]

    b747 = next(item for item in models if item["id"] == "b747")
    b747_point = next(item for item in b747["realizations"] if item["id"] == "point_mass_3dof")
    assert b747_point["fidelities"][0]["controller_automation"]["status"] == "campaign_registered"
    assert b747_point["fidelities"][0]["controller_automation"]["campaign_ids"] == ["b747-language-backed-guidance-local-lqi-v1"]
    b747_pseudo = next(item for item in b747["realizations"] if item["id"] == "pseudo_6dof")
    assert b747_pseudo["fidelities"][0]["controller_automation"]["campaign_ids"] == ["b747-language-backed-pseudo-guidance-local-lqi-v1"]

    f16 = next(item for item in models if item["id"] == "f16_s119")
    f16_point = next(item for item in f16["realizations"] if item["id"] == "point_mass_3dof")
    f16_pseudo = next(item for item in f16["realizations"] if item["id"] == "pseudo_6dof")
    assert f16_point["fidelities"][0]["controller_automation"]["campaign_ids"] == ["f16-point-source-trim-translation-v1"]
    assert f16_pseudo["fidelities"][0]["controller_automation"]["campaign_ids"] == ["f16-pseudo-source-trim-attitude-v1"]
    f16_surface = next(item for item in f16["realizations"] if item["id"] == "rigid_body_6dof_surface_allocated")
    f16_surface_tuning = f16_surface["fidelities"][0]["controller_automation"]
    assert f16_surface_tuning["status"] == "provider_managed_with_campaign"
    assert f16_surface_tuning["campaign_ids"] == [
        "f16-source-surface-local-lqr-v1",
        "f16-source-surface-schedule-lqr-v1",
        "f16-source-surface-local-lqi-v1",
        "f16-source-surface-schedule-lqi-v1",
    ]

    x15 = next(item for item in models if item["id"] == "x15")
    x15_direct = next(item for item in x15["realizations"] if item["id"] == "rigid_body_6dof_direct_wrench")
    assert x15_direct["fidelities"][0]["controller_automation"]["campaign_ids"] == [
        "x15-source-release-direct-wrench-v1",
        "x15-source-release-direct-wrench-lqi-v1",
    ]

    hl20 = next(item for item in models if item["id"] == "hl20_mod_k")
    hl20_direct = next(item for item in hl20["realizations"] if item["id"] == "rigid_body_6dof_direct_wrench")
    assert hl20_direct["fidelities"][0]["controller_automation"]["campaign_ids"] == [
        "hl20-source-subsonic-direct-wrench-v1",
        "hl20-source-subsonic-direct-wrench-lqi-v1",
    ]

    summary = build_model_automation_readiness_summary(assessment)
    assert summary["schema"] == "taoryx.model-automation-readiness-summary/v1"
    assert summary["controller_readiness_summary"]["external_control_gaps"] == []  # type: ignore[index]
    summary_x8 = next(
        item
        for provider in summary["providers"]  # type: ignore[index]
        for item in provider["models"]
        if item["id"] == "skywalker_x8"
    )
    summary_x8_point = next(item for item in summary_x8["realizations"] if item["id"] == "point_mass_3dof")
    summary_x8_point_tuning = summary_x8_point["fidelities"][0]
    assert summary_x8_point_tuning["campaign_ids"] == ["x8-language-backed-guidance-local-lqi-v1"]
    assert summary_x8_point_tuning["tuning_adapter_ids"] == ["taoryx.fixed_wing.language_backed_guidance.v1"]
    ####


@pytest.mark.slow
@pytest.mark.matrix
def test_model_automation_assessment_fails_closed_when_common_plan_omits_advertisement_sections(
    monkeypatch: pytest.MonkeyPatch,
    plugins: PluginCatalog,
) -> None:
    """The advertisement flag records an exercised common seam, not a default."""

    def incomplete_plan(*args: object, **_kwargs: object) -> dict[str, object]:
        return {
            "status": "ready_to_author",
            "selection": {
                "provider_id": args[2],
                "model_id": args[3],
                "fidelity": "test",
                "sources": {},
            },
        }
        ####

    monkeypatch.setattr("taoryx.model_authoring.build_model_authoring_plan", incomplete_plan)
    assessment = build_model_automation_assessment(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        family_adapters=plugins.build_family_adapter_registry(),
        local_controller_screens=plugins.build_local_controller_screen_advertisement_registry(),
    )
    models = [
        model
        for provider in assessment["providers"]  # type: ignore[index]
        for model in provider["models"]  # type: ignore[index]
    ]

    assert all(model["advertisement"]["status"] == "incomplete" for model in models)  # type: ignore[index]
    exercise = models[0]["advertisement"]["plan_exercise"]  # type: ignore[index]
    assert exercise["status"] == "fail"
    assert "data_contract: missing mapping" in exercise["findings"]
    advertisement_summary = assessment["advertisement_readiness_summary"]
    assert advertisement_summary["status"] == "incomplete"
    assert advertisement_summary["model_count"] == len(models)
    assert advertisement_summary["complete_model_count"] == 0
    assert advertisement_summary["incomplete_models"] == [
        {"provider_id": str(provider["id"]), "model_id": str(model["id"])}
        for provider in assessment["providers"]  # type: ignore[index]
        for model in provider["models"]  # type: ignore[index]
    ]
    ####


def test_model_plan_exposes_exact_selected_endpoint_maturity(
    plugins: PluginCatalog,
) -> None:
    """Plans retain declared batch/step boundaries without inventing runtime support."""

    providers = plugins.build_mission_composition_provider_registry()
    campaigns = plugins.build_controller_tuning_campaign_registry()
    adapters = plugins.build_family_adapter_registry()

    a320 = build_model_authoring_plan(
        providers,
        campaigns,
        "taoryx.registry.mission-composition",
        "a320_openap_3dof",
        family_adapters=adapters,
        fidelity="pseudo_6dof",
        realization_id="pseudo_6dof",
        mission_template_id="powered_fixed_wing_racetrack_v1",
    )
    a320_execution = a320["execution_advertisement"]
    assert a320_execution["status"] == "runnable"
    assert a320_execution["endpoint_maturity"] == "batch_and_step_ready"
    assert a320_execution["available_operations"] == ["validate", "batch", "step"]
    a320_maturity = a320["maturity_advertisement"]
    assert a320_maturity["maturity"] == "M4"
    assert a320_maturity["next_gate"] == "expanded_operating_point_schedule_and_source_grounded_surface_path"
    a320_endpoint = a320["focused_endpoint_verification"]
    assert a320_endpoint["selected_endpoint_status"] == "different_declared_endpoint"
    assert a320_endpoint["selected_endpoint_ids"] == []
    assert a320_endpoint["endpoints"] == [
        {
            "id": "a320-pseudo6dof-native-coordinate-lqi",
            "kind": "vehicle_composition",
            "maturity_record_id": "a320_openap_3dof",
            "matches_selected_mission_and_fidelity": False,
            "match_scope": "provider_id, model_id, mission_id, fidelity",
            "command": "taoryx vehicle verify a320-pseudo6dof-native-coordinate-lqi",
            "execute_command": "taoryx vehicle verify a320-pseudo6dof-native-coordinate-lqi --execute",
        }
    ]

    nesc = build_model_authoring_plan(
        providers,
        campaigns,
        "taoryx.registry.mission-composition",
        "reference_nesc_two_stage_rocket",
        family_adapters=adapters,
        fidelity="pseudo_6dof",
        realization_id="pseudo_6dof",
        mission_template_id="staged_rocket_launch_target_state_v1",
    )
    nesc_execution = nesc["execution_advertisement"]
    assert nesc_execution["status"] == "runnable"
    assert nesc_execution["endpoint_maturity"] == "batch_and_step_ready"
    assert nesc_execution["available_operations"] == ["validate", "batch", "step"]
    assert nesc_execution["blocked_operations"] == []
    assert nesc["maturity_advertisement"]["maturity"] == "M4"
    assert nesc["focused_endpoint_verification"]["selected_endpoint_status"] == "matching_endpoint_available"
    assert nesc["focused_endpoint_verification"]["selected_endpoint_ids"] == [
        "nesc-source-history-replay-pseudo6dof"
    ]

    for model_id, mission_template_id in (
        ("x15", "x15_staged_booster_reachability_v1"),
        ("hl20_mod_k", "hl20_source_booster_release_replay_v1"),
        ("tumbling_body", "tumbling_body_release_damping_impact_v1"),
    ):
        plan = build_model_authoring_plan(
            providers,
            campaigns,
            "taoryx.registry.mission-composition",
            model_id,
            family_adapters=adapters,
            fidelity="pseudo_6dof",
            realization_id="pseudo_6dof",
            mission_template_id=mission_template_id,
        )
        execution = plan["execution_advertisement"]

        assert execution["status"] == "runnable"
        assert execution["endpoint_maturity"] == "batch_and_step_ready"
        assert execution["available_operations"] == ["validate", "batch", "step"]
        assert execution["blocked_operations"] == []
    ####


@pytest.mark.parametrize(
    ("provider_id", "model_id", "fidelity", "realization_id", "mission_template_id", "endpoint_id"),
    (
        (
            "taoryx.simple-aero.mission-composition",
            "simple_aero",
            "point_mass_3dof",
            "fixed_ld_point_mass",
            "fixed_ld_baseline",
            "simple-aero-fixed-ld-batch",
        ),
        (
            "taoryx.registry.mission-composition",
            "dual_launch_glider",
            "point_mass_3dof",
            "generated_native_problem",
            "attached_booster_waypoint",
            "dual-launch-attached-booster-batch",
        ),
        (
            "taoryx.reference.mission-composition",
            "reference_ballistic_3dof",
            "point_mass_3dof",
            "analytical_point_mass",
            "reference_ballistic_3dof_repeatable_sequence_v1",
            "reference-ballistic-3dof-batch",
        ),
        (
            "taoryx.reference.mission-composition",
            "reference_constant_velocity_waypoint_3dof",
            "point_mass_3dof",
            "analytical_point_mass",
            "reference_constant_velocity_waypoint_3dof_repeatable_sequence_v1",
            "reference-waypoint-3dof-batch",
        ),
        (
            "taoryx.debug.mission-composition-contract-probe",
            "contract_probe_vehicle",
            "medium",
            "medium",
            "deployment_walkthrough",
            "debug-contract-probe-batch",
        ),
    ),
)
def test_model_plan_advertises_exact_workflow_endpoint_verifier(
    plugins: PluginCatalog,
    provider_id: str,
    model_id: str,
    fidelity: str,
    realization_id: str,
    mission_template_id: str,
    endpoint_id: str,
) -> None:
    """Nonphysical workflows expose their provider-owned proof command directly."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        provider_id,
        model_id,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity=fidelity,
        realization_id=realization_id,
        mission_template_id=mission_template_id,
    )

    endpoint = plan["focused_endpoint_verification"]
    assert endpoint["status"] == "available"
    assert endpoint["selected_endpoint_status"] == "matching_endpoint_available"
    assert endpoint["selected_endpoint_ids"] == [endpoint_id]
    assert endpoint["endpoints"] == [
        {
            "id": endpoint_id,
            "kind": "mission_workflow",
            "maturity_record_id": model_id,
            "matches_selected_configuration": True,
            "match_scope": "provider_id (or declared alias), model_id, mission_template_id, fidelity, realization_id",
            "command": f"taoryx model verify {endpoint_id}",
            "execute_command": f"taoryx model verify {endpoint_id} --execute",
        }
    ]
    ####


@pytest.mark.slow
@pytest.mark.matrix
def test_model_cli_lists_plans_and_scaffolds_installed_models(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    inventory = tmp_path / "models.json"
    assessment = tmp_path / "assessment.json"
    assessment_summary = tmp_path / "assessment-summary.json"
    plan = tmp_path / "hummingbird-plan.json"
    draft = tmp_path / "tumbling-body.yaml"
    prepared_path = tmp_path / "tumbling-body.prepared.json"
    ballistic_prepared_path = tmp_path / "ballistic.prepared.json"
    ballistic_response_path = tmp_path / "ballistic.response.json"

    assert main(["model", "list", "--output", str(inventory)]) == 0
    assert main(["model", "assess", "--output", str(assessment)]) == 0
    assert main(["model", "assess", "--summary", "--output", str(assessment_summary)]) == 0
    assert (
        main(
            [
                "model",
                "plan",
                "taoryx.registry.mission-composition",
                "hummingbird",
                "--fidelity",
                "pseudo_6dof",
                "--output",
                str(plan),
            ]
        )
        == 0
    )
    assert (
        main(
            [
                "model",
                "scaffold",
                "taoryx.registry.mission-composition",
                "tumbling_body",
                "--output",
                str(draft),
            ]
        )
        == 0
    )

    inventory_payload = json.loads(inventory.read_text(encoding="utf-8"))
    assessment_payload = json.loads(assessment.read_text(encoding="utf-8"))
    assessment_summary_payload = json.loads(assessment_summary.read_text(encoding="utf-8"))
    plan_payload = json.loads(plan.read_text(encoding="utf-8"))
    editable = load_model_authoring_draft(draft)
    completed = editable.model_copy(
        update={
            "values": {
                "initialization": select_variant(
                    "atmospheric_release",
                    altitude_m=1000.0,
                    speed_m_s=120.0,
                    body_rates_rad_s=[0.1, 0.2, 0.3],
                    area_policy="orientation_averaged_projected_area",
                ),
                "segments": sequence_template(
                    "tumbling_body_release_damping_impact_v1",
                    {"duration_s": 10.0},
                    {"impact_plane_altitude_m": 0.0},
                ),
            }
        }
    )
    write_model_authoring_draft(completed, draft)
    assert main(["model", "compile", str(draft), "--output", str(prepared_path)]) == 0

    reference = discover_plugins(include_external=False).build_mission_composition_provider_registry().provider("taoryx.reference.mission-composition")
    ballistic = author_configuration(
        reference,
        configuration_id="cli-ballistic",
        model_id="reference_ballistic_3dof",
        fidelity="point_mass_3dof",
        realization_id="analytical_point_mass",
        values={
            "initialization": select_variant(
                "launch_state",
                altitude_m=1000.0,
                speed_m_s=150.0,
                heading_deg=0.0,
                flight_path_angle_deg=20.0,
            ),
            "segments": custom_sequence(segment_occurrence("ballistic_coast", duration_s=2.0)),
        },
    )
    ballistic_prepared_path.write_text(ballistic.model_dump_json(by_alias=True), encoding="utf-8")
    assert (
        main(
            [
                "model",
                "run",
                "taoryx.reference.mission-composition",
                str(ballistic_prepared_path),
                "--request-id",
                "cli-ballistic-run",
                "--output",
                str(ballistic_response_path),
            ]
        )
        == 0
    )

    model_counts = {
        provider["metadata"]["id"]: len(provider["models"])
        for provider in inventory_payload["providers"]
    }
    assert inventory_payload["plugin_revisions"]
    assert all(item["version"] and len(item["fingerprint"]) == 64 for item in inventory_payload["plugin_revisions"])
    for provider in inventory_payload["providers"]:
        metadata = provider["metadata"]
        revision = provider["revision"]
        assert metadata["version"]
        assert len(metadata["metadata_fingerprint"]) == 64
        assert revision["provider_id"] == metadata["id"]
        assert revision["version"] == metadata["version"]
        assert revision["metadata_fingerprint"] == metadata["metadata_fingerprint"]
        assert len(revision["model_catalog_fingerprint"]) == 64
        for model in provider["models"]:
            model_revision = model["revision"]
            assert model["version"]
            assert len(model["metadata_fingerprint"]) == 64
            assert model_revision["model_id"] == model["id"]
            assert model_revision["version"] == model["version"]
            assert model_revision["metadata_fingerprint"] == model["metadata_fingerprint"]
            assert len(model_revision["configuration_schema_fingerprint"]) == 64
            assert len(model_revision["output_schema_fingerprint"]) == 64
        ####
    ####
    # The core inventory stays fixed while separately installed optional
    # distributions, such as source-bound CADAC, legitimately add providers.
    assert {
        "taoryx.registry.mission-composition": 11,
        "taoryx.reference.mission-composition": 2,
        "taoryx.debug.mission-composition-contract-probe": 1,
    }.items() <= model_counts.items()
    model_count = sum(model_counts.values())
    assert assessment_payload["schema"] == "taoryx.model-automation-assessment/v1"
    assert assessment_payload["rslqr"]["status"] == "deferred"
    assert assessment_payload["advertisement_readiness_summary"] == {
        "status": "complete",
        "model_count": model_count,
        "complete_model_count": model_count,
        "incomplete_models": [],
        "claim_boundary": (
            "Complete means the common plan/scaffold join consumed each typed advertisement. It does not execute a "
            "runtime, controller campaign, or qualification procedure."
        ),
    }
    assert assessment_payload["controller_readiness_summary"]["status"] == "complete"
    assert assessment_payload["controller_readiness_summary"]["external_control_gaps"] == []
    assert assessment_summary_payload["schema"] == "taoryx.model-automation-readiness-summary/v1"
    assert assessment_summary_payload["advertisement_readiness_summary"]["status"] == "complete"
    assert assessment_summary_payload["controller_readiness_summary"]["status"] == "complete"
    assert plan_payload["controller_automation"]["status"] == "campaign_registered"
    assert plan_payload["controller_automation"]["tuning_adapter"]["status"] == "campaign_owned"
    assert plan_payload["execution_advertisement"]["status"] == "runnable"
    assert plan_payload["execution_advertisement"]["endpoint_maturity"] == "batch_and_step_ready"
    assert json.loads(prepared_path.read_text(encoding="utf-8"))["configuration"]["model_id"] == "tumbling_body"
    ballistic_response = json.loads(ballistic_response_path.read_text(encoding="utf-8"))
    assert ballistic_response["kind"] == "trajectory"
    assert ballistic_response["result"]["request_id"] == "cli-ballistic-run"
    assert ballistic_response["result"]["primary_model_id"] == "reference_ballistic_3dof"
    assert "inputs_required" in capsys.readouterr().out
    ####
