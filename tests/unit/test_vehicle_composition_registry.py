from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path
from typing import Any, cast

import pytest
import taoryx.mission_composition_maturity as mission_composition_maturity
import yaml
from taoryx.a320_reduced_execution import execute_a320_reduced_composition
from taoryx.mission_composition_maturity import build_mission_composition_maturity_report

import taoryx.runtime.cli as runtime_cli
import taoryx.vehicle_execution_preflight as execution_preflight
import taoryx.vehicle_execution_witnesses as execution_witnesses
from taoryx.family_adapter_registry import AdapterRegistrationError
from taoryx.fidelity_contracts import CANONICAL_FIDELITY_TIERS
from taoryx.language_backed_execution import execute_powered_fixed_wing_composition
from taoryx.parameter_value_spaces import (
    load_parameter_value_space_catalog,
    validate_parameter_value_space_coverage,
)
from taoryx.runtime.cli import main
from taoryx.trajectory.evaluation import TrajectoryEvaluation
from taoryx.vehicle_composition import (
    CompositionValue,
    MissionGraphNodeSelection,
    MissionGraphSelection,
    MissionTransitionSelection,
    VehicleCompositionError,
    compile_vehicle_composition,
    load_vehicle_composition_request,
)
from taoryx.vehicle_composition_registry import (
    ParameterSpec,
    VariantParameterBinding,
    VehicleCompositionRegistry,
    _authoring_parameter_dict,
    build_vehicle_composition_topology_report,
    load_resolved_vehicle_composition_catalog,
    mission_graph_execution_contract,
    mission_semantic_translator_id,
)
from taoryx.vehicle_execution_preflight import preflight_vehicle_composition
from taoryx.vehicle_runtime_lowering import build_vehicle_runtime_adapter_registry, lower_vehicle_composition

ROOT = Path(__file__).resolve().parents[2]


def test_composition_registry_covers_every_unified_family() -> None:
    catalog = load_resolved_vehicle_composition_catalog()

    assert len(catalog.vehicles) == 9
    assert {item.family.family_id for item in catalog.vehicles} == {
        "skywalker_x8",
        "b747",
        "a320_openap_3dof",
        "f16_s119",
        "x15",
        "hummingbird",
        "hl20_mod_k",
        "reference_nesc_two_stage_rocket",
        "tumbling_body",
    }
    ####


def test_topology_report_covers_every_public_composition_surface(
    capsys: pytest.CaptureFixture[str],
) -> None:
    report = build_vehicle_composition_topology_report()

    assert report["schema"] == "taoryx.vehicle-composition-topology-report/v1alpha1"
    assert report["status"] == "pass"
    assert report["family_count"] >= 9
    assert report["parameter_count"] > 0
    assert report["interface_channel_count"] > 0
    assert report["finding_count"] == 0
    assert report["truth_objective_findings"] == []
    resource_coverage = cast(dict[str, dict[str, object]], report["resource_channel_coverage"])
    assert "resources.mass.total" in resource_coverage
    assert "resources.battery.fraction_remaining" in resource_coverage
    mass_bindings = cast(list[dict[str, object]], resource_coverage["resources.mass.total"]["bindings"])
    assert any(binding["family_id"] == "skywalker_x8" for binding in mass_bindings)
    battery_bindings = cast(list[dict[str, object]], resource_coverage["resources.battery.fraction_remaining"]["bindings"])
    assert all(binding["family_id"] == "hummingbird" for binding in battery_bindings)
    maturity = cast(dict[str, int], report["parameter_contract_maturity"])
    assert maturity["effective_hard_bound_count"] > 0
    assert maturity["provenance_count"] > 0
    assert maturity["runtime_backed_variant_count"] == 2
    assert report["parameter_value_space_catalog_status"] == "pass"
    assert report["interface_channel_value_space_catalog_status"] == "pass"
    assert report["truth_objective_channel_value_space_catalog_status"] == "pass"
    assert all(item["status"] == "pass" for item in cast(list[dict[str, object]], report["vehicles"]))
    a320 = next(item for item in cast(list[dict[str, object]], report["vehicles"]) if item["family_id"] == "a320_openap_3dof")
    assert all(item["status"] == "pass" for item in cast(list[dict[str, object]], a320["interfaces"]))
    a320_maturity = cast(dict[str, int], a320["parameter_contract_maturity"])
    assert a320_maturity["runtime_backed_variant_count"] == 1
    assert a320_maturity["derivation_count"] >= 1
    a320_descriptor = cast(dict[str, object], load_resolved_vehicle_composition_catalog().vehicle("a320_openap_3dof").as_dict())
    assert a320_descriptor["parameter_contract_maturity"] == a320_maturity
    a320_worklist = load_resolved_vehicle_composition_catalog().vehicle("a320_openap_3dof").authoring_worklist_dict()
    assert a320_worklist["parameter_contract_maturity"] == a320_maturity
    a320_variants = cast(dict[str, object], a320_worklist["variant_worklist"])
    assert a320_variants["status"] == "runnable"
    a320_variant_records = cast(list[dict[str, object]], a320_variants["variants"])
    assert a320_variant_records[0]["runtime_binding"] == {
        "adapter_id": "taoryx.fixed_wing.openap.v1",
        "input_path": "A320OpenAPOperatingPoint.mass_kg",
    }
    assert "retrim and requalify" in cast(list[str], a320_variant_records[0]["next_steps"])[1]

    assert main(["vehicle", "topology-report"]) == 0
    payload = cast(dict[str, Any], json.loads(capsys.readouterr().out))
    assert payload["schema"] == report["schema"]
    assert payload["status"] == "pass"
    ####


def test_shared_parameter_spec_never_invents_defaults_or_projection_policy() -> None:
    absent = ParameterSpec(id="target_speed_m_s", canonical_unit="m/s", description="Declared target speed.")
    descriptor = absent.public_dict()

    assert descriptor["default"] is None
    assert descriptor["default_declared"] is False
    assert descriptor["projection_policy"] == "reject_invalid"
    assert descriptor["qualified_lower"] is None
    assert descriptor["safe_extended_upper"] is None

    bounded = ParameterSpec(
        id="payload_mass_kg",
        canonical_unit="kg",
        description="Bounded payload mass.",
        default_value=25.0,
        default_declared=True,
        hard_lower=0.001,
        hard_upper=50.0,
        qualified_lower=0.001,
        qualified_upper=40.0,
        transform="log",
        coupling_group="operating_mass",
        derivation="recompute total mass",
        invalidations=("retrim:mass_kg",),
        provenance="source-backed loadout envelope",
    )
    bounded_descriptor = bounded.public_dict()

    assert bounded_descriptor["default"] == 25.0
    assert bounded_descriptor["hard_upper"] == 50.0
    assert bounded_descriptor["qualified_upper"] == 40.0
    assert bounded_descriptor["transform"] == "log"
    assert bounded_descriptor["coupling_group"] == "operating_mass"
    assert bounded_descriptor["invalidations"] == ["retrim:mass_kg"]

    declared_enum = ParameterSpec(
        id="operating_point_id",
        description="Source-defined operating point identity.",
        value_type_declared="enum",
    )
    assert declared_enum.value_type == "enum"
    assert declared_enum.public_dict()["value_type_source"] == "declared"
    assert declared_enum.public_dict()["value_space_source"] == "declared_value_type"
    assert declared_enum.value_space.topology == "finite_set"

    with pytest.raises(ValueError, match="null default"):
        ParameterSpec(id="bad_default", description="Invalid default.", default_declared=True)
    with pytest.raises(ValueError, match="inverted hard bounds"):
        ParameterSpec(id="bad_bounds", description="Invalid bounds.", hard_lower=2.0, hard_upper=1.0)
    with pytest.raises(ValueError, match="qualified upper bound above hard validity"):
        ParameterSpec(
            id="bad_qualified",
            description="Invalid evidence range.",
            hard_lower=0.0,
            hard_upper=1.0,
            qualified_upper=1.1,
        )
    with pytest.raises(ValueError, match="default is above hard validity"):
        ParameterSpec(
            id="bad_default_bound",
            description="Invalid bounded default.",
            default_value=1.1,
            default_declared=True,
            hard_lower=0.0,
            hard_upper=1.0,
        )
    with pytest.raises(ValueError, match="unit norm"):
        ParameterSpec(
            id="quaternion_wxyz",
            description="Invalid attitude default.",
            default_value=[1.1, 0.0, 0.0, 0.0],
            default_declared=True,
        )

    authoring = _authoring_parameter_dict(bounded)
    assert authoring["authoring_value"] == 25.0
    assert authoring["authoring_value_source"] == "declared_recommendation"
    assert authoring["authoring_value_rule"] == "optional_declared_recommendation"
    assert _authoring_parameter_dict(absent)["authoring_value_source"] == "not_declared"
    ####


def test_parameter_and_runtime_variant_transforms_match_their_declared_topology() -> None:
    log_parameter = ParameterSpec(
        id="positive_mass_kg",
        canonical_unit="kg",
        description="Strictly positive mass for log-space search.",
        hard_lower=0.001,
        hard_upper=10.0,
        transform="log",
    )
    logit_parameter = ParameterSpec(
        id="throttle_fraction",
        description="Bounded normalized control input.",
        hard_lower=0.0,
        hard_upper=1.0,
        transform="logit",
    )

    assert log_parameter.value_space.topology == "positive_half_line"
    assert logit_parameter.value_space.topology == "unit_interval"

    with pytest.raises(ValueError, match="strictly positive hard_lower"):
        ParameterSpec(
            id="zero_mass_log",
            description="A log transform cannot represent zero.",
            hard_lower=0.0,
            hard_upper=10.0,
            transform="log",
        )
    with pytest.raises(ValueError, match=r"hard bounds \[0, 1\]"):
        ParameterSpec(
            id="bad_fraction_logit",
            description="A logit must carry unit-interval bounds.",
            hard_lower=0.0,
            hard_upper=2.0,
            transform="logit",
        )
    with pytest.raises(ValueError, match="no simplex membership contract"):
        ParameterSpec(
            id="mass_fraction",
            description="A scalar cannot represent a simplex allocation.",
            transform="simplex",
        )

    def binding(*, hard_lower: float | None = None, hard_upper: float | None = None, transform: str) -> VariantParameterBinding:
        return VariantParameterBinding.model_validate(
            {
                "id": "mass_modifier",
                "canonical_unit": "kg",
                "target_initialization_id": "airborne_trim",
                "target_parameter_id": "mass_kg",
                "status": "planned",
                "hard_lower": hard_lower,
                "hard_upper": hard_upper,
                "transform": transform,
                "provenance": "unit-test fixture",
                "description": "Bounded scalar variant fixture.",
            }
        )
        ####

    log_binding = binding(hard_lower=0.001, hard_upper=10.0, transform="log")
    logit_binding = binding(hard_lower=0.0, hard_upper=1.0, transform="logit")

    assert log_binding.value_space.topology == "positive_half_line"
    assert logit_binding.value_space.topology == "unit_interval"
    with pytest.raises(ValueError, match="requires an explicit value_space_profile"):
        VariantParameterBinding.model_validate(
            {
                "id": "runnable_mass_modifier",
                "canonical_unit": "kg",
                "target_initialization_id": "airborne_trim",
                "target_parameter_id": "mass_kg",
                "status": "runnable",
                "compatible_fidelities": ["point_mass_3dof"],
                "runtime_adapter_id": "taoryx.test.adapter.v1",
                "runtime_input_path": "model.mass_kg",
                "hard_lower": 0.001,
                "hard_upper": 10.0,
                "transform": "log",
                "coupling_group": "mass",
                "resource_derivation": "runtime_adapter_owned",
                "derivation_claim_boundary": "Test-only runtime input binding.",
                "provenance": "unit-test fixture",
                "description": "Runnable bounded scalar variant fixture.",
            }
        )
    with pytest.raises(ValueError, match="unsupported by the scalar runtime variant contract"):
        binding(transform="categorical")
    with pytest.raises(ValueError, match="unsupported by the scalar runtime variant contract"):
        binding(transform="simplex")
    ####


def test_x8_composition_exposes_four_tiers_and_racetrack_recipe() -> None:
    x8 = cast(dict[str, Any], load_resolved_vehicle_composition_catalog().vehicle("skywalker_x8").as_dict())

    assert set(x8["fidelities"]) == {
        "point_mass_3dof",
        "pseudo_6dof",
        "rigid_body_6dof_direct_wrench",
        "rigid_body_6dof_surface_allocated",
    }
    assert x8["fidelities"]["rigid_body_6dof_direct_wrench"]["control_realization"] == "direct_wrench"
    assert x8["fidelities"]["rigid_body_6dof_surface_allocated"]["control_realization"] == "surface_allocated"
    assert x8["interfaces"]["pseudo_6dof"]["available_authority_profiles"] == [
        "kinematic_guidance",
        "native_control_bridge",
    ]
    assert x8["interfaces"]["pseudo_6dof"]["available_observation_profiles"] == ["truth_debug"]
    airborne_trim = next(item for item in x8["initialization_contracts"] if item["id"] == "airborne_trim")
    heading = next(item for item in airborne_trim["parameters"] if item["id"] == "heading_deg")
    assert heading["value_space"]["topology"] == "periodic_circle"
    assert heading["value_space_source"] == "parameter_value_space_catalog"
    fly_by_turn = next(item for item in x8["segment_contracts"] if item["id"] == "fly_by_turn")
    turn = next(item for item in fly_by_turn["parameters"] if item["id"] == "turn_direction")
    assert turn["options"] == ["left", "right"]
    racetrack = next(item for item in x8["mission_templates"] if item["id"] == "powered_fixed_wing_racetrack_v1")
    assert racetrack["segment_sequence"][-1] == "terminal_state_gate"
    graph = racetrack["graph"]
    assert graph["status"] == "linear_sequence_only"
    assert graph["entry_instance_id"] == "01-trim_hold"
    assert graph["nodes"][2]["success_transition"]["target_instance_id"] == "04-descent_level_gate"
    assert graph["nodes"][-1]["success_transition"] is None
    ####


def test_registry_parameters_have_explicit_value_space_catalog_entries() -> None:
    """Coordinates, angles, and attitudes must not be guessed from suffixes."""

    catalog = load_parameter_value_space_catalog()
    resolved = load_resolved_vehicle_composition_catalog()
    parameters = [
        parameter
        for vehicle in resolved.vehicles
        for contract in (*vehicle.declaration.initialization_contracts, *vehicle.declaration.segment_contracts)
        for parameter in contract.parameters
    ]

    assert {parameter.id for parameter in parameters} <= set(catalog)
    assert all(parameter.public_dict()["value_space_source"] == "parameter_value_space_catalog" for parameter in parameters)
    assert catalog["north_m"].value_space.topology == "euclidean"
    assert catalog["east_m"].value_space.topology == "euclidean"
    assert catalog["pad_east_m"].value_space.topology == "euclidean"
    assert catalog["heading_deg"].value_space.topology == "periodic_circle"
    assert catalog["quaternion_wxyz"].value_space.topology == "rotation_group_so3"
    x8_parameters = {
        parameter.id: parameter.public_dict()
        for contract in (
            *resolved.vehicle("skywalker_x8").declaration.initialization_contracts,
            *resolved.vehicle("skywalker_x8").declaration.segment_contracts,
        )
        for parameter in contract.parameters
    }
    assert x8_parameters["north_m"]["frame"] == "NED"
    assert x8_parameters["gate_center_ned_m"]["frame"] == "NED"
    with pytest.raises(ValueError, match="lack explicit value-space contracts"):
        validate_parameter_value_space_coverage(["new_public_parameter"])
    ####


def test_passive_tumbling_body_hides_inapplicable_actuator_tiers() -> None:
    tumbling = cast(dict[str, Any], load_resolved_vehicle_composition_catalog().vehicle("tumbling_body").as_dict())

    assert tumbling["fidelities"]["rigid_body_6dof_direct_wrench"]["declared"] is False
    assert tumbling["fidelities"]["rigid_body_6dof_surface_allocated"]["declared"] is False
    assert tumbling["fidelities"]["pseudo_6dof"]["control_realization"] == "uncontrolled"
    assert tumbling["mission_templates"][0]["compatible_fidelities"] == ["point_mass_3dof", "pseudo_6dof"]
    ####


def test_mission_graph_execution_extension_is_declared_by_the_mission_registry() -> None:
    contract = mission_graph_execution_contract(
        "hummingbird",
        "multirotor_pad_box_yaw_recovery_land_v1",
        "pseudo_6dof",
    )

    assert contract["status"] == "family_extension_declared"
    assert contract["translator_id"] == "taoryx.hummingbird.hover_yaw_contact.pseudo6dof.v1"
    assert contract["supported_transition_kinds"] == ["success", "timeout"]
    assert contract["timeout_constraint"] == {
        "source_nodes": "any nonterminal declared mission instance",
        "target": "declared final touchdown instance only",
        "result": "safe touchdown recovery; skipped required objectives remain failures",
    }

    default_contract = mission_graph_execution_contract(
        "skywalker_x8",
        "powered_fixed_wing_racetrack_v1",
        "pseudo_6dof",
    )
    assert default_contract["status"] == "template_success_sequence_only"
    assert default_contract["supported_transition_kinds"] == ["success"]
    ####


def test_mission_graph_execution_extension_rejects_incompatible_template_fidelity() -> None:
    registry_payload = yaml.safe_load((ROOT / "verification/vehicle_composition_registry.yaml").read_text(encoding="utf-8"))
    assert isinstance(registry_payload, dict)
    vehicles = registry_payload["vehicles"]
    assert isinstance(vehicles, list)
    hummingbird = next(item for item in vehicles if item["family_id"] == "hummingbird")
    assert isinstance(hummingbird, dict)
    missions = hummingbird["mission_templates"]
    assert isinstance(missions, list)
    mission = next(item for item in missions if item["id"] == "multirotor_pad_box_yaw_recovery_land_v1")
    assert isinstance(mission, dict)
    extension = mission["graph_execution_extension"]
    assert isinstance(extension, dict)
    extension["fidelity"] = "point_mass_3dof"
    mission["compatible_fidelities"] = ["pseudo_6dof"]

    with pytest.raises(ValueError, match="declares a graph extension for incompatible fidelity"):
        VehicleCompositionRegistry.model_validate(registry_payload)
    ####


def test_executable_mission_templates_declare_their_semantic_translator() -> None:
    catalog = load_resolved_vehicle_composition_catalog()

    expected = {
        ("skywalker_x8", "x8_local_physical_surface_lqr_screen_v1"): (
            "taoryx.x8_local_physical_surface_lqr_screen.capability.v1",
            ("rigid_body_6dof_surface_allocated",),
        ),
        ("skywalker_x8", "x8_local_physical_surface_lqi_screen_v1"): (
            "taoryx.x8_local_physical_surface_lqi_screen.capability.v1",
            ("rigid_body_6dof_surface_allocated",),
        ),
        ("skywalker_x8", "x8_local_physical_surface_lqi_long_recovery_screen_v1"): (
            "taoryx.x8_local_physical_surface_lqi_long_recovery_screen.capability.v1",
            ("rigid_body_6dof_surface_allocated",),
        ),
        ("skywalker_x8", "powered_fixed_wing_racetrack_v1"): ("taoryx.x8_racetrack.source_route.v1", CANONICAL_FIDELITY_TIERS),
        ("b747", "b747_condition3_local_physical_surface_lqr_screen_v1"): (
            "taoryx.b747_condition3_local_physical_surface_lqr_screen.capability.v1",
            ("rigid_body_6dof_surface_allocated",),
        ),
        ("b747", "b747_condition3_local_physical_surface_lqi_screen_v1"): (
            "taoryx.b747_condition3_local_physical_surface_lqi_screen.capability.v1",
            ("rigid_body_6dof_surface_allocated",),
        ),
        ("b747", "powered_fixed_wing_racetrack_v1"): ("taoryx.b747_racetrack.source_route.v1", CANONICAL_FIDELITY_TIERS),
        ("a320_openap_3dof", "powered_fixed_wing_racetrack_v1"): (
            "taoryx.a320_openap_racetrack.capability_scaled.v1",
            ("point_mass_3dof", "pseudo_6dof"),
        ),
        ("a320_openap_3dof", "a320_local_native_coordinate_lqi_screen_v1"): (
            "taoryx.a320.local_native_coordinate_lqi_screen.capability.v1",
            ("pseudo_6dof",),
        ),
        ("f16_s119", "f16_local_physical_control_screen_v1"): (
            "taoryx.f16_local_physical_control_screen.capability.v1",
            ("rigid_body_6dof_direct_wrench", "rigid_body_6dof_surface_allocated"),
        ),
        ("f16_s119", "f16_local_physical_surface_lqi_screen_v1"): (
            "taoryx.f16_local_physical_surface_lqi_screen.capability.v1",
            ("rigid_body_6dof_surface_allocated",),
        ),
        ("f16_s119", "f16_local_physical_surface_lqr_schedule_interior_screen_v1"): (
            "taoryx.f16_local_physical_surface_lqr_schedule_interior_screen.capability.v1",
            ("rigid_body_6dof_surface_allocated",),
        ),
        ("f16_s119", "f16_local_physical_surface_lqi_schedule_interior_screen_v1"): (
            "taoryx.f16_local_physical_surface_lqi_schedule_interior_screen.capability.v1",
            ("rigid_body_6dof_surface_allocated",),
        ),
        ("f16_s119", "f16_local_physical_surface_lqr_schedule_transition_screen_v1"): (
            "taoryx.f16_local_physical_surface_lqr_schedule_transition_screen.capability.v1",
            ("rigid_body_6dof_surface_allocated",),
        ),
        ("f16_s119", "powered_fixed_wing_racetrack_v1"): ("taoryx.f16_racetrack.source_route.v1", CANONICAL_FIDELITY_TIERS),
        ("hummingbird", "hummingbird_local_individual_rotor_lqi_screen_v1"): (
            "taoryx.hummingbird.local_individual_rotor_lqi_screen.capability.v1",
            ("rigid_body_6dof_surface_allocated",),
        ),
        ("hummingbird", "hummingbird_local_horizontal_translation_lqi_screen_v1"): (
            "taoryx.hummingbird.local_horizontal_translation_lqi_screen.capability.v1",
            ("rigid_body_6dof_surface_allocated",),
        ),
        ("hummingbird", "hummingbird_local_vertical_translation_lqi_screen_v1"): (
            "taoryx.hummingbird.local_vertical_translation_lqi_screen.capability.v1",
            ("rigid_body_6dof_surface_allocated",),
        ),
        ("hummingbird", "hummingbird_local_direct_wrench_screen_v1"): (
            "taoryx.hummingbird.local_direct_wrench_screen.capability.v1",
            ("rigid_body_6dof_direct_wrench",),
        ),
        ("hummingbird", "multirotor_pad_box_yaw_recovery_land_v1"): ("taoryx.hummingbird.hover_yaw_contact.pseudo6dof.v1", ("pseudo_6dof",)),
        ("x15", "x15_local_direct_wrench_screen_v1"): ("taoryx.x15_local_direct_wrench_screen.capability.v1", ("rigid_body_6dof_direct_wrench",)),
        ("x15", "x15_local_direct_wrench_lqi_screen_v1"): (
            "taoryx.x15_local_direct_wrench_lqi_screen.capability.v1",
            ("rigid_body_6dof_direct_wrench",),
        ),
        ("x15", "x15_source_surface_authority_screen_v1"): (
            "taoryx.x15_source_surface_authority_screen.capability.v1",
            ("rigid_body_6dof_surface_allocated",),
        ),
        ("x15", "x15_source_surface_attitude_rate_lqi_screen_v1"): (
            "taoryx.x15_source_surface_attitude_rate_lqi_screen.capability.v1",
            ("rigid_body_6dof_surface_allocated",),
        ),
        ("x15", "x15_staged_booster_reachability_v1"): ("taoryx.x15_staged_reachability.capability.v1", ("point_mass_3dof", "pseudo_6dof")),
        ("reference_nesc_two_stage_rocket", "staged_rocket_launch_target_state_v1"): (
            "taoryx.nesc_staged_source_replay.capability.v1",
            ("point_mass_3dof", "pseudo_6dof"),
        ),
        ("tumbling_body", "tumbling_body_release_damping_impact_v1"): ("taoryx.passive_tumbling_release.capability.v1", ("point_mass_3dof", "pseudo_6dof")),
        ("hl20_mod_k", "hl20_source_booster_release_replay_v1"): (
            "taoryx.hl20_source_booster_release_replay.v1",
            ("point_mass_3dof", "pseudo_6dof"),
        ),
        ("hl20_mod_k", "lifting_body_glide_energy_management_v1"): (
            "taoryx.hl20_glide_energy_intent.v1",
            ("point_mass_3dof", "pseudo_6dof"),
        ),
        ("hl20_mod_k", "hl20_local_direct_wrench_screen_v1"): (
            "taoryx.hl20_local_direct_wrench_screen.capability.v1",
            ("rigid_body_6dof_direct_wrench",),
        ),
        ("hl20_mod_k", "hl20_local_direct_wrench_lqi_screen_v1"): (
            "taoryx.hl20_local_direct_wrench_lqi_screen.capability.v1",
            ("rigid_body_6dof_direct_wrench",),
        ),
        ("hl20_mod_k", "hl20_source_surface_pitch_authority_screen_v1"): (
            "taoryx.hl20_source_surface_pitch_authority_screen.capability.v1",
            ("rigid_body_6dof_surface_allocated",),
        ),
        ("hl20_mod_k", "hl20_source_surface_attitude_rate_lqi_screen_v1"): (
            "taoryx.hl20_source_surface_attitude_rate_lqi_screen.capability.v1",
            ("rigid_body_6dof_surface_allocated",),
        ),
    }

    for vehicle in catalog.vehicles:
        for mission in vehicle.declaration.mission_templates:
            declaration = expected.get((vehicle.family.family_id, mission.id))
            for fidelity in mission.compatible_fidelities:
                expected_translator = None if declaration is None or fidelity not in declaration[1] else declaration[0]
                assert mission_semantic_translator_id(vehicle.family.family_id, mission.id, fidelity) == expected_translator
    ####


def test_preflight_fails_closed_when_registry_translator_has_no_installed_handler(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    composition = compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_compose.yaml"))
    monkeypatch.setattr(
        execution_preflight,
        "mission_semantic_translator_id",
        lambda *_args: "taoryx.synthetic.mismatched_translator.v1",
    )

    result = preflight_vehicle_composition(composition)

    assert result.status == "blocked"
    assert result.translator_id == "taoryx.synthetic.mismatched_translator.v1"
    assert "no installed semantic preflight handler" in result.diagnostics[0]
    ####


def test_preflight_fails_closed_when_handler_reports_a_different_translator() -> None:
    composition = compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_compose.yaml"))
    translator_id = "taoryx.x8_racetrack.source_route.v1"

    def wrong_handler(candidate: object) -> object:
        assert candidate is composition
        return replace(
            execution_preflight.preflight_powered_fixed_wing_racetrack(composition),
            translator_id="taoryx.synthetic.mismatched_translator.v1",
        )
        ####

    default_registry = execution_preflight.semantic_preflight_handler_registry()
    handler_registry = execution_preflight.SemanticPreflightHandlerRegistry(
        tuple(item for item in default_registry.handlers if item.translator_id != translator_id)
        + (execution_preflight.SemanticPreflightHandler(translator_id, wrong_handler),)
    )

    result = preflight_vehicle_composition(composition, handler_registry=handler_registry)

    assert result.status == "blocked"
    assert result.translator_id == translator_id
    assert "does not match the selected mission template" in result.diagnostics[0]
    ####


def test_semantic_preflight_handler_registry_rejects_duplicate_translator_ids() -> None:
    default_registry = execution_preflight.semantic_preflight_handler_registry()
    first = default_registry.handlers[0]

    with pytest.raises(ValueError, match="duplicate translator IDs"):
        execution_preflight.SemanticPreflightHandlerRegistry((first, first))
    ####


def test_semantic_preflight_handler_report_audits_every_declared_translator(
    capsys: pytest.CaptureFixture[str],
) -> None:
    catalog = load_resolved_vehicle_composition_catalog()
    report = execution_preflight.build_semantic_preflight_handler_report(catalog)

    assert report["schema"] == "taoryx.semantic-preflight-handler-report/v1alpha1"
    assert report["status"] == "pass"
    assert report["installed_handler_count"] >= 9
    assert report["installed_capability_adapter_count"] >= 9
    assert report["error_count"] == 0
    counts = cast(dict[str, int], report["status_counts"])
    assert counts["registered"] == report["declared_translator_count"]
    assert counts["missing_capability_adapter"] == 0
    assert counts["missing_handler"] == 0
    assert counts["translator_pending"] == 0
    assert all(record["handler_registered"] for record in cast(list[dict[str, object]], report["records"]) if record["semantic_translator_id"] is not None)
    assert all(
        record["capability_adapter_registered"]
        for record in cast(list[dict[str, object]], report["records"])
        if record["mission_capability_adapter_id"] is not None
    )

    assert main(["vehicle", "semantic-preflight-handler-report"]) == 0
    payload = cast(dict[str, Any], json.loads(capsys.readouterr().out))
    assert payload["status"] == "pass"
    assert payload["records"] == report["records"]
    ####


def test_semantic_preflight_handler_report_fails_closed_for_a_missing_declared_handler() -> None:
    report = execution_preflight.build_semantic_preflight_handler_report(handler_registry=execution_preflight.SemanticPreflightHandlerRegistry(()))

    assert report["status"] == "fail"
    assert report["error_count"] == report["declared_translator_count"]
    counts = cast(dict[str, int], report["status_counts"])
    assert counts["missing_handler"] == report["declared_translator_count"]
    ####


def test_semantic_preflight_handler_report_fails_closed_for_a_missing_declared_capability_adapter() -> None:
    report = execution_preflight.build_semantic_preflight_handler_report(capability_adapter_ids=())

    assert report["status"] == "fail"
    assert report["error_count"] == report["declared_capability_adapter_count"]
    counts = cast(dict[str, int], report["status_counts"])
    assert counts["missing_capability_adapter"] == report["declared_capability_adapter_count"]
    ####


def test_registry_preflight_dispatch_has_no_legacy_family_conditional_selector() -> None:
    assert not hasattr(execution_preflight, "_legacy_family_conditional_preflight")
    ####


def test_every_declared_mission_tier_exports_a_no_invented_default_authoring_kit() -> None:
    catalog = load_resolved_vehicle_composition_catalog()

    for vehicle in catalog.vehicles:
        for mission in vehicle.declaration.mission_templates:
            for fidelity in mission.compatible_fidelities:
                kit = vehicle.authoring_kit_dict(mission.id, fidelity)

                assert kit["schema"] == "taoryx.vehicle-composition-authoring-kit/v1alpha1"
                selection = cast(dict[str, object], kit["selection"])
                assert selection["fidelity"] == fidelity
                assert "semantic_translator_id" in selection
                request_shape = cast(dict[str, object], kit["composition_request_shape"])
                objective_schema = cast(dict[str, object], request_shape["truth_objective_topology_schema"])
                assert objective_schema["schema"] == "taoryx.truth-objective-topology/v1alpha1"
                assert objective_schema["target_channel_policy"]["declared_channels"]["heading_deg"]["value_space"]["topology"] == "periodic_circle"
                segments = cast(list[dict[str, object]], request_shape["segments"])
                assert segments
                completion = cast(dict[str, object], kit["input_completion"])
                assert completion["schema"] == "taoryx.vehicle-composition-authoring-completion/v1alpha1"
                initialization_completion = cast(list[dict[str, object]], completion["initialization_options"])
                assert initialization_completion
                assert all(item["must_select_exactly_one"] is True for item in initialization_completion)
                segment_completion = cast(list[dict[str, object]], completion["segments"])
                assert len(segment_completion) == len(segments)
                assert all(item["suggested_instance_id"].startswith("0") for item in segment_completion)
                graph_execution = cast(dict[str, object], request_shape["graph_execution_contract"])
                if vehicle.family.family_id == "hummingbird" and fidelity == "pseudo_6dof":
                    assert graph_execution["supported_transition_kinds"] == ["success", "timeout"]
                else:
                    assert graph_execution["supported_transition_kinds"] == ["success"]
                initialization = cast(dict[str, object], request_shape["initialization"])
                choices = cast(list[dict[str, object]], initialization["select_exactly_one"])
                assert choices
                inputs = [parameter for choice in choices for parameter in cast(list[dict[str, object]], choice["inputs"])] + [
                    parameter for segment in segments for parameter in cast(list[dict[str, object]], segment["inputs"])
                ]
                assert all(parameter["authoring_value"] is None for parameter in inputs)
                assert all("value_space" in parameter for parameter in inputs)
                endpoints = cast(list[dict[str, object]], kit["execution_endpoints"])
                assert all(item["execution_mode"] is not None for item in endpoints)
                commands = cast(list[str], kit["authoring_commands"])
                runnable_operations = {item["operation"] for item in endpoints if item["status"] == "runnable"}
                if "batch" in runnable_operations:
                    assert any(command.startswith("taoryx vehicle run ") for command in commands)
                else:
                    assert not any(command.startswith("taoryx vehicle run ") for command in commands)
                if "episode" in runnable_operations:
                    assert "taoryx vehicle episode-info <composition.json>" in commands
                else:
                    assert "taoryx vehicle episode-info <composition.json>" not in commands
    ####


def test_vehicle_cli_lists_and_exports_composition_sections(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["vehicle", "catalog"]) == 0
    catalog = json.loads(capsys.readouterr().out)
    assert catalog["schema"] == "taoryx.vehicle-composition-catalog/v1alpha1"
    assert catalog["detail"] == "summary"
    assert catalog["vehicle_count"] >= 9
    assert len(catalog["vehicles"]) == catalog["vehicle_count"]
    assert any(item["family_id"] == "skywalker_x8" for item in catalog["vehicles"])

    assert main(["vehicle", "catalog", "--detail", "full"]) == 0
    full_catalog = json.loads(capsys.readouterr().out)
    assert full_catalog["detail"] == "full"
    a320_descriptor = next(item for item in full_catalog["vehicles"] if item["family_id"] == "a320_openap_3dof")
    assert a320_descriptor["variant_parameters"][0]["runtime_input_path"] == "A320OpenAPOperatingPoint.mass_kg"

    assert main(["vehicle", "list"]) == 0
    listing = json.loads(capsys.readouterr().out)
    assert len(listing) == catalog["vehicle_count"]
    assert any(item["family_id"] == "hummingbird" for item in listing)

    assert main(["vehicle", "schema", "b747", "segments"]) == 0
    schema = json.loads(capsys.readouterr().out)
    assert schema["family_id"] == "b747"
    assert any(item["id"] == "fly_by_turn" for item in schema["segment_contracts"])

    assert main(["vehicle", "describe", "skywalker_x8"]) == 0
    descriptor = json.loads(capsys.readouterr().out)
    assert descriptor["family_id"] == "skywalker_x8"
    assert descriptor["interfaces"]["pseudo_6dof"]["validation_status"] == "pass"

    assert main(["vehicle", "parameters", "skywalker_x8", "--scope", "segment"]) == 0
    parameters = json.loads(capsys.readouterr().out)
    assert parameters["variant_space_status"] == "planned"
    assert any(item["id"] == "target_heading_deg" for item in parameters["parameters"])
    assert all(item["scope"] == "segment" for item in parameters["parameters"])

    assert main(["vehicle", "segment", "skywalker_x8", "fly_by_turn"]) == 0
    segment = json.loads(capsys.readouterr().out)
    assert segment["segment_contract"]["id"] == "fly_by_turn"

    assert main(["vehicle", "missions", "skywalker_x8"]) == 0
    missions = json.loads(capsys.readouterr().out)
    assert missions["mission_templates"][0]["graph"]["status"] == "linear_sequence_only"

    assert main(["vehicle", "authoring", "skywalker_x8"]) == 0
    authoring = json.loads(capsys.readouterr().out)
    assert authoring["schema"] == "taoryx.vehicle-composition-authoring-worklist/v1alpha1"
    x8_mission = next(item for item in authoring["missions"] if item["mission_id"] == "powered_fixed_wing_racetrack_v1")
    point_mass = next(item for item in x8_mission["tiers"] if item["tier"] == "point_mass_3dof")
    assert point_mass["status"] == "runnable"
    assert {"batch", "episode"} <= set(point_mass["runnable_operations"])
    assert point_mass["batch_action_trace_dispositions"] == ["emits_committed_interval_trace"]
    assert point_mass["execution_modes"] == ["closed_loop_controller", "source_native_autonomous"]
    assert not any("committed interval command history" in step for step in point_mass["next_steps"])
    assert point_mass["batch_episode_parity"]["availability"] == "registered"
    assert point_mass["mission_capability_adapter"] == "taoryx.x8_racetrack.source_route.v1"

    assert (
        main(
            [
                "vehicle",
                "authoring-template",
                "skywalker_x8",
                "powered_fixed_wing_racetrack_v1",
                "point_mass_3dof",
            ]
        )
        == 0
    )
    kit = json.loads(capsys.readouterr().out)
    assert kit["schema"] == "taoryx.vehicle-composition-authoring-kit/v1alpha1"
    assert kit["selection"]["control_realization"] == "force_model"
    assert kit["variant_admission"]["status"] == "not_declared"
    assert "No bounded runtime variant" in kit["variant_admission"]["next_steps"][0]
    request_shape = kit["composition_request_shape"]
    assert request_shape["mission"] == {"id": "powered_fixed_wing_racetrack_v1"}
    segments = request_shape["segments"]
    assert isinstance(segments, list)
    left_turn = next(item for item in segments if item["segment_id"] == "fly_by_turn" and item["occurrence_index"] == 3)
    assert left_turn["instance_id_required"] is True
    assert all(item["authoring_value"] is None for item in left_turn["inputs"])
    assert all("value_space" in item for item in left_turn["inputs"])
    assert {(item["operation"], item["factory_id"]) for item in kit["execution_endpoints"] if item["status"] == "runnable"} == {
        ("batch", "language_backed_powered_fixed_wing.v1"),
        ("episode", "language_backed_interactive.v1"),
    }
    assert {item["operation"]: item["batch_action_trace"] for item in kit["execution_endpoints"]} == {
        "batch": "emits_committed_interval_trace",
        "episode": "not_applicable",
    }
    assert {item["operation"]: item["execution_mode"] for item in kit["execution_endpoints"]} == {
        "batch": "source_native_autonomous",
        "episode": "closed_loop_controller",
    }
    assert "taoryx vehicle episode-info <composition.json>" in kit["authoring_commands"]

    assert main(["vehicle", "authoring-all"]) == 0
    authoring_all = json.loads(capsys.readouterr().out)
    assert authoring_all["schema"] == "taoryx.vehicle-composition-authoring-catalog/v1alpha1"
    assert authoring_all["vehicle_count"] == catalog["vehicle_count"]
    assert authoring_all["mission_tier_count"] >= authoring_all["batch_runnable_mission_tier_count"]
    assert authoring_all["runnable_variant_count"] == 2
    assert authoring_all["variant_space_status_counts"] == {"not_declared": 7, "runnable": 2}
    assert any(item["family_id"] == "skywalker_x8" for item in authoring_all["vehicles"])

    assert main(["vehicle", "maturity-report"]) == 0
    maturity = json.loads(capsys.readouterr().out)
    assert maturity["schema"] == "taoryx.mission-composition-maturity-report/v1alpha1"
    assert maturity["topology"]["status"] == "pass"
    assert maturity["topology"]["canonical_channel_count"] > 0
    assert maturity["topology"]["resource_channel_count"] == 5
    assert maturity["execution"]["semantic_preflight_handlers"]["status"] == "pass"
    assert maturity["execution"]["concrete_capability_preflight"]["status"] == "not_checked"
    assert maturity["execution"]["episode_contract_conformance"]["status"] == "not_checked"
    assert maturity["execution"]["execution_witnesses"]["status"] == "not_checked"
    assert maturity["execution"]["batch_interface_trace_conformance"]["status"] == "not_checked"
    discovery = maturity["discovery_and_authoring"]
    variant_admission = discovery["variant_admission"]
    assert variant_admission == {
        "status": "pass",
        "variant_space_status_counts": {"not_declared": 7, "runnable": 2},
        "runnable_variant_count": 2,
        "topology_runtime_backed_variant_count": 2,
        "claim_boundary": (
            "This verifies that Mission Composition discovery, authoring, and topology agree about declared runtime-bound "
            "variants. It does not prove the modifier's physical coupling, trim, or qualification beyond its "
            "separate runtime evidence."
        ),
    }
    assert discovery["planned_execution_binding_count"] == 3
    promotion_blockers = discovery["fidelity_promotion_blocker_counts"]
    assert promotion_blockers["envelope_robustness_and_flight_qualification"] == 2
    assert promotion_blockers["source_trim_acceptance"] >= 3
    blockers = discovery["planned_execution_blocker_counts"]
    assert blockers["energy_glide_segment_translator"] == 1
    assert blockers["source_bounded_handoff_evaluator"] == 2

    direct_wrench = next(item for item in x8_mission["tiers"] if item["tier"] == "rigid_body_6dof_direct_wrench")
    assert direct_wrench["status"] == "runnable"
    assert direct_wrench["fidelity_promotion_blockers"] == ["envelope_robustness_and_flight_qualification"]
    assert any("envelope_robustness_and_flight_qualification" in item for item in direct_wrench["next_steps"])
    assert not any("batch factory" in item for item in direct_wrench["next_steps"])
    assert direct_wrench["planned_execution_blockers"] == {}

    hl20_worklist = load_resolved_vehicle_composition_catalog().vehicle("hl20_mod_k").authoring_worklist_dict()
    hl20_variants = cast(dict[str, object], hl20_worklist["variant_worklist"])
    assert hl20_variants["status"] == "not_declared"
    assert "No bounded runtime variant" in cast(list[str], hl20_variants["next_steps"])[0]
    hl20_glide = next(item for item in hl20_worklist["missions"] if item["mission_id"] == "lifting_body_glide_energy_management_v1")
    hl20_direct_wrench = next(item for item in hl20_glide["tiers"] if item["tier"] == "rigid_body_6dof_direct_wrench")
    assert hl20_direct_wrench["fidelity_promotion_blockers"] == ["source_trim_acceptance"]
    assert hl20_direct_wrench["planned_execution_blockers"] == {
        "batch": [
            "declared_direct_wrench_authority_deficit_at_mach2_source_point",
            "release_trim_and_gravity_state_binding",
            "energy_glide_segment_translator",
            "source_bounded_handoff_evaluator",
        ]
    }
    assert any("energy_glide_segment_translator" in item for item in hl20_direct_wrench["next_steps"])
    ####


def test_authoring_kit_advertises_exact_endpoints_and_only_supported_commands() -> None:
    catalog = load_resolved_vehicle_composition_catalog()
    f16 = catalog.vehicle("f16_s119").authoring_kit_dict("powered_fixed_wing_racetrack_v1", "pseudo_6dof")
    f16_endpoints = cast(list[dict[str, object]], f16["execution_endpoints"])
    f16_commands = cast(list[str], f16["authoring_commands"])

    assert {(item["operation"], item["factory_id"]) for item in f16_endpoints if item["status"] == "runnable"} == {
        ("batch", "reduced_fixed_wing_f16_source.v1"),
        ("episode", "reduced_fixed_wing_f16_episode.v1"),
    }
    assert "taoryx vehicle episode-info <composition.json>" in f16_commands
    assert "taoryx vehicle batch-episode-parity <composition.json> <policy-trace.json>" in f16_commands

    source_replay = catalog.vehicle("hl20_mod_k").authoring_kit_dict(
        "hl20_source_booster_release_replay_v1",
        "pseudo_6dof",
    )
    source_endpoints = cast(list[dict[str, object]], source_replay["execution_endpoints"])
    assert source_endpoints[0]["execution_mode"] == "source_scheduled_replay"
    assert source_endpoints[0]["operation"] == "batch"

    hl20 = catalog.vehicle("hl20_mod_k").authoring_kit_dict("lifting_body_glide_energy_management_v1", "rigid_body_6dof_direct_wrench")
    hl20_commands = cast(list[str], hl20["authoring_commands"])
    assert not any(command.startswith("taoryx vehicle run ") for command in hl20_commands)
    assert "taoryx vehicle episode-info <composition.json>" not in hl20_commands
    ####


def test_hl20_authoring_kit_distinguishes_signed_bank_intent_from_native_lowering() -> None:
    catalog = load_resolved_vehicle_composition_catalog()

    kit = catalog.vehicle("hl20_mod_k").authoring_kit_dict(
        "lifting_body_glide_energy_management_v1",
        "point_mass_3dof",
    )

    tier = cast(dict[str, object], kit["runtime_and_evidence_worklist"])
    assert tier["mission_capability_adapter"] == "taoryx.hl20_glide_energy.capability.v1"
    assert tier["semantic_translator_id"] == "taoryx.hl20_glide_energy_intent.v1"
    assert not any("declare a source-owned semantic translator" in step for step in cast(list[str], tier["next_steps"]))
    segments = cast(list[dict[str, object]], kit["composition_request_shape"]["segments"])
    glide_reversal = next(item for item in segments if item["segment_id"] == "glide_bank_reversal")
    bank = next(item for item in cast(list[dict[str, object]], glide_reversal["inputs"]) if item["id"] == "bank_command_deg")
    assert bank["value_space"]["topology"] == "bounded_interval"
    assert bank["hard_lower"] == -89.0
    assert bank["hard_upper"] == 89.0
    ####


def test_mission_composition_maturity_report_keeps_coverage_and_evidence_distinct() -> None:
    report = build_mission_composition_maturity_report()

    assert report["status"] == "pass"
    topology = cast(dict[str, object], report["topology"])
    assert topology["finding_count"] == 0
    # The public catalogue may legitimately grow as additional vehicle
    # controls, status, resource, and diagnostic channels are advertised.
    # Keep the established coverage floor without turning that growth into a
    # false regression.
    assert topology["interface_channel_count"] >= 940
    execution = cast(dict[str, object], report["execution"])
    assert cast(dict[str, int], execution["runnable_operation_counts"])["episode"] >= 1
    release_packet_conformance = cast(dict[str, object], execution["release_packet_conformance"])
    assert release_packet_conformance["status"] == "not_checked"
    assert cast(dict[str, int], execution["parity_disposition_counts"])["registered"] >= 1
    assert cast(dict[str, int], execution["endpoint_execution_mode_counts"])["closed_loop_controller"] >= 18
    assert cast(dict[str, object], execution["execution_witnesses"])["status"] == "not_checked"
    assert cast(dict[str, object], execution["concrete_capability_preflight"])["status"] == "not_checked"
    assert cast(dict[str, object], execution["episode_contract_conformance"])["status"] == "not_checked"
    assert cast(dict[str, object], execution["graph_extension_conformance"])["status"] == "not_checked"
    assert cast(dict[str, object], execution["batch_episode_parity_witnesses"])["status"] == "not_checked"
    discovery = cast(dict[str, object], report["discovery_and_authoring"])
    trace_dispositions = cast(dict[str, int], discovery["batch_action_trace_disposition_counts"])
    assert trace_dispositions["emits_committed_interval_trace"] >= 1
    execution_modes = cast(dict[str, int], discovery["execution_mode_counts"])
    assert execution_modes["closed_loop_controller"] >= 1
    assert execution_modes["source_scheduled_replay"] >= 1
    assert trace_dispositions.get("not_emitted", 0) == 0
    integration = cast(dict[str, object], report["family_integration"])
    assert integration["strategy_conformance_status"] == "pass"
    assert integration["tier_count"] == 36
    assert cast(dict[str, int], integration["work_status_counts"])["strategy_probe_ready"] >= 1
    assert "not a Mission Composition catalog failure" in str(integration["claim_boundary"])
    assert cast(dict[str, object], report["result_catalog"])["status"] == "not_supplied"
    ####


def test_mission_composition_maturity_batch_smoke_implies_endpoint_witness_check(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[bool] = []

    def fake_witness_check(
        *,
        execute_batch: bool = False,
        retained_results_directory: Path | None = None,
    ) -> dict[str, object]:
        assert retained_results_directory is None
        calls.append(execute_batch)
        return {
            "status": "pass",
            "batch_execution_smoke": execute_batch,
            "records": [
                {
                    "operation": "batch",
                    "batch_execution": {
                        "release_packet": {"status": "pass"},
                        "interface_trace": {
                            "status": "pass",
                            "batch_visible_channels": {
                                "status": ["position.altitude"],
                                "resources": ["resources.mass.total"],
                                "diagnostics": ["diagnostics.segment_index"],
                            },
                        },
                    },
                }
            ],
        }
        ####

    monkeypatch.setattr(execution_witnesses, "validate_vehicle_execution_witnesses", fake_witness_check)

    report = build_mission_composition_maturity_report(execute_batch_witnesses=True)

    assert report["status"] == "pass"
    assert calls == [True]
    execution = cast(dict[str, object], report["execution"])
    witness = cast(dict[str, object], execution["execution_witnesses"])
    assert witness["batch_execution_smoke"] is True
    interface_trace = cast(dict[str, object], execution["batch_interface_trace_conformance"])
    assert interface_trace["status"] == "pass"
    assert interface_trace["channel_counts"] == {"status": 1, "resources": 1, "diagnostics": 1}
    release_packet_conformance = cast(dict[str, object], execution["release_packet_conformance"])
    assert release_packet_conformance == {
        "status": "pass",
        "batch_endpoint_count": 1,
        "verified_packet_count": 1,
        "failure_count": 0,
        "claim_boundary": (
            "This verifies that each generated public batch packet can be indexed and packaged as one "
            "hash-bound release packet with a verified reproduction identity. It does not establish "
            "robustness, physical fidelity, or mission qualification."
        ),
    }
    ####


def test_mission_composition_maturity_projects_family_graph_extension_witnesses(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def fake_witness_check(*, execute_batch: bool = False) -> dict[str, object]:
        return {
            "status": "pass",
            "records": [],
            "graph_extension_records": [
                {
                    "graph_status": "authored_graph_not_lowered",
                    "required_transition_kind": "timeout",
                    "supported_transition_kinds": ["success", "timeout"],
                    "batch_execution": (
                        {
                            "graph_execution": {
                                "observation_status": "observed",
                            }
                        }
                        if execute_batch
                        else None
                    ),
                }
            ],
        }
        ####

    monkeypatch.setattr(execution_witnesses, "validate_vehicle_execution_witnesses", fake_witness_check)

    static_report = build_mission_composition_maturity_report(check_execution_witnesses=True)
    static_execution = cast(dict[str, object], static_report["execution"])
    static_graph = cast(dict[str, object], static_execution["graph_extension_conformance"])
    assert static_graph["status"] == "pass"
    assert static_graph["witness_count"] == 1
    assert static_graph["observed_execution_count"] is None

    batch_report = build_mission_composition_maturity_report(execute_batch_witnesses=True)
    batch_execution = cast(dict[str, object], batch_report["execution"])
    batch_graph = cast(dict[str, object], batch_execution["graph_extension_conformance"])
    assert batch_graph["status"] == "pass"
    assert batch_graph["observed_execution_count"] == 1
    ####


def test_mission_composition_maturity_can_execute_registered_parity_witnesses(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[bool] = []

    def fake_parity_witnesses() -> dict[str, object]:
        calls.append(True)
        return {"status": "pass", "registered_binding_count": 9}
        ####

    monkeypatch.setattr(mission_composition_maturity, "validate_vehicle_execution_parity_witnesses", fake_parity_witnesses)

    report = build_mission_composition_maturity_report(execute_parity_witnesses=True)

    assert report["status"] == "pass"
    assert calls == [True]
    execution = cast(dict[str, object], report["execution"])
    assert cast(dict[str, object], execution["batch_episode_parity_witnesses"])["registered_binding_count"] == 9
    ####


def test_mission_composition_maturity_report_can_index_retained_results_without_promoting_them(tmp_path: Path) -> None:
    report = build_mission_composition_maturity_report(results_directory=tmp_path)

    assert report["status"] == "pass"
    result_catalog = cast(dict[str, object], report["result_catalog"])
    assert result_catalog["status"] == "empty"
    assert result_catalog["origin"] == "supplied_result_directory"
    assert result_catalog["result_count"] == 0
    assert result_catalog["semantic_action_trace_status_counts"] == {}
    assert "does not rerun a vehicle" in str(result_catalog["claim_boundary"])
    ####


def test_mission_composition_maturity_retains_and_indexes_batch_witness_results(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    retained = tmp_path / "retained"
    captured: dict[str, object] = {}

    def fake_witness_check(
        *,
        execute_batch: bool = False,
        retained_results_directory: Path | None = None,
    ) -> dict[str, object]:
        captured["execute_batch"] = execute_batch
        captured["retained_results_directory"] = retained_results_directory
        assert retained_results_directory is not None
        retained_results_directory.mkdir()
        return {"status": "pass", "records": []}
        ####

    monkeypatch.setattr(execution_witnesses, "validate_vehicle_execution_witnesses", fake_witness_check)

    report = build_mission_composition_maturity_report(
        execute_batch_witnesses=True,
        retained_batch_results_directory=retained,
    )

    assert report["status"] == "pass"
    assert captured == {"execute_batch": True, "retained_results_directory": retained}
    result_catalog = cast(dict[str, object], report["result_catalog"])
    assert result_catalog["status"] == "empty"
    assert result_catalog["origin"] == "generated_batch_witnesses"
    assert result_catalog["root_directory"] == str(retained)
    assert "generated during this report" in str(result_catalog["claim_boundary"])
    ####


def test_mission_composition_maturity_rejects_ambiguous_or_inactive_batch_retention(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="requires execute_batch_witnesses"):
        build_mission_composition_maturity_report(retained_batch_results_directory=tmp_path / "retained")
    with pytest.raises(ValueError, match="cannot be combined"):
        build_mission_composition_maturity_report(
            execute_batch_witnesses=True,
            results_directory=tmp_path / "existing",
            retained_batch_results_directory=tmp_path / "retained",
        )
    ####


def test_mission_composition_maturity_report_fails_for_invalid_retained_evidence(tmp_path: Path) -> None:
    broken = tmp_path / "broken" / "evaluation.json"
    broken.parent.mkdir()
    broken.write_text("not-json", encoding="utf-8")

    report = build_mission_composition_maturity_report(results_directory=tmp_path)

    assert report["status"] == "fail"
    result_catalog = cast(dict[str, object], report["result_catalog"])
    assert result_catalog["status"] == "fail"
    assert result_catalog["error_count"] == 1
    ####


def test_mission_composition_maturity_report_is_available_through_the_public_cli(capsys: pytest.CaptureFixture[str], tmp_path: Path) -> None:
    assert main(["vehicle", "maturity-report", "--results-dir", str(tmp_path)]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["schema"] == "taoryx.mission-composition-maturity-report/v1alpha1"
    assert payload["status"] == "pass"
    assert payload["execution"]["execution_witnesses"]["status"] == "not_checked"
    assert payload["result_catalog"]["status"] == "empty"
    ####


def test_mission_composition_maturity_cli_exposes_batch_witness_smoke(
    capsys: pytest.CaptureFixture[str],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    captured: dict[str, object] = {}

    def fake_maturity_report(
        catalog: object,
        *,
        check_execution_witnesses: bool,
        execute_batch_witnesses: bool,
        execute_parity_witnesses: bool,
        results_directory: Path | None,
        retained_batch_results_directory: Path | None,
    ) -> dict[str, object]:
        captured.update(
            {
                "check_execution_witnesses": check_execution_witnesses,
                "execute_batch_witnesses": execute_batch_witnesses,
                "execute_parity_witnesses": execute_parity_witnesses,
                "results_directory": results_directory,
                "retained_batch_results_directory": retained_batch_results_directory,
            }
        )
        return {"status": "pass", "schema": "test-maturity"}
        ####

    monkeypatch.setattr(runtime_cli, "build_mission_composition_maturity_report", fake_maturity_report)

    assert main(["vehicle", "maturity-report", "--execute-batch-witnesses"]) == 0
    assert json.loads(capsys.readouterr().out)["schema"] == "test-maturity"
    assert captured == {
        "check_execution_witnesses": False,
        "execute_batch_witnesses": True,
        "execute_parity_witnesses": False,
        "results_directory": None,
        "retained_batch_results_directory": None,
    }
    ####


def test_mission_composition_maturity_cli_forwards_batch_result_retention(
    capsys: pytest.CaptureFixture[str],
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    captured: dict[str, object] = {}
    retained = tmp_path / "retained"

    def fake_maturity_report(
        catalog: object,
        *,
        check_execution_witnesses: bool,
        execute_batch_witnesses: bool,
        execute_parity_witnesses: bool,
        results_directory: Path | None,
        retained_batch_results_directory: Path | None,
    ) -> dict[str, object]:
        captured.update(
            {
                "check_execution_witnesses": check_execution_witnesses,
                "execute_batch_witnesses": execute_batch_witnesses,
                "execute_parity_witnesses": execute_parity_witnesses,
                "results_directory": results_directory,
                "retained_batch_results_directory": retained_batch_results_directory,
            }
        )
        return {"status": "pass", "schema": "test-maturity"}
        ####

    monkeypatch.setattr(runtime_cli, "build_mission_composition_maturity_report", fake_maturity_report)

    assert main(
        [
            "vehicle",
            "maturity-report",
            "--execute-batch-witnesses",
            "--retain-batch-results-dir",
            str(retained),
        ]
    ) == 0
    assert json.loads(capsys.readouterr().out)["schema"] == "test-maturity"
    assert captured == {
        "check_execution_witnesses": False,
        "execute_batch_witnesses": True,
        "execute_parity_witnesses": False,
        "results_directory": None,
        "retained_batch_results_directory": retained,
    }
    ####


def test_mission_composition_maturity_cli_rejects_ambiguous_or_inactive_batch_retention(
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    retained = tmp_path / "retained"

    assert main(["vehicle", "maturity-report", "--retain-batch-results-dir", str(retained)]) == 2
    assert "requires --execute-batch-witnesses" in capsys.readouterr().out

    assert main(
        [
            "vehicle",
            "maturity-report",
            "--execute-batch-witnesses",
            "--results-dir",
            str(tmp_path / "existing"),
            "--retain-batch-results-dir",
            str(retained),
        ]
    ) == 2
    assert "either --results-dir or --retain-batch-results-dir" in capsys.readouterr().out
    ####


def test_vehicle_witness_report_cli_keeps_a_batch_smoke_family_scoped(
    capsys: pytest.CaptureFixture[str],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The installed CLI exposes the same focused proof as the developer tool."""

    captured: dict[str, object] = {}

    def fake_witness_report(
        *,
        execute_batch: bool,
        family_ids: list[str] | None,
        witness_ids: list[str] | None,
        results_directory: Path | None,
    ) -> dict[str, object]:
        captured.update(
            {
                "execute_batch": execute_batch,
                "family_ids": family_ids,
                "witness_ids": witness_ids,
                "results_directory": results_directory,
            }
        )
        return {"status": "pass", "schema": "test-vehicle-witness-report"}
        ####

    monkeypatch.setattr(runtime_cli, "build_vehicle_execution_witness_report", fake_witness_report)

    assert main(["vehicle", "witness-report", "--family", "hummingbird", "--execute-batch"]) == 0
    assert json.loads(capsys.readouterr().out)["schema"] == "test-vehicle-witness-report"
    assert captured == {
        "execute_batch": True,
        "family_ids": ["hummingbird"],
        "witness_ids": None,
        "results_directory": None,
    }
    ####


def test_vehicle_witness_report_cli_forwards_a_retained_result_directory(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    captured: dict[str, object] = {}
    destination = tmp_path / "retained-results"

    def fake_witness_report(
        *,
        execute_batch: bool,
        family_ids: list[str] | None,
        witness_ids: list[str] | None,
        results_directory: Path | None,
    ) -> dict[str, object]:
        captured.update(
            {
                "execute_batch": execute_batch,
                "family_ids": family_ids,
                "witness_ids": witness_ids,
                "results_directory": results_directory,
            }
        )
        return {"status": "pass", "schema": "test-retained-vehicle-witness-report"}
        ####

    monkeypatch.setattr(runtime_cli, "build_vehicle_execution_witness_report", fake_witness_report)

    assert main(["vehicle", "witness-report", "--witness", "x8-3dof-batch", "--execute-batch", "--results-dir", str(destination)]) == 0
    assert json.loads(capsys.readouterr().out)["schema"] == "test-retained-vehicle-witness-report"
    assert captured == {
        "execute_batch": True,
        "family_ids": None,
        "witness_ids": ["x8-3dof-batch"],
        "results_directory": destination,
    }
    ####


def test_mission_composition_maturity_cli_exposes_parity_witness_smoke(
    capsys: pytest.CaptureFixture[str],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    captured: dict[str, object] = {}

    def fake_maturity_report(
        catalog: object,
        *,
        check_execution_witnesses: bool,
        execute_batch_witnesses: bool,
        execute_parity_witnesses: bool,
        results_directory: Path | None,
        retained_batch_results_directory: Path | None,
    ) -> dict[str, object]:
        captured.update(
            {
                "check_execution_witnesses": check_execution_witnesses,
                "execute_batch_witnesses": execute_batch_witnesses,
                "execute_parity_witnesses": execute_parity_witnesses,
                "results_directory": results_directory,
                "retained_batch_results_directory": retained_batch_results_directory,
            }
        )
        return {"status": "pass", "schema": "test-parity-maturity"}
        ####

    monkeypatch.setattr(runtime_cli, "build_mission_composition_maturity_report", fake_maturity_report)

    assert main(["vehicle", "maturity-report", "--execute-parity-witnesses"]) == 0
    assert json.loads(capsys.readouterr().out)["schema"] == "test-parity-maturity"
    assert captured == {
        "check_execution_witnesses": False,
        "execute_batch_witnesses": False,
        "execute_parity_witnesses": True,
        "results_directory": None,
        "retained_batch_results_directory": None,
    }
    ####


def test_vehicle_mission_cli_reuses_the_authoring_and_compilation_contracts(
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    assert main(["vehicle", "mission", "inspect", "skywalker_x8", "powered_fixed_wing_racetrack_v1"]) == 0
    inspection = json.loads(capsys.readouterr().out)
    assert inspection["schema"] == "taoryx.vehicle-mission-inspection/v1alpha1"
    assert inspection["mission_template"]["graph"]["status"] == "linear_sequence_only"
    assert inspection["fidelity_contracts"]["point_mass_3dof"]["control_realization"] == "force_model"

    kit_path = tmp_path / "x8-mission-kit.json"
    assert (
        main(
            [
                "vehicle",
                "mission",
                "create",
                "skywalker_x8",
                "powered_fixed_wing_racetrack_v1",
                "point_mass_3dof",
                "--output",
                str(kit_path),
            ]
        )
        == 0
    )
    capsys.readouterr()
    kit = json.loads(kit_path.read_text(encoding="utf-8"))
    assert kit["schema"] == "taoryx.vehicle-composition-authoring-kit/v1alpha1"
    assert kit["selection"]["fidelity"] == "point_mass_3dof"

    compiled_path = tmp_path / "x8-mission.json"
    assert (
        main(
            [
                "vehicle",
                "mission",
                "validate",
                str(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml"),
                "--compiled-output",
                str(compiled_path),
            ]
        )
        == 0
    )
    validation = json.loads(capsys.readouterr().out)
    assert validation["schema"] == "taoryx.vehicle-mission-validation/v1alpha1"
    assert validation["semantic_validation"] == "pass"
    assert validation["runtime_readiness"]["preflight_status"] == "translation_ready"
    advertisement = validation["runtime_readiness"]["preflight"]["capability_estimate"]["capability_advertisement"]
    assert advertisement["schema"] == "taoryx.vehicle-capability-advertisement/v1alpha1"
    assert advertisement["selection"]["mission_id"] == "powered_fixed_wing_racetrack_v1"
    assert advertisement["interface"]["interface_id"] == "skywalker_x8/point_mass_3dof"
    assert json.loads(compiled_path.read_text(encoding="utf-8"))["mission"] == "powered_fixed_wing_racetrack_v1"
    ####


def test_vehicle_mission_validate_exposes_blocked_hl20_runtime_admission(
    capsys: pytest.CaptureFixture[str],
) -> None:
    assert (
        main(
            [
                "vehicle",
                "mission",
                "validate",
                str(ROOT / "examples/vehicle_composition/hl20_glide_energy_capability_3dof_compose.yaml"),
            ]
        )
        == 0
    )
    validation = json.loads(capsys.readouterr().out)

    readiness = validation["runtime_readiness"]
    assert readiness["preflight_status"] == "translation_ready"
    assert readiness["lowering_status"] == "blocked"
    advertisement = readiness["preflight"]["capability_estimate"]["capability_advertisement"]
    assert advertisement["interface"]["execution_records"] == []
    assert "lifting_body_glide_energy_management_v1" in advertisement["interface"]["claim_boundary"]
    admission = advertisement["family_owned"]["source_runtime_admission"]
    assert admission["status"] == "outside_source_domain"
    assert admission["runtime_factory_declared"] is False
    ####


def test_vehicle_variant_cli_retains_runtime_binding_and_qualification_boundary(
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    request = ROOT / "examples/vehicle_composition/hummingbird_grounded_mass_variant_pseudo6dof_compose.yaml"
    assert main(["vehicle", "variant", "validate", str(request)]) == 0
    validation = json.loads(capsys.readouterr().out)
    assert validation["schema"] == "taoryx.vehicle-variant-validation/v1alpha1"
    assert validation["status"] == "valid"
    assert validation["variant"]["qualification"] == "extended"
    binding = validation["variant"]["runtime_bindings"]["grounded_operating_mass_kg"]
    assert binding["runtime_input_path"] == "HummingbirdPseudo6DOFModel.mass_kg"
    assert validation["resulting_initialization_inputs"]["mass_kg"]["value"] == 0.55

    output = tmp_path / "hummingbird-variant.json"
    assert main(["vehicle", "variant", "resolve", str(request), "--output", str(output)]) == 0
    resolution = json.loads(capsys.readouterr().out)
    assert resolution["schema"] == "taoryx.vehicle-variant-resolution/v1alpha1"
    assert resolution["compiled_composition"] == str(output)
    assert json.loads(output.read_text(encoding="utf-8"))["variant"]["inputs"]["grounded_operating_mass_kg"]["value"] == 0.55
    ####


def test_compose_x8_racetrack_creates_immutable_semantic_adapter_handoff() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_compose.yaml")
    compiled = compile_vehicle_composition(request)

    assert compiled.vehicle_id == "skywalker_x8"
    assert compiled.fidelity == "pseudo_6dof"
    assert compiled.composition_status == "development"
    assert compiled.segments[2].instance_id == "left-turn"
    assert compiled.segments[2].state_transfer == "previous_terminal_truth_state"
    assert compiled.native_adapter_handoff["adapter_id"] == "taoryx.fixed_wing.source_table.v1"
    assert compiled.native_adapter_handoff["handoff_status"] == "semantic_compiled_runtime_lowering_pending"
    assert len(compiled.identity_sha256) == 64
    ####


def test_vehicle_cli_opens_only_the_declared_episode_and_exports_committed_truth(
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    request = ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml"
    compiled_path = tmp_path / "x8-racetrack.json"

    assert main(["vehicle", "compose", str(request), "--output", str(compiled_path)]) == 0
    capsys.readouterr()
    assert main(["vehicle", "episode-info", str(compiled_path), "--seed", "17"]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["schema"] == "taoryx.vehicle-composition-episode-info/v1alpha1"
    assert payload["execution_binding"]["factory_id"] == "language_backed_interactive.v1"
    assert payload["initial_truth"]["time_s"] == 0.0
    assert payload["initial_status"]["time_s"] == 0.0
    assert payload["initial_observation"]["time_s"] == 0.0
    throttle = next(channel for channel in payload["action_schema"] if channel["name"] == "throttle")
    assert throttle["value_space"]["topology"] == "unit_interval"
    assert all(channel["value_space"]["topology"] != "topology_pending" for channel in payload["observation_schema"])
    ####


def test_vehicle_cli_reads_a_fingerprint_bound_normalized_result(
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    request_path = ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml"
    composition = compile_vehicle_composition(load_vehicle_composition_request(request_path))
    composition_path = tmp_path / "x8-racetrack.json"
    composition_path.write_text(json.dumps(composition.model_dump(mode="json", by_alias=True)), encoding="utf-8")
    output_dir = tmp_path / "run"
    output_dir.mkdir()
    evaluation = TrajectoryEvaluation(
        scenario_id=composition.id,
        scenario_contract_sha256=composition.identity_sha256,
        validity="valid",
        qualification="unqualified",
        feasibility="feasible",
        outcome="completed",
        claim_boundary="synthetic CLI result-reader fixture",
    )
    (output_dir / "evaluation.json").write_text(json.dumps(evaluation.as_dict()), encoding="utf-8")
    (output_dir / "objective_report.json").write_text("{}", encoding="utf-8")
    (output_dir / "nonlinear_validation.json").write_text("{}", encoding="utf-8")

    assert main(["vehicle", "result", str(output_dir), "--composition", str(composition_path)]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["schema"] == "taoryx.vehicle-composition-result/v1alpha1"
    assert payload["evaluation"]["outcome"] == "completed"
    assert payload["composition_identity_sha256"] == composition.identity_sha256
    assert payload["artifacts"]["objective_report.json"] == str(output_dir / "objective_report.json")
    assert payload["artifacts"]["nonlinear_validation.json"] == str(output_dir / "nonlinear_validation.json")
    ####


def test_compose_rejects_missing_or_reordered_required_segments() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_compose.yaml")
    missing_input = request.model_copy(update={"initialization": request.initialization.model_copy(update={"inputs": {}})})
    with pytest.raises(VehicleCompositionError, match="missing-input"):
        compile_vehicle_composition(missing_input)

    reordered = request.model_copy(update={"segments": tuple(reversed(request.segments))})
    with pytest.raises(VehicleCompositionError, match="segment-sequence-mismatch"):
        compile_vehicle_composition(reordered)
    ####


def test_caller_authored_mission_graph_validates_but_cannot_silently_execute() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    instance_ids = tuple(segment.instance_id or f"{index:02d}-{segment.id}" for index, segment in enumerate(request.segments, start=1))
    segments = request.segments
    graph = MissionGraphSelection(
        entry_instance_id=instance_ids[0],
        nodes=tuple(
            MissionGraphNodeSelection(
                instance_id=instance_ids[index],
                success_transition=(None if index + 1 == len(segments) else MissionTransitionSelection(target_instance_id=instance_ids[index + 1])),
                abort_transition=(MissionTransitionSelection(target_instance_id=instance_ids[-1]) if index == 0 else None),
            )
            for index, segment in enumerate(segments)
        ),
    )
    composition = compile_vehicle_composition(request.model_copy(update={"segments": segments, "mission_graph": graph}))

    assert composition.mission_graph is not None
    assert composition.mission_graph.status == "authored_graph_not_lowered"
    first = composition.mission_graph.nodes[0]
    assert first.abort_transition is not None
    assert first.abort_transition.target_instance_id == instance_ids[-1]

    preflight = preflight_vehicle_composition(composition)
    assert preflight.status == "blocked"
    assert "graph" in preflight.diagnostics[0]
    lowering = lower_vehicle_composition(composition)
    assert lowering.status == "blocked"
    assert "graph" in lowering.diagnostics[0]
    ####


def test_caller_authored_exact_linear_graph_reuses_declared_native_sequence() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    instance_ids = tuple(segment.instance_id or f"{index:02d}-{segment.id}" for index, segment in enumerate(request.segments, start=1))
    graph = MissionGraphSelection(
        entry_instance_id=instance_ids[0],
        nodes=tuple(
            MissionGraphNodeSelection(
                instance_id=instance_ids[index],
                success_transition=(None if index + 1 == len(request.segments) else MissionTransitionSelection(target_instance_id=instance_ids[index + 1])),
            )
            for index in range(len(request.segments))
        ),
    )
    composition = compile_vehicle_composition(request.model_copy(update={"mission_graph": graph}))

    assert composition.mission_graph is not None
    assert composition.mission_graph.status == "authored_linear_sequence_lowered"
    assert preflight_vehicle_composition(composition).status == "translation_ready"
    lowering = lower_vehicle_composition(composition)
    assert lowering.status == "factory_bound"
    assert lowering.execution_binding is not None
    assert lowering.execution_binding["factory_id"] == "language_backed_powered_fixed_wing.v1"
    assert lowering.translator_status == "translation_ready"
    assert all("graph" not in diagnostic for diagnostic in lowering.diagnostics)
    ####


def test_runtime_lowering_rejects_a_preflight_artifact_from_another_composition() -> None:
    x8 = compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml"))
    hummingbird = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/hummingbird_grounded_mass_variant_pseudo6dof_compose.yaml")
    )
    x15 = compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x15_local_direct_wrench_screen_compose.yaml"))

    foreign_preflight = preflight_vehicle_composition(hummingbird)
    for composition in (x8, x15):
        result = lower_vehicle_composition(composition, preflight_result=foreign_preflight)

        assert result.status == "blocked"
        assert "does not belong" in result.diagnostics[0]
    ####


def test_checked_in_authored_linear_graph_example_lowers_to_the_native_sequence() -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_authored_linear_graph_3dof_compose.yaml")
    )

    assert composition.mission_graph is not None
    assert composition.mission_graph.status == "authored_linear_sequence_lowered"
    assert composition.mission_graph.entry_instance_id == "trim-hold"
    assert preflight_vehicle_composition(composition).status == "translation_ready"
    ####


def test_caller_authored_mission_graph_rejects_unknown_targets_and_cycles() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    instance_ids = tuple(f"{index:02d}-{segment.id}" for index, segment in enumerate(request.segments, start=1))
    segments = tuple(segment.model_copy(update={"instance_id": instance_ids[index]}) for index, segment in enumerate(request.segments))
    unknown_target = MissionGraphSelection(
        entry_instance_id=instance_ids[0],
        nodes=tuple(
            MissionGraphNodeSelection(
                instance_id=instance_ids[index],
                success_transition=MissionTransitionSelection(target_instance_id=("not-a-selected-segment" if index == 0 else None)),
            )
            for index, segment in enumerate(segments)
        ),
    )
    with pytest.raises(VehicleCompositionError, match="unknown-mission-graph-target"):
        compile_vehicle_composition(request.model_copy(update={"segments": segments, "mission_graph": unknown_target}))

    cyclic_nodes = [
        MissionGraphNodeSelection(
            instance_id=instance_ids[index],
            success_transition=MissionTransitionSelection(target_instance_id=(instance_ids[index + 1] if index + 1 < len(segments) else instance_ids[0])),
        )
        for index, segment in enumerate(segments)
    ]
    cyclic = MissionGraphSelection(entry_instance_id=instance_ids[0], nodes=tuple(cyclic_nodes))
    with pytest.raises(VehicleCompositionError, match="cyclic-mission-graph"):
        compile_vehicle_composition(request.model_copy(update={"segments": segments, "mission_graph": cyclic}))
    ####


def test_compose_rejects_a_value_that_violates_the_declared_parameter_space() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_compose.yaml")
    invalid_initialization = request.initialization.model_copy(
        update={
            "inputs": {
                **request.initialization.inputs,
                "heading_deg": CompositionValue(value="north", unit="deg"),
            }
        }
    )

    with pytest.raises(VehicleCompositionError, match="value-space-mismatch"):
        compile_vehicle_composition(request.model_copy(update={"initialization": invalid_initialization}))

    tumbling = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/tumbling_body_direct_release_pseudo6dof_compose.yaml")
    invalid_tumbling_initialization = tumbling.initialization.model_copy(
        update={
            "inputs": {
                **tumbling.initialization.inputs,
                "quaternion_wxyz": CompositionValue(value=[2.0, 0.0, 0.0, 0.0], unit="dimensionless"),
            }
        }
    )
    with pytest.raises(VehicleCompositionError, match="unit norm"):
        compile_vehicle_composition(tumbling.model_copy(update={"initialization": invalid_tumbling_initialization}))

    invalid_turn = request.segments[2].model_copy(update={"inputs": {**request.segments[2].inputs, "turn_direction": CompositionValue(value="up")}})
    with pytest.raises(VehicleCompositionError, match="option-mismatch"):
        compile_vehicle_composition(request.model_copy(update={"segments": (*request.segments[:2], invalid_turn, *request.segments[3:])}))
    ####


def test_a320_runtime_bound_mass_variant_replaces_the_operating_point_input() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/a320_racetrack_mass_variant_3dof_compose.yaml")
    compiled = compile_vehicle_composition(request)

    assert compiled.initialization.inputs["mass_kg"].value == 66000.0
    assert compiled.variant.inputs["operating_mass_kg"].value == 66000.0
    binding = compiled.variant.runtime_bindings["operating_mass_kg"]
    assert binding.runtime_input_path == "A320OpenAPOperatingPoint.mass_kg"
    assert binding.coupling_group == "operating_mass"
    assert binding.coupling_policy == "exclusive"
    assert binding.derived_status_channels == ("resources.mass.total",)
    assert binding.status_derivation_relation == "equal_to_target"
    assert binding.resource_derivation == "runtime_adapter_owned"
    assert binding.compatible_fidelities == ("point_mass_3dof", "pseudo_6dof")
    assert compiled.variant.invalidations == ("requalification:mass_kg", "retrim:mass_kg")
    assert compiled.variant.resolution_policy == "reject_invalid"
    assert compiled.variant.qualification == "extended"
    assert "no declared qualified range" in compiled.variant.qualification_findings[0]

    catalog = load_resolved_vehicle_composition_catalog().vehicle("a320_openap_3dof")
    variant_parameters = catalog.parameter_dict("variant_configuration")
    descriptors = cast(list[dict[str, object]], variant_parameters["parameters"])
    descriptor = descriptors[0]
    assert descriptor["id"] == "operating_mass_kg"
    assert descriptor["status"] == "runnable"
    assert descriptor["hard_lower"] == 42600.0
    assert descriptor["hard_upper"] == 78000.0
    assert descriptor["resolution_policy"] == "reject_invalid"
    assert descriptor["coupling_policy"] == "exclusive"
    assert descriptor["status_derivation_relation"] == "equal_to_target"

    with pytest.raises(VehicleCompositionError, match="variant-target-conflict"):
        compile_vehicle_composition(
            request.model_copy(
                update={
                    "initialization": request.initialization.model_copy(
                        update={"inputs": {**request.initialization.inputs, "mass_kg": CompositionValue(value=60000.0, unit="kg")}}
                    )
                }
            )
        )
    with pytest.raises(VehicleCompositionError, match="variant-hard-bound-violation"):
        compile_vehicle_composition(
            request.model_copy(
                update={
                    "variant": request.variant.model_copy(update={"inputs": {"operating_mass_kg": CompositionValue(value=80000.0, unit="kg")}}),
                }
            )
        )
    ####


def test_variant_projection_is_explicit_when_a_family_opted_into_project_to_valid() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/a320_racetrack_mass_variant_3dof_compose.yaml")
    registry_payload = yaml.safe_load((ROOT / "verification/vehicle_composition_registry.yaml").read_text(encoding="utf-8"))
    assert isinstance(registry_payload, dict)
    vehicles = registry_payload["vehicles"]
    assert isinstance(vehicles, list)
    a320 = next(item for item in vehicles if item["family_id"] == "a320_openap_3dof")
    assert isinstance(a320, dict)
    variants = a320["variant_parameters"]
    assert isinstance(variants, list)
    variants[0]["resolution_policy"] = "project_to_valid"
    catalog = load_resolved_vehicle_composition_catalog(registry=VehicleCompositionRegistry.model_validate(registry_payload))
    projected_request = request.model_copy(
        update={"variant": request.variant.model_copy(update={"inputs": {"operating_mass_kg": CompositionValue(value=80000.0, unit="kg")}})}
    )

    compiled = compile_vehicle_composition(projected_request, catalog=catalog)

    assert compiled.initialization.inputs["mass_kg"].value == 78000.0
    assert compiled.variant.inputs["operating_mass_kg"].value == 78000.0
    assert compiled.variant.resolution_policy == "project_to_valid"
    projection = compiled.variant.projections["operating_mass_kg"]
    assert projection.requested_value == 80000.0
    assert projection.resolved_value == 78000.0
    assert projection.constraints == ("hard_upper",)
    assert projection.normalized_distance > 0.0
    ####


def test_variant_coupling_groups_reject_conflicting_modifiers_unless_every_member_is_composable() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/a320_racetrack_mass_variant_3dof_compose.yaml")
    registry_payload = yaml.safe_load((ROOT / "verification/vehicle_composition_registry.yaml").read_text(encoding="utf-8"))
    assert isinstance(registry_payload, dict)
    vehicles = registry_payload["vehicles"]
    assert isinstance(vehicles, list)
    a320 = next(item for item in vehicles if item["family_id"] == "a320_openap_3dof")
    assert isinstance(a320, dict)
    initialization_contracts = a320["initialization_contracts"]
    assert isinstance(initialization_contracts, list)
    initialization = next(item for item in initialization_contracts if item["id"] == "airborne_trim")
    assert isinstance(initialization, dict)
    parameters = initialization["parameters"]
    assert isinstance(parameters, list)
    parameters.append(
        {
            "id": "payload_mass_kg",
            "canonical_unit": "kg",
            "required": False,
            "description": "Synthetic coupling-policy fixture payload mass.",
        }
    )
    variants = a320["variant_parameters"]
    assert isinstance(variants, list)
    payload_variant = {**variants[0], "id": "payload_mass_kg", "target_parameter_id": "payload_mass_kg"}
    variants.append(payload_variant)
    catalog = load_resolved_vehicle_composition_catalog(registry=VehicleCompositionRegistry.model_validate(registry_payload))
    coupled_request = request.model_copy(
        update={
            "variant": request.variant.model_copy(
                update={
                    "inputs": {
                        "operating_mass_kg": CompositionValue(value=66000.0, unit="kg"),
                        "payload_mass_kg": CompositionValue(value=50000.0, unit="kg"),
                    }
                }
            )
        }
    )

    with pytest.raises(VehicleCompositionError, match="variant-coupling-conflict"):
        compile_vehicle_composition(coupled_request, catalog=catalog)

    variants[0]["coupling_policy"] = "composable"
    variants[1]["coupling_policy"] = "composable"
    composable_catalog = load_resolved_vehicle_composition_catalog(registry=VehicleCompositionRegistry.model_validate(registry_payload))
    compiled = compile_vehicle_composition(coupled_request, catalog=composable_catalog)

    assert set(compiled.variant.inputs) == {"operating_mass_kg", "payload_mass_kg"}
    assert compiled.variant.runtime_bindings["payload_mass_kg"].coupling_policy == "composable"
    ####


def test_hummingbird_runtime_bound_grounded_mass_variant_has_a_hover_capacity_hard_limit() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/hummingbird_grounded_mass_variant_pseudo6dof_compose.yaml")
    compiled = compile_vehicle_composition(request)

    assert compiled.initialization.inputs["mass_kg"].value == 0.55
    binding = compiled.variant.runtime_bindings["grounded_operating_mass_kg"]
    assert binding.runtime_adapter_id == ("taoryx.multirotor.aggregate_thrust_pseudo6dof.v1")
    assert binding.coupling_group == "aggregate_thrust_hover_mass"
    assert binding.transform == "log"
    assert binding.value_space["topology"] == "positive_half_line"
    assert binding.hard_lower == 0.000001
    assert binding.hard_upper == 5.1112
    assert binding.resource_derivation == "not_represented"
    assert binding.compatible_fidelities == ("pseudo_6dof",)
    descriptor = load_resolved_vehicle_composition_catalog().vehicle("hummingbird").parameter_dict("variant_configuration")
    parameters = cast(list[dict[str, object]], descriptor["parameters"])
    mass = next(item for item in parameters if item["id"] == "grounded_operating_mass_kg")
    assert mass["hard_upper"] == 5.1112
    assert mass["qualified_lower"] is None
    assert mass["qualified_upper"] is None

    with pytest.raises(VehicleCompositionError, match="variant-fidelity-incompatible"):
        compile_vehicle_composition(request.model_copy(update={"fidelity": "point_mass_3dof"}))

    with pytest.raises(VehicleCompositionError, match="variant-hard-bound-violation"):
        compile_vehicle_composition(
            request.model_copy(
                update={"variant": request.variant.model_copy(update={"inputs": {"grounded_operating_mass_kg": CompositionValue(value=5.2, unit="kg")}})}
            )
        )
    ####


def test_vehicle_compose_cli_writes_reproducible_output(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    output = tmp_path / "composition.json"

    assert main(["vehicle", "compose", str(ROOT / "examples/vehicle_composition/x8_racetrack_compose.yaml"), "--output", str(output)]) == 0
    emitted = json.loads(capsys.readouterr().out)
    written = json.loads(output.read_text(encoding="utf-8"))
    assert written == emitted
    assert emitted["mission"] == "powered_fixed_wing_racetrack_v1"
    ####


def test_vehicle_materialize_requires_preflight_ready_geometry(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    composition_path = tmp_path / "composition.json"
    output_dir = tmp_path / "native-inputs"
    request = ROOT / "examples/vehicle_composition/x8_racetrack_capability_compose.yaml"
    assert main(["vehicle", "compose", str(request), "--output", str(composition_path)]) == 0
    _ = capsys.readouterr()

    assert main(["vehicle", "materialize", str(composition_path), "--output-dir", str(output_dir)]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["source_mission_id"] == "x8-racetrack-altitude-turns-pseudo-6dof-v1"
    assert payload["proposal"]["derived"]["selected_straight_length_m"] == 1000.0
    assert Path(payload["inputs"]["problem"]).is_file()
    assert Path(payload["inputs"]["mission_config"]).is_file()
    assert Path(payload["inputs"]["racetrack_config"]).is_file()
    ####


def test_vehicle_run_executes_x8_translation_ready_composition(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    composition_path = tmp_path / "composition.json"
    output_dir = tmp_path / "execution"
    request = ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml"

    assert main(["vehicle", "compose", str(request), "--output", str(composition_path)]) == 0
    _ = capsys.readouterr()
    assert main(["vehicle", "run", str(composition_path), "--output-dir", str(output_dir)]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["mission_pass"] is True
    assert payload["preflight"]["status"] == "translation_ready"
    assert payload["truth_evaluation"]["mission_pass"] is True
    assert (output_dir / "truth_telemetry.csv").is_file()
    evaluation = json.loads((output_dir / "evaluation.json").read_text(encoding="utf-8"))
    assert evaluation["outcome"] == "completed"
    assert evaluation["scenario_contract_sha256"] == payload["composition"]["identity_sha256"]
    assert (output_dir / "objective_report.json").is_file()
    assert (output_dir / "execution.json").is_file()
    graph_execution = json.loads((output_dir / "mission_graph_execution.json").read_text(encoding="utf-8"))
    assert graph_execution["schema"] == "taoryx.mission-graph-execution/v1alpha1"
    assert graph_execution["observation_status"] == "unobserved"
    assert graph_execution["completed_nominal_success_path"] is None
    control_provenance = json.loads((output_dir / "control_provenance.json").read_text(encoding="utf-8"))
    assert control_provenance["schema"] == "taoryx.runtime-control-provenance/v1alpha1"
    assert "not a semantic_action_trace" in control_provenance["claim_boundary"]
    assert payload["control_provenance"]["detail"] == "intervals"
    action_trace = json.loads((output_dir / "semantic_action_trace.json").read_text(encoding="utf-8"))
    assert action_trace["schema"] == "taoryx.composition-semantic-action-trace/v1alpha1"
    assert action_trace["requested_action_channels"]
    interface = json.loads((output_dir / "vehicle_interface.json").read_text(encoding="utf-8"))
    assert payload["vehicle_interface_artifact"]["fingerprint_sha256"] == interface["fingerprint_sha256"]
    assert interface["schema"] == "taoryx.vehicle-interface/v1alpha1"
    assert interface["validation"] == {"status": "pass", "findings": []}
    ####


def test_vehicle_run_executes_b747_point_mass_translation_ready_composition(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    composition_path = tmp_path / "b747-composition.json"
    output_dir = tmp_path / "b747-execution"
    request = ROOT / "examples/vehicle_composition/b747_racetrack_capability_3dof_compose.yaml"

    assert main(["vehicle", "compose", str(request), "--output", str(composition_path)]) == 0
    _ = capsys.readouterr()
    assert main(["vehicle", "run", str(composition_path), "--output-dir", str(output_dir)]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["mission_pass"] is True
    assert payload["composition"]["fidelity"] == "point_mass_3dof"
    assert payload["preflight"]["status"] == "translation_ready"
    assert payload["truth_evaluation"]["mission_pass"] is True
    assert (output_dir / "truth_telemetry.csv").is_file()
    assert (output_dir / "objective_report.json").is_file()
    assert (output_dir / "control_provenance.json").is_file()
    action_trace = json.loads((output_dir / "semantic_action_trace.json").read_text(encoding="utf-8"))
    assert action_trace["schema"] == "taoryx.composition-semantic-action-trace/v1alpha1"
    assert action_trace["requested_action_channels"] == [
        "control.longitudinal.bridge.command",
        "guidance.flight_path_angle.command",
        "guidance.heading.command",
        "guidance.override.enabled",
        "guidance.speed.command",
        "propulsion.command.fraction",
    ]
    interface = json.loads((output_dir / "vehicle_interface.json").read_text(encoding="utf-8"))
    assert interface["fidelity"] == "point_mass_3dof"
    assert interface["validation"] == {"status": "pass", "findings": []}
    ####


def test_x8_sensor_composition_writes_a_no_interpolation_batch_trace(tmp_path: Path) -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_sensor_episode_3dof_compose.yaml")
    )

    result = execute_powered_fixed_wing_composition(
        composition,
        tmp_path / "x8-sensor-execution",
        max_steps=3,
    )

    assert result.sensor_trace is not None
    assert result.sensor_trace["interpolation"] == "forbidden"
    samples = cast(list[dict[str, object]], result.sensor_trace["samples"])
    assert samples[0]["source_time_s"] is None
    assert samples[1]["source_time_s"] == pytest.approx(0.0)
    assert (result.output_dir / "sensor_observations.json").is_file()
    action_trace = result.semantic_action_trace
    assert action_trace is not None
    assert action_trace["schema"] == "taoryx.composition-semantic-action-trace/v1alpha1"
    assert action_trace["requested_action_channels"] == [
        "control.lateral.bridge.command",
        "control.longitudinal.bridge.command",
        "guidance.flight_path_angle.command",
        "guidance.heading.command",
        "guidance.override.enabled",
        "guidance.speed.command",
        "propulsion.command.fraction",
    ]
    trace_samples = cast(list[dict[str, object]], action_trace["samples"])
    assert len(trace_samples) == 3
    assert all(sample["achieved_effectors"] == {} for sample in trace_samples)
    assert all(set(cast(dict[str, object], sample["requested_actions"])) == set(action_trace["requested_action_channels"]) for sample in trace_samples)
    cases = cast(list[dict[str, object]], result.control_provenance["cases"])
    assert len(cases) == 1
    vehicles = cast(dict[str, dict[str, object]], cases[0]["vehicles"])
    assert len(vehicles) == 1
    vehicle = next(iter(vehicles.values()))
    assert vehicle["committed_resolver_control_names"] == [
        "_command_gamgd",
        "_command_psi",
        "_command_vel",
        "guidance_override_active",
    ]
    intervals = cast(list[dict[str, object]], vehicle["control_interval_records"])
    assert intervals
    assert all(interval["solver_stage_control_mutation_detected"] is False for interval in intervals)
    assert all(
        {"_command_gamgd", "_command_psi", "_command_vel"}.issubset(cast(dict[str, float], interval["controls_at_interval_start"])) for interval in intervals
    )
    assert (result.output_dir / "semantic_action_trace.json").is_file()
    ####


@pytest.mark.parametrize(
    ("request_name", "expected_action_channels"),
    (
        (
            "x8_racetrack_capability_compose.yaml",
            (
                "control.lateral.bridge.command",
                "control.longitudinal.bridge.command",
                "guidance.bank.command",
                "guidance.flight_path_angle.command",
                "guidance.heading.command",
                "guidance.override.enabled",
                "guidance.speed.command",
                "propulsion.command.fraction",
            ),
        ),
        (
            "b747_racetrack_capability_3dof_compose.yaml",
            (
                "control.longitudinal.bridge.command",
                "guidance.flight_path_angle.command",
                "guidance.heading.command",
                "guidance.override.enabled",
                "guidance.speed.command",
                "propulsion.command.fraction",
            ),
        ),
        (
            "b747_racetrack_capability_pseudo6dof_compose.yaml",
            (
                "control.longitudinal.bridge.command",
                "guidance.bank.command",
                "guidance.flight_path_angle.command",
                "guidance.heading.command",
                "guidance.override.enabled",
                "guidance.speed.command",
                "propulsion.command.fraction",
            ),
        ),
    ),
)
def test_language_backed_batch_emits_registered_bridge_actions(
    tmp_path: Path,
    request_name: str,
    expected_action_channels: tuple[str, ...],
) -> None:
    """Every binding-promoted language path has a complete held-action trace."""

    composition = compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / request_name))

    result = execute_powered_fixed_wing_composition(composition, tmp_path / request_name, max_steps=3)

    assert result.semantic_action_trace is not None
    assert tuple(result.semantic_action_trace["requested_action_channels"]) == expected_action_channels
    assert result.control_provenance["detail"] == "intervals"
    assert (result.output_dir / "control_provenance.json").is_file()
    assert (result.output_dir / "semantic_action_trace.json").is_file()
    ####


def test_vehicle_run_executes_a320_runtime_bound_mass_variant(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    composition_path = tmp_path / "composition.json"
    output_dir = tmp_path / "execution"
    request = ROOT / "examples/vehicle_composition/a320_racetrack_mass_variant_3dof_compose.yaml"

    assert main(["vehicle", "compose", str(request), "--output", str(composition_path)]) == 0
    _ = capsys.readouterr()
    assert main(["vehicle", "run", str(composition_path), "--output-dir", str(output_dir)]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["mission_pass"] is True
    assert payload["runtime"]["adapter_id"] == "taoryx.fixed_wing.openap.v1"
    assert payload["runtime"]["native_operating_point"]["mass_kg"] == 66000.0
    variant_evidence = payload["runtime"]["variant_runtime_evidence"]
    assert variant_evidence["status"] == "pass"
    assert variant_evidence["bindings"][0]["consumed_native_input_match"] is True
    assert variant_evidence["bindings"][0]["status_channels"][0]["relation_match"] is True
    assert payload["composition"]["variant"]["runtime_bindings"]["operating_mass_kg"]["runtime_input_path"] == "A320OpenAPOperatingPoint.mass_kg"
    assert "trim" in payload
    assert "provenance" in payload
    assert (output_dir / "trim.json").is_file()
    assert (output_dir / "model_provenance.json").is_file()
    assert (output_dir / "variant_runtime_evidence.json").is_file()
    ####


@pytest.mark.parametrize(
    ("request_name", "expected_family"),
    (
        ("a320_racetrack_capability_3dof_compose.yaml", "a320_openap_3dof"),
        ("a320_racetrack_capability_pseudo6dof_compose.yaml", "a320_openap_3dof"),
    ),
)
def test_reduced_a320_compositions_execute_the_common_racetrack(
    tmp_path: Path,
    request_name: str,
    expected_family: str,
) -> None:
    composition = compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / request_name))

    result = execute_a320_reduced_composition(composition, tmp_path / request_name.removesuffix(".yaml"))

    assert result.composition.family_id == expected_family
    assert result.mission_pass is True
    assert result.truth_evaluation["required_passed"] == 4
    evaluation = json.loads((result.output_dir / "evaluation.json").read_text(encoding="utf-8"))
    resources = {item["id"]: item for item in evaluation["resources"]}
    assert resources["resources.mass.total"]["status"] == "available"
    status_channels = cast(list[str], result.status_trace["channels"])
    assert {"position.north", "position.east", "position.altitude", "velocity.speed", "resources.mass.total"} <= set(status_channels)
    status_samples = cast(list[dict[str, object]], result.status_trace["samples"])
    first_status = cast(dict[str, object], status_samples[0]["values"])
    final_status = cast(dict[str, object], status_samples[-1]["values"])
    assert resources["resources.mass.total"]["value"] == pytest.approx(float(final_status["resources.mass.total"]))
    requested_controls = {item["id"]: item for item in evaluation["requested_controls"]}
    assert requested_controls["requested.guidance.speed.command"]["status"] == "available"
    assert requested_controls["requested.guidance.bank.command"]["status"] == "available"
    heading = requested_controls["requested.guidance.heading.command"]["value"]
    assert isinstance(heading, int | float)
    assert 0.0 <= heading < 360.0
    assert (result.output_dir / "semantic_action_trace.json").is_file()
    speed = first_status["velocity.speed"]
    mass = first_status["resources.mass.total"]
    assert isinstance(speed, int | float)
    assert isinstance(mass, int | float)
    assert float(speed) > 0.0
    assert float(mass) > 0.0
    if composition.fidelity == "pseudo_6dof":
        assert "attitude.euler" in first_status
        assert "body_rate" in first_status
    assert (result.output_dir / "truth_telemetry.csv").is_file()
    assert (result.output_dir / "execution.json").is_file()
    graph_execution = json.loads((result.output_dir / "mission_graph_execution.json").read_text(encoding="utf-8"))
    assert graph_execution["observation_status"] == "unobserved"
    ####


def test_runtime_lowering_never_falls_back_to_another_family_adapter() -> None:
    composition = compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_compose.yaml"))

    result = lower_vehicle_composition(composition)

    assert result.status == "blocked"
    assert result.adapter_descriptor is None
    assert "skywalker_x8" in result.diagnostics[0]
    ####


def test_x8_legacy_semantic_example_is_blocked_when_its_route_does_not_match_the_native_translator() -> None:
    composition = compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_compose.yaml"))

    result = preflight_vehicle_composition(composition)

    assert result.status == "blocked"
    failed = {check.id for check in result.checks if not check.passed}
    assert "initialization.heading" in failed
    assert "left_turn.radius" in failed
    assert "right_turn.exit_gate" in failed
    ####


def test_x8_capability_derived_composition_is_ready_for_native_route_translation() -> None:
    for path in (
        ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml",
        ROOT / "examples/vehicle_composition/x8_racetrack_capability_compose.yaml",
    ):
        composition = compile_vehicle_composition(load_vehicle_composition_request(path))
        result = preflight_vehicle_composition(composition)

        assert result.status == "translation_ready"
        assert all(check.passed for check in result.checks)
        assert result.translator_id == "taoryx.x8_racetrack.source_route.v1"
    ####


def test_b747_capability_derived_composition_reuses_the_fixed_wing_translator(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/b747_racetrack_capability_pseudo6dof_compose.yaml")
    )

    result = preflight_vehicle_composition(composition)
    assert result.status == "translation_ready"
    assert all(check.passed for check in result.checks)
    assert result.derived_mission is not None
    assert result.derived_mission["resolved_route"]["vehicle_id"] == "b747"

    output_dir = tmp_path / "native-inputs"
    assert (
        main(
            [
                "vehicle",
                "compose",
                str(ROOT / "examples/vehicle_composition/b747_racetrack_capability_pseudo6dof_compose.yaml"),
                "--output",
                str(tmp_path / "composition.json"),
            ]
        )
        == 0
    )
    _ = capsys.readouterr()
    assert main(["vehicle", "materialize", str(tmp_path / "composition.json"), "--output-dir", str(output_dir)]) == 0
    materialized = json.loads(capsys.readouterr().out)
    assert materialized["source_mission_id"] == "b747-racetrack-altitude-turns-pseudo-6dof-v1"
    assert materialized["proposal"]["derived"]["selected_straight_length_m"] == 30000.0
    assert Path(materialized["inputs"]["problem"]).is_file()
    ####


def test_runtime_lowering_binds_an_available_source_adapter() -> None:
    x15_composition = compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x15_direct_wrench_compose.yaml"))

    result = lower_vehicle_composition(x15_composition)

    assert result.status == "adapter_bound"
    assert result.adapter_descriptor is not None
    assert result.adapter_descriptor["adapter_id"] == "taoryx.high_energy.fixed_wing.v1"
    assert result.translator_status == "not_applicable"
    ####


@pytest.mark.parametrize("family_id", ("skywalker_x8", "b747", "hummingbird", "f16_s119"))
def test_runtime_registry_owns_source_table_adapters_and_operation_probes(family_id: str) -> None:
    """Developer LQR scripts and Mission Composition use the same physical plant seam."""

    registry = build_vehicle_runtime_adapter_registry()
    for tier in ("rigid_body_6dof_direct_wrench", "rigid_body_6dof_surface_allocated"):
        check = registry.check(family_id, tier)

        assert check.status == "pass"
        assert check.conformance is not None
        assert check.probe is not None
        operations = {item.operation: item.status for item in check.probe.operations}
        assert operations["state_derivative"] == "pass"
        assert operations["trim"] == "pass"
        assert operations["linearize"] == "pass"
        if tier == "rigid_body_6dof_surface_allocated":
            assert operations["effectiveness"] == "pass"
            assert operations["allocate"] == "pass"
        else:
            assert operations["effectiveness"] == "not_applicable"
            assert operations["allocate"] == "not_applicable"
    ####


def test_runtime_registry_fails_closed_for_unadvertised_lower_tiers() -> None:
    """A local 6-DOF factory must not masquerade as a lower-fidelity product."""

    registry = build_vehicle_runtime_adapter_registry()
    for family_id in ("skywalker_x8", "b747", "hummingbird", "f16_s119"):
        with pytest.raises(AdapterRegistrationError, match="tier"):
            registry.build(family_id, "pseudo_6dof")
    ####
