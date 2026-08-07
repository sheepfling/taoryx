from __future__ import annotations

import pytest

from taoryx.fidelity_contracts import CANONICAL_FIDELITY_TIERS
from taoryx.language import parse_problem_text
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.trajectory.configuration_contract import (
    ConfigurationChoiceSchema,
    ConfigurationChoiceValue,
    ConfigurationContractError,
    ConfigurationGroupSchema,
    ConfigurationGroupValue,
    ConfigurationOptionalValue,
    ConfigurationParameterSchema,
    ConfigurationParameterValue,
    ConfigurationPeriodicity,
    ConfigurationSequenceSchema,
    ConfigurationSequenceValue,
    TrajectoryConfigurationInstance,
    TrajectoryConfigurationSchema,
    render_configuration_schema,
    validate_configuration_instance,
)
from taoryx.trajectory.mission_composition import ExampleMissionCompositionProvider
from taoryx.trajectory.registry_mission_composition import RegistryMissionCompositionProvider
from taoryx.trajectory.simple_aero_mission_composition import build_simple_aero_prepared_configuration
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog


def _parameter(value: object, unit: str | None) -> ConfigurationParameterValue:
    return ConfigurationParameterValue(value=value, unit=unit)
    ####


def _tumbling_configuration(
    provider: RegistryMissionCompositionProvider,
    *,
    segment_order: tuple[str, ...] = ("passive_coast", "atmospheric_descent"),
    fingerprint: str | None = None,
) -> TrajectoryConfigurationInstance:
    schema = provider.get_model_schema("tumbling_body")
    segment_parameters = {
        "passive_coast": ConfigurationGroupValue(
            values={"duration_s": _parameter(10.0, "s")},
        ),
        "atmospheric_descent": ConfigurationGroupValue(
            values={"impact_plane_altitude_m": _parameter(0.0, "m")},
        ),
    }
    return TrajectoryConfigurationInstance(
        configuration_id="tumbling-example",
        model_id="tumbling_body",
        model_version=schema.model_version,
        schema_fingerprint=fingerprint or schema.fingerprint,
        fidelity="point_mass_3dof",
        root=ConfigurationGroupValue(
            values={
                "initialization": ConfigurationChoiceValue(
                    selected="atmospheric_release",
                    value=ConfigurationGroupValue(
                        values={
                            "altitude_m": _parameter(1000.0, "m"),
                            "speed_m_s": _parameter(120.0, "m/s"),
                            "body_rates_rad_s": _parameter((0.1, 0.2, 0.3), "rad/s"),
                            "area_policy": _parameter("orientation_averaged_projected_area", None),
                        }
                    ),
                ),
                "segments": ConfigurationSequenceValue(
                    items=tuple(ConfigurationChoiceValue(selected=item, value=segment_parameters[item]) for item in segment_order),
                ),
            }
        ),
    )
    ####


def _simple_aero_configuration(
    provider: RegistryMissionCompositionProvider,
    *,
    aimpoint_longitude_deg: float = -5.60999,
    maneuver: str = "weave",
) -> TrajectoryConfigurationInstance:
    schema = provider.get_model_schema("simple_aero")
    empty = ConfigurationGroupValue(values={})
    maneuver_parameters = (
        ConfigurationGroupValue(
            values={
                "end_range_to_go_m": ConfigurationOptionalValue(
                    enabled=True,
                    value=_parameter(200_000.0, "m"),
                )
            }
        )
        if maneuver == "weave"
        else empty
    )
    return TrajectoryConfigurationInstance(
        configuration_id="simple-aero-weave-example",
        model_id="simple_aero",
        model_version=schema.model_version,
        schema_fingerprint=schema.fingerprint,
        fidelity="point_mass_3dof",
        root=ConfigurationGroupValue(
            values={
                "launch_state": ConfigurationGroupValue(
                    values={
                        "launch.latitude_deg": _parameter(35.8766, "deg"),
                        "launch.longitude_deg": _parameter(14.4425, "deg"),
                        "mission.initial_speed": _parameter(10.0, "m/s"),
                    }
                ),
                "endpoint": ConfigurationChoiceValue(
                    selected="geodetic_aimpoint",
                    value=ConfigurationGroupValue(
                        values={
                            "aimpoint.latitude_deg": _parameter(36.000975, "deg"),
                            "aimpoint.longitude_deg": _parameter(aimpoint_longitude_deg, "deg"),
                        }
                    ),
                ),
                "endpoint_state": empty,
                "vehicle_surrogate": ConfigurationGroupValue(
                    values={"vehicle.mass.initial": _parameter(1000.0, "kg")}
                ),
                "trajectory_checkpoints": ConfigurationGroupValue(
                    values={"mission.burnout_speed": _parameter(1200.0, "m/s")}
                ),
                "segments": ConfigurationSequenceValue(
                    items=(
                        ConfigurationChoiceValue(selected="powered_ascent", value=empty),
                        ConfigurationChoiceValue(selected="ballistic_coast", value=empty),
                        ConfigurationChoiceValue(
                            selected=maneuver,
                            value=maneuver_parameters,
                        ),
                        ConfigurationChoiceValue(selected="terminal_pronav", value=empty),
                    )
                ),
                "runtime": empty,
            }
        ),
    )
    ####


def test_registry_provider_advertises_every_canonical_vehicle_family_and_registry_ready_workflow() -> None:
    provider = RegistryMissionCompositionProvider()
    canonical = {item.family.family_id for item in load_resolved_vehicle_composition_catalog().vehicles}
    expected = canonical | {"simple_aero", "dual_launch_glider"}

    assert provider.metadata.model_count == 11
    assert {item.id for item in provider.list_models()} == expected
    assert all(provider.model(item).model_kind == "canonical_vehicle_family" for item in canonical)
    assert provider.model("simple_aero").model_kind == "trajectory_workflow"
    assert all(item.configuration_schema_fingerprint == provider.get_model_schema(item.id).fingerprint for item in provider.list_models())
    ####


def test_simple_aero_advertises_endpoints_parameters_segments_and_fidelity_boundary() -> None:
    provider = RegistryMissionCompositionProvider()
    model = provider.model("simple_aero")
    schema = provider.get_model_schema("simple_aero")

    assert isinstance(schema.root, ConfigurationGroupSchema)
    endpoint = next(item for item in schema.root.children if item.id == "endpoint")
    assert isinstance(endpoint, ConfigurationChoiceSchema)
    assert {item.id for item in endpoint.variants} == {"range_bearing", "geodetic_aimpoint"}

    vehicle = next(item for item in schema.root.children if item.id == "vehicle_surrogate")
    assert isinstance(vehicle, ConfigurationGroupSchema)
    mass = next(item for item in vehicle.children if item.id == "vehicle.mass.initial")
    assert isinstance(mass, ConfigurationParameterSchema)
    assert mass.quantity == "mass"
    assert mass.canonical_unit == "kg"
    assert mass.default == pytest.approx(1000.0)

    segments = next(item for item in schema.root.children if item.id == "segments")
    assert isinstance(segments, ConfigurationSequenceSchema)
    assert segments.allow_custom
    assert segments.minimum_items == 1
    assert {item.id for item in segments.templates} >= {"ballistic", "phugoid", "weave", "fixed_ld_baseline"}
    assert next(item for item in segments.templates if item.id == "weave").item_variants == (
        "powered_ascent",
        "ballistic_coast",
        "weave",
        "terminal_pronav",
    )

    assert [item.declared for item in model.fidelities] == [True, False, False, False]
    assert all(item.status == "not_available" for item in model.fidelity_transitions)
    baseline = next(item for item in model.mission_templates if item.id == "fixed_ld_baseline")
    weave = next(item for item in model.mission_templates if item.id == "weave")
    assert next(item for item in baseline.operations if item.operation == "batch").status == "available"
    assert next(item for item in weave.operations if item.operation == "batch").status == "blocked"
    ####


def test_simple_aero_weave_configuration_validates_with_per_segment_defaults() -> None:
    provider = RegistryMissionCompositionProvider()
    prepared = provider.validate_configuration(_simple_aero_configuration(provider))

    assert prepared.resolved["launch_state"]["mission.initial_speed"] == pytest.approx(10.0)
    assert prepared.resolved["vehicle_surrogate"]["vehicle.mass.initial"] == pytest.approx(1000.0)
    assert prepared.resolved["segments"][2]["selected"] == "weave"
    assert prepared.resolved["segments"][2]["value"]["minimum_time_to_go_s"] == pytest.approx(60.0)
    assert prepared.resolved["segments"][2]["value"]["end_range_to_go_m"] == pytest.approx(200_000.0)
    ####


def test_simple_aero_fixed_ld_baseline_compiles_from_geodetic_configuration() -> None:
    provider = RegistryMissionCompositionProvider()
    prepared = provider.validate_configuration(_simple_aero_configuration(provider, maneuver="bank_maneuver"))
    build = build_simple_aero_prepared_configuration(prepared)
    document = parse_problem_text(build.problem_text, profile=GrammarProfile.TAORYX)

    assert not [item for item in document.diagnostics if item.severity.value == "error"]
    assert build.derived.target_latitude_deg == pytest.approx(36.000975)
    assert build.derived.target_longitude_deg == pytest.approx(-5.60999)
    assert build.derived.target_range_m > 0.0
    assert "*segment 3 bank-maneuver" in build.problem_text
    ####


def test_simple_aero_builder_rejects_fixture_template_without_execution_adapter() -> None:
    provider = RegistryMissionCompositionProvider()
    prepared = provider.validate_configuration(_simple_aero_configuration(provider))

    with pytest.raises(ConfigurationContractError, match="execution-template-unavailable"):
        build_simple_aero_prepared_configuration(prepared)
    ####


def test_simple_aero_rejects_noncanonical_geodetic_endpoint_longitude() -> None:
    provider = RegistryMissionCompositionProvider()

    with pytest.raises(ConfigurationContractError, match="noncanonical-periodic-value"):
        provider.validate_configuration(_simple_aero_configuration(provider, aimpoint_longitude_deg=180.0))
    ####


def test_analytical_examples_publish_the_same_common_schema_contract() -> None:
    provider = ExampleMissionCompositionProvider()

    assert {item.id for item in provider.list_models()} == {
        "reference_ballistic_3dof",
        "reference_constant_velocity_waypoint_3dof",
    }
    waypoint = provider.get_model_schema("reference_constant_velocity_waypoint_3dof")
    assert isinstance(waypoint.root, ConfigurationGroupSchema)
    segments = next(item for item in waypoint.root.children if item.id == "segments")
    assert isinstance(segments, ConfigurationSequenceSchema)
    assert segments.allow_custom
    assert segments.maximum_items is None
    ####


def test_common_model_metadata_keeps_fidelity_evidence_and_transitions_separate() -> None:
    provider = RegistryMissionCompositionProvider()
    skywalker = provider.model("skywalker_x8")
    tumbling = provider.model("tumbling_body")

    assert tuple(item.id for item in skywalker.fidelities) == CANONICAL_FIDELITY_TIERS
    assert len(skywalker.fidelity_transitions) == 6
    assert not any(item.automatic for item in skywalker.fidelity_transitions if item.direction == "step_up")
    assert all(item.selection_policy == "validated_lower_only" for item in skywalker.fidelity_transitions if item.direction == "step_down")
    assert all(item.state_transfer.startswith("not_advertised") for item in skywalker.fidelity_transitions)
    unavailable = next(item for item in tumbling.fidelities if item.id == "rigid_body_6dof_direct_wrench")
    assert not unavailable.declared
    assert any(item.status == "not_available" for item in tumbling.fidelity_transitions)

    nesc_mission = provider.model("reference_nesc_two_stage_rocket").mission_templates[0]
    point_mass_operations = {item.operation: item.status for item in nesc_mission.operations if item.fidelity == "point_mass_3dof"}
    assert point_mass_operations == {"validate": "available", "batch": "available", "step": "blocked"}
    ####


def test_registry_schema_publishes_topology_units_choices_and_closed_templates() -> None:
    provider = RegistryMissionCompositionProvider()
    schema = provider.get_model_schema("reference_nesc_two_stage_rocket")
    assert isinstance(schema.root, ConfigurationGroupSchema)
    segments = next(item for item in schema.root.children if item.id == "segments")
    assert isinstance(segments, ConfigurationSequenceSchema)
    assert not segments.allow_custom
    assert segments.templates[0].item_variants == (
        "ignition_ascent",
        "stage_separation",
        "ignition_ascent",
        "coast_terminal_state",
    )
    terminal_variant = next(item for item in segments.item.variants if item.id == "coast_terminal_state")
    assert isinstance(terminal_variant.node, ConfigurationGroupSchema)
    terminal_kind = next(item for item in terminal_variant.node.children if item.id == "terminal_kind")
    assert isinstance(terminal_kind, ConfigurationParameterSchema)
    assert terminal_kind.choices == ("apogee", "impact", "reentry", "orbit")

    rendered = render_configuration_schema(schema)
    assert "Mission Configuration [group]" in rendered
    assert "Coast Terminal State [variant]" in rendered
    round_tripped = TrajectoryConfigurationSchema.model_validate_json(schema.model_dump_json(by_alias=True))
    assert round_tripped.fingerprint == schema.fingerprint
    ####


def test_registry_provider_validates_and_fingerprints_a_typed_configuration() -> None:
    provider = RegistryMissionCompositionProvider()
    prepared = provider.validate_configuration(_tumbling_configuration(provider))

    assert prepared.configuration.model_id == "tumbling_body"
    assert prepared.resolved["initialization"]["selected"] == "atmospheric_release"
    assert len(prepared.fingerprint) == 64
    assert prepared == provider.validate_configuration(_tumbling_configuration(provider))
    ####


def test_registry_provider_rejects_stale_schema_and_unsupported_segment_order() -> None:
    provider = RegistryMissionCompositionProvider()
    with pytest.raises(ConfigurationContractError, match="schema-fingerprint-mismatch"):
        provider.validate_configuration(_tumbling_configuration(provider, fingerprint="0" * 64))
    with pytest.raises(ConfigurationContractError, match="unsupported-sequence"):
        provider.validate_configuration(
            _tumbling_configuration(
                provider,
                segment_order=("atmospheric_descent", "passive_coast"),
            )
        )
    ####


def test_periodic_parameter_validation_is_distinct_from_ordinary_bounds() -> None:
    schema = TrajectoryConfigurationSchema(
        model_id="periodic-example",
        model_version="1",
        supported_fidelities=("point_mass_3dof",),
        root=ConfigurationParameterSchema(
            id="heading",
            label="Heading",
            description="Canonical heading.",
            canonical_unit="deg",
            role="initialization",
            required=True,
            periodicity=ConfigurationPeriodicity(period=360.0, canonical_minimum=0.0),
        ),
        claim_boundary="Configuration validation only.",
    )
    valid = TrajectoryConfigurationInstance(
        configuration_id="heading-0",
        model_id="periodic-example",
        model_version="1",
        schema_fingerprint=schema.fingerprint,
        fidelity="point_mass_3dof",
        root=_parameter(359.0, "deg"),
    )
    assert validate_configuration_instance(schema, valid).resolved == pytest.approx(359.0)

    noncanonical = valid.model_copy(update={"root": _parameter(360.0, "deg")})
    with pytest.raises(ConfigurationContractError, match="noncanonical-periodic-value"):
        validate_configuration_instance(schema, noncanonical)
    wrong_unit = valid.model_copy(update={"root": _parameter(1.0, "rad")})
    with pytest.raises(ConfigurationContractError, match="unit-mismatch"):
        validate_configuration_instance(schema, wrong_unit)
    ####
