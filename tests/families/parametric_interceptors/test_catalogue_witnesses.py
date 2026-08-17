"""Three prototype catalogue records exercising distinct resolver paths."""

from __future__ import annotations

from pathlib import Path

from taoryx_parametric_interceptors import (
    ParametricInterceptorMissionCompositionProvider,
    ResolutionStatus,
    ValueOrigin,
    aim9x_block2_profile,
    aim120_c5_c7_profile,
    load_catalogue_interceptor_record,
    pac3_mse_profile,
    resolve_interceptor,
    runnable_prototype_profiles,
)

from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunRequest,
    MissionCompositionTrajectoryResponse,
    audit_provider_advertisement,
)

_EXAMPLES = Path("examples/parametric_interceptors")


def test_aim9x_keeps_classified_performance_out_of_numeric_parameters() -> None:
    profile = aim9x_block2_profile()
    resolved = resolve_interceptor(profile)

    assert profile.catalogue_interceptor_id == "int:us-aim9x-block2"
    assert resolved.resolution_status is ResolutionStatus.SOURCE_DOMINANT
    assert resolved.parameters["launch_mass_kg"].origin is ValueOrigin.OBSERVED
    assert resolved.number("launch_mass_kg") == 84.37
    assert resolved.number("reference_area_m2") == 0.01327
    gaps = {item.parameter_id: item.status for item in resolved.evidence_gaps}
    assert gaps["reported_max_speed_mps"] == "classified"
    assert gaps["reported_max_range_m"] == "classified"
    assert gaps["thrust_profile_schedule"] == "resolver_input_missing"
    assert "reported_max_speed_mps" not in resolved.parameters
    assert "reported_max_range_m" not in resolved.parameters
    assert resolved.parameters["burnout_mass_kg"].origin is ValueOrigin.ARCHETYPE_ASSUMPTION
    assert resolved.parameters["propellant_fraction"].origin is ValueOrigin.ARCHETYPE_ASSUMPTION
    assert resolved.text("guidance_family") == "infrared_homing_with_datalink"
    assert resolved.parameters["guidance_family"].origin is ValueOrigin.OBSERVED
    assert resolved.text("guidance_archetype") == "proportional_navigation"
    assert resolved.parameters["guidance_archetype"].origin is ValueOrigin.ARCHETYPE_ASSUMPTION
    assert resolved.number("navigation_constant") == 3.5
    ####


def test_aim120_retains_source_units_and_family_interval_without_flattening() -> None:
    profile = aim120_c5_c7_profile()
    resolved = resolve_interceptor(profile)
    mass = resolved.parameters["launch_mass_kg"]

    assert profile.model_id == "aim120-amraam-c5-c7"
    assert profile.variant_basis == "AIM-120C5_or_C7"
    assert mass.origin is ValueOrigin.OBSERVED
    assert mass.source_value is not None
    assert mass.source_value.value == 356
    assert mass.source_value.unit == "lb"
    assert resolved.number("launch_mass_kg") == 161.48
    assert len(resolved.evidence_intervals) == 1
    interval = resolved.evidence_intervals[0]
    assert interval.parameter_id == "launch_mass_kg"
    assert (interval.minimum_value, interval.maximum_value, interval.unit) == (157.85, 162.39, "kg")
    assert "family_identity_narrowed_to_c5_or_c7" in resolved.required_diagnostics
    assert "thrust_profile_schedule" in {item.parameter_id for item in resolved.evidence_gaps}
    assert resolved.text("guidance_family") == "active_radar"
    assert resolved.text("guidance_archetype") == "proportional_navigation"
    assert resolved.number("navigation_constant") == 3.0
    ####


def test_pac3_remains_an_archetype_dominant_resolver_witness() -> None:
    profile = pac3_mse_profile()
    resolved = resolve_interceptor(profile)

    assert resolved.resolution_status is ResolutionStatus.ARCHETYPE_DOMINANT
    assert resolved.parameters["propulsion_architecture"].origin is ValueOrigin.OBSERVED
    assert "cl:us-mse-propulsion" in resolved.parameters["propulsion_architecture"].source_record_ids
    assert resolved.parameters["launch_mass_kg"].origin is ValueOrigin.ARCHETYPE_ASSUMPTION
    assert resolved.parameters["length_m"].origin is ValueOrigin.ARCHETYPE_ASSUMPTION
    assert resolved.parameters["reference_area_m2"].origin is ValueOrigin.ARCHETYPE_ASSUMPTION
    assert resolved.parameters["burnout_mass_kg"].origin is ValueOrigin.ARCHETYPE_ASSUMPTION
    assert {item.parameter_id for item in resolved.evidence_gaps} >= {
        "launch_mass_kg",
        "length_m",
        "body_diameter_m",
        "reported_max_range_m",
        "reported_max_altitude_m",
    }
    assert set(resolved.required_diagnostics) == {
        "geometry_missing",
        "mass_properties_missing",
        "motor_pulse_timing_missing",
        "aerodynamic_model_missing",
        "control_authority_missing",
    }
    assert "reported_max_range_m" not in resolved.parameters
    assert "reported_max_altitude_m" not in resolved.parameters
    assert {item.model_id for item in runnable_prototype_profiles()} == {
        "aim9x-block2",
        "aim120-amraam-c5-c7",
    }
    ####


def test_yaml_catalogue_seeds_match_the_public_profile_builders() -> None:
    pairs = (
        ("aim9x_block2_catalogue_seed.yaml", aim9x_block2_profile()),
        ("aim120_c5_c7_catalogue_seed.yaml", aim120_c5_c7_profile()),
        ("pac3_mse_catalogue_seed.yaml", pac3_mse_profile()),
    )

    for filename, expected in pairs:
        loaded = load_catalogue_interceptor_record(_EXAMPLES / filename)
        assert resolve_interceptor(loaded).fingerprint == resolve_interceptor(expected).fingerprint

    provider = ParametricInterceptorMissionCompositionProvider.from_catalogue_yaml(
        _EXAMPLES / "aim9x_block2_catalogue_seed.yaml",
        _EXAMPLES / "aim120_c5_c7_catalogue_seed.yaml",
    )
    assert {item.id for item in provider.list_models()} == {"aim9x-block2", "aim120-amraam-c5-c7"}
    ####


def test_two_source_populated_witnesses_advertise_and_run_vertically() -> None:
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(*runnable_prototype_profiles())
    audit = audit_provider_advertisement(provider, provider.build_runner())
    assert audit.status == "pass", audit.diagnostics

    for model_id in ("aim9x-block2", "aim120-amraam-c5-c7"):
        model = provider.model(model_id)
        properties = {item.id: item for item in model.presentation.properties}
        assert properties["resolution_status"].value == "source_dominant"
        assert "classified" in str(properties["evidence_gaps"].value)
        if model_id == "aim120-amraam-c5-c7":
            assert "source_value=356 lb" in properties["launch_mass_kg"].provenance
            assert "launch_mass_kg=[157.85, 162.39] kg" in str(properties["evidence_intervals"].value)
        configuration = provider.configuration(
            model_id,
            configuration_id=f"{model_id}-prototype-smoke",
            runtime_duration_s=0.2,
            runtime_time_step_s=0.1,
        )
        prepared = provider.validate_configuration(configuration)
        response = provider.build_runner().run(
            MissionCompositionRunRequest(
                request_id=f"{model_id}-prototype-request",
                provider_id=provider.metadata.id,
                provider_version=provider.metadata.version,
                prepared_configuration=prepared,
                output=MissionCompositionOutputSelection(mode="core"),
            )
        )
        assert isinstance(response, MissionCompositionTrajectoryResponse)
        assert response.result.objects[0].samples[-1].time_s == 0.2
    ####
