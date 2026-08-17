"""Focused evidence and archetype tests for parametric interceptor authoring."""

from __future__ import annotations

import pytest
from taoryx_parametric_interceptors import (
    AssumptionCase,
    ParametricInterceptorMissionCompositionProvider,
    ValueOrigin,
    derived,
    inferred,
    interceptor,
    load_interceptor_profile,
    observed,
    reported,
    resolve_interceptor,
    unavailable,
    variant_interval,
)


def test_compact_profile_never_launders_raw_values_into_observations() -> None:
    profile = interceptor(
        "catalog-medium-sam",
        launch_mass_kg=420.0,
        length_m=5.2,
        body_diameter_m=0.36,
        guidance_family="sarh",
    )

    assert profile.launch_mass_kg is not None
    assert profile.launch_mass_kg.origin is ValueOrigin.SIMULATION_ASSUMPTION

    resolved = resolve_interceptor(profile)
    assert resolved.parameters["launch_mass_kg"].origin is ValueOrigin.SIMULATION_ASSUMPTION
    assert resolved.parameters["reference_area_m2"].origin is ValueOrigin.SIMULATION_ASSUMPTION
    assert resolved.parameters["burn_time_s"].origin is ValueOrigin.ARCHETYPE_ASSUMPTION
    assert resolved.parameters["burn_time_s"].archetype_id == "generic_slender_sam_v1"
    assert resolved.parameters["reference_area_m2"].depends_on == ("body_diameter_m",)
    assert resolved.text("guidance_family") == "sarh"
    assert resolved.text("guidance_archetype") == "waypoint_pursuit"
    assert resolved.parameters["guidance_archetype"].origin is ValueOrigin.ARCHETYPE_ASSUMPTION
    assert resolved.number("navigation_constant") == 3.0
    assert len(resolved.fingerprint) == 64
    ####


def test_observed_and_reported_values_retain_source_record_ids() -> None:
    profile = interceptor(
        "sourced-sam",
        launch_mass_kg=observed(510.0, unit="kg", source_record_id="ontology:sam-7:launch-mass"),
        length_m=observed(5.8, unit="m", source_record_id="ontology:sam-7:length"),
        reported_max_range_m=reported(60_000.0, unit="m", source_record_id="source:range-claim"),
    )

    resolved = resolve_interceptor(profile)

    assert resolved.parameters["launch_mass_kg"].origin is ValueOrigin.OBSERVED
    assert resolved.parameters["launch_mass_kg"].source_record_ids == ("ontology:sam-7:launch-mass",)
    assert resolved.parameters["reported_max_range_m"].origin is ValueOrigin.REPORTED
    assert resolved.parameters["reported_max_range_m"].source_record_ids == ("source:range-claim",)
    ####


def test_python_helpers_cover_multi_source_derived_inferred_gap_and_interval_evidence() -> None:
    profile = interceptor(
        "ergonomic-evidence-sam",
        launch_mass_kg=observed(
            161.48,
            unit="kg",
            source_record_ids=("source:navair:mass", "ontology:amraam:mass-claim"),
        ),
        reference_area_m2=derived(
            0.02483,
            unit="m^2",
            method="circular_area_from_body_diameter",
            source_record_ids=("source:navair:diameter", "ontology:amraam:diameter-claim"),
        ),
        aero_archetype=inferred(
            "generic_slender_supersonic",
            method="selected_from_public_geometry_and_speed_regime",
            source_record_id="resolver:archetype-selection-v1",
        ),
        evidence_gaps=(
            unavailable(
                "reported_max_speed_mps",
                "classified",
                unit="m/s",
                source_record_id="source:navair:performance-table",
            ),
        ),
        evidence_intervals=(
            variant_interval(
                "launch_mass_kg",
                157.85,
                162.39,
                unit="kg",
                source_record_ids=("source:navair:variant-masses", "ontology:amraam:family"),
                source_minimum_value=348.0,
                source_maximum_value=358.0,
                source_unit="lb",
                note="Family range retained separately from the selected variant value.",
            ),
        ),
    )
    resolved = resolve_interceptor(profile)

    assert profile.launch_mass_kg is not None
    assert profile.launch_mass_kg.source_record_ids == (
        "source:navair:mass",
        "ontology:amraam:mass-claim",
    )
    assert resolved.parameters["reference_area_m2"].origin is ValueOrigin.DERIVED
    assert resolved.parameters["reference_area_m2"].method == "circular_area_from_body_diameter"
    assert resolved.parameters["aero_archetype"].origin is ValueOrigin.INFERRED
    assert resolved.parameters["aero_archetype"].method == "selected_from_public_geometry_and_speed_regime"
    assert resolved.evidence_gaps[0].source_record_ids == ("source:navair:performance-table",)
    assert resolved.evidence_intervals[0].source_record_ids == (
        "source:navair:variant-masses",
        "ontology:amraam:family",
    )
    assert resolved.evidence_intervals[0].source_unit == "lb"
    ####


def test_evidence_helpers_reject_ambiguous_or_malformed_source_arguments() -> None:
    with pytest.raises(ValueError, match="source_record_id or source_record_ids"):
        observed(
            100.0,
            unit="kg",
            source_record_id="source:one",
            source_record_ids=("source:two",),
        )
    with pytest.raises(TypeError, match="sequence of complete identifiers"):
        reported(100.0, unit="m/s", source_record_ids="source:not-a-sequence")
    with pytest.raises(ValueError, match="must be unique"):
        derived(
            0.02,
            unit="m^2",
            method="geometry",
            source_record_ids=("source:diameter", "source:diameter"),
        )
    with pytest.raises(ValueError, match="without surrounding whitespace"):
        unavailable("reported_max_range_m", "classified", source_record_id=" source:range ")
    with pytest.raises(ValueError, match="source bounds and unit must be supplied together"):
        variant_interval(
            "launch_mass_kg",
            100.0,
            120.0,
            unit="kg",
            source_minimum_value=220.0,
        )
    ####


def test_assumption_cases_are_reproducible_and_directional() -> None:
    profile = interceptor("case-sam", launch_mass_kg=400.0, body_diameter_m=0.4)
    conservative = resolve_interceptor(profile, assumption_case=AssumptionCase.CONSERVATIVE)
    nominal = resolve_interceptor(profile)
    optimistic = resolve_interceptor(profile, assumption_case=AssumptionCase.OPTIMISTIC)

    assert conservative.number("thrust_n") < nominal.number("thrust_n") < optimistic.number("thrust_n")
    assert conservative.number("drag_coefficient") > nominal.number("drag_coefficient") > optimistic.number("drag_coefficient")
    assert len({conservative.fingerprint, nominal.fingerprint, optimistic.fingerprint}) == 3
    ####


def test_pseudo6_response_defaults_are_visible_and_individually_overridable() -> None:
    default = resolve_interceptor(interceptor("default-response", control_bandwidth_class="fast"))
    tuned = resolve_interceptor(
        interceptor(
            "tuned-response",
            attitude_bandwidth_rad_s=8.5,
            attitude_damping_ratio=0.82,
            max_body_rate_rad_s=3.2,
            max_body_acceleration_rad_s2=14.0,
            max_bank_angle_rad=1.1,
        )
    )

    assert default.parameters["attitude_bandwidth_rad_s"].origin is ValueOrigin.ARCHETYPE_ASSUMPTION
    assert default.number("attitude_bandwidth_rad_s") == 10.0
    assert tuned.number("attitude_bandwidth_rad_s") == 8.5
    assert tuned.number("attitude_damping_ratio") == 0.82
    assert tuned.number("max_body_rate_rad_s") == 3.2
    assert tuned.parameters["max_bank_angle_rad"].origin is ValueOrigin.SIMULATION_ASSUMPTION
    ####


def test_invalid_class_and_noncanonical_unit_fail_at_the_resolution_boundary() -> None:
    with pytest.raises(ValueError, match="unsupported drag_class"):
        resolve_interceptor(interceptor("bad-class", drag_class="magic"))

    with pytest.raises(ValueError, match="unsupported guidance_archetype"):
        resolve_interceptor(interceptor("bad-guidance", guidance_archetype="magic"))

    with pytest.raises(ValueError, match="canonical unit 'kg'"):
        resolve_interceptor(
            interceptor(
                "bad-unit",
                launch_mass_kg=observed(900.0, unit="lb", source_record_id="source:mass"),
            )
        )
    ####


def test_yaml_loader_accepts_raw_values_and_inline_evidence(tmp_path) -> None:
    path = tmp_path / "developer-sam.yaml"
    path.write_text(
        """\
interceptor_id: developer-sam
launch_mass_kg:
  value: 420.0
  unit: kg
  origin: observed
  source_record_ids: [ontology:developer-sam:mass]
  confidence: high
length_m: 5.2
body_diameter_m: 0.36
target_classes: [air_breathing, cruise_missile]
""",
        encoding="utf-8",
    )

    profile = load_interceptor_profile(path)

    assert profile.launch_mass_kg is not None
    assert profile.launch_mass_kg.origin is ValueOrigin.OBSERVED
    assert profile.length_m is not None
    assert profile.length_m.origin is ValueOrigin.SIMULATION_ASSUMPTION
    assert profile.target_classes is not None
    assert profile.target_classes.value == ("air_breathing", "cruise_missile")

    provider = ParametricInterceptorMissionCompositionProvider.from_yaml(
        path,
        assumption_case=AssumptionCase.CONSERVATIVE,
    )
    assert provider.list_models()[0].id == "developer-sam"
    assert provider.resolved_profile("developer-sam").assumption_case is AssumptionCase.CONSERVATIVE
    ####
