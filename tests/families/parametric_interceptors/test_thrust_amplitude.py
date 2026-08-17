"""Focused absolute thrust authoring, provenance, and fallback witnesses."""

from __future__ import annotations

import pytest
from pydantic import ValidationError
from taoryx_parametric_interceptors import (
    AssumptionCase,
    ParametricInterceptorMissionCompositionProvider,
    ThrustProfileSchedule,
    ValueOrigin,
    calibrated,
    interceptor,
    interceptor_from_catalogue_record,
    interceptor_parameter_usage,
    observed,
    resolve_interceptor,
    supported_interceptor_units,
)


def _explicit_schedule() -> ThrustProfileSchedule:
    return ThrustProfileSchedule(
        schedule_id="explicit-amplitude-shape-v1",
        points=(
            {"burn_fraction": 0.0, "multiplier": 1.4},
            {"burn_fraction": 0.3, "multiplier": 1.0},
            {"burn_fraction": 1.0, "multiplier": 0.8},
        ),
        method="focused explicit amplitude witness shape",
    )
    ####


def test_coarse_path_preserves_legacy_mean_thrust_and_case_order() -> None:
    profile = interceptor(
        "coarse-thrust-amplitude",
        launch_mass_kg=100.0,
        thrust_profile_class="neutral",
    )
    resolved = {case: resolve_interceptor(profile, assumption_case=case) for case in AssumptionCase}
    nominal = resolved[AssumptionCase.NOMINAL]

    expected_mean = 100.0 * 9.80665 * 6.0
    assert nominal.number("nominal_thrust_n") == pytest.approx(expected_mean)
    assert nominal.number("thrust_n") == pytest.approx(expected_mean)
    assert all(item.number("nominal_thrust_n") == pytest.approx(expected_mean) for item in resolved.values())
    assert resolved[AssumptionCase.CONSERVATIVE].number("thrust_n") < nominal.number("thrust_n") < resolved[AssumptionCase.OPTIMISTIC].number("thrust_n")
    assert nominal.parameters["nominal_thrust_n"].origin is ValueOrigin.SIMULATION_ASSUMPTION
    assert nominal.parameters["nominal_thrust_n"].depends_on == (
        "launch_mass_kg",
        "thrust_profile_class",
    )
    ####


def test_explicit_mean_and_shape_remove_the_coarse_class_without_losing_evidence() -> None:
    profile = interceptor(
        "fully-explicit-propulsion",
        nominal_thrust_n=observed(
            25_000.0,
            unit="N",
            source_record_id="source:static-test-mean-thrust",
        ),
        burn_time_s=5.0,
        thrust_profile_schedule=_explicit_schedule(),
    )
    nominal = resolve_interceptor(profile)
    conservative = resolve_interceptor(profile, assumption_case="conservative")
    usage = interceptor_parameter_usage(nominal)

    assert "thrust_profile_class" not in nominal.parameters
    assert nominal.thrust_profile_schedule_depends_on == ()
    assert nominal.number("nominal_thrust_n") == 25_000.0
    assert nominal.parameters["nominal_thrust_n"].origin is ValueOrigin.OBSERVED
    assert nominal.parameters["nominal_thrust_n"].source_record_ids == ("source:static-test-mean-thrust",)
    assert nominal.number("thrust_n") == 25_000.0
    assert nominal.parameters["thrust_n"].origin is ValueOrigin.DERIVED
    assert nominal.parameters["thrust_n"].source_record_ids == ("source:static-test-mean-thrust",)
    assert nominal.number("active_burn_total_impulse_n_s") == 125_000.0
    assert usage["nominal_thrust_n"].usage_class == "resolution_input"
    assert usage["nominal_thrust_n"].resolved_sink_ids == ("thrust_n",)
    assert usage["active_burn_total_impulse_n_s"].usage_class == "derived_summary"

    assert conservative.number("nominal_thrust_n") == 25_000.0
    assert conservative.parameters["nominal_thrust_n"].origin is ValueOrigin.OBSERVED
    assert conservative.number("thrust_n") == pytest.approx(22_500.0)
    assert conservative.parameters["thrust_n"].origin is ValueOrigin.SIMULATION_ASSUMPTION
    ####


def test_partial_overrides_keep_the_coarse_class_only_for_the_missing_side() -> None:
    explicit_amplitude = resolve_interceptor(
        interceptor(
            "explicit-amplitude-only",
            nominal_thrust_n=20_000.0,
        )
    )
    amplitude_usage = interceptor_parameter_usage(explicit_amplitude)
    assert amplitude_usage["thrust_profile_class"].resolved_sink_ids == ("thrust_profile_schedule",)
    assert explicit_amplitude.thrust_profile_schedule_depends_on == ("thrust_profile_class",)

    explicit_shape = resolve_interceptor(
        interceptor(
            "explicit-shape-only",
            thrust_profile_schedule=_explicit_schedule(),
        )
    )
    shape_usage = interceptor_parameter_usage(explicit_shape)
    assert explicit_shape.thrust_profile_schedule_depends_on == ()
    assert shape_usage["thrust_profile_class"].resolved_sink_ids == ("thrust_n",)
    assert explicit_shape.parameters["nominal_thrust_n"].depends_on == (
        "launch_mass_kg",
        "thrust_profile_class",
    )
    ####


def test_redundant_explicit_coarse_selector_is_rejected() -> None:
    with pytest.raises(ValidationError, match="fully replace an explicitly supplied"):
        interceptor(
            "ambiguous-explicit-propulsion",
            nominal_thrust_n=20_000.0,
            thrust_profile_schedule=_explicit_schedule(),
            thrust_profile_class="neutral",
        )
    ####


def test_calibration_scale_changes_executable_thrust_not_source_amplitude() -> None:
    resolved = resolve_interceptor(
        interceptor(
            "scaled-source-thrust",
            nominal_thrust_n=observed(
                30_000.0,
                unit="N",
                source_record_id="source:mean-thrust",
            ),
            thrust_scale=calibrated(
                1.1,
                unit="1",
                method="synthetic propulsion calibration",
            ),
        )
    )

    assert resolved.number("nominal_thrust_n") == 30_000.0
    assert resolved.parameters["nominal_thrust_n"].origin is ValueOrigin.OBSERVED
    assert resolved.number("thrust_n") == pytest.approx(33_000.0)
    assert resolved.parameters["thrust_n"].origin is ValueOrigin.CALIBRATED
    ####


def test_catalogue_mean_thrust_alias_converts_kn_and_retains_source_scalar() -> None:
    profile = interceptor_from_catalogue_record(
        {
            "kind": "interceptor_evidence_record",
            "interceptor_id": "int:catalogue-thrust-witness",
            "evidence": {
                "mean_thrust": {
                    "value": 25.0,
                    "unit": "kN",
                    "origin": "observed",
                    "source_record_ids": ["source:motor-data-sheet"],
                }
            },
        }
    )
    resolved = resolve_interceptor(profile)
    thrust = resolved.parameters["nominal_thrust_n"]

    assert thrust.value == 25_000.0
    assert thrust.unit == "N"
    assert thrust.source_value is not None
    assert thrust.source_value.value == 25.0
    assert thrust.source_value.unit == "kN"
    assert {"N", "kN", "lbf"} <= set(supported_interceptor_units("nominal_thrust_n"))
    ####


def test_composition_advertises_absolute_amplitude_impulse_and_schedule_dependency() -> None:
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(
        interceptor(
            "advertised-explicit-propulsion",
            nominal_thrust_n=25_000.0,
            burn_time_s=5.0,
            thrust_profile_schedule=_explicit_schedule(),
        )
    )
    properties = {item.id: item for item in provider.model("advertised-explicit-propulsion").presentation.properties}

    assert properties["nominal_thrust_n"].value == 25_000.0
    assert properties["nominal_thrust_n"].canonical_unit == "N"
    assert properties["thrust_n"].value == 25_000.0
    assert properties["active_burn_total_impulse_n_s"].value == 125_000.0
    assert properties["active_burn_total_impulse_n_s"].quantity == "impulse"
    assert properties["active_burn_total_impulse_n_s"].canonical_unit == "N*s"
    assert properties["thrust_profile_schedule.depends_on"].value == "none"
    assert "thrust_profile_class" not in properties
    ####
