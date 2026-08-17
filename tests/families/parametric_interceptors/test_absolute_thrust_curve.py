"""Focused source-shaped absolute thrust-curve compilation witnesses."""

from __future__ import annotations

import pytest
from pydantic import ValidationError
from taoryx_parametric_interceptors import (
    AbsoluteThrustCurve,
    ParametricInterceptorMissionCompositionProvider,
    PropulsionProgram,
    ValueOrigin,
    build_interceptor_authoring_report,
    flat_interceptor_profile_schema,
    interceptor,
    interceptor_from_catalogue_record,
    interceptor_parameter_usage,
    load_interceptor_profile,
    resolve_interceptor,
)


def _triangle_curve(**updates: object) -> AbsoluteThrustCurve:
    values: dict[str, object] = {
        "curve_id": "triangle-motor-curve-v1",
        "points": (
            {"time": 0.0, "thrust": 0.0},
            {"time": 1.0, "thrust": 100.0},
            {"time": 2.0, "thrust": 0.0},
        ),
        "method": "focused triangular motor curve",
    }
    values.update(updates)
    return AbsoluteThrustCurve.model_validate(values)
    ####


def test_linear_curve_integrates_and_normalizes_without_manual_fraction_math() -> None:
    curve = _triangle_curve()
    schedule = curve.normalized_schedule()

    assert curve.duration_s == 2.0
    assert curve.total_impulse_n_s == 100.0
    assert curve.mean_thrust_n == 50.0
    assert curve.canonical_times_s == (0.0, 1.0, 2.0)
    assert curve.canonical_thrusts_n == (0.0, 100.0, 0.0)
    assert [item.burn_fraction for item in schedule.points] == [0.0, 0.5, 1.0]
    assert [item.multiplier for item in schedule.points] == [0.0, 2.0, 0.0]
    assert schedule.raw_area == 1.0
    assert schedule.cumulative_fraction_at(0.5) == 0.5
    assert len(curve.fingerprint) == 64
    ####


def test_curve_converts_source_time_and_force_units_before_integration() -> None:
    curve = _triangle_curve(
        time_unit="ms",
        thrust_unit="kN",
        points=(
            {"time": 0.0, "thrust": 0.0},
            {"time": 1_000.0, "thrust": 10.0},
            {"time": 2_000.0, "thrust": 0.0},
        ),
    )

    assert curve.duration_s == 2.0
    assert curve.total_impulse_n_s == 10_000.0
    assert curve.mean_thrust_n == 5_000.0
    assert curve.canonical_thrusts_n == (0.0, 10_000.0, 0.0)
    ####


def test_step_previous_curve_uses_held_source_ordinates() -> None:
    curve = AbsoluteThrustCurve(
        curve_id="step-motor-curve-v1",
        interpolation="step_previous",
        points=(
            {"time": 0.0, "thrust": 10.0},
            {"time": 1.0, "thrust": 5.0},
            {"time": 3.0, "thrust": 0.0},
        ),
    )

    assert curve.total_impulse_n_s == 20.0
    assert curve.mean_thrust_n == pytest.approx(20.0 / 3.0)
    assert curve.normalized_schedule().interpolation == "step_previous"
    ####


def test_curve_rejects_invalid_domains_units_and_untraceable_source_claims() -> None:
    with pytest.raises(ValidationError, match="start at time zero"):
        _triangle_curve(
            points=(
                {"time": 0.1, "thrust": 10.0},
                {"time": 1.0, "thrust": 0.0},
            )
        )
    with pytest.raises(ValidationError, match="strictly increasing"):
        _triangle_curve(
            points=(
                {"time": 0.0, "thrust": 10.0},
                {"time": 0.0, "thrust": 5.0},
            )
        )
    with pytest.raises(ValidationError, match="integrated impulse must be positive"):
        _triangle_curve(
            points=(
                {"time": 0.0, "thrust": 0.0},
                {"time": 1.0, "thrust": 0.0},
            )
        )
    with pytest.raises(ValidationError, match="observed absolute thrust curve requires"):
        _triangle_curve(origin="observed")
    with pytest.raises(ValidationError, match="requires a unit compatible with 'N'"):
        _triangle_curve(thrust_unit="kg")
    ####


def test_curve_compiles_into_existing_program_and_removes_coarse_baggage() -> None:
    resolved = resolve_interceptor(
        interceptor(
            "curve-compiled-motor",
            launch_mass_kg=100.0,
            burnout_mass_kg=80.0,
            thrust_time_curve=_triangle_curve(),
        )
    )
    program = PropulsionProgram.from_profile(resolved)
    usage = interceptor_parameter_usage(resolved)

    assert resolved.number("burn_time_s") == 2.0
    assert resolved.number("first_pulse_burn_time_s") == 2.0
    assert resolved.number("nominal_thrust_n") == 50.0
    assert resolved.number("thrust_n") == 50.0
    assert resolved.number("active_burn_total_impulse_n_s") == 100.0
    assert "burn_time_class" not in resolved.parameters
    assert "thrust_profile_class" not in resolved.parameters
    assert resolved.thrust_time_curve is not None
    assert resolved.thrust_profile_schedule.schedule_id == "triangle-motor-curve-v1-normalized-v1"
    assert program.nominal_total_impulse_ns == 100.0
    assert program.sample(1.0).mass_kg == 90.0
    assert usage["thrust_time_curve"].usage_class == "resolution_input"
    assert set(usage["thrust_time_curve"].resolved_sink_ids) == {
        "first_pulse_burn_time_s",
        "thrust_n",
        "thrust_profile_schedule",
    }
    ####


def test_observed_curve_derives_values_without_laundering_the_source_claim() -> None:
    curve = _triangle_curve(
        origin="observed",
        confidence="high",
        source_record_ids=("source:static-motor-test",),
    )
    resolved = resolve_interceptor(interceptor("observed-motor-curve", thrust_time_curve=curve))

    assert resolved.thrust_time_curve is curve
    assert resolved.parameters["burn_time_s"].origin is ValueOrigin.DERIVED
    assert resolved.parameters["nominal_thrust_n"].origin is ValueOrigin.DERIVED
    assert resolved.parameters["nominal_thrust_n"].source_record_ids == ("source:static-motor-test",)
    assert resolved.thrust_profile_schedule.origin == "derived"
    assert resolved.thrust_profile_schedule.source_record_ids == ("source:static-motor-test",)
    ####


def test_curve_rejects_overlapping_fields_and_dual_pulse_interpretation() -> None:
    with pytest.raises(ValidationError, match="derives burn duration"):
        interceptor(
            "ambiguous-curve-input",
            thrust_time_curve=_triangle_curve(),
            burn_time_s=2.0,
        )
    with pytest.raises(ValidationError, match="first_pulse_burn_time_s"):
        interceptor(
            "ambiguous-curve-pulse-input",
            thrust_time_curve=_triangle_curve(),
            first_pulse_burn_time_s=2.0,
        )
    with pytest.raises(ValueError, match="one continuous active-burn interval"):
        resolve_interceptor(
            interceptor(
                "ambiguous-dual-pulse-curve",
                propulsion_architecture="dual_pulse_solid",
                thrust_time_curve=_triangle_curve(),
            )
        )
    ####


def test_catalogue_and_composition_preserve_the_original_curve() -> None:
    profile = interceptor_from_catalogue_record(
        {
            "kind": "interceptor_evidence_record",
            "interceptor_id": "int:absolute-curve-witness",
            "thrust_time_curve": {
                "curve_id": "catalogue-curve-v1",
                "time_unit": "ms",
                "thrust_unit": "kN",
                "points": [
                    {"time": 0.0, "thrust": 0.0},
                    {"time": 500.0, "thrust": 20.0},
                    {"time": 1_000.0, "thrust": 0.0},
                ],
                "origin": "reported",
                "source_record_ids": ["source:public-thrust-plot"],
                "confidence": "medium",
                "method": "digitized public curve",
            },
        }
    )
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(profile)
    resolved = provider.resolved_profile(profile.model_id)
    properties = {item.id: item for item in provider.model(profile.model_id).presentation.properties}

    assert resolved.number("burn_time_s") == 1.0
    assert resolved.number("nominal_thrust_n") == 10_000.0
    assert properties["thrust_time_curve.contract"].value == ("taoryx.parametric-interceptors.absolute-thrust-curve/v1")
    assert properties["thrust_time_curve.time_unit"].value == "ms"
    assert properties["thrust_time_curve.thrust_unit"].value == "kN"
    assert properties["thrust_time_curve.total_impulse"].value == 10_000.0
    assert properties["thrust_time_curve.total_impulse"].canonical_unit == "N*s"
    assert properties["thrust_time_curve.points"].source_refs == ("source:public-thrust-plot",)
    assert properties["evidence_value_count"].value == 1
    ####


def test_flat_schema_and_copy_ready_example_expose_the_curve_contract() -> None:
    schema = flat_interceptor_profile_schema()
    curve_schema = schema["properties"]["thrust_time_curve"]
    assert "$ref" in curve_schema["anyOf"][0]
    assert "AbsoluteThrustCurve" in curve_schema["anyOf"][0]["$ref"]

    profile = load_interceptor_profile("examples/parametric_interceptors/absolute_thrust_curve_sam.yaml")
    resolved = resolve_interceptor(profile)
    assert profile.thrust_time_curve is not None
    assert resolved.number("burn_time_s") == 4.0
    assert resolved.number("nominal_thrust_n") == pytest.approx(29_062.5)
    assert resolved.number("active_burn_total_impulse_n_s") == pytest.approx(116_250.0)

    report = build_interceptor_authoring_report("examples/parametric_interceptors/absolute_thrust_curve_sam.yaml")
    assert report.parameter_usage["thrust_time_curve"].usage_class == "resolution_input"
    assert "thrust_time_curve" in report.readiness.assumption_parameter_ids
    assert sum(report.usage_counts.values()) == len(report.resolved_profile.parameters) + 3
    ####
