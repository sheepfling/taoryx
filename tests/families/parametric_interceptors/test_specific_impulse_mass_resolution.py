"""Focused effective-Isp lowering and mass-history witnesses."""

from __future__ import annotations

import pytest
from pydantic import ValidationError
from taoryx_parametric_interceptors import (
    AbsoluteThrustCurve,
    DualPulseThrustProgram,
    ParametricInterceptorMissionCompositionProvider,
    PropulsionProgram,
    ValueOrigin,
    interceptor,
    interceptor_from_catalogue_record,
    interceptor_parameter_usage,
    observed,
    resolve_interceptor,
)

STANDARD_GRAVITY_MPS2 = 9.80665


def _triangle_curve(
    curve_id: str,
    *,
    duration_s: float,
    peak_thrust_n: float,
    source_record_id: str = "source:motor-static-test",
) -> AbsoluteThrustCurve:
    return AbsoluteThrustCurve.model_validate(
        {
            "curve_id": curve_id,
            "points": (
                {"time": 0.0, "thrust": 0.0},
                {"time": duration_s / 2.0, "thrust": peak_thrust_n},
                {"time": duration_s, "thrust": 0.0},
            ),
            "origin": "observed",
            "source_record_ids": (source_record_id,),
            "confidence": "high",
            "method": "focused triangular static-test witness",
        }
    )
    ####


def test_effective_isp_derives_single_pulse_mass_and_retains_lineage() -> None:
    curve = _triangle_curve(
        "single-pulse-isp-v1",
        duration_s=2.0,
        peak_thrust_n=STANDARD_GRAVITY_MPS2 * 1_000.0,
    )
    profile = interceptor(
        "single-pulse-isp",
        launch_mass_kg=observed(
            100.0,
            unit="kg",
            source_record_id="source:wet-mass",
        ),
        effective_specific_impulse_s=observed(
            100.0,
            unit="s",
            source_record_id="source:effective-isp",
        ),
        thrust_time_curve=curve,
    )
    resolved = resolve_interceptor(profile)
    runtime = PropulsionProgram.from_profile(resolved)
    usage = interceptor_parameter_usage(resolved)

    assert curve.total_impulse_n_s == pytest.approx(STANDARD_GRAVITY_MPS2 * 1_000.0)
    assert resolved.number("propellant_fraction") == pytest.approx(0.1)
    assert resolved.number("burnout_mass_kg") == pytest.approx(90.0)
    assert resolved.parameters["burnout_mass_kg"].origin is ValueOrigin.DERIVED
    assert resolved.parameters["burnout_mass_kg"].source_record_ids == (
        "source:wet-mass",
        "source:effective-isp",
        "source:motor-static-test",
    )
    assert usage["effective_specific_impulse_s"].usage_class == "resolution_input"
    assert "burnout_mass_kg" in usage["effective_specific_impulse_s"].resolved_sink_ids
    assert usage["propellant_fraction"].usage_class == "inactive_resolution_input"
    assert runtime.sample(1.0).mass_kg == pytest.approx(95.0)
    assert runtime.sample(2.0).mass_kg == pytest.approx(90.0)
    ####


def test_effective_isp_mass_is_invariant_to_executable_thrust_case() -> None:
    profile = interceptor(
        "isp-case-invariance",
        launch_mass_kg=100.0,
        effective_specific_impulse_s=100.0,
        thrust_time_curve=_triangle_curve(
            "case-invariant-curve-v1",
            duration_s=2.0,
            peak_thrust_n=STANDARD_GRAVITY_MPS2 * 1_000.0,
        ),
    )

    conservative = resolve_interceptor(profile, assumption_case="conservative")
    optimistic = resolve_interceptor(profile, assumption_case="optimistic")

    assert conservative.number("burnout_mass_kg") == pytest.approx(90.0)
    assert optimistic.number("burnout_mass_kg") == pytest.approx(90.0)
    assert conservative.number("thrust_n") < optimistic.number("thrust_n")
    ####


def test_common_effective_isp_derives_dual_pulse_mass_share_from_impulse() -> None:
    program = DualPulseThrustProgram(
        program_id="dual-isp-program-v1",
        first_pulse=_triangle_curve(
            "dual-isp-first-v1",
            duration_s=1.0,
            peak_thrust_n=STANDARD_GRAVITY_MPS2 * 1_000.0,
            source_record_id="source:first-pulse",
        ),
        inter_pulse_coast_time=2.0,
        second_pulse=_triangle_curve(
            "dual-isp-second-v1",
            duration_s=2.0,
            peak_thrust_n=STANDARD_GRAVITY_MPS2 * 500.0,
            source_record_id="source:second-pulse",
        ),
        method="focused two-pulse Isp witness",
    )
    resolved = resolve_interceptor(
        interceptor(
            "dual-pulse-isp",
            launch_mass_kg=100.0,
            effective_specific_impulse_s=100.0,
            dual_pulse_thrust_program=program,
        )
    )
    runtime = PropulsionProgram.from_profile(resolved)

    assert program.first_pulse.total_impulse_n_s == pytest.approx(STANDARD_GRAVITY_MPS2 * 500.0)
    assert program.second_pulse.total_impulse_n_s == pytest.approx(STANDARD_GRAVITY_MPS2 * 500.0)
    assert resolved.number("propellant_fraction") == pytest.approx(0.1)
    assert resolved.number("burnout_mass_kg") == pytest.approx(90.0)
    assert resolved.number("second_pulse_propellant_fraction") == pytest.approx(0.5)
    assert runtime.sample(1.0).mass_kg == pytest.approx(95.0)
    assert runtime.sample(3.0).mass_kg == pytest.approx(95.0)
    assert runtime.sample(4.0).mass_kg == pytest.approx(92.5)
    assert runtime.sample(5.0).mass_kg == pytest.approx(90.0)
    ####


def test_effective_isp_rejects_overdetermined_underspecified_and_impossible_mass() -> None:
    with pytest.raises(ValidationError, match="derives burnout mass and propellant allocation"):
        interceptor(
            "isp-overdetermined",
            launch_mass_kg=100.0,
            burnout_mass_kg=80.0,
            effective_specific_impulse_s=100.0,
            thrust_time_curve=_triangle_curve("overdetermined-v1", duration_s=2.0, peak_thrust_n=1_000.0),
        )
    with pytest.raises(ValidationError, match="requires nominal_thrust_n"):
        interceptor(
            "isp-no-amplitude",
            launch_mass_kg=100.0,
            burn_time_s=2.0,
            effective_specific_impulse_s=100.0,
        )
    with pytest.raises(ValidationError, match="requires burn_time_s"):
        interceptor(
            "isp-no-timing",
            launch_mass_kg=100.0,
            nominal_thrust_n=1_000.0,
            effective_specific_impulse_s=100.0,
        )
    with pytest.raises(ValueError, match="propellant mass strictly between zero and launch mass"):
        resolve_interceptor(
            interceptor(
                "isp-impossible-mass",
                launch_mass_kg=1.0,
                effective_specific_impulse_s=1.0,
                thrust_time_curve=_triangle_curve(
                    "impossible-mass-v1",
                    duration_s=2.0,
                    peak_thrust_n=100.0,
                ),
            )
        )
    ####


def test_catalogue_alias_and_composition_advertise_effective_isp_derivation() -> None:
    profile = interceptor_from_catalogue_record(
        {
            "kind": "interceptor_evidence_record",
            "interceptor_id": "int:catalogue-isp",
            "evidence": {
                "launch_mass": {
                    "value": 100.0,
                    "unit": "kg",
                    "origin": "observed",
                    "source_record_ids": ["source:catalogue-wet-mass"],
                },
                "specific_impulse": {
                    "value": 100.0,
                    "unit": "s",
                    "origin": "reported",
                    "source_record_ids": ["source:catalogue-isp"],
                },
            },
            "thrust_time_curve": _triangle_curve(
                "catalogue-isp-curve-v1",
                duration_s=2.0,
                peak_thrust_n=STANDARD_GRAVITY_MPS2 * 1_000.0,
            ).model_dump(mode="json"),
        }
    )
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(profile)
    resolved = provider.resolved_profile(profile.model_id)
    properties = {item.id: item for item in provider.model(profile.model_id).presentation.properties}

    assert resolved.number("effective_specific_impulse_s") == 100.0
    assert properties["effective_specific_impulse_s"].quantity == "time"
    assert properties["effective_specific_impulse_s"].canonical_unit == "s"
    assert "usage_class=resolution_input" in properties["effective_specific_impulse_s"].provenance
    assert "source:catalogue-isp" in properties["burnout_mass_kg"].source_refs
    assert "effective_specific_impulse_s" in properties["burnout_mass_kg"].provenance
    ####
