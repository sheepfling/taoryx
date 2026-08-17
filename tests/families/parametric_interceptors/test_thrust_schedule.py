"""Focused normalized thrust-schedule authoring and runtime witnesses."""

from __future__ import annotations

import pytest
from pydantic import ValidationError
from taoryx_parametric_interceptors import (
    ParametricInterceptorMissionCompositionProvider,
    PropulsionProgram,
    ThrustProfileSchedule,
    ValueOrigin,
    flat_interceptor_profile_schema,
    interceptor,
    load_interceptor_profile,
    resolve_interceptor,
)


def _authored_schedule(**updates: object) -> ThrustProfileSchedule:
    values: dict[str, object] = {
        "schedule_id": "developer-pulse-shape-v1",
        "points": (
            {"burn_fraction": 0.0, "multiplier": 4.0},
            {"burn_fraction": 0.5, "multiplier": 2.0},
            {"burn_fraction": 1.0, "multiplier": 0.0},
        ),
        "origin": "simulation_assumption",
        "method": "focused developer-authored pulse shape",
    }
    values.update(updates)
    return ThrustProfileSchedule.model_validate(values)
    ####


def test_schedule_normalizes_raw_points_and_integrates_exactly() -> None:
    schedule = _authored_schedule()

    assert schedule.raw_area == pytest.approx(2.0)
    assert schedule.multiplier_at(0.0) == pytest.approx(2.0)
    assert schedule.multiplier_at(0.5) == pytest.approx(1.0)
    assert schedule.multiplier_at(1.0) == 0.0
    assert schedule.cumulative_fraction_at(0.5) == pytest.approx(0.75)
    assert schedule.cumulative_fraction_at(1.0) == 1.0
    assert schedule.normalized_peak == pytest.approx(2.0)
    assert len(schedule.fingerprint) == 64
    with pytest.raises(ValueError, match=r"lie in \[0, 1\]"):
        schedule.multiplier_at(1.1)
    ####


def test_schedule_supports_exact_step_previous_boost_sustain_shape() -> None:
    schedule = ThrustProfileSchedule(
        schedule_id="step-boost-sustain-v1",
        interpolation="step_previous",
        points=(
            {"burn_fraction": 0.0, "multiplier": 1.6},
            {"burn_fraction": 0.2, "multiplier": 0.85},
            {"burn_fraction": 1.0, "multiplier": 0.85},
        ),
    )

    assert schedule.raw_area == pytest.approx(1.0)
    assert schedule.multiplier_at(0.1) == pytest.approx(1.6)
    assert schedule.multiplier_at(0.2) == pytest.approx(0.85)
    assert schedule.cumulative_fraction_at(0.2) == pytest.approx(0.32)
    ####


def test_schedule_rejects_incomplete_zero_area_or_untraceable_authoring() -> None:
    with pytest.raises(ValidationError, match="must start"):
        _authored_schedule(
            points=(
                {"burn_fraction": 0.1, "multiplier": 1.0},
                {"burn_fraction": 1.0, "multiplier": 1.0},
            )
        )
    with pytest.raises(ValidationError, match="integrated multiplier must be positive"):
        _authored_schedule(
            points=(
                {"burn_fraction": 0.0, "multiplier": 0.0},
                {"burn_fraction": 1.0, "multiplier": 0.0},
            )
        )
    with pytest.raises(ValidationError, match="observed thrust schedule requires"):
        _authored_schedule(origin="observed")
    ####


def test_explicit_schedule_drives_common_program_without_changing_nominal_impulse() -> None:
    schedule = _authored_schedule(
        origin="observed",
        confidence="high",
        source_record_ids=("source:motor-static-test",),
    )
    resolved = resolve_interceptor(
        interceptor(
            "explicit-thrust-schedule",
            launch_mass_kg=100.0,
            burnout_mass_kg=60.0,
            burn_time_s=4.0,
            thrust_profile_class="neutral",
            thrust_profile_schedule=schedule,
        )
    )
    program = PropulsionProgram.from_profile(resolved)

    assert resolved.thrust_profile_schedule is schedule
    assert resolved.thrust_profile_schedule.origin == "observed"
    assert resolved.parameters["thrust_profile_class"].origin is ValueOrigin.SIMULATION_ASSUMPTION
    assert program.sample(0.0).thrust_n == pytest.approx(2.0 * resolved.number("thrust_n"))
    assert program.sample(2.0).thrust_n == pytest.approx(resolved.number("thrust_n"))
    assert program.sample(2.0).mass_kg == pytest.approx(70.0)
    assert program.nominal_total_impulse_ns == pytest.approx(resolved.number("thrust_n") * 4.0)
    ####


def test_flat_yaml_schema_and_composition_advertise_explicit_schedule() -> None:
    schema = flat_interceptor_profile_schema()
    schedule_schema = schema["properties"]["thrust_profile_schedule"]
    assert "$ref" in schedule_schema["anyOf"][0]
    assert "ThrustProfileSchedule" in schedule_schema["anyOf"][0]["$ref"]

    profile = load_interceptor_profile("examples/parametric_interceptors/custom_thrust_profile_sam.yaml")
    assert profile.thrust_profile_schedule is not None
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(profile)
    resolved = provider.resolved_profile(profile.model_id)
    properties = {item.id: item for item in provider.model(profile.model_id).presentation.properties}

    assert properties["thrust_profile_schedule.contract"].value == ("taoryx.parametric-interceptors.thrust-profile-schedule/v1")
    assert properties["thrust_profile_schedule.fingerprint"].value == resolved.thrust_profile_schedule.fingerprint
    assert properties["thrust_profile_schedule.raw_area"].value == pytest.approx(1.0)
    assert properties["thrust_profile_schedule.normalization"].value == "unit_mean_active_burn"
    ####


def test_observed_schedule_counts_as_evidence_in_model_presentation() -> None:
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(
        interceptor(
            "observed-thrust-shape-count",
            thrust_profile_schedule=_authored_schedule(
                origin="observed",
                confidence="high",
                source_record_ids=("source:static-test-shape",),
            ),
        )
    )
    properties = {item.id: item for item in provider.model("observed-thrust-shape-count").presentation.properties}

    assert properties["evidence_value_count"].value == 1
    ####
