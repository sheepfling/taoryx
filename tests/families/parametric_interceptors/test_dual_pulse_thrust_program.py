"""Focused source-shaped dual-pulse compilation and runtime witnesses."""

from __future__ import annotations

import pytest
from pydantic import ValidationError
from taoryx_parametric_interceptors import (
    AbsoluteThrustCurve,
    DualPulseThrustProgram,
    ParametricInterceptorMissionCompositionProvider,
    PointMassMission,
    PropulsionPhase,
    PropulsionProgram,
    ValueOrigin,
    build_interceptor_authoring_report,
    flat_interceptor_profile_schema,
    interceptor,
    interceptor_from_catalogue_record,
    interceptor_parameter_usage,
    load_interceptor_profile,
    resolve_interceptor,
    run_point_mass_interceptor,
    run_pseudo6_interceptor,
)


def _pulse(
    curve_id: str,
    *,
    duration_s: float,
    peak_thrust_n: float,
    **updates: object,
) -> AbsoluteThrustCurve:
    values: dict[str, object] = {
        "curve_id": curve_id,
        "points": (
            {"time": 0.0, "thrust": 0.0},
            {"time": duration_s / 2.0, "thrust": peak_thrust_n},
            {"time": duration_s, "thrust": 0.0},
        ),
        "method": f"focused {curve_id} triangular pulse",
    }
    values.update(updates)
    return AbsoluteThrustCurve.model_validate(values)
    ####


def _program(**updates: object) -> DualPulseThrustProgram:
    values: dict[str, object] = {
        "program_id": "two-curve-motor-v1",
        "first_pulse": _pulse(
            "first-pulse-v1",
            duration_s=1.0,
            peak_thrust_n=100.0,
        ),
        "inter_pulse_coast_time": 3.0,
        "second_pulse": _pulse(
            "second-pulse-v1",
            duration_s=2.0,
            peak_thrust_n=50.0,
        ),
        "method": "focused source-shaped dual-pulse witness",
    }
    values.update(updates)
    return DualPulseThrustProgram.model_validate(values)
    ####


def test_dual_program_derives_timing_impulse_mean_and_ratio() -> None:
    program = _program()

    assert program.first_pulse.duration_s == 1.0
    assert program.second_pulse.duration_s == 2.0
    assert program.inter_pulse_coast_time_s == 3.0
    assert program.active_burn_time_s == 3.0
    assert program.duration_s == 6.0
    assert program.first_pulse.total_impulse_n_s == 50.0
    assert program.second_pulse.total_impulse_n_s == 50.0
    assert program.total_impulse_n_s == 100.0
    assert program.mean_active_burn_thrust_n == pytest.approx(100.0 / 3.0)
    assert program.second_to_first_mean_thrust_ratio == 0.5
    assert len(program.fingerprint) == 64
    ####


def test_compiler_removes_coarse_baggage_and_runtime_reproduces_both_curves() -> None:
    resolved = resolve_interceptor(
        interceptor(
            "compiled-dual-curves",
            launch_mass_kg=100.0,
            burnout_mass_kg=60.0,
            dual_pulse_thrust_program=_program(),
        )
    )
    runtime = PropulsionProgram.from_profile(resolved)
    usage = interceptor_parameter_usage(resolved)

    assert resolved.text("propulsion_architecture") == "dual_pulse_solid"
    assert resolved.number("burn_time_s") == 3.0
    assert resolved.number("first_pulse_burn_time_s") == 1.0
    assert resolved.number("inter_pulse_coast_time_s") == 3.0
    assert resolved.number("second_pulse_burn_time_s") == 2.0
    assert resolved.number("nominal_thrust_n") == pytest.approx(100.0 / 3.0)
    assert resolved.number("second_pulse_thrust_ratio") == 0.5
    assert resolved.number("active_burn_total_impulse_n_s") == pytest.approx(100.0)
    assert "burn_time_class" not in resolved.parameters
    assert "thrust_profile_class" not in resolved.parameters
    assert resolved.second_pulse_thrust_profile_schedule is not None
    assert resolved.thrust_profile_schedule.schedule_id == "first-pulse-v1-normalized-v1"
    assert resolved.second_pulse_thrust_profile_schedule.schedule_id == ("second-pulse-v1-normalized-v1")

    assert runtime.sample(0.5).thrust_n == pytest.approx(100.0)
    assert runtime.sample(0.5).mass_kg == pytest.approx(90.0)
    assert runtime.sample(1.0).phase is PropulsionPhase.INTER_PULSE_COAST
    assert runtime.sample(1.0).mass_kg == 80.0
    assert runtime.sample(5.0).phase is PropulsionPhase.SECOND_PULSE
    assert runtime.sample(5.0).thrust_n == pytest.approx(50.0)
    assert runtime.sample(5.0).mass_kg == 70.0
    assert runtime.sample(6.0).phase is PropulsionPhase.BURNOUT
    assert runtime.nominal_total_impulse_ns == pytest.approx(100.0)

    assert usage["dual_pulse_thrust_program"].usage_class == "resolution_input"
    assert set(usage["dual_pulse_thrust_program"].resolved_sink_ids) == {
        "first_pulse_burn_time_s",
        "inter_pulse_coast_time_s",
        "propulsion_architecture",
        "second_pulse_burn_time_s",
        "second_pulse_thrust_profile_schedule",
        "second_pulse_propellant_fraction",
        "second_pulse_thrust_ratio",
        "thrust_n",
        "thrust_profile_schedule",
    }
    assert usage["second_pulse_thrust_profile_schedule"].usage_class == ("dynamics_input")
    ####


def test_both_fidelity_tiers_consume_identical_dual_curve_thrust_history() -> None:
    resolved = resolve_interceptor(
        interceptor(
            "dual-curve-tier-parity",
            launch_mass_kg=100.0,
            burnout_mass_kg=60.0,
            dual_pulse_thrust_program=_program(),
        )
    )
    mission = PointMassMission(
        launch_altitude_m=1_000.0,
        launch_speed_mps=100.0,
        launch_flight_path_deg=0.0,
        waypoint_north_m=20_000.0,
        waypoint_altitude_m=1_000.0,
        duration_s=6.0,
        time_step_s=0.5,
    )
    point = run_point_mass_interceptor(resolved, mission)
    pseudo = run_pseudo6_interceptor(resolved, mission)

    assert [item.time_s for item in point.samples] == [item.time_s for item in pseudo.samples]
    assert [item.thrust_n for item in point.samples] == pytest.approx([item.thrust_n for item in pseudo.samples])
    assert [item.propulsion_phase for item in point.samples] == [item.propulsion_phase for item in pseudo.samples]
    ####


def test_program_rejects_untraceable_claims_conflicts_and_wrong_architecture() -> None:
    with pytest.raises(ValidationError, match="observed dual-pulse program requires"):
        _program(origin="observed")
    with pytest.raises(ValidationError, match="derives architecture"):
        interceptor(
            "overlapping-dual-program",
            dual_pulse_thrust_program=_program(),
            second_pulse_burn_time_s=2.0,
        )
    with pytest.raises(ValidationError, match="requires propulsion_architecture"):
        interceptor(
            "wrong-dual-program-architecture",
            propulsion_architecture="single_stage_solid",
            dual_pulse_thrust_program=_program(),
        )
    ####


def test_catalogue_composition_and_authoring_preserve_dual_source_lineage() -> None:
    profile = interceptor_from_catalogue_record(
        {
            "kind": "interceptor_evidence_record",
            "interceptor_id": "int:dual-source-curves",
            "dual_pulse_thrust_program": {
                "program_id": "catalogue-dual-program-v1",
                "first_pulse": {
                    "curve_id": "catalogue-first-pulse-v1",
                    "time_unit": "ms",
                    "thrust_unit": "kN",
                    "points": [
                        {"time": 0.0, "thrust": 0.0},
                        {"time": 500.0, "thrust": 20.0},
                        {"time": 1000.0, "thrust": 0.0},
                    ],
                    "origin": "observed",
                    "source_record_ids": ["source:first-pulse-plot"],
                    "confidence": "high",
                    "method": "digitized first-pulse plot",
                },
                "inter_pulse_coast_time": 2500.0,
                "coast_time_unit": "ms",
                "second_pulse": {
                    "curve_id": "catalogue-second-pulse-v1",
                    "time_unit": "ms",
                    "thrust_unit": "kN",
                    "points": [
                        {"time": 0.0, "thrust": 0.0},
                        {"time": 1000.0, "thrust": 10.0},
                        {"time": 2000.0, "thrust": 0.0},
                    ],
                    "origin": "reported",
                    "source_record_ids": ["source:second-pulse-plot"],
                    "confidence": "medium",
                    "method": "digitized second-pulse plot",
                },
                "origin": "reported",
                "source_record_ids": ["source:pulse-timing"],
                "confidence": "medium",
                "method": "public dual-pulse timing record",
            },
        }
    )
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(profile)
    resolved = provider.resolved_profile(profile.model_id)
    properties = {item.id: item for item in provider.model(profile.model_id).presentation.properties}

    assert resolved.number("first_pulse_burn_time_s") == 1.0
    assert resolved.number("inter_pulse_coast_time_s") == 2.5
    assert resolved.number("second_pulse_burn_time_s") == 2.0
    assert resolved.number("nominal_thrust_n") == pytest.approx(20_000.0 / 3.0)
    assert resolved.parameters["first_pulse_burn_time_s"].origin is ValueOrigin.DERIVED
    assert resolved.parameters["inter_pulse_coast_time_s"].origin is ValueOrigin.DERIVED
    assert properties["dual_pulse_thrust_program.contract"].value == ("taoryx.parametric-interceptors.dual-pulse-thrust-program/v1")
    assert properties["dual_pulse_thrust_program.coast_time.canonical"].value == 2.5
    assert properties["dual_pulse_thrust_program.first_pulse.points"].source_refs == ("source:first-pulse-plot",)
    assert properties["second_pulse_thrust_profile_schedule.fingerprint"].value == (
        resolved.second_pulse_thrust_profile_schedule.fingerprint if resolved.second_pulse_thrust_profile_schedule is not None else None
    )
    assert properties["evidence_value_count"].value == 3
    ####


def test_flat_schema_example_and_authoring_report_expose_dual_program() -> None:
    schema = flat_interceptor_profile_schema()
    program_schema = schema["properties"]["dual_pulse_thrust_program"]
    assert "$ref" in program_schema["anyOf"][0]
    assert "DualPulseThrustProgram" in program_schema["anyOf"][0]["$ref"]

    path = "examples/parametric_interceptors/dual_pulse_thrust_curves_sam.yaml"
    profile = load_interceptor_profile(path)
    resolved = resolve_interceptor(profile)
    report = build_interceptor_authoring_report(path)

    assert profile.dual_pulse_thrust_program is not None
    assert resolved.text("propulsion_architecture") == "dual_pulse_solid"
    assert report.parameter_usage["dual_pulse_thrust_program"].usage_class == ("resolution_input")
    assert "dual_pulse_thrust_program" in report.readiness.assumption_parameter_ids
    assert "second_pulse_thrust_profile_schedule" in report.readiness.assumption_parameter_ids
    assert sum(report.usage_counts.values()) == len(report.resolved_profile.parameters) + 4
    ####
