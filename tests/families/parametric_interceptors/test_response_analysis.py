"""Focused response-law analysis and configuration-preflight tests."""

from __future__ import annotations

import math
from pathlib import Path

import pytest
from taoryx_parametric_interceptors import (
    ParametricInterceptorMissionCompositionProvider,
    PointMassMission,
    Pseudo6OperatingPointAnalysisRequest,
    Pseudo6ResponseAnalysisRequest,
    Pseudo6ResponseOperatingPoint,
    analyze_pseudo6_response,
    analyze_pseudo6_response_at_operating_point,
    compare_pseudo6_responses,
    compare_pseudo6_responses_at_operating_point,
    interceptor,
    load_pseudo6_operating_point_analysis_case,
    load_pseudo6_response_analysis_request,
    resolve_interceptor,
    resolve_pseudo6_response_operating_point,
    run_pseudo6_attitude_step,
    run_pseudo6_interceptor,
)

from taoryx.trajectory.configuration_contract import ConfigurationContractError

ROOT = Path(__file__).resolve().parents[3]


def _response_profile(
    model_id: str,
    *,
    bandwidth: float,
    damping: float = 0.7,
    maximum_rate: float = 2.0,
    maximum_acceleration: float = 20.0,
    maximum_bank: float = math.radians(60.0),
):
    return interceptor(
        model_id,
        attitude_bandwidth_rad_s=bandwidth,
        attitude_damping_ratio=damping,
        max_body_rate_rad_s=maximum_rate,
        max_body_acceleration_rad_s2=maximum_acceleration,
        max_bank_angle_rad=maximum_bank,
    )
    ####


def test_analysis_exposes_exact_continuous_poles_and_parameter_provenance() -> None:
    report = analyze_pseudo6_response(
        _response_profile("analytic-response", bandwidth=8.0),
        Pseudo6ResponseAnalysisRequest(
            analysis_id="analytic-response-check",
            sample_time_s=0.02,
        ),
    )

    assert report.continuous_status == "stable"
    assert report.continuous_spectral_abscissa_rad_s == pytest.approx(-5.6)
    assert [abs(item.imaginary) for item in report.continuous_poles] == pytest.approx([8.0 * math.sqrt(1.0 - 0.7**2)] * 2)
    assert report.step_overshoot_fraction == pytest.approx(math.exp(-math.pi * 0.7 / math.sqrt(1.0 - 0.7**2)))
    assert report.discrete_status == "stable"
    assert report.sampling_quality == "adequate"
    assert {item.parameter_id for item in report.parameter_trace} == {
        "attitude_bandwidth_rad_s",
        "attitude_damping_ratio",
        "max_body_acceleration_rad_s2",
        "max_body_rate_rad_s",
        "max_bank_angle_rad",
    }
    assert {item.origin.value for item in report.parameter_trace} == {"simulation_assumption"}
    assert report.local_unsaturated_continuous_linear_claim is True
    assert report.physical_controller_qualification_claim is False
    assert report.airframe_stability_claim is False
    assert report.command_support_fraction == 1.0
    assert report.effective_natural_frequency_rad_s == report.natural_frequency_rad_s
    assert report.effective_damping_ratio == report.damping_ratio
    assert "frozen command-support" in report.claim_boundary
    assert "fraction of one is nominal full support" in report.claim_boundary
    ####


def test_frozen_command_support_scales_the_runtime_response_equation_and_handles_zero_authority() -> None:
    profile = _response_profile("authority-coupled-response", bandwidth=8.0)
    full = analyze_pseudo6_response(
        profile,
        Pseudo6ResponseAnalysisRequest(sample_time_s=0.02),
    )
    half = analyze_pseudo6_response(
        profile,
        Pseudo6ResponseAnalysisRequest(
            analysis_id="half-support",
            sample_time_s=0.02,
            command_support_fraction=0.5,
        ),
    )
    zero = analyze_pseudo6_response(
        profile,
        Pseudo6ResponseAnalysisRequest(
            analysis_id="zero-support",
            sample_time_s=0.02,
            command_support_fraction=0.0,
        ),
    )

    assert half.authority_coupling_assumption == "frozen_command_support_fraction"
    assert half.response_authority_available
    assert half.effective_natural_frequency_rad_s == pytest.approx(8.0 * math.sqrt(0.5))
    assert half.effective_damping_ratio == pytest.approx(0.7 * math.sqrt(0.5))
    assert half.initial_angular_acceleration_demand_rad_s2 == pytest.approx(0.5 * half.pre_support_initial_angular_acceleration_demand_rad_s2)
    assert half.authority_scaled_maximum_body_acceleration_rad_s2 == pytest.approx(0.5 * half.maximum_body_acceleration_rad_s2)
    assert half.continuous_stability_margin_rad_s < full.continuous_stability_margin_rad_s
    assert half.maximum_stable_sample_time_s is not None
    assert full.maximum_stable_sample_time_s is not None
    assert half.maximum_stable_sample_time_s > full.maximum_stable_sample_time_s

    assert not zero.response_authority_available
    assert zero.effective_natural_frequency_rad_s == 0.0
    assert zero.effective_damping_ratio == 0.0
    assert all(pole.real == pole.imaginary == pole.magnitude == 0.0 for pole in zero.continuous_poles)
    assert all(pole.real == pole.magnitude == 1.0 and pole.imaginary == 0.0 for pole in zero.discrete_poles)
    assert zero.continuous_status == zero.discrete_status == "marginal"
    assert zero.initial_angular_acceleration_demand_rad_s2 == 0.0
    assert zero.maximum_stable_sample_time_s is None
    assert zero.sample_time_fraction_of_stability_limit is None
    assert zero.sampling_quality == "not_applicable"
    assert zero.continuous_settling_time_estimate_s is None
    assert zero.discrete_settling_time_estimate_s is None
    assert zero.body_rate_headroom_ratio is None
    ####


def test_operating_point_resolves_support_with_the_shared_force_authority_equation() -> None:
    profile = interceptor(
        "operating-point-authority",
        reference_area_m2=0.1,
        normal_force_coefficient_limit=2.0,
        control_configuration="aerodynamic",
    )
    operating_point = Pseudo6ResponseOperatingPoint(
        operating_point_id="q5000-command20",
        dynamic_pressure_pa=5_000.0,
        mass_kg=100.0,
        thrust_n=0.0,
        commanded_lateral_acceleration_mps2=20.0,
        source_basis="Focused q*S*Cn/m equation witness.",
    )

    resolution = resolve_pseudo6_response_operating_point(profile, operating_point)
    report = analyze_pseudo6_response_at_operating_point(
        profile,
        operating_point,
        Pseudo6OperatingPointAnalysisRequest(
            analysis_id="q5000-operating-response",
            sample_time_s=0.02,
        ),
    )

    assert resolution.aerodynamic_authority_mps2 == pytest.approx(10.0)
    assert resolution.thrust_vector_authority_mps2 == 0.0
    assert resolution.available_authority_mps2 == pytest.approx(10.0)
    assert resolution.command_support_fraction == pytest.approx(0.5)
    assert resolution.response_support_fraction == pytest.approx(0.5)
    assert resolution.force_authority_available is True
    assert resolution.zero_command_availability_convention_applied is False
    assert {item.parameter_id for item in resolution.authority_parameter_trace} == {
        "reference_area_m2",
        "normal_force_coefficient_limit",
        "max_thrust_vector_angle_rad",
        "max_lateral_acceleration_mps2",
    }
    assert report.operating_point_resolution == resolution
    assert report.response_analysis.command_support_fraction == pytest.approx(0.5)
    assert report.response_analysis.operating_point_id == "q5000-command20"
    assert report.response_analysis.effective_natural_frequency_rad_s == pytest.approx(report.response_analysis.natural_frequency_rad_s * math.sqrt(0.5))
    assert report.response_analysis.physical_controller_qualification_claim is False
    assert "same q*S*Cn/m" in report.claim_boundary
    ####


def test_runtime_sample_constructor_reproduces_pseudo6_authority_support() -> None:
    profile = resolve_interceptor(_response_profile("runtime-operating-point", bandwidth=6.0))
    run = run_pseudo6_interceptor(
        profile,
        PointMassMission(
            launch_altitude_m=1_000.0,
            launch_speed_mps=150.0,
            objective_kind="direct_lateral_acceleration",
            direct_lateral_acceleration_east_mps2=20.0,
            duration_s=0.05,
            time_step_s=0.05,
        ),
    )
    sample = run.samples[0]
    operating_point = Pseudo6ResponseOperatingPoint.from_sample(
        sample,
        operating_point_id="initial-runtime-sample",
    )
    resolution = resolve_pseudo6_response_operating_point(profile, operating_point)

    assert operating_point.source_sample_time_s == sample.time_s
    assert operating_point.source_phase_id == sample.phase_id
    assert resolution.aerodynamic_authority_mps2 == pytest.approx(sample.aerodynamic_lateral_authority_mps2)
    assert resolution.thrust_vector_authority_mps2 == pytest.approx(sample.thrust_vector_lateral_authority_mps2)
    assert resolution.available_authority_mps2 == pytest.approx(sample.lateral_acceleration_available_mps2)
    assert resolution.command_support_fraction == pytest.approx(sample.attitude_response_command_support_fraction)
    ####


def test_zero_command_operating_point_preserves_runtime_authority_availability_convention() -> None:
    profile = _response_profile("zero-command-operating-point", bandwidth=8.0)
    unavailable_point = Pseudo6ResponseOperatingPoint(
        operating_point_id="vacuum-coast-zero-command",
        dynamic_pressure_pa=0.0,
        mass_kg=100.0,
        thrust_n=0.0,
        commanded_lateral_acceleration_mps2=0.0,
    )
    available_point = unavailable_point.model_copy(
        update={
            "operating_point_id": "dynamic-pressure-zero-command",
            "dynamic_pressure_pa": 5_000.0,
        }
    )

    unavailable = analyze_pseudo6_response_at_operating_point(profile, unavailable_point)
    available = analyze_pseudo6_response_at_operating_point(profile, available_point)

    assert unavailable.operating_point_resolution.command_support_fraction == 1.0
    assert unavailable.operating_point_resolution.response_support_fraction == 0.0
    assert unavailable.operating_point_resolution.force_authority_available is False
    assert unavailable.response_analysis.continuous_status == "marginal"
    assert available.operating_point_resolution.command_support_fraction == 1.0
    assert available.operating_point_resolution.response_support_fraction == 1.0
    assert available.operating_point_resolution.force_authority_available is True
    assert available.response_analysis.continuous_status == "stable"
    ####


def test_operating_point_comparison_separates_authority_support_from_tuning() -> None:
    baseline = interceptor(
        "low-authority-response",
        reference_area_m2=0.05,
        normal_force_coefficient_limit=2.0,
        control_configuration="aerodynamic",
        attitude_bandwidth_rad_s=5.0,
    )
    candidate = interceptor(
        "high-authority-response",
        reference_area_m2=0.20,
        normal_force_coefficient_limit=2.0,
        control_configuration="aerodynamic",
        attitude_bandwidth_rad_s=5.0,
    )
    operating_point = Pseudo6ResponseOperatingPoint(
        operating_point_id="matched-q5000-force-point",
        dynamic_pressure_pa=5_000.0,
        mass_kg=100.0,
        thrust_n=0.0,
        commanded_lateral_acceleration_mps2=20.0,
    )
    request = Pseudo6OperatingPointAnalysisRequest(
        analysis_id="matched-force-comparison",
        sample_time_s=0.02,
    )

    comparison = compare_pseudo6_responses_at_operating_point(
        baseline,
        candidate,
        operating_point,
        request,
    )

    assert comparison.baseline.operating_point_resolution.available_authority_mps2 == pytest.approx(5.0)
    assert comparison.candidate.operating_point_resolution.available_authority_mps2 == pytest.approx(20.0)
    assert comparison.available_authority_delta_mps2 == pytest.approx(15.0)
    assert comparison.response_support_fraction_delta == pytest.approx(0.75)
    assert comparison.greater_available_authority_model_id == "high-authority-response"
    assert comparison.greater_response_support_model_id == "high-authority-response"
    assert comparison.faster_settling_model_id == "high-authority-response"
    assert comparison.overall_winner_model_id is None
    assert comparison.physical_controller_qualification_claim is False
    assert any("available lateral authority" in note for note in comparison.tradeoffs)
    assert any("response support" in note for note in comparison.tradeoffs)

    with pytest.raises(ValueError, match="distinct model IDs"):
        compare_pseudo6_responses_at_operating_point(
            baseline,
            baseline,
            operating_point,
            request,
        )
    ####


@pytest.mark.parametrize(
    ("damping", "expected_poles"),
    (
        (1.0, (-8.0, -8.0)),
        (1.25, (-4.0, -16.0)),
    ),
)
def test_critical_and_overdamped_continuous_poles(
    damping: float,
    expected_poles: tuple[float, float],
) -> None:
    report = analyze_pseudo6_response(
        _response_profile("nonoscillatory-response", bandwidth=8.0, damping=damping),
        Pseudo6ResponseAnalysisRequest(sample_time_s=0.01),
    )

    assert sorted(item.real for item in report.continuous_poles) == pytest.approx(sorted(expected_poles))
    assert all(abs(item.imaginary) < 1.0e-9 for item in report.continuous_poles)
    assert report.step_overshoot_fraction == 0.0
    ####


def test_discrete_preflight_distinguishes_continuous_response_from_update_stability() -> None:
    profile = _response_profile("fast-response", bandwidth=10.0, damping=0.9)
    stable = analyze_pseudo6_response(
        profile,
        Pseudo6ResponseAnalysisRequest(
            analysis_id="stable-fast-response",
            sample_time_s=0.05,
        ),
    )
    unstable = analyze_pseudo6_response(
        profile,
        Pseudo6ResponseAnalysisRequest(
            analysis_id="unstable-fast-response",
            sample_time_s=0.1,
        ),
    )

    assert stable.continuous_status == unstable.continuous_status == "stable"
    assert stable.discrete_status == "stable"
    assert stable.discrete_spectral_radius < 1.0
    assert unstable.discrete_status == "unstable"
    assert unstable.discrete_spectral_radius > 1.0
    assert unstable.sampling_quality == "unstable"
    assert stable.maximum_stable_sample_time_s == pytest.approx(2.0 * (math.sqrt(1.0 + 0.9**2) - 0.9) / 10.0)
    ####


def test_step_demand_reports_authority_headroom_without_simulating_saturation() -> None:
    profile = _response_profile(
        "limited-response",
        bandwidth=8.0,
        maximum_rate=0.5,
        maximum_acceleration=8.0,
        maximum_bank=math.radians(15.0),
    )
    report = analyze_pseudo6_response(
        profile,
        Pseudo6ResponseAnalysisRequest(command_step_rad=math.radians(20.0)),
    )

    assert report.predicted_acceleration_saturation is True
    assert report.predicted_body_rate_saturation is True
    assert report.analysis_axis == "roll"
    assert report.angle_topology == "bounded"
    assert report.command_step_limited is True
    assert report.effective_command_step_rad == pytest.approx(math.radians(15.0))
    assert report.predicted_angle_saturation is True
    assert report.acceleration_headroom_ratio < 1.0
    assert report.body_rate_headroom_ratio < 1.0
    assert report.angle_headroom_rad is not None
    assert report.angle_headroom_rad < 0.0
    ####


def test_axis_metadata_separates_symmetric_linear_law_from_angle_topology() -> None:
    profile = _response_profile("axis-topology", bandwidth=5.0, maximum_bank=math.radians(55.0))
    roll = analyze_pseudo6_response(
        profile,
        Pseudo6ResponseAnalysisRequest(axis="roll"),
    )
    pitch = analyze_pseudo6_response(
        profile,
        Pseudo6ResponseAnalysisRequest(axis="pitch"),
    )
    yaw = analyze_pseudo6_response(
        profile,
        Pseudo6ResponseAnalysisRequest(axis="yaw"),
    )

    assert roll.linear_response_axes_symmetric is True
    assert roll.angle_topology_axis_specific is True
    assert roll.angle_limit_rad == pytest.approx(math.radians(55.0))
    assert "origin=simulation_assumption" in roll.angle_limit_provenance
    assert pitch.angle_topology == "bounded"
    assert pitch.angle_limit_rad == pytest.approx(math.radians(89.0))
    assert "pitch-coordinate guard" in pitch.angle_limit_provenance
    assert "max_bank_angle_rad" not in {item.parameter_id for item in pitch.parameter_trace}
    assert yaw.angle_topology == "periodic"
    assert yaw.angle_limit_rad is None
    assert yaw.angle_headroom_ratio is None
    assert yaw.angle_headroom_rad is None
    assert yaw.predicted_angle_saturation is False
    ####


def test_unsaturated_step_witness_agrees_with_analytic_demand_and_response() -> None:
    profile = _response_profile(
        "unsaturated-step",
        bandwidth=4.0,
        damping=0.9,
        maximum_rate=10.0,
        maximum_acceleration=100.0,
    )
    request = Pseudo6ResponseAnalysisRequest(
        analysis_id="unsaturated-pitch-step",
        axis="pitch",
        sample_time_s=0.01,
        command_step_rad=math.radians(5.0),
        command_support_fraction=0.5,
    )
    report = analyze_pseudo6_response(profile, request)
    run = run_pseudo6_attitude_step(
        resolve_interceptor(profile),
        axis=request.axis,
        command_step_rad=request.command_step_rad,
        duration_s=3.0,
        time_step_s=request.sample_time_s,
        settling_band_fraction=request.settling_band_fraction,
        command_support_fraction=request.command_support_fraction,
    )

    assert run.samples[0].axis_acceleration_rad_s2 == pytest.approx(report.initial_angular_acceleration_demand_rad_s2)
    assert run.command_limited is False
    assert run.acceleration_limited is False
    assert run.rate_limited is False
    assert run.angle_limited is False
    assert run.samples[-1].angle_rad == pytest.approx(request.command_step_rad, rel=0.02)
    peak_overshoot = max(sample.angle_rad for sample in run.samples) / request.command_step_rad - 1.0
    assert peak_overshoot == pytest.approx(report.step_overshoot_fraction, abs=0.01)
    assert run.settling_time_s is not None
    assert report.discrete_settling_time_estimate_s is not None
    assert run.settling_time_s == pytest.approx(
        report.discrete_settling_time_estimate_s,
        abs=0.35,
    )
    assert run.command_support_fraction == 0.5
    assert all(sample.command_support_fraction == 0.5 for sample in run.samples)
    assert "frozen command-support fraction" in run.claim_boundary
    assert "changing environment" in run.claim_boundary
    ####


def test_bounded_step_witness_confirms_predicted_roll_saturation() -> None:
    profile = _response_profile(
        "bounded-step",
        bandwidth=8.0,
        maximum_rate=0.5,
        maximum_acceleration=8.0,
        maximum_bank=math.radians(15.0),
    )
    request = Pseudo6ResponseAnalysisRequest(
        analysis_id="bounded-roll-step",
        axis="roll",
        sample_time_s=0.01,
        command_step_rad=math.radians(20.0),
    )
    report = analyze_pseudo6_response(profile, request)
    run = run_pseudo6_attitude_step(
        resolve_interceptor(profile),
        axis=request.axis,
        command_step_rad=request.command_step_rad,
        duration_s=3.0,
        time_step_s=request.sample_time_s,
    )

    assert report.predicted_acceleration_saturation is True
    assert report.predicted_body_rate_saturation is True
    assert report.predicted_angle_saturation is True
    assert run.command_limited is True
    assert run.acceleration_limited is True
    assert run.rate_limited is True
    assert run.angle_limited is True
    assert run.effective_command_rad == pytest.approx(math.radians(15.0))
    assert max(abs(sample.angle_rad) for sample in run.samples) <= math.radians(15.0) + 1.0e-12
    ####


def test_comparison_keeps_response_speed_and_authority_as_separate_tradeoffs() -> None:
    slow = _response_profile("slow-response", bandwidth=4.0)
    fast = _response_profile("fast-response", bandwidth=8.0)
    comparison = compare_pseudo6_responses(
        slow,
        fast,
        Pseudo6ResponseAnalysisRequest(
            analysis_id="same-request-comparison",
            sample_time_s=0.02,
        ),
    )

    assert comparison.faster_settling_model_id == "fast-response"
    assert comparison.greater_acceleration_headroom_model_id == "slow-response"
    assert comparison.greater_body_rate_headroom_model_id == "slow-response"
    assert comparison.continuous_settling_time_delta_s is not None
    assert comparison.continuous_settling_time_delta_s < 0.0
    assert comparison.acceleration_headroom_ratio_delta < 0.0
    assert comparison.overall_winner_model_id is None
    assert comparison.physical_controller_qualification_claim is False
    assert any("settles faster" in item for item in comparison.tradeoffs)
    assert any("headroom" in item for item in comparison.tradeoffs)
    ####


def test_provider_advertises_analysis_and_rejects_only_unstable_pseudo6_step() -> None:
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(
        _response_profile("provider-fast-response", bandwidth=10.0, damping=0.9),
        _response_profile("provider-slow-response", bandwidth=5.0, damping=0.9),
    )
    properties = {item.id: item for item in provider.model("provider-fast-response").presentation.properties}

    assert properties["response_analysis_status"].value == "available_local_unsaturated_frozen_command_support"
    assert properties["response_analysis.authority_coupling"].value == ("request_frozen_command_support_fraction_inclusive_0_to_1")
    assert properties["response_comparison_status"].value == "available_like_for_like"
    assert properties["default_response_discrete_status"].value == "stable"
    assert properties["response_analysis_axes"].value == "roll, pitch, yaw"
    assert "yaw=periodic" in str(properties["response_angle_topology"].value)
    assert properties["response_step_witness_status"].value == "available_via_provider_api"
    assert properties["response_operating_point_analysis_status"].value == ("available_shared_force_authority_binding_v1")
    assert properties["response_operating_point_comparison_status"].value == ("available_matched_force_point_authority_and_tuning_deltas_v1")
    assert provider.analyze_pseudo6_response("provider-fast-response").discrete_status == "stable"
    operating_report = provider.analyze_pseudo6_response_at_operating_point(
        "provider-fast-response",
        Pseudo6ResponseOperatingPoint(
            operating_point_id="provider-q-point",
            dynamic_pressure_pa=2_000.0,
            mass_kg=100.0,
            thrust_n=0.0,
            commanded_lateral_acceleration_mps2=10.0,
        ),
    )
    assert operating_report.response_analysis.operating_point_id == "provider-q-point"
    assert operating_report.operating_point_resolution.model_id == "provider-fast-response"
    operating_comparison = provider.compare_pseudo6_responses_at_operating_point(
        "provider-slow-response",
        "provider-fast-response",
        Pseudo6ResponseOperatingPoint(
            operating_point_id="provider-comparison-point",
            dynamic_pressure_pa=2_000.0,
            mass_kg=100.0,
            thrust_n=0.0,
            commanded_lateral_acceleration_mps2=10.0,
        ),
    )
    assert operating_comparison.overall_winner_model_id is None
    assert operating_comparison.baseline.response_analysis.model_id == "provider-slow-response"
    assert (
        provider.run_pseudo6_attitude_step(
            "provider-fast-response",
            axis="pitch",
            command_step_rad=math.radians(2.0),
            duration_s=0.1,
            time_step_s=0.01,
        )
        .samples[-1]
        .time_s
        == 0.1
    )
    assert provider.validate_configuration(
        provider.configuration(
            "provider-fast-response",
            fidelity="point_mass_3dof",
            runtime_time_step_s=0.1,
        )
    )
    assert provider.validate_configuration(
        provider.configuration(
            "provider-fast-response",
            fidelity="attitude_response_pseudo_6dof",
            runtime_time_step_s=0.05,
        )
    )
    with pytest.raises(ConfigurationContractError) as caught:
        provider.validate_configuration(
            provider.configuration(
                "provider-fast-response",
                fidelity="attitude_response_pseudo_6dof",
                runtime_time_step_s=0.1,
            )
        )
    assert caught.value.code == "unstable-response-discretization"
    assert caught.value.path == "configuration.root.runtime.time_step_s"
    assert "spectral radius" in str(caught.value)
    ####


def test_copy_ready_yaml_request_loads_with_stable_fingerprint() -> None:
    request = load_pseudo6_response_analysis_request(ROOT / "examples/parametric_interceptors/generic_medium_sam_response_analysis.yaml")

    assert request.analysis_id == "generic-medium-sam-response-v1"
    assert request.sample_time_s == 0.05
    assert request.command_step_rad == pytest.approx(math.radians(10.0))
    assert request.command_support_fraction == 0.6
    assert len(request.fingerprint) == 64
    assert request.fingerprint == Pseudo6ResponseAnalysisRequest.model_validate(request.model_dump(mode="json")).fingerprint
    ####


def test_copy_ready_operating_point_case_loads_with_separate_point_and_analysis_identity() -> None:
    case = load_pseudo6_operating_point_analysis_case(ROOT / "examples/parametric_interceptors/generic_medium_sam_operating_point_response.yaml")

    assert case.operating_point.operating_point_id == "boost-turn-local-point"
    assert case.analysis.analysis_id == "generic-medium-sam-operating-response-v1"
    assert case.operating_point.commanded_lateral_acceleration_mps2 == 40.0
    assert len(case.operating_point.fingerprint) == 64
    assert len(case.analysis.fingerprint) == 64
    assert len(case.fingerprint) == 64
    ####
