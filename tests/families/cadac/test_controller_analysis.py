"""Controller-neutral CADAC analysis tests."""

from __future__ import annotations

import pytest
from taoryx.families.cadac.controller_analysis import (
    CadacControllerTrace,
    analyze_closed_loop_state_matrix,
    analyze_controller_trace,
    cadac_controller_trace_from_samples,
    compare_controller_traces,
)


def _trace(controller_id: str, response: tuple[float, ...]) -> CadacControllerTrace:
    return CadacControllerTrace(
        controller_id=controller_id,
        model_id="cadac.test.vehicle",
        channel_id="normal_acceleration_g",
        unit="g",
        time_s=(0.0, 0.5, 1.0, 1.5, 2.0),
        reference=(0.0, 1.0, 1.0, 1.0, 1.0),
        response=response,
        requested_control=(0.0, 2.0, 1.0, 0.5, 0.2),
        realized_control=(0.0, 1.8, 1.0, 0.5, 0.2),
        saturated=(False, True, False, False, False),
        provenance="synthetic accepted-boundary controller witness",
    )
    ####


def test_time_domain_analysis_reports_tracking_without_claiming_stability() -> None:
    report = analyze_controller_trace(
        _trace("source", (0.0, 0.7, 0.96, 1.0, 1.0)),
        settling_absolute_tolerance=0.05,
    )

    assert report.tracking_pass
    assert report.settling_time_s == pytest.approx(0.5)
    assert report.saturation_fraction == pytest.approx(0.2)
    assert report.control_tracking_rmse is not None
    assert report.formal_stability_claim is False
    ####


def test_controller_comparison_requires_identical_experiment_and_reports_deltas() -> None:
    baseline = _trace("source", (0.0, 0.5, 0.8, 0.9, 0.95))
    candidate = _trace("candidate", (0.0, 0.8, 1.0, 1.0, 1.0))

    report = compare_controller_traces(
        baseline,
        candidate,
        settling_absolute_tolerance=0.05,
    )

    assert report.rmse_delta < 0.0
    assert report.lower_rmse_controller_id == "candidate"
    assert report.formal_stability_claim is False

    mismatched = candidate.model_copy(update={"time_s": (0.0, 0.4, 1.0, 1.5, 2.0)})
    with pytest.raises(ValueError, match="identical time and reference"):
        compare_controller_traces(baseline, mismatched)
    ####


def test_standard_output_extractor_builds_analyzable_scalar_controller_trace() -> None:
    samples = (
        {
            "time_s": 0.0,
            "values": {
                "normal_command_g": 0.0,
                "normal_acceleration_g": 0.0,
                "fin_request_deg": 0.0,
                "fin_achieved_deg": 0.0,
            },
        },
        {
            "time_s": 0.5,
            "values": {
                "normal_command_g": 1.0,
                "normal_acceleration_g": 0.8,
                "fin_request_deg": 2.0,
                "fin_achieved_deg": 1.8,
            },
        },
        {
            "time_s": 1.0,
            "values": {
                "normal_command_g": 1.0,
                "normal_acceleration_g": 1.0,
                "fin_request_deg": 1.0,
                "fin_achieved_deg": 1.0,
            },
        },
    )

    trace = cadac_controller_trace_from_samples(
        samples,
        controller_id="source",
        model_id="cadac.test.vehicle",
        reference_channel_id="normal_command_g",
        response_channel_id="normal_acceleration_g",
        unit="g",
        requested_control_channel_id="fin_request_deg",
        realized_control_channel_id="fin_achieved_deg",
    )

    assert trace.time_s == (0.0, 0.5, 1.0)
    assert trace.reference == (0.0, 1.0, 1.0)
    assert trace.requested_control == (0.0, 2.0, 1.0)
    assert analyze_controller_trace(trace, settling_absolute_tolerance=0.05).formal_stability_claim is False

    with pytest.raises(ValueError, match="must be a scalar"):
        cadac_controller_trace_from_samples(
            (
                {"time_s": 0.0, "values": {"reference": (1.0, 2.0), "response": 0.0}},
                {"time_s": 1.0, "values": {"reference": (1.0, 2.0), "response": 1.0}},
            ),
            controller_id="source",
            model_id="cadac.test.vehicle",
            reference_channel_id="reference",
            response_channel_id="response",
        )
    ####


def test_local_stability_analysis_handles_continuous_and_discrete_domains() -> None:
    continuous = analyze_closed_loop_state_matrix(
        controller_id="source",
        model_id="cadac.test.vehicle",
        operating_point_id="trim-1",
        state_names=("alpha", "q"),
        closed_loop_state_matrix=((-1.0, 0.0), (0.0, -2.0)),
    )
    discrete = analyze_closed_loop_state_matrix(
        controller_id="candidate",
        model_id="cadac.test.vehicle",
        operating_point_id="trim-1",
        state_names=("alpha", "q"),
        closed_loop_state_matrix=((0.8, 0.0), (0.0, 1.01)),
        domain="discrete",
        sample_time_s=0.02,
    )

    assert continuous.status == "stable"
    assert continuous.spectral_abscissa == pytest.approx(-1.0)
    assert discrete.status == "unstable"
    assert discrete.spectral_radius == pytest.approx(1.01)
    assert not continuous.nonlinear_or_robust_stability_claim
    ####


def test_local_stability_rejects_incomplete_matrix_identity() -> None:
    with pytest.raises(ValueError, match="square in declared state order"):
        analyze_closed_loop_state_matrix(
            controller_id="source",
            model_id="cadac.test.vehicle",
            operating_point_id="trim-1",
            state_names=("alpha", "q"),
            closed_loop_state_matrix=((-1.0,),),
        )
    ####
