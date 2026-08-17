"""Focused bounded multi-scenario interceptor fitting tests."""

from __future__ import annotations

from pathlib import Path

import pytest
from taoryx_parametric_interceptors import (
    CalibrationTarget,
    InterceptorCalibrationScenario,
    InterceptorFitCampaign,
    InterceptorFitVariable,
    InterceptorSensorSuite,
    PointMassMission,
    ResolvedInterceptorProfile,
    ValueOrigin,
    apply_interceptor_fit_receipt,
    calibrated,
    evaluate_interceptor_calibration,
    fit_interceptor_profile,
    interceptor,
    load_interceptor_fit_campaign,
    load_interceptor_fit_receipt,
    observed,
    resolve_interceptor,
    resolve_interceptor_fit_receipt,
    write_interceptor_fit_receipt,
)


def _mission(*, duration_s: float, altitude_m: float) -> PointMassMission:
    return PointMassMission(
        launch_altitude_m=altitude_m,
        launch_speed_mps=50.0,
        launch_flight_path_deg=30.0,
        waypoint_north_m=8_000.0,
        waypoint_altitude_m=2_000.0,
        duration_s=duration_s,
        time_step_s=0.1,
    )
    ####


def _speed_scenario(
    profile_id: str,
    resolved_truth: ResolvedInterceptorProfile,
    *,
    duration_s: float,
    altitude_m: float,
) -> InterceptorCalibrationScenario:
    mission = _mission(duration_s=duration_s, altitude_m=altitude_m)
    seed = InterceptorCalibrationScenario(
        scenario_id=f"{profile_id}-seed-{duration_s:g}-{altitude_m:g}",
        mission=mission,
        targets=(
            CalibrationTarget(
                metric_id="peak_speed_mps",
                target=100.0,
                tolerance=100.0,
                unit="m/s",
                source_basis="Synthetic truth extraction seed.",
            ),
        ),
        scenario_basis="Synthetic deterministic recovery witness.",
    )
    truth = evaluate_interceptor_calibration(resolved_truth, seed)
    target = float(truth.observables["peak_speed_mps"])
    return InterceptorCalibrationScenario(
        scenario_id=f"{profile_id}-{duration_s:g}-{altitude_m:g}",
        mission=mission,
        targets=(
            CalibrationTarget(
                metric_id="peak_speed_mps",
                target=target,
                tolerance=2.0,
                unit="m/s",
                severity="required",
                source_basis="Synthetic calibrated-scale recovery target.",
            ),
        ),
        scenario_basis="Synthetic deterministic recovery witness.",
    )
    ####


def test_calibrated_scale_provenance_reaches_runtime_parameters() -> None:
    profile = interceptor(
        "calibrated-scale-witness",
        thrust_scale=calibrated(
            1.2,
            unit="1",
            method="synthetic fit receipt witness",
        ),
    )
    resolved = resolve_interceptor(profile)

    assert profile.thrust_scale is not None
    assert profile.thrust_scale.origin is ValueOrigin.CALIBRATED
    assert resolved.parameters["thrust_scale"].origin is ValueOrigin.CALIBRATED
    assert resolved.parameters["thrust_n"].origin is ValueOrigin.CALIBRATED
    assert "thrust_scale" in resolved.parameters["thrust_n"].depends_on
    assert resolved.number("thrust_n") > resolve_interceptor(interceptor("uncalibrated-scale-witness")).number("thrust_n")
    ####


def test_multi_scenario_fit_recovers_scale_and_applies_a_replayable_receipt(
    tmp_path: Path,
) -> None:
    base = interceptor(
        "fit-recovery-witness",
        launch_mass_kg=observed(
            300.0,
            unit="kg",
            source_record_id="source:fit-recovery:mass",
        ),
        thrust_profile_class="neutral",
    )
    truth = base.model_copy(
        update={
            "thrust_scale": calibrated(
                1.18,
                unit="1",
                method="hidden synthetic truth for recovery test",
            )
        }
    )
    resolved_truth = resolve_interceptor(truth)
    scenarios = (
        _speed_scenario(
            "fit-recovery",
            resolved_truth,
            duration_s=0.8,
            altitude_m=50.0,
        ),
        _speed_scenario(
            "fit-recovery",
            resolved_truth,
            duration_s=1.2,
            altitude_m=1_000.0,
        ),
    )
    campaign = InterceptorFitCampaign(
        campaign_id="recover-thrust-scale-v1",
        scenarios=scenarios,
        variables=(
            InterceptorFitVariable(
                parameter_id="thrust_scale",
                lower_bound=0.7,
                upper_bound=1.4,
                initial_value=0.9,
            ),
        ),
        optimizer_backend="builtin-rqp",
        max_iterations=80,
        tolerance=1.0e-5,
        derivative_step=2.0e-3,
    )
    base_fingerprint = base.fingerprint
    receipt = fit_interceptor_profile(base, campaign)

    assert base.fingerprint == base_fingerprint
    assert len(receipt.baseline_results) == 2
    assert len(receipt.fitted_results) == 2
    assert receipt.accepted
    assert receipt.objective_final < receipt.objective_initial * 1.0e-3
    fitted_parameter = receipt.fitted_parameters[0]
    assert fitted_parameter.parameter_id == "thrust_scale"
    assert fitted_parameter.fitted_value == pytest.approx(1.18, abs=0.01)
    fitted = apply_interceptor_fit_receipt(base, receipt)
    assert fitted.launch_mass_kg == base.launch_mass_kg
    assert fitted.launch_mass_kg is not None
    assert fitted.launch_mass_kg.origin is ValueOrigin.OBSERVED
    assert fitted.thrust_scale is not None
    assert fitted.thrust_scale.origin is ValueOrigin.CALIBRATED
    resolved = resolve_interceptor_fit_receipt(base, receipt)
    assert resolved.fingerprint == receipt.fitted_resolved_profile_fingerprint
    assert resolved.parameters["thrust_n"].origin is ValueOrigin.CALIBRATED

    receipt_path = write_interceptor_fit_receipt(receipt, tmp_path / "fit-receipt.json")
    loaded = load_interceptor_fit_receipt(receipt_path)
    assert loaded.fingerprint == receipt.fingerprint
    assert loaded.model_dump(mode="json") == receipt.model_dump(mode="json")
    ####


def test_fit_rejects_protected_evidence_stale_receipts_and_implicit_out_of_bounds_initials() -> None:
    scenario = InterceptorCalibrationScenario(
        scenario_id="fit-rejection-screen",
        mission=_mission(duration_s=0.2, altitude_m=10.0),
        targets=(
            CalibrationTarget(
                metric_id="peak_speed_mps",
                target=60.0,
                tolerance=10.0,
                unit="m/s",
                source_basis="Synthetic rejection witness.",
            ),
        ),
        scenario_basis="Synthetic rejection witness.",
    )
    protected = interceptor(
        "protected-fit-scale",
        thrust_scale=observed(
            1.0,
            unit="1",
            source_record_id="source:protected-scale",
        ),
    )
    campaign = InterceptorFitCampaign(
        campaign_id="protected-fit-v1",
        scenarios=(scenario,),
        variables=(
            InterceptorFitVariable(
                parameter_id="thrust_scale",
                lower_bound=0.8,
                upper_bound=1.2,
            ),
        ),
    )
    with pytest.raises(ValueError, match="protected origin 'observed'"):
        fit_interceptor_profile(protected, campaign)

    base = interceptor("stale-fit-receipt")
    receipt = fit_interceptor_profile(base, campaign.model_copy(update={"campaign_id": "stale-fit-v1"}))
    changed = base.model_copy(update={"model_notes": "changed after fit"})
    with pytest.raises(ValueError, match="base profile fingerprint"):
        apply_interceptor_fit_receipt(changed, receipt, allow_unaccepted=True)

    invalid_initial_campaign = campaign.model_copy(
        update={
            "campaign_id": "invalid-initial-v1",
            "variables": (
                InterceptorFitVariable(
                    parameter_id="thrust_scale",
                    lower_bound=1.1,
                    upper_bound=1.3,
                ),
            ),
        }
    )
    with pytest.raises(ValueError, match="defaults to 1.0"):
        fit_interceptor_profile(base, invalid_initial_campaign)

    unaccepted_campaign = campaign.model_copy(
        update={
            "campaign_id": "unaccepted-fit-v1",
            "max_iterations": 1,
            "acceptance_normalized_error": 0.0,
        }
    )
    unaccepted = fit_interceptor_profile(base, unaccepted_campaign)
    assert unaccepted.accepted is False
    with pytest.raises(ValueError, match="did not meet its acceptance threshold"):
        apply_interceptor_fit_receipt(base, unaccepted)
    assert (
        apply_interceptor_fit_receipt(
            base,
            unaccepted,
            allow_unaccepted=True,
        ).thrust_scale
        is not None
    )
    ####


def test_fit_campaign_requires_and_replays_named_sensor_suite() -> None:
    sensor_suite = InterceptorSensorSuite(
        id="test.fit-sensors-v1",
        version="1.0.0",
        provenance="focused fit sensor witness",
        claim_boundary="Synthetic fit dependency witness.",
    )
    scenario = InterceptorCalibrationScenario(
        scenario_id="fit-sensor-dependency-screen",
        mission=_mission(duration_s=0.2, altitude_m=10.0),
        targets=(
            CalibrationTarget(
                metric_id="peak_speed_mps",
                target=60.0,
                tolerance=10.0,
                unit="m/s",
                source_basis="Synthetic fit dependency witness.",
            ),
        ),
        sensor_suite_id=sensor_suite.id,
        sensor_suite_version=sensor_suite.version,
        sensor_suite_fingerprint=sensor_suite.fingerprint,
        scenario_basis="Synthetic fit dependency witness.",
    )
    campaign = InterceptorFitCampaign(
        campaign_id="sensor-bound-fit-v1",
        scenarios=(scenario,),
        variables=(
            InterceptorFitVariable(
                parameter_id="thrust_scale",
                lower_bound=0.8,
                upper_bound=1.2,
            ),
        ),
        max_iterations=1,
    )
    profile = interceptor("sensor-bound-fit")

    with pytest.raises(KeyError, match="requires registered sensor suite"):
        fit_interceptor_profile(profile, campaign)
    receipt = fit_interceptor_profile(
        profile,
        campaign,
        sensor_suites={sensor_suite.id: sensor_suite},
    )
    assert receipt.baseline_results[0].sensor_suite_fingerprint == sensor_suite.fingerprint
    assert receipt.fitted_results[0].sensor_suite_fingerprint == sensor_suite.fingerprint
    ####


def test_copy_ready_fit_campaign_loads() -> None:
    campaign = load_interceptor_fit_campaign(Path("examples/parametric_interceptors/generic_medium_sam_fit_campaign.yaml"))

    assert campaign.campaign_id == "generic-medium-sam-fit-v1"
    assert len(campaign.scenarios) == 2
    assert {item.parameter_id for item in campaign.variables} == {
        "thrust_scale",
        "drag_scale",
    }
    assert len(campaign.fingerprint) == 64
    ####
