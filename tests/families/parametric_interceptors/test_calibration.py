"""Focused scenario-qualified calibration workflow tests."""

from __future__ import annotations

from pathlib import Path

import pytest
from taoryx_parametric_interceptors import (
    AssumptionCase,
    CalibrationTarget,
    InterceptorCalibrationScenario,
    InterceptorSensorSuite,
    ParametricInterceptorMissionCompositionProvider,
    PointMassMission,
    ValueOrigin,
    aim9x_block2_profile,
    calibration_scenario_from_reported_profile,
    compare_assumption_cases,
    evaluate_interceptor_calibration,
    interceptor,
    load_interceptor_calibration_scenario,
    reported,
    resolve_interceptor,
)

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.runtime.environment_runtime import EnvironmentSample, StaticEnvironmentProvider
from taoryx.runtime.sensor_contracts import SensorProviderConfig

_EXAMPLES = Path("examples/parametric_interceptors")


def _mission() -> PointMassMission:
    return PointMassMission(
        launch_altitude_m=50.0,
        launch_speed_mps=40.0,
        launch_flight_path_deg=35.0,
        waypoint_north_m=5_000.0,
        waypoint_altitude_m=1_500.0,
        duration_s=1.0,
        time_step_s=0.1,
    )
    ####


def test_reported_target_helper_requires_available_evidence_and_scenario_basis() -> None:
    classified = resolve_interceptor(aim9x_block2_profile())
    with pytest.raises(ValueError, match="none of the requested reported calibration parameters are available"):
        calibration_scenario_from_reported_profile(
            classified,
            "aim9x-classified-screen",
            scenario_basis="No public maximum-performance definition is available.",
        )

    resolved = resolve_interceptor(
        interceptor(
            "reported-speed-witness",
            reported_max_speed_mps=reported(
                900.0,
                unit="m/s",
                source_record_id="source:scenario-qualified-speed",
            ),
        )
    )
    scenario = calibration_scenario_from_reported_profile(
        resolved,
        "reported-speed-screen",
        mission=_mission(),
        scenario_basis="Public launch-condition speed screen normalized to this prototype mission.",
        relative_tolerance=0.1,
    )

    assert len(scenario.fingerprint) == 64
    assert scenario.unavailable_target_parameter_ids == (
        "reported_max_altitude_m",
        "reported_max_range_m",
    )
    target = scenario.targets[0]
    assert target.metric_id == "peak_speed_mps"
    assert target.target == 900.0
    assert target.tolerance == 90.0
    assert target.source_origin is ValueOrigin.REPORTED
    assert target.source_record_ids == ("source:scenario-qualified-speed",)
    result = evaluate_interceptor_calibration(resolved, scenario)
    assert "source_record_ids=source:scenario-qualified-speed" in result.evaluation.metrics[0].source
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(resolved)
    provider_scenario = provider.reported_calibration_scenario(
        "reported-speed-witness",
        "provider-speed-screen",
        scenario_basis="Provider convenience-path version of the same prototype speed screen.",
        mission=_mission(),
        include=("reported_max_speed_mps",),
    )
    assert provider.evaluate_calibration("reported-speed-witness", provider_scenario).model_id == "reported-speed-witness"
    ####


@pytest.mark.parametrize("fidelity", ("point_mass_3dof", "attitude_response_pseudo_6dof"))
def test_calibration_evaluation_uses_the_same_observable_contract_at_both_tiers(fidelity: str) -> None:
    resolved = resolve_interceptor(interceptor("calibration-tier-witness"))
    scenario = InterceptorCalibrationScenario(
        scenario_id=f"calibration-{fidelity.replace('_', '-')}",
        mission=_mission(),
        fidelity=fidelity,  # type: ignore[arg-type]
        targets=(
            CalibrationTarget(
                metric_id="peak_speed_mps",
                target=100.0,
                tolerance=100.0,
                unit="m/s",
                source_basis="Synthetic interface witness target.",
            ),
            CalibrationTarget(
                metric_id="waypoint_captured",
                target=False,
                unit="1",
                source_basis="One-second screen is intentionally shorter than waypoint capture.",
            ),
        ),
        scenario_basis="Same launch, guidance, atmosphere, and gravity contract at both tiers.",
    )
    result = evaluate_interceptor_calibration(resolved, scenario)

    assert result.fidelity == fidelity
    assert result.scenario == scenario
    assert result.scenario_fingerprint == scenario.fingerprint
    assert result.evaluation.scenario_contract_sha256 == scenario.fingerprint
    assert result.evaluation.qualification == "unqualified"
    assert set(result.observables) == {
        "peak_speed_mps",
        "maximum_altitude_m",
        "horizontal_distance_m",
        "elapsed_time_s",
        "terminal_waypoint_range_m",
        "propellant_remaining_kg",
        "waypoint_captured",
    }
    assert {item.id for item in result.evaluation.metrics} == {
        "calibration.peak_speed_mps",
        "calibration.waypoint_captured",
    }
    assert result.evaluation.metrics[1].status == "pass"
    assert result.evaluation.requested_controls[0].frame == "local_ned"
    assert result.evaluation.resources[0].id == "resource.propellant.remaining"
    assert result.evaluation.events[0].value is False
    assert "surrogate" in result.claim_boundary.lower()
    assert '"scenario_basis"' in result.model_dump_json()
    ####


def test_provider_binds_calibration_to_registered_environment_and_gravity_ids() -> None:
    environment_model_id = "test.calibration-environment-v1"
    gravity_model_id = "test.calibration-gravity-v1"
    resolved = resolve_interceptor(
        interceptor(
            "registered-calibration-dependencies",
            reported_max_speed_mps=reported(
                400.0,
                unit="m/s",
                source_record_id="source:registered-calibration-speed",
            ),
        )
    )
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(
        resolved,
        environment_models={
            environment_model_id: StaticEnvironmentProvider(
                EnvironmentSample(
                    density=0.8,
                    pressure=80_000.0,
                    temperature=240.0,
                    speed_of_sound=300.0,
                    wind=FrameVector3(Vector3(0.0, 0.0, 5.0), Frame.ECFC),
                )
            )
        },
        gravity_models={gravity_model_id: lambda _: 4.0},
        default_environment_model_id=environment_model_id,
        default_gravity_model_id=gravity_model_id,
    )
    scenario = provider.reported_calibration_scenario(
        resolved.model_id,
        "registered-dependency-screen",
        scenario_basis="Synthetic registered runtime-dependency witness.",
        mission=_mission(),
        include=("reported_max_speed_mps",),
    )

    assert scenario.environment_model_id == environment_model_id
    assert scenario.gravity_model_id == gravity_model_id
    result = provider.evaluate_calibration(resolved.model_id, scenario)
    dependency_gate = next(item for item in result.evaluation.gates if item.id == "runtime-dependency-identity")
    assert environment_model_id in dependency_gate.message
    assert gravity_model_id in dependency_gate.message

    with pytest.raises(KeyError, match="unregistered calibration environment"):
        provider.evaluate_calibration(
            resolved.model_id,
            scenario.model_copy(update={"environment_model_id": "test.not-registered"}),
        )
    ####


def test_calibration_binds_exact_sensor_suite_and_rejects_identity_drift() -> None:
    sensor_suite = InterceptorSensorSuite(
        id="test.calibration-sensors-v1",
        version="2.3.4",
        target_track_provider=SensorProviderConfig(
            kind="relative-state-track",
            config={"target_id": "guidance-target", "azimuth_bias_rad": 0.05},
        ),
        provenance="focused calibration sensor witness",
        claim_boundary="Synthetic calibration dependency witness.",
    )
    resolved = resolve_interceptor(
        interceptor(
            "registered-calibration-sensors",
            reported_max_speed_mps=reported(
                400.0,
                unit="m/s",
                source_record_id="source:registered-calibration-sensor-speed",
            ),
        )
    )
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(
        resolved,
        sensor_suites={sensor_suite.id: sensor_suite},
        default_sensor_suite_id=sensor_suite.id,
    )
    scenario = provider.reported_calibration_scenario(
        resolved.model_id,
        "registered-sensor-screen",
        scenario_basis="Synthetic sensor-suite identity witness.",
        mission=_mission(),
        include=("reported_max_speed_mps",),
    )

    assert scenario.sensor_suite_id == sensor_suite.id
    assert scenario.sensor_suite_version == sensor_suite.version
    assert scenario.sensor_suite_fingerprint == sensor_suite.fingerprint
    result = provider.evaluate_calibration(resolved.model_id, scenario)
    assert result.sensor_suite_id == sensor_suite.id
    assert result.sensor_suite_version == sensor_suite.version
    assert result.sensor_suite_fingerprint == sensor_suite.fingerprint
    assert result.navigation_sensor_provider_kind == "translation-acceleration"
    assert result.target_track_sensor_provider_kind == "relative-state-track"
    dependency_gate = next(item for item in result.evaluation.gates if item.id == "runtime-dependency-identity")
    assert sensor_suite.id in dependency_gate.message
    assert sensor_suite.fingerprint in dependency_gate.message

    with pytest.raises(ValueError, match="does not match the scenario-bound identity"):
        provider.evaluate_calibration(
            resolved.model_id,
            scenario.model_copy(update={"sensor_suite_version": "2.3.5"}),
        )
    with pytest.raises(KeyError, match="unregistered calibration sensor suite"):
        provider.evaluate_calibration(
            resolved.model_id,
            scenario.model_copy(update={"sensor_suite_id": "test.not-registered"}),
        )
    ####


def test_assumption_case_comparison_ranks_without_mutating_or_claiming_calibration() -> None:
    profile = interceptor("assumption-comparison-witness")
    nominal = resolve_interceptor(profile, assumption_case=AssumptionCase.NOMINAL)
    seed_scenario = InterceptorCalibrationScenario(
        scenario_id="nominal-observable-seed",
        mission=_mission(),
        targets=(
            CalibrationTarget(
                metric_id="peak_speed_mps",
                target=100.0,
                tolerance=100.0,
                unit="m/s",
                source_basis="Seed screen used only to extract a nominal observable.",
            ),
        ),
        scenario_basis="Fixed one-second assumption-case sensitivity screen.",
    )
    nominal_speed = float(evaluate_interceptor_calibration(nominal, seed_scenario).observables["peak_speed_mps"])
    comparison_scenario = seed_scenario.model_copy(
        update={
            "scenario_id": "assumption-case-comparison",
            "targets": (
                CalibrationTarget(
                    metric_id="peak_speed_mps",
                    target=nominal_speed,
                    tolerance=1.0,
                    unit="m/s",
                    source_basis="Nominal-case regression witness, not external validation evidence.",
                ),
            ),
        }
    )
    comparison = compare_assumption_cases(profile, comparison_scenario)

    assert comparison.contract == "taoryx.parametric-interceptors.assumption-case-comparison/v1"
    assert comparison.schema_version == 1
    assert comparison.source_profile_fingerprint == profile.fingerprint
    assert comparison.best_case is AssumptionCase.NOMINAL
    assert comparison.ranking[0].aggregate_normalized_error == pytest.approx(0.0)
    assert {item.assumption_case for item in comparison.results} == set(AssumptionCase)
    assert all(item.scenario_fingerprint == comparison_scenario.fingerprint for item in comparison.results)
    assert len({item.profile_fingerprint for item in comparison.results}) == 3
    assert profile.default_assumption_case is AssumptionCase.NOMINAL
    assert "not parameter estimation" in comparison.claim_boundary
    ####


def test_calibration_target_rejects_unit_and_boolean_semantic_mismatches() -> None:
    with pytest.raises(ValueError, match="requires unit 'm/s'"):
        CalibrationTarget(
            metric_id="peak_speed_mps",
            target=500.0,
            tolerance=10.0,
            unit="kt",
            source_basis="Invalid unit witness.",
        )
    with pytest.raises(ValueError, match="boolean target without tolerance"):
        CalibrationTarget(
            metric_id="waypoint_captured",
            target=True,
            tolerance=1.0,
            unit="1",
            source_basis="Invalid boolean tolerance witness.",
        )
    ####


def test_copy_ready_yaml_scenario_loads_and_runs() -> None:
    scenario = load_interceptor_calibration_scenario(_EXAMPLES / "generic_medium_sam_calibration.yaml")
    profile = resolve_interceptor(interceptor("yaml-calibration-witness"))
    result = evaluate_interceptor_calibration(profile, scenario)

    assert scenario.scenario_id == "generic-medium-sam-speed-screen"
    assert scenario.targets[0].source_origin is ValueOrigin.SIMULATION_ASSUMPTION
    assert result.scenario_fingerprint == scenario.fingerprint
    assert result.evaluation.metrics[0].severity == "advisory"
    ####
