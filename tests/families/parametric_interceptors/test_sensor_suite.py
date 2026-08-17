"""Native sensor-registry integration for both interceptor runtime tiers."""

from __future__ import annotations

from pathlib import Path

import pytest
from taoryx_parametric_interceptors import (
    DEFAULT_SENSOR_SUITE_ID,
    InterceptorSensorSuite,
    ParametricInterceptorMissionCompositionProvider,
    interceptor,
    standard_interceptor_sensor_suite,
)

from taoryx.runtime.sensor_contracts import SensorProviderConfig
from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunRequest,
    MissionCompositionTrajectoryResponse,
)
from taoryx.trajectory.session_contract import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionManager,
    MissionCompositionSessionStepRequest,
)

_DELAYED_SUITE_ID = "test.sensors.delayed-interceptor-suite-v1"


def _delayed_suite() -> InterceptorSensorSuite:
    return InterceptorSensorSuite(
        id=_DELAYED_SUITE_ID,
        version="1.2.3",
        point_mass_provider=SensorProviderConfig(
            kind="translation-acceleration",
            config={"delivery_delay_s": 0.25},
        ),
        pseudo6_provider=SensorProviderConfig(kind="ideal"),
        provenance="focused delayed-delivery test suite",
        claim_boundary="Synthetic integration witness only.",
    )
    ####


def _provider() -> ParametricInterceptorMissionCompositionProvider:
    suite = _delayed_suite()
    return ParametricInterceptorMissionCompositionProvider.from_profiles(
        interceptor("sensor-suite-sam"),
        sensor_suites={suite.id: suite},
        default_sensor_suite_id=suite.id,
    )
    ####


def test_sensor_suite_validates_truth_mode_payload_and_versioned_fingerprint() -> None:
    standard = standard_interceptor_sensor_suite()
    assert standard.id == DEFAULT_SENSOR_SUITE_ID
    assert standard.point_mass_provider.kind == "translation-acceleration"
    assert standard.pseudo6_provider.kind == "ideal"
    assert standard.target_track_provider.kind == "relative-state-track"
    assert standard.target_track_provider.config["target_id"] == "guidance-target"
    assert standard.version == "1.1.0"
    assert len(standard.fingerprint) == 64

    delayed = _delayed_suite()
    changed_version = delayed.model_copy(update={"version": "1.2.4"})
    assert changed_version.fingerprint != delayed.fingerprint

    profile = interceptor("sensor-suite-schema-identity")
    first_provider = ParametricInterceptorMissionCompositionProvider.from_profiles(
        profile,
        sensor_suites={delayed.id: delayed},
        default_sensor_suite_id=delayed.id,
    )
    changed_provider = ParametricInterceptorMissionCompositionProvider.from_profiles(
        profile,
        sensor_suites={changed_version.id: changed_version},
        default_sensor_suite_id=changed_version.id,
    )
    first_schema = first_provider.get_model_schema(profile.model_id)
    changed_schema = changed_provider.get_model_schema(profile.model_id)
    assert first_schema.fingerprint != changed_schema.fingerprint
    assert (
        first_provider.validate_configuration(first_provider.configuration(profile.model_id)).fingerprint
        != changed_provider.validate_configuration(changed_provider.configuration(profile.model_id)).fingerprint
    )

    with pytest.raises(ValueError, match="point_mass_3dof truth mode"):
        InterceptorSensorSuite(
            id="test.invalid-point-suite",
            version="1",
            point_mass_provider=SensorProviderConfig(kind="ideal"),
            pseudo6_provider=SensorProviderConfig(kind="ideal"),
            provenance="test",
            claim_boundary="test only",
        )
    with pytest.raises(ValueError, match="attitude_response_pseudo_6dof truth mode"):
        InterceptorSensorSuite(
            id="test.invalid-pseudo6-suite",
            version="1",
            point_mass_provider=SensorProviderConfig(kind="translation-acceleration"),
            pseudo6_provider=SensorProviderConfig(kind="translation-acceleration"),
            provenance="test",
            claim_boundary="test only",
        )
    with pytest.raises(ValueError, match="must select entity 'guidance-target'"):
        InterceptorSensorSuite(
            id="test.invalid-target-selector-suite",
            version="1",
            target_track_provider=SensorProviderConfig(
                kind="relative-state-track",
                config={"target_id": "another-target"},
            ),
            provenance="test",
            claim_boundary="test only",
        )
    ####


@pytest.mark.parametrize(
    ("fidelity", "prefix", "provider_kind", "expected_latency", "schema_id"),
    (
        (
            "point_mass_3dof",
            "sensor.translation_acceleration",
            "translation-acceleration",
            0.25,
            "taoryx.acceleration.increment/v1",
        ),
        (
            "attitude_response_pseudo_6dof",
            "sensor.imu",
            "ideal",
            0.0,
            "taoryx.imu.increment/v1",
        ),
    ),
)
def test_composition_selects_registered_suite_and_publishes_packet_envelope(
    fidelity: str,
    prefix: str,
    provider_kind: str,
    expected_latency: float,
    schema_id: str,
) -> None:
    provider = _provider()
    model = provider.model("sensor-suite-sam")
    parameters = {item.id: item for item in provider.get_model_schema(model.id).root.children}
    assert parameters["runtime.sensor_suite_id"].choices == (
        DEFAULT_SENSOR_SUITE_ID,
        _DELAYED_SUITE_ID,
    )
    assert parameters["runtime.sensor_suite_id"].default == _DELAYED_SUITE_ID
    properties = {item.id: item for item in model.presentation.properties}
    assert properties["default_sensor_suite_id"].value == _DELAYED_SUITE_ID
    assert "@1.2.3" in str(properties["available_sensor_suites"].value)

    configuration = provider.configuration(
        model.id,
        fidelity=fidelity,
        launch_altitude_m=100.0,
        launch_speed_mps=100.0,
        launch_flight_path_deg=0.0,
        navigation_waypoint_north_command=10_000.0,
        navigation_waypoint_altitude_command=100.0,
        runtime_duration_s=0.4,
        runtime_time_step_s=0.1,
    )
    prepared = provider.validate_configuration(configuration)
    built_in = provider.validate_configuration(
        provider.configuration(
            model.id,
            fidelity=fidelity,
            launch_altitude_m=100.0,
            launch_speed_mps=100.0,
            launch_flight_path_deg=0.0,
            navigation_waypoint_north_command=10_000.0,
            navigation_waypoint_altitude_command=100.0,
            runtime_duration_s=0.4,
            runtime_time_step_s=0.1,
            runtime_sensor_suite_id=DEFAULT_SENSOR_SUITE_ID,
        )
    )
    assert prepared.resolved["runtime.sensor_suite_id"] == _DELAYED_SUITE_ID
    assert prepared.fingerprint != built_in.fingerprint

    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id=f"sensor-suite-{fidelity}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )
    assert isinstance(response, MissionCompositionTrajectoryResponse)
    samples = response.result.objects[0].samples
    assert samples[0].values[f"{prefix}.sampled_at"] == 0.0
    assert samples[0].values[f"{prefix}.schema_id"] == schema_id
    if expected_latency:
        assert samples[0].values[f"{prefix}.valid"] is False
        assert samples[0].values[f"{prefix}.delivery_fresh"] is False
        assert samples[0].values[f"{prefix}.sequence"] == -1
        assert samples[-1].values[f"{prefix}.valid"] is True
        assert samples[-1].values[f"{prefix}.sampled_at"] == pytest.approx(0.1)
        assert samples[-1].values[f"{prefix}.available_at"] == pytest.approx(0.35)
        assert samples[-1].values[f"{prefix}.latency"] == pytest.approx(expected_latency)
        assert samples[-1].values[f"{prefix}.delivery_fresh"] is True
        assert samples[-1].values[f"{prefix}.sequence"] == 1
    else:
        assert samples[1].values[f"{prefix}.sampled_at"] == pytest.approx(0.1)
        assert samples[1].values[f"{prefix}.available_at"] == pytest.approx(0.1)
        assert samples[1].values[f"{prefix}.latency"] == 0.0
        assert samples[1].values[f"{prefix}.delivery_fresh"] is True
        assert samples[1].values[f"{prefix}.sequence"] == 1
    details = response.result.diagnostics[0].details
    assert details["sensor_suite_id"] == _DELAYED_SUITE_ID
    assert details["sensor_suite_version"] == "1.2.3"
    assert details["sensor_provider_kind"] == provider_kind
    assert details["sensor_suite_fingerprint"] == _delayed_suite().fingerprint
    ####


def test_live_session_uses_selected_registered_sensor_and_reports_lowering_identity(
    tmp_path: Path,
) -> None:
    provider = _provider()
    prepared = provider.validate_configuration(
        provider.configuration(
            "sensor-suite-sam",
            startup_authority_profile_id="live_waypoint_guidance",
            launch_altitude_m=100.0,
            launch_speed_mps=100.0,
            launch_flight_path_deg=0.0,
            navigation_waypoint_north_command=10_000.0,
            navigation_waypoint_altitude_command=100.0,
            runtime_duration_s=0.5,
            runtime_time_step_s=0.1,
        )
    )
    sessions = MissionCompositionSessionManager(provider)
    opened = sessions.open(
        MissionCompositionOpenSessionRequest(
            session_id="sensor-suite-live",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            integration_step_s=0.1,
            seed=17,
        )
    )
    assert opened.initial_observation.values["sensor.translation_acceleration.valid"] is False
    assert opened.initial_observation.values["sensor.translation_acceleration.delivery_fresh"] is False
    stepped = sessions.step(
        MissionCompositionSessionStepRequest(
            session_id=opened.session_id,
            action={"navigation.waypoint.east.command": 100.0},
            duration_s=0.4,
            expected_sequence=0,
        )
    )
    assert stepped.observation.values["sensor.translation_acceleration.valid"] is True
    assert stepped.observation.values["sensor.translation_acceleration.latency"] == pytest.approx(0.25)
    assert stepped.observation.values["sensor.translation_acceleration.sampled_at"] == pytest.approx(0.1)
    assert stepped.observation.values["sensor.translation_acceleration.available_at"] == pytest.approx(0.35)
    assert stepped.observation.values["sensor.translation_acceleration.delivery_fresh"] is True
    assert stepped.observation.values["sensor.translation_acceleration.sequence"] == 1
    assert stepped.lowering_evidence["sensor_suite_id"] == _DELAYED_SUITE_ID
    assert stepped.lowering_evidence["sensor_suite_version"] == "1.2.3"
    assert stepped.lowering_evidence["sensor_provider_kind"] == "translation-acceleration"
    assert stepped.lowering_evidence["sensor_suite_fingerprint"] == _delayed_suite().fingerprint

    episode = provider.open_session_episode(prepared, seed=17, integration_step_s=0.1)
    pending = episode.step({}, 0.1)
    assert pending.observation.values["sensor.translation_acceleration.valid"] is False
    checkpoint = episode.save_checkpoint(tmp_path / "delayed-sensor-session.json")
    advanced = episode.step({}, 0.3)
    episode.load_checkpoint(checkpoint)
    replayed = episode.step({}, 0.3)
    assert replayed.observation.values == advanced.observation.values
    assert replayed.observation.values["sensor.translation_acceleration.sequence"] == 1
    assert replayed.observation.values["sensor.translation_acceleration.available_at"] == pytest.approx(0.35)
    ####
