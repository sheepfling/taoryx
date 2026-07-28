from __future__ import annotations

from importlib.metadata import version as package_version
from pathlib import Path

import numpy as np
import pytest

from taoryx.navigation import AttitudeNavigationState, AttitudeOnlyNavigator, DeadReckoningNavigator, MultiplicativeEkf, NavigationState
from taoryx.sensors import GyroIncrement, IdealGyroscopeAdapter, ImuErrorModelAdapter, TruthPoint

ROOT = Path(__file__).resolve().parents[2]


def _truth(time_s: float, velocity_without_gravity_x_mps: float) -> TruthPoint:
    return TruthPoint(
        time_s=time_s,
        position_eci_m=np.array([0.0, 0.0, 0.0]),
        velocity_eci_mps=np.array([velocity_without_gravity_x_mps, 0.0, 0.0]),
        velocity_without_gravity_eci_mps=np.array([velocity_without_gravity_x_mps, 0.0, 0.0]),
        orientation_eci_from_body=np.eye(3),
        gravity_eci_mps2=np.zeros(3),
        angular_rate_body_radps=np.zeros(3),
    )


def _adapter() -> ImuErrorModelAdapter:
    pytest.importorskip("imu_error_model")
    return ImuErrorModelAdapter.from_config(seed=4)


def test_external_imu_adapter_marks_initial_baseline_invalid() -> None:
    adapter = _adapter()
    first = adapter.sample(_truth(0.0, 0.0))
    second = adapter.sample(_truth(0.01, 0.1))

    assert not first.valid
    assert first.payload is None
    assert second.valid
    assert second.payload is not None
    assert second.payload.delta_v_body_mps == pytest.approx([0.1, 0.0, 0.0])
    assert second.payload.dt_s == pytest.approx(0.01)


def test_dead_reckoning_consumes_adapter_packet() -> None:
    adapter = _adapter()
    adapter.sample(_truth(0.0, 0.0))
    packet = adapter.sample(_truth(0.01, 0.1))
    initial = NavigationState(0.0, np.zeros(3), np.zeros(3), np.eye(3))
    navigator = DeadReckoningNavigator(initial, np.zeros(3))

    state = navigator.propagate(packet)

    assert state.time_s == pytest.approx(0.01)
    assert state.velocity_eci_mps == pytest.approx([0.1, 0.0, 0.0])
    assert state.position_eci_m == pytest.approx([0.0005, 0.0, 0.0])


def test_gyro_only_navigation_integrates_rotation_without_translation() -> None:
    rate = 0.4
    adapter = IdealGyroscopeAdapter()

    def truth(time_s: float) -> TruthPoint:
        return TruthPoint(
            time_s=time_s,
            position_eci_m=np.zeros(3),
            velocity_eci_mps=np.zeros(3),
            velocity_without_gravity_eci_mps=np.zeros(3),
            orientation_eci_from_body=np.eye(3),
            gravity_eci_mps2=np.zeros(3),
            angular_rate_body_radps=np.array([0.0, 0.0, rate]),
        )

    assert not adapter.sample(truth(0.0)).valid
    packet = adapter.sample(truth(0.5))
    assert packet.payload is not None
    assert packet.payload.delta_theta_body_rad == pytest.approx([0.0, 0.0, 0.2])

    navigator = AttitudeOnlyNavigator(AttitudeNavigationState(0.0, np.eye(3)))
    state = navigator.propagate(packet)
    assert state.time_s == pytest.approx(0.5)
    assert state.orientation_eci_from_body @ state.orientation_eci_from_body.T == pytest.approx(np.eye(3), abs=1.0e-12)


def test_gyro_increment_rejects_nonpositive_interval() -> None:
    with pytest.raises(ValueError, match="positive finite interval"):
        GyroIncrement(np.zeros(3), 1.0, 1.0)


def test_mekf_position_update_reduces_position_uncertainty() -> None:
    initial = NavigationState(0.0, np.zeros(3), np.zeros(3), np.eye(3))
    covariance = np.eye(15)
    filter_ = MultiplicativeEkf(initial, covariance, np.zeros(3))

    filter_.update_position(np.array([2.0, 0.0, 0.0]), np.eye(3) * 0.01)

    assert filter_.state.position_eci_m[0] == pytest.approx(1.980198, rel=1.0e-5)
    assert filter_.covariance[0, 0] < covariance[0, 0]


def test_adapter_rejects_nonchronological_truth() -> None:
    adapter = _adapter()
    adapter.sample(_truth(1.0, 0.0))
    with pytest.raises(ValueError, match="strictly chronological"):
        adapter.sample(_truth(1.0, 0.0))


def test_adapter_captures_earth_rotation_in_eci_attitude() -> None:
    adapter = _adapter()
    earth_rate_radps = 7.2921151467e-5

    def rotation_eci_from_body(time_s: float) -> np.ndarray:
        angle = earth_rate_radps * time_s
        return np.array(
            [
                [np.cos(angle), -np.sin(angle), 0.0],
                [np.sin(angle), np.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )

    def truth(time_s: float) -> TruthPoint:
        return TruthPoint(
            time_s=time_s,
            position_eci_m=np.zeros(3),
            velocity_eci_mps=np.zeros(3),
            velocity_without_gravity_eci_mps=np.zeros(3),
            orientation_eci_from_body=rotation_eci_from_body(time_s),
            gravity_eci_mps2=np.zeros(3),
            angular_rate_body_radps=np.array([0.0, 0.0, earth_rate_radps]),
        )

    adapter.sample(truth(0.0))
    packet = adapter.sample(truth(1.0))

    assert packet.payload is not None
    assert packet.payload.delta_theta_body_rad == pytest.approx([0.0, 0.0, earth_rate_radps], abs=1.0e-10)


def test_adapter_converts_accepted_acceleration_minus_gravity_for_hover() -> None:
    adapter = _adapter()

    def truth(time_s: float) -> TruthPoint:
        return TruthPoint(
            time_s=time_s,
            position_eci_m=np.zeros(3),
            velocity_eci_mps=np.zeros(3),
            velocity_without_gravity_eci_mps=np.zeros(3),
            orientation_eci_from_body=np.eye(3),
            gravity_eci_mps2=np.array([0.0, 0.0, -9.81]),
            angular_rate_body_radps=np.zeros(3),
            acceleration_eci_mps2=np.zeros(3),
        )

    adapter.sample(truth(0.0))
    packet = adapter.sample(truth(1.0))

    assert packet.payload is not None
    assert packet.payload.delta_v_body_mps == pytest.approx([0.0, 0.0, 9.81])


def test_adapter_removes_declared_profile_output_scales() -> None:
    imu_error_model = pytest.importorskip("imu_error_model")
    config = imu_error_model.ImuConfig(
        accelerometer=imu_error_model.AxisConfig(output_scale=10.0),
        gyroscope=imu_error_model.AxisConfig(output_scale=20.0),
    )
    adapter = ImuErrorModelAdapter.from_config(config, seed=5)
    adapter.sample(_truth(0.0, 0.0))
    packet = adapter.sample(_truth(0.01, 0.1))

    assert packet.payload is not None
    assert packet.payload.delta_v_body_mps == pytest.approx([0.1, 0.0, 0.0])


def test_adapter_loads_profile_document_and_preserves_provenance() -> None:
    pytest.importorskip("imu_error_model")

    adapter = ImuErrorModelAdapter.from_profile(
        ROOT / "tests" / "fixtures" / "imu_profiles" / "taoryx_demo.yaml",
        seed=6,
    )

    assert adapter.provenance["model_name"] == "TaoryxDemoIMU"
    assert adapter.provenance["sample_period_s"] == pytest.approx(0.1)
    assert adapter.provenance["accelerometer_output_scale"] == pytest.approx(10.0)
    assert adapter.provenance["gyroscope_output_scale"] == pytest.approx(20.0)


def test_packaged_hardware_profile_and_checkpoint_replay() -> None:
    pytest.importorskip("imu_error_model")
    assert package_version("imu-error-model") == "0.1.3"

    adapter = ImuErrorModelAdapter.from_example_profile("hg1700ag58.yaml", seed=41)
    adapter.sample(_truth(0.0, 0.0))
    adapter.sample(_truth(0.01, 1.0))
    checkpoint = adapter.snapshot()
    original = adapter.sample(_truth(0.02, 2.0))

    restored = ImuErrorModelAdapter.from_example_profile("hg1700ag58.yaml", seed=999)
    restored.restore(checkpoint)
    replay = restored.sample(_truth(0.02, 2.0))

    assert adapter.provenance["model_name"] == "HG1700AG58"
    assert adapter.provenance["sample_period_s"] == pytest.approx(0.01)
    assert original.payload is not None
    assert replay.payload is not None
    assert replay.payload.delta_v_body_mps == pytest.approx(original.payload.delta_v_body_mps)
    assert replay.payload.delta_theta_body_rad == pytest.approx(original.payload.delta_theta_body_rad)
