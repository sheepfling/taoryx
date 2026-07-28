from __future__ import annotations

import numpy as np
import pytest

from taoryx.runtime.observation_models import ObservationPipeline, parse_observation_config
from taoryx.sensors import ImuIncrement, MeasurementPacket, TruthPoint


def _truth(time_s: float) -> TruthPoint:
    return TruthPoint(
        time_s=time_s,
        position_eci_m=np.zeros(3),
        velocity_eci_mps=np.zeros(3),
        velocity_without_gravity_eci_mps=np.zeros(3),
        orientation_eci_from_body=np.eye(3),
        gravity_eci_mps2=np.zeros(3),
        angular_rate_body_radps=np.zeros(3),
    )


class _ConstantImu:
    provenance = {"provider": "test"}

    def sample(self, truth: TruthPoint) -> MeasurementPacket[ImuIncrement]:
        return MeasurementPacket(
            truth.time_s,
            truth.time_s,
            truth.time_s - 0.5,
            ImuIncrement(
                np.array([1.0, 2.0, 3.0]),
                np.array([0.1, 0.2, 0.3]),
                truth.time_s - 0.5,
                truth.time_s,
            ),
        )


def test_observation_pipeline_applies_typed_stages_in_declared_order() -> None:
    config = parse_observation_config(
        {
            "stages": [
                {"kind": "scale-misalignment", "accelerometer_matrix": [[2, 0, 0], [0, 1, 0], [0, 0, 1]]},
                {"kind": "bias", "accelerometer_bias_mps2": [0.2, 0, 0], "gyroscope_bias_radps": [0, 0.04, 0]},
                {"kind": "quantization", "accelerometer_step_mps": 0.1, "gyroscope_step_rad": 0.02},
            ]
        }
    )
    assert config is not None

    packet = ObservationPipeline(_ConstantImu(), config).sample(_truth(1.0))

    assert packet.payload is not None
    assert packet.payload.delta_v_body_mps == pytest.approx([2.1, 2.0, 3.0])
    assert packet.payload.delta_theta_body_rad == pytest.approx([0.1, 0.22, 0.3])


def test_observation_dropout_is_deterministic_and_keeps_packet_timestamps() -> None:
    config = parse_observation_config({"stages": [{"kind": "dropout", "every_n": 2}]})
    assert config is not None
    pipeline = ObservationPipeline(_ConstantImu(), config)

    first = pipeline.sample(_truth(1.0))
    second = pipeline.sample(_truth(2.0))

    assert not first.valid
    assert first.payload is None
    assert first.sampled_at_s == pytest.approx(1.0)
    assert second.valid
    assert second.payload is not None


def test_observation_noise_checkpoint_replays_the_next_sample() -> None:
    config = parse_observation_config(
        {"stages": [{"kind": "gaussian-noise", "accelerometer_density_mps2_sqrt_hz": 0.1, "seed": 17}]}
    )
    assert config is not None
    pipeline = ObservationPipeline(_ConstantImu(), config)
    pipeline.sample(_truth(1.0))
    checkpoint = pipeline.snapshot()
    original = pipeline.sample(_truth(2.0))

    restored = ObservationPipeline(_ConstantImu(), config)
    restored.restore(checkpoint)
    replay = restored.sample(_truth(2.0))

    assert original.payload is not None
    assert replay.payload is not None
    assert replay.payload.delta_v_body_mps == pytest.approx(original.payload.delta_v_body_mps)
    assert replay.payload.delta_theta_body_rad == pytest.approx(original.payload.delta_theta_body_rad)
