"""Regression coverage for declared-sensor batch artifact projection."""

from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.committed_boundary_sensor import ChannelMeasurementError, CommittedBoundarySensor
from taoryx.composition_sensor_trace import BatchTruthSample, build_declared_sensor_trace
from taoryx.vehicle_composition import (
    VehicleCompositionRequest,
    compile_vehicle_composition,
    load_vehicle_composition_request,
    resolve_vehicle_composition_interface_contract,
)

ROOT = Path(__file__).resolve().parents[2]


def _composition(name: str):
    return compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / name))
    ####


def _x8_sample(time_s: float, *, altitude_ft: float) -> BatchTruthSample:
    return BatchTruthSample(
        time_s=time_s,
        raw_values={
            "1": {
                "alt": altitude_ft,
                "vel": 58.0 + time_s,
                "gama": 0.0,
                "psi": 90.0,
                "mass": 12.5,
            }
        },
    )
    ####


def test_declared_sensor_batch_trace_holds_only_released_committed_truth() -> None:
    composition = _composition("x8_racetrack_sensor_episode_3dof_compose.yaml")

    trace = build_declared_sensor_trace(
        composition,
        (
            _x8_sample(0.0, altitude_ft=580.0),
            _x8_sample(0.05, altitude_ft=600.0),
            _x8_sample(0.10, altitude_ft=620.0),
            _x8_sample(0.15, altitude_ft=640.0),
        ),
    )

    assert trace is not None
    assert trace["interpolation"] == "forbidden"
    samples = trace["samples"]
    assert isinstance(samples, list)
    assert samples[0]["source_time_s"] is None
    assert samples[0]["valid"]["position.altitude"] is False
    assert samples[1]["source_time_s"] == pytest.approx(0.0)
    assert samples[1]["values"]["position.altitude"] == pytest.approx(580.0 * 0.3048)
    assert samples[2]["source_time_s"] == pytest.approx(0.0)
    assert samples[3]["source_time_s"] == pytest.approx(0.1)
    assert samples[3]["values"]["position.altitude"] == pytest.approx(620.0 * 0.3048)
    ####


def test_declared_sensor_batch_trace_rejects_a_missing_committed_boundary() -> None:
    composition = _composition("x8_racetrack_sensor_episode_3dof_compose.yaml")

    with pytest.raises(ValueError, match="required truth boundary"):
        build_declared_sensor_trace(
            composition,
            (
                _x8_sample(0.0, altitude_ft=580.0),
                _x8_sample(0.1, altitude_ft=620.0),
            ),
        )
    ####


def test_truth_only_composition_does_not_emit_a_sensor_trace() -> None:
    composition = _composition("x8_racetrack_capability_3dof_compose.yaml")

    assert build_declared_sensor_trace(composition, (_x8_sample(0.0, altitude_ft=580.0),)) is None
    ####


def test_boundary_sensor_measurement_transform_is_seeded_and_checkpointable() -> None:
    errors = {
        "altitude": ChannelMeasurementError(bias=0.2, gaussian_stddev=0.5, quantization_step=0.05),
    }
    first = CommittedBoundarySensor(
        profile_id="test-sensor",
        channel_ids=("altitude",),
        cadence_s=0.1,
        latency_s=0.0,
        channel_errors=errors,
        seed=17,
    )
    same = CommittedBoundarySensor(
        profile_id="test-sensor",
        channel_ids=("altitude",),
        cadence_s=0.1,
        latency_s=0.0,
        channel_errors=errors,
        seed=17,
    )
    initial = first.advance(0.0, {"altitude": 100.0})
    assert initial.values == same.advance(0.0, {"altitude": 100.0}).values
    assert initial.values["altitude"] != pytest.approx(100.0)

    checkpoint = first.checkpoint_payload()
    restored = CommittedBoundarySensor(
        profile_id="test-sensor",
        channel_ids=("altitude",),
        cadence_s=0.1,
        latency_s=0.0,
        channel_errors=errors,
        seed=17,
    )
    restored.restore_checkpoint(checkpoint)
    next_first = first.advance(0.1, {"altitude": 101.0})
    next_restored = restored.advance(0.1, {"altitude": 101.0})
    assert next_restored.values == next_first.values
    assert checkpoint["seed"] == 17
    assert checkpoint["channel_errors"]["altitude"]["quantization_step"] == pytest.approx(0.05)
    ####


def test_composition_declares_scalar_measurement_errors_in_the_interface_fingerprint() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_sensor_episode_3dof_compose.yaml")
    payload = request.model_dump(mode="json", by_alias=True)
    payload["observation"]["declared_sensor"]["channel_errors"] = {
        "position.altitude": {"bias": 0.5, "gaussian_stddev": 0.2, "quantization_step": 0.1},
    }
    noisy = compile_vehicle_composition(VehicleCompositionRequest.model_validate(payload))
    baseline = compile_vehicle_composition(request)

    noisy_contract = resolve_vehicle_composition_interface_contract(noisy)
    profile = noisy_contract.observation_profile("declared_sensor")
    assert noisy_contract.fingerprint != resolve_vehicle_composition_interface_contract(baseline).fingerprint
    assert profile.channel_errors["position.altitude"] == {
        "bias": pytest.approx(0.5),
        "gaussian_stddev": pytest.approx(0.2),
        "quantization_step": pytest.approx(0.1),
    }
    trace = build_declared_sensor_trace(
        noisy,
        (
            _x8_sample(0.0, altitude_ft=580.0),
            _x8_sample(0.05, altitude_ft=600.0),
        ),
        seed=29,
    )
    assert trace is not None
    assert trace["seed"] == 29
    assert trace["channel_errors"]["position.altitude"]["bias"] == pytest.approx(0.5)
    samples = trace["samples"]
    assert isinstance(samples, list)
    assert samples[1]["values"]["position.altitude"] != pytest.approx(580.0 * 0.3048)
    ####
