from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from taoryx.runtime import RuntimeProblem, RuntimeState, RuntimeVehicle, SensorClockSpec
from taoryx.runtime.engine import compute_trajectories
from taoryx.runtime.sensor_scenario import SensorScenarioSpec, attach_sensor_scenario
from taoryx.sensor_api import (
    EntityTruth,
    MeasurementPacket,
    SensorBuildContext,
    SensorContext,
    SensorSampleRequest,
    packet_from_record,
    packet_to_record,
    sensor_plugin_registry,
)
from taoryx.sensor_plugins.gnss import GnssFix
from taoryx.sensor_plugins.infrared import BearingDetection, FocalPlaneDetection
from taoryx.sensor_plugins.relative_state import RelativeStateTrack
from taoryx.sensors import TruthPoint

ROOT = Path(__file__).resolve().parents[2]


def _rigid_truth(state: RuntimeState) -> TruthPoint:
    position = np.array([state.values[0], 0.0, 0.0])
    velocity = np.array([1.0, 0.0, 0.0])
    return TruthPoint(
        state.time,
        position,
        velocity,
        velocity,
        np.eye(3),
        np.zeros(3),
        np.zeros(3),
        np.zeros(3),
    )
    ####


def _translation_truth(state: RuntimeState) -> TruthPoint:
    position = np.array([1000.0 + state.time, 2000.0, 3000.0])
    velocity = np.array([1.0, 2.0, 3.0])
    return TruthPoint(
        state.time,
        position,
        velocity,
        velocity,
        None,
        np.zeros(3),
        None,
        np.zeros(3),
    )
    ####


def _vehicle(truth_provider: object, *, step_size: float = 0.1, final_time: float = 0.2) -> RuntimeProblem:
    vehicle = RuntimeVehicle(
        "vehicle",
        RuntimeState(0.0, (0.0,)),
        lambda _: (1.0,),
        step_size=step_size,
        truth_provider=truth_provider,
    )
    return RuntimeProblem({vehicle.name: vehicle}, final_time=final_time)
    ####


def _run_ir_with_config(
    config: dict[str, object],
    *,
    seed: int,
) -> tuple[MeasurementPacket[BearingDetection], ...]:
    problem = _vehicle(_rigid_truth)
    spec = SensorScenarioSpec(
        "nose_ir",
        "vehicle",
        provider="ir-bearing",
        provider_config={
            "kind": "ir-bearing",
            "config": {"target_id": "target", **config},
        },
        cadence_s=0.1,
        estimator_modes=(),
        seed=seed,
    )

    def scene(state: RuntimeState, host: TruthPoint) -> SensorContext:
        return SensorContext(
            f"scene@{state.time:.17g}",
            host,
            {"target": EntityTruth("target", np.array([100.0, 10.0, 0.0]))},
        )
        ####

    runtime = attach_sensor_scenario(problem, spec, sensor_context_provider=scene)
    assert compute_trajectories(problem, max_steps=5).completed
    return runtime.bus.packets("nose_ir")
    ####


def _run_ir(seed: int) -> tuple[MeasurementPacket[BearingDetection], ...]:
    return _run_ir_with_config({"angular_noise_stddev_rad": 0.001}, seed=seed)
    ####


def test_registry_exposes_distinct_sensor_family_capabilities() -> None:
    manifests = {manifest.kind: manifest for manifest in sensor_plugin_registry().manifests()}

    assert manifests["ideal"].family == "inertial"
    assert manifests["ideal"].language_kinds == {"imu"}
    assert manifests["ir-bearing"].required_truth >= {"orientation", "entities"}
    assert manifests["ir-bearing"].language_kinds == {"infrared", "camera"}
    assert manifests["gnss-fix"].required_truth == {"position", "velocity"}
    assert manifests["gnss-fix"].language_kinds == {"gnss", "gps"}
    assert manifests["ir-bearing"].outputs[0].schema_id == "taoryx.ir.bearing-detection/v1"
    assert manifests["ir-point-source"].outputs[0].schema_id == "taoryx.ir.focal-plane-detection/v1"
    assert manifests["relative-state-track"].family == "tracking"
    assert manifests["relative-state-track"].outputs[0].schema_id == "taoryx.tracking.relative-state/v1"
    assert manifests["gnss-fix"].outputs[0].schema_id == "taoryx.gnss.fix/v1"


def test_relative_state_tracker_projects_typed_sensor_frame_geometry() -> None:
    host = TruthPoint(
        2.0,
        np.zeros(3),
        np.asarray((10.0, 0.0, 0.0)),
        np.asarray((10.0, 0.0, 0.0)),
        np.eye(3),
        np.zeros(3),
        np.zeros(3),
    )
    context = SensorContext(
        "track@2",
        host,
        {"target": EntityTruth("target", np.asarray((100.0, 10.0, 0.0)), np.asarray((0.0, 20.0, 0.0)))},
    )
    tracker = sensor_plugin_registry().create(
        "relative-state-track",
        {"target_id": "target"},
        SensorBuildContext("tracker", 17),
    )

    packet = tracker.sample_request(SensorSampleRequest(point=context, rng=np.random.default_rng(19)))

    assert packet.valid
    assert packet.port == "track"
    assert packet.schema_id == "taoryx.tracking.relative-state/v1"
    assert isinstance(packet.payload, RelativeStateTrack)
    payload = packet.payload
    expected_relative_position = np.asarray((100.0, 10.0, 0.0))
    expected_relative_velocity = np.asarray((-10.0, 20.0, 0.0))
    expected_range = float(np.linalg.norm(expected_relative_position))
    expected_unit = expected_relative_position / expected_range
    assert payload.range_m == pytest.approx(expected_range)
    assert payload.relative_position_sensor_m == pytest.approx(tuple(expected_relative_position))
    assert payload.relative_velocity_sensor_mps == pytest.approx(tuple(expected_relative_velocity))
    assert payload.unit_los_sensor == pytest.approx(tuple(expected_unit))
    assert payload.closing_speed_mps == pytest.approx(-float(expected_unit @ expected_relative_velocity))
    assert payload.line_of_sight_rate_sensor_rad_s == pytest.approx(tuple(np.cross(expected_unit, expected_relative_velocity) / expected_range))
    record = packet_to_record(packet)
    assert record["payload_contract"]["truth_position_output"] is False
    recovered = packet_from_record(record)
    assert isinstance(recovered.payload, RelativeStateTrack)
    assert recovered.payload == payload
    ####


def test_existing_imu_scenario_is_constructed_through_plugin_registry() -> None:
    problem = _vehicle(_rigid_truth)
    spec = SensorScenarioSpec(
        "imu",
        "vehicle",
        provider="ideal",
        cadence_s=0.1,
        estimator_modes=("dead-reckoning",),
    )
    runtime = attach_sensor_scenario(problem, spec)

    assert compute_trajectories(problem, max_steps=5).completed
    plugin = runtime.bus.bindings[0].provenance["sensor_plugin"]
    assert plugin == {
        "api_version": "1",
        "kind": "ideal",
        "family": "inertial",
        "language_kinds": ["imu"],
        "config": {},
    }
    assert runtime.bus.packets("imu")[0].schema_id == "taoryx.imu.increment/v1"


def test_ir_bearing_plugin_uses_context_without_emitting_target_truth_position() -> None:
    first = _run_ir(17)
    replay = _run_ir(17)

    assert len(first) == len(replay) == 3
    assert [packet.sequence for packet in first] == [0, 1, 2]
    for packet, repeated in zip(first, replay, strict=True):
        assert packet.sensor_id == "nose_ir"
        assert packet.port == "detections"
        assert packet.schema_id == "taoryx.ir.bearing-detection/v1"
        assert isinstance(packet.payload, BearingDetection)
        assert isinstance(repeated.payload, BearingDetection)
        assert packet.payload.azimuth_rad == repeated.payload.azimuth_rad
        record = packet_to_record(packet)
        assert record["payload_contract"]["truth_position_output"] is False
        assert "position" not in record["payload"]


def test_ir_bearing_plugin_fails_closed_outside_field_of_view() -> None:
    problem = _vehicle(_rigid_truth, final_time=0.0)
    spec = SensorScenarioSpec(
        "nose_ir",
        "vehicle",
        provider="ir-bearing",
        provider_config={"kind": "ir-bearing", "config": {"target_id": "target"}},
        cadence_s=0.1,
        estimator_modes=(),
    )

    def scene(state: RuntimeState, host: TruthPoint) -> SensorContext:
        return SensorContext(
            f"scene@{state.time:.17g}",
            host,
            {"target": EntityTruth("target", np.array([-100.0, 0.0, 0.0]))},
        )
        ####

    runtime = attach_sensor_scenario(problem, spec, sensor_context_provider=scene)
    packet = runtime.bus.packets("nose_ir")[0]

    assert not packet.valid
    assert packet.payload is None
    assert packet.invalid_reason == "outside-field-of-view"
    with pytest.raises(TypeError, match="requires explicit rebind"):
        runtime.checkpoint_payload()


def test_ir_bearing_bias_is_an_explicit_corruption_of_ideal_geometry() -> None:
    ideal = _run_ir_with_config({}, seed=3)
    biased = _run_ir_with_config(
        {
            "azimuth_bias_rad": 0.01,
            "elevation_bias_rad": -0.02,
            "angular_noise_stddev_rad": 0.0,
        },
        seed=999,
    )

    for ideal_packet, biased_packet in zip(ideal, biased, strict=True):
        assert isinstance(ideal_packet.payload, BearingDetection)
        assert isinstance(biased_packet.payload, BearingDetection)
        assert biased_packet.payload.azimuth_rad - ideal_packet.payload.azimuth_rad == pytest.approx(0.01)
        assert biased_packet.payload.elevation_rad - ideal_packet.payload.elevation_rad == pytest.approx(-0.02)
        assert biased_packet.payload.covariance_rad2 == ((0.0, 0.0), (0.0, 0.0))
    ####


def test_point_source_focal_plane_projects_and_corrupts_a_centroid() -> None:
    def sample(config: dict[str, object], seed: int) -> FocalPlaneDetection:
        problem = _vehicle(_rigid_truth, final_time=0.0)
        spec = SensorScenarioSpec(
            "fpa",
            "vehicle",
            provider="ir-point-source",
            provider_config={
                "kind": "ir-point-source",
                "config": {"target_id": "target", **config},
            },
            estimator_modes=(),
            seed=seed,
        )

        def scene(state: RuntimeState, host: TruthPoint) -> SensorContext:
            return SensorContext(
                f"scene@{state.time:.17g}",
                host,
                {"target": EntityTruth("target", np.array([100.0, 10.0, 0.0]))},
            )
            ####

        runtime = attach_sensor_scenario(problem, spec, sensor_context_provider=scene)
        packet = runtime.bus.packets("fpa")[0]
        assert packet.schema_id == "taoryx.ir.focal-plane-detection/v1"
        assert isinstance(packet.payload, FocalPlaneDetection)
        record = packet_to_record(packet)
        assert "position" not in record["payload"]
        assert isinstance(packet_from_record(record).payload, FocalPlaneDetection)
        return packet.payload
        ####

    ideal = sample({}, 1)
    biased = sample({"centroid_bias_pixels": [2.0, -3.0]}, 2)
    noisy = sample({"centroid_noise_stddev_pixels": 0.5}, 17)
    repeated = sample({"centroid_noise_stddev_pixels": 0.5}, 17)

    assert ideal.image_size_pixels == (640, 480)
    assert ideal.column_px > 0.5 * 640
    assert ideal.row_px == pytest.approx(0.5 * (480 - 1))
    assert biased.column_px - ideal.column_px == pytest.approx(2.0)
    assert biased.row_px - ideal.row_px == pytest.approx(-3.0)
    assert noisy.column_px == repeated.column_px
    assert noisy.row_px == repeated.row_px
    assert (noisy.column_px, noisy.row_px) != pytest.approx((ideal.column_px, ideal.row_px))
    ####


def test_instantaneous_scene_context_is_queried_only_at_due_sample_boundaries() -> None:
    problem = _vehicle(_rigid_truth)
    spec = SensorScenarioSpec(
        "nose_ir",
        "vehicle",
        provider="ir-bearing",
        provider_config={"kind": "ir-bearing", "config": {"target_id": "target"}},
        cadence_s=0.1,
        phase_s=0.1,
        estimator_modes=(),
    )
    context_times: list[float] = []

    def scene(state: RuntimeState, host: TruthPoint) -> SensorContext:
        context_times.append(state.time)
        return SensorContext(
            f"scene@{state.time:.17g}",
            host,
            {"target": EntityTruth("target", np.array([100.0, 0.0, 0.0]))},
        )
        ####

    runtime = attach_sensor_scenario(problem, spec, sensor_context_provider=scene)
    assert context_times == []
    assert compute_trajectories(problem, max_steps=5).completed

    assert context_times == pytest.approx([0.1, 0.2])
    assert [packet.sampled_at_s for packet in runtime.bus.packets("nose_ir")] == pytest.approx([0.1, 0.2])
    ####


def test_gnss_fix_plugin_is_context_free_typed_and_reproducible() -> None:
    def run() -> tuple[MeasurementPacket[GnssFix], ...]:
        problem = _vehicle(_translation_truth, step_size=0.5, final_time=1.0)
        spec = SensorScenarioSpec(
            "gnss",
            "vehicle",
            provider="gnss-fix",
            provider_config={
                "kind": "gnss-fix",
                "config": {
                    "position_stddev_m": 2.0,
                    "velocity_stddev_mps": 0.1,
                },
            },
            truth_mode="translation-only",
            cadence_s=0.5,
            estimator_modes=(),
            seed=23,
        )
        runtime = attach_sensor_scenario(problem, spec)
        assert compute_trajectories(problem, max_steps=4).completed
        checkpoint = runtime.checkpoint_payload()
        assert checkpoint["bus"]["bindings"]["gnss"]["rng_state"]
        runtime.restore_checkpoint(checkpoint)
        return runtime.bus.packets("gnss")
        ####

    packets = run()
    replay = run()

    assert len(packets) == len(replay) == 3
    for packet, repeated in zip(packets, replay, strict=True):
        assert isinstance(packet.payload, GnssFix)
        assert isinstance(repeated.payload, GnssFix)
        assert packet.schema_id == "taoryx.gnss.fix/v1"
        assert packet.payload.frame_id == "ECI"
        assert packet.payload.position_eci_m == pytest.approx(repeated.payload.position_eci_m)
        assert np.diag(packet.payload.position_covariance_eci_m2) == pytest.approx([4.0, 4.0, 4.0])


def test_gnss_fix_exposes_ideal_bias_and_seeded_noise_rungs() -> None:
    def sample(config: dict[str, object], seed: int) -> GnssFix:
        problem = _vehicle(_translation_truth, step_size=0.5, final_time=0.0)
        spec = SensorScenarioSpec(
            "gnss",
            "vehicle",
            provider="gnss-fix",
            provider_config={"kind": "gnss-fix", "config": config},
            truth_mode="translation-only",
            estimator_modes=(),
            seed=seed,
        )
        runtime = attach_sensor_scenario(problem, spec)
        payload = runtime.bus.packets("gnss")[0].payload
        assert isinstance(payload, GnssFix)
        return payload
        ####

    ideal = sample({}, 1)
    biased = sample(
        {
            "position_bias_eci_m": [10.0, -20.0, 30.0],
            "velocity_bias_eci_mps": [0.1, -0.2, 0.3],
        },
        2,
    )
    noisy = sample({"position_stddev_m": 2.0, "velocity_stddev_mps": 0.1}, 7)
    repeated = sample({"position_stddev_m": 2.0, "velocity_stddev_mps": 0.1}, 7)

    assert ideal.position_eci_m == pytest.approx([1000.0, 2000.0, 3000.0])
    assert ideal.velocity_eci_mps == pytest.approx([1.0, 2.0, 3.0])
    assert biased.position_eci_m - ideal.position_eci_m == pytest.approx([10.0, -20.0, 30.0])
    assert biased.velocity_eci_mps - ideal.velocity_eci_mps == pytest.approx([0.1, -0.2, 0.3])
    assert noisy.position_eci_m == pytest.approx(repeated.position_eci_m)
    assert noisy.velocity_eci_mps == pytest.approx(repeated.velocity_eci_mps)
    assert not np.allclose(noisy.position_eci_m, ideal.position_eci_m)
    ####


def test_language_clock_family_must_match_selected_plugin_provider() -> None:
    problem = _vehicle(_translation_truth, final_time=0.0)
    problem.sensor_clocks = (SensorClockSpec("sensor", "gnss", cadence_s=1.0),)
    spec = SensorScenarioSpec(
        "sensor",
        "vehicle",
        provider="ir-bearing",
        provider_config={"kind": "ir-bearing", "config": {"target_id": "target"}},
        estimator_modes=(),
    )

    with pytest.raises(ValueError, match="declares language kind 'gnss'.*ir-bearing"):
        attach_sensor_scenario(problem, spec, sensor_context_provider=lambda state, host: SensorContext.from_host(host))
    ####


def test_language_clock_family_accepts_a_compatible_plugin_provider() -> None:
    problem = _vehicle(_translation_truth, final_time=0.0)
    problem.sensor_clocks = (SensorClockSpec("sensor", "gnss", cadence_s=0.5),)
    spec = SensorScenarioSpec(
        "sensor",
        "vehicle",
        provider="gnss-fix",
        truth_mode="translation-only",
        estimator_modes=(),
    )

    runtime = attach_sensor_scenario(problem, spec)

    assert runtime.bus.bindings[0].clock.cadence_s == pytest.approx(0.5)
    assert runtime.bus.bindings[0].clock.kind == "gnss"
    assert runtime.artifact()["sensor_plugin"]["language_kinds"] == ["gnss", "gps"]
    assert runtime.artifact()["sensor_plugin"]["standalone_clock_kind"] == "gnss-receiver"
    ####


def test_versioned_payload_codec_round_trips_sensor_envelope() -> None:
    packet = _run_ir(9)[0]
    record = packet_to_record(packet, provenance={"case": "codec-roundtrip"})
    restored = packet_from_record(record)

    assert restored.sensor_id == packet.sensor_id
    assert restored.port == packet.port
    assert restored.sequence == packet.sequence
    assert restored.schema_id == packet.schema_id
    assert isinstance(restored.payload, BearingDetection)
    assert isinstance(packet.payload, BearingDetection)
    assert restored.payload.azimuth_rad == packet.payload.azimuth_rad


def test_proof_sidecars_use_plugin_owned_nested_configuration() -> None:
    infrared = SensorScenarioSpec.from_file(ROOT / "examples/sensors/ir_bearing_v1.yaml")
    focal_plane = SensorScenarioSpec.from_file(ROOT / "examples/sensors/ir_point_source_v1.yaml")
    gnss = SensorScenarioSpec.from_file(ROOT / "examples/sensors/gnss_fix_v1.yaml")

    assert infrared.provider == "ir-bearing"
    assert infrared.resolved_provider_config().config["target_id"] == "target"
    assert infrared.estimator_modes == ()
    assert focal_plane.provider == "ir-point-source"
    assert focal_plane.resolved_provider_config().config["centroid_noise_stddev_pixels"] == 0.35
    assert focal_plane.estimator_modes == ()
    assert gnss.provider == "gnss-fix"
    assert gnss.resolved_provider_config().config["position_stddev_m"] == 2.5
    assert gnss.estimator_modes == ()
    assert gnss.to_metadata()["provider_config"]["config"]["outage_probability"] == 0.01
    ####
