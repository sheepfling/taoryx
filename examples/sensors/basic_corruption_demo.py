"""Emit deterministic ideal and corrupted packets from the basic sensor plug-ins."""

from __future__ import annotations

import json
from collections.abc import Mapping

import numpy as np

from taoryx.runtime import SensorBinding, SensorClockSpec
from taoryx.sensor_api import (
    EntityTruth,
    SensorBuildContext,
    SensorContext,
    TruthPoint,
    packet_to_record,
    sensor_plugin_registry,
)


def _truth() -> TruthPoint:
    return TruthPoint(
        0.0,
        np.array([1000.0, 2000.0, 3000.0]),
        np.array([1.0, 2.0, 3.0]),
        np.array([1.0, 2.0, 3.0]),
        np.eye(3),
        np.zeros(3),
        np.zeros(3),
        np.zeros(3),
    )
    ####


def _packet(
    sensor_id: str,
    provider: str,
    config: Mapping[str, object],
    context: SensorContext,
    *,
    seed: int,
) -> dict[str, object]:
    registry = sensor_plugin_registry()
    manifest = registry.descriptor(provider).manifest
    model = registry.create(
        provider,
        config,
        SensorBuildContext(sensor_id=sensor_id, seed=seed),
    )
    binding = SensorBinding(
        sensor_id,
        "demo-vehicle",
        SensorClockSpec(
            sensor_id,
            sorted(manifest.language_kinds)[0],
            cadence_s=manifest.default_cadence_s,
        ),
        model,
        rng_seed=seed,
    )
    packet = binding.sample_point(context.host, context)
    return packet_to_record(packet, provenance=model.provenance)
    ####


def build_demo() -> dict[str, object]:
    """Return a debug witness that intentionally shows truth beside measurements."""

    truth = _truth()
    context = SensorContext(
        "corruption-demo@0",
        truth,
        {"target": EntityTruth("target", np.array([1100.0, 2010.0, 3000.0]))},
    )
    return {
        "purpose": "debug-only ideal-versus-corrupted measurement witness",
        "truth_is_immutable": True,
        "truth": {
            "host_position_eci_m": truth.position_eci_m.tolist(),
            "host_velocity_eci_mps": truth.velocity_eci_mps.tolist(),
            "target_position_eci_m": context.entities["target"].position_eci_m.tolist(),
        },
        "infrared_bearing": {
            "ideal": _packet("ir-ideal", "ir-bearing", {"target_id": "target"}, context, seed=11),
            "corrupted": _packet(
                "ir-corrupted",
                "ir-bearing",
                {
                    "target_id": "target",
                    "azimuth_bias_rad": 0.001,
                    "elevation_bias_rad": -0.0005,
                    "angular_noise_stddev_rad": 0.0002,
                },
                context,
                seed=11,
            ),
        },
        "infrared_point_source": {
            "ideal": _packet("fpa-ideal", "ir-point-source", {"target_id": "target"}, context, seed=12),
            "corrupted": _packet(
                "fpa-corrupted",
                "ir-point-source",
                {
                    "target_id": "target",
                    "centroid_bias_pixels": [0.2, -0.1],
                    "centroid_noise_stddev_pixels": 0.35,
                },
                context,
                seed=12,
            ),
        },
        "gnss_fix": {
            "ideal": _packet("gnss-ideal", "gnss-fix", {}, context, seed=13),
            "corrupted": _packet(
                "gnss-corrupted",
                "gnss-fix",
                {
                    "position_bias_eci_m": [0.5, -0.25, 0.75],
                    "velocity_bias_eci_mps": [0.01, -0.01, 0.02],
                    "position_stddev_m": 2.5,
                    "velocity_stddev_mps": 0.08,
                },
                context,
                seed=13,
            ),
        },
    }
    ####


if __name__ == "__main__":
    print(json.dumps(build_demo(), indent=2, sort_keys=True))
    ####
