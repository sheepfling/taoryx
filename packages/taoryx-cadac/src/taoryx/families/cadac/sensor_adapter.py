"""CADAC local-level truth adapter for Taoryx native sensor projections.

CADAC source modules commonly exchange a local north/east/down vector and a
body-from-local direction-cosine matrix.  The Taoryx sensor API currently
names its generic vector fields ``*_eci`` because it was first used by the
vehicle runtime.  This adapter uses those fields only as a *right-handed,
local orthonormal embedding* for instantaneous geometry.  It makes no ECI,
geodetic, gravity, or atmosphere assertion, and must not be used to bridge
CADAC's environment model into the Taoryx environment service.
"""

from __future__ import annotations

from functools import lru_cache
from typing import TypeAlias

import numpy as np

from taoryx.sensor_api import EntityTruth, SensorContext, TruthPoint
from taoryx.sensor_plugins.relative_state import (
    RelativeStateTrack,
    RelativeStateTrackerConfig,
    relative_state_track_from_geometry,
)

CadacRelativeStateResult: TypeAlias = RelativeStateTrack | str


def cadac_local_ned_sensor_context(
    *,
    time_s: float,
    host_position_ned_m: np.ndarray,
    host_velocity_ned_mps: np.ndarray,
    target_id: str,
    target_position_ned_m: np.ndarray,
    target_velocity_ned_mps: np.ndarray,
    body_from_local: np.ndarray,
    host_body_rate_rad_s: np.ndarray | None = None,
) -> SensorContext:
    """Embed one CADAC local-NED observation in the native sensor contract.

    ``body_from_local`` maps a local vector to the host body frame.  The
    sensor API needs the inverse (local from body), so the transpose is used
    as the orientation argument.  The zero gravity field is intentionally a
    non-physical placeholder for a geometry-only projection; no environment
    model is consulted or carried by the resulting context.
    """

    body_from_local_array = _nearest_proper_rotation(np.asarray(body_from_local, dtype=float))
    local_from_body = body_from_local_array.T
    host_velocity = np.asarray(host_velocity_ned_mps, dtype=float)
    body_rate = np.zeros(3, dtype=float) if host_body_rate_rad_s is None else np.asarray(host_body_rate_rad_s, dtype=float)
    host = TruthPoint(
        time_s=time_s,
        position_eci_m=np.asarray(host_position_ned_m, dtype=float),
        velocity_eci_mps=host_velocity,
        velocity_without_gravity_eci_mps=host_velocity,
        orientation_eci_from_body=local_from_body,
        gravity_eci_mps2=np.zeros(3, dtype=float),
        angular_rate_body_radps=body_rate,
    )
    return SensorContext(
        snapshot_id=f"cadac-local-ned@{time_s:.17g}:{target_id}",
        host=host,
        entities={
            target_id: EntityTruth(
                target_id,
                np.asarray(target_position_ned_m, dtype=float),
                np.asarray(target_velocity_ned_mps, dtype=float),
            )
        },
    )
    ####


def _nearest_proper_rotation(matrix: np.ndarray) -> np.ndarray:
    """Project a numerically drifted source DCM to a physical orientation.

    Several CADAC compatibility paths integrate their DCM directly, so its
    orthogonality can drift slightly between normalisation events.  The common
    near-rotation case uses a fast polar-iteration correction; an SVD remains
    the safe fallback for a substantially malformed source matrix.  Neither
    path adds a frame or environment model.
    """

    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("CADAC body_from_local must be a finite 3x3 matrix")
    identity = np.eye(3, dtype=float)
    gram_error = float(np.max(np.abs(matrix.T @ matrix - identity)))
    determinant = float(np.linalg.det(matrix))
    if gram_error <= 1.0e-10 and abs(determinant - 1.0) <= 1.0e-10:
        return matrix
    if determinant > 0.0 and gram_error <= 1.0e-2:
        # Newton's polar iteration squares small orthogonality error on each
        # pass and avoids an SVD in the source-timestep hot path.
        rotation = matrix
        for _ in range(2):
            rotation = 0.5 * rotation @ (3.0 * identity - rotation.T @ rotation)
        return rotation
    left, _, right = np.linalg.svd(matrix)
    rotation = left @ right
    if np.linalg.det(rotation) < 0.0:
        left[:, -1] *= -1.0
        rotation = left @ right
    return rotation
    ####


def cadac_local_ned_relative_state_track(
    *,
    time_s: float,
    host_position_ned_m: np.ndarray,
    host_velocity_ned_mps: np.ndarray,
    target_id: str,
    target_position_ned_m: np.ndarray,
    target_velocity_ned_mps: np.ndarray,
    body_from_local: np.ndarray,
    host_body_rate_rad_s: np.ndarray | None = None,
    config: RelativeStateTrackerConfig | None = None,
    rng: np.random.Generator | None = None,
) -> CadacRelativeStateResult:
    """Evaluate the native Taoryx relative-state sensor in CADAC local NED.

    CADAC source integrations retain their own gimbal, acquisition, lock,
    filtering, track-management, and scheduling state above this raw typed
    observation.  A caller can supply the native configuration and RNG to
    reproduce a source measurement-noise sequence explicitly.
    """

    selected_config = config or _default_relative_state_config(target_id)
    if selected_config.target_id != target_id:
        raise ValueError("CADAC sensor adapter target_id must match the native sensor configuration")
    local_from_body = _nearest_proper_rotation(np.asarray(body_from_local, dtype=float)).T
    return relative_state_track_from_geometry(
        target_id=target_id,
        host_position_world_m=np.asarray(host_position_ned_m, dtype=float),
        host_velocity_world_mps=np.asarray(host_velocity_ned_mps, dtype=float),
        orientation_world_from_body=local_from_body,
        target_position_world_m=np.asarray(target_position_ned_m, dtype=float),
        target_velocity_world_mps=np.asarray(target_velocity_ned_mps, dtype=float),
        host_body_rate_rad_s=None if host_body_rate_rad_s is None else np.asarray(host_body_rate_rad_s, dtype=float),
        config=selected_config,
        rng=rng,
    )
    ####


@lru_cache(maxsize=32)
def _default_relative_state_config(target_id: str) -> RelativeStateTrackerConfig:
    """Reuse immutable native defaults in source-timestep hot paths."""

    return RelativeStateTrackerConfig(target_id=target_id)
    ####


__all__ = [
    "CadacRelativeStateResult",
    "cadac_local_ned_relative_state_track",
    "cadac_local_ned_sensor_context",
]
####
