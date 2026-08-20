"""The mandatory Earth-fixed kinematic and orientation contract.

This module is intentionally free of projection code.  A provider or its
adapter is responsible for converting native state into the standard frame;
the contract package validates the resulting portable state without importing
a simulator, a model catalogue, or the TAORYX language runtime.
"""

from __future__ import annotations

import math
from typing import Literal, TypeAlias

from pydantic import BaseModel, ConfigDict, Field, model_validator

Vector3: TypeAlias = tuple[float, float, float]
QuaternionWxyz: TypeAlias = tuple[float, float, float, float]

EcefProjectionKind = Literal[
    "native_ecfc",
    "geodetic_wgs84",
    "ecic_to_ecfc_zero_epoch",
    "local_ned_wgs84_equatorial_embedding",
    "local_cartesian_wgs84_equatorial_embedding",
    "reference_origin_wgs84_equatorial_embedding",
    "provider_defined",
]
EcefOrientationKind = Literal[
    "native_ecef_from_body",
    "native_ned_from_body",
    "native_ecic_from_body",
    "native_body_from_local_ned",
    "native_inertial_to_body",
    "kinematic_velocity_aligned",
    "held_kinematic_orientation",
    "local_ned_reference",
    "provider_defined",
]
EcefAccelerationKind = Literal[
    "native_earth_relative_acceleration",
    "finite_difference_earth_relative_velocity",
    "provider_defined",
]
EcefAngularVelocityKind = Literal[
    "native_body_rate",
    "orientation_finite_difference",
    "provider_defined",
]


class StandardEcefState(BaseModel):
    """Required ECEF kinematics and ECEF/world-from-body orientation.

    ``ecef_from_body_wxyz`` maps a forward/right/down vector expressed in the
    body's reference frame into the Earth-centred Earth-fixed world frame.
    Consumers can dead reckon position with the kinematic fields and attitude
    with the body-frame angular velocity and orientation quaternion.  The
    provenance fields distinguish native truth from an explicitly documented
    kinematic projection.
    """

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.standard-ecef-state/v1"] = Field(
        default="taoryx.standard-ecef-state/v1",
        alias="schema",
        serialization_alias="schema",
    )
    frame_id: Literal["ecfc"] = "ecfc"
    position_ecef_m: Vector3
    velocity_ecef_mps: Vector3
    acceleration_ecef_mps2: Vector3
    acceleration_kind: EcefAccelerationKind = "finite_difference_earth_relative_velocity"
    angular_velocity_body_radps: Vector3
    angular_velocity_kind: EcefAngularVelocityKind
    ecef_from_body_wxyz: QuaternionWxyz
    position_projection: EcefProjectionKind
    source_frame: str = Field(min_length=1)
    orientation_kind: EcefOrientationKind

    @model_validator(mode="after")
    def validate_state(self) -> StandardEcefState:
        values = (
            *self.position_ecef_m,
            *self.velocity_ecef_mps,
            *self.acceleration_ecef_mps2,
            *self.angular_velocity_body_radps,
            *self.ecef_from_body_wxyz,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("standard ECEF state requires finite kinematic and orientation values")
        norm = math.sqrt(sum(value * value for value in self.ecef_from_body_wxyz))
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-9):
            raise ValueError("standard ECEF orientation quaternion must have unit norm")
        return self
        ####

    ####


__all__ = [
    "EcefAccelerationKind",
    "EcefAngularVelocityKind",
    "EcefOrientationKind",
    "EcefProjectionKind",
    "QuaternionWxyz",
    "StandardEcefState",
    "Vector3",
]
