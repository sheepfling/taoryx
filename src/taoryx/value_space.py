"""Mathematical value-space contracts for public Taoryx channels.

Units and primitive storage shape do not completely describe a value. A
heading stored as a scalar in degrees lives on a circle, while its rate lives
on a line. A quaternion is a redundant representation of an element of
``SO(3)``. This module records those distinctions at the semantic boundary.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

ValueSpaceTopology = Literal[
    "euclidean",
    "bounded_interval",
    "periodic_circle",
    "unit_sphere",
    "rotation_group_so3",
    "positive_half_line",
    "unit_interval",
    "simplex",
    "finite_set",
    "boolean",
    "event",
    "product",
    "topology_pending",
]


@dataclass(frozen=True, slots=True)
class ValueSpaceSpec:
    """Topology and legal numerical operations for one public value."""

    topology: ValueSpaceTopology
    representation: str
    error_rule: str
    interpolation_rule: str
    normalization_rule: str | None = None
    period: float | None = None
    equivalence: str | None = None
    coordinate_chart: str | None = None
    components: tuple[ValueSpaceSpec, ...] = ()

    def __post_init__(self) -> None:
        if not self.representation.strip() or not self.error_rule.strip() or not self.interpolation_rule.strip():
            raise ValueError("value-space specifications require representation, error, and interpolation rules")
        if self.topology == "periodic_circle":
            if self.period is None or not math.isfinite(self.period) or self.period <= 0.0:
                raise ValueError("periodic-circle value spaces require a positive finite period")
        elif self.period is not None:
            raise ValueError("only periodic-circle value spaces may declare a period")
        if self.topology == "product" and not self.components:
            raise ValueError("product value spaces require component specifications")
        if self.topology != "product" and self.components:
            raise ValueError("only product value spaces may declare component specifications")
        ####
    ####

    def as_dict(self) -> dict[str, object]:
        """Return a portable machine-readable contract."""

        return {
            "topology": self.topology,
            "representation": self.representation,
            "error_rule": self.error_rule,
            "interpolation_rule": self.interpolation_rule,
            "normalization_rule": self.normalization_rule,
            "period": self.period,
            "equivalence": self.equivalence,
            "coordinate_chart": self.coordinate_chart,
            "components": [item.as_dict() for item in self.components],
        }
        ####
    ####


def euclidean(dimension: int = 1) -> ValueSpaceSpec:
    """Describe an ordinary Cartesian scalar or vector."""

    if dimension < 1:
        raise ValueError("Euclidean value spaces require a positive dimension")
    return ValueSpaceSpec(
        "euclidean",
        "scalar" if dimension == 1 else f"vector{dimension}",
        "componentwise subtraction",
        "linear",
        coordinate_chart=f"R^{dimension}",
    )
    ####


def bounded_interval(*, representation: str = "scalar") -> ValueSpaceSpec:
    """Describe a bounded, non-periodic scalar coordinate."""

    return ValueSpaceSpec(
        "bounded_interval",
        representation,
        "linear subtraction",
        "linear with explicit bound handling",
    )
    ####


def periodic_circle(period: float, *, representation: str = "scalar") -> ValueSpaceSpec:
    """Describe a scalar angular coordinate on ``S1``."""

    return ValueSpaceSpec(
        "periodic_circle",
        representation,
        "wrapped shortest signed difference",
        "unwrap then interpolate along the shortest arc",
        normalization_rule="wrap to declared principal interval",
        period=period,
        equivalence="values separated by integer periods are equivalent",
        coordinate_chart="S1",
    )
    ####


def unit_interval() -> ValueSpaceSpec:
    """Describe a bounded fraction in ``[0, 1]``."""

    return ValueSpaceSpec(
        "unit_interval",
        "scalar",
        "linear subtraction",
        "linear with explicit clamp/reject policy",
        normalization_rule="bounded to [0, 1]",
        coordinate_chart="[0, 1]",
    )
    ####


def positive_half_line() -> ValueSpaceSpec:
    """Describe a nonnegative physical magnitude."""

    return ValueSpaceSpec(
        "positive_half_line",
        "scalar",
        "linear subtraction or log-space difference when configured",
        "linear; log-space only when an explicit consumer selects it",
        normalization_rule="nonnegative",
        coordinate_chart="[0, inf)",
    )
    ####


def quaternion_so3() -> ValueSpaceSpec:
    """Describe a unit quaternion representation of attitude."""

    return ValueSpaceSpec(
        "rotation_group_so3",
        "vector4 quaternion [w, x, y, z]",
        "geodesic/log-map rotation error with q equivalent to -q",
        "shortest-arc spherical interpolation (slerp)",
        normalization_rule="unit norm",
        equivalence="q and -q represent the same attitude",
        coordinate_chart="SO(3)",
    )
    ####


def unit_vector_sphere() -> ValueSpaceSpec:
    """Describe a three-dimensional unit direction."""

    return ValueSpaceSpec(
        "unit_sphere",
        "vector3",
        "angular/geodesic direction error",
        "normalized spherical interpolation",
        normalization_rule="unit norm",
        coordinate_chart="S2",
    )
    ####


def finite_set(*, event: bool = False, representation: str = "string") -> ValueSpaceSpec:
    """Describe enumerated state or an event label in a declared representation."""

    return ValueSpaceSpec(
        "event" if event else "finite_set",
        representation,
        "exact equality",
        "not interpolable",
    )
    ####


def boolean() -> ValueSpaceSpec:
    """Describe a boolean/discrete state."""

    return ValueSpaceSpec("boolean", "boolean", "exact equality", "not interpolable")
    ####


def product(*components: ValueSpaceSpec, representation: str) -> ValueSpaceSpec:
    """Describe an ordered tuple whose coordinates have different spaces."""

    return ValueSpaceSpec(
        "product",
        representation,
        "componentwise using each component error rule",
        "componentwise using each component interpolation rule",
        components=components,
    )
    ####


def default_value_space_for_value_type(value_type: str) -> ValueSpaceSpec:
    """Return the conservative default for a primitive representation."""

    if value_type == "scalar":
        return euclidean()
    if value_type == "vector3":
        return euclidean(3)
    if value_type == "vector4":
        return euclidean(4)
    if value_type == "boolean":
        return boolean()
    if value_type == "enum":
        return finite_set()
    if value_type == "event":
        return finite_set(event=True)
    raise ValueError(f"unsupported primitive value type {value_type!r}")
    ####


def validate_value_space_value(specification: ValueSpaceSpec, value: object, *, context: str) -> None:
    """Reject values that violate the representation invariant of a space."""

    if specification.topology == "topology_pending":
        raise ValueError(f"{context}: value space remains topology_pending")
    if specification.topology in {"finite_set", "event"}:
        if specification.representation == "scalar source state code":
            if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
                raise ValueError(f"{context}: source state code requires a finite numeric value")
            return
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{context}: finite-set and event values require a non-empty string")
        return
    if specification.topology == "boolean":
        if not isinstance(value, bool):
            raise ValueError(f"{context}: boolean value space requires a boolean")
        return
    if specification.topology == "product":
        if not isinstance(value, list | tuple) or len(value) != len(specification.components):
            raise ValueError(f"{context}: product value has the wrong component count")
        for component, item in zip(specification.components, value, strict=True):
            validate_value_space_value(component, item, context=context)
        return
    if specification.representation.startswith("vector"):
        size = int(specification.representation.removeprefix("vector").split()[0])
        if not isinstance(value, list | tuple) or len(value) != size:
            raise ValueError(f"{context}: {specification.representation} value has the wrong component count")
        if any(isinstance(item, bool) or not isinstance(item, int | float) or not math.isfinite(float(item)) for item in value):
            raise ValueError(f"{context}: vector value requires finite numeric components")
        components = tuple(float(item) for item in value)
        if specification.topology in {"unit_sphere", "rotation_group_so3"}:
            norm = math.sqrt(sum(item * item for item in components))
            if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-6):
                raise ValueError(f"{context}: {specification.topology} value requires unit norm")
        return
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"{context}: scalar value space requires a finite scalar")
    numeric = float(value)
    if specification.topology == "unit_interval" and not 0.0 <= numeric <= 1.0:
        raise ValueError(f"{context}: unit-interval value must be in [0, 1]")
    if specification.topology == "positive_half_line" and numeric < 0.0:
        raise ValueError(f"{context}: positive-half-line value must be nonnegative")
    ####


def signed_value_space_error(specification: ValueSpaceSpec, actual: float, target: float) -> float:
    """Return ``actual - target`` using the declared scalar topology.

    This deliberately covers scalar state and command channels only.  Vector
    spaces such as ``SO(3)`` require a vector-valued logarithmic-map error and
    must use their family-specific attitude operation instead of being
    accidentally flattened into a scalar residual.
    """

    if not math.isfinite(actual) or not math.isfinite(target):
        raise ValueError("value-space error requires finite scalar values")
    if specification.topology == "periodic_circle":
        if specification.period is None:
            raise ValueError("periodic-circle value space has no period")
        half_period = specification.period / 2.0
        return (actual - target + half_period) % specification.period - half_period
    if specification.representation != "scalar":
        raise ValueError(
            f"scalar error is not defined for {specification.topology} represented as {specification.representation}"
        )
    if specification.topology in {"finite_set", "event", "boolean", "product", "topology_pending"}:
        raise ValueError(f"scalar error is not defined for {specification.topology}")
    return actual - target
    ####


__all__ = [
    "ValueSpaceSpec",
    "ValueSpaceTopology",
    "boolean",
    "bounded_interval",
    "default_value_space_for_value_type",
    "euclidean",
    "finite_set",
    "periodic_circle",
    "positive_half_line",
    "product",
    "quaternion_so3",
    "signed_value_space_error",
    "unit_interval",
    "unit_vector_sphere",
    "validate_value_space_value",
]
