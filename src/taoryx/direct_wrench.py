"""Explicit bounded direct-wrench control for the rigid-body bridge tier.

The direct-wrench tier is intentionally between a response-law pseudo model and
physical effector allocation.  A controller may request a body-frame force and
moment, but the request is projected through declared authority and slew
limits before it reaches the nonlinear rigid-body plant.  The resulting load
records the injected control force and moment separately from aerodynamic and
propulsive loads.

This module does not model elevons, rotors, gimbals, or thrusters. Consumers
must label the resulting evidence ``direct_wrench`` and must not promote it to
physical-effector evidence. It is nevertheless a valid rigid-body bridge tier:
the applied wrench is integrated by the same Newton--Euler plant that later
receives allocated physical-effectors.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Literal, Mapping

from .contracts import Vector3
from .rigid_body import RigidBodyForceMoment

DIRECT_WRENCH_NAMES: tuple[str, ...] = (
    "force_x_n",
    "force_y_n",
    "force_z_n",
    "moment_x_nm",
    "moment_y_nm",
    "moment_z_nm",
)
DirectWrenchStatus = Literal["feasible", "partially_achievable"]


def _finite_mapping(values: Mapping[str, float], names: tuple[str, ...], label: str) -> dict[str, float]:
    """Validate and order a named wrench mapping."""

    missing = set(names) - set(values)
    extra = set(values) - set(names)
    if missing or extra:
        raise ValueError(f"{label} axes mismatch; missing={sorted(missing)}, extra={sorted(extra)}")
    result = {name: float(values[name]) for name in names}
    if any(not math.isfinite(value) for value in result.values()):
        raise ValueError(f"{label} values must be finite")
    return result
####


@dataclass(frozen=True, slots=True)
class DirectWrenchLimits:
    """Per-axis direct-wrench authority and slew limits in SI units."""

    lower: Mapping[str, float]
    upper: Mapping[str, float]
    rate_limit_per_s: Mapping[str, float | None]
    axes: tuple[str, ...] = DIRECT_WRENCH_NAMES

    def __post_init__(self) -> None:
        if self.axes != DIRECT_WRENCH_NAMES:
            raise ValueError("direct-wrench axes must use the canonical six-axis ordering")
        lower = _finite_mapping(self.lower, self.axes, "lower limits")
        upper = _finite_mapping(self.upper, self.axes, "upper limits")
        rates = dict(self.rate_limit_per_s)
        if set(rates) != set(self.axes):
            raise ValueError("direct-wrench rate limits must declare every canonical axis")
        for name in self.axes:
            if lower[name] > upper[name]:
                raise ValueError(f"direct-wrench lower limit exceeds upper limit for {name}")
            rate = rates[name]
            if rate is not None and (not math.isfinite(float(rate)) or float(rate) <= 0.0):
                raise ValueError(f"direct-wrench rate limit must be positive for {name}")
        ####
    ####

    def project(
        self,
        requested: Mapping[str, float],
        previous: Mapping[str, float],
        dt_s: float,
    ) -> DirectWrenchProjection:
        """Project a requested wrench through authority and rate limits."""

        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("direct-wrench projection dt_s must be finite and positive")
        request = _finite_mapping(requested, self.axes, "requested wrench")
        prior = _finite_mapping(previous, self.axes, "previous wrench")
        achieved: dict[str, float] = {}
        position_saturated: list[str] = []
        rate_limited: list[str] = []
        for name in self.axes:
            value = request[name]
            bounded = min(float(self.upper[name]), max(float(self.lower[name]), value))
            if not math.isclose(bounded, value, rel_tol=0.0, abs_tol=1e-12):
                position_saturated.append(name)
            rate = self.rate_limit_per_s[name]
            if rate is not None:
                delta = float(rate) * dt_s
                slew_limited = min(prior[name] + delta, max(prior[name] - delta, bounded))
                if not math.isclose(slew_limited, bounded, rel_tol=0.0, abs_tol=1e-12):
                    rate_limited.append(name)
                bounded = slew_limited
            achieved[name] = bounded
        status: DirectWrenchStatus = "feasible" if not position_saturated and not rate_limited else "partially_achievable"
        return DirectWrenchProjection(
            requested=request,
            achieved=achieved,
            residual={name: request[name] - achieved[name] for name in self.axes},
            status=status,
            position_saturated=tuple(position_saturated),
            rate_limited=tuple(rate_limited),
        )
    ####
####


@dataclass(frozen=True, slots=True)
class DirectWrenchProjection:
    """Auditable result of one bounded direct-wrench projection."""

    requested: Mapping[str, float]
    achieved: Mapping[str, float]
    residual: Mapping[str, float]
    status: DirectWrenchStatus
    position_saturated: tuple[str, ...] = ()
    rate_limited: tuple[str, ...] = ()

    @property
    def residual_norm(self) -> float:
        """Return the Euclidean six-axis requested/achieved residual."""

        return math.sqrt(sum(float(value) ** 2 for value in self.residual.values()))
        ####
    ####

    def as_dict(self) -> dict[str, object]:
        """Return a stable machine-readable direct-wrench record."""

        return {
            "control_realization": "direct_wrench",
            "physical_effector_allocation": False,
            "requested_wrench": dict(self.requested),
            "achieved_wrench": dict(self.achieved),
            "residual_wrench": dict(self.residual),
            "residual_norm": self.residual_norm,
            "status": self.status,
            "position_saturated": list(self.position_saturated),
            "rate_limited": list(self.rate_limited),
        }
        ####
    ####


def compose_direct_wrench_load(
    base: RigidBodyForceMoment,
    projection: DirectWrenchProjection,
) -> RigidBodyForceMoment:
    """Add an achieved direct wrench to a source/load-model result.

    The original aerodynamic and propulsion components are retained.  The
    direct control contribution is separately recorded so force/moment closure
    and claim review can distinguish source loads from injected control.
    """

    force = Vector3(
        projection.achieved["force_x_n"],
        projection.achieved["force_y_n"],
        projection.achieved["force_z_n"],
    )
    moment = Vector3(
        projection.achieved["moment_x_nm"],
        projection.achieved["moment_y_nm"],
        projection.achieved["moment_z_nm"],
    )
    return replace(
        base,
        force_body=base.force_body + force,
        moment_body=base.moment_body + moment,
        control_force_body=force,
        control_moment_body=moment,
    )
    ####


def add_direct_wrench_to_local_derivative(
    base_derivative: Mapping[str, float],
    projection: DirectWrenchProjection,
    *,
    mass_kg: float,
    inertia_kg_m2: Vector3,
) -> dict[str, float]:
    """Add a projected body wrench to a local velocity/rate derivative.

    This helper is for local control-analysis adapters whose state contains
    ``u_m_s``, ``v_m_s``, ``w_m_s``, ``p_rad_s``, ``q_rad_s``, and ``r_rad_s``.
    The source plant supplies the nonlinear derivative and this function adds
    only the incremental direct-wrench contribution using the declared mass
    and diagonal inertia.  It is not a replacement for a full load provider.
    """

    if not math.isfinite(mass_kg) or mass_kg <= 0.0:
        raise ValueError("direct-wrench local mass must be finite and positive")
    if not all(math.isfinite(value) and value > 0.0 for value in (inertia_kg_m2.x, inertia_kg_m2.y, inertia_kg_m2.z)):
        raise ValueError("direct-wrench local inertia must be finite and positive")
    required = ("u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s")
    missing = set(required) - set(base_derivative)
    if missing:
        raise ValueError(f"local derivative is missing channels: {', '.join(sorted(missing))}")
    result = {name: float(value) for name, value in base_derivative.items()}
    result["u_m_s"] += projection.achieved["force_x_n"] / mass_kg
    result["v_m_s"] += projection.achieved["force_y_n"] / mass_kg
    result["w_m_s"] += projection.achieved["force_z_n"] / mass_kg
    result["p_rad_s"] += projection.achieved["moment_x_nm"] / inertia_kg_m2.x
    result["q_rad_s"] += projection.achieved["moment_y_nm"] / inertia_kg_m2.y
    result["r_rad_s"] += projection.achieved["moment_z_nm"] / inertia_kg_m2.z
    return result
    ####


__all__ = [
    "DIRECT_WRENCH_NAMES",
    "DirectWrenchLimits",
    "DirectWrenchProjection",
    "add_direct_wrench_to_local_derivative",
    "compose_direct_wrench_load",
]
