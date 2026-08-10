"""Explicit body-axis dynamics binding for DAVE-ML family loads."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass

import numpy as np

from .daveml_import import DAVEMLFixedWingLoadBinding


@dataclass(frozen=True, slots=True)
class DAVEMLFixedWingDynamicsBinding:
    """Evaluate body-axis translational and rotational state derivatives."""

    loads: DAVEMLFixedWingLoadBinding
    mass_kg: float
    inertia_matrix_kg_m2: tuple[tuple[float, ...], ...]
    gravity_body_m_s2: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if not math.isfinite(self.mass_kg) or self.mass_kg <= 0.0:
            raise ValueError("DAVE-ML dynamics mass must be positive and finite")
        matrix = np.asarray(self.inertia_matrix_kg_m2, dtype=float)
        if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
            raise ValueError("DAVE-ML dynamics inertia matrix must be finite 3x3")
        if not np.allclose(matrix, matrix.T) or np.any(np.linalg.eigvalsh(matrix) <= 0.0):
            raise ValueError("DAVE-ML dynamics inertia matrix must be symmetric positive definite")
        if len(self.gravity_body_m_s2) != 3 or not all(math.isfinite(value) for value in self.gravity_body_m_s2):
            raise ValueError("DAVE-ML body gravity must contain three finite values")
        ####

    def evaluate(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        environment: Mapping[str, float] | None = None,
    ) -> dict[str, float]:
        """Return ``u_dot,v_dot,w_dot,p_dot,q_dot,r_dot`` in SI units."""

        names = ("u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s")
        missing = set(names) - set(state)
        if missing:
            raise KeyError("DAVE-ML dynamics state is missing: " + ", ".join(sorted(missing)))
        values = {name: float(state[name]) for name in names}
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError("DAVE-ML dynamics state must be finite")
        loads = self.loads.evaluate(state, controls, environment)
        u, v, w = (values[name] for name in names[:3])
        p, q, r = (values[name] for name in names[3:])
        fx = loads["total_force_x_n"] / self.mass_kg + self.gravity_body_m_s2[0]
        fy = loads["total_force_y_n"] / self.mass_kg + self.gravity_body_m_s2[1]
        fz = loads["total_force_z_n"] / self.mass_kg + self.gravity_body_m_s2[2]
        linear = {
            "u_m_s": fx - q * w + r * v,
            "v_m_s": fy - r * u + p * w,
            "w_m_s": fz - p * v + q * u,
        }
        inertia = np.asarray(self.inertia_matrix_kg_m2, dtype=float)
        omega = np.array((p, q, r), dtype=float)
        moment = np.array(
            (
                loads["total_moment_x_nm"],
                loads["total_moment_y_nm"],
                loads["total_moment_z_nm"],
            ),
            dtype=float,
        )
        angular = np.linalg.solve(inertia, moment - np.cross(omega, inertia @ omega))
        linear.update({"p_rad_s": float(angular[0]), "q_rad_s": float(angular[1]), "r_rad_s": float(angular[2])})
        return linear
        ####

    def as_evaluator(
        self,
        environment: Mapping[str, float] | None = None,
    ) -> Callable[[Mapping[str, float], Mapping[str, float]], Mapping[str, float]]:
        """Return the true state-derivative evaluator for linearization."""

        return lambda state, controls: self.evaluate(state, controls, environment)
        ####


__all__ = ["DAVEMLFixedWingDynamicsBinding"]

