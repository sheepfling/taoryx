"""Explicit lower-fidelity reductions for the F-16 S-119 reference plant.

These reductions deliberately sit below :mod:`f16_reference`.  The point-mass
model preserves the source translational force channels but carries no
attitude or moment state.  The pseudo-6DOF model adds a named kinematic
attitude response whose body-rate law is the source-derived local linear
model.  Neither class is a substitute for the source rigid-body plant or a
flight-envelope qualification.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np

from ..control_allocation import ProvenancedLinearization
from ..trim import TrimResult
from .f16_reference import F16ReferencePlant


@dataclass(frozen=True, slots=True)
class F16PointMass3DOFModel:
    """Source-force point-mass reduction at one fixed operating condition."""

    source: F16ReferencePlant
    trim: TrimResult
    trim_pitch_rad: float
    altitude_m: float = 0.0

    @property
    def state_names(self) -> tuple[str, ...]:
        """Return the retained body translational channels."""

        return ("u_m_s", "v_m_s", "w_m_s")
        ####
    ####

    def force_channels(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
    ) -> dict[str, float]:
        """Evaluate source force channels without retaining moments."""

        source_state = {
            **{name: float(state[name]) for name in self.state_names},
            "p_rad_s": 0.0,
            "q_rad_s": 0.0,
            "r_rad_s": 0.0,
        }
        loads = self.source.evaluate_loads(source_state, controls, altitude_m=self.altitude_m)
        return {name: float(loads[name]) for name in ("total_force_x_n", "total_force_y_n", "total_force_z_n")}
        ####
    ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
    ) -> dict[str, float]:
        """Return local body translational acceleration only.

        The attitude is held at the declared trim pitch and the retained
        channels are the source body translational dynamics.  This is an
        intentionally local 3DOF reduction; navigation position and attitude
        reconstruction belong to the family mission adapter.
        """

        force = self.force_channels(state, controls)
        gravity = {
            "u_m_s": -self.source.gravity_m_s2 * math.sin(self.trim_pitch_rad),
            "v_m_s": 0.0,
            "w_m_s": self.source.gravity_m_s2 * math.cos(self.trim_pitch_rad),
        }
        return {
            "u_m_s": force["total_force_x_n"] / self.source.mass_kg + gravity["u_m_s"],
            "v_m_s": force["total_force_y_n"] / self.source.mass_kg + gravity["v_m_s"],
            "w_m_s": force["total_force_z_n"] / self.source.mass_kg + gravity["w_m_s"],
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class F16AttitudeResponsePseudo6DOFModel:
    """Source-calibrated 3T+3K response bridge for local F-16 studies."""

    source: F16ReferencePlant
    trim: TrimResult
    linearization: ProvenancedLinearization
    trim_pitch_rad: float
    altitude_m: float = 0.0

    @property
    def state_names(self) -> tuple[str, ...]:
        """Return translational, rate, and kinematic attitude channels."""

        return (
            "u_m_s",
            "v_m_s",
            "w_m_s",
            "p_rad_s",
            "q_rad_s",
            "r_rad_s",
            "roll_rad",
            "pitch_rad",
            "yaw_rad",
        )
        ####
    ####

    def _local_error(self, state: Mapping[str, float]) -> np.ndarray:
        return np.asarray(
            [float(state[name]) - float(self.trim.state[name]) for name in self.linearization.primary.state_names],
            dtype=float,
        )
        ####
    ####

    def _control_error(self, controls: Mapping[str, float]) -> np.ndarray:
        return np.asarray(
            [float(controls[name]) - float(self.trim.controls[name]) for name in self.linearization.primary.control_names],
            dtype=float,
        )
        ####
    ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
    ) -> dict[str, float]:
        """Return source-force translation plus a kinematic rate response.

        The angular response uses the source-derived local A/B rows as a
        bounded response law.  It does not inject those rates back as a
        physical moment; that distinction is the reason this model remains a
        pseudo-6DOF realization.
        """

        source_state = {name: float(state[name]) for name in self.linearization.primary.state_names}
        loads = self.source.evaluate_loads(source_state, controls, altitude_m=self.altitude_m)
        p, q, r = (float(state[name]) for name in ("p_rad_s", "q_rad_s", "r_rad_s"))
        gravity = {
            "u_m_s": -self.source.gravity_m_s2 * math.sin(float(state["pitch_rad"])),
            "v_m_s": self.source.gravity_m_s2 * math.sin(float(state["roll_rad"])) * math.cos(float(state["pitch_rad"])),
            "w_m_s": self.source.gravity_m_s2 * math.cos(float(state["roll_rad"])) * math.cos(float(state["pitch_rad"])),
        }
        translational = {
            "u_m_s": loads["total_force_x_n"] / self.source.mass_kg + gravity["u_m_s"] - q * float(state["w_m_s"]) + r * float(state["v_m_s"]),
            "v_m_s": loads["total_force_y_n"] / self.source.mass_kg + gravity["v_m_s"] - r * float(state["u_m_s"]) + p * float(state["w_m_s"]),
            "w_m_s": loads["total_force_z_n"] / self.source.mass_kg + gravity["w_m_s"] - p * float(state["v_m_s"]) + q * float(state["u_m_s"]),
        }
        angular_rows = self.linearization.primary.a_matrix[3:, :] @ self._local_error(state)
        angular_rows = angular_rows + self.linearization.primary.b_matrix[3:, :] @ self._control_error(controls)
        phi = float(state["roll_rad"])
        theta = float(state["pitch_rad"])
        cosine_theta = math.cos(theta)
        if abs(cosine_theta) < 1.0e-6:
            raise ValueError("F-16 pseudo-6DOF attitude response is singular near pitch +/-90 degrees")
        tangent_theta = math.tan(theta)
        return {
            **translational,
            "p_rad_s": float(angular_rows[0]),
            "q_rad_s": float(angular_rows[1]),
            "r_rad_s": float(angular_rows[2]),
            "roll_rad": p + math.sin(phi) * tangent_theta * q + math.cos(phi) * tangent_theta * r,
            "pitch_rad": math.cos(phi) * q - math.sin(phi) * r,
            "yaw_rad": math.sin(phi) / cosine_theta * q + math.cos(phi) / cosine_theta * r,
        }
        ####
    ####


__all__ = ["F16AttitudeResponsePseudo6DOFModel", "F16PointMass3DOFModel"]
