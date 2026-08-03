"""Common trim/linearize adapters for the source-derived F-16 reductions.

The F-16 point-mass and pseudo-6DOF reductions already evaluate source force
channels and a source-derived local attitude-response law.  This module puts
those existing products behind the same lower-tier adapter seam used by every
other family.  It does not expose physical-effector allocation: actual
surface authority remains the separate rigid-body allocated tier.
"""

from __future__ import annotations

from collections.abc import Mapping

from ..trim import TrimResult
from .f16_reductions import F16AttitudeResponsePseudo6DOFModel, F16PointMass3DOFModel
from .reduced_control_plant import ReducedOrderControlPlant, declared_equilibrium_trim

F16_POINT_STATE_NAMES = ("u_m_s", "v_m_s", "w_m_s")
F16_PSEUDO_STATE_NAMES = (
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
F16_CONTROL_NAMES = ("elevator_deg", "aileron_deg", "rudder_deg", "throttle_fraction")


def _reduced_trim(
    *,
    state_names: tuple[str, ...],
    model: F16PointMass3DOFModel | F16AttitudeResponsePseudo6DOFModel,
    source_trim: TrimResult,
    trim_pitch_rad: float,
    target: Mapping[str, float],
    initial_guess: Mapping[str, float],
) -> TrimResult:
    """Project a verified source trim into one declared reduced-state basis."""

    state = {name: float(source_trim.state[name]) for name in F16_POINT_STATE_NAMES}
    if state_names == F16_PSEUDO_STATE_NAMES:
        state.update(
            {
                "p_rad_s": float(source_trim.state["p_rad_s"]),
                "q_rad_s": float(source_trim.state["q_rad_s"]),
                "r_rad_s": float(source_trim.state["r_rad_s"]),
                "roll_rad": 0.0,
                "pitch_rad": trim_pitch_rad,
                "yaw_rad": 0.0,
            }
        )
    for name in state_names:
        if name in target:
            state[name] = float(target[name])
    controls = {name: float(initial_guess.get(name, source_trim.controls[name])) for name in F16_CONTROL_NAMES}
    derivative = model.state_derivative(state, controls)
    residuals = {name: float(derivative[name]) for name in state_names}
    return declared_equilibrium_trim(
        state_names=state_names,
        control_names=F16_CONTROL_NAMES,
        state=state,
        controls=controls,
        residuals=residuals,
        operating_point={
            "model": "reference-f16-s119-source-reduction",
            "altitude_m": float(model.altitude_m),
            "trim_pitch_rad": trim_pitch_rad,
        },
        message="source-trim projection into the declared F-16 reduced-order state basis",
    )
    ####


class F16PointMassControlPlant(ReducedOrderControlPlant):
    """Source-force F-16 point-mass control-plant adapter."""

    model: F16PointMass3DOFModel

    def __init__(self, model: F16PointMass3DOFModel) -> None:
        self.model = model
        super().__init__(
            state_names=F16_POINT_STATE_NAMES,
            control_names=F16_CONTROL_NAMES,
            derivative_evaluator=self._derivative,
            trim_provider=self._trim,
            nonlinear_plant_id="taoryx.reference_f16_s119.point_mass_source_force",
            nonlinear_plant_revision="f16-s119-reduction-v1",
            state_units={name: "m/s" for name in F16_POINT_STATE_NAMES},
            control_units={
                "elevator_deg": "deg",
                "aileron_deg": "deg",
                "rudder_deg": "deg",
                "throttle_fraction": "1",
            },
            claim_boundary=(
                "source-force local point-mass reduction with fixed trim attitude; no physical attitude, moment, or surface-allocation claim"
            ),
            linearization_metadata={"source_trim": "reference_f16_s119", "reduction": "point_mass_3dof"},
        )
        ####

    def _derivative(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        del environment
        return self.model.state_derivative(state, controls)
        ####

    def _trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        return _reduced_trim(
            state_names=F16_POINT_STATE_NAMES,
            model=self.model,
            source_trim=self.model.trim,
            trim_pitch_rad=self.model.trim_pitch_rad,
            target=target,
            initial_guess=initial_guess,
        )
        ####
    ####


class F16Pseudo6DOFControlPlant(ReducedOrderControlPlant):
    """Source-calibrated F-16 attitude-response pseudo-6DOF adapter."""

    model: F16AttitudeResponsePseudo6DOFModel

    def __init__(self, model: F16AttitudeResponsePseudo6DOFModel) -> None:
        self.model = model
        super().__init__(
            state_names=F16_PSEUDO_STATE_NAMES,
            control_names=F16_CONTROL_NAMES,
            derivative_evaluator=self._derivative,
            trim_provider=self._trim,
            nonlinear_plant_id="taoryx.reference_f16_s119.source_calibrated_attitude_response",
            nonlinear_plant_revision="f16-s119-reduction-v1",
            state_units={
                "u_m_s": "m/s",
                "v_m_s": "m/s",
                "w_m_s": "m/s",
                "p_rad_s": "rad/s",
                "q_rad_s": "rad/s",
                "r_rad_s": "rad/s",
                "roll_rad": "rad",
                "pitch_rad": "rad",
                "yaw_rad": "rad",
            },
            control_units={
                "elevator_deg": "deg",
                "aileron_deg": "deg",
                "rudder_deg": "deg",
                "throttle_fraction": "1",
            },
            claim_boundary=(
                "source-calibrated local attitude-response pseudo-6DOF; no physical moment or surface-allocation claim"
            ),
            linearization_metadata={"source_trim": "reference_f16_s119", "reduction": "attitude_response_pseudo_6dof"},
        )
        ####

    def _derivative(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        del environment
        return self.model.state_derivative(state, controls)
        ####

    def _trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        return _reduced_trim(
            state_names=F16_PSEUDO_STATE_NAMES,
            model=self.model,
            source_trim=self.model.trim,
            trim_pitch_rad=self.model.trim_pitch_rad,
            target=target,
            initial_guess=initial_guess,
        )
        ####
    ####


def build_f16_reduced_control_plant(
    *,
    point_model: F16PointMass3DOFModel | None = None,
    pseudo_model: F16AttitudeResponsePseudo6DOFModel | None = None,
) -> ReducedOrderControlPlant:
    """Return exactly one F-16 reduction adapter with no fidelity ambiguity."""

    if (point_model is None) == (pseudo_model is None):
        raise ValueError("provide exactly one F-16 reduced model")
    if point_model is not None:
        return F16PointMassControlPlant(point_model)
    assert pseudo_model is not None
    return F16Pseudo6DOFControlPlant(pseudo_model)
    ####


__all__ = [
    "F16_CONTROL_NAMES",
    "F16_POINT_STATE_NAMES",
    "F16_PSEUDO_STATE_NAMES",
    "F16PointMassControlPlant",
    "F16Pseudo6DOFControlPlant",
    "build_f16_reduced_control_plant",
]
