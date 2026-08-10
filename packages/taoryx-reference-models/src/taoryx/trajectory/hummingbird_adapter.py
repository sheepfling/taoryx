"""Common trim/linearize adapters for Hummingbird reduced-fidelity models.

These adapters use the already executable aggregate-thrust 3DOF and named
thrust-vector pseudo-6DOF models.  They make those models available to the
same automatic trim/derivative checks as other families, but deliberately do
not expose individual rotor effectiveness or allocation.  That remains the
native rigid-body surface-allocated tier.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Literal

from ..control_automation import ControlAutomationDeclaration
from ..fidelity_contracts import FidelityTier
from ..trim import TrimResult
from ..tuning_campaign import TuningCampaign
from .hummingbird_3dof import Hummingbird3DOFModel
from .hummingbird_pseudo6dof import HummingbirdPseudo6DOFModel
from .reduced_control_plant import ReducedOrderControlPlant, declared_equilibrium_trim
from .response_laws import AxisResponseState, bounded_axis_acceleration

HummingbirdReducedTier = Literal["point_mass_3dof", "pseudo_6dof"]
HUMMINGBIRD_POINT_STATE_NAMES = (
    "north_m",
    "east_m",
    "altitude_m",
    "north_velocity_m_s",
    "east_velocity_m_s",
    "vertical_velocity_m_s",
    "battery_fraction",
)
HUMMINGBIRD_POINT_CONTROL_NAMES = ("thrust_north_n", "thrust_east_n", "thrust_up_n")
HUMMINGBIRD_PSEUDO_STATE_NAMES = (
    *HUMMINGBIRD_POINT_STATE_NAMES[:6],
    "roll_rad",
    "pitch_rad",
    "yaw_rad",
    "roll_rate_rad_s",
    "pitch_rate_rad_s",
    "yaw_rate_rad_s",
    "battery_fraction",
)
HUMMINGBIRD_PSEUDO_CONTROL_NAMES = ("roll_command_rad", "pitch_command_rad", "yaw_command_rad", "thrust_ratio")


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))
    ####


def _vector_norm(vector: tuple[float, float, float]) -> float:
    return math.sqrt(sum(value * value for value in vector))
    ####


def _target_state(
    names: tuple[str, ...],
    target: Mapping[str, float],
    defaults: Mapping[str, float],
) -> dict[str, float]:
    """Merge a declared trim target into a complete stable state schema."""

    return {name: float(target.get(name, defaults[name])) for name in names}
    ####


class HummingbirdPointMassControlPlant(ReducedOrderControlPlant):
    """Aggregate-world-thrust Hummingbird point-mass plant."""

    model: Hummingbird3DOFModel

    def __init__(self, model: Hummingbird3DOFModel | None = None) -> None:
        resolved = model or Hummingbird3DOFModel()
        self.model = resolved
        super().__init__(
            state_names=HUMMINGBIRD_POINT_STATE_NAMES,
            control_names=HUMMINGBIRD_POINT_CONTROL_NAMES,
            derivative_evaluator=self._derivative,
            trim_provider=self._trim,
            nonlinear_plant_id="taoryx.hummingbird.aggregate_thrust.point_mass",
            nonlinear_plant_revision="hummingbird-3dof-v1",
            state_units={
                "north_m": "m",
                "east_m": "m",
                "altitude_m": "m",
                "north_velocity_m_s": "m/s",
                "east_velocity_m_s": "m/s",
                "vertical_velocity_m_s": "m/s",
                "battery_fraction": "1",
            },
            control_units={name: "N" for name in HUMMINGBIRD_POINT_CONTROL_NAMES},
            claim_boundary=(
                "point-mass aggregate world-frame thrust; no attitude, individual motor, rotor, or physical allocation claim"
            ),
            linearization_metadata={"resource_equilibrium": "quasi_steady_battery_depleting_hover"},
        )
        ####

    def _derivative(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        del environment
        requested = (
            float(controls["thrust_north_n"]),
            float(controls["thrust_east_n"]),
            float(controls["thrust_up_n"]),
        )
        requested_norm = _vector_norm(requested)
        available_norm = self.model.maximum_thrust_n * _clamp(float(state["battery_fraction"]), 0.0, 1.0)
        achieved_norm = min(requested_norm, available_norm)
        ratio = achieved_norm / requested_norm if requested_norm > 0.0 else 0.0
        thrust = tuple(component * ratio for component in requested)
        return {
            "north_m": float(state["north_velocity_m_s"]),
            "east_m": float(state["east_velocity_m_s"]),
            "altitude_m": float(state["vertical_velocity_m_s"]),
            "north_velocity_m_s": thrust[0] / self.model.mass_kg,
            "east_velocity_m_s": thrust[1] / self.model.mass_kg,
            "vertical_velocity_m_s": thrust[2] / self.model.mass_kg - self.model.gravity_m_s2,
            "battery_fraction": -achieved_norm / self.model.battery_energy_j,
        }
        ####

    def _trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        defaults = {
            "north_m": 0.0,
            "east_m": 0.0,
            "altitude_m": 1.0,
            "north_velocity_m_s": 0.0,
            "east_velocity_m_s": 0.0,
            "vertical_velocity_m_s": 0.0,
            "battery_fraction": 1.0,
        }
        state = _target_state(HUMMINGBIRD_POINT_STATE_NAMES, target, defaults)
        controls = {
            "thrust_north_n": float(initial_guess.get("thrust_north_n", 0.0)),
            "thrust_east_n": float(initial_guess.get("thrust_east_n", 0.0)),
            "thrust_up_n": float(initial_guess.get("thrust_up_n", self.model.mass_kg * self.model.gravity_m_s2)),
        }
        derivative = self._derivative(state, controls, {})
        residuals = {name: derivative[name] for name in HUMMINGBIRD_POINT_STATE_NAMES[:-1]}
        return declared_equilibrium_trim(
            state_names=HUMMINGBIRD_POINT_STATE_NAMES,
            control_names=HUMMINGBIRD_POINT_CONTROL_NAMES,
            state=state,
            controls=controls,
            residuals=residuals,
            operating_point={"model": "hummingbird-aggregate-thrust", "battery_fraction": state["battery_fraction"]},
            message="analytic quasi-steady hover trim for the aggregate-thrust point-mass model",
        )
        ####
    ####


class HummingbirdPseudo6DOFControlPlant(ReducedOrderControlPlant):
    """Named aggregate-thrust-vector Hummingbird attitude-response plant."""

    model: HummingbirdPseudo6DOFModel
    thrust_frame: Literal["world_euler", "body_euler"] = "world_euler"

    def __init__(
        self,
        model: HummingbirdPseudo6DOFModel | None = None,
        *,
        thrust_frame: Literal["world_euler", "body_euler"] = "world_euler",
    ) -> None:
        resolved = model or HummingbirdPseudo6DOFModel()
        self.model = resolved
        self.thrust_frame = thrust_frame
        super().__init__(
            state_names=HUMMINGBIRD_PSEUDO_STATE_NAMES,
            control_names=HUMMINGBIRD_PSEUDO_CONTROL_NAMES,
            derivative_evaluator=self._derivative,
            trim_provider=self._trim,
            nonlinear_plant_id="taoryx.hummingbird.aggregate_thrust.attitude_response_p6dof",
            nonlinear_plant_revision=resolved.profile_id,
            state_units={
                "north_m": "m",
                "east_m": "m",
                "altitude_m": "m",
                "north_velocity_m_s": "m/s",
                "east_velocity_m_s": "m/s",
                "vertical_velocity_m_s": "m/s",
                "roll_rad": "rad",
                "pitch_rad": "rad",
                "yaw_rad": "rad",
                "roll_rate_rad_s": "rad/s",
                "pitch_rate_rad_s": "rad/s",
                "yaw_rate_rad_s": "rad/s",
                "battery_fraction": "1",
            },
            control_units={
                "roll_command_rad": "rad",
                "pitch_command_rad": "rad",
                "yaw_command_rad": "rad",
                "thrust_ratio": "1",
            },
            claim_boundary=(
                "named aggregate thrust-vector response law; no individual motor, rotor inflow, physical moment, or rotor allocation claim"
            ),
            linearization_metadata={
                "response_profile_id": resolved.profile_id,
                "thrust_frame": thrust_frame,
                "resource_equilibrium": "quasi_steady_battery_depleting_hover",
            },
        )
        ####

    def _derivative(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        del environment
        assert self.model.profile is not None
        angles = tuple(float(state[name]) for name in ("roll_rad", "pitch_rad", "yaw_rad"))
        rates = tuple(float(state[name]) for name in ("roll_rate_rad_s", "pitch_rate_rad_s", "yaw_rate_rad_s"))
        commands = tuple(float(controls[name]) for name in ("roll_command_rad", "pitch_command_rad", "yaw_command_rad"))
        profiles = (self.model.profile.response["roll"], self.model.profile.response["pitch"], self.model.profile.response["yaw"])
        angular_acceleration = tuple(
            bounded_axis_acceleration(profile, AxisResponseState(angle, rate), command)
            for profile, angle, rate, command in zip(profiles, angles, rates, commands, strict=True)
        )
        requested_ratio = _clamp(float(controls["thrust_ratio"]), 0.0, 1.0)
        available_ratio = _clamp(float(state["battery_fraction"]), 0.0, 1.0)
        thrust = requested_ratio * available_ratio * self.model.maximum_thrust_n
        roll, pitch, yaw = angles
        if self.thrust_frame == "body_euler":
            body = (
                math.sin(pitch) * math.cos(roll) * thrust,
                -math.sin(roll) * math.cos(pitch) * thrust,
                math.cos(roll) * math.cos(pitch) * thrust,
            )
            thrust_vector = (
                math.cos(yaw) * body[0] - math.sin(yaw) * body[1],
                math.sin(yaw) * body[0] + math.cos(yaw) * body[1],
                body[2],
            )
        else:
            thrust_vector = (
                math.sin(pitch) * thrust,
                -math.sin(roll) * math.cos(pitch) * thrust,
                math.cos(roll) * math.cos(pitch) * thrust,
            )
        return {
            "north_m": float(state["north_velocity_m_s"]),
            "east_m": float(state["east_velocity_m_s"]),
            "altitude_m": float(state["vertical_velocity_m_s"]),
            "north_velocity_m_s": thrust_vector[0] / self.model.mass_kg,
            "east_velocity_m_s": thrust_vector[1] / self.model.mass_kg,
            "vertical_velocity_m_s": thrust_vector[2] / self.model.mass_kg - self.model.gravity_m_s2,
            "roll_rad": rates[0],
            "pitch_rad": rates[1],
            "yaw_rad": rates[2],
            "roll_rate_rad_s": angular_acceleration[0],
            "pitch_rate_rad_s": angular_acceleration[1],
            "yaw_rate_rad_s": angular_acceleration[2],
            "battery_fraction": -thrust / self.model.battery_energy_j,
        }
        ####

    def _trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        defaults = {
            "north_m": 0.0,
            "east_m": 0.0,
            "altitude_m": 1.0,
            "north_velocity_m_s": 0.0,
            "east_velocity_m_s": 0.0,
            "vertical_velocity_m_s": 0.0,
            "roll_rad": 0.0,
            "pitch_rad": 0.0,
            "yaw_rad": 0.0,
            "roll_rate_rad_s": 0.0,
            "pitch_rate_rad_s": 0.0,
            "yaw_rate_rad_s": 0.0,
            "battery_fraction": 1.0,
        }
        state = _target_state(HUMMINGBIRD_PSEUDO_STATE_NAMES, target, defaults)
        hover_ratio = self.model.mass_kg * self.model.gravity_m_s2 / (
            self.model.maximum_thrust_n * _clamp(state["battery_fraction"], 1.0e-9, 1.0)
        )
        controls = {
            "roll_command_rad": float(initial_guess.get("roll_command_rad", state["roll_rad"])),
            "pitch_command_rad": float(initial_guess.get("pitch_command_rad", state["pitch_rad"])),
            "yaw_command_rad": float(initial_guess.get("yaw_command_rad", state["yaw_rad"])),
            "thrust_ratio": float(initial_guess.get("thrust_ratio", hover_ratio)),
        }
        derivative = self._derivative(state, controls, {})
        residual_names = HUMMINGBIRD_PSEUDO_STATE_NAMES[:-1]
        residuals = {name: derivative[name] for name in residual_names}
        return declared_equilibrium_trim(
            state_names=HUMMINGBIRD_PSEUDO_STATE_NAMES,
            control_names=HUMMINGBIRD_PSEUDO_CONTROL_NAMES,
            state=state,
            controls=controls,
            residuals=residuals,
            operating_point={
                "model": "hummingbird-aggregate-thrust-attitude-response",
                "profile_id": self.model.profile_id,
                "thrust_frame": self.thrust_frame,
            },
            message="analytic quasi-steady hover trim for the named thrust-vector response model",
        )
        ####
    ####


def build_hummingbird_reduced_control_plant(tier: FidelityTier) -> ReducedOrderControlPlant:
    """Build one common-tooling plant for a supported Hummingbird reduced tier."""

    if tier == "point_mass_3dof":
        return HummingbirdPointMassControlPlant()
    if tier == "pseudo_6dof":
        return HummingbirdPseudo6DOFControlPlant()
    raise ValueError(f"Hummingbird has no reduced control plant for {tier}")
    ####


def build_hummingbird_pseudo_tuning_campaign() -> TuningCampaign:
    """Return the declared hover attitude inner-loop design-screen campaign.

    Position, velocity, altitude, and battery state belong to outer guidance
    and resource loops.  The campaign therefore makes the six attitude/rate
    states and three semantic attitude commands explicit, while the generic
    campaign runner verifies that this projected inner loop is closed before
    synthesizing its normalized LQR candidate.
    """

    states = (
        "roll_rad",
        "pitch_rad",
        "yaw_rad",
        "roll_rate_rad_s",
        "pitch_rate_rad_s",
        "yaw_rate_rad_s",
    )
    controls = ("roll_command_rad", "pitch_command_rad", "yaw_command_rad")
    return ControlAutomationDeclaration(
        id="hummingbird-pseudo-hover-attitude",
        campaign_id="hummingbird-pseudo-hover-attitude-v1",
        family_id="hummingbird",
        tier="pseudo_6dof",
        strategy_id="multirotor_hover_translation.v1",
        node_id="hover-attitude-inner-loop",
        state_scales=dict(zip(states, (0.3, 0.3, 0.5, 1.0, 1.0, 1.0), strict=True)),
        control_scales=dict(zip(controls, (0.3, 0.3, 0.5), strict=True)),
        authority_state_names=states,
        offset_free_outputs=("roll_rad", "pitch_rad", "yaw_rad"),
        profile_grid_id_prefix="hummingbird-pseudo-hover",
    ).build_campaign()
    ####


__all__ = [
    "HUMMINGBIRD_POINT_CONTROL_NAMES",
    "HUMMINGBIRD_POINT_STATE_NAMES",
    "HUMMINGBIRD_PSEUDO_CONTROL_NAMES",
    "HUMMINGBIRD_PSEUDO_STATE_NAMES",
    "HummingbirdPointMassControlPlant",
    "HummingbirdPseudo6DOFControlPlant",
    "build_hummingbird_reduced_control_plant",
    "build_hummingbird_pseudo_tuning_campaign",
]
