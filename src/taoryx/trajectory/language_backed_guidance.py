"""Control-plant adapters for language-backed fixed-wing guidance.

The X8 and B747 language-backed point-mass packets retain an explicit native
commanded-state law: altitude integrates the committed speed and flight-path
angle, while speed, flight-path angle, and heading approach held command
coordinates at the segment guidance cadence.  Autonomous racetrack guidance
normally supplies those commands.  Composition materialization can instead
enable a bounded external guidance authority without converting the lower
fidelity into a physical surface-controlled model.

This module exposes that exact lower-tier response law to the common
trim/linearize/tuning seam.  It intentionally has no source-effector
effectiveness or allocator: source throttle/surface values remain load probes
at these realizations, and physical control belongs to their separate local
surface screens.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

from ..control_automation import ControlAutomationDeclaration
from ..trim import TrimResult
from ..tuning_campaign import TuningCampaign
from .pseudo6dof_profiles import Pseudo6DOFProfile, load_pseudo6dof_catalog
from .reduced_control_plant import ReducedOrderControlPlant, declared_equilibrium_trim
from .response_laws import bounded_axis_rate_command

GUIDANCE_STATE_NAMES = (
    "altitude_m",
    "speed_m_s",
    "flight_path_angle_deg",
    "heading_deg",
)
GUIDANCE_CONTROL_NAMES = (
    "speed_command_m_s",
    "flight_path_angle_command_deg",
    "heading_command_deg",
)
PSEUDO_GUIDANCE_STATE_NAMES = (*GUIDANCE_STATE_NAMES, "bank_deg", "pitch_deg", "yaw_deg")
PSEUDO_GUIDANCE_CONTROL_NAMES = (*GUIDANCE_CONTROL_NAMES, "bank_command_deg")
PSEUDO_GUIDANCE_DESIGN_STATE_NAMES = (*GUIDANCE_STATE_NAMES, "bank_deg")


@dataclass(frozen=True, slots=True)
class LanguageBackedGuidanceOperatingPoint:
    """One pinned level-flight point for a native lower-tier guidance law."""

    family_id: str
    id: str
    altitude_m: float
    speed_m_s: float
    heading_deg: float
    guidance_interval_s: float
    maximum_speed_m_s: float
    maximum_flight_path_angle_deg: float
    maximum_bank_deg: float
    angle_of_attack_reference_deg: float
    pseudo_response_profile_id: str

    def __post_init__(self) -> None:
        values = (
            self.altitude_m,
            self.speed_m_s,
            self.heading_deg,
            self.guidance_interval_s,
            self.maximum_speed_m_s,
            self.maximum_flight_path_angle_deg,
            self.maximum_bank_deg,
            self.angle_of_attack_reference_deg,
        )
        if not self.family_id or not self.id or not all(math.isfinite(value) for value in values):
            raise ValueError("language-backed guidance operating points require finite named values")
        if self.speed_m_s <= 0.0 or self.guidance_interval_s <= 0.0 or self.maximum_speed_m_s <= 0.0:
            raise ValueError("language-backed guidance operating points require positive speed and cadence")
        if not 0.0 <= self.heading_deg <= 360.0 or self.maximum_flight_path_angle_deg <= 0.0 or self.maximum_bank_deg <= 0.0:
            raise ValueError("language-backed guidance operating points have invalid angular bounds")
        if not self.pseudo_response_profile_id:
            raise ValueError("language-backed guidance operating points require a pseudo response profile")
        ####

    @property
    def state(self) -> dict[str, float]:
        """Return the exact native level-flight state in canonical units."""

        return {
            "altitude_m": self.altitude_m,
            "speed_m_s": self.speed_m_s,
            "flight_path_angle_deg": 0.0,
            "heading_deg": self.heading_deg,
        }
        ####

    @property
    def controls(self) -> dict[str, float]:
        """Return held external targets that preserve the level point."""

        return {
            "speed_command_m_s": self.speed_m_s,
            "flight_path_angle_command_deg": 0.0,
            "heading_command_deg": self.heading_deg,
        }
        ####


_OPERATING_POINTS = {
    "skywalker_x8": LanguageBackedGuidanceOperatingPoint(
        family_id="skywalker_x8",
        id="x8-racetrack-level-guidance-v1",
        altitude_m=178.0,
        speed_m_s=17.9,
        heading_deg=90.0,
        guidance_interval_s=0.05,
        maximum_speed_m_s=27.0,
        maximum_flight_path_angle_deg=20.0,
        maximum_bank_deg=45.0,
        angle_of_attack_reference_deg=7.8095001441,
        pseudo_response_profile_id="skywalker_x8.attitude_response_p6dof.v1",
    ),
    "b747": LanguageBackedGuidanceOperatingPoint(
        family_id="b747",
        id="b747-condition3-racetrack-level-guidance-v1",
        altitude_m=100.0,
        speed_m_s=153.0,
        heading_deg=90.0,
        guidance_interval_s=0.05,
        maximum_speed_m_s=220.0,
        maximum_flight_path_angle_deg=10.0,
        maximum_bank_deg=30.0,
        angle_of_attack_reference_deg=3.1,
        pseudo_response_profile_id="b747.attitude_response_p6dof.v1",
    ),
}


def language_backed_guidance_operating_point(family_id: str) -> LanguageBackedGuidanceOperatingPoint:
    """Return the source-problem level point for one supported family."""

    try:
        return _OPERATING_POINTS[family_id]
    except KeyError as error:
        raise ValueError(f"language-backed guidance has no declared operating point for {family_id!r}") from error
    ####


def language_backed_pseudo_guidance_profile(family_id: str) -> Pseudo6DOFProfile:
    """Return the declared response-law profile paired with one guidance point."""

    point = language_backed_guidance_operating_point(family_id)
    profile = next(
        (item for item in load_pseudo6dof_catalog().profiles if item.id == point.pseudo_response_profile_id),
        None,
    )
    if profile is None or profile.family_id != family_id:
        raise ValueError(f"language-backed pseudo guidance has no matching profile for {family_id!r}")
    return profile
    ####


class LanguageBackedGuidanceControlPlant(ReducedOrderControlPlant):
    """Exact local control adapter for native point-mass commanded states."""

    operating_point: LanguageBackedGuidanceOperatingPoint

    def __init__(self, family_id: str) -> None:
        self.operating_point = language_backed_guidance_operating_point(family_id)
        point = self.operating_point
        super().__init__(
            state_names=GUIDANCE_STATE_NAMES,
            control_names=GUIDANCE_CONTROL_NAMES,
            derivative_evaluator=self._derivative,
            trim_provider=self._trim,
            nonlinear_plant_id="taoryx.language_backed_fixed_wing.native_guidance_state_rate",
            nonlinear_plant_revision="language-backed-guidance-v1",
            state_units={
                "altitude_m": "m",
                "speed_m_s": "m/s",
                "flight_path_angle_deg": "deg",
                "heading_deg": "deg",
            },
            control_units={
                "speed_command_m_s": "m/s",
                "flight_path_angle_command_deg": "deg",
                "heading_command_deg": "deg",
            },
            claim_boundary=(
                f"{family_id} native language-backed point-mass commanded-state response at {point.id}; "
                "external speed, flight-path, and heading targets replace autonomous route targets only when "
                "explicitly enabled, with no physical surface, actuator, or moment-balance claim"
            ),
            linearization_metadata={
                "family_id": family_id,
                "operating_point": point.id,
                "guidance_interval_s": point.guidance_interval_s,
                "native_runtime_law": "_runtime_point_mass_route_commands",
            },
        )
        ####

    def _derivative(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        """Evaluate the same held-command law as native composition lowering."""

        del environment
        speed = float(state["speed_m_s"])
        gamma_deg = float(state["flight_path_angle_deg"])
        heading_deg = float(state["heading_deg"])
        speed_command = float(controls["speed_command_m_s"])
        gamma_command = float(controls["flight_path_angle_command_deg"])
        heading_command = float(controls["heading_command_deg"])
        interval = self.operating_point.guidance_interval_s
        return {
            "altitude_m": speed * math.sin(math.radians(gamma_deg)),
            "speed_m_s": (speed_command - speed) / interval,
            "flight_path_angle_deg": (gamma_command - gamma_deg) / interval,
            "heading_deg": _wrapped_degrees(heading_command - heading_deg) / interval,
        }
        ####

    def _trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        """Return the declared level equilibrium of the native command law."""

        point = self.operating_point
        state = {name: float(target.get(name, value)) for name, value in point.state.items()}
        controls = {
            "speed_command_m_s": float(initial_guess.get("speed_command_m_s", state["speed_m_s"])),
            "flight_path_angle_command_deg": float(initial_guess.get("flight_path_angle_command_deg", state["flight_path_angle_deg"])),
            "heading_command_deg": float(initial_guess.get("heading_command_deg", state["heading_deg"])),
        }
        residuals = self._derivative(state, controls, {})
        return declared_equilibrium_trim(
            state_names=GUIDANCE_STATE_NAMES,
            control_names=GUIDANCE_CONTROL_NAMES,
            residuals=residuals,
            state=state,
            controls=controls,
            operating_point={
                "family_id": point.family_id,
                "operating_point_id": point.id,
                "guidance_interval_s": point.guidance_interval_s,
                "authority": "explicit_kinematic_guidance",
            },
            message="native language-backed level commanded-state equilibrium",
        )
        ####

    ####


class LanguageBackedPseudoGuidanceControlPlant(ReducedOrderControlPlant):
    """Local adapter for coupled point-mass guidance and kinematic attitude response."""

    operating_point: LanguageBackedGuidanceOperatingPoint
    response_profile: Pseudo6DOFProfile

    def __init__(self, family_id: str) -> None:
        self.operating_point = language_backed_guidance_operating_point(family_id)
        self.response_profile = language_backed_pseudo_guidance_profile(family_id)
        point = self.operating_point
        super().__init__(
            state_names=PSEUDO_GUIDANCE_STATE_NAMES,
            control_names=PSEUDO_GUIDANCE_CONTROL_NAMES,
            derivative_evaluator=self._derivative,
            trim_provider=self._trim,
            nonlinear_plant_id="taoryx.language_backed_fixed_wing.native_guidance_kinematic_response",
            nonlinear_plant_revision="language-backed-pseudo-guidance-v1",
            state_units={
                **LanguageBackedGuidanceControlPlant(family_id).state_units,
                "bank_deg": "deg",
                "pitch_deg": "deg",
                "yaw_deg": "deg",
            },
            control_units={
                **LanguageBackedGuidanceControlPlant(family_id).control_units,
                "bank_command_deg": "deg",
            },
            claim_boundary=(
                f"{family_id} native language-backed point-mass commanded-state response coupled to the declared "
                f"{self.response_profile.id} kinematic attitude profile at {point.id}; external targets replace "
                "autonomous route and sidecar targets only when explicitly enabled, with no physical surface, actuator, or moment-balance claim"
            ),
            linearization_metadata={
                "family_id": family_id,
                "operating_point": point.id,
                "guidance_interval_s": point.guidance_interval_s,
                "native_runtime_law": "_runtime_point_mass_route_commands + _build_kinematic_body_rate_provider",
                "pseudo_response_profile_id": self.response_profile.id,
                "pseudo_response_evidence_grade": self.response_profile.evidence_grade,
            },
        )
        ####

    def _derivative(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        """Evaluate the native command law and declared bounded attitude profile."""

        del environment
        point = self.operating_point
        interval = point.guidance_interval_s
        speed_command = float(controls["speed_command_m_s"])
        gamma_command = float(controls["flight_path_angle_command_deg"])
        heading_command = float(controls["heading_command_deg"])
        bank_command = float(controls["bank_command_deg"])
        bank_deg = float(state["bank_deg"])
        pitch_deg = float(state["pitch_deg"])
        yaw_deg = float(state["yaw_deg"])
        return {
            "altitude_m": float(state["speed_m_s"]) * math.sin(math.radians(float(state["flight_path_angle_deg"]))),
            "speed_m_s": (speed_command - float(state["speed_m_s"])) / interval,
            "flight_path_angle_deg": (gamma_command - float(state["flight_path_angle_deg"])) / interval,
            "heading_deg": _wrapped_degrees(heading_command - float(state["heading_deg"])) / interval,
            "bank_deg": math.degrees(
                bounded_axis_rate_command(self.response_profile.response["roll"], math.radians(_wrapped_degrees(bank_command - bank_deg)))
            ),
            "pitch_deg": math.degrees(
                bounded_axis_rate_command(
                    self.response_profile.response["pitch"],
                    math.radians(_wrapped_degrees(gamma_command + point.angle_of_attack_reference_deg - pitch_deg)),
                )
            ),
            "yaw_deg": math.degrees(
                bounded_axis_rate_command(self.response_profile.response["yaw"], math.radians(_wrapped_degrees(heading_command - yaw_deg)))
            ),
        }
        ####

    def _trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        """Return the pinned route equilibrium and profile-aligned sidecar target."""

        point = self.operating_point
        state = {
            **{name: float(target.get(name, value)) for name, value in point.state.items()},
            "bank_deg": float(target.get("bank_deg", 0.0)),
            "pitch_deg": float(target.get("pitch_deg", point.angle_of_attack_reference_deg)),
            "yaw_deg": float(target.get("yaw_deg", point.heading_deg)),
        }
        controls = {
            "speed_command_m_s": float(initial_guess.get("speed_command_m_s", state["speed_m_s"])),
            "flight_path_angle_command_deg": float(initial_guess.get("flight_path_angle_command_deg", state["flight_path_angle_deg"])),
            "heading_command_deg": float(initial_guess.get("heading_command_deg", state["heading_deg"])),
            "bank_command_deg": float(initial_guess.get("bank_command_deg", state["bank_deg"])),
        }
        residuals = self._derivative(state, controls, {})
        return declared_equilibrium_trim(
            state_names=PSEUDO_GUIDANCE_STATE_NAMES,
            control_names=PSEUDO_GUIDANCE_CONTROL_NAMES,
            residuals=residuals,
            state=state,
            controls=controls,
            operating_point={
                "family_id": point.family_id,
                "operating_point_id": point.id,
                "guidance_interval_s": point.guidance_interval_s,
                "authority": "explicit_kinematic_guidance_with_profile_sidecar",
                "pseudo_response_profile_id": self.response_profile.id,
            },
            message="native language-backed pseudo-6DOF guidance and sidecar equilibrium",
        )
        ####

    ####


def build_language_backed_guidance_tuning_campaign(family_id: str) -> TuningCampaign:
    """Return the shared local LQI screen for one native lower-tier route law."""

    point = language_backed_guidance_operating_point(family_id)
    prefix = "x8" if family_id == "skywalker_x8" else "b747"
    return ControlAutomationDeclaration(
        id=f"{prefix}-language-backed-guidance",
        campaign_id=f"{prefix}-language-backed-guidance-local-lqi-v1",
        family_id=family_id,
        tier="point_mass_3dof",
        strategy_id="powered_fixed_wing.native_guidance.v1",
        node_id=point.id,
        state_scales={
            "altitude_m": 10.0 if family_id == "skywalker_x8" else 100.0,
            "speed_m_s": 5.0 if family_id == "skywalker_x8" else 25.0,
            "flight_path_angle_deg": 3.0,
            "heading_deg": 10.0,
        },
        control_scales={
            "speed_command_m_s": 5.0 if family_id == "skywalker_x8" else 25.0,
            "flight_path_angle_command_deg": 3.0,
            "heading_command_deg": 10.0,
        },
        trim_target=point.state,
        trim_initial_guess=point.controls,
        authority_state_names=GUIDANCE_STATE_NAMES,
        offset_free_outputs=("altitude_m", "speed_m_s", "heading_deg"),
        preferred_method="lqi",
        profile_grid_id_prefix=f"{prefix}-language-guidance",
        integral_weight_multiplier=8.0,
    ).build_campaign()
    ####


def build_language_backed_pseudo_guidance_tuning_campaign(family_id: str) -> TuningCampaign:
    """Return the local LQI screen for the profile-backed pseudo-6DOF bridge."""

    point = language_backed_guidance_operating_point(family_id)
    prefix = "x8" if family_id == "skywalker_x8" else "b747"
    return ControlAutomationDeclaration(
        id=f"{prefix}-language-backed-pseudo-guidance",
        campaign_id=f"{prefix}-language-backed-pseudo-guidance-local-lqi-v1",
        family_id=family_id,
        tier="pseudo_6dof",
        strategy_id="powered_fixed_wing.native_guidance_kinematic_sidecar.v1",
        node_id=f"{point.id}-pseudo-6dof",
        state_scales={
            "altitude_m": 10.0 if family_id == "skywalker_x8" else 100.0,
            "speed_m_s": 5.0 if family_id == "skywalker_x8" else 25.0,
            "flight_path_angle_deg": 3.0,
            "heading_deg": 10.0,
            "bank_deg": 10.0,
            "pitch_deg": 5.0,
            "yaw_deg": 10.0,
        },
        control_scales={
            "speed_command_m_s": 5.0 if family_id == "skywalker_x8" else 25.0,
            "flight_path_angle_command_deg": 3.0,
            "heading_command_deg": 10.0,
            "bank_command_deg": 10.0,
        },
        trim_target={
            **point.state,
            "bank_deg": 0.0,
            "pitch_deg": point.angle_of_attack_reference_deg,
            "yaw_deg": point.heading_deg,
        },
        trim_initial_guess={**point.controls, "bank_command_deg": 0.0},
        # Pitch and yaw are observed sidecar response states rather than
        # independently commanded axes: pitch follows the flight-path target
        # plus the declared angle-of-attack reference, and yaw follows the
        # heading target.  Keeping them out of the LQI state removes a false
        # full-authority claim while preserving the dynamically closed,
        # directly controllable guidance-plus-bank design subsystem.
        design_state_names=PSEUDO_GUIDANCE_DESIGN_STATE_NAMES,
        authority_state_names=PSEUDO_GUIDANCE_DESIGN_STATE_NAMES,
        offset_free_outputs=("altitude_m", "speed_m_s", "heading_deg", "bank_deg"),
        preferred_method="lqi",
        profile_grid_id_prefix=f"{prefix}-language-pseudo-guidance",
        integral_weight_multiplier=8.0,
    ).build_campaign()
    ####


def _wrapped_degrees(value: float) -> float:
    """Return the native shortest heading error in degrees."""

    return (value + 180.0) % 360.0 - 180.0
    ####


__all__ = [
    "GUIDANCE_CONTROL_NAMES",
    "GUIDANCE_STATE_NAMES",
    "LanguageBackedGuidanceControlPlant",
    "LanguageBackedGuidanceOperatingPoint",
    "LanguageBackedPseudoGuidanceControlPlant",
    "PSEUDO_GUIDANCE_CONTROL_NAMES",
    "PSEUDO_GUIDANCE_DESIGN_STATE_NAMES",
    "PSEUDO_GUIDANCE_STATE_NAMES",
    "build_language_backed_guidance_tuning_campaign",
    "build_language_backed_pseudo_guidance_tuning_campaign",
    "language_backed_guidance_operating_point",
    "language_backed_pseudo_guidance_profile",
]
