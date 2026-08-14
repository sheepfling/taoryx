"""F-16 binding of the shared reduced-flight Composition episode contract."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any, Literal, cast

from taoryx_f16.resources import model_resource_root

from taoryx.composition_episode import (
    EpisodeChannel,
    ReducedFixedWingCompositionEpisode,
    VehicleCompositionEpisode,
)
from taoryx.trajectory.pseudo6dof_profiles import load_pseudo6dof_catalog
from taoryx.vehicle_composition import CompiledVehicleComposition

from .f16_mission_translation import compile_f16_powered_fixed_wing_racetrack_from_composition
from .f16_reduced_execution import _f16_source_trim
from .trajectory.f16_reduced_racetrack import (
    F16GuidanceOverride,
    F16ReducedRacetrackRunner,
    F16ReducedRacetrackStepper,
    F16ReducedRacetrackStepperState,
)
from .trajectory.f16_reductions import F16AttitudeResponsePseudo6DOFModel, F16PointMass3DOFModel


def _composition_number(values: Mapping[str, Any], name: str, *, default: float) -> float:
    """Read one optional F-16 composition input without relying on core internals."""

    value = values.get(name)
    return default if value is None else float(value.value)
    ####


def _finite_number(value: object, label: str) -> float:
    """Return one finite checkpoint number with a F-16-owned error boundary."""

    if isinstance(value, bool) or value is None:
        raise ValueError(f"{label} must be finite numeric")
    try:
        numeric = float(cast(Any, value))
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} must be finite numeric") from error
    if not math.isfinite(numeric):
        raise ValueError(f"{label} must be finite numeric")
    return numeric
    ####


def _f16_reduced_body_rate_episode_channels() -> tuple[EpisodeChannel, ...]:
    """Return F-16 pseudo-6DOF body-rate reference coordinates."""

    return (
        EpisodeChannel("body-roll-rate-command-rad-s", "rad/s", -0.5, 0.5, "held body roll-rate reference p"),
        EpisodeChannel("body-pitch-rate-command-rad-s", "rad/s", -0.35, 0.35, "held body pitch-rate reference q"),
        EpisodeChannel("body-yaw-rate-command-rad-s", "rad/s", -0.35, 0.35, "held body yaw-rate reference r"),
    )
    ####


class F16ReducedCompositionEpisode(ReducedFixedWingCompositionEpisode):
    """Accepted-truth F-16 episode with explicit lower-tier authority modes.

    It publishes reduced guidance and body-rate modes only.  It does not
    expose source surface coordinates, physical moment commands, or actuator
    allocation through the reduced model.
    """

    claim_boundary = (
        "This episode uses the declared source-owned F-16 reduced-flight stepper. It accepts bounded kinematic "
        "guidance through the point-mass or named attitude/rate response law; it does not establish physical "
        "surface allocation, actuator dynamics, moment balance, batch/episode parity, or mission qualification."
    )

    _ACTION_SCHEMA = (
        EpisodeChannel("speed_m_s", "m/s", 50.0, 500.0, "held reduced-model speed target"),
        EpisodeChannel("flight_path_angle_deg", "deg", -20.0, 20.0, "held reduced-model flight-path target"),
        EpisodeChannel("heading_deg", "deg", 0.0, 360.0, "held local-navigation heading target"),
        EpisodeChannel("bank_angle_deg", "deg", -60.0, 60.0, "held response-law bank or lift-vector target"),
    )

    _FAMILY_ID = "f16_s119"
    _FAMILY_LABEL = "F-16"
    _CHECKPOINT_SCHEMA = "taoryx.f16-reduced-composition-episode/v1alpha1"
    _MAXIMUM_SPEED_M_S = 500.0

    def _additional_authority_action_schema(self) -> tuple[EpisodeChannel, ...]:
        return _f16_reduced_body_rate_episode_channels() if self.composition.fidelity == "pseudo_6dof" else ()
        ####

    @staticmethod
    def _status_values(row: Mapping[str, object]) -> dict[str, object]:
        """Add scalar F-16 rate readback beside the portable rate vector.

        The generic reduced contract publishes ``body_rate`` as a vector.  The
        F-16 body-rate authority needs axis-specific feedback channels so a
        streaming client can distinguish an accepted reference from the
        response actually achieved by the pseudo-6DOF law.
        """

        values = ReducedFixedWingCompositionEpisode._status_values(row)
        rates = values.get("body_rate_rad_s")
        if isinstance(rates, list | tuple) and len(rates) == 3:
            values["body_rate.roll"] = rates[0]
            values["body_rate.pitch"] = rates[1]
            values["body_rate.yaw"] = rates[2]
        return values
        ####

    def _build_stepper(self, composition: CompiledVehicleComposition) -> F16ReducedRacetrackStepper:
        return _f16_stepper_for_composition(composition)
        ####

    @staticmethod
    def _guidance_override(action: Mapping[str, float]) -> F16GuidanceOverride:
        return _f16_guidance_override(action)
        ####

    @staticmethod
    def _serialize_stepper_state(snapshot: F16ReducedRacetrackStepperState) -> dict[str, object]:
        return {
            "time_s": snapshot.time_s,
            "north_m": snapshot.north_m,
            "east_m": snapshot.east_m,
            "altitude_m": snapshot.altitude_m,
            "speed_m_s": snapshot.speed_m_s,
            "heading_rad": snapshot.heading_rad,
            "flight_path_angle_rad": snapshot.flight_path_angle_rad,
            "roll_rad": snapshot.roll_rad,
            "pitch_rad": snapshot.pitch_rad,
            "yaw_rad": snapshot.yaw_rad,
            "p_rad_s": snapshot.p_rad_s,
            "q_rad_s": snapshot.q_rad_s,
            "r_rad_s": snapshot.r_rad_s,
            "controls": dict(snapshot.controls),
            "numerical_valid": snapshot.numerical_valid,
            "failure": snapshot.failure,
        }
        ####

    @staticmethod
    def _deserialize_stepper_state(payload: Mapping[str, object]) -> F16ReducedRacetrackStepperState:
        return _f16_stepper_state(payload)
        ####

    ####


def open_f16_reduced_composition_episode(
    composition: CompiledVehicleComposition,
    seed: int | None,
    integration_step_s: float,
) -> VehicleCompositionEpisode:
    """Open a package-owned F-16 reduced episode through the plug-in registry."""

    del integration_step_s
    return F16ReducedCompositionEpisode(composition, seed=seed)
    ####


def _f16_stepper_for_composition(composition: CompiledVehicleComposition) -> F16ReducedRacetrackStepper:
    """Build the exact source-trimmed F-16 reduction selected by a composition."""

    route = compile_f16_powered_fixed_wing_racetrack_from_composition(composition).route
    source, trim, trim_pitch_rad = _f16_source_trim()
    observed_mach = float(source.evaluate_loads(trim.state, trim.controls, altitude_m=0.0)["mach"])
    requested_mach = _composition_number(composition.initialization.inputs, "mach", default=observed_mach)
    if not math.isclose(requested_mach, observed_mach, abs_tol=1.0e-9):
        raise ValueError(
            f"F-16 reduced episode is pinned to its source trim Mach; requested {requested_mach:.12g}, "
            f"expected {observed_mach:.12g}"
        )
    model: F16PointMass3DOFModel | F16AttitudeResponsePseudo6DOFModel
    mode: Literal["point_mass_3dof", "pseudo_6dof_kinematic_bridge"]
    if composition.fidelity == "point_mass_3dof":
        model = F16PointMass3DOFModel(source, trim, trim_pitch_rad)
        response_profile = None
        mode = "point_mass_3dof"
    elif composition.fidelity == "pseudo_6dof":
        linearization = source.linearize_local(
            trim.state,
            trim.controls,
            trim_pitch_rad=trim_pitch_rad,
            altitude_m=0.0,
            state_step=1.0e-5,
            control_step=1.0e-5,
        )
        model = F16AttitudeResponsePseudo6DOFModel(source, trim, linearization, trim_pitch_rad)
        _, response_profile = load_pseudo6dof_catalog(model_resource_root() / "verification/pseudo6dof_profiles.yaml").for_family(
            "f16_s119"
        )
        mode = "pseudo_6dof_kinematic_bridge"
    else:
        raise ValueError(f"F-16 reduced episode has no fidelity adapter for {composition.fidelity!r}")
    return F16ReducedRacetrackStepper(
        F16ReducedRacetrackRunner(model, trim, route, mode, dt_s=0.2, response_profile=response_profile)
    )
    ####


def _f16_guidance_override(action: Mapping[str, float]) -> F16GuidanceOverride:
    """Translate held F-16 guidance coordinates into SI/radians."""

    return F16GuidanceOverride(
        speed_m_s=action.get("speed_m_s"),
        flight_path_angle_rad=(None if "flight_path_angle_deg" not in action else math.radians(action["flight_path_angle_deg"])),
        heading_rad=None if "heading_deg" not in action else math.radians(action["heading_deg"]),
        bank_angle_rad=None if "bank_angle_deg" not in action else math.radians(action["bank_angle_deg"]),
    )
    ####


def _f16_stepper_state(payload: Mapping[str, object]) -> F16ReducedRacetrackStepperState:
    """Decode a checkpointed F-16 stepper state with no permissive coercion."""

    controls = payload.get("controls")
    if not isinstance(controls, Mapping):
        raise ValueError("F-16 checkpoint stepper state has no control mapping")
    scalar_controls = {str(name): _finite_number(value, f"F-16 checkpoint control {name!r}") for name, value in controls.items()}
    numerical_valid = payload.get("numerical_valid")
    failure = payload.get("failure")
    if not isinstance(numerical_valid, bool):
        raise ValueError("F-16 checkpoint has invalid numerical status")
    if failure is not None and not isinstance(failure, str):
        raise ValueError("F-16 checkpoint failure must be string or null")
    return F16ReducedRacetrackStepperState(
        time_s=_finite_number(payload.get("time_s"), "F-16 checkpoint time_s"),
        north_m=_finite_number(payload.get("north_m"), "F-16 checkpoint north_m"),
        east_m=_finite_number(payload.get("east_m"), "F-16 checkpoint east_m"),
        altitude_m=_finite_number(payload.get("altitude_m"), "F-16 checkpoint altitude_m"),
        speed_m_s=_finite_number(payload.get("speed_m_s"), "F-16 checkpoint speed_m_s"),
        heading_rad=_finite_number(payload.get("heading_rad"), "F-16 checkpoint heading_rad"),
        flight_path_angle_rad=_finite_number(
            payload.get("flight_path_angle_rad"),
            "F-16 checkpoint flight_path_angle_rad",
        ),
        roll_rad=_finite_number(payload.get("roll_rad"), "F-16 checkpoint roll_rad"),
        pitch_rad=_finite_number(payload.get("pitch_rad"), "F-16 checkpoint pitch_rad"),
        yaw_rad=_finite_number(payload.get("yaw_rad"), "F-16 checkpoint yaw_rad"),
        p_rad_s=_finite_number(payload.get("p_rad_s"), "F-16 checkpoint p_rad_s"),
        q_rad_s=_finite_number(payload.get("q_rad_s"), "F-16 checkpoint q_rad_s"),
        r_rad_s=_finite_number(payload.get("r_rad_s"), "F-16 checkpoint r_rad_s"),
        controls=scalar_controls,
        numerical_valid=numerical_valid,
        failure=failure,
    )
    ####


__all__ = ["F16ReducedCompositionEpisode", "open_f16_reduced_composition_episode"]
