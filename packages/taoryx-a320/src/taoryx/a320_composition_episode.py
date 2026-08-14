"""A320/OpenAP binding of the shared reduced-flight Composition episode contract."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any, Literal, cast

from taoryx_a320.resources import model_resource_root

from taoryx.composition_episode import (
    EpisodeChannel,
    ReducedFixedWingCompositionEpisode,
    VehicleCompositionEpisode,
)
from taoryx.mission_capability import compile_powered_fixed_wing_racetrack_from_composition
from taoryx.trajectory.pseudo6dof_profiles import load_pseudo6dof_catalog
from taoryx.vehicle_composition import CompiledVehicleComposition

from .a320_reduced_execution import _a320_operating_point_for_speed
from .trajectory.a320_openap import A320OpenAPModel
from .trajectory.a320_pseudo6dof import A320Pseudo6DOFModel
from .trajectory.a320_racetrack import (
    A320GuidanceOverride,
    A320RacetrackRunner,
    A320RacetrackStepper,
    A320RacetrackStepperState,
)


class A320ReducedCompositionEpisode(ReducedFixedWingCompositionEpisode):
    """Accepted-truth A320 episode with reduced guidance-only authority."""

    claim_boundary = (
        "This episode uses the declared OpenAP A320 reduced-flight stepper. It accepts bounded kinematic guidance "
        "through the point-mass or named route-lag response law; it does not establish physical surface allocation, "
        "actuator dynamics, moment balance, batch/episode parity, or mission qualification."
    )

    _ACTION_SCHEMA = (
        EpisodeChannel("speed_m_s", "m/s", 50.0, 300.0, "held reduced-model speed target"),
        EpisodeChannel("flight_path_angle_deg", "deg", -20.0, 20.0, "held reduced-model flight-path target"),
        EpisodeChannel("heading_deg", "deg", 0.0, 360.0, "held local-navigation heading target"),
        EpisodeChannel("bank_angle_deg", "deg", -60.0, 60.0, "held response-law bank or lift-vector target"),
    )

    _FAMILY_ID = "a320_openap_3dof"
    _FAMILY_LABEL = "A320"
    _CHECKPOINT_SCHEMA = "taoryx.a320-reduced-composition-episode/v1alpha1"
    _MAXIMUM_SPEED_M_S = 300.0

    def _build_stepper(self, composition: CompiledVehicleComposition) -> A320RacetrackStepper:
        return _a320_stepper_for_composition(composition)
        ####

    @staticmethod
    def _guidance_override(action: Mapping[str, float]) -> A320GuidanceOverride:
        return _a320_guidance_override(action)
        ####

    @staticmethod
    def _status_values(row: Mapping[str, object]) -> dict[str, object]:
        return _a320_status_values(row)
        ####

    @staticmethod
    def _serialize_stepper_state(snapshot: A320RacetrackStepperState) -> dict[str, object]:
        return {
            "time_s": snapshot.time_s,
            "north_m": snapshot.north_m,
            "east_m": snapshot.east_m,
            "state": dict(snapshot.state),
            "controls": dict(snapshot.controls),
            "terminal_hold": snapshot.terminal_hold,
            "numerical_valid": snapshot.numerical_valid,
            "failure": snapshot.failure,
        }
        ####

    @staticmethod
    def _deserialize_stepper_state(payload: Mapping[str, object]) -> A320RacetrackStepperState:
        return _a320_stepper_state(payload)
        ####

    ####


def open_a320_reduced_composition_episode(
    composition: CompiledVehicleComposition,
    seed: int | None,
    integration_step_s: float,
) -> VehicleCompositionEpisode:
    """Open the A320-owned reduced episode through the plug-in registry."""

    del integration_step_s
    return A320ReducedCompositionEpisode(composition, seed=seed)
    ####


def _a320_stepper_for_composition(composition: CompiledVehicleComposition) -> A320RacetrackStepper:
    """Build the exact A320 reduced plant selected by one composition."""

    route = compile_powered_fixed_wing_racetrack_from_composition(composition).route
    inputs = composition.initialization.inputs
    altitude_m = _composition_number(inputs, "altitude_m", default=6000.0)
    mass_kg = _composition_number(inputs, "mass_kg", default=60000.0)
    root = model_resource_root()
    base_model = A320OpenAPModel.from_repository(root)
    operating_point = _a320_operating_point_for_speed(base_model, altitude_m, mass_kg, route.speed_m_s)
    model: A320OpenAPModel | A320Pseudo6DOFModel
    mode: Literal["point_mass_3dof", "pseudo_6dof_kinematic_bridge"]
    if composition.fidelity == "point_mass_3dof":
        model = base_model
        trim = model.trim_level_flight(operating_point)
        response_profile = None
        mode = "point_mass_3dof"
    elif composition.fidelity == "pseudo_6dof":
        model = A320Pseudo6DOFModel.from_repository(root)
        trim = model.trim_pseudo6dof(operating_point)
        _, response_profile = load_pseudo6dof_catalog(model_resource_root() / "verification/pseudo6dof_profiles.yaml").for_family(
            "a320_openap_3dof"
        )
        mode = "pseudo_6dof_kinematic_bridge"
    else:
        raise ValueError(f"A320 reduced episode has no fidelity adapter for {composition.fidelity!r}")
    return A320RacetrackStepper(
        A320RacetrackRunner(
            model,
            trim,
            route,
            mode,
            dt_s=0.2,
            response_profile=response_profile,
            operating_point=operating_point,
        )
    )
    ####


def _a320_guidance_override(action: Mapping[str, float]) -> A320GuidanceOverride:
    """Translate held public/native A320 guidance coordinates into SI/radians."""

    return A320GuidanceOverride(
        speed_m_s=action.get("speed_m_s"),
        flight_path_angle_rad=(None if "flight_path_angle_deg" not in action else math.radians(action["flight_path_angle_deg"])),
        heading_rad=None if "heading_deg" not in action else math.radians(action["heading_deg"]),
        bank_angle_rad=None if "bank_angle_deg" not in action else math.radians(action["bank_angle_deg"]),
    )
    ####


def _a320_status_values(row: Mapping[str, object]) -> dict[str, object]:
    """Enrich one reduced telemetry row with only declared portable vectors."""

    values = dict(row)
    attitude_keys = ("route_bank_achieved_deg", "route_pitch_achieved_deg", "route_heading_achieved_deg")
    rate_keys = ("p_rad_s", "q_rad_s", "r_rad_s")
    if all(key in values for key in attitude_keys):
        values["attitude_euler_deg"] = [values[key] for key in attitude_keys]
    if all(key in values for key in rate_keys):
        values["body_rate_rad_s"] = [values[key] for key in rate_keys]
    values["control_realization"] = (
        "response_law" if values.get("control_path") == "pseudo_6dof_kinematic_bridge" else "force_model"
    )
    return values
    ####


def _a320_stepper_state(payload: Mapping[str, object]) -> A320RacetrackStepperState:
    """Decode a checkpointed A320 stepper state with no permissive coercion."""

    state = payload.get("state")
    controls = payload.get("controls")
    if not isinstance(state, Mapping) or not isinstance(controls, Mapping):
        raise ValueError("A320 checkpoint stepper state has no scalar state/control mappings")
    scalar_state = {str(name): _finite_number(value, f"A320 checkpoint state {name!r}") for name, value in state.items()}
    scalar_controls = {str(name): _finite_number(value, f"A320 checkpoint control {name!r}") for name, value in controls.items()}
    terminal_hold = payload.get("terminal_hold")
    numerical_valid = payload.get("numerical_valid")
    failure = payload.get("failure")
    if not isinstance(terminal_hold, bool) or not isinstance(numerical_valid, bool):
        raise ValueError("A320 checkpoint has invalid terminal or numerical status")
    if failure is not None and not isinstance(failure, str):
        raise ValueError("A320 checkpoint failure must be string or null")
    return A320RacetrackStepperState(
        _finite_number(payload.get("time_s"), "A320 checkpoint time_s"),
        _finite_number(payload.get("north_m"), "A320 checkpoint north_m"),
        _finite_number(payload.get("east_m"), "A320 checkpoint east_m"),
        scalar_state,
        scalar_controls,
        terminal_hold,
        numerical_valid,
        failure,
    )
    ####


def _composition_number(values: Mapping[str, Any], name: str, *, default: float) -> float:
    value = values.get(name)
    return default if value is None else float(value.value)
    ####


def _finite_number(value: object, label: str) -> float:
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


__all__ = ["A320ReducedCompositionEpisode", "open_a320_reduced_composition_episode"]
