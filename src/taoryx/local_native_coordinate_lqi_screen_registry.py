"""Declared Composition endpoints for local native-coordinate LQI screens."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from copy import deepcopy
from dataclasses import dataclass

from .local_native_coordinate_lqi import LocalNativeCoordinateLqiScreenConfig
from .vehicle_composition import CompiledVehicleComposition


@dataclass(frozen=True, slots=True)
class LocalNativeCoordinateLqiScreenDefinition:
    """One exact model-owned local LQI endpoint using named controls."""

    family_id: str
    mission_id: str
    fidelity: str
    initialization_id: str
    segment_id: str
    capability_adapter_id: str
    config_factory: Callable[[], LocalNativeCoordinateLqiScreenConfig]
    advertisement: Mapping[str, object]
    claim_boundary: str

    def __post_init__(self) -> None:
        """Reject incomplete static metadata without constructing a candidate."""

        required = {
            "schema",
            "id",
            "fidelity",
            "operations",
            "control_realization",
            "controller",
            "native_controls",
            "claim_boundary",
        }
        missing = sorted(required - set(self.advertisement))
        if missing:
            raise ValueError(f"local native-coordinate LQI advertisement is missing: {', '.join(missing)}")
        if self.advertisement["fidelity"] != self.fidelity:
            raise ValueError("local native-coordinate LQI advertisement fidelity must match its definition")
        if self.advertisement["control_realization"] != "native_named_coordinates":
            raise ValueError("local native-coordinate LQI advertisement must retain named-coordinate realization")
        if self.advertisement["operations"] != ["batch"]:
            raise ValueError("local native-coordinate LQI advertisements must be batch-only")
        if not self.claim_boundary.strip():
            raise ValueError("local native-coordinate LQI definitions require a claim boundary")
        ####
    ####

    def public_advertisement(self) -> dict[str, object]:
        """Return portable static screen metadata without tuning or executing."""

        return deepcopy(dict(self.advertisement))
        ####

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Return whether a compiled request selects this exact endpoint."""

        return (
            composition.family_id == self.family_id
            and composition.mission == self.mission_id
            and composition.fidelity == self.fidelity
            and composition.initialization.id == self.initialization_id
        )
        ####
    ####


def local_native_coordinate_lqi_screen_definitions() -> tuple[LocalNativeCoordinateLqiScreenDefinition, ...]:
    """Return installed model-owned native-coordinate LQI endpoints."""

    from .trajectory.a320_adapter import build_a320_local_native_coordinate_lqi_screen_config

    return (
        LocalNativeCoordinateLqiScreenDefinition(
            family_id="a320_openap_3dof",
            mission_id="a320_local_native_coordinate_lqi_screen_v1",
            fidelity="pseudo_6dof",
            initialization_id="pseudo6dof_cruise_attitude_local_point",
            segment_id="native_coordinate_attitude_lqi_screen",
            capability_adapter_id="taoryx.a320.local_native_coordinate_lqi_screen.capability.v1",
            config_factory=build_a320_local_native_coordinate_lqi_screen_config,
            advertisement={
                "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
                "id": "a320-pseudo-cruise-native-coordinate-lqi-screen-v1",
                "fidelity": "pseudo_6dof",
                "operations": ["batch"],
                "control_realization": "native_named_coordinates",
                "action_trace": "emits_committed_interval_trace_without_batch_visible_actions",
                "physical_effector_allocation": False,
                "controller": {
                    "method": "lqi",
                    "campaign_id": "a320-pseudo-cruise-attitude-v1",
                    "integral_output_names": ["bank_angle_rad"],
                    "fixed_cadence_s": 0.02,
                    "screen_duration_s": 20.0,
                },
                "native_controls": [
                    {"id": "aileron_rad", "unit": "rad", "lower": -0.2, "upper": 0.2},
                    {"id": "elevator_rad", "unit": "rad", "lower": -0.2, "upper": 0.2},
                    {"id": "rudder_rad", "unit": "rad", "lower": -0.2, "upper": 0.2},
                ],
                "assessment_state_names": [
                    "alpha_rad",
                    "beta_rad",
                    "roll_rate_rad_s",
                    "pitch_rate_rad_s",
                    "yaw_rate_rad_s",
                    "bank_angle_rad",
                ],
                "claim_boundary": (
                    "These are internally generated named response coordinates for the exact local controller "
                    "screen, not caller-owned actions, physical A320 surfaces, an allocator, or a route controller."
                ),
            },
            claim_boundary=(
                "This is one local A320 OpenAP/JSBSim surrogate-composite attitude LQI recovery at the pinned "
                "cruise point. It proves only the named aileron/elevator/rudder response coordinates, retained "
                "common-host LQI candidate, and bounded nonlinear recovery. It does not establish a physical "
                "A320 surface allocator, actuator dynamics, route guidance, wind rejection, or aircraft qualification."
            ),
        ),
    )
    ####


def resolve_local_native_coordinate_lqi_screen_definition(
    composition: CompiledVehicleComposition,
) -> LocalNativeCoordinateLqiScreenDefinition | None:
    """Resolve a screen without family or fidelity fallback."""

    return next((definition for definition in local_native_coordinate_lqi_screen_definitions() if definition.supports(composition)), None)
    ####


def resolve_local_native_coordinate_lqi_screen_advertisement(
    *,
    family_id: str | None,
    mission_id: str | None,
    fidelity: str,
) -> dict[str, object] | None:
    """Return selected static metadata without compiling a composition or tuning."""

    if family_id is None or mission_id is None:
        return None
    definition = next(
        (
            item
            for item in local_native_coordinate_lqi_screen_definitions()
            if item.family_id == family_id and item.mission_id == mission_id and item.fidelity == fidelity
        ),
        None,
    )
    return definition.public_advertisement() if definition is not None else None
    ####


__all__ = [
    "LocalNativeCoordinateLqiScreenDefinition",
    "local_native_coordinate_lqi_screen_definitions",
    "resolve_local_native_coordinate_lqi_screen_advertisement",
    "resolve_local_native_coordinate_lqi_screen_definition",
]
