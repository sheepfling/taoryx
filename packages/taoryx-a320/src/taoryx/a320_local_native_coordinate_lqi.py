"""A320-owned declaration for its native-coordinate local LQI screen.

This module contains only immutable endpoint metadata and one lazy factory.
It is safe to import during plug-in discovery: the OpenAP/JSBSim surrogate
plant and the retained controller candidate are constructed only when the
selected batch or preflight path calls ``config_factory``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from taoryx.local_controller_screen_advertisements import LocalControllerScreenAdvertisement
from taoryx.local_native_coordinate_lqi_screen_registry import LocalNativeCoordinateLqiScreenDefinition

if TYPE_CHECKING:
    from taoryx.local_native_coordinate_lqi import LocalNativeCoordinateLqiScreenConfig

_AGGREGATE_PROVIDER_ID = "taoryx.registry.mission-composition"
_PROVIDER_ID = "taoryx.a320.mission-composition"
_FAMILY_ID = "a320_openap_3dof"
_MISSION_ID = "a320_local_native_coordinate_lqi_screen_v1"
_FIDELITY = "pseudo_6dof"
_REALIZATION_ID = "jsbsim_surrogate_composite_pseudo6dof"
_SCREEN_ID = "a320-pseudo-cruise-native-coordinate-lqi-screen-v1"
_CAPABILITY_ADAPTER_ID = "taoryx.a320.local_native_coordinate_lqi_screen.capability.v1"


def _advertisement() -> dict[str, object]:
    """Return the exact static LQI contract without constructing its plant."""

    return {
        "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
        "id": _SCREEN_ID,
        "mission_template_id": _MISSION_ID,
        "fidelity": _FIDELITY,
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
            "These are internally generated named response coordinates for the exact local controller screen, "
            "not caller-owned actions, physical A320 surfaces, an allocator, or a route controller."
        ),
    }
    ####


def _config_factory() -> LocalNativeCoordinateLqiScreenConfig:
    """Load the A320 surrogate implementation only after exact screen selection."""

    from taoryx.trajectory.a320_adapter import build_a320_local_native_coordinate_lqi_screen_config

    return build_a320_local_native_coordinate_lqi_screen_config()
    ####


def a320_local_native_coordinate_lqi_screen_definition() -> LocalNativeCoordinateLqiScreenDefinition:
    """Declare the executable A320-owned named-control LQI endpoint."""

    return LocalNativeCoordinateLqiScreenDefinition(
        id=_SCREEN_ID,
        family_id=_FAMILY_ID,
        mission_id=_MISSION_ID,
        fidelity=_FIDELITY,
        initialization_id="pseudo6dof_cruise_attitude_local_point",
        segment_id="native_coordinate_attitude_lqi_screen",
        capability_adapter_id=_CAPABILITY_ADAPTER_ID,
        config_factory=_config_factory,
        advertisement=_advertisement(),
        claim_boundary=(
            "This is one local A320 OpenAP/JSBSim surrogate-composite attitude LQI recovery at the pinned cruise "
            "point. It proves only the named aileron/elevator/rudder response coordinates, retained common-host LQI "
            "candidate, and bounded nonlinear recovery. It does not establish a physical A320 surface allocator, "
            "actuator dynamics, route guidance, wind rejection, or aircraft qualification."
        ),
    )
    ####


def a320_local_native_coordinate_lqi_controller_screen_advertisement() -> LocalControllerScreenAdvertisement:
    """Publish the same exact endpoint through the generic authoring/UI registry."""

    return LocalControllerScreenAdvertisement(
        id=_SCREEN_ID,
        provider_id=_PROVIDER_ID,
        provider_aliases=(_AGGREGATE_PROVIDER_ID,),
        model_id=_FAMILY_ID,
        family_id=_FAMILY_ID,
        fidelity=_FIDELITY,
        realization_id=_REALIZATION_ID,
        mission_template_id=_MISSION_ID,
        advertisement=_advertisement(),
    )
    ####


__all__ = [
    "a320_local_native_coordinate_lqi_controller_screen_advertisement",
    "a320_local_native_coordinate_lqi_screen_definition",
]
