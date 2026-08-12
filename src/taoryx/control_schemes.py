"""Cross-family control-scheme vocabulary for discovery and UI projection.

Authority profiles remain the exact model-specific selectable action surfaces.
This module supplies the smaller, stable vocabulary used to compare those
profiles across vehicle families and fidelity tiers without pretending that
their native controls are interchangeable.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

ControlSchemeLayer = Literal[
    "open_loop",
    "mission",
    "kinematic",
    "pilot",
    "body_motion",
    "wrench",
    "effector",
    "event",
    "provider_specific",
]
ControlSchemeConsumerRole = Literal[
    "autonomy",
    "remote_operator",
    "human_pilot",
    "test_engineer",
    "provider",
]
ControlSchemeStreamingPreference = Literal[
    "primary",
    "alternative",
    "diagnostic",
    "provider_managed",
    "not_streamable",
]


@dataclass(frozen=True, slots=True)
class ControlSchemeDefinition:
    """One stable cross-family control concept."""

    id: str
    label: str
    layer: ControlSchemeLayer
    description: str
    consumer_roles: tuple[ControlSchemeConsumerRole, ...]
    streaming_preference: ControlSchemeStreamingPreference
    ui_order: int


_DEFINITIONS = (
    ControlSchemeDefinition(
        "open_loop.coast",
        "Open Loop / Coast",
        "open_loop",
        "No caller action; the selected model advances its declared open-loop program or coast.",
        ("provider", "test_engineer"),
        "provider_managed",
        10,
    ),
    ControlSchemeDefinition(
        "provider.program",
        "Provider Program",
        "mission",
        "A source or provider-owned mission, guidance, or control program with no caller action vector.",
        ("provider", "autonomy", "test_engineer"),
        "provider_managed",
        20,
    ),
    ControlSchemeDefinition(
        "mission.destination",
        "Destination Guidance",
        "mission",
        "A high-level destination or terminal objective lowered by a registered navigator and controller.",
        ("autonomy", "remote_operator"),
        "primary",
        30,
    ),
    ControlSchemeDefinition(
        "mission.waypoint",
        "Waypoint Guidance",
        "mission",
        "One live waypoint or waypoint sequence lowered by the selected guidance implementation.",
        ("autonomy", "remote_operator"),
        "primary",
        40,
    ),
    ControlSchemeDefinition(
        "mission.route",
        "Route Guidance",
        "mission",
        "A route, path, or segment target lowered by the selected guidance implementation.",
        ("autonomy", "remote_operator"),
        "primary",
        50,
    ),
    ControlSchemeDefinition(
        "mission.orbit_target",
        "Orbit Target",
        "mission",
        "A future spacecraft orbit, rendezvous, or maneuver target with an explicit registered lowering path.",
        ("autonomy", "remote_operator"),
        "primary",
        60,
    ),
    ControlSchemeDefinition(
        "mission.relative_pose",
        "Relative Pose Target",
        "mission",
        "A future relative position and attitude target for rendezvous, formation, or docking guidance.",
        ("autonomy", "remote_operator"),
        "primary",
        70,
    ),
    ControlSchemeDefinition(
        "kinematic.position",
        "Position Command",
        "kinematic",
        "A position or altitude target accepted by a reduced translational response law.",
        ("autonomy", "remote_operator"),
        "primary",
        100,
    ),
    ControlSchemeDefinition(
        "kinematic.velocity",
        "Velocity Command",
        "kinematic",
        "A velocity-vector, speed, course, or climb target accepted by a reduced response law.",
        ("autonomy", "remote_operator"),
        "primary",
        110,
    ),
    ControlSchemeDefinition(
        "kinematic.flight_path",
        "Flight-Path Command",
        "kinematic",
        "Speed, heading, path-angle, bank, or equivalent reduced flight-path commands.",
        ("autonomy", "remote_operator", "human_pilot"),
        "primary",
        120,
    ),
    ControlSchemeDefinition(
        "kinematic.energy",
        "Energy / Propulsion Command",
        "kinematic",
        "A reduced energy, aggregate thrust, or normalized propulsion request without physical-effector claims.",
        ("autonomy", "remote_operator", "human_pilot"),
        "alternative",
        130,
    ),
    ControlSchemeDefinition(
        "kinematic.relative_motion",
        "Relative-Motion Command",
        "kinematic",
        "A future relative translation or closing-rate request for formation and rendezvous models.",
        ("autonomy", "remote_operator"),
        "primary",
        140,
    ),
    ControlSchemeDefinition(
        "pilot.normalized_axes",
        "Normalized Pilot Controls",
        "pilot",
        "Controller-friendly propulsion, longitudinal, lateral, and yaw axes lowered by a reduced response law.",
        ("human_pilot", "remote_operator", "autonomy"),
        "alternative",
        160,
    ),
    ControlSchemeDefinition(
        "pilot.rotorcraft",
        "Rotorcraft Pilot Controls",
        "pilot",
        "Future collective, cyclic, pedal, and propulsion intent with family-specific lowering and labels.",
        ("human_pilot", "remote_operator", "autonomy"),
        "alternative",
        170,
    ),
    ControlSchemeDefinition(
        "pilot.ground_vehicle",
        "Ground-Vehicle Pilot Controls",
        "pilot",
        "Future propulsion, steering, and braking intent with family-specific lowering and labels.",
        ("human_pilot", "remote_operator", "autonomy"),
        "alternative",
        180,
    ),
    ControlSchemeDefinition(
        "pilot.marine",
        "Marine Pilot Controls",
        "pilot",
        "Future propulsion, rudder, lateral, or dive-plane intent with family-specific lowering and labels.",
        ("human_pilot", "remote_operator", "autonomy"),
        "alternative",
        190,
    ),
    ControlSchemeDefinition(
        "body_motion.attitude",
        "Attitude Command",
        "body_motion",
        "Attitude references accepted by a declared rotational response law.",
        ("autonomy", "remote_operator", "human_pilot", "test_engineer"),
        "alternative",
        200,
    ),
    ControlSchemeDefinition(
        "body_motion.body_rate",
        "Body-Rate Command",
        "body_motion",
        "Body-frame angular-rate references accepted by a declared rotational response law.",
        ("autonomy", "remote_operator", "human_pilot", "test_engineer"),
        "alternative",
        210,
    ),
    ControlSchemeDefinition(
        "body_motion.angular_acceleration",
        "Angular-Acceleration Command",
        "body_motion",
        "Angular-acceleration references accepted by an explicitly declared response law.",
        ("autonomy", "test_engineer"),
        "alternative",
        220,
    ),
    ControlSchemeDefinition(
        "wrench.direct",
        "Direct Wrench",
        "wrench",
        "Direct force and moment requests at a declared rigid-body bridge.",
        ("test_engineer", "autonomy"),
        "diagnostic",
        300,
    ),
    ControlSchemeDefinition(
        "effector.direct",
        "Direct Effectors",
        "effector",
        "Physical or source-declared actuator commands with exact allocation and actuator boundaries.",
        ("test_engineer", "human_pilot"),
        "diagnostic",
        400,
    ),
    ControlSchemeDefinition(
        "event.mission",
        "Mission Events",
        "event",
        "Named discrete or one-shot mission events with explicit repeat and reset policy.",
        ("autonomy", "remote_operator", "test_engineer"),
        "alternative",
        500,
    ),
    ControlSchemeDefinition(
        "provider.native_bridge",
        "Provider-Native Bridge",
        "provider_specific",
        "Provider-specific control coordinates retained without claiming cross-family equivalence.",
        ("provider", "test_engineer"),
        "diagnostic",
        600,
    ),
    ControlSchemeDefinition(
        "debug.mixed",
        "Mixed API Stress Controls",
        "provider_specific",
        "Non-physical continuous, vector, periodic, or increment controls used to test the API contract.",
        ("test_engineer",),
        "diagnostic",
        900,
    ),
    ControlSchemeDefinition(
        "debug.discrete",
        "Discrete API Stress Controls",
        "provider_specific",
        "Non-physical detent, increment, enum, and boolean controls used to test the API contract.",
        ("test_engineer",),
        "diagnostic",
        910,
    ),
    ControlSchemeDefinition(
        "debug.event",
        "Event API Stress Controls",
        "provider_specific",
        "Non-physical one-shot event controls used to test masks and repeat policy.",
        ("test_engineer",),
        "diagnostic",
        920,
    ),
)

CONTROL_SCHEME_CATALOG: dict[str, ControlSchemeDefinition] = {
    item.id: item for item in _DEFINITIONS
}


def control_scheme_definition(identifier: str) -> ControlSchemeDefinition | None:
    """Return one canonical definition, allowing provider-specific extensions."""

    return CONTROL_SCHEME_CATALOG.get(identifier)
    ####


def default_control_scheme_id(authority: str, authority_profile_id: str) -> str:
    """Return a conservative compatibility mapping for undeclared profiles.

    New or promoted profiles should declare their exact scheme ID. This fallback
    keeps legacy discovery complete while its support record remains labeled as
    ``authority_kind_fallback``.
    """

    normalized = authority_profile_id.casefold()
    if "waypoint" in normalized:
        return "mission.waypoint"
    if "pilot" in normalized:
        return "pilot.normalized_axes"
    if "body_rate" in normalized or "rate_command" in normalized:
        return "body_motion.body_rate"
    if "event" in normalized:
        return "event.mission"
    if "discrete" in normalized:
        return "debug.discrete"
    if "guidance" in normalized:
        return "kinematic.flight_path"
    if "program" in normalized or "generated" in normalized or "synthetic_mission" in normalized:
        return "provider.program"
    return {
        "open_loop": "open_loop.coast",
        "mission": "mission.route",
        "kinematic": "kinematic.flight_path",
        "body_motion": "body_motion.attitude",
        "wrench": "wrench.direct",
        "effector": "effector.direct",
        "native_bridge": "provider.native_bridge",
        "provider_defined": "provider.native_bridge",
    }.get(authority, "provider.native_bridge")
    ####


__all__ = [
    "CONTROL_SCHEME_CATALOG",
    "ControlSchemeConsumerRole",
    "ControlSchemeDefinition",
    "ControlSchemeLayer",
    "ControlSchemeStreamingPreference",
    "control_scheme_definition",
    "default_control_scheme_id",
]
