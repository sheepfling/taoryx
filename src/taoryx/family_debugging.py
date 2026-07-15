"""Family-specific debugging plans built from canonical run artifacts."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Iterable

from taoryx.outputs import EventRecord, RunArtifact, VehicleKind


class DebugFamily(StrEnum):
    """Vehicle/mission families with specialized debugging views."""

    ROCKET = "rocket"
    GLIDER = "glider"
    AIRBREATHER = "airbreather"
    ORBITAL = "orbital"
    SUBORBITAL = "suborbital"
    QUADCOPTER = "quadcopter"
####


@dataclass(frozen=True, slots=True)
class DebugPanelSpec:
    """A named plot panel with required and optional semantic channels."""

    panel_id: str
    title: str
    required_channels: tuple[str, ...] = ()
    optional_channels: tuple[str, ...] = ()
    event_kinds: tuple[str, ...] = ()
####


@dataclass(frozen=True, slots=True)
class ResolvedDebugPanel:
    """Panel availability after resolving a profile against telemetry."""

    spec: DebugPanelSpec
    available_channels: tuple[str, ...]
    missing_required: tuple[str, ...]
    missing_optional: tuple[str, ...]

    @property
    def renderable(self) -> bool:
        """Whether the panel has all of its required inputs."""

        return not self.missing_required
    ####
####


@dataclass(frozen=True, slots=True)
class FamilyDebugPlan:
    """Resolved family panels and structured event log for one artifact."""

    family: DebugFamily
    panels: tuple[ResolvedDebugPanel, ...]
    events: tuple[EventRecord, ...]

    @property
    def diagnostics(self) -> tuple[str, ...]:
        """Return non-fatal diagnostics for missing optional/required data."""

        messages: list[str] = []
        for panel in self.panels:
            if panel.missing_required:
                messages.append(f"{panel.spec.panel_id}: missing required channels {', '.join(panel.missing_required)}")
            if panel.missing_optional:
                messages.append(f"{panel.spec.panel_id}: missing optional channels {', '.join(panel.missing_optional)}")
        return tuple(messages)
    ####


_COMMON_PANELS = (
    DebugPanelSpec("phase_timeline", "Phase and event timeline", optional_channels=("phase.segment",), event_kinds=("segment_transition",)),
)

_PROFILES: dict[DebugFamily, tuple[DebugPanelSpec, ...]] = {
    DebugFamily.ROCKET: _COMMON_PANELS
    + (
        DebugPanelSpec("altitude_range", "Altitude and range", ("position.altitude.geodetic",), ("position.ecfc.x", "position.ecfc.y")),
        DebugPanelSpec("mass_propulsion", "Mass, thrust, and mass flow", ("mass.total",), ("propulsion.thrust", "mass.flow")),
        DebugPanelSpec("aero_load", "Dynamic pressure and load", (), ("aerodynamics.dynamic_pressure", "aerodynamics.normal_load")),
    ),
    DebugFamily.GLIDER: _COMMON_PANELS
    + (
        DebugPanelSpec("ground_track", "Ground track", ("position.ecfc.x", "position.ecfc.y")),
        DebugPanelSpec("energy", "Speed, angle of attack, and lift-to-drag", ("velocity.ecfc.x",), ("aerodynamics.angle_of_attack", "aerodynamics.lift_to_drag")),
        DebugPanelSpec("wind", "Air-relative wind", (), ("environment.wind.east", "environment.wind.north", "environment.wind.down")),
    ),
    DebugFamily.AIRBREATHER: _COMMON_PANELS
    + (
        DebugPanelSpec("altitude_mach", "Altitude and Mach", ("position.altitude.geodetic",), ("aerodynamics.mach",)),
        DebugPanelSpec("engine", "Throttle, thrust, fuel flow, and mass", ("mass.total",), ("propulsion.throttle_command", "propulsion.thrust", "mass.flow")),
        DebugPanelSpec("operating_envelope", "Dynamic pressure and angle of attack", (), ("aerodynamics.dynamic_pressure", "aerodynamics.angle_of_attack")),
    ),
    DebugFamily.ORBITAL: _COMMON_PANELS
    + (
        DebugPanelSpec("orbit_state", "Altitude and inertial speed", ("position.altitude.geodetic",), ("velocity.ecfc.x", "velocity.ecfc.y", "velocity.ecfc.z")),
        DebugPanelSpec("orbital_elements", "Orbital energy and angular momentum", (), ("orbital.specific_energy", "orbital.angular_momentum"), ("apoapsis", "periapsis")),
        DebugPanelSpec("ground_track", "Orbital ground track", ("position.ecfc.x", "position.ecfc.y"), ("position.latitude.geodetic", "position.longitude.geodetic")),
        DebugPanelSpec("reentry", "Reentry corridor", ("position.altitude.geodetic",), ("aerodynamics.angle_of_attack", "aerodynamics.dynamic_pressure"), ("reentry",)),
    ),
    DebugFamily.SUBORBITAL: _COMMON_PANELS
    + (
        DebugPanelSpec("ascent_return", "Altitude and speed", ("position.altitude.geodetic",), ("velocity.ecfc.x", "velocity.ecfc.y", "velocity.ecfc.z")),
        DebugPanelSpec("speed_mach", "Speed and Mach history", (), ("velocity.ecfc.x", "aerodynamics.mach")),
        DebugPanelSpec("return_corridor", "Dynamic pressure and load", (), ("aerodynamics.dynamic_pressure", "aerodynamics.normal_load"), ("apogee", "atmospheric-return", "ground")),
        DebugPanelSpec("mass_drag", "Mass and propulsion", ("mass.total",), ("propulsion.thrust", "mass.flow")),
    ),
    DebugFamily.QUADCOPTER: _COMMON_PANELS
    + (
        DebugPanelSpec("ground_track", "Ground track and lap markers", ("position.ecfc.x", "position.ecfc.y"), (), ("lap-complete",)),
        DebugPanelSpec("altitude_hold", "Altitude hold", ("position.altitude.geodetic",), ("guidance.altitude_command", "velocity.ecfc.z")),
        DebugPanelSpec("airspeed_wind", "Airspeed and changing wind", ("velocity.airspeed",), ("environment.wind.east", "environment.wind.north", "environment.wind.down")),
        DebugPanelSpec("attitude_rates", "Attitude and body rates", (), ("attitude.roll", "attitude.pitch", "attitude.yaw", "rates.body")),
        DebugPanelSpec("controls", "Commands and saturation", (), ("control.throttle", "control.roll", "control.pitch", "control.yaw"), ("control-saturation",)),
    ),
}


def family_profile(family: DebugFamily | str) -> tuple[DebugPanelSpec, ...]:
    """Return the immutable panel profile for a family."""

    return _PROFILES[DebugFamily(family)]
####


def build_family_debug_plan(artifact: RunArtifact, family: DebugFamily | str, *, vehicle_id: str | None = None) -> FamilyDebugPlan:
    """Resolve a family profile against one or all vehicles in an artifact."""

    selected_family = DebugFamily(family)
    vehicles = [artifact.vehicles[vehicle_id]] if vehicle_id is not None else list(artifact.vehicles.values())
    available = {name for vehicle in vehicles for name in vehicle.channels}
    events = tuple(event for vehicle in vehicles for event in vehicle.events)
    panels = tuple(_resolve_panel(spec, available, events) for spec in family_profile(selected_family))
    return FamilyDebugPlan(selected_family, panels, tuple(sorted(events, key=lambda event: (event.time, event.vehicle, event.name))))
####


def infer_debug_family(kind: VehicleKind) -> DebugFamily:
    """Map canonical vehicle kind to the closest standard debugging profile."""

    return {VehicleKind.ROCKET: DebugFamily.ROCKET, VehicleKind.GLIDER: DebugFamily.GLIDER, VehicleKind.AIRBREATHER: DebugFamily.AIRBREATHER}.get(kind, DebugFamily.SUBORBITAL)
####


def _resolve_panel(spec: DebugPanelSpec, available: set[str], events: Iterable[EventRecord]) -> ResolvedDebugPanel:
    event_names = {event.name for event in events} | {event.kind for event in events}
    available_channels = tuple(name for name in (*spec.required_channels, *spec.optional_channels) if name in available)
    missing_required = tuple(name for name in spec.required_channels if name not in available)
    missing_optional = tuple(name for name in spec.optional_channels if name not in available)
    if spec.event_kinds and not any(kind in event_names for kind in spec.event_kinds):
        missing_optional += tuple(f"event:{kind}" for kind in spec.event_kinds)
    return ResolvedDebugPanel(spec, available_channels, missing_required, missing_optional)
####
