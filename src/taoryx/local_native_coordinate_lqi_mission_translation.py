"""Fail-closed lowering for local native-coordinate LQI controller screens."""

from __future__ import annotations

from dataclasses import dataclass

from .vehicle_composition import CompiledVehicleComposition


@dataclass(frozen=True, slots=True)
class LocalNativeCoordinateLqiScreenMissionPlan:
    """Portable semantic lowering for one pinned native-coordinate screen."""

    family_id: str
    mission_id: str
    fidelity: str
    initialization_id: str
    segment_instance_id: str
    segment_id: str
    screen_config_id: str

    def manifest(self) -> dict[str, object]:
        """Return the immutable plan persisted with a public batch packet."""

        return {
            "schema": "taoryx.local-native-coordinate-lqi-screen-plan/v1alpha1",
            "family_id": self.family_id,
            "mission_id": self.mission_id,
            "fidelity": self.fidelity,
            "initialization_id": self.initialization_id,
            "segments": [
                {
                    "instance_id": self.segment_instance_id,
                    "id": self.segment_id,
                    "transition_semantics": "screen_completion_only",
                }
            ],
            "screen_config_id": self.screen_config_id,
            "claim_boundary": (
                "This lowering selects one pinned local native-coordinate LQI recovery screen. It does not create "
                "a route, retune a controller, schedule an envelope, or establish a physical effector path."
            ),
        }
        ####
    ####


def compile_local_native_coordinate_lqi_screen_mission(
    composition: CompiledVehicleComposition,
    *,
    family_id: str,
    mission_id: str,
    fidelity: str,
    initialization_id: str,
    segment_id: str,
    screen_config_id: str,
) -> LocalNativeCoordinateLqiScreenMissionPlan:
    """Reject every request that deviates from a declared pinned LQI screen."""

    if composition.family_id != family_id:
        raise ValueError(f"local native-coordinate LQI screen requires family {family_id!r}")
    if composition.mission != mission_id:
        raise ValueError(f"local native-coordinate LQI screen requires mission {mission_id!r}")
    if composition.fidelity != fidelity:
        raise ValueError(f"local native-coordinate LQI screen requires fidelity {fidelity!r}")
    if composition.initialization.id != initialization_id:
        raise ValueError(f"local native-coordinate LQI screen requires initialization {initialization_id!r}")
    if composition.initialization.inputs:
        raise ValueError("local native-coordinate LQI screen does not accept initialization overrides")
    if len(composition.segments) != 1:
        raise ValueError("local native-coordinate LQI screen requires exactly one semantic segment")
    segment = composition.segments[0]
    if segment.id != segment_id:
        raise ValueError(f"local native-coordinate LQI screen requires segment {segment_id!r}")
    if segment.inputs:
        raise ValueError("local native-coordinate LQI screen does not accept segment overrides")
    return LocalNativeCoordinateLqiScreenMissionPlan(
        family_id,
        mission_id,
        fidelity,
        initialization_id,
        segment.instance_id,
        segment_id,
        screen_config_id,
    )
    ####


__all__ = ["LocalNativeCoordinateLqiScreenMissionPlan", "compile_local_native_coordinate_lqi_screen_mission"]
