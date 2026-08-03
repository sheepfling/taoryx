"""Fail-closed lowering for local direct-wrench controller screens.

This translator deliberately handles a *local controller screen*, not a
navigation mission.  A source plant can therefore expose the useful evidence
that it supports today—local source-load recovery through an explicit bounded
direct wrench—without being misrepresented as a release-to-handoff or landing
executor.  Families opt in by supplying their exact configuration factory to
the shared executor after this semantic contract has compiled.
"""

from __future__ import annotations

from dataclasses import dataclass

from .vehicle_composition import CompiledVehicleComposition


@dataclass(frozen=True, slots=True)
class LocalDirectWrenchScreenMissionPlan:
    """Immutable semantic lowering for one source-local recovery screen."""

    family_id: str
    mission_id: str
    fidelity: str
    initialization_id: str
    segment_instance_id: str
    screen_config_id: str

    def manifest(self) -> dict[str, object]:
        """Return the portable lowering record embedded in run artifacts."""

        return {
            "schema": "taoryx.local-direct-wrench-screen-plan/v1alpha1",
            "family_id": self.family_id,
            "mission_id": self.mission_id,
            "fidelity": self.fidelity,
            "initialization_id": self.initialization_id,
            "segments": [
                {
                    "instance_id": self.segment_instance_id,
                    "id": "local_wrench_recovery_screen",
                    "transition_semantics": "screen_completion_only",
                }
            ],
            "screen_config_id": self.screen_config_id,
            "claim_boundary": (
                "This lowering selects one pinned local source-load recovery screen. It does not create a "
                "navigation path, trim an external flight state, schedule propulsion, or establish an end-to-end mission."
            ),
        }
        ####
    ####


def compile_local_direct_wrench_screen_mission(
    composition: CompiledVehicleComposition,
    *,
    family_id: str,
    mission_id: str,
    initialization_id: str,
    screen_config_id: str,
) -> LocalDirectWrenchScreenMissionPlan:
    """Validate one intentionally parameter-free local-screen composition.

    The source configuration owns the operating point, perturbation,
    integration cadence, authority bounds, and LQR weights.  Accepting nearby
    semantic values here would silently turn a checked local evidence case
    into a new, unqualified controller experiment.
    """

    if composition.family_id != family_id:
        raise ValueError(f"local direct-wrench screen requires family {family_id!r}")
    if composition.mission != mission_id:
        raise ValueError(f"local direct-wrench screen requires mission {mission_id!r}")
    if composition.fidelity != "rigid_body_6dof_direct_wrench":
        raise ValueError("local direct-wrench screen requires rigid_body_6dof_direct_wrench")
    if composition.initialization.id != initialization_id:
        raise ValueError(f"local direct-wrench screen requires initialization {initialization_id!r}")
    if composition.initialization.inputs:
        raise ValueError("local direct-wrench screen does not accept initialization overrides")
    if len(composition.segments) != 1:
        raise ValueError("local direct-wrench screen requires exactly one semantic segment")
    segment = composition.segments[0]
    if segment.id != "local_wrench_recovery_screen":
        raise ValueError("local direct-wrench screen requires local_wrench_recovery_screen")
    if segment.inputs:
        raise ValueError("local direct-wrench screen does not accept segment overrides")
    return LocalDirectWrenchScreenMissionPlan(
        family_id=family_id,
        mission_id=mission_id,
        fidelity=composition.fidelity,
        initialization_id=initialization_id,
        segment_instance_id=segment.instance_id,
        screen_config_id=screen_config_id,
    )
    ####


__all__ = ["LocalDirectWrenchScreenMissionPlan", "compile_local_direct_wrench_screen_mission"]
