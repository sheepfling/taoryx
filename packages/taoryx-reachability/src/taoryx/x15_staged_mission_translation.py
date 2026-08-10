"""Semantic lowering for the retained X-15-scaled staged reachability witness.

The retained X-15 rocket-to-Hawaii source deck contributes concrete staging
anchors: launch mass and speed magnitude, attached-booster cutoff, and release
time.  The current executable reduced model is deliberately local-frame and
open-loop.  This translator admits only its pinned nominal composition so a
caller cannot mistake a nearby launch request for a source-backed X-15
mission, or mistake the separate synthetic California-to-Hawaii showcase for
this X-15-scaled witness.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

from .reachability_envelope import LaunchCommand, ReachabilityFidelity
from .vehicle_composition import CompiledVehicleComposition
from .x15_reachability import x15_integration_preflight, x15_source_staging_contract, x15_surrogate_vehicle


@dataclass(frozen=True, slots=True)
class X15StagedReachabilityMissionSegment:
    """One source-pinned semantic segment in the reduced staged witness."""

    instance_id: str
    segment_id: str
    start_time_s: float
    end_time_s: float
    required_truth_events: tuple[str, ...]

    def manifest(self) -> dict[str, object]:
        """Return one traceable semantic-to-runtime segment mapping."""

        return {
            "instance_id": self.instance_id,
            "segment_id": self.segment_id,
            "start_time_s": self.start_time_s,
            "end_time_s": self.end_time_s,
            "required_truth_events": list(self.required_truth_events),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class X15StagedReachabilityMissionPlan:
    """Exact semantic lowering of the local X-15-scaled staged witness."""

    fidelity: ReachabilityFidelity
    command: LaunchCommand
    horizon_s: float
    source_staging: tuple[tuple[str, float], ...]
    segments: tuple[X15StagedReachabilityMissionSegment, ...]

    def manifest(self) -> dict[str, object]:
        """Return the complete, claim-bounded lowering artifact."""

        return {
            "translator_id": "taoryx.x15.staged_reachability.v1",
            "fidelity": self.fidelity.value,
            "command": {
                "launch_azimuth_rad": self.command.azimuth_rad,
                "launch_elevation_rad": self.command.elevation_rad,
                "glide_bank_rad": self.command.bank_rad,
            },
            "horizon_s": self.horizon_s,
            "source_staging": dict(self.source_staging),
            "segments": [segment.manifest() for segment in self.segments],
            "claim_boundary": (
                "This is a source-pinned, local-frame X-15-scaled reduced staged witness. "
                "Its launch direction and fixed-L/D glide are declared engineering assumptions. "
                "It is not the synthetic California-to-Hawaii vehicle, a native X-15 rigid-body "
                "mission, a physical-effector simulation, or a controlled terminal handoff."
            ),
        }
        ####
    ####


def compile_x15_staged_reachability_mission(
    composition: CompiledVehicleComposition,
) -> X15StagedReachabilityMissionPlan:
    """Lower the one exact staged X-15-scaled composition to its reduced runtime.

    The explicit equality checks intentionally make this a reproducible witness
    rather than a falsely general X-15 mission generator.  Broader launch,
    guidance, and terminal variations belong to a future bounded variant space
    with its own qualification contract.
    """

    if composition.family_id != "x15":
        raise ValueError(f"X-15 staged translator cannot lower family {composition.family_id!r}")
    if composition.mission != "x15_staged_booster_reachability_v1":
        raise ValueError(f"X-15 staged translator cannot lower mission {composition.mission!r}")
    fidelity = _reachability_fidelity(composition.fidelity)
    if composition.initialization.id != "staged_booster_launch":
        raise ValueError("X-15 staged reachability requires staged_booster_launch initialization")
    expected_segments = (
        "booster_powered",
        "booster_coast_release",
        "unpowered_glide_handoff",
        "impact_witness",
    )
    if tuple(segment.id for segment in composition.segments) != expected_segments:
        raise ValueError("X-15 staged reachability requires the declared booster/coast/glide/impact sequence")

    source = x15_source_staging_contract()
    vehicle = x15_surrogate_vehicle()
    integration = x15_integration_preflight(vehicle)
    if not integration.passed:
        raise ValueError(f"X-15 staged reachability source preflight failed: {', '.join(integration.failures)}")

    initialization = composition.initialization.inputs
    _require_close(initialization, "initial_mass_kg", source["launch_mass_kg"], "kg")
    _require_close(initialization, "initial_speed_m_s", source["initial_speed_m_s"], "m/s")
    _require_close(initialization, "initial_altitude_m", vehicle.initial_altitude_m, "m")
    azimuth_deg = _number(initialization, "launch_azimuth_deg")
    elevation_deg = _number(initialization, "launch_elevation_deg")
    bank_deg = _number(initialization, "glide_bank_deg")
    _require_value("launch_azimuth_deg", azimuth_deg, 0.0, "deg")
    _require_value("launch_elevation_deg", elevation_deg, 45.0, "deg")
    _require_value("glide_bank_deg", bank_deg, 0.0, "deg")

    powered, coast, glide, impact = composition.segments
    _require_close(powered.inputs, "cutoff_time_s", source["powered_duration_s"], "s")
    _require_close(coast.inputs, "release_time_s", source["release_time_s"], "s")
    horizon_s = _number(glide.inputs, "horizon_s")
    _require_value("horizon_s", horizon_s, 1_000.0, "s")
    terminal_kind = impact.inputs.get("terminal_kind")
    if terminal_kind is None or terminal_kind.value != "impact":
        raise ValueError("X-15 staged reachability requires terminal_kind='impact'")

    return X15StagedReachabilityMissionPlan(
        fidelity=fidelity,
        command=LaunchCommand(math.radians(azimuth_deg), math.radians(elevation_deg), math.radians(bank_deg)),
        horizon_s=horizon_s,
        source_staging=tuple(sorted(source.items())),
        segments=(
            X15StagedReachabilityMissionSegment(
                powered.instance_id,
                powered.id,
                0.0,
                source["powered_duration_s"],
                ("booster_cutoff",),
            ),
            X15StagedReachabilityMissionSegment(
                coast.instance_id,
                coast.id,
                source["powered_duration_s"],
                source["release_time_s"],
                ("booster_release",),
            ),
            X15StagedReachabilityMissionSegment(
                glide.instance_id,
                glide.id,
                source["release_time_s"],
                horizon_s,
                ("high_energy_terminal_corridor", "atmospheric_terminal_handoff"),
            ),
            X15StagedReachabilityMissionSegment(
                impact.instance_id,
                impact.id,
                source["release_time_s"],
                horizon_s,
                ("terminal_impact_witness",),
            ),
        ),
    )
    ####


def _reachability_fidelity(value: str) -> ReachabilityFidelity:
    """Map canonical composition tiers to the implemented reduced runtimes."""

    if value == "point_mass_3dof":
        return ReachabilityFidelity.POINT_MASS_3DOF
    if value == "pseudo_6dof":
        return ReachabilityFidelity.PSEUDO_6DOF
    raise ValueError(f"X-15 staged reachability has no reduced runtime for tier {value!r}")
    ####


def _number(inputs: Mapping[str, object], name: str) -> float:
    """Return one finite compiled numeric composition input."""

    item = inputs.get(name)
    value = getattr(item, "value", None)
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"X-15 staged composition input {name!r} must be finite numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"X-15 staged composition input {name!r} must be finite numeric")
    return result
    ####


def _require_close(inputs: Mapping[str, object], name: str, expected: float, unit: str) -> None:
    """Fail closed when the pinned source value differs from a composition input."""

    actual = _number(inputs, name)
    _require_value(name, actual, expected, unit)
    ####


def _require_value(name: str, actual: float, expected: float, unit: str) -> None:
    """Require the deterministic witness value with a stable diagnostic."""

    if not math.isclose(actual, expected, abs_tol=1.0e-9):
        raise ValueError(f"X-15 staged reachability requires {name}={expected:.12g} {unit}")
    ####


__all__ = [
    "X15StagedReachabilityMissionPlan",
    "X15StagedReachabilityMissionSegment",
    "compile_x15_staged_reachability_mission",
]
