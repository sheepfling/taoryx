"""Semantic lowering for the retained NASA/NESC two-stage source replay.

The retained NESC product is an immutable, source-qualified translation
history.  It is not a participating propulsion, guidance, or gimbal plant.
This translator therefore accepts only a composition whose declared launch,
staging delay, and terminal kind match that pinned witness.  It produces an
ordered phase/event plan for source-replay execution rather than pretending
that the semantic controller intents can change the source trajectory.
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from .trajectory.nesc_pseudo6dof import DEFAULT_REDUCTION
from .vehicle_composition import CompiledSegment, CompiledVehicleComposition


@dataclass(frozen=True, slots=True)
class NescReplayMissionSegment:
    """One semantic segment mapped to an immutable source-history interval."""

    instance_id: str
    segment_id: str
    start_time_s: float
    end_time_s: float
    required_truth_events: tuple[str, ...]

    def manifest(self) -> dict[str, object]:
        """Return stable lowering provenance for one source interval."""

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
class NescReplayMissionPlan:
    """Exact source-replay lowering of one NESC semantic composition."""

    source_artifact: Path
    fidelity: str
    segments: tuple[NescReplayMissionSegment, ...]
    source_initial_mass_kg: float
    source_initial_heading_deg: float

    def manifest(self) -> dict[str, object]:
        """Return the self-contained semantic-to-source-replay lowering."""

        return {
            "translator_id": "taoryx.nesc.staged_source_replay.v1",
            "source_artifact": str(self.source_artifact),
            "fidelity": self.fidelity,
            "source_initial_mass_kg": self.source_initial_mass_kg,
            "source_initial_heading_deg": self.source_initial_heading_deg,
            "segments": [segment.manifest() for segment in self.segments],
            "claim_boundary": (
                "This plan maps only to the pinned NESC source-history replay. "
                "Its semantic pitch/throttle/attitude intents do not command a participating plant."
            ),
        }
        ####

    ####


def compile_nesc_source_replay_mission(
    composition: CompiledVehicleComposition,
    *,
    source_artifact: str | Path = DEFAULT_REDUCTION,
) -> NescReplayMissionPlan:
    """Lower the one source-compatible NESC composition to replay intervals."""

    if composition.family_id != "reference_nesc_two_stage_rocket":
        raise ValueError(f"NESC source translator cannot lower family {composition.family_id!r}")
    if composition.mission != "staged_rocket_launch_target_state_v1":
        raise ValueError(f"NESC source translator cannot lower mission {composition.mission!r}")
    if composition.fidelity not in {"point_mass_3dof", "pseudo_6dof"}:
        raise ValueError(f"NESC source translator has no replay for tier {composition.fidelity!r}")
    if composition.initialization.id != "pad_launch":
        raise ValueError("NESC source replay requires the declared pad_launch initialization")
    expected_ids = ("ignition_ascent", "stage_separation", "ignition_ascent", "coast_terminal_state")
    if tuple(segment.id for segment in composition.segments) != expected_ids:
        raise ValueError("NESC source replay requires the declared staged-launch segment sequence")

    artifact = Path(source_artifact)
    history = _history(artifact)
    phase_times = _phase_start_times(history)
    _validate_source_compatible_inputs(composition, history, phase_times)
    return NescReplayMissionPlan(
        source_artifact=artifact,
        fidelity=composition.fidelity,
        source_initial_mass_kg=_number(history[0], "mass_kg"),
        source_initial_heading_deg=_heading_deg(_vector(history[0], "source_velocity_eci_mps")),
        segments=(
            NescReplayMissionSegment(
                composition.segments[0].instance_id,
                "ignition_ascent",
                phase_times["stage1_burn"],
                phase_times["stack_coast"],
                ("liftoff_stage1_ignition", "stage1_cutoff"),
            ),
            NescReplayMissionSegment(
                composition.segments[1].instance_id,
                "stage_separation",
                phase_times["stack_coast"],
                phase_times["stage2_burn"],
                ("stage_separation", "upper_stage_ignition"),
            ),
            NescReplayMissionSegment(
                composition.segments[2].instance_id,
                "ignition_ascent",
                phase_times["stage2_burn"],
                phase_times["orbit_coast"],
                ("stage2_cutoff",),
            ),
            NescReplayMissionSegment(
                composition.segments[3].instance_id,
                "coast_terminal_state",
                phase_times["orbit_coast"],
                _number(history[-1], "time_s"),
                ("nominal_endpoint",),
            ),
        ),
    )
    ####


def _validate_source_compatible_inputs(
    composition: CompiledVehicleComposition,
    history: tuple[dict[str, object], ...],
    phase_times: dict[str, float],
) -> None:
    """Reject semantic requests the immutable replay cannot truthfully honor."""

    initialization = composition.initialization.inputs
    initial_mass = _input_number(initialization, "initial_mass_kg")
    source_mass = _number(history[0], "mass_kg")
    if not math.isclose(initial_mass, source_mass, abs_tol=1.0e-6):
        raise ValueError(f"NESC source replay requires initial_mass_kg={source_mass:.12g}")
    for name in ("latitude_deg", "longitude_deg"):
        if not math.isclose(_input_number(initialization, name), 0.0, abs_tol=1.0e-9):
            raise ValueError(f"NESC source replay requires {name}=0.0 for its pinned ECI launch origin")
    source_heading = _heading_deg(_vector(history[0], "source_velocity_eci_mps"))
    if not math.isclose(_input_number(initialization, "initial_heading_deg"), source_heading, abs_tol=0.1):
        raise ValueError(f"NESC source replay requires initial_heading_deg={source_heading:.6g}")

    first_ascent, separation, second_ascent, coast = composition.segments
    _finite_input(first_ascent, "pitch_program")
    _finite_input(second_ascent, "pitch_program")
    expected_delay = phase_times["stage2_burn"] - phase_times["stack_coast"]
    supplied_delay = _input_number(separation.inputs, "separation_delay_s")
    if not math.isclose(supplied_delay, expected_delay, abs_tol=1.0e-6):
        raise ValueError(f"NESC source replay requires separation_delay_s={expected_delay:.12g}")
    terminal_kind = coast.inputs["terminal_kind"].value
    if terminal_kind != "orbit":
        raise ValueError("NESC source replay terminates at the pinned orbit-coast endpoint; terminal_kind must be 'orbit'")
    ####


def _history(path: Path) -> tuple[dict[str, object], ...]:
    """Load the pinned history without accepting a nearby malformed replay."""

    payload = json.loads(path.read_text(encoding="utf-8"))
    rows = payload.get("history") if isinstance(payload, dict) else None
    if not isinstance(rows, list) or not rows:
        raise ValueError(f"NESC source replay has no history: {path}")
    converted = tuple(row for row in rows if isinstance(row, dict))
    if len(converted) != len(rows):
        raise ValueError("NESC source replay history contains a non-mapping row")
    return converted
    ####


def _phase_start_times(history: tuple[dict[str, object], ...]) -> dict[str, float]:
    """Return source-observed starts for every phase required by this plan."""

    expected = ("stage1_burn", "stack_coast", "stage2_burn", "orbit_coast")
    starts: dict[str, float] = {}
    for row in history:
        phase = row.get("phase")
        if isinstance(phase, str) and phase not in starts:
            starts[phase] = _number(row, "time_s")
    missing = [name for name in expected if name not in starts]
    if missing:
        raise ValueError(f"NESC source replay is missing required phase starts: {', '.join(missing)}")
    return {name: starts[name] for name in expected}
    ####


def _input_number(inputs: Mapping[str, object], name: str) -> float:
    item = inputs.get(name)
    value = getattr(item, "value", None)
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"NESC composition input {name!r} must be finite numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"NESC composition input {name!r} must be finite numeric")
    return result
    ####


def _finite_input(segment: CompiledSegment, name: str) -> float:
    return _input_number(segment.inputs, name)
    ####


def _number(row: dict[str, object], name: str) -> float:
    value = row.get(name)
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"NESC source history value {name!r} must be finite numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"NESC source history value {name!r} must be finite numeric")
    return result
    ####


def _vector(row: dict[str, object], name: str) -> tuple[float, float, float]:
    value = row.get(name)
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"NESC source history vector {name!r} must have three values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"NESC source history vector {name!r} must be finite")
    return result  # type: ignore[return-value]
    ####


def _heading_deg(velocity_eci_mps: tuple[float, float, float]) -> float:
    """Return the local east-of-north launch heading implied by ECI velocity."""

    return math.degrees(math.atan2(velocity_eci_mps[1], velocity_eci_mps[0]))
    ####


__all__ = [
    "NescReplayMissionPlan",
    "NescReplayMissionSegment",
    "compile_nesc_source_replay_mission",
]
