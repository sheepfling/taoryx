"""Semantic lowering for the public HL-20 glide-energy mission intent.

This translator is intentionally narrower than a native runner.  It converts a
validated composition into an immutable release/trim/bank/handoff plan so
preflight can prove that the selected inputs have one exact family-owned
meaning.  It does not construct the DAVE-ML plant, solve trim, choose
elevons, inject a wrench, or execute a trajectory.  Those remain separate
runtime promotion work.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

from .hl20_reachability import HL20_FIXED_MASS_KG
from .vehicle_composition import CompiledVehicleComposition


@dataclass(frozen=True, slots=True)
class HL20GlideEnergyMissionSegment:
    """One immutable semantic segment in a public HL-20 glide plan."""

    instance_id: str
    segment_id: str
    intent: dict[str, float]

    def manifest(self) -> dict[str, object]:
        """Return a portable source-independent semantic segment record."""

        return {
            "instance_id": self.instance_id,
            "segment_id": self.segment_id,
            "intent": dict(self.intent),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class HL20GlideEnergyMissionPlan:
    """One exact public lowering of the selected release-to-handoff intent."""

    fidelity: str
    release: dict[str, float]
    segments: tuple[HL20GlideEnergyMissionSegment, ...]

    def manifest(self) -> dict[str, object]:
        """Return the immutable translator product consumed by later runners."""

        return {
            "schema": "taoryx.hl20-glide-energy-mission-plan/v1alpha1",
            "translator_id": "taoryx.hl20_glide_energy_intent.v1",
            "fidelity": self.fidelity,
            "release": dict(self.release),
            "segments": [segment.manifest() for segment in self.segments],
            "claim_boundary": (
                "This is a semantic release/trim/opposing-bank/energy-handoff lowering only. It does not "
                "construct a native HL-20 plant, solve trim, establish crossrange closure, allocate effectors, "
                "inject a wrench, execute dynamics, or qualify the vehicle."
            ),
        }
        ####
    ####


def compile_hl20_glide_energy_mission(
    composition: CompiledVehicleComposition,
) -> HL20GlideEnergyMissionPlan:
    """Lower exactly the supported public HL-20 reduced-fidelity mission shape."""

    if composition.family_id != "hl20_mod_k":
        raise ValueError(f"HL-20 glide translator cannot lower family {composition.family_id!r}")
    if composition.mission != "lifting_body_glide_energy_management_v1":
        raise ValueError(f"HL-20 glide translator cannot lower mission {composition.mission!r}")
    if composition.fidelity not in {"point_mass_3dof", "pseudo_6dof"}:
        raise ValueError(f"HL-20 glide translator has no public lowered plan for tier {composition.fidelity!r}")
    if composition.initialization.id != "high_altitude_release":
        raise ValueError("HL-20 glide-energy mission requires high_altitude_release initialization")
    expected_segment_ids = ("trim_capture", "glide_bank_reversal", "glide_bank_reversal", "energy_handoff")
    if tuple(segment.id for segment in composition.segments) != expected_segment_ids:
        raise ValueError("HL-20 glide-energy mission requires the declared trim/bank/bank/handoff sequence")

    release = {
        "altitude_m": _number(composition.initialization.inputs, "altitude_m"),
        "mach": _number(composition.initialization.inputs, "mach"),
        "heading_deg": _number(composition.initialization.inputs, "heading_deg"),
        "mass_kg": _optional_number(composition.initialization.inputs, "mass_kg", HL20_FIXED_MASS_KG),
    }
    trim, first_bank, second_bank, handoff = composition.segments
    bank_segments = (first_bank, second_bank)
    bank_commands = tuple(_number(segment.inputs, "bank_command_deg") for segment in bank_segments)
    if bank_commands[0] * bank_commands[1] >= 0.0:
        raise ValueError("HL-20 glide-energy mission requires two strictly opposing signed bank commands")

    return HL20GlideEnergyMissionPlan(
        fidelity=composition.fidelity,
        release=release,
        segments=(
            HL20GlideEnergyMissionSegment(
                trim.instance_id,
                trim.id,
                {"duration_s": _number(trim.inputs, "duration_s")},
            ),
            *(
                HL20GlideEnergyMissionSegment(
                    segment.instance_id,
                    segment.id,
                    {
                        "bank_command_deg": _number(segment.inputs, "bank_command_deg"),
                        "target_crossrange_m": _number(segment.inputs, "target_crossrange_m"),
                    },
                )
                for segment in bank_segments
            ),
            HL20GlideEnergyMissionSegment(
                handoff.instance_id,
                handoff.id,
                {
                    "target_altitude_m": _number(handoff.inputs, "target_altitude_m"),
                    "target_energy_j_kg": _number(handoff.inputs, "target_energy"),
                    "target_heading_deg": _number(handoff.inputs, "target_heading_deg"),
                },
            ),
        ),
    )
    ####


def _number(inputs: Mapping[str, object], name: str) -> float:
    """Read one compiled finite numeric intent without unit reconstruction."""

    item = inputs.get(name)
    value = getattr(item, "value", None)
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"HL-20 glide input {name!r} must be numeric")
    numeric_value = float(value)
    if not math.isfinite(numeric_value):
        raise ValueError(f"HL-20 glide input {name!r} must be finite")
    return numeric_value
    ####


def _optional_number(inputs: Mapping[str, object], name: str, default: float) -> float:
    """Use the declared source fixture only for an omitted optional release mass."""

    if name not in inputs:
        return default
    return _number(inputs, name)
    ####


__all__ = [
    "HL20GlideEnergyMissionPlan",
    "HL20GlideEnergyMissionSegment",
    "compile_hl20_glide_energy_mission",
]
