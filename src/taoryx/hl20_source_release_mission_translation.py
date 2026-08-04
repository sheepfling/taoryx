"""Strict lowering for the retained HL-20 source-scheduled release witness.

This is deliberately a different mission from ``lifting_body_glide_energy_management_v1``.
The latter is a user-authored high-altitude glide intent and has no native
lowerer yet.  This module accepts only the exact synthetic-booster/source-aero
configuration already implemented by :mod:`taoryx.hl20_reachability`; its bank
profile is a retained source schedule, not a guidance command.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

from .hl20_reachability import (
    HL20_ENERGY_MANAGED_BANK_SCHEDULE_DEG,
    HL20_ENERGY_MANAGED_SEGMENTS,
    HL20_FIXED_MASS_KG,
)
from .vehicle_composition import CompiledVehicleComposition

_BOOSTER_DRY_MASS_KG = 1_500.0
_BOOSTER_PROPELLANT_MASS_KG = 1_500.0
_SOURCE_INITIAL_SPEED_M_S = 350.0
_SOURCE_LAUNCH_AZIMUTH_DEG = 0.0
_SOURCE_LAUNCH_ELEVATION_DEG = 60.0
_SOURCE_RELEASE_TIME_S = 25.0
_SOURCE_HORIZON_S = 120.0


@dataclass(frozen=True, slots=True)
class HL20SourceReleaseMissionSegment:
    """One declared semantic segment mapped to the immutable source schedule."""

    instance_id: str
    segment_id: str
    start_time_s: float
    end_time_s: float
    required_truth_events: tuple[str, ...]

    def manifest(self) -> dict[str, object]:
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
class HL20SourceReleaseMissionPlan:
    """One exact lowering to the existing source-aerodynamics release runner."""

    fidelity: str
    segments: tuple[HL20SourceReleaseMissionSegment, ...]

    @property
    def source_initial_mass_kg(self) -> float:
        return HL20_FIXED_MASS_KG + _BOOSTER_DRY_MASS_KG + _BOOSTER_PROPELLANT_MASS_KG
        ####

    def manifest(self) -> dict[str, object]:
        return {
            "translator_id": "taoryx.hl20_source_booster_release_replay.v1",
            "fidelity": self.fidelity,
            "source_initial_speed_m_s": _SOURCE_INITIAL_SPEED_M_S,
            "source_initial_mass_kg": self.source_initial_mass_kg,
            "source_launch_azimuth_deg": _SOURCE_LAUNCH_AZIMUTH_DEG,
            "source_launch_elevation_deg": _SOURCE_LAUNCH_ELEVATION_DEG,
            "source_release_time_s": _SOURCE_RELEASE_TIME_S,
            "source_bank_schedule_deg": [list(item) for item in HL20_ENERGY_MANAGED_BANK_SCHEDULE_DEG],
            "source_segment_schedule": [list(item) for item in HL20_ENERGY_MANAGED_SEGMENTS],
            "source_horizon_s": _SOURCE_HORIZON_S,
            "segments": [segment.manifest() for segment in self.segments],
            "claim_boundary": (
                "This plan lowers only to the fixed source-aerodynamic/synthetic-booster release witness. "
                "Its bank profile is source-scheduled and not a caller-configurable guidance or effector command."
            ),
        }
        ####
    ####


def compile_hl20_source_booster_release_mission(
    composition: CompiledVehicleComposition,
) -> HL20SourceReleaseMissionPlan:
    """Accept only the semantic composition exactly represented by the source runner."""

    if composition.family_id != "hl20_mod_k":
        raise ValueError(f"HL-20 source release translator cannot lower family {composition.family_id!r}")
    if composition.mission != "hl20_source_booster_release_replay_v1":
        raise ValueError(f"HL-20 source release translator cannot lower mission {composition.mission!r}")
    if composition.fidelity not in {"point_mass_3dof", "pseudo_6dof"}:
        raise ValueError(f"HL-20 source release translator has no replay for tier {composition.fidelity!r}")
    if composition.initialization.id != "source_booster_launch":
        raise ValueError("HL-20 source release replay requires source_booster_launch initialization")
    expected_ids = ("source_booster_release", "source_scheduled_bank", "source_scheduled_bank", "source_ground_contact")
    if tuple(segment.id for segment in composition.segments) != expected_ids:
        raise ValueError("HL-20 source release replay requires the declared source release/bank/contact sequence")
    _validate_source_compatible_inputs(composition)
    return HL20SourceReleaseMissionPlan(
        fidelity=composition.fidelity,
        segments=(
            HL20SourceReleaseMissionSegment(
                composition.segments[0].instance_id,
                "source_booster_release",
                0.0,
                _SOURCE_RELEASE_TIME_S,
                ("booster-release", "mission-stabilization-start"),
            ),
            HL20SourceReleaseMissionSegment(
                composition.segments[1].instance_id,
                "source_scheduled_bank",
                35.0,
                45.0,
                ("mission-right_bank-start",),
            ),
            HL20SourceReleaseMissionSegment(
                composition.segments[2].instance_id,
                "source_scheduled_bank",
                45.0,
                55.0,
                ("mission-left_bank-start",),
            ),
            HL20SourceReleaseMissionSegment(
                composition.segments[3].instance_id,
                "source_ground_contact",
                55.0,
                _SOURCE_HORIZON_S,
                ("ground-contact",),
            ),
        ),
    )
    ####


def _validate_source_compatible_inputs(composition: CompiledVehicleComposition) -> None:
    initialization = composition.initialization.inputs
    _require_close(initialization, "initial_speed_m_s", _SOURCE_INITIAL_SPEED_M_S)
    _require_close(initialization, "initial_mass_kg", HL20_FIXED_MASS_KG + _BOOSTER_DRY_MASS_KG + _BOOSTER_PROPELLANT_MASS_KG)
    _require_close(initialization, "launch_azimuth_deg", _SOURCE_LAUNCH_AZIMUTH_DEG)
    _require_close(initialization, "launch_elevation_deg", _SOURCE_LAUNCH_ELEVATION_DEG)
    release, right_bank, left_bank, terminal = composition.segments
    _require_close(release.inputs, "release_time_s", _SOURCE_RELEASE_TIME_S)
    _require_close(right_bank.inputs, "bank_command_deg", 30.0)
    _require_close(right_bank.inputs, "duration_s", 10.0)
    _require_close(left_bank.inputs, "bank_command_deg", -30.0)
    _require_close(left_bank.inputs, "duration_s", 10.0)
    _require_close(terminal.inputs, "horizon_s", _SOURCE_HORIZON_S)
    ####


def _require_close(inputs: Mapping[str, object], name: str, expected: float) -> None:
    item = inputs.get(name)
    value = getattr(item, "value", None)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"HL-20 source release input {name!r} must be finite numeric")
    if not math.isclose(float(value), expected, abs_tol=1.0e-6):
        raise ValueError(f"HL-20 source release replay requires {name}={expected:.12g}")
    ####


__all__ = [
    "HL20SourceReleaseMissionPlan",
    "HL20SourceReleaseMissionSegment",
    "compile_hl20_source_booster_release_mission",
]
