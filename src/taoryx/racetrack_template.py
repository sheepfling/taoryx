"""Reusable powered-fixed-wing racetrack mission bindings.

The racetrack is a semantic mission template, not a vehicle model.  A vehicle
binding supplies its nominal speed, attainable vertical rates, scale, and
acceptance tolerances.  Resolution derives the route horizon, phase windows,
oriented gate geometry, and runtime route attributes from those values.

This layer intentionally does not choose a controller or inject a force or
moment.  The same resolved mission can therefore bind to a point-mass,
named-pseudo-6DOF, or rigid-body realization, with the realization declaring
which controls are physical and which are reduced-order.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Final, Literal

import yaml
from pydantic import BaseModel, ConfigDict, model_validator

from taoryx.racetrack_timing import RacetrackTimingEstimate, estimate_racetrack_timing

RacetrackFidelity = Literal[
    "point_mass_3dof",
    "pseudo_6dof_kinematic_bridge",
    "rigid_body_6dof_direct_wrench",
    "rigid_body_6dof_surface_allocated",
]
RACETRACK_FIDELITIES: Final[tuple[RacetrackFidelity, ...]] = (
    "point_mass_3dof",
    "pseudo_6dof_kinematic_bridge",
    "rigid_body_6dof_direct_wrench",
    "rigid_body_6dof_surface_allocated",
)


class RacetrackBinding(BaseModel):
    """Validated declarative input for one reusable racetrack mission.

    This is the owner boundary for YAML/catalog and compiler-derived route
    settings.  Once parsed, downstream code receives named attributes rather
    than indexing an open configuration bag.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    vehicle_id: str
    fidelity: RacetrackFidelity
    straight_length_m: float
    turn_radius_m: float
    speed_m_s: float
    low_altitude_m: float
    high_altitude_m: float
    climb_rate_m_s: float
    descent_rate_m_s: float
    left_turn_bank_deg: float
    right_turn_bank_deg: float
    source_realization: str = "unspecified"
    status: str = "unclassified"
    simulation_margin_s: float = 0.0
    altitude_capture_gain_per_s: float = 0.0
    altitude_capture_max_mps: float = 0.0
    position_capture_gain: float = 0.0
    position_capture_max_correction_mps: float = 0.0
    gate_corridor_m: float = 250.0
    gate_altitude_tolerance_m: float = 35.0
    gate_speed_tolerance_mps: float = 12.0
    position_capture_bank_gain_rad_per_m: float = 2.5e-5
    position_capture_max_bank_correction_deg: float = 3.0
    turn_rate_command_scale: float = 1.0

    @model_validator(mode="after")
    def validate_route_values(self) -> RacetrackBinding:
        """Reject non-finite values and invalid owner-level constraints."""

        numeric_fields = (
            "straight_length_m",
            "turn_radius_m",
            "speed_m_s",
            "low_altitude_m",
            "high_altitude_m",
            "climb_rate_m_s",
            "descent_rate_m_s",
            "left_turn_bank_deg",
            "right_turn_bank_deg",
            "simulation_margin_s",
            "altitude_capture_gain_per_s",
            "altitude_capture_max_mps",
            "position_capture_gain",
            "position_capture_max_correction_mps",
            "gate_corridor_m",
            "gate_altitude_tolerance_m",
            "gate_speed_tolerance_mps",
            "position_capture_bank_gain_rad_per_m",
            "position_capture_max_bank_correction_deg",
            "turn_rate_command_scale",
        )
        invalid = [name for name in numeric_fields if not math.isfinite(getattr(self, name))]
        if invalid:
            raise ValueError("racetrack binding values must be finite: " + ", ".join(invalid))
        if self.high_altitude_m < self.low_altitude_m:
            raise ValueError("racetrack binding has high altitude below low altitude")
        nonnegative = (
            "simulation_margin_s",
            "altitude_capture_gain_per_s",
            "altitude_capture_max_mps",
            "position_capture_gain",
            "position_capture_max_correction_mps",
        )
        invalid_nonnegative = [name for name in nonnegative if getattr(self, name) < 0.0]
        if invalid_nonnegative:
            raise ValueError("racetrack binding requires non-negative values: " + ", ".join(invalid_nonnegative))
        return self
        ####

    ####


@dataclass(frozen=True, slots=True)
class RacetrackPhaseWindow:
    """One ordered phase window in the resolved racetrack."""

    name: str
    start_s: float
    end_s: float

    def __post_init__(self) -> None:
        if not self.name or not math.isfinite(self.start_s) or not math.isfinite(self.end_s):
            raise ValueError("racetrack phase windows require finite names and times")
        if self.start_s < 0.0 or self.end_s < self.start_s:
            raise ValueError("racetrack phase window must be ordered and non-negative")
        ####

    ####


@dataclass(frozen=True, slots=True)
class RacetrackGate:
    """Vehicle-independent local gate geometry for truth evaluation."""

    id: str
    north_m: float
    east_m: float
    altitude_m: float
    gate_normal_north: float
    gate_normal_east: float
    phase: str

    def target(self, speed_m_s: float) -> dict[str, float]:
        """Return canonical evaluator target channels for this gate."""

        return {
            "north_m": self.north_m,
            "east_m": self.east_m,
            "altitude_m": self.altitude_m,
            "speed_m_s": speed_m_s,
        }
        ####

    def gate_normal(self) -> list[float]:
        """Return the local-NED plane normal used by ``fly_by_gate``."""

        return [self.gate_normal_north, self.gate_normal_east, 0.0]
        ####

    ####


@dataclass(frozen=True, slots=True)
class ResolvedRacetrack:
    """Fully derived mission geometry for one vehicle binding."""

    template_id: str
    binding_id: str
    vehicle_id: str
    fidelity: RacetrackFidelity
    source_realization: str
    status: str
    straight_length_m: float
    turn_radius_m: float
    speed_m_s: float
    low_altitude_m: float
    high_altitude_m: float
    climb_rate_m_s: float
    descent_rate_m_s: float
    left_turn_bank_deg: float
    right_turn_bank_deg: float
    timing: RacetrackTimingEstimate
    simulation_margin_s: float
    altitude_capture_gain_per_s: float
    altitude_capture_max_mps: float
    position_capture_gain: float
    position_capture_max_correction_mps: float
    gate_corridor_m: float = 250.0
    gate_altitude_tolerance_m: float = 35.0
    gate_speed_tolerance_mps: float = 12.0
    position_capture_bank_gain_rad_per_m: float = 2.5e-5
    position_capture_max_bank_correction_deg: float = 3.0
    turn_rate_command_scale: float = 1.0

    @property
    def declared_duration_s(self) -> float:
        """Return the route duration used by the runtime reference."""

        return self.timing.total_time_s
        ####

    @property
    def horizon_s(self) -> float:
        """Return route duration plus post-gate observation margin."""

        return self.declared_duration_s + self.simulation_margin_s
        ####

    @property
    def phase_windows(self) -> tuple[RacetrackPhaseWindow, ...]:
        """Return ordered windows matching the runtime route phase index."""

        ends = (0.0, *self._cumulative_phase_ends())
        names = (
            "outbound-climb",
            "outbound-level",
            "left-turn",
            "inbound-descent",
            "inbound-level",
            "right-turn",
        )
        return tuple(RacetrackPhaseWindow(name, ends[index], ends[index + 1]) for index, name in enumerate(names))
        ####

    @property
    def gates(self) -> tuple[RacetrackGate, ...]:
        """Return phase-boundary gates in local north/east coordinates."""

        radius = self.turn_radius_m
        length = self.straight_length_m
        return (
            RacetrackGate("high-altitude-level-gate", 0.0, length, self.high_altitude_m, 0.0, 1.0, "outbound-level"),
            RacetrackGate("left-turn-exit-gate", 2.0 * radius, length, self.high_altitude_m, 0.0, -1.0, "left-turn"),
            RacetrackGate("low-altitude-level-gate", 2.0 * radius, 0.0, self.low_altitude_m, 0.0, -1.0, "inbound-level"),
            RacetrackGate("terminal-start-finish-gate", 0.0, 0.0, self.low_altitude_m, 0.0, 1.0, "right-turn"),
        )
        ####

    def route_attributes(self) -> dict[str, str]:
        """Return attributes accepted by the TAORYX ``mode=racetrack`` seam."""

        return {
            "mode": "racetrack",
            "racetrack-length-m": _number(self.straight_length_m),
            "racetrack-turn-radius-m": _number(self.turn_radius_m),
            "racetrack-speed-mps": _number(self.speed_m_s),
            "racetrack-low-altitude-m": _number(self.low_altitude_m),
            "racetrack-high-altitude-m": _number(self.high_altitude_m),
            "racetrack-climb-rate-mps": _number(self.climb_rate_m_s),
            "racetrack-descent-rate-mps": _number(self.descent_rate_m_s),
            "racetrack-left-bank-deg": _number(self.left_turn_bank_deg),
            "racetrack-right-bank-deg": _number(self.right_turn_bank_deg),
            "duration-s": _number(self.declared_duration_s),
            "racetrack-altitude-capture-gain-per-s": _number(self.altitude_capture_gain_per_s),
            "racetrack-altitude-capture-max-mps": _number(self.altitude_capture_max_mps),
            "position-capture-gain": _number(self.position_capture_gain),
            "position-capture-max-correction-mps": _number(self.position_capture_max_correction_mps),
        }
        ####

    def timing_manifest(self) -> dict[str, float]:
        """Return auditable timing values for packet metadata."""

        return {
            "straight_time_s": self.timing.straight_time_s,
            "turn_time_s": self.timing.turn_time_s,
            "climb_time_s": self.timing.climb_time_s,
            "descent_time_s": self.timing.descent_time_s,
            "outbound_level_time_s": self.timing.outbound_level_time_s,
            "inbound_level_time_s": self.timing.inbound_level_time_s,
            "route_duration_s": self.declared_duration_s,
            "simulation_horizon_s": self.horizon_s,
            "gate_corridor_m": self.gate_corridor_m,
            "gate_altitude_tolerance_m": self.gate_altitude_tolerance_m,
            "gate_speed_tolerance_mps": self.gate_speed_tolerance_mps,
            "position_capture_bank_gain_rad_per_m": self.position_capture_bank_gain_rad_per_m,
            "position_capture_max_bank_correction_deg": self.position_capture_max_bank_correction_deg,
            "turn_rate_command_scale": self.turn_rate_command_scale,
        }
        ####

    def _cumulative_phase_ends(self) -> tuple[float, ...]:
        elapsed = 0.0
        ends: list[float] = []
        for duration in self.timing.phase_durations():
            elapsed += duration
            ends.append(elapsed)
        return tuple(ends)
        ####

    ####


@dataclass(frozen=True, slots=True)
class RacetrackTemplateCatalog:
    """Resolved bindings loaded from a declarative racetrack catalog."""

    template_id: str
    bindings: dict[str, ResolvedRacetrack]

    def get(self, binding_id: str) -> ResolvedRacetrack:
        """Return one binding or raise a useful catalog error."""

        try:
            return self.bindings[binding_id]
        except KeyError as error:
            raise KeyError(f"unknown racetrack binding: {binding_id}") from error
        ####

    ####


def resolve_racetrack_binding(template_id: str, binding_id: str, binding: RacetrackBinding) -> ResolvedRacetrack:
    """Resolve one vehicle binding using the common powered-fixed-wing template."""

    timing = estimate_racetrack_timing(
        straight_length_m=binding.straight_length_m,
        turn_radius_m=binding.turn_radius_m,
        speed_m_s=binding.speed_m_s,
        altitude_delta_m=binding.high_altitude_m - binding.low_altitude_m,
        climb_rate_m_s=binding.climb_rate_m_s,
        descent_rate_m_s=binding.descent_rate_m_s,
    )
    return ResolvedRacetrack(
        template_id=template_id,
        binding_id=binding_id,
        vehicle_id=binding.vehicle_id,
        fidelity=binding.fidelity,
        source_realization=binding.source_realization,
        status=binding.status,
        straight_length_m=binding.straight_length_m,
        turn_radius_m=binding.turn_radius_m,
        speed_m_s=binding.speed_m_s,
        low_altitude_m=binding.low_altitude_m,
        high_altitude_m=binding.high_altitude_m,
        climb_rate_m_s=binding.climb_rate_m_s,
        descent_rate_m_s=binding.descent_rate_m_s,
        left_turn_bank_deg=binding.left_turn_bank_deg,
        right_turn_bank_deg=binding.right_turn_bank_deg,
        timing=timing,
        simulation_margin_s=binding.simulation_margin_s,
        altitude_capture_gain_per_s=binding.altitude_capture_gain_per_s,
        altitude_capture_max_mps=binding.altitude_capture_max_mps,
        position_capture_gain=binding.position_capture_gain,
        position_capture_max_correction_mps=binding.position_capture_max_correction_mps,
        gate_corridor_m=binding.gate_corridor_m,
        gate_altitude_tolerance_m=binding.gate_altitude_tolerance_m,
        gate_speed_tolerance_mps=binding.gate_speed_tolerance_mps,
        position_capture_bank_gain_rad_per_m=binding.position_capture_bank_gain_rad_per_m,
        position_capture_max_bank_correction_deg=binding.position_capture_max_bank_correction_deg,
        turn_rate_command_scale=binding.turn_rate_command_scale,
    )
    ####


def load_racetrack_template_catalog(path: Path) -> RacetrackTemplateCatalog:
    """Load and resolve the declarative racetrack binding catalog."""

    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        raise ValueError(f"unsupported racetrack catalog schema in {path}")
    template = payload.get("template")
    if not isinstance(template, dict) or not template.get("id"):
        raise ValueError("racetrack catalog requires template.id")
    template_id = str(template["id"])
    bindings_payload = payload.get("bindings")
    if not isinstance(bindings_payload, dict) or not bindings_payload:
        raise ValueError("racetrack catalog requires non-empty bindings")
    bindings = {
        str(binding_id): resolve_racetrack_binding(
            template_id,
            str(binding_id),
            RacetrackBinding.model_validate(values),
        )
        for binding_id, values in bindings_payload.items()
    }
    return RacetrackTemplateCatalog(template_id=template_id, bindings=bindings)
    ####


def _number(value: float) -> str:
    return format(value, ".16g")
    ####


__all__ = [
    "RACETRACK_FIDELITIES",
    "RacetrackBinding",
    "RacetrackGate",
    "RacetrackFidelity",
    "RacetrackPhaseWindow",
    "RacetrackTemplateCatalog",
    "ResolvedRacetrack",
    "load_racetrack_template_catalog",
    "resolve_racetrack_binding",
]
####
