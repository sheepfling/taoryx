"""Capability-scaled mission planning for powered fixed-wing vehicles.

This compiler resolves an initial racetrack geometry from declared vehicle
capability data and semantic mission intent.  It intentionally stops before a
controller or nonlinear plant is selected: a feasible kinematic route is not
evidence that any particular fidelity realization can fly it.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Final

import yaml

from taoryx.racetrack_template import RACETRACK_FIDELITIES, RacetrackFidelity, ResolvedRacetrack, resolve_racetrack_binding

STANDARD_GRAVITY_M_S2: Final[float] = 9.80665


@dataclass(frozen=True, slots=True)
class PoweredFixedWingCapability:
    """Declared planning capability at one operating condition.

    Values can be source-backed, trim-derived, or engineering estimates, but
    ``provenance`` and ``evidence_class`` make that distinction visible to a
    route compiler and its artifact.
    """

    vehicle_id: str
    operating_point_id: str
    minimum_speed_m_s: float
    nominal_speed_m_s: float
    maximum_speed_m_s: float
    maximum_bank_deg: float
    maximum_climb_rate_m_s: float
    maximum_descent_rate_m_s: float
    provenance: str
    evidence_class: str

    def __post_init__(self) -> None:
        if not self.vehicle_id or not self.operating_point_id:
            raise ValueError("vehicle_id and operating_point_id are required")
        values = {
            "minimum_speed_m_s": self.minimum_speed_m_s,
            "nominal_speed_m_s": self.nominal_speed_m_s,
            "maximum_speed_m_s": self.maximum_speed_m_s,
            "maximum_climb_rate_m_s": self.maximum_climb_rate_m_s,
            "maximum_descent_rate_m_s": self.maximum_descent_rate_m_s,
        }
        for name, value in values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not self.minimum_speed_m_s <= self.nominal_speed_m_s <= self.maximum_speed_m_s:
            raise ValueError("speed capability must satisfy minimum <= nominal <= maximum")
        if not math.isfinite(self.maximum_bank_deg) or not 0.0 < self.maximum_bank_deg < 89.0:
            raise ValueError("maximum_bank_deg must be in (0, 89)")
        if not self.provenance or not self.evidence_class:
            raise ValueError("capability provenance and evidence_class are required")
        ####
    ####

@dataclass(frozen=True, slots=True)
class PoweredFixedWingRacetrackIntent:
    """Vehicle-neutral intent for the common climb/turn/descent racetrack."""

    id: str
    low_altitude_m: float
    high_altitude_m: float
    requested_speed_m_s: float | None = None
    requested_bank_deg: float | None = None
    minimum_straight_length_m: float = 0.0
    level_dwell_s: float = 30.0
    turn_radius_margin: float = 1.10
    simulation_margin_s: float = 30.0
    gate_corridor_m: float = 250.0
    gate_altitude_tolerance_m: float = 35.0
    gate_speed_tolerance_mps: float = 12.0

    def __post_init__(self) -> None:
        if not self.id:
            raise ValueError("mission intent id is required")
        values = {
            "low_altitude_m": self.low_altitude_m,
            "high_altitude_m": self.high_altitude_m,
            "minimum_straight_length_m": self.minimum_straight_length_m,
            "level_dwell_s": self.level_dwell_s,
            "turn_radius_margin": self.turn_radius_margin,
            "simulation_margin_s": self.simulation_margin_s,
            "gate_corridor_m": self.gate_corridor_m,
            "gate_altitude_tolerance_m": self.gate_altitude_tolerance_m,
            "gate_speed_tolerance_mps": self.gate_speed_tolerance_mps,
        }
        for name, value in values.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.high_altitude_m < self.low_altitude_m:
            raise ValueError("high altitude must not be below low altitude")
        if self.level_dwell_s <= 0.0:
            raise ValueError("level_dwell_s must be positive")
        if self.turn_radius_margin < 1.0:
            raise ValueError("turn_radius_margin must be at least one")
        for requested_name, requested_value in (
            ("requested_speed_m_s", self.requested_speed_m_s),
            ("requested_bank_deg", self.requested_bank_deg),
        ):
            if requested_value is not None and (not math.isfinite(requested_value) or requested_value <= 0.0):
                raise ValueError(f"{requested_name} must be finite and positive when supplied")
        ####
    ####

@dataclass(frozen=True, slots=True)
class CapabilityScaledRacetrack:
    """Compiled route plus auditable clipping and planning metrics."""

    capability: PoweredFixedWingCapability
    intent: PoweredFixedWingRacetrackIntent
    route: ResolvedRacetrack
    status: str
    diagnostics: tuple[str, ...]
    minimum_turn_radius_m: float
    minimum_straight_length_m: float

    def manifest(self) -> dict[str, Any]:
        """Return machine-readable provenance for a compiled mission."""

        return {
            "schema_version": "taoryx.powered-fixed-wing-mission-compiler/v1",
            "status": self.status,
            "resolved_route": {
                "template_id": self.route.template_id,
                "binding_id": self.route.binding_id,
                "vehicle_id": self.route.vehicle_id,
                "fidelity": self.route.fidelity,
                "source_realization": self.route.source_realization,
                "binding_status": self.route.status,
            },
            "capability": asdict(self.capability),
            "intent": asdict(self.intent),
            "diagnostics": list(self.diagnostics),
            "derived": {
                "minimum_turn_radius_m": self.minimum_turn_radius_m,
                "minimum_straight_length_m": self.minimum_straight_length_m,
                "selected_turn_radius_m": self.route.turn_radius_m,
                "selected_straight_length_m": self.route.straight_length_m,
                "selected_speed_m_s": self.route.speed_m_s,
                "selected_bank_deg": abs(self.route.right_turn_bank_deg),
                **self.route.timing_manifest(),
            },
            "route_attributes": self.route.route_attributes(),
            "nonclaim": "This planning artifact does not validate controller tracking, nonlinear dynamics, allocation, or physical effectors.",
        }
        ####
    ####


def compile_powered_fixed_wing_racetrack(
    capability: PoweredFixedWingCapability,
    intent: PoweredFixedWingRacetrackIntent,
    *,
    binding_id: str | None = None,
    fidelity: RacetrackFidelity = "point_mass_3dof",
    source_realization: str = "capability_scaled_planning_only",
) -> CapabilityScaledRacetrack:
    """Compile a conservative route without silently exceeding capability.

    Requested speed and bank are clipped to the declared operating-point
    capability and reported.  The geometry always reserves a level dwell after
    both vertical maneuvers, which isolates pitch/energy work from turn work.
    """

    if fidelity not in RACETRACK_FIDELITIES:
        raise ValueError(f"unsupported racetrack fidelity: {fidelity}")
    diagnostics: list[str] = []
    requested_speed = capability.nominal_speed_m_s if intent.requested_speed_m_s is None else intent.requested_speed_m_s
    speed = _clip(requested_speed, capability.minimum_speed_m_s, capability.maximum_speed_m_s)
    if not math.isclose(speed, requested_speed):
        diagnostics.append(
            f"requested speed {requested_speed:.6g} m/s clipped to declared capability [{capability.minimum_speed_m_s:.6g}, {capability.maximum_speed_m_s:.6g}] m/s"
        )
    requested_bank = capability.maximum_bank_deg if intent.requested_bank_deg is None else intent.requested_bank_deg
    bank = min(requested_bank, capability.maximum_bank_deg)
    if not math.isclose(bank, requested_bank):
        diagnostics.append(f"requested bank {requested_bank:.6g} deg clipped to maximum {capability.maximum_bank_deg:.6g} deg")

    bank_rad = math.radians(bank)
    minimum_turn_radius = speed**2 / (STANDARD_GRAVITY_M_S2 * math.tan(bank_rad))
    selected_turn_radius = intent.turn_radius_margin * minimum_turn_radius
    altitude_delta = intent.high_altitude_m - intent.low_altitude_m
    climb_time = altitude_delta / capability.maximum_climb_rate_m_s
    descent_time = altitude_delta / capability.maximum_descent_rate_m_s
    minimum_straight_length = speed * max(climb_time + intent.level_dwell_s, descent_time + intent.level_dwell_s)
    selected_straight_length = max(intent.minimum_straight_length_m, minimum_straight_length)
    if intent.minimum_straight_length_m > 0.0 and intent.minimum_straight_length_m < minimum_straight_length:
        diagnostics.append(
            f"requested straight length {intent.minimum_straight_length_m:.6g} m increased to {minimum_straight_length:.6g} m to isolate climb/descent and dwell"
        )
    if intent.requested_bank_deg is None:
        diagnostics.append("no bank requested; selected declared maximum bank for planning")
    status = "capability_clipped" if diagnostics else "capability_feasible"
    values: dict[str, Any] = {
        "vehicle_id": capability.vehicle_id,
        "fidelity": fidelity,
        "straight_length_m": selected_straight_length,
        "turn_radius_m": selected_turn_radius,
        "speed_m_s": speed,
        "low_altitude_m": intent.low_altitude_m,
        "high_altitude_m": intent.high_altitude_m,
        "climb_rate_m_s": capability.maximum_climb_rate_m_s,
        "descent_rate_m_s": capability.maximum_descent_rate_m_s,
        "left_turn_bank_deg": -bank,
        "right_turn_bank_deg": bank,
        "simulation_margin_s": intent.simulation_margin_s,
        "gate_corridor_m": intent.gate_corridor_m,
        "gate_altitude_tolerance_m": intent.gate_altitude_tolerance_m,
        "gate_speed_tolerance_mps": intent.gate_speed_tolerance_mps,
        "source_realization": source_realization,
        "status": status,
    }
    resolved_binding_id = binding_id or f"{capability.vehicle_id}-{intent.id}-{fidelity}"
    route = resolve_racetrack_binding("powered_fixed_wing_racetrack_v1", resolved_binding_id, values)
    return CapabilityScaledRacetrack(
        capability=capability,
        intent=intent,
        route=route,
        status=status,
        diagnostics=tuple(diagnostics),
        minimum_turn_radius_m=minimum_turn_radius,
        minimum_straight_length_m=minimum_straight_length,
    )
    ####


def load_powered_fixed_wing_mission_profiles(path: Path) -> dict[str, tuple[PoweredFixedWingCapability, PoweredFixedWingRacetrackIntent]]:
    """Load declared capability/intention pairs for first-mission planning."""

    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        raise ValueError(f"unsupported powered fixed-wing mission profile schema in {path}")
    profiles_payload = payload.get("profiles")
    if not isinstance(profiles_payload, dict) or not profiles_payload:
        raise ValueError("powered fixed-wing mission profiles require a non-empty profiles mapping")
    profiles: dict[str, tuple[PoweredFixedWingCapability, PoweredFixedWingRacetrackIntent]] = {}
    for profile_id, value in profiles_payload.items():
        if not isinstance(value, dict):
            raise ValueError(f"profile {profile_id!r} must be a mapping")
        capability_values = value.get("capability")
        intent_values = value.get("intent")
        if not isinstance(capability_values, dict) or not isinstance(intent_values, dict):
            raise ValueError(f"profile {profile_id!r} requires capability and intent mappings")
        profiles[str(profile_id)] = (
            PoweredFixedWingCapability(**capability_values),
            PoweredFixedWingRacetrackIntent(**intent_values),
        )
    return profiles
    ####


def resolve_powered_fixed_wing_mission_profile(
    path: Path,
    profile_id: str,
    fidelity: RacetrackFidelity,
) -> tuple[PoweredFixedWingCapability, PoweredFixedWingRacetrackIntent]:
    """Resolve a profile with any declared fidelity-specific mission intent.

    A family may use distinct validated altitude/configuration corridors at
    different tiers.  The capability profile remains shared where meaningful,
    but a lower-fidelity lane must not inherit an incompatible direct-wrench
    altitude just because both describe the same named aircraft.
    """

    profiles = load_powered_fixed_wing_mission_profiles(path)
    if profile_id not in profiles:
        raise KeyError(f"unknown powered fixed-wing mission profile: {profile_id}")
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or not isinstance(raw.get("profiles"), dict):
        raise ValueError(f"invalid powered fixed-wing mission profile schema in {path}")
    profile = raw["profiles"].get(profile_id)
    if not isinstance(profile, dict):
        raise ValueError(f"profile {profile_id!r} must be a mapping")
    overrides = profile.get("fidelity_intent_overrides", {})
    override = overrides.get(fidelity, {}) if isinstance(overrides, dict) else {}
    if not isinstance(override, dict):
        raise ValueError(f"profile {profile_id!r} has a non-mapping fidelity intent override for {fidelity!r}")
    capability, intent = profiles[profile_id]
    return capability, PoweredFixedWingRacetrackIntent(**{**asdict(intent), **override})
    ####


def compare_compiled_racetrack_to_baseline(
    compiled: CapabilityScaledRacetrack,
    baseline: ResolvedRacetrack,
) -> dict[str, Any]:
    """Compare a proposed route with an existing hand-selected binding.

    This preserves legacy qualification geometry while making incompatible
    planning assumptions visible.  It does not declare the generated geometry
    better; nonlinear execution and truth evaluation decide that separately.
    """

    findings: list[str] = []
    if baseline.vehicle_id != compiled.capability.vehicle_id:
        findings.append(
            f"baseline vehicle {baseline.vehicle_id!r} does not match capability vehicle {compiled.capability.vehicle_id!r}"
        )
    if baseline.turn_radius_m < compiled.minimum_turn_radius_m:
        findings.append(
            "baseline turn radius is below the capability-derived minimum at the selected speed and bank limit"
        )
    if baseline.straight_length_m < compiled.minimum_straight_length_m:
        findings.append(
            "baseline straight length is too short to isolate both vertical maneuvers and the declared level dwell"
        )
    return {
        "baseline_binding_id": baseline.binding_id,
        "vehicle_match": baseline.vehicle_id == compiled.capability.vehicle_id,
        "baseline_turn_radius_m": baseline.turn_radius_m,
        "compiled_turn_radius_m": compiled.route.turn_radius_m,
        "minimum_turn_radius_m": compiled.minimum_turn_radius_m,
        "baseline_straight_length_m": baseline.straight_length_m,
        "compiled_straight_length_m": compiled.route.straight_length_m,
        "minimum_straight_length_m": compiled.minimum_straight_length_m,
        "baseline_duration_s": baseline.declared_duration_s,
        "compiled_duration_s": compiled.route.declared_duration_s,
        "findings": findings,
        "status": "baseline_review_required" if findings else "baseline_planning_consistent",
    }
    ####


def _clip(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)
    ####


__all__ = [
    "CapabilityScaledRacetrack",
    "PoweredFixedWingCapability",
    "PoweredFixedWingRacetrackIntent",
    "compare_compiled_racetrack_to_baseline",
    "compile_powered_fixed_wing_racetrack",
    "load_powered_fixed_wing_mission_profiles",
    "resolve_powered_fixed_wing_mission_profile",
]
####
