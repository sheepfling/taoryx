"""Evidence-bounded Anduril-inspired flight-dynamics surrogate adapters.

This module intentionally starts at the common kinematic boundary.  It loads
the versioned parameter pack, performs physics sanity checks, and provides a
deterministic point-mass / attitude-response stepper.  It does not claim
manufacturer-level aerodynamics, torque-level 6DOF, or executable support for
an undisclosed aircraft.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml

ROOT = Path(__file__).resolve().parents[2]
PARAMETER_PACK = ROOT / "verification/anduril_surrogate_parameters_v1.yaml"
STANDARD_GRAVITY_M_S2 = 9.80665
SEA_LEVEL_DENSITY_KG_M3 = 1.225


@dataclass(frozen=True, slots=True)
class SurrogateState:
    """Minimal deterministic translational and attitude-response state."""

    time_s: float = 0.0
    position_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity_m_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    roll_rate_rad_s: float = 0.0
    pitch_rate_rad_s: float = 0.0
    yaw_rate_rad_s: float = 0.0
    energy_j: float | None = None

    def __post_init__(self) -> None:
        values = (*self.position_m, *self.velocity_m_s, self.time_s, self.roll_rad, self.pitch_rad, self.yaw_rad, self.roll_rate_rad_s, self.pitch_rate_rad_s, self.yaw_rate_rad_s)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("surrogate state values must be finite")
        if self.energy_j is not None and (not math.isfinite(self.energy_j) or self.energy_j < 0.0):
            raise ValueError("energy_j must be finite and nonnegative")
        ####
    ####


@dataclass(frozen=True, slots=True)
class SurrogateControl:
    """Normalized acceleration and attitude commands at the adapter boundary."""

    acceleration_m_s2: tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw_rate_rad_s: float = 0.0
    roll_command_rad: float = 0.0
    pitch_command_rad: float = 0.0
    yaw_command_rad: float | None = None


@dataclass(frozen=True, slots=True)
class PhysicsCheck:
    """One named sanity check from a surrogate validation report."""

    name: str
    passed: bool
    value: float | None = None
    limit: float | None = None
    detail: str = ""


@dataclass(frozen=True, slots=True)
class SurrogateValidationReport:
    """Machine-readable result of validating one catalog configuration."""

    vehicle_id: str
    checks: tuple[PhysicsCheck, ...]

    @property
    def passed(self) -> bool:
        return all(check.passed for check in self.checks)
        ####

    @property
    def failures(self) -> tuple[PhysicsCheck, ...]:
        return tuple(check for check in self.checks if not check.passed)
        ####
    ####


def load_surrogate_pack(path: str | Path = PARAMETER_PACK) -> dict[str, Any]:
    """Load the versioned surrogate pack without promoting execution claims."""

    document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema") != "taoryx.flight-dynamics-surrogate-parameters/v1":
        raise ValueError("unsupported Anduril surrogate parameter-pack schema")
    if not isinstance(document.get("vehicles"), dict) or not document["vehicles"]:
        raise ValueError("surrogate parameter pack must contain vehicles")
    contract = document.get("parameter_contract")
    if not isinstance(contract, dict):
        raise ValueError("surrogate parameter pack must declare parameter_contract")
    grades = contract.get("evidence_grades")
    if grades != ["P", "D", "E", "S"]:
        raise ValueError("surrogate parameter contract must declare ordered P/D/E/S grades")
    backends = document.get("energy_backends")
    if not isinstance(backends, dict) or not {"battery_electric", "fuel_burning", "series_hybrid", "selectable"} <= set(backends):
        raise ValueError("surrogate parameter pack must declare all energy backends")
    required_fields = {"energy_backend", "flight_modes", "resource_policy", "parameter_ledger"}
    for vehicle_id, definition in document["vehicles"].items():
        if not isinstance(definition, dict):
            raise TypeError(f"surrogate definition {vehicle_id!r} must be a mapping")
        if not required_fields <= set(definition):
            missing = sorted(required_fields - set(definition))
            raise ValueError(f"surrogate definition {vehicle_id!r} is missing data-contract fields: {missing}")
        ledger = definition["parameter_ledger"]
        if not isinstance(ledger, list) or not ledger:
            raise ValueError(f"surrogate definition {vehicle_id!r} must contain a non-empty parameter ledger")
        for record in ledger:
            if not isinstance(record, dict):
                raise TypeError(f"surrogate parameter ledger for {vehicle_id!r} must contain mappings")
            if not {"path", "value", "unit", "evidence_grade", "source_ref", "uncertainty_policy"} <= set(record):
                raise ValueError(f"surrogate parameter ledger for {vehicle_id!r} contains an incomplete record")
            if record["evidence_grade"] not in grades:
                raise ValueError(f"surrogate parameter ledger for {vehicle_id!r} contains an invalid evidence grade")
    return document
    ####


def surrogate_definition(vehicle_id: str, *, pack: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Return one surrogate definition with a diagnostic unknown-ID error."""

    document = dict(pack or load_surrogate_pack())
    vehicles = document.get("vehicles", {})
    try:
        definition = vehicles[vehicle_id]
    except KeyError as error:
        raise KeyError(f"unknown Anduril surrogate vehicle {vehicle_id!r}") from error
    if not isinstance(definition, dict):
        raise TypeError(f"surrogate definition {vehicle_id!r} must be a mapping")
    return definition
    ####


def _scalar_or_midpoint(value: Any) -> float:
    if isinstance(value, (list, tuple)):
        if len(value) != 2:
            raise ValueError("range-valued surrogate fields must have two endpoints")
        return (float(value[0]) + float(value[1])) / 2.0
    return float(value)
    ####


def validate_surrogate(vehicle_id: str, *, pack: Mapping[str, Any] | None = None) -> SurrogateValidationReport:
    """Run data-level physics gates before an executable adapter is promoted."""

    definition = surrogate_definition(vehicle_id, pack=pack)
    checks: list[PhysicsCheck] = []
    mass = definition.get("mass", {})
    gross_mass = _scalar_or_midpoint(mass.get("gross_kg", 0.0))
    checks.append(PhysicsCheck("positive_gross_mass", gross_mass > 0.0, gross_mass, 0.0))

    family = str(definition.get("family", ""))
    propulsion = definition.get("propulsion", {})
    loading_states = definition.get("loading_states", {})
    nominal_loading = loading_states.get("nominal", {}) if isinstance(loading_states, dict) else {}
    if family in {"multirotor_or_helicopter", "vectored_thrust_hybrid", "tiltrotor"}:
        hover = propulsion.get("practical_hover_power_kw", nominal_loading.get("hover_power_kw", definition.get("performance", {}).get("hover_power_kw")))
        peak = propulsion.get("installed_peak_power_kw")
        hover_scale = 1.0
        peak_scale = 1.0
        if hover is None and "practical_hover_power_mw" in propulsion:
            hover = propulsion["practical_hover_power_mw"]
            hover_scale = 1000.0
        if peak is None and "installed_peak_power_mw" in propulsion:
            peak = propulsion["installed_peak_power_mw"]
            peak_scale = 1000.0
        if hover is None:
            hover = definition.get("performance", {}).get("hover_power_kw")
        if peak is None:
            peak = definition.get("energy", {}).get("installed_peak_power_kw")
        if hover is not None and peak is not None:
            hover_value = _scalar_or_midpoint(hover) * hover_scale
            peak_value = _scalar_or_midpoint(peak) * peak_scale
            checks.append(PhysicsCheck("hover_power_margin", hover_value <= 0.65 * peak_value, hover_value / peak_value, 0.65))
        elif propulsion.get("static_thrust_n") is not None:
            checks.append(PhysicsCheck("hover_power_data_present", True, detail="static thrust is present; power model remains a later gate"))
        else:
            checks.append(PhysicsCheck("hover_power_data_present", False, detail="hover and installed peak power are required for hover-capable families"))

    geometry = definition.get("geometry", {})
    performance = definition.get("performance", {})
    aero = definition.get("aerodynamics", {})
    if family == "conventional_fixed_wing" and {"reference_area_m2", "stall_speed_m_s"} <= set(geometry) | set(performance):
        area = _scalar_or_midpoint(geometry.get("reference_area_m2", 0.0))
        stall = _scalar_or_midpoint(performance.get("stall_speed_m_s", 0.0))
        cl_max = _scalar_or_midpoint(aero.get("cl_max", 1.0))
        derived_stall = math.sqrt(2.0 * gross_mass * STANDARD_GRAVITY_M_S2 / (SEA_LEVEL_DENSITY_KG_M3 * area * cl_max))
        relative_error = abs(derived_stall - stall) / max(stall, 1.0e-12)
        checks.append(PhysicsCheck("stall_speed_consistency", relative_error <= 0.03, relative_error, 0.03, f"derived={derived_stall:.3f} m/s"))
    elif family == "conventional_fixed_wing":
        checks.append(PhysicsCheck("fixed_wing_stall_inputs", False, detail="reference area, stall speed, and CLmax are required"))

    if definition.get("claim_boundary") == "concept_level_only":
        checks.append(PhysicsCheck("concept_boundary_preserved", str(definition.get("execution_status")) == "concept_adapter_smoke_supported"))
    return SurrogateValidationReport(vehicle_id, tuple(checks))
    ####


def _response_time(vehicle: Mapping[str, Any]) -> float:
    control = vehicle.get("control", {})
    value = control.get("response_time_s", 0.25)
    return max(0.02, _scalar_or_midpoint(value))
    ####


def step_surrogate(
    vehicle_id: str,
    state: SurrogateState,
    control: SurrogateControl,
    dt_s: float,
    *,
    fidelity: str = "point_mass_3dof",
    pack: Mapping[str, Any] | None = None,
) -> SurrogateState:
    """Advance one deterministic kinematic or named response-profile step.

    The control frame is deliberately acceleration-based so all four shared
    equation families can use the same provider seam. Family-specific
    aerodynamic and actuator overlays are added only after their own data
    contracts exist.
    """

    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("dt_s must be finite and positive")
    if fidelity not in {"point_mass_3dof", "pseudo_6dof"}:
        raise ValueError("Anduril surrogate adapter supports point_mass_3dof or pseudo_6dof")
    vehicle = surrogate_definition(vehicle_id, pack=pack)
    acceleration = tuple(float(value) for value in control.acceleration_m_s2)
    if len(acceleration) != 3 or not all(math.isfinite(value) for value in acceleration):
        raise ValueError("acceleration_m_s2 must contain three finite values")
    new_velocity = (
        state.velocity_m_s[0] + acceleration[0] * dt_s,
        state.velocity_m_s[1] + acceleration[1] * dt_s,
        state.velocity_m_s[2] + acceleration[2] * dt_s,
    )
    new_position = (
        state.position_m[0] + state.velocity_m_s[0] * dt_s + 0.5 * acceleration[0] * dt_s * dt_s,
        state.position_m[1] + state.velocity_m_s[1] * dt_s + 0.5 * acceleration[1] * dt_s * dt_s,
        state.position_m[2] + state.velocity_m_s[2] * dt_s + 0.5 * acceleration[2] * dt_s * dt_s,
    )
    if fidelity == "point_mass_3dof":
        return SurrogateState(state.time_s + dt_s, new_position, new_velocity, energy_j=state.energy_j)

    response_time = _response_time(vehicle)
    roll_rate = state.roll_rate_rad_s + (control.roll_command_rad - state.roll_rate_rad_s) * min(1.0, dt_s / response_time)
    pitch_rate = state.pitch_rate_rad_s + (control.pitch_command_rad - state.pitch_rate_rad_s) * min(1.0, dt_s / response_time)
    yaw_target = control.yaw_command_rad if control.yaw_command_rad is not None else state.yaw_rad + control.yaw_rate_rad_s * dt_s
    yaw_rate = state.yaw_rate_rad_s + (yaw_target - state.yaw_rate_rad_s) * min(1.0, dt_s / response_time)
    return SurrogateState(
        state.time_s + dt_s,
        new_position,
        new_velocity,
        state.roll_rad + roll_rate * dt_s,
        state.pitch_rad + pitch_rate * dt_s,
        state.yaw_rad + yaw_rate * dt_s,
        roll_rate,
        pitch_rate,
        yaw_rate,
        state.energy_j,
    )
    ####


__all__ = [
    "PARAMETER_PACK",
    "PhysicsCheck",
    "SurrogateControl",
    "SurrogateState",
    "SurrogateValidationReport",
    "load_surrogate_pack",
    "step_surrogate",
    "surrogate_definition",
    "validate_surrogate",
]
