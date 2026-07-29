"""Family adapters that bind declarative trim worklists to source evaluators.

The orchestration layer deliberately stops at :class:`TrimSpec`.  This module
contains the small amount of family-specific physics binding needed to turn
the F-16 and HL-20 pilot worklists into reproducible solve evidence.  It does
not create a fallback model when a source evaluator is unavailable.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Literal

from .trajectory import (
    DAVEMLFixedWingLoadBinding,
    DAVEMLInertiaBinding,
    load_daveml_atmosphere,
    load_daveml_family_graph,
    load_daveml_trim_binding,
)
from .trim import TrimResult, solve_trim
from .vehicle_registry import ROOT
from .vehicle_trim_orchestration import TrimWorkItem, orchestrate_trim_recipe

TrimSolveStatus = Literal["verified", "blocked", "failed"]


@dataclass(frozen=True, slots=True)
class TrimSolvePoint:
    """One family-adapter trim result."""

    point_id: str
    success: bool
    state: Mapping[str, float]
    controls: Mapping[str, float]
    residuals: Mapping[str, float]
    max_residual: float
    iterations: int
    solver_status: int
    solver_message: str

    @classmethod
    def from_result(cls, point_id: str, result: TrimResult) -> TrimSolvePoint:
        """Convert the generic solver result into stable scalar JSON values."""

        return cls(
            point_id,
            result.success,
            {str(key): float(value) for key, value in result.state.items()},
            {str(key): float(value) for key, value in result.controls.items()},
            {str(key): float(value) for key, value in result.residuals.items()},
            float(result.max_residual),
            int(result.iterations),
            int(result.status),
            str(result.message),
        )

    def as_dict(self) -> dict[str, object]:
        """Return a stable evidence record."""

        return {
            "point_id": self.point_id,
            "success": self.success,
            "state": dict(self.state),
            "controls": dict(self.controls),
            "residuals": dict(self.residuals),
            "max_residual": self.max_residual,
            "iterations": self.iterations,
            "solver_status": self.solver_status,
            "solver_message": self.solver_message,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class VehicleTrimSolveReport:
    """Aggregate adapter-bound trim evidence."""

    family_id: str
    adapter: str | None
    status: TrimSolveStatus
    claim_boundary: str
    points: tuple[TrimSolvePoint, ...]
    findings: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        """Return a stable machine-readable report."""

        return {
            "schema_version": "taoryx.vehicle-trim-solve/v1",
            "family_id": self.family_id,
            "adapter": self.adapter,
            "status": self.status,
            "claim_boundary": self.claim_boundary,
            "points": [point.as_dict() for point in self.points],
            "findings": list(self.findings),
        }
        ####
    ####


def _f16_bindings() -> tuple[DAVEMLFixedWingLoadBinding, Any, float, Any]:
    sidecar = ROOT / "families/reference_f16_s119/plant/daveml-import.json"
    aero = load_daveml_trim_binding(
        sidecar,
        role="aerodynamics",
        state_inputs={"alpha_deg": "alpha"},
        control_inputs={"elevator_deg": "el"},
        residual_outputs={"cx": "cx", "cy": "cy", "cz": "cz", "cl": "cl", "cm": "cm", "cn": "cn"},
        fixed_inputs={"vt": 500.0, "beta": 0.0, "p": 0.0, "q": 0.0, "r": 0.0, "ail": 0.0, "rdr": 0.0, "xcg": 0.35},
    )
    propulsion = load_daveml_trim_binding(
        sidecar,
        role="propulsion",
        state_inputs={"altitude_ft": "altitudeMSL", "mach": "mach"},
        control_inputs={"power_pct": "powerLeverAngle"},
        residual_outputs={"thrust_lbf": "thrustBodyForce_X"},
    )
    atmosphere = load_daveml_atmosphere(ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml")
    inertia_graph = load_daveml_family_graph(sidecar, role="mass_properties")
    mass = float(DAVEMLInertiaBinding(inertia_graph).evaluate()["mass_kg"])
    loads = DAVEMLFixedWingLoadBinding(
        aerodynamics=aero,
        reference_area_m2=27.870912,
        mean_aerodynamic_chord_m=3.450336,
        span_m=9.144,
        dynamic_pressure_pa=1.0,
        propulsion=propulsion,
    )
    return loads, atmosphere, mass, aero
    ####


def _solve_f16_work_item(item: TrimWorkItem, loads: DAVEMLFixedWingLoadBinding, atmosphere: Any, mass: float) -> TrimSolvePoint:
    environment = item.trim_spec.operating_point
    altitude = float(environment.get("geometric_altitude_m", 0.0))
    true_airspeed = float(environment.get("true_airspeed_m_s", 0.0))
    if true_airspeed <= 0.0:
        raise ValueError(f"{item.point_id}: true airspeed must be positive")

    def evaluate(state: Mapping[str, float], controls: Mapping[str, float]) -> dict[str, float]:
        pitch_rad = math.radians(state["alpha_deg"])
        sound_speed = float(atmosphere.evaluate(altitude)["speed_of_sound_m_s"])
        values = loads.evaluate_with_atmosphere(
            {"alpha_deg": state["alpha_deg"], "altitude_ft": altitude / 0.3048, "mach": true_airspeed / sound_speed},
            controls,
            atmosphere,
            geometric_altitude_m=altitude,
            true_airspeed_m_s=true_airspeed,
        )
        return {
            "force_x_n": float(values["total_force_x_n"]) - mass * 9.80665 * math.sin(pitch_rad),
            "force_z_n": float(values["total_force_z_n"]) + mass * 9.80665 * math.cos(pitch_rad),
            "moment_y_nm": float(values["total_moment_y_nm"]),
        }

    result = solve_trim(item.trim_spec, evaluate, max_nfev=500, residual_tolerance=1.0e-10, acceptance_tolerance=1.0e-6)
    return TrimSolvePoint.from_result(item.point_id, result)
    ####


def _hl20_binding() -> Any:
    sidecar = ROOT / "families/reference_hl20_mod_k/plant/daveml-import.json"
    return load_daveml_trim_binding(
        sidecar,
        role="aerodynamics",
        state_inputs={"alpha_deg": "ALP_UNLIM"},
        control_inputs={},
        residual_outputs={"pitch_cm": "CM"},
        fixed_inputs={
            "BETA": 0.0,
            "XMACH": 1.0,
            "PB": 0.0,
            "QB": 0.0,
            "RB": 0.0,
            "VRW": 100.0,
            "H_rwy": 0.0,
            "DBFUL": 0.0,
            "DBFUR": 0.0,
            "DBFLL": 0.0,
            "DBFLR": 0.0,
            "DWFL": 0.0,
            "DWFR": 0.0,
            "DRUD": 0.0,
            "DLG": 0.0,
        },
    )
    ####


def _solve_hl20_work_item(item: TrimWorkItem, binding: Any) -> TrimSolvePoint:
    result = solve_trim(item.trim_spec, binding.as_evaluator(), max_nfev=100, residual_tolerance=1.0e-10, acceptance_tolerance=1.0e-10)
    return TrimSolvePoint.from_result(item.point_id, result)
    ####


def solve_vehicle_trim_evidence(family_id: str) -> VehicleTrimSolveReport:
    """Bind a pilot worklist to its source evaluator and solve every point."""

    orchestration = orchestrate_trim_recipe(family_id)
    if orchestration.status == "blocked":
        return VehicleTrimSolveReport(family_id, None, "blocked", orchestration.claim_boundary, (), tuple(item.message for item in orchestration.errors))
    items = orchestration.work_items
    if family_id == "reference_f16_s119":
        loads, atmosphere, mass, _aero = _f16_bindings()
        points = tuple(_solve_f16_work_item(item, loads, atmosphere, mass) for item in items)
        adapter = "taoryx.adapters.reference_f16_s119.daveml_equilibrium"
    elif family_id == "reference_hl20_mod_k":
        binding = _hl20_binding()
        points = tuple(_solve_hl20_work_item(item, binding) for item in items)
        adapter = "taoryx.adapters.reference_hl20_mod_k.daveml_pitch_channel"
    else:
        return VehicleTrimSolveReport(family_id, orchestration.recipe_id, "blocked", orchestration.claim_boundary, (), ("no family trim adapter is registered",))
    failures = tuple(point.point_id for point in points if not point.success)
    status: TrimSolveStatus = "verified" if not failures else "failed"
    findings = (f"solve failed at: {', '.join(failures)}",) if failures else ()
    return VehicleTrimSolveReport(family_id, adapter, status, orchestration.claim_boundary, points, findings)
    ####


def solve_all_vehicle_trim_evidence() -> tuple[VehicleTrimSolveReport, ...]:
    """Solve the two conformance-pilot worklists."""

    return tuple(solve_vehicle_trim_evidence(family_id) for family_id in ("reference_f16_s119", "reference_hl20_mod_k"))
    ####


__all__ = [
    "TrimSolvePoint",
    "VehicleTrimSolveReport",
    "solve_all_vehicle_trim_evidence",
    "solve_vehicle_trim_evidence",
]
####
