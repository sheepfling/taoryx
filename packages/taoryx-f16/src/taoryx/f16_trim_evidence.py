"""F-16-owned source binding for declarative trim-worklist evidence.

The shared trim orchestration produces a declarative worklist.  This module
owns only the F-16 DAVE-ML source bindings needed to solve that worklist; the
caller owns report aggregation and presentation.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

from taoryx_f16.resources import model_resource_root

from .trajectory import (
    DAVEMLFixedWingLoadBinding,
    DAVEMLInertiaBinding,
    load_daveml_atmosphere,
    load_daveml_family_graph,
    load_daveml_trim_binding,
)
from .trim import TrimResult, solve_trim
from .vehicle_trim_adapters import VehicleTrimEvidenceBinding
from .vehicle_trim_orchestration import TrimWorkItem

_ROOT = model_resource_root()


def trim_evidence_binding() -> VehicleTrimEvidenceBinding:
    """Publish the F-16-owned source trim binding to the generic host."""

    return VehicleTrimEvidenceBinding(
        family_id="reference_f16_s119",
        adapter="taoryx.adapters.reference_f16_s119.daveml_equilibrium",
        resource_root=_ROOT,
        trim_recipe_path=_ROOT / "families/reference_f16_s119/qualification/trim-recipe.yaml",
        solve_work_items=solve_f16_trim_work_items,
    )
    ####


def solve_f16_trim_work_items(items: Sequence[TrimWorkItem]) -> tuple[tuple[str, TrimResult], ...]:
    """Solve source-grounded F-16 trim work items without a reference-package dependency."""

    loads, atmosphere, mass = _bindings()
    return tuple((item.point_id, _solve_work_item(item, loads, atmosphere, mass)) for item in items)
    ####


def _bindings() -> tuple[DAVEMLFixedWingLoadBinding, Any, float]:
    """Build the exact packaged F-16 aerodynamic, propulsion, and inertia bindings."""

    sidecar = _ROOT / "families/reference_f16_s119/plant/daveml-import.json"
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
    atmosphere = load_daveml_atmosphere(_ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml")
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
    return loads, atmosphere, mass
    ####


def _solve_work_item(
    item: TrimWorkItem,
    loads: DAVEMLFixedWingLoadBinding,
    atmosphere: Any,
    mass: float,
) -> TrimResult:
    """Solve one F-16 force/moment equilibrium item against its source bindings."""

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

    return solve_trim(item.trim_spec, evaluate, max_nfev=500, residual_tolerance=1.0e-10, acceptance_tolerance=1.0e-6)
    ####


__all__ = ["solve_f16_trim_work_items", "trim_evidence_binding"]
