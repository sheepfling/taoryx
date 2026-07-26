"""Generate a bounded source-backed F-16 force/moment equilibrium trim."""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from taoryx.trajectory import DAVEMLFixedWingLoadBinding, DAVEMLInertiaBinding, load_daveml_atmosphere, load_daveml_family_graph, load_daveml_trim_binding
from taoryx.trim import TrimSpec, solve_trim

ROOT = Path(__file__).resolve().parents[1]
SIDECAR = ROOT / "families/reference_f16_s119/plant/daveml-import.json"
ALTITUDE_M = 0.0
TRUE_AIRSPEED_M_S = 152.4
GRAVITY_M_S2 = 9.80665


def main() -> int:
    aero = load_daveml_trim_binding(
        SIDECAR,
        role="aerodynamics",
        state_inputs={"alpha_deg": "alpha"},
        control_inputs={"elevator_deg": "el"},
        residual_outputs={"cx": "cx", "cy": "cy", "cz": "cz", "cl": "cl", "cm": "cm", "cn": "cn"},
        fixed_inputs={"vt": 500.0, "beta": 0.0, "p": 0.0, "q": 0.0, "r": 0.0, "ail": 0.0, "rdr": 0.0, "xcg": 0.35},
    )
    propulsion = load_daveml_trim_binding(
        SIDECAR,
        role="propulsion",
        state_inputs={"altitude_ft": "altitudeMSL", "mach": "mach"},
        control_inputs={"power_pct": "powerLeverAngle"},
        residual_outputs={"thrust_lbf": "thrustBodyForce_X"},
    )
    atmosphere = load_daveml_atmosphere(ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml")
    inertia_graph = load_daveml_family_graph(SIDECAR, role="mass_properties")
    inertia = DAVEMLInertiaBinding(inertia_graph)
    mass = inertia.evaluate()["mass_kg"]
    loads = DAVEMLFixedWingLoadBinding(
        aerodynamics=aero,
        reference_area_m2=27.870912,
        mean_aerodynamic_chord_m=3.450336,
        span_m=9.144,
        dynamic_pressure_pa=1.0,
        propulsion=propulsion,
    )

    def evaluate(state: dict[str, float], controls: dict[str, float]) -> dict[str, float]:
        values = loads.evaluate_with_atmosphere(
            {"alpha_deg": state["alpha_deg"], "altitude_ft": 0.0, "mach": 0.0},
            controls,
            atmosphere,
            geometric_altitude_m=ALTITUDE_M,
            true_airspeed_m_s=TRUE_AIRSPEED_M_S,
        )
        return {
            "force_x_n": values["total_force_x_n"],
            "force_z_n": values["total_force_z_n"] + mass * GRAVITY_M_S2,
            "moment_y_nm": values["total_moment_y_nm"],
        }

    spec = TrimSpec(
        state_names=("alpha_deg",),
        control_names=("elevator_deg", "power_pct"),
        residual_names=("force_x_n", "force_z_n", "moment_y_nm"),
        state_initial={"alpha_deg": 6.0},
        control_initial={"elevator_deg": 0.0, "power_pct": 50.0},
        state_lower={"alpha_deg": 0.0},
        state_upper={"alpha_deg": 15.0},
        control_lower={"elevator_deg": -24.0, "power_pct": 0.0},
        control_upper={"elevator_deg": 24.0, "power_pct": 100.0},
        residual_scales={"force_x_n": 10000.0, "force_z_n": 10000.0, "moment_y_nm": 10000.0},
    )
    result = solve_trim(spec, evaluate, max_nfev=500, residual_tolerance=1.0e-10, acceptance_tolerance=1.0e-6)
    report = {
        "schema_version": "taoryx.daveml-equilibrium-trim/v1",
        "status": "verified" if result.success else "failed",
        "claim_boundary": "source-backed fixed-altitude/airspeed force-moment equilibrium; not flight qualification",
        "family_id": "reference_f16_s119",
        "operating_point": {"geometric_altitude_m": ALTITUDE_M, "true_airspeed_m_s": TRUE_AIRSPEED_M_S, "gravity_m_s2": GRAVITY_M_S2, "mass_kg": mass},
        "bounds": {"alpha_deg": [0.0, 15.0], "elevator_deg": [-24.0, 24.0], "power_pct": [0.0, 100.0]},
        "state": dict(result.state),
        "controls": dict(result.controls),
        "residuals": dict(result.residuals),
        "max_residual": result.max_residual,
        "scaled_residual_norm": result.scaled_residual_norm,
        "solver": {"success": result.success, "status": result.status, "iterations": result.iterations, "message": result.message},
        "provenance": {
            "aerodynamics_document_sha256": aero.graph.document_sha256,
            "aerodynamics_package_sha256": aero.graph.package_sha256,
            "propulsion_document_sha256": propulsion.graph.document_sha256,
            "propulsion_package_sha256": propulsion.graph.package_sha256,
            "mass_properties_document_sha256": inertia_graph.document_sha256,
            "mass_properties_package_sha256": inertia_graph.package_sha256,
            "atmosphere_sha256": atmosphere.source_sha256,
        },
    }
    output = ROOT / "verification/daveml_f16_equilibrium_trim_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if result.success else 1


if __name__ == "__main__":
    raise SystemExit(main())

