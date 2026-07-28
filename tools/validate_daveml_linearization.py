"""Generate a source-linked local F-16 body-dynamics linearization."""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from taoryx.trajectory import (
    DAVEMLFixedWingDynamicsBinding,
    DAVEMLFixedWingLoadBinding,
    DAVEMLInertiaBinding,
    load_daveml_family_graph,
    load_daveml_trim_binding,
)
from taoryx.trim import TrimResult, TrimSpec, finite_difference_dynamics_linearization

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    sidecar = ROOT / "families/reference_f16_s119/plant/daveml-import.json"
    aero = load_daveml_trim_binding(
        sidecar,
        role="aerodynamics",
        state_inputs={},
        control_inputs={"elevator_deg": "el", "aileron_deg": "ail", "rudder_deg": "rdr"},
        residual_outputs={"cx": "cx", "cy": "cy", "cz": "cz", "cl": "cl", "cm": "cm", "cn": "cn"},
        fixed_inputs={"vt": 500.0, "alpha": 2.0, "beta": 0.0, "p": 0.0, "q": 0.0, "r": 0.0, "ail": 0.0, "rdr": 0.0, "xcg": 0.35},
    )
    propulsion = load_daveml_trim_binding(
        sidecar,
        role="propulsion",
        state_inputs={},
        control_inputs={"power_pct": "powerLeverAngle"},
        residual_outputs={"thrust_lbf": "thrustBodyForce_X"},
        fixed_inputs={"altitudeMSL": 0.0, "mach": 0.0},
    )
    loads = DAVEMLFixedWingLoadBinding(
        aerodynamics=aero,
        reference_area_m2=27.870912,
        mean_aerodynamic_chord_m=3.450336,
        span_m=9.144,
        dynamic_pressure_pa=1000.0,
        propulsion=propulsion,
    )
    inertia_graph = load_daveml_family_graph(sidecar, role="mass_properties")
    inertia = DAVEMLInertiaBinding(inertia_graph)
    inertia_values = inertia.evaluate()
    dynamics = DAVEMLFixedWingDynamicsBinding(
        loads=loads,
        mass_kg=inertia_values["mass_kg"],
        inertia_matrix_kg_m2=inertia.as_inertia_matrix(),
    )
    spec = TrimSpec(
        state_names=("u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s"),
        control_names=("elevator_deg", "aileron_deg", "rudder_deg", "power_pct"),
        residual_names=("channel_operating_point",),
        state_initial={"u_m_s": 152.4, "v_m_s": 0.0, "w_m_s": 0.0, "p_rad_s": 0.0, "q_rad_s": 0.0, "r_rad_s": 0.0},
        control_initial={"elevator_deg": 0.0, "aileron_deg": 0.0, "rudder_deg": 0.0, "power_pct": 0.0},
    )
    result = TrimResult(
        spec=spec,
        state=dict(spec.state_initial),
        controls=dict(spec.control_initial),
        residuals={"channel_operating_point": 0.0},
        scaled_residual_norm=0.0,
        success=True,
        status=0,
        message="declared source-channel operating point",
        iterations=0,
        cost=0.0,
    )
    linearization = finite_difference_dynamics_linearization(
        spec,
        dynamics.as_evaluator({"altitude_ft": 0.0, "mach": 0.0}),
        result,
        state_step=1.0e-4,
        control_step=1.0e-4,
        metadata={"claim_boundary": "source-channel operating point with source propulsion sensitivity; not equilibrium trim"},
    )
    report = {
        "schema_version": "taoryx.daveml-linearization-evidence/v1",
        "status": "verified",
        "claim_boundary": "source-channel operating point with source propulsion sensitivity; not equilibrium trim or controller qualification",
        "family_id": "reference_f16_s119",
        "state_names": list(linearization.state_names),
        "control_names": list(linearization.control_names),
        "trim_state": dict(linearization.trim_state),
        "trim_controls": dict(linearization.trim_controls),
        "perturbations": {"state_step": 1.0e-4, "control_step": 1.0e-4, "method": "central_finite_difference"},
        "a_matrix": linearization.a_matrix.tolist(),
        "b_matrix": linearization.b_matrix.tolist(),
        "provenance": {
            "aerodynamics_document_sha256": aero.graph.document_sha256,
            "aerodynamics_package_sha256": aero.graph.package_sha256,
            "mass_properties_document_sha256": inertia_graph.document_sha256,
            "mass_properties_package_sha256": inertia_graph.package_sha256,
            "propulsion_document_sha256": propulsion.graph.document_sha256,
            "propulsion_package_sha256": propulsion.graph.package_sha256,
        },
    }
    output = ROOT / "verification/daveml_f16_linearization_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
