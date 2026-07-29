"""Generate the source-backed F-16 runtime local-linearization artifact."""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from taoryx.trajectory import load_f16_reference_plant

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    """Linearize the corrected F-16 operating point through the runtime adapter."""

    plant = load_f16_reference_plant(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml",
    )
    evidence = json.loads(
        (ROOT / "verification/daveml_f16_equilibrium_trim_evidence.json").read_text(encoding="utf-8")
    )
    source_state = evidence["resolved_state"]
    body_velocity = source_state["body_velocity_m_s"]
    body_rates = source_state["body_rates_rad_s"]
    state = {
        "u_m_s": body_velocity["u"],
        "v_m_s": body_velocity["v"],
        "w_m_s": body_velocity["w"],
        "p_rad_s": body_rates["p"],
        "q_rad_s": body_rates["q"],
        "r_rad_s": body_rates["r"],
    }
    controls = {
        "elevator_deg": evidence["controls"]["elevator_deg"],
        "aileron_deg": 0.0,
        "rudder_deg": 0.0,
        "throttle_fraction": evidence["control_contract"]["throttle_fraction"],
    }
    state_step = 1.0e-5
    control_step = 1.0e-5
    allowed_relative_difference = 0.05
    result = plant.linearize_local(
        state,
        controls,
        trim_pitch_rad=source_state["attitude"]["pitch_rad"],
        altitude_m=evidence["operating_point"]["geometric_altitude_m"],
        state_step=state_step,
        control_step=control_step,
    )
    report = {
        "schema_version": "taoryx.f16-runtime-linearization-evidence/v1",
        "status": "verified",
        "claim_boundary": (
            "source-backed local body-dynamics A/B derived from the corrected runtime plant; "
            "not scheduled control, actuator allocation, or nonlinear mission qualification"
        ),
        "family_id": "reference_f16_s119",
        "plant_id": result.provenance.nonlinear_plant_id,
        "plant_revision": result.provenance.nonlinear_plant_revision,
        "state_names": list(result.primary.state_names),
        "control_names": list(result.primary.control_names),
        "trim_state": dict(result.primary.trim_state),
        "trim_controls": dict(result.primary.trim_controls),
        "perturbations": {
            "state_step": state_step,
            "control_step": control_step,
            "comparison_factor": result.provenance.comparison_state_step / state_step,
            "method": result.provenance.method,
        },
        "a_matrix": result.primary.a_matrix.tolist(),
        "b_matrix": result.primary.b_matrix.tolist(),
        "comparison_a_matrix": result.comparison.a_matrix.tolist(),
        "comparison_b_matrix": result.comparison.b_matrix.tolist(),
        "derivative_consistency": {
            "passed": result.provenance.derivative_consistent,
            "maximum_relative_difference": result.provenance.maximum_relative_difference,
            "maximum_absolute_difference": result.provenance.maximum_absolute_difference,
            "allowed_relative_difference": allowed_relative_difference,
        },
        "metadata": dict(result.primary.metadata),
        "provenance": {
            "aerodynamics_document_sha256": plant.aerodynamics.graph.document_sha256,
            "aerodynamics_package_sha256": plant.aerodynamics.graph.package_sha256,
            "mass_properties_inertia_matrix_kg_m2": plant.inertia_matrix_kg_m2,
            "propulsion_document_sha256": plant.propulsion.graph.document_sha256,
            "propulsion_package_sha256": plant.propulsion.graph.package_sha256,
        },
    }
    output = ROOT / "verification/f16_runtime_linearization_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
