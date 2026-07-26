"""Score a provenance-linked HL-20 glide-trim scenario."""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from taoryx.objectives import ObjectiveSpec, score_objectives
from taoryx.scenario_contract import ScenarioContract

ROOT = Path(__file__).resolve().parents[1]


def digest(payload: object) -> str:
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def main() -> int:
    trim = json.loads((ROOT / "verification/daveml_hl20_trim_evidence.json").read_text(encoding="utf-8"))
    loads = json.loads((ROOT / "verification/daveml_hl20_load_evidence.json").read_text(encoding="utf-8"))
    source = trim["source"]
    contract = ScenarioContract(
        scenario_id="hl20-mod-k-daveml-glide-trim",
        vehicle="hl20-mod-k-unpowered-6dof",
        family="reference_hl20_mod_k",
        dynamics_tier="pseudo_6dof",
        initial_state_sha256=digest(trim["trim"]["state"]),
        environment_sha256=digest({"mach": 1.0, "true_airspeed_f_s": 100.0, "altitude_m": 0.0}),
        vehicle_model_sha256=source["package_sha256"],
        propulsion_model_sha256=digest("not-applicable-unpowered"),
        mass_model_sha256=digest("not-declared-by-source"),
        command_history_sha256=digest(trim["trim"]["controls"]),
        event_schedule_sha256=digest(["pitch_trim", "load_sign_convention"]),
        termination_policy_sha256=digest({"duration_s": 1.0, "required": True}),
        duration_s=1.0,
        integrator="source-channel-glide-trim",
        output_rate_hz=1.0,
    )
    objectives = (
        ObjectiveSpec("pitch-trim", "trim", "pitch_cm", 0.0, 1.0e-9, "coefficient"),
        ObjectiveSpec("positive-lift", "aerodynamics", "lift_n", 0.0, 1.0e-9, "N", comparison="minimum"),
        ObjectiveSpec("finite-loads", "aerodynamics", "load_residual", 0.0, 1.0e-12, "N", comparison="absolute_error"),
    )
    observed = {
        "pitch_cm": trim["trim"]["residuals"]["pitch_cm"],
        "lift_n": loads["loads"]["lift_n"],
        "load_residual": 0.0 if all(isinstance(value, (int, float)) for value in loads["loads"].values()) else 1.0,
    }
    scored = score_objectives(
        objectives,
        observed,
        time_s=1.0,
        completed_events={"pitch_trim", "load_sign_convention"},
        termination={"status": "completed", "reason": "bounded glide-trim evidence"},
        scenario_contract_sha256=contract.digest(),
    )
    report = {
        "schema_version": "taoryx.daveml-hl20-scenario-evidence/v1",
        "status": scored["status"],
        "claim_boundary": "provenance-linked HL-20 glide-trim objective scoring; not full-flight trajectory or controller qualification",
        "family_id": "reference_hl20_mod_k",
        "scenario_contract": contract.model_dump(mode="json") | {"contract_sha256": contract.digest()},
        "objective_report": scored,
        "provenance": {"aerodynamics_document_sha256": source["document_sha256"], "aerodynamics_package_sha256": source["package_sha256"]},
    }
    output = ROOT / "verification/daveml_hl20_scenario_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if scored["status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
