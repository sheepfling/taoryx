"""Score a provenance-linked DAVE-ML controller smoke scenario."""

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
    tuning_path = ROOT / "verification/daveml_f16_tuning_evidence.json"
    tuning = json.loads(tuning_path.read_text(encoding="utf-8"))
    contract = ScenarioContract(
        scenario_id="f16-s119-daveml-controller-smoke",
        vehicle="f16-s119-reference",
        family="reference_f16_s119",
        dynamics_tier="6dof",
        initial_state_sha256=digest(tuning["state_names"]),
        environment_sha256=digest({"atmosphere": "atmos_76", "altitude_m": 0.0}),
        vehicle_model_sha256=tuning["provenance"]["aerodynamics_package_sha256"],
        propulsion_model_sha256=tuning["provenance"]["aerodynamics_package_sha256"],
        mass_model_sha256=tuning["provenance"]["mass_properties_package_sha256"],
        command_history_sha256=digest(tuning["bounded_probe"]),
        event_schedule_sha256=digest(["controller_hurwitz", "bounded_surface_command"]),
        termination_policy_sha256=digest({"duration_s": 1.0, "required": True}),
        duration_s=1.0,
        integrator="source-channel-smoke",
        output_rate_hz=10.0,
    )
    objectives = (
        ObjectiveSpec("closed-loop-hurwitz", "stability", "maximum_real_pole", -0.01, 0.001, "1/s", comparison="maximum"),
        ObjectiveSpec("bounded-surface-command", "actuator", "control_saturation_fraction", 0.0, 0.001, "fraction"),
        ObjectiveSpec("controller-event", "event", "controller_hurwitz", True, None, "event", comparison="event"),
    )
    observed = {
        "maximum_real_pole": tuning["maximum_real_pole"],
        "control_saturation_fraction": 0.0 if not tuning["bounded_probe"]["saturated"] else 1.0,
    }
    scored = score_objectives(
        objectives,
        observed,
        time_s=1.0,
        completed_events={"controller_hurwitz", "bounded-surface-command"},
        termination={"status": "completed", "reason": "bounded smoke probe"},
        scenario_contract_sha256=contract.digest(),
    )
    report = {
        "schema_version": "taoryx.daveml-scenario-evidence/v1",
        "status": scored["status"],
        "claim_boundary": "provenance-linked controller smoke scoring; not flight scenario qualification",
        "family_id": "reference_f16_s119",
        "scenario_contract": contract.model_dump(mode="json") | {"contract_sha256": contract.digest()},
        "objective_report": scored,
        "tuning_provenance": tuning["provenance"],
    }
    output = ROOT / "verification/daveml_f16_scenario_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if scored["status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())

