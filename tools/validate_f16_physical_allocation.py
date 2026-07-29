"""Generate local F-16 source-effectiveness and bounded-allocation evidence."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from taoryx.control_allocation import EffectorLimits
from taoryx.trajectory import F16ReferencePhysicalPlant, load_f16_reference_plant
from taoryx.trim import TrimResult, TrimSpec

ROOT = Path(__file__).resolve().parents[1]


def _trim(evidence: dict[str, Any]) -> TrimResult:
    """Build the declared operating-point trim contract."""

    state = {name: float(value) for name, value in evidence["trim_state"].items()}
    controls = {name: float(value) for name, value in evidence["trim_controls"].items()}
    spec = TrimSpec(
        state_names=tuple(state),
        control_names=tuple(controls),
        residual_names=tuple(state),
        state_initial=state,
        control_initial=controls,
        operating_point={"trim_pitch_rad": float(evidence["metadata"]["trim_pitch_rad"])},
    )
    return TrimResult(spec, state, controls, {name: 0.0 for name in state}, 0.0, True, 1, "source trim", 0, 0.0)
    ####


def _limits(payload: dict[str, Any]) -> dict[str, EffectorLimits]:
    """Resolve the explicit development actuator overlay."""

    limits: dict[str, EffectorLimits] = {}
    names = {
        "elevator": "elevator_deg",
        "aileron": "aileron_deg",
        "rudder": "rudder_deg",
        "throttle": "throttle_fraction",
    }
    for source_name, values in payload["limits"].items():
        name = names[source_name]
        limits[name] = EffectorLimits(
            name,
            float(values["lower"]),
            float(values["upper"]),
            str(values["unit"]),
            float(values["rate_limit_per_s"]),
            float(values["time_constant_s"]),
        )
    return limits
    ####


def main() -> int:
    """Run feasible and deliberately infeasible local allocation witnesses."""

    linearization = json.loads(
        (ROOT / "verification/f16_runtime_linearization_evidence.json").read_text(encoding="utf-8")
    )
    actuator_profile = yaml.safe_load(
        (ROOT / "families/reference_f16_s119/actuators/reference-first-order-v1.yaml").read_text(encoding="utf-8")
    )
    trim = _trim(linearization)
    source = load_f16_reference_plant(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml",
    )
    adapter = F16ReferencePhysicalPlant(
        source,
        trim,
        float(linearization["metadata"]["trim_pitch_rad"]),
        0.0,
        _limits(actuator_profile),
    )
    state = dict(trim.state)
    effectors = dict(trim.controls)
    effectiveness = adapter.effectiveness(state, effectors)
    feasible = adapter.allocate(state, effectiveness.reference_wrench, effectors, 0.01)
    infeasible_request = dict(effectiveness.reference_wrench)
    infeasible_request["total_force_x_n"] += 1.0e7
    infeasible_request["total_moment_y_nm"] += 1.0e8
    infeasible = adapter.allocate(state, infeasible_request, effectors, 0.01)
    report = {
        "schema_version": "taoryx.f16-physical-allocation-evidence/v1",
        "status": "development_screen_passed",
        "family_id": "reference_f16_s119",
        "plant_id": "reference-f16-s119-source-runtime-plant",
        "actuator_profile_id": actuator_profile["profile_id"],
        "wrench_names": list(effectiveness.wrench_names),
        "effector_names": list(effectiveness.effector_names),
        "effectiveness": {
            "matrix": [list(row) for row in effectiveness.matrix],
            "rank": int(np.linalg.matrix_rank(effectiveness.array)),
            "source": effectiveness.source,
            "reference_wrench": dict(effectiveness.reference_wrench),
            "reference_effectors": dict(effectiveness.reference_effectors),
        },
        "trim_witness": {
            "status": feasible.allocation.status,
            "controlled_residual_norm": feasible.achieved_controlled_residual_norm,
            "commands": dict(feasible.actuator.commanded_positions),
            "actual_effectors": dict(feasible.actuator.actual_positions),
        },
        "infeasible_witness": {
            "status": infeasible.allocation.status,
            "requested_wrench": infeasible_request,
            "achieved_wrench": dict(infeasible.achieved_wrench),
            "residual_wrench": dict(infeasible.achieved_residual_wrench),
            "saturated_channels": sorted(
                set(infeasible.allocation.position_saturated)
                | set(infeasible.allocation.rate_limited)
                | set(infeasible.actuator.position_saturated)
                | set(infeasible.actuator.rate_limited)
            ),
        },
        "claim_boundary": (
            "source-derived local load-effectiveness and bounded allocation screen; "
            "not source-validated actuator dynamics, closed-loop maneuver qualification, "
            "or operational F-16 fidelity"
        ),
    }
    output = ROOT / "verification/f16_physical_allocation_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if feasible.allocation.status == "feasible" and infeasible.allocation.status != "feasible" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
