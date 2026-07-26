"""Generate a bounded LQR tuning artifact from DAVE-ML linearization evidence."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from taoryx.controller_design import ControllerDesignSpec, build_lqr_controller
from taoryx.trim import TrimResult, TrimSpec

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    source = json.loads((ROOT / "verification/daveml_f16_linearization_evidence.json").read_text(encoding="utf-8"))
    full_states = tuple(source["state_names"])
    controls = tuple(source["control_names"])
    reduced_indices = (1, 2, 3, 4, 5)
    states = tuple(full_states[index] for index in reduced_indices)
    a_matrix = np.asarray(source["a_matrix"], dtype=float)[np.ix_(reduced_indices, reduced_indices)]
    b_matrix = np.asarray(source["b_matrix"], dtype=float)[list(reduced_indices), :]
    trim_spec = TrimSpec(
        state_names=states,
        control_names=controls,
        residual_names=("channel_operating_point",),
        state_initial={name: float(source["trim_state"][name]) for name in states},
        control_initial={name: float(source["trim_controls"][name]) for name in controls},
    )
    trim = TrimResult(
        spec=trim_spec,
        state=dict(trim_spec.state_initial),
        controls=dict(trim_spec.control_initial),
        residuals={"channel_operating_point": 0.0},
        scaled_residual_norm=0.0,
        success=True,
        status=0,
        message="declared source-channel operating point",
        iterations=0,
        cost=0.0,
    )
    design = ControllerDesignSpec(
        id="f16-s119-source-channel-lqr",
        method="lqr",
        trim="f16-s119-source-channel-operating-point",
        allocator="f16-s119-source-channel-surface-allocator",
        states=states,
        controls=controls,
        notes="Longitudinal speed is excluded because the source-channel A/B pair is rank 5.",
    )
    q = tuple(tuple(2.0 if row == column else 0.0 for column in range(len(states))) for row in range(len(states)))
    r = tuple(tuple(1.0 if row == column else 0.0 for column in range(len(controls))) for row in range(len(controls)))
    controller = build_lqr_controller(
        design,
        trim,
        a_matrix.tolist(),
        b_matrix.tolist(),
        q,
        r,
        lower={name: -24.0 for name in controls},
        upper={name: 24.0 for name in controls},
    )
    command = controller.command({name: float(trim.state[name]) + (0.1 if name == states[0] else 0.0) for name in states})
    result = controller.result
    report = {
        "schema_version": "taoryx.daveml-tuning-evidence/v1",
        "status": "verified",
        "claim_boundary": "source-channel reduced-order LQR screen; not flight qualification",
        "family_id": "reference_f16_s119",
        "design_id": design.id,
        "excluded_states": {"u_m_s": "source-channel A/B controllability rank is 5 of 6"},
        "state_names": list(states),
        "control_names": list(controls),
        "q_matrix": [list(row) for row in q],
        "r_matrix": [list(row) for row in r],
        "gain": result.gain.tolist(),
        "closed_loop_eigenvalues": [[float(value.real), float(value.imag)] for value in result.closed_loop_eigenvalues],
        "controllable": result.controllable,
        "hurwitz": result.hurwitz,
        "maximum_real_pole": result.maximum_real_pole,
        "condition_number": result.condition_number,
        "bounded_probe": {
            "controls": dict(command.controls),
            "unsaturated": dict(command.unsaturated),
            "saturated": list(command.saturated),
        },
        "provenance": source["provenance"],
    }
    output = ROOT / "verification/daveml_f16_tuning_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if result.hurwitz and result.controllable else 1


if __name__ == "__main__":
    raise SystemExit(main())

