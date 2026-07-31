#!/usr/bin/env python3
"""Write auditable Alpha 3 3DOF and pseudo-6DOF X-15 evidence.

The output can earn a narrowly scoped ``nominal_case_pass`` for the staged
event/energy/impact witness. It is not a qualification shortcut for the
controlled X-15 approach and handoff still required by the plan.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from taoryx.reachability_envelope import LaunchCommand, ReachabilityFidelity, simulate_rocket_glide
from taoryx.x15_reachability import build_x15_fidelity_evidence, x15_surrogate_vehicle

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/x15_fidelity_ladder"


def write_x15_fidelity_ladder(output: Path, *, step_size_s: float, horizon_s: float) -> dict[str, object]:
    """Run both reduced tiers and write their self-contained evidence files."""

    output.mkdir(parents=True, exist_ok=True)
    vehicle = x15_surrogate_vehicle()
    command = LaunchCommand(0.0, math.radians(45.0))
    records: list[dict[str, object]] = []
    for fidelity in (ReachabilityFidelity.POINT_MASS_3DOF, ReachabilityFidelity.PSEUDO_6DOF):
        trajectory = simulate_rocket_glide(
            vehicle,
            command,
            fidelity=fidelity,
            step_size_s=step_size_s,
            horizon_s=horizon_s,
            spawn_children=True,
        )
        artifact = build_x15_fidelity_evidence(
            trajectory,
            fidelity=fidelity,
            command=command,
            step_size_s=step_size_s,
            horizon_s=horizon_s,
        )
        if artifact["evaluation"]["mission_pass"] is True and artifact["mission"]["independent_truth_evaluation"] is True:  # type: ignore[index]
            artifact["status"] = "nominal_case_pass"
            artifact["nominal_case_scope"] = "x15.staged_event_energy_corridor_impact_v1"
            artifact["physical_promotion_boundary"] = (
                "This promotion covers only the independently evaluated staged event chain, unpowered glide, "
                "high-energy corridor, and impact witness. It does not claim a controlled terminal handoff, "
                "native X-15 batch-provider equivalence, or physical stabilator/rudder/throttle/RCS allocation."
            )
        path = output / f"{fidelity.value}_evidence.json"
        path.write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        try:
            artifact_reference = str(path.relative_to(ROOT))
        except ValueError:
            artifact_reference = path.name
        records.append(
            {
                "fidelity": fidelity.value,
                "artifact": artifact_reference,
                "status": artifact["status"],
                "mission_pass": artifact["evaluation"]["mission_pass"],  # type: ignore[index]
                "claim_boundary": artifact["claim"]["claim_boundary"],  # type: ignore[index]
            }
        )
    promoted = all(record["status"] == "nominal_case_pass" for record in records)
    manifest: dict[str, object] = {
        "schema": "taoryx.x15-fidelity-ladder/v1alpha1",
        "vehicle_id": "x15",
        "status": "nominal_case_pass" if promoted else "development",
        "records": records,
        "reproduction": "PYTHONPATH=src python3 tools/validate_x15_fidelity_ladder.py",
        "nominal_case_scope": "x15.staged_event_energy_corridor_impact_v1" if promoted else None,
        "promotion_blocker": "controlled terminal handoff and source-backed physical-effector validation remain pending",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def main() -> int:
    """Parse the deterministic witness settings and write the packet."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--step-size-s", type=float, default=0.5)
    parser.add_argument("--horizon-s", type=float, default=600.0)
    arguments = parser.parse_args()
    manifest = write_x15_fidelity_ladder(
        arguments.output,
        step_size_s=arguments.step_size_s,
        horizon_s=arguments.horizon_s,
    )
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
