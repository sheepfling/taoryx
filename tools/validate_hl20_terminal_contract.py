"""Run the explicit HL-20 California-to-Hawaii terminal contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.hl20_reachability import hl20_ca_hi_terminal_criteria, run_hl20_source_fidelity_ladder

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "artifacts/verification/hl20-ca-hi-terminal-contract.json"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--horizon-s", type=float, default=120.0)
    parser.add_argument("--step-size-s", type=float, default=0.5)
    args = parser.parse_args()
    if args.horizon_s <= 25.0 or args.step_size_s <= 0.0:
        parser.error("--horizon-s must exceed the 25 s release and --step-size-s must be positive")

    criteria = hl20_ca_hi_terminal_criteria()
    envelopes = run_hl20_source_fidelity_ladder(
        horizon_s=args.horizon_s,
        step_size_s=args.step_size_s,
        spawn_children=True,
        criteria=criteria,
    )
    tiers: list[dict[str, object]] = []
    for envelope in envelopes:
        tiers.append(
            {
                "fidelity": envelope.fidelity.value,
                "classification_counts": envelope.classification_counts,
                "timed_out_query_ids": list(envelope.timed_out_query_ids),
                "failure_reason_counts": envelope.failure_reason_counts,
                "candidate_margins": [
                    {
                        "query_id": sample.query_id,
                        "classification": sample.classification,
                        "limiting_factor": sample.limiting_factor,
                        "terminal_impact_radius_m": dict(sample.path_metrics).get("terminal_impact_radius_m"),
                        "terminal_impact_speed_m_s": dict(sample.path_metrics).get("terminal_impact_speed_m_s"),
                        "terminal_margins": dict(sample.terminal_margins),
                    }
                    for sample in envelope.samples
                ],
            }
        )
    report = {
        "schema": "taoryx.hl20-ca-hi-terminal-contract/v1alpha1",
        "status": "contract_executed_without_capability_claim",
        "claim_boundary": "explicit local-tangent target, impact radius, and impact-speed classification; not a source-exact route or mission-performance claim",
        "route": {
            "start": {"latitude_deg": 34.7, "longitude_deg": -120.6},
            "target": {"latitude_deg": 21.31, "longitude_deg": -157.86},
            "great_circle_distance_km": 3920.0,
        },
        "terminal_criteria": criteria.as_dict(),
        "execution": {"horizon_s": args.horizon_s, "step_size_s": args.step_size_s, "deterministic": True},
        "tiers": tiers,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
