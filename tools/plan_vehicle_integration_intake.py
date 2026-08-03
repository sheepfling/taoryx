#!/usr/bin/env python3
"""Print or write an explicit existing-family or new-topology intake plan."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.vehicle_integration_intake import (
    ExistingFamilyIntakeRequest,
    NewTopologyIntakeRequest,
    StrategySelectionError,
    build_existing_family_intake_blueprint,
    build_new_topology_intake_scaffold,
)


def _parser() -> argparse.ArgumentParser:
    """Return the intake-planning CLI parser."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("existing-family", "new-topology"), required=True)
    parser.add_argument("--family-id", required=True)
    parser.add_argument("--physical-family", required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--mission-overlay")
    parser.add_argument("--adapter-id")
    parser.add_argument("--strategy-id")
    parser.add_argument("--topology-summary")
    return parser
    ####


def main(argv: list[str] | None = None) -> int:
    """Build a non-promotable intake plan and render it as stable JSON."""

    args = _parser().parse_args(argv)
    try:
        if args.mode == "existing-family":
            if not args.mission_overlay or not args.adapter_id:
                raise ValueError("existing-family mode requires --mission-overlay and --adapter-id")
            payload = build_existing_family_intake_blueprint(
                ExistingFamilyIntakeRequest(
                    family_id=args.family_id,
                    physical_family=args.physical_family,
                    mission_overlay=args.mission_overlay,
                    adapter_id=args.adapter_id,
                    strategy_id=args.strategy_id,
                )
            ).as_dict()
        else:
            if not args.strategy_id or not args.topology_summary:
                raise ValueError("new-topology mode requires --strategy-id and --topology-summary")
            payload = build_new_topology_intake_scaffold(
                NewTopologyIntakeRequest(
                    family_id=args.family_id,
                    physical_family=args.physical_family,
                    proposed_strategy_id=args.strategy_id,
                    topology_summary=args.topology_summary,
                )
            ).as_dict()
    except (StrategySelectionError, ValueError) as error:
        _parser().error(str(error))
    rendered = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
