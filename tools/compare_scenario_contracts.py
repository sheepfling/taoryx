"""Compare two scenario contracts for fidelity-parity eligibility."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.scenario_contract import ScenarioContract, compare_contracts


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("left", type=Path)
    parser.add_argument("right", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = compare_contracts(ScenarioContract.read_json(args.left), ScenarioContract.read_json(args.right))
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output is None:
        print(rendered, end="")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    return 0 if report["comparable"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
