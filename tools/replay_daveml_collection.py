"""Replay the declared runtime artifact inside one Taoryx DAVE-ML collection."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.trajectory import replay_reference_collection


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("collection", type=Path, help="verified .txcollection archive")
    parser.add_argument("--json", dest="json_path", type=Path, help="write the report to this path")
    arguments = parser.parse_args()
    report = replay_reference_collection(arguments.collection).to_dict()
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if arguments.json_path is not None:
        arguments.json_path.parent.mkdir(parents=True, exist_ok=True)
        arguments.json_path.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
