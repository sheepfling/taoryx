"""Replay verified F-16 and HL-20 DAVE-ML packages through Taoryx."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.trajectory import replay_reference_package


def main() -> int:
    """Run package replay and emit one JSON report."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package", nargs="+", type=Path, help="verified .txair package path")
    parser.add_argument("--json", dest="json_path", type=Path, help="write the report to this path")
    arguments = parser.parse_args()
    reports = [replay_reference_package(path).to_dict() for path in arguments.package]
    payload = {"status": "passed", "models": reports}
    rendered = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if arguments.json_path is not None:
        arguments.json_path.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
