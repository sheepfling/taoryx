"""Command-line entrypoint for the opt-in TAOS v23 corpus."""

from __future__ import annotations

import argparse
import json

from tests.e2e.support.loader import load_manifest
from tests.e2e.support.runtime import run_case, taos_executable
from tests.e2e.support.static_validation import validate_case


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate or run the TAOS v23 end-to-end corpus.")
    parser.add_argument("--case", action="append", default=[])
    parser.add_argument("--runtime", action="store_true")
    parser.add_argument("--json", action="store_true", dest="as_json")
    arguments = parser.parse_args()
    selected = [item for item in load_manifest() if not arguments.case or item.id in arguments.case]
    rows: list[dict[str, object]] = []
    for item in selected:
        static = validate_case(item)
        row: dict[str, object] = {
            "id": item.id,
            "kind": item.kind,
            "static_errors": static.error_codes,
            "static_warnings": static.warning_codes,
        }
        if arguments.runtime and item.kind == "positive" and taos_executable() is not None:
            result = run_case(item)
            row["returncode"] = result.returncode
            row["workdir"] = str(result.workdir)
        rows.append(row)
    if arguments.as_json:
        print(json.dumps(rows, indent=2))
    else:
        for row in rows:
            print(f"{row['id']}: errors={row['static_errors']} warnings={row['static_warnings']}")
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
####
