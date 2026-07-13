from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from taoryx.language.problem_parser import parse_problem_file
from taoryx.language.semantic_validation import validate_problem, validate_table_file
from taoryx.language.table_parser import parse_table_file


def _serialize(value: Any) -> Any:
    if hasattr(value, "model_dump"):
        return value.model_dump(mode="json")
    ####
    return value
####


def main() -> int:
    parser = argparse.ArgumentParser(description="Parse and validate TAOS .prb and .tbl files.")
    parser.add_argument("paths", nargs="+", type=Path)
    parser.add_argument("--json", action="store_true", dest="as_json")
    arguments = parser.parse_args()
    reports: list[dict[str, Any]] = []
    error_count = 0
    for path in arguments.paths:
        if path.suffix.lower() == ".prb":
            document = parse_problem_file(path)
            diagnostics = validate_problem(document)
        elif path.suffix.lower() == ".tbl":
            document = parse_table_file(path)
            diagnostics = validate_table_file(document)
        else:
            parser.error(f"Unsupported file extension for {path}.")
        ####
        error_count += sum(item.severity == "error" for item in diagnostics)
        reports.append({"path": str(path), "document": _serialize(document), "diagnostics": [_serialize(item) for item in diagnostics]})
    ####
    if arguments.as_json:
        print(json.dumps(reports, indent=2))
    else:
        for report in reports:
            print(report["path"])
            for diagnostic in report["diagnostics"]:
                location = diagnostic.get("location")
                prefix = f"{location['line']}: " if location else ""
                print(f"  {diagnostic['severity']}: {prefix}{diagnostic['code']}: {diagnostic['message']}")
            ####
            if not report["diagnostics"]:
                print("  ok")
            ####
        ####
    ####
    return 1 if error_count else 0
####


if __name__ == "__main__":
    raise SystemExit(main())
####
