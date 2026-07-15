from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from taoryx.language.diagnostics import Diagnostic, Severity, SourceLocation
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.ingest import ingest_file


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
    parser.add_argument("--report", type=Path, help="Write the complete machine-readable report to this JSON file.")
    parser.add_argument("--profile", choices=tuple(profile.value for profile in GrammarProfile), default=GrammarProfile.TAOS96.value)
    arguments = parser.parse_args()
    reports: list[dict[str, Any]] = []
    error_count = 0
    for path in arguments.paths:
        try:
            ingested = ingest_file(path, profile=arguments.profile)
            document = ingested.document
            diagnostics = list(ingested.diagnostics)
        except (OSError, UnicodeError, ValueError) as exc:
            document = None
            diagnostics = [
                Diagnostic(
                    severity=Severity.ERROR,
                    code="file-ingest-failed",
                    message=str(exc),
                    location=SourceLocation(path=str(path), line=1),
                )
            ]
        ####
        error_count += sum(item.severity == "error" for item in diagnostics)
        reports.append({"path": str(path), "document": _serialize(document), "diagnostics": [_serialize(item) for item in diagnostics]})
    ####
    if arguments.report:
        arguments.report.write_text(json.dumps(reports, indent=2) + "\n", encoding="utf-8")
    ####
    if arguments.as_json:
        print(json.dumps(reports, indent=2))
    else:
        for report in reports:
            print(report["path"])
            for diagnostic in report["diagnostics"]:
                location = diagnostic.get("location")
                prefix = f"{location['line']}:{location.get('column', 1)}: " if location else ""
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
