"""Build or verify the Mission Composition completion matrix artifacts."""

from __future__ import annotations

import argparse
from pathlib import Path

from taoryx.mission_composition_completion import (
    build_mission_composition_completion_report,
    render_mission_composition_completion_markdown,
)
from taoryx.vehicle_registry import ROOT

DEFAULT_JSON = ROOT / "verification/mission_composition_completion.json"
DEFAULT_MARKDOWN = ROOT / "docs/architecture/mission-composition-coverage-matrix.md"


def _write_or_check(path: Path, contents: str, *, check: bool) -> None:
    if check:
        if not path.is_file():
            raise SystemExit(f"generated Mission Composition artifact is missing: {path.relative_to(ROOT)}")
        if path.read_text(encoding="utf-8") != contents:
            raise SystemExit(f"generated Mission Composition artifact is stale: {path.relative_to(ROOT)}")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(contents, encoding="utf-8")
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Fail if checked-in artifacts differ from current contracts.")
    parser.add_argument("--json", type=Path, default=DEFAULT_JSON)
    parser.add_argument("--markdown", type=Path, default=DEFAULT_MARKDOWN)
    args = parser.parse_args()

    report = build_mission_composition_completion_report()
    if report.status != "pass":
        raise SystemExit("Mission Composition completion audit failed: " + "; ".join(report.diagnostics))
    json_contents = report.model_dump_json(indent=2, by_alias=True) + "\n"
    markdown_contents = render_mission_composition_completion_markdown(report)
    _write_or_check(args.json, json_contents, check=args.check)
    _write_or_check(args.markdown, markdown_contents, check=args.check)
    print(
        f"Mission Composition completion: {report.status}; families={report.family_count}; "
        f"realizations={report.realization_count}; batch={report.registered_batch_tuple_count}; "
        f"interactive={report.registered_interactive_tuple_count}"
    )
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
