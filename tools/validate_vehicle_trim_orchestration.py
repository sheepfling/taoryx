"""Validate declarative trim recipes and emit adapter-ready worklists."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.vehicle_trim_orchestration import (  # noqa: E402
    TrimOrchestrationReport,
    orchestrate_trim_recipe,
)

PILOT_FAMILIES = ("reference_f16_s119", "reference_hl20_mod_k")


def _render(report: TrimOrchestrationReport) -> str:
    lines = [
        f"{report.family_id}: {report.status} ({report.recipe_id or 'no recipe'})",
        f"  catalog: {report.catalog_path or 'unavailable'}",
        f"  adapter-ready work items: {len(report.work_items)}",
    ]
    for finding in report.findings:
        lines.append(f"  {finding.severity.upper()} {finding.code}: {finding.message}")
    return "\n".join(lines)
    ####


def main(argv: list[str] | None = None) -> int:
    """Validate pilot trim recipes without invoking a family plant solver."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--family", default="all", help="pilot family ID or 'all'")
    parser.add_argument("--json", type=Path, help="write a machine-readable report")
    parser.add_argument("--allow-blocked", action="store_true", help="report blockers but return success")
    args = parser.parse_args(argv)
    families = PILOT_FAMILIES if args.family == "all" else (args.family,)
    reports = tuple(orchestrate_trim_recipe(family) for family in families)
    for report in reports:
        print(_render(report))
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(
            json.dumps(
                {
                    "schema_version": "taoryx.vehicle-trim-orchestration/v1",
                    "families": [report.as_dict() for report in reports],
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
    return 0 if args.allow_blocked or all(report.status == "ready_for_adapter" for report in reports) else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
