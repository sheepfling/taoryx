"""Validate one language-family example corpus and write a JSON report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.ingest import ingest_file


ROOT = Path(__file__).resolve().parent.parent
EXAMPLES = ROOT / "examples"


def _paths(family: str) -> tuple[Path, ...]:
    if family == "taos96":
        roots = (EXAMPLES / "chapter03", EXAMPLES / "chapter04")
    else:
        roots = (EXAMPLES / "taoryx" / "syntax_fragments", EXAMPLES / "taoryx" / "full_examples")
    return tuple(sorted(path for root in roots for path in root.rglob("*") if path.suffix.casefold() in {".prb", ".tbl"}))


def validate(family: str, destination: Path) -> int:
    profile = GrammarProfile(family)
    reports: list[dict[str, object]] = []
    error_count = 0
    for path in _paths(family):
        ingested = ingest_file(path, profile=profile)
        diagnostics = [item.model_dump(mode="json") for item in ingested.diagnostics]
        error_count += sum(item["severity"] == "error" for item in diagnostics)
        reports.append(
            {
                "path": str(path.relative_to(ROOT)),
                "kind": ingested.kind.value,
                "valid": not any(item["severity"] == "error" for item in diagnostics),
                "diagnostics": diagnostics,
                "document": ingested.document.model_dump(mode="json"),
            }
        )
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "parser-report.json").write_text(json.dumps(reports, indent=2) + "\n", encoding="utf-8")
    print(f"validated {len(reports)} {family} source file(s); errors={error_count}")
    print(destination / "parser-report.json")
    return 1 if error_count else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--family", choices=("taos96", "taoryx"), required=True)
    parser.add_argument("--output", type=Path, default=None)
    arguments = parser.parse_args()
    destination = arguments.output or ROOT / "artifacts" / "examples" / arguments.family
    return validate(arguments.family, destination)


if __name__ == "__main__":
    raise SystemExit(main())
