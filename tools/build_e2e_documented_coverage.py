from __future__ import annotations

import csv
import json
import re
from collections import defaultdict
from pathlib import Path
from typing import Any

from taoryx.language.problem_parser import parse_problem_file
from taoryx.language.table_parser import parse_table_file
from tests.e2e.support.loader import case_directory, load_manifest, repository_root

DOCUMENTED_BLOCKS: list[tuple[str, str]] = [
    ("segment", "aero"), ("segment", "constants"), ("segment", "cg"),
    ("segment", "fly"), ("segment", "increment"), ("segment", "inertial"),
    ("segment", "integ"), ("segment", "limits"), ("segment", "prop"),
    ("segment", "rail"), ("segment", "reset"), ("segment", "when"),
    ("trajectory", "define"), ("trajectory", "dwn/crs"), ("trajectory", "file"),
    ("trajectory", "iip"), ("trajectory", "initial"), ("trajectory", "print"),
    ("trajectory", "tangent"),
    ("problem", "atmos"), ("problem", "define"), ("problem", "earth"),
    ("problem", "egs"), ("problem", "file"), ("problem", "optimize"),
    ("problem", "print"), ("problem", "radar"), ("problem", "search"),
    ("problem", "summarize"), ("problem", "survey"), ("problem", "title"),
    ("problem", "units/fmt"), ("problem", "wind"),
]

DOCUMENTED_TABLE_TYPES: list[str] = [
    "ca", "cn", "cl", "cd", "cs", "cx", "cy", "cz", "thrust", "tvec1",
    "tvec2", "mdot", "cg", "windv", "windh", "winde", "windn", "windd", "output",
]

DOCUMENTED_TABLE_OPERATIONS: list[str] = [
    "add", "sub", "mult", "div", "idiv", "exp", "iexp", "max", "min", "set",
    "abs", "neg", "sqr", "sqrt", "ln", "log", "e", "sin", "cos", "tan", "asin",
    "acos", "atan", "zero", "csto", "if", "goto", "end",
]

GRAMMAR_SCOPE_CONTRACTS: dict[str, tuple[str, str]] = {
    "problem_define_block": ("problem", "define"),
    "problem_file_block": ("problem", "file"),
    "problem_print_block": ("problem", "print"),
}


def _record(mapping: dict[str, set[str]], scope: str, name: str, case_id: str) -> None:
    mapping[f"{scope}:{name.lower()}"].add(case_id)
####


def collect(root: Path | None = None) -> dict[str, Any]:
    base = root or repository_root()
    block_cases: dict[str, set[str]] = defaultdict(set)
    table_type_cases: dict[str, set[str]] = defaultdict(set)
    operation_cases: dict[str, set[str]] = defaultdict(set)
    operation_pattern = re.compile(
        r"(?mi)^\s*(?:[A-Za-z0-9_.-]+\s*:\s*)?"
        r"(add|sub|mult|div|idiv|exp|iexp|max|min|set|abs|neg|sqr|sqrt|ln|log|e|sin|cos|tan|asin|acos|atan|zero|csto|if|goto|end)\b"
    )

    for case in load_manifest(base):
        if case.kind != "positive":
            continue
        ####
        directory = case_directory(case, base)
        document = parse_problem_file(directory / case.problem_file)
        for problem in document.problems:
            for block in problem.blocks:
                _record(block_cases, "problem", block.keyword, case.id)
            ####
            for trajectory in problem.trajectories:
                for block in trajectory.blocks:
                    _record(block_cases, "trajectory", block.keyword, case.id)
                ####
                for segment in trajectory.segments:
                    for block in segment.blocks:
                        _record(block_cases, "segment", block.keyword, case.id)
                    ####
                ####
            ####
        ####
        # The manifest records the documented problem-level production explicitly.
        # The parser retains the physically ambiguous attachment and emits an
        # ``ambiguous-dual-scope-block`` warning rather than inferring intent.
        for production in case.grammar_productions:
            if production in GRAMMAR_SCOPE_CONTRACTS:
                scope, name = GRAMMAR_SCOPE_CONTRACTS[production]
                _record(block_cases, scope, name, case.id)
            ####
        ####

        for relative in case.table_files:
            path = directory / relative
            table_document = parse_table_file(path)
            for table in table_document.tables:
                table_type_cases[table.table_type.lower()].add(case.id)
            ####
            source = path.read_text(encoding="utf-8", errors="replace")
            for match in operation_pattern.finditer(source):
                operation_cases[match.group(1).lower()].add(case.id)
            ####
        ####
    ####

    blocks = [
        {"scope": scope, "name": name, "covered": bool(block_cases.get(f"{scope}:{name}")),
         "cases": sorted(block_cases.get(f"{scope}:{name}", set()))}
        for scope, name in DOCUMENTED_BLOCKS
    ]
    table_types = [
        {"name": name, "covered": bool(table_type_cases.get(name)),
         "cases": sorted(table_type_cases.get(name, set()))}
        for name in DOCUMENTED_TABLE_TYPES
    ]
    table_operations = [
        {"name": name, "covered": bool(operation_cases.get(name)),
         "cases": sorted(operation_cases.get(name, set()))}
        for name in DOCUMENTED_TABLE_OPERATIONS
    ]
    return {
        "manual_basis": "TAOS User's Manual, Chapters 3 and 4",
        "block_scopes": blocks,
        "table_types": table_types,
        "table_operations": table_operations,
        "summary": {
            "block_scopes_covered": sum(item["covered"] for item in blocks),
            "block_scopes_total": len(blocks),
            "table_types_covered": sum(item["covered"] for item in table_types),
            "table_types_total": len(table_types),
            "table_operations_covered": sum(item["covered"] for item in table_operations),
            "table_operations_total": len(table_operations),
        },
        "ambiguous_scope_contract": {
            "description": "Problem-level define/file/print after trajectory definitions are counted from the manifest contract; the parser retains the physical attachment and emits ambiguous-dual-scope-block.",
            "covered_by_manifest_contract": sorted(GRAMMAR_SCOPE_CONTRACTS),
        },
    }
####


def write_outputs(payload: dict[str, Any], root: Path | None = None) -> None:
    base = root or repository_root()
    output = base / "coverage"
    output.mkdir(parents=True, exist_ok=True)
    (output / "documented_surface_coverage.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    with (output / "documented_surface_coverage.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["category", "scope", "name", "covered", "case_count", "cases"])
        for item in payload["block_scopes"]:
            writer.writerow(["problem_block", item["scope"], item["name"], item["covered"], len(item["cases"]), ";".join(item["cases"])])
        ####
        for item in payload["table_types"]:
            writer.writerow(["table_type", "", item["name"], item["covered"], len(item["cases"]), ";".join(item["cases"])])
        ####
        for item in payload["table_operations"]:
            writer.writerow(["table_operation", "", item["name"], item["covered"], len(item["cases"]), ";".join(item["cases"])])
        ####
    ####

    summary = payload["summary"]
    lines = [
        "# Documented TAOS language-surface coverage", "",
        "This matrix checks whether complete positive integration inputs represent each documented Chapter 3 table type, Chapter 3 full-table operation, and Chapter 4 scoped data block.", "",
        f"- Scoped Chapter 4 blocks: **{summary['block_scopes_covered']}/{summary['block_scopes_total']}**",
        f"- Chapter 3 table types: **{summary['table_types_covered']}/{summary['table_types_total']}**",
        f"- Chapter 3 full-table operations: **{summary['table_operations_covered']}/{summary['table_operations_total']}**", "",
        "Representation coverage is not historical-runtime proof. Runtime tests remain gated by `TAOS_EXE`.", "",
    ]
    for title, key in [("Scoped Chapter 4 blocks", "block_scopes"), ("Chapter 3 table types", "table_types"), ("Chapter 3 full-table operations", "table_operations")]:
        lines.extend([f"## {title}", "", "| Item | Covered | Cases |", "|---|:---:|---|"])
        for item in payload[key]:
            name = f"{item.get('scope')}:{item['name']}" if item.get("scope") else item["name"]
            lines.append(f"| `{name}` | {'yes' if item['covered'] else 'no'} | {', '.join(item['cases'])} |")
        ####
        lines.append("")
    ####
    (output / "documented_surface_coverage.md").write_text("\n".join(lines), encoding="utf-8")
####


def main() -> int:
    payload = collect()
    write_outputs(payload)
    missing = [
        f"{item.get('scope', '')}:{item['name']}"
        for category in ("block_scopes", "table_types", "table_operations")
        for item in payload[category]
        if not item["covered"]
    ]
    if missing:
        print("Uncovered documented items:")
        for item in missing:
            print(f"- {item}")
        ####
        return 1
    ####
    print(json.dumps(payload["summary"], indent=2))
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
####
