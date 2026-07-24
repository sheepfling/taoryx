"""Generate the bounded Alpha 1 language feature/requirement matrix."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
COVERAGE = ROOT / "tests/fixtures/taos_e2e_v23/coverage/documented_surface_coverage.json"
OUTPUT = ROOT / "verification/alpha1_feature_matrix.yaml"

BLOCK_REQUIREMENTS = {
    "segment": ["TAOS-REQ-CH4-SEG-0001"],
    "trajectory": ["TAOS-REQ-CH4-TRAJ-0001"],
    "problem": ["TAOS-REQ-CH4-PROB-0001"],
}


def _slug(value: str) -> str:
    """Convert a documented name into a stable feature-ID component."""

    return re.sub(r"[^A-Z0-9]+", "_", value.upper()).strip("_")
    ####


def _base(
    feature_id: str,
    category: str,
    name: str,
    manual_scope: str,
    grammar: str,
    semantics: str,
    implementation_owner: str,
    implementation_status: str,
    fixtures: list[str],
    tests: list[str],
    requirement_ids: list[str],
) -> dict[str, Any]:
    """Build one normalized feature row."""

    return {
        "feature_id": feature_id,
        "category": category,
        "name": name,
        "manual_scope": manual_scope,
        "grammar": grammar,
        "semantics": semantics,
        "implementation_owner": implementation_owner,
        "implementation_status": implementation_status,
        "fixtures": fixtures,
        "tests": tests,
        "requirement_ids": requirement_ids,
        "claim_boundary": "bounded parser/corpus coverage; not historical TAOS runtime compatibility",
    }
    ####


def build(payload: dict[str, Any]) -> dict[str, Any]:
    """Build the feature matrix from the canonical documented coverage JSON."""

    features: list[dict[str, Any]] = []
    for item in payload["block_scopes"]:
        scope = str(item["scope"])
        name = str(item["name"])
        features.append(
            _base(
                f"TAOS-FTR-CH4-{_slug(scope)}-{_slug(name)}",
                "chapter4_block",
                f"{scope}:{name}",
                "TAOS User's Manual Chapter 4",
                "grammars/taos_problem.ebnf",
                "verification/spec/chapter4_normative_inventory.yaml",
                "src/taoryx/language/problem_parser.py",
                "covered_by_bounded_corpus" if item["covered"] else "missing_fixture",
                list(item["cases"]),
                ["tests/e2e/test_documented_coverage.py", "tests/parser/test_problem_parser.py"],
                BLOCK_REQUIREMENTS[scope],
            )
        )
        ####
    ####
    for item in payload["table_types"]:
        name = str(item["name"])
        features.append(
            _base(
                f"TAOS-FTR-CH3-TABLE-{_slug(name)}",
                "chapter3_table_type",
                name,
                "TAOS User's Manual Chapter 3",
                "grammars/taos_table.ebnf",
                "verification/spec/requirements.yaml#TAOS-REQ-GRAMMAR-0002",
                "src/taoryx/language/table_parser.py",
                "covered_by_bounded_corpus" if item["covered"] else "missing_fixture",
                list(item["cases"]),
                ["tests/e2e/test_documented_coverage.py", "tests/parser/test_table_parser.py"],
                ["TAOS-REQ-GRAMMAR-0002"],
            )
        )
        ####
    ####
    for item in payload["table_operations"]:
        name = str(item["name"])
        features.append(
            _base(
                f"TAOS-FTR-CH3-OP-{_slug(name)}",
                "chapter3_table_operation",
                name,
                "TAOS User's Manual Chapter 3",
                "grammars/taos_table.ebnf",
                "verification/spec/requirements.yaml#TAOS-REQ-GRAMMAR-0002",
                "src/taoryx/language/table_parser.py",
                "covered_by_bounded_corpus" if item["covered"] else "missing_fixture",
                list(item["cases"]),
                ["tests/e2e/test_documented_coverage.py", "tests/parser/test_table_parser.py"],
                ["TAOS-REQ-GRAMMAR-0002"],
            )
        )
        ####
    ####
    summary = payload["summary"]
    return {
        "schema_version": 1,
        "release": "taoryx-alpha-1",
        "description": "Generated bounded feature matrix for the Alpha 1 TAOS 96 table/problem language scope.",
        "source_coverage": "tests/fixtures/taos_e2e_v23/coverage/documented_surface_coverage.json",
        "generation_command": "python tools/dev.py alpha1-feature-matrix",
        "summary": {
            "feature_count": len(features),
            "covered_count": sum(feature["implementation_status"] == "covered_by_bounded_corpus" for feature in features),
            "block_scopes": summary["block_scopes_total"],
            "table_types": summary["table_types_total"],
            "table_operations": summary["table_operations_total"],
        },
        "features": features,
    }
    ####


def main() -> int:
    """Generate the tracked Alpha 1 feature matrix."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--coverage", type=Path, default=COVERAGE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    payload = json.loads(args.coverage.read_text(encoding="utf-8"))
    matrix = build(payload)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        "# Generated by tools/build_alpha1_feature_matrix.py; edit the source registries instead.\n"
        + yaml.safe_dump(matrix, sort_keys=False),
        encoding="utf-8",
    )
    print(json.dumps(matrix["summary"], indent=2))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
