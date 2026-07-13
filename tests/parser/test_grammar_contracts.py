import re
from pathlib import Path

import pytest
import yaml

from taoryx.language._legacy_table import TABLE_OPERATIONS, TABLE_TYPES
from taoryx.language.grammar_contracts import (
    DOCUMENTED_STATE_VARIABLES,
    SUPPORTED_FLY_GUIDANCE_RULES,
    SUPPORTED_GRAMMAR_CONTRACTS,
    SUPPORTED_PROBLEM_BLOCKS,
    SUPPORTED_SEGMENT_BLOCKS,
    SUPPORTED_TRAJECTORY_BLOCKS,
    TAOS96_FREE_FIELD_CONTRACT,
)
from taoryx.language.problem_parser import parse_problem_text
from taoryx.language.table_parser import parse_table_text

CORPUS_GRAMMAR_REFERENCE = Path(__file__).parents[1] / "fixtures" / "taos_manual_corpus_v22" / "grammar_reference"


def test_supported_contracts_are_evidence_linked() -> None:
    assert len(SUPPORTED_GRAMMAR_CONTRACTS) == 4
    assert all(contract.status == "supported-subset" for contract in SUPPORTED_GRAMMAR_CONTRACTS)
    assert all(contract.evidence_ids for contract in SUPPORTED_GRAMMAR_CONTRACTS)


def test_free_field_contract_accepts_case_and_punctuation_delimiters() -> None:
    document = parse_problem_text("(demo)\n*EGS output.egs, ALT, VEL\n*END\n")
    assert document.problems[0].ended
    block = document.problems[0].blocks[0]
    assert block.keyword == "egs"
    assert block.filename == "output.egs"
    assert block.variables == ["ALT", "VEL"]
    assert TAOS96_FREE_FIELD_CONTRACT.evidence_ids


def test_problem_catalog_is_explicit() -> None:
    assert "egs" in SUPPORTED_PROBLEM_BLOCKS
    assert "not-a-taos-block" not in SUPPORTED_PROBLEM_BLOCKS


def test_manual_problem_block_registry_matches_parser_catalogs() -> None:
    reference = yaml.safe_load((CORPUS_GRAMMAR_REFERENCE / "problem_blocks_chapter4.yaml").read_text(encoding="utf-8"))
    expected = {
        scope: {item["name"] for item in details["blocks"] if item["status"] == "complete"}
        for scope, details in reference["scopes"].items()
    }
    assert expected == {
        "problem": set(SUPPORTED_PROBLEM_BLOCKS),
        "trajectory": set(SUPPORTED_TRAJECTORY_BLOCKS),
        "segment": set(SUPPORTED_SEGMENT_BLOCKS),
    }


def test_manual_guidance_rule_registry_matches_reviewed_catalog() -> None:
    reference = yaml.safe_load((CORPUS_GRAMMAR_REFERENCE / "guidance_rules_chapter4.yaml").read_text(encoding="utf-8"))

    assert {item["name"] for item in reference["rules"]} == SUPPORTED_FLY_GUIDANCE_RULES


def test_manual_state_variable_registry_matches_parser_contract() -> None:
    reference = yaml.safe_load((CORPUS_GRAMMAR_REFERENCE / "state_variables_chapter3.yaml").read_text(encoding="utf-8"))

    assert {item["name"] for item in reference["state_variables"]} == DOCUMENTED_STATE_VARIABLES


def test_manual_table_registry_matches_parser_catalogs() -> None:
    types = yaml.safe_load((CORPUS_GRAMMAR_REFERENCE / "table_types.yaml").read_text(encoding="utf-8"))
    operations = yaml.safe_load((CORPUS_GRAMMAR_REFERENCE / "table_math_operations.yaml").read_text(encoding="utf-8"))

    assert {item["name"] for item in types["table_types"]} == TABLE_TYPES
    assert {item["name"] for item in operations["math_operations"]} == TABLE_OPERATIONS


def test_documentary_table_ebnf_enumerates_the_reviewed_table_catalogs() -> None:
    grammar = (Path(__file__).parents[2] / "grammars" / "taos_table.ebnf").read_text(encoding="utf-8")

    assert "table-type" in grammar
    assert all(f'"{table_type}"' in grammar for table_type in TABLE_TYPES)
    assert all(f'"{operation}"' in grammar for operation in TABLE_OPERATIONS)


def test_documentary_problem_ebnf_enumerates_each_reviewed_scope_catalog() -> None:
    grammar_paths = [
        Path(__file__).parents[2] / "grammars" / "taos_problem.ebnf",
        Path(__file__).parents[1] / "fixtures" / "taos_manual_corpus_v22" / "grammar_reference" / "taos_problem.ebnf",
    ]
    expected = {
        "problem-block": {
            "atmos": "atmosphere-block",
            "define": "define-block",
            "earth": "earth-block",
            "egs": "egs-block",
            "file": "file-block",
            "optimize": "optimize-block",
            "print": "print-block",
            "radar": "radar-block",
            "search": "search-block",
            "summarize": "summarize-block",
            "survey": "survey-block",
            "title": "title-block",
            "units/fmt": "units-block",
            "wind": "wind-block",
        },
        "trajectory-block": {
            "define": "define-block",
            "dwn/crs": "downrange-block",
            "file": "file-block",
            "iip": "iip-block",
            "initial": "initial-block",
            "print": "print-block",
            "tangent": "tangent-block",
        },
        "segment-block": {
            "aero": "aero-block",
            "constants": "constants-block",
            "cg": "cg-block",
            "fly": "fly-block",
            "increment": "increment-block",
            "inertial": "inertial-block",
            "integ": "integration-block",
            "limits": "limits-block",
            "prop": "propulsion-block",
            "rail": "rail-block",
            "reset": "reset-block",
            "when": "when-block",
        },
    }
    for path in grammar_paths:
        grammar = path.read_text(encoding="utf-8")
        rules = {
            match.group("name"): match.group("body")
            for match in re.finditer(
                r"(?ms)^(?P<name>[a-z][a-z0-9-]*)\s*=\s*(?P<body>.*?);\s*$",
                grammar,
            )
        }
        for rule, catalog in expected.items():
            assert rule in rules, (path, rule)
            for _, production in catalog.items():
                assert production in rules[rule], (path, rule, production)
    ####


def test_documentary_ebnf_has_no_undefined_nonterminals() -> None:
    grammar_paths = [
        Path(__file__).parents[2] / "grammars" / "taos_problem.ebnf",
        Path(__file__).parents[2] / "grammars" / "taos_table.ebnf",
        CORPUS_GRAMMAR_REFERENCE / "taos_problem.ebnf",
        CORPUS_GRAMMAR_REFERENCE / "taos_table.ebnf",
    ]
    for path in grammar_paths:
        source = path.read_text(encoding="utf-8")
        source = re.sub(r"\(\*.*?\*\)", "", source, flags=re.DOTALL)
        source = re.sub(r'"(?:[^"\\]|\\.)*"', "", source)
        definitions = set(re.findall(r"(?m)^\s*([a-z][a-z0-9-]*)\s*=", source))
        references = set(re.findall(r"\b([a-z][a-z0-9-]*)\b", source))
        assert references <= definitions, (path, sorted(references - definitions))
    ####


@pytest.mark.parametrize("table_type", sorted(TABLE_TYPES))
def test_every_manual_table_type_has_a_valid_minimal_definition(table_type: str) -> None:
    document = parse_table_text(f"({table_type}-fixture) table {table_type}\n{table_type}=1\n")

    assert not document.diagnostics, table_type
    assert document.tables[0].table_type == table_type
    ####


@pytest.mark.parametrize("operation", sorted(TABLE_OPERATIONS))
def test_every_manual_table_operation_has_a_valid_form(operation: str) -> None:
    if operation == "end":
        statement = "end"
    elif operation == "if":
        statement = "if (mach > 0) then add 1"
    elif operation == "csto":
        statement = f"{operation} label"
    elif operation == "goto":
        statement = "label: add 1\ngoto label"
    elif operation in {"abs", "neg", "sqr", "sqrt", "ln", "log", "e", "sin", "cos", "tan", "asin", "acos", "atan", "zero"}:
        statement = operation
    else:
        statement = f"{operation} 1"

    terminator = "" if operation == "end" else "end\n"
    document = parse_table_text(f"(operation-{operation}) table output\nstart\n{statement}\n{terminator}")

    assert not document.diagnostics, operation
    assert any(item.operator == operation for item in document.tables[0].operations)
    ####
