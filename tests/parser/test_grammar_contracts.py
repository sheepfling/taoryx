from taoryx.language.grammar_contracts import (
    SUPPORTED_GRAMMAR_CONTRACTS,
    SUPPORTED_PROBLEM_BLOCKS,
    TAOS96_FREE_FIELD_CONTRACT,
)
from taoryx.language.problem_parser import parse_problem_text


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
