from pathlib import Path

import pytest

from taoryx.language.ebnf import EbnfReferenceError, EbnfSyntaxError, load_ebnf, parse_ebnf

ROOT = Path(__file__).parents[2]


@pytest.mark.parametrize(
    "path",
    [
        ROOT / "grammars" / "taos_table.ebnf",
        ROOT / "grammars" / "taos_problem.ebnf",
        ROOT / "tests" / "fixtures" / "taos_manual_corpus_v22" / "grammar_reference" / "taos_table.ebnf",
        ROOT / "tests" / "fixtures" / "taos_manual_corpus_v22" / "grammar_reference" / "taos_problem.ebnf",
    ],
)
def test_repository_ebnf_loads_as_a_valid_executable_grammar(path: Path) -> None:
    grammar = load_ebnf(path)

    assert grammar.rules
    assert not grammar.undefined_references()
    assert "identifier" in grammar.rule_names
    ####


def test_ebnf_syntax_errors_are_source_located() -> None:
    with pytest.raises(EbnfSyntaxError) as caught:
        parse_ebnf('rule = "unterminated;\n', "broken.ebnf")

    assert caught.value.location.path == "broken.ebnf"
    assert caught.value.location.line == 1
    assert caught.value.location.column == 8
    ####


def test_ebnf_undefined_references_are_source_located() -> None:
    with pytest.raises(EbnfReferenceError) as caught:
        parse_ebnf('rule = missing ;\n', "broken.ebnf")

    assert caught.value.name == "missing"
    assert caught.value.location.path == "broken.ebnf"
    assert caught.value.location.line == 1
    assert caught.value.location.column == 8
    ####
