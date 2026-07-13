from __future__ import annotations

import random

from taoryx.language.problem_parser import parse_problem_text
from taoryx.language.table_parser import parse_table_text

_MALFORMED_ATOMS = (
    "",
    "*",
    "(",
    ")",
    "*end",
    "*trajectory",
    "*segment",
    "*define",
    "*when",
    "*fly",
    "*initial",
    "*optimize",
    "*search",
    "*wind",
    "=",
    "<",
    ">",
    "&&",
    "||",
    "{",
    "}",
    ";",
    "else",
    "if",
    "???",
    "wgs-72",
    "0",
    "-1",
    "a[1]",
    "# comment",
    "table(x)",
    "mystery",
)


def _malformed_source(randomizer: random.Random) -> str:
    lines = ["(fuzz)"]
    for _ in range(randomizer.randrange(1, 15)):
        fields = " ".join(randomizer.choice(_MALFORMED_ATOMS) for _ in range(randomizer.randrange(0, 9)))
        lines.append(" " * randomizer.randrange(6) + fields)
    return "\n".join(lines) + "\n"


def _assert_recovery_is_source_exact(text: str, document) -> None:
    source_lines = text.splitlines()
    assert all(diagnostic.location is not None for diagnostic in document.diagnostics)
    for record in document.recovered_records:
        assert 1 <= record.location.line <= len(source_lines)
        assert record.text == source_lines[record.location.line - 1]
    ####


def test_deterministic_malformed_problem_inputs_never_raise_or_lose_source() -> None:
    randomizer = random.Random(1995)
    for index in range(200):
        text = _malformed_source(randomizer)
        _assert_recovery_is_source_exact(text, parse_problem_text(text, f"<problem-fuzz-{index}>"))
    ####


def test_deterministic_malformed_table_inputs_never_raise_or_lose_source() -> None:
    randomizer = random.Random(1995)
    for index in range(200):
        text = _malformed_source(randomizer)
        _assert_recovery_is_source_exact(text, parse_table_text(text, f"<table-fuzz-{index}>"))
    ####
