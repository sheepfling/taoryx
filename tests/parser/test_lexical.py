from taoryx.language.lexical import TokenKind, lex_line, lex_text, lex_title_tail
from taoryx.language.lossless import parse_lossless_text


def test_lexical_classifies_numbers_references_and_delimiters() -> None:
    line = lex_line("SURV-2, srch-3 = optA-4 1 -2.5 3.0D+02 (AERO)", line=7, source_path="demo.prb")

    assert [token.kind for token in line.tokens] == [
        TokenKind.SURVEY_REFERENCE,
        TokenKind.PUNCTUATION,
        TokenKind.SEARCH_REFERENCE,
        TokenKind.PUNCTUATION,
        TokenKind.OPTIMIZATION_REFERENCE,
        TokenKind.INTEGER,
        TokenKind.REAL,
        TokenKind.SCIENTIFIC,
        TokenKind.PUNCTUATION,
        TokenKind.IDENTIFIER,
        TokenKind.PUNCTUATION,
    ]
    assert line.tokens[7].value == 300.0
    assert line.tokens[0].normalized == "surv-2"
    assert line.tokens[0].line == 7


def test_lexical_comment_and_offsets_are_preserved() -> None:
    line = lex_line("  *TITLE Hello # keep this", line=3)

    assert line.content == "  *TITLE Hello "
    assert line.comment == " keep this"
    assert line.tokens[0].raw == "*TITLE"
    assert line.tokens[0].offset_start == 2
    assert line.tokens[0].column == 3


def test_ingestion_exposes_lexical_document_and_lossless_inline_comment() -> None:
    text = "(demo)\n*TITLE Demo # preserved\n*END\n"
    lossless = parse_lossless_text(text, source_path="demo.prb")
    lexical = lex_text(text, source_path="demo.prb")

    assert lossless.records[1].inline_comment == " preserved"
    assert lexical.lines[1].comment == " preserved"


def test_quoted_values_protect_delimiters_and_comment_markers() -> None:
    line = lex_line('NAME = "A,B # not a comment"; # real comment')

    assert line.comment == " real comment"
    assert [token.kind for token in line.tokens] == [TokenKind.IDENTIFIER, TokenKind.PUNCTUATION, TokenKind.STRING, TokenKind.PUNCTUATION]
    assert line.tokens[2].value == "A,B # not a comment"


def test_title_tail_preserves_case_and_delimiters() -> None:
    title = lex_title_tail("  Flight, Case: A # title", line=4, source_path="demo.prb")

    assert title.text == "Flight, Case: A # title"
    assert title.source.line_start == 4
