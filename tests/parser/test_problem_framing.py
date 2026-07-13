from taoryx.language.problem_parser import parse_problem_text


def test_end_closes_one_problem_and_allows_the_next_problem() -> None:
    document = parse_problem_text("(first)\n*title first\n*end\n(second)\n*title second\n*end\n")

    assert [problem.name for problem in document.problems] == ["first", "second"]
    assert all(problem.ended for problem in document.problems)
    assert not [diagnostic for diagnostic in document.diagnostics if diagnostic.severity == "error"]


def test_block_after_end_is_not_attached_to_the_closed_problem() -> None:
    document = parse_problem_text("(demo)\n*end\n*title invalid\n")

    assert [block.keyword for block in document.problems[0].blocks] == []
    assert any(diagnostic.code == "block-after-end" for diagnostic in document.diagnostics)


def test_end_before_a_problem_is_an_error() -> None:
    document = parse_problem_text("*end\n")

    assert {diagnostic.code for diagnostic in document.diagnostics} >= {"end-before-problem", "missing-problem"}
