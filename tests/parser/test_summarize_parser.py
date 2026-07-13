from taoryx.language.models import SummarizeBlock
from taoryx.language.problem_parser import parse_problem_text


def test_summarize_operations_preserve_documented_operands() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*summarize delta\n"
        "add last(range[1]) trajectory 1\n"
        "sub alt on segment 4, trajectory 2\n"
        "mult 1.852\n"
        "neg\n"
        "*end\n"
    )

    block = document.problems[0].blocks[0]
    assert isinstance(block, SummarizeBlock)
    assert [operation.operation for operation in block.operations] == ["add", "sub", "mult", "neg"]
    assert block.operations[0].operand is not None
    assert block.operations[0].operand.function == "last"
    assert block.operations[0].operand.trajectory == 1
    assert block.operations[1].operand is not None
    assert block.operations[1].operand.segment == 4
    assert block.operations[1].operand.trajectory == 2
    assert block.operations[2].operand is not None
    assert not document.diagnostics


def test_summarize_reports_multiple_bad_operations_and_recovers() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*summarize bad\n"
        "add\n"
        "bogus 4\n"
        "neg 1\n"
        "sub 1\n"
        "*print time\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert codes.count("missing-summary-operand") == 1
    assert codes.count("unsupported-summary-operation") == 1
    assert codes.count("unexpected-summary-operand") == 1
    assert len(document.recovered_records) >= 3
    assert len(document.problems[0].blocks[0].operations) == 1


def test_summarize_rejects_missing_name() -> None:
    document = parse_problem_text("(demo)\n*summarize\nadd 1\n*end\n")

    assert any(diagnostic.code == "invalid-summarize-header" for diagnostic in document.diagnostics)
