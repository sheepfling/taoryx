from pathlib import Path

import pytest

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.models import ProblemDocument, RuntimeBlock
from taoryx.language.problem_parser import parse_problem_text


ROOT = Path(__file__).resolve().parents[2]
FRAGMENTS = ROOT / "examples" / "fragments" / "controllers"


@pytest.mark.parametrize(
    "fragment_name, expected_lqr_count",
    [
        ("attitude-lqr-6dof.prbfrag", 1),
        ("point-mass-track-lqr.prbfrag", 1),
        ("quadrotor-hover-lqr-6dof.prbfrag", 2),
    ],
)
def test_lqr_fragments_are_parseable_controller_contracts(fragment_name: str, expected_lqr_count: int) -> None:
    fragment = (FRAGMENTS / fragment_name).read_text(encoding="utf-8")
    source = (
        "(lqr-fragment)\n"
        "*atmos none\n"
        f"{fragment}\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=1 ydt=0 zdt=0 mass=1 time=0\n"
        "  *segment 1 flight\n"
        "    *integ dt=0.1\n"
        "    *when time>0.1 stop\n"
        "*end\n"
    )
    document = parse_problem_text(source, "<lqr-fragment>", profile=GrammarProfile.TAORYX)

    assert isinstance(document, ProblemDocument)
    assert not [item for item in document.diagnostics if item.severity.value == "error"]
    blocks = document.problems[0].blocks
    assert sum(isinstance(block, RuntimeBlock) and block.declaration == "lqr" for block in blocks) == expected_lqr_count
