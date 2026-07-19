from __future__ import annotations

from taoryx.language import parse_problem_text
from tools.compose_problem import compose


def test_external_fragments_emit_standard_problem_text() -> None:
    text = compose(
        "(fragment-test)\n*trajectory 1 vehicle start on 1\n  *initial ecic x=6379000 y=0 z=0 xdt=0 ydt=100 zdt=0 mass=1\n  *segment 1 demo\n    {{FRAGMENT:guidance}}\n    {{FRAGMENT:release}}\n    *when time>10 stop\n*end\n",
        {
            "guidance": "*fly propnav=4",
            "release": "*when altitude>{{ALTITUDE}} goto 2",
        },
        {"ALTITUDE": "18000"},
    )

    document = parse_problem_text(text)
    assert document.diagnostics == []
    ####
