from __future__ import annotations

from taoryx.outputs import build_output_evaluation_plan


def test_output_plan_deduplicates_sources_and_reports_unavailable_evaluators() -> None:
    result = build_output_evaluation_plan(
        ("alt", "vel"),
        final_conditions=("vel", "mach"),
        expressions=("range",),
        available_evaluators={"alt", "vel"},
    )

    assert result.required == ("alt", "vel", "mach", "range")
    assert result.unavailable == ("mach", "range")
