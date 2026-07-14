from __future__ import annotations

from pathlib import Path

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


def test_ballistic_reentry_summary_listing_preserves_source_numeric_oracle() -> None:
    lines = Path("examples/chapter04/ballistic-reentry-summary.txt").read_text(encoding="utf-8").splitlines()
    header_index = next(index for index, line in enumerate(lines) if line.strip().startswith("ventry"))
    assert lines[header_index].split() == ["ventry", "gentry", "max_g", "max_q"]

    rows = [line.split() for line in lines[header_index + 1 :] if line.strip()]
    numeric_rows = [
        row
        for row in rows
        if len(row) == 4 and all(part.replace(".", "", 1).replace("-", "", 1).isdigit() for part in row)
    ]

    assert len(numeric_rows) == 16
    assert tuple(map(float, numeric_rows[0])) == (15000.0, -35.0, 34.481, 129020.0)
    assert tuple(map(float, numeric_rows[-1])) == (18000.0, -20.0, 31.966, 130690.0)
####


def test_problem_output_example_preserves_source_numeric_oracle() -> None:
    columns = Path("examples/chapter04/problem-output-example.txt").read_text(encoding="utf-8").splitlines()
    assert columns[0].split() == ["Time[1]", "Alt[1]", "Alt[2]", "Alt[3]", "Vel[1]", "Vel[2]", "Vel[3]"]

    first = [float(part) for part in columns[1].split()]
    last = [float(part) for part in columns[-1].split()]
    assert first == [0.0, 100000.0, 80000.0, 90000.0, 9000.0, 7000.0, 8000.0]
    assert last == [5.0, 100000.0, 80000.0, 90000.0, 8928.24, 6879.78, 7910.86]
####
