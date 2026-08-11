from __future__ import annotations

from pathlib import Path

import pytest
from taoryx.families.cadac.parity import compare_cadac_plot_channels, parse_cadac_plot

PLOT = """\
1 Synthetic AIM5 plot
  0  0  4
 time mach alphax betax
 0.0 0.8 0.0 0.0
 0.1 0.9 1.0 -2.0
"""


def test_parse_cadac_plot_groups_wrapped_tokens_by_declared_width() -> None:
    series = parse_cadac_plot(PLOT, source_name="plot1.asc")

    assert series.channels == ("time", "mach", "alphax", "betax")
    assert series.column("MACH") == pytest.approx((0.8, 0.9))
    assert len(series.rows) == 2


####


def test_compare_cadac_plot_channels_reports_pass_and_error() -> None:
    series = parse_cadac_plot(PLOT, source_name="plot1.asc")
    report = compare_cadac_plot_channels(
        series,
        {"time": (0.0, 0.1), "mach": (0.8, 0.9000001)},
        absolute_tolerance=1.0e-5,
        relative_tolerance=0.0,
    )

    assert report.passed is True
    assert report.channels[1].max_absolute_error == pytest.approx(1.0e-7)


####


def test_aim5_parity_bridge_compares_generated_plot_fixture(tmp_path: Path) -> None:
    from taoryx.families.cadac.aim5 import aim5_plot_projection, load_aim5_source_definition, run_aim5_source_compatibility
    from taoryx.families.cadac.aim5_parity import compare_aim5_source_plot
    from test_aim5 import _write_case

    input_path = _write_case(tmp_path)
    definition = load_aim5_source_definition(input_path)
    run = run_aim5_source_compatibility(definition, sample_step_s=0.1)
    projection = aim5_plot_projection(run)
    count = len(run.samples)
    channels = ("time", "mach", "alphax", "betax")
    plot_path = tmp_path / "plot1.asc"
    lines = ["1 synthetic", f"0 0 {len(channels)}", " ".join(channels)]
    for index in range(count):
        lines.append(" ".join(str(projection[channel][index]) for channel in channels))
    ####
    plot_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    report = compare_aim5_source_plot(input_path, plot_path, absolute_tolerance=1.0e-12, relative_tolerance=1.0e-12)
    assert report.report.passed is True
    assert set(report.compared_channels) >= set(channels)


####
