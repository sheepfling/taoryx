from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from taoryx.tables import ExtrapolationMode, prepare_table
from taoryx.visualization import render_table_png, render_table_svg, slice_table, table_dimension


def _scalar_svg(value: float) -> ET.Element:
    svg = render_table_svg(value, title="scalar")
    return ET.fromstring(svg)
####


def test_scalar_plot_renders_a_numeric_card() -> None:
    svg = _scalar_svg(12.5)

    assert svg.attrib["role"] == "img"
    assert svg.find(".//*[@data-role='scalar']") is not None
    assert "12.5" in ET.tostring(svg, encoding="unicode")


def test_one_dimensional_table_renders_a_line_plot() -> None:
    table = prepare_table(((0.0, 1.0, 2.0, 3.0),), (1.0, 2.0, 4.0, 8.0))
    svg = ET.fromstring(render_table_svg(table, axis_labels=("mach",), title="line"))

    assert svg.find(".//*[@data-role='line']") is not None
    assert "mach" in ET.tostring(svg, encoding="unicode")
    assert "8" in ET.tostring(svg, encoding="unicode")


def test_two_dimensional_table_renders_a_heatmap() -> None:
    table = prepare_table(((0.0, 1.0), (10.0, 20.0)), (1.0, 2.0, 3.0, 4.0))
    svg = ET.fromstring(render_table_svg(table, axis_labels=("x", "y"), title="heat"))

    assert svg.find(".//*[@data-role='heatmap']") is not None
    assert svg.findall(".//*[@fill]")  # at least some drawn cells
    assert "heat" in ET.tostring(svg, encoding="unicode")


def test_three_dimensional_table_becomes_a_facet_stack() -> None:
    table = prepare_table(
        ((0.0, 1.0), (10.0, 20.0), (100.0, 200.0)),
        tuple(float(index) for index in range(8)),
        extrapolation=ExtrapolationMode.CLAMP,
    )
    svg = ET.fromstring(render_table_svg(table, axis_labels=("mach", "alpha", "beta"), title="stack"))

    assert svg.find(".//*[@data-role='facet-stack']") is not None
    assert svg.find(".//*[@data-role='heatmap']") is not None
    text = ET.tostring(svg, encoding="unicode")
    assert "beta" in text
    assert "alpha" in text


def test_four_dimensional_table_can_be_sliced_to_a_heatmap() -> None:
    table = prepare_table(
        ((0.0, 1.0), (10.0, 20.0), (100.0, 200.0), (1000.0, 2000.0)),
        tuple(float(index) for index in range(16)),
    )
    sliced = slice_table(table, {0: 1.0, "gamma": 2000.0}, axis_labels=("mach", "alpha", "beta", "gamma"))

    assert table_dimension(sliced) == 2
    svg = ET.fromstring(
        render_table_svg(table, axis_labels=("mach", "alpha", "beta", "gamma"), fixed_axes={0: 1.0, "gamma": 2000.0}, title="slice")
    )
    assert svg.find(".//*[@data-role='heatmap']") is not None
    assert "slice" in ET.tostring(svg, encoding="unicode")


def test_four_dimensional_table_png_supports_fixed_axis_slices() -> None:
    table = prepare_table(
        ((0.0, 1.0), (10.0, 20.0), (100.0, 200.0), (1000.0, 2000.0)),
        tuple(float(index) for index in range(16)),
    )

    png = render_table_png(
        table,
        axis_labels=("mach", "alpha", "beta", "gamma"),
        fixed_axes={0: 1.0, "gamma": 2000.0},
        title="png-slice",
    )

    assert png[:8] == b"\x89PNG\r\n\x1a\n"


@pytest.mark.artifact
def test_table_svg_can_be_written_to_the_artifact_directory(artifact_dir) -> None:
    table = prepare_table(((0.0, 1.0, 2.0),), (1.0, 2.0, 4.0))
    output = artifact_dir / "table.svg"

    output.write_text(render_table_svg(table, axis_labels=("mach",), title="artifact"), encoding="utf-8")

    assert output.exists()
    assert '<svg xmlns="http://www.w3.org/2000/svg"' in output.read_text(encoding="utf-8")
