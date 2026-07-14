"""Generic SVG renderers for table-shaped data.

The plotting layer is intentionally dependency-light.  It supports:

- scalar summaries for 0D values;
- line plots for 1D tables;
- heatmaps for 2D tables;
- recursive facet stacks for 3D tables;
- sliced higher-dimensional tables by fixing one or more axes.
"""

from __future__ import annotations

import io
import math
from dataclasses import dataclass
from html import escape
from itertools import product
from typing import Any

from taoryx.tables import PreparedTable, prepare_table

AxisKey = int | str


@dataclass(frozen=True, slots=True)
class RenderedSvgPanel:
    """A self-contained SVG fragment with explicit geometry."""

    width: int
    height: int
    body: str
#####


def table_dimension(value: PreparedTable | float | int) -> int:
    """Return the plotting dimension for a scalar or prepared table."""

    if isinstance(value, PreparedTable):
        return value.dimension
    return 0
####


def slice_table(
    table: PreparedTable,
    fixed_axes: dict[AxisKey, float] | None = None,
    *,
    axis_labels: tuple[str, ...] = (),
) -> PreparedTable | float:
    """Return a lower-dimensional slice of a prepared table.

    ``fixed_axes`` may address axes by integer index or by label.  Axis values
    must match an existing coordinate in the table exactly.
    """

    normalized_labels = _normalize_axis_labels(axis_labels, table.dimension)
    fixed_indices = _normalize_fixed_axes(table, fixed_axes or {}, normalized_labels)
    return _slice_prepared_table(table, fixed_indices)
####


def render_table_svg(
    value: PreparedTable | float | int,
    *,
    axis_labels: tuple[str, ...] = (),
    value_label: str = "value",
    title: str = "",
    fixed_axes: dict[AxisKey, float] | None = None,
    max_facets: int = 12,
) -> str:
    """Render scalar or prepared table data to standalone SVG."""

    panel = _render_value_panel(
        value,
        axis_labels=_normalize_axis_labels(axis_labels, table_dimension(value)),
        value_label=value_label,
        title=title,
        fixed_axes=fixed_axes or {},
        max_facets=max_facets,
    )
    return _wrap_svg(panel, title=title)
####


def render_table_png(
    value: PreparedTable | float | int,
    *,
    axis_labels: tuple[str, ...] = (),
    value_label: str = "value",
    title: str = "",
    fixed_axes: dict[AxisKey, float] | None = None,
    max_facets: int = 12,
    dpi: int = 160,
) -> bytes:
    """Render scalar or prepared table data to a standalone PNG."""

    plt = _get_matplotlib()
    figure = _render_value_figure(
        value,
        axis_labels=_normalize_axis_labels(axis_labels, table_dimension(value)),
        value_label=value_label,
        title=title,
        fixed_axes=fixed_axes or {},
        max_facets=max_facets,
        matplotlib=plt,
    )
    buffer = io.BytesIO()
    figure.savefig(buffer, format="png", dpi=dpi, bbox_inches="tight")
    plt.close(figure)
    return buffer.getvalue()
####


def _get_matplotlib() -> Any:
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    return plt
####


def _render_value_panel(
    value: PreparedTable | float | int,
    *,
    axis_labels: tuple[str, ...],
    value_label: str,
    title: str,
    fixed_axes: dict[AxisKey, float],
    max_facets: int,
) -> RenderedSvgPanel:
    if not isinstance(value, PreparedTable):
        return _render_scalar_panel(float(value), title=title or "scalar")
    ####
    normalized_fixed = _normalize_fixed_axes(value, fixed_axes, axis_labels)
    sliced = _slice_prepared_table(value, normalized_fixed)
    if isinstance(sliced, float):
        return _render_scalar_panel(sliced, title=title or "scalar")
    ####
    remaining_labels = tuple(label for index, label in enumerate(axis_labels) if index not in normalized_fixed)
    labels = _normalize_axis_labels(remaining_labels, sliced.dimension)
    if sliced.dimension == 1:
        return _render_line_panel(sliced, labels, value_label=value_label, title=title)
    if sliced.dimension == 2:
        return _render_heatmap_panel(sliced, labels, value_label=value_label, title=title)
    return _render_facet_stack_panel(sliced, labels, value_label=value_label, title=title, max_facets=max_facets)
####


def _render_value_figure(
    value: PreparedTable | float | int,
    *,
    axis_labels: tuple[str, ...],
    value_label: str,
    title: str,
    fixed_axes: dict[AxisKey, float],
    max_facets: int,
    matplotlib: Any,
) -> Any:
    if not isinstance(value, PreparedTable):
        return _render_scalar_figure(float(value), title=title or "scalar", plt=matplotlib)
    ####
    normalized_fixed = _normalize_fixed_axes(value, fixed_axes, axis_labels)
    sliced = _slice_prepared_table(value, normalized_fixed)
    if isinstance(sliced, float):
        return _render_scalar_figure(sliced, title=title or "scalar", plt=matplotlib)
    ####
    remaining_labels = tuple(label for index, label in enumerate(axis_labels) if index not in normalized_fixed)
    labels = _normalize_axis_labels(remaining_labels, sliced.dimension)
    if sliced.dimension == 1:
        return _render_line_figure(sliced, labels, value_label=value_label, title=title, plt=matplotlib)
    if sliced.dimension == 2:
        return _render_heatmap_figure(sliced, labels, value_label=value_label, title=title, plt=matplotlib)
    if sliced.dimension == 3:
        return _render_facet_stack_figure(sliced, labels, value_label=value_label, title=title, max_facets=max_facets, plt=matplotlib)
    raise ValueError("render_table_png requires fixed axes that reduce tables to at most three dimensions")
####


def _render_scalar_figure(value: float, *, title: str, plt: Any) -> Any:
    figure, axis = plt.subplots(figsize=(4.0, 2.1), layout="constrained")
    figure.patch.set_facecolor("white")
    axis.axis("off")
    axis.text(0.5, 0.70, title, ha="center", va="center", fontsize=14, fontweight="semibold")
    axis.text(0.5, 0.34, _format_number(value), ha="center", va="center", fontsize=28, fontweight="bold")
    axis.text(0.5, 0.12, "0D scalar", ha="center", va="center", fontsize=10)
    return figure
####


def _render_line_figure(table: PreparedTable, labels: tuple[str, ...], *, value_label: str, title: str, plt: Any) -> Any:
    figure, axis = plt.subplots(figsize=(7.2, 3.6), layout="constrained")
    figure.patch.set_facecolor("white")
    axis_x = table.axes[0]
    axis.plot(axis_x, table.values, color="#2563eb", linewidth=2.2, marker="o", markersize=4)
    axis.set_xlabel(labels[0])
    axis.set_ylabel(value_label)
    if title:
        axis.set_title(title, loc="left", fontsize=13, fontweight="semibold")
    axis.grid(True, color="#cbd5e1", linewidth=0.8)
    axis.tick_params(labelsize=9)
    return figure
####


def _render_heatmap_figure(table: PreparedTable, labels: tuple[str, ...], *, value_label: str, title: str, plt: Any) -> Any:
    figure, axis = plt.subplots(figsize=(7.2, 4.4), layout="constrained")
    figure.patch.set_facecolor("white")
    matrix = _reshape_values(table)
    x_axis, y_axis = table.axes
    mesh = axis.pcolormesh(
        _axis_edges(x_axis),
        _axis_edges(y_axis),
        matrix,
        cmap="viridis",
        shading="auto",
    )
    axis.set_xlabel(labels[0])
    axis.set_ylabel(labels[1])
    colorbar = figure.colorbar(mesh, ax=axis, shrink=0.88)
    colorbar.ax.set_ylabel(value_label, rotation=270, labelpad=12)
    if title:
        axis.set_title(title, loc="left", fontsize=13, fontweight="semibold")
    colorbar.ax.tick_params(labelsize=8)
    axis.tick_params(labelsize=9)
    return figure
####


def _render_facet_stack_figure(table: PreparedTable, labels: tuple[str, ...], *, value_label: str, title: str, max_facets: int, plt: Any) -> Any:
    facet_values = table.axes[0]
    if len(facet_values) > max_facets:
        raise ValueError(f"table facet axis {labels[0]!r} has {len(facet_values)} slices; limit is {max_facets}")
    figure, axes = plt.subplots(len(facet_values), 1, figsize=(7.6, max(3.0 * len(facet_values), 3.0)), layout="constrained")
    figure.patch.set_facecolor("white")
    if len(facet_values) == 1:
        axes = (axes,)
    if title:
        figure.suptitle(title, x=0.02, ha="left", fontsize=14, fontweight="semibold")
    for axis, coordinate in zip(axes, facet_values, strict=True):
        child = slice_table(table, {0: coordinate}, axis_labels=labels)
        assert isinstance(child, PreparedTable)
        child_labels = labels[1:]
        child_title = f"{labels[0]} = {_format_number(coordinate)}"
        if child.dimension == 1:
            axis.plot(child.axes[0], child.values, color="#2563eb", linewidth=2.0, marker="o", markersize=3)
            axis.set_xlabel(child_labels[0])
            axis.set_ylabel(value_label)
            axis.set_title(child_title, loc="left", fontsize=10, fontweight="semibold")
            axis.grid(True, color="#cbd5e1", linewidth=0.7)
        elif child.dimension == 2:
            matrix = _reshape_values(child)
            mesh = axis.pcolormesh(
                _axis_edges(child.axes[0]),
                _axis_edges(child.axes[1]),
                matrix,
                cmap="viridis",
                shading="auto",
            )
            axis.set_xlabel(child_labels[0])
            axis.set_ylabel(child_labels[1])
            axis.set_title(child_title, loc="left", fontsize=10, fontweight="semibold")
            colorbar = figure.colorbar(mesh, ax=axis, shrink=0.85)
            colorbar.ax.set_ylabel(value_label, rotation=270, labelpad=10)
        else:
            raise ValueError("facet rendering expects slices to reduce to one or two dimensions")
        axis.tick_params(labelsize=8)
    return figure
####


def _render_scalar_panel(value: float, *, title: str) -> RenderedSvgPanel:
    width = 320
    height = 140
    label = _format_number(value)
    body = "\n".join(
        [
            f'<rect x="0" y="0" width="{width}" height="{height}" rx="14" ry="14" fill="#f8fafc" stroke="#cbd5e1" stroke-width="1.5" />',
            f'<text x="{width / 2:.1f}" y="34" text-anchor="middle" font-size="18" font-family="sans-serif" fill="#0f172a">{escape(title)}</text>',
            f'<text x="{width / 2:.1f}" y="{height / 2 + 20:.1f}" text-anchor="middle" font-size="34" font-family="monospace" font-weight="700" fill="#1d4ed8">{escape(label)}</text>',
            f'<text x="{width / 2:.1f}" y="{height - 18}" text-anchor="middle" font-size="12" font-family="sans-serif" fill="#475569">0D scalar</text>',
        ]
    )
    return RenderedSvgPanel(width, height, f'<g data-role="scalar">{body}</g>')
####


def _render_line_panel(table: PreparedTable, labels: tuple[str, ...], *, value_label: str, title: str) -> RenderedSvgPanel:
    width = 720
    height = 360
    left = 78
    right = 28
    top = 48
    bottom = 60
    plot_width = width - left - right
    plot_height = height - top - bottom
    axis_x = table.axes[0]
    values = table.values
    y_min = min(values)
    y_max = max(values)
    if math.isclose(y_min, y_max):
        y_min -= 1.0
        y_max += 1.0
    x_min = min(axis_x)
    x_max = max(axis_x)
    if math.isclose(x_min, x_max):
        x_min -= 1.0
        x_max += 1.0
    points = []
    for coordinate, value in zip(axis_x, values, strict=True):
        px = left + ((coordinate - x_min) / (x_max - x_min)) * plot_width
        py = top + (1.0 - (value - y_min) / (y_max - y_min)) * plot_height
        points.append(f"{px:.2f},{py:.2f}")
    tick_indices = _sample_indices(len(axis_x), 5)
    y_tick_values = _sample_values(y_min, y_max, 5)
    lines = [
        '<g data-role="line">',
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="#ffffff" stroke="#cbd5e1" stroke-width="1.2" rx="10" ry="10" />',
        f'<text x="{left}" y="26" font-size="18" font-family="sans-serif" font-weight="700" fill="#0f172a">{escape(title)}</text>' if title else "",
        f'<text x="{left}" y="{height - 18}" font-size="13" font-family="sans-serif" fill="#334155">{escape(labels[0])}</text>',
        f'<text x="14" y="{top - 6}" font-size="13" font-family="sans-serif" fill="#334155" transform="rotate(-90 14 {top - 6})">{escape(value_label)}</text>',
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_height}" stroke="#94a3b8" stroke-width="1" />',
        f'<line x1="{left}" y1="{top + plot_height}" x2="{left + plot_width}" y2="{top + plot_height}" stroke="#94a3b8" stroke-width="1" />',
        f'<polyline fill="none" stroke="#2563eb" stroke-width="2.5" points="{" ".join(points)}" />',
    ]
    for index in tick_indices:
        coordinate = axis_x[index]
        px = left + ((coordinate - x_min) / (x_max - x_min)) * plot_width
        lines.append(f'<line x1="{px:.2f}" y1="{top + plot_height}" x2="{px:.2f}" y2="{top + plot_height + 6}" stroke="#64748b" stroke-width="1" />')
        lines.append(
            f'<text x="{px:.2f}" y="{height - 24}" text-anchor="middle" font-size="11" font-family="monospace" fill="#475569">{escape(_format_number(coordinate))}</text>'
        )
    for value in y_tick_values:
        py = top + (1.0 - (value - y_min) / (y_max - y_min)) * plot_height
        lines.append(f'<line x1="{left - 6}" y1="{py:.2f}" x2="{left}" y2="{py:.2f}" stroke="#64748b" stroke-width="1" />')
        lines.append(
            f'<text x="{left - 10}" y="{py + 4:.2f}" text-anchor="end" font-size="11" font-family="monospace" fill="#475569">{escape(_format_number(value))}</text>'
        )
    for coordinate, value in zip(axis_x, values, strict=True):
        px = left + ((coordinate - x_min) / (x_max - x_min)) * plot_width
        py = top + (1.0 - (value - y_min) / (y_max - y_min)) * plot_height
        lines.append(f'<circle cx="{px:.2f}" cy="{py:.2f}" r="3.2" fill="#1d4ed8" />')
    lines.append("</g>")
    return RenderedSvgPanel(width, height, "\n".join(filter(None, lines)))
####


def _render_heatmap_panel(table: PreparedTable, labels: tuple[str, ...], *, value_label: str, title: str) -> RenderedSvgPanel:
    width = 720
    height = 420
    left = 84
    right = 28
    top = 56
    bottom = 54
    plot_width = width - left - right
    plot_height = height - top - bottom
    x_axis, y_axis = table.axes
    rows = len(y_axis)
    cols = len(x_axis)
    values = _reshape_values(table)
    flat_values = tuple(value for row in values for value in row)
    minimum = min(flat_values)
    maximum = max(flat_values)
    cell_width = plot_width / max(cols, 1)
    cell_height = plot_height / max(rows, 1)
    lines = [
        '<g data-role="heatmap">',
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="#ffffff" stroke="#cbd5e1" stroke-width="1.2" rx="10" ry="10" />',
        f'<text x="{left}" y="28" font-size="18" font-family="sans-serif" font-weight="700" fill="#0f172a">{escape(title)}</text>' if title else "",
        f'<text x="{left + plot_width / 2:.2f}" y="{height - 16}" text-anchor="middle" font-size="13" font-family="sans-serif" fill="#334155">{escape(labels[0])}</text>',
        f'<text x="18" y="{top + plot_height / 2:.2f}" text-anchor="middle" font-size="13" font-family="sans-serif" fill="#334155" transform="rotate(-90 18 {top + plot_height / 2:.2f})">{escape(labels[1])}</text>',
        f'<text x="{width - 18}" y="{top + 14}" text-anchor="end" font-size="12" font-family="sans-serif" fill="#475569">{escape(value_label)}</text>',
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_height}" stroke="#94a3b8" stroke-width="1" />',
        f'<line x1="{left}" y1="{top + plot_height}" x2="{left + plot_width}" y2="{top + plot_height}" stroke="#94a3b8" stroke-width="1" />',
    ]
    for row_index, row in enumerate(reversed(values)):
        for col_index, value in enumerate(row):
            x = left + col_index * cell_width
            y = top + row_index * cell_height
            fill = _heatmap_color(value, minimum, maximum)
            lines.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{cell_width:.2f}" height="{cell_height:.2f}" fill="{fill}" stroke="#ffffff" stroke-width="1" />')
            if rows * cols <= 25:
                text_fill = "#0f172a" if _luminance(fill) > 0.65 else "#ffffff"
                lines.append(
                    f'<text x="{x + cell_width / 2:.2f}" y="{y + cell_height / 2 + 4:.2f}" text-anchor="middle" font-size="11" font-family="monospace" fill="{text_fill}">{escape(_format_number(value))}</text>'
                )
        ####
    ####
    for index, coordinate in enumerate(x_axis):
        x = left + (index + 0.5) * cell_width
        lines.append(f'<text x="{x:.2f}" y="{height - 26}" text-anchor="middle" font-size="11" font-family="monospace" fill="#475569">{escape(_format_number(coordinate))}</text>')
    for index, coordinate in enumerate(reversed(y_axis)):
        y = top + (index + 0.5) * cell_height
        lines.append(f'<text x="{left - 12}" y="{y + 4:.2f}" text-anchor="end" font-size="11" font-family="monospace" fill="#475569">{escape(_format_number(coordinate))}</text>')
    lines.append("</g>")
    return RenderedSvgPanel(width, height, "\n".join(filter(None, lines)))
####


def _render_facet_stack_panel(
    table: PreparedTable,
    labels: tuple[str, ...],
    *,
    value_label: str,
    title: str,
    max_facets: int,
) -> RenderedSvgPanel:
    facet_axis = 0
    facet_values = table.axes[facet_axis]
    if len(facet_values) > max_facets:
        raise ValueError(f"table facet axis {labels[0]!r} has {len(facet_values)} slices; limit is {max_facets}")
    child_width = 0
    child_height = 0
    children: list[tuple[float, RenderedSvgPanel]] = []
    for facet_index, coordinate in enumerate(facet_values):
        child = _slice_prepared_table(table, {facet_axis: facet_index})
        assert isinstance(child, PreparedTable)
        child_labels = labels[1:]
        child_title = f"{labels[0]} = {_format_number(coordinate)}"
        panel = _render_value_panel(
            child,
            axis_labels=child_labels,
            value_label=value_label,
            title=child_title,
            fixed_axes={},
            max_facets=max_facets,
        )
        children.append((coordinate, panel))
        child_width = max(child_width, panel.width)
        child_height += panel.height + 24
    ####
    width = max(child_width + 40, 420)
    height = max(child_height + 56, 260)
    parts = [
        '<g data-role="facet-stack">',
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="#ffffff" stroke="#cbd5e1" stroke-width="1.2" rx="10" ry="10" />',
        f'<text x="18" y="28" font-size="18" font-family="sans-serif" font-weight="700" fill="#0f172a">{escape(title)}</text>' if title else "",
    ]
    cursor = 44
    for coordinate, panel in children:
        parts.append(f'<text x="18" y="{cursor - 10}" font-size="12" font-family="sans-serif" fill="#334155">{escape(labels[0])} = {escape(_format_number(coordinate))}</text>')
        parts.append(f'<g transform="translate(18,{cursor})">{panel.body}</g>')
        cursor += panel.height + 24
    parts.append("</g>")
    return RenderedSvgPanel(width, height, "\n".join(filter(None, parts)))
####


def _slice_prepared_table(table: PreparedTable, fixed_axes: dict[int, int]) -> PreparedTable | float:
    remaining = tuple(index for index in range(table.dimension) if index not in fixed_axes)
    if not remaining:
        indices = [0] * table.dimension
        for axis_index, coordinate_index in fixed_axes.items():
            indices[axis_index] = coordinate_index
        return _value_at_indices(table, tuple(indices))
    new_axes = tuple(table.axes[index] for index in remaining)
    new_values: list[float] = []
    for combination in product(*(range(len(axis)) for axis in new_axes)):
        indices = [0] * table.dimension
        for axis_index, coordinate_index in fixed_axes.items():
            indices[axis_index] = coordinate_index
        for axis_index, coordinate_index in zip(remaining, combination, strict=True):
            indices[axis_index] = coordinate_index
        new_values.append(_value_at_indices(table, tuple(indices)))
    return prepare_table(new_axes, new_values, extrapolation=table.extrapolation)
####


def _value_at_indices(table: PreparedTable, indices: tuple[int, ...]) -> float:
    flat_index = sum(index * stride for index, stride in zip(indices, table.strides, strict=True))
    return table.values[flat_index]
####


def _reshape_values(table: PreparedTable) -> tuple[tuple[float, ...], ...]:
    if table.dimension != 2:
        raise ValueError("reshape_values requires a 2D table")
    rows = len(table.axes[1])
    cols = len(table.axes[0])
    matrix: list[tuple[float, ...]] = []
    for row in range(rows):
        values = []
        for col in range(cols):
            values.append(_value_at_indices(table, (col, row)))
        matrix.append(tuple(values))
    return tuple(matrix)
####


def _axis_edges(axis: tuple[float, ...]) -> list[float]:
    if len(axis) == 1:
        value = axis[0]
        return [value - 0.5, value + 0.5]
    ascending = axis[0] < axis[-1]
    centers = list(axis)
    edges: list[float] = []
    for index, value in enumerate(centers):
        if index == 0:
            step = (centers[1] - centers[0]) / 2.0
            edges.append(value - step)
        else:
            edges.append((centers[index - 1] + value) / 2.0)
    last_step = (centers[-1] - centers[-2]) / 2.0
    edges.append(centers[-1] + last_step)
    if not ascending:
        edges = list(reversed(edges))
    return edges
####


def _normalize_axis_labels(axis_labels: tuple[str, ...], dimension: int) -> tuple[str, ...]:
    if len(axis_labels) > dimension:
        raise ValueError("too many axis labels supplied for the table dimension")
    normalized = list(axis_labels)
    while len(normalized) < dimension:
        normalized.append(f"axis {len(normalized) + 1}")
    return tuple(normalized)
####


def _normalize_fixed_axes(table: PreparedTable, fixed_axes: dict[AxisKey, float], axis_labels: tuple[str, ...]) -> dict[int, int]:
    normalized: dict[int, int] = {}
    for key, coordinate in fixed_axes.items():
        if isinstance(key, str):
            try:
                axis_index = axis_labels.index(key)
            except ValueError as error:
                raise KeyError(f"unknown axis label: {key}") from error
        else:
            axis_index = key
        if axis_index < 0 or axis_index >= table.dimension:
            raise IndexError(f"axis index out of range: {axis_index}")
        axis = table.axes[axis_index]
        match = next((index for index, value in enumerate(axis) if value == float(coordinate)), None)
        if match is None:
            raise ValueError(f"slice value {coordinate!r} is not present on axis {axis_labels[axis_index]!r}")
        normalized[axis_index] = match
    return normalized
####


def _sample_indices(length: int, max_ticks: int) -> tuple[int, ...]:
    if length <= max_ticks:
        return tuple(range(length))
    candidates = {0, length - 1}
    for index in range(1, max_ticks - 1):
        candidates.add(round(index * (length - 1) / (max_ticks - 1)))
    return tuple(sorted(candidates))
####


def _sample_values(start: float, end: float, count: int) -> tuple[float, ...]:
    if count <= 1:
        return (start,)
    if math.isclose(start, end):
        return tuple(start for _ in range(count))
    return tuple(start + (end - start) * index / (count - 1) for index in range(count))
####


def _heatmap_color(value: float, minimum: float, maximum: float) -> str:
    if math.isclose(minimum, maximum):
        return "#93c5fd"
    fraction = min(1.0, max(0.0, (value - minimum) / (maximum - minimum)))
    red = round(30 + 210 * fraction)
    green = round(80 + 90 * (1.0 - abs(0.5 - fraction) * 2.0))
    blue = round(230 - 160 * fraction)
    return f"#{red:02x}{green:02x}{blue:02x}"
####


def _luminance(color: str) -> float:
    red = int(color[1:3], 16) / 255.0
    green = int(color[3:5], 16) / 255.0
    blue = int(color[5:7], 16) / 255.0
    return 0.2126 * red + 0.7152 * green + 0.0722 * blue
####


def _format_number(value: float) -> str:
    if math.isfinite(value) and float(value).is_integer():
        return str(int(value))
    return f"{value:.6g}"
####


def _wrap_svg(panel: RenderedSvgPanel, *, title: str) -> str:
    label = f'<title>{escape(title)}</title>' if title else ""
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{panel.width}" height="{panel.height}" '
        f'viewBox="0 0 {panel.width} {panel.height}" role="img" aria-label="{escape(title or "table visualization")}">'
        f"{label}{panel.body}</svg>"
    )
####
