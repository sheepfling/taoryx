from __future__ import annotations

from collections.abc import Iterable, Sequence
from pathlib import Path


def format_scalar(value: float, *, precision: int = 6) -> str:
    text = f"{value:.{precision}f}"
    return text.rstrip("0").rstrip(".") if "." in text else text
####


def format_row(values: Sequence[float], *, precision: int = 6) -> str:
    return ", ".join(format_scalar(value, precision=precision) for value in values)
####


def wrap_assignment(name: str, values: Sequence[float], *, indent: str = "  ", continuation_indent: str = "             ", precision: int = 6) -> list[str]:
    rendered = [format_scalar(value, precision=precision) for value in values]
    if not rendered:
        raise ValueError("table rows must contain at least one value")
    ####
    lines: list[str] = []
    line = f"{indent}{name} = "
    for index, token in enumerate(rendered):
        candidate = token if index == 0 else f", {token}"
        if len(line) + len(candidate) > 78 and line.strip():
            lines.append(line.rstrip())
            line = continuation_indent + token
        else:
            line += candidate
    ####
    lines.append(line.rstrip())
    return lines
####


def write_one_dimensional_table(
    path: Path,
    *,
    title: str,
    table_type: str,
    axis_name: str,
    value_name: str,
    axis_values: Sequence[float],
    value_values: Sequence[float],
    sref: float | None = None,
    precision: int = 6,
) -> None:
    if len(axis_values) != len(value_values):
        raise ValueError("axis and value arrays must have the same length")
    ####
    lines = [f"({title})"]
    header = f"  table {table_type}({axis_name}) no-extrap"
    if sref is not None:
        header += f" sref={format_scalar(sref, precision=precision)}"
    ####
    lines.append(header)
    lines.extend(wrap_assignment(axis_name, axis_values, precision=precision))
    lines.extend(wrap_assignment(value_name, value_values, precision=precision))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
####


def write_multi_axis_table(
    path: Path,
    *,
    title: str,
    table_type: str,
    axis_names: Sequence[str],
    axis_values: Sequence[Sequence[float]],
    value_name: str,
    value_values: Sequence[float],
    sref: float | None = None,
    precision: int = 6,
) -> None:
    if not axis_names:
        raise ValueError("multi-axis tables require at least one axis")
    if len(axis_names) != len(axis_values):
        raise ValueError("axis name and value counts must match")
    if any(len(values) == 0 for values in axis_values):
        raise ValueError("axis values must be non-empty")
    expected = 1
    for values in axis_values:
        expected *= len(values)
    if len(value_values) != expected:
        raise ValueError(f"table requires {expected} values for its axis sizes")
    ####
    lines = [f"({title})"]
    header = f"  table {table_type}({','.join(axis_names)}) no-extrap"
    if sref is not None:
        header += f" sref={format_scalar(sref, precision=precision)}"
    ####
    lines.append(header)
    for name, values in zip(axis_names, axis_values, strict=True):
        lines.extend(wrap_assignment(name, values, precision=precision))
    ####
    lines.extend(wrap_assignment(value_name, value_values, precision=precision))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
####


def write_manifest(path: Path, entries: Iterable[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(entries) + "\n", encoding="utf-8")
####
