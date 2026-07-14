"""Opt-in analytical aerodynamic-table declarations.

The historical table parser deliberately does not recognize this syntax. This
module parses the extension only when the caller enables its profile, then
lowers declarations to ordinary ``cd`` table text for the existing validator.
"""

from __future__ import annotations

import hashlib
import json
import re
from typing import Literal

import numpy as np
from pydantic import BaseModel, Field

from taoryx.aero_drag_tables import DragModel, drag_coefficient, projected_area, triaxial_projected_area
from taoryx.language.diagnostics import Diagnostic, Severity, SourceLocation

_HEADER = re.compile(r"^\s*\((?P<name>[A-Za-z_][A-Za-z0-9_.-]*)\)\s*$")
_GENERATE = re.compile(
    r"^\s*generate\s+(?P<table_type>[A-Za-z_][A-Za-z0-9_-]*)\((?P<variables>[^)]*)\)\s+from\s+shape=(?P<shape>[A-Za-z_][A-Za-z0-9_-]*)\s*$",
    re.IGNORECASE,
)
_ASSIGNMENT = re.compile(r"(?P<key>[A-Za-z_][A-Za-z0-9_-]*)=(?P<value>[^\s]+)")


class AnalyticalAxis(BaseModel):
    name: str
    start: float
    stop: float
    step: float
####


class AnalyticalTableDeclaration(BaseModel):
    name: str
    table_type: Literal["cd"]
    independent_variables: list[str]
    shape: Literal["sphere", "spheroid", "cylinder", "cone", "triaxial-ellipsoid"]
    parameters: dict[str, float] = Field(default_factory=dict)
    model: str = "projected-area"
    axes: list[AnalyticalAxis] = Field(default_factory=list)
    no_extrap: bool = True
    source_text: str
    location: SourceLocation
####


class AnalyticalParseResult(BaseModel):
    source_text: str
    declaration: AnalyticalTableDeclaration | None = None
    diagnostics: list[Diagnostic] = Field(default_factory=list)
####


class LoweredAnalyticalTable(BaseModel):
    source_text: str
    table_text: str
    provenance: dict[str, str | float | list[str]]
    provenance_fingerprint: str
####


def _diagnostic(code: str, message: str, path: str, line: int) -> Diagnostic:
    return Diagnostic(
        severity=Severity.ERROR,
        code=code,
        message=message,
        location=SourceLocation(path=path, line=line),
    )
####


def _axis_values(axis: AnalyticalAxis) -> np.ndarray:
    count = int(round((axis.stop - axis.start) / axis.step))
    return axis.start + axis.step * np.arange(count + 1, dtype=np.float64)
####


def _parse_float(value: str) -> float | None:
    try:
        parsed = float(value.rstrip(","))
    except ValueError:
        return None
    return parsed if np.isfinite(parsed) else None
####


def parse_analytical_table_text(text: str, path: str = "<memory>", *, enabled: bool = False) -> AnalyticalParseResult:
    """Parse one analytical declaration when the extension profile is enabled."""

    lines = text.splitlines()
    result = AnalyticalParseResult(source_text=text)
    if not enabled:
        result.diagnostics.append(_diagnostic("analytical-extension-disabled", "Analytical table syntax requires the analytical extension profile.", path, 1))
        return result
    ####
    header = next(((index, match) for index, line in enumerate(lines, 1) if (match := _HEADER.match(line))), None)
    generate = next(((index, match) for index, line in enumerate(lines, 1) if (match := _GENERATE.match(line))), None)
    if header is None:
        result.diagnostics.append(_diagnostic("analytical-missing-name", "Analytical table declaration requires a parenthesized name.", path, 1))
        return result
    if generate is None:
        result.diagnostics.append(_diagnostic("analytical-missing-generator", "Analytical table declaration requires a generate header.", path, header[0] + 1))
        return result
    ####
    line_number, match = generate
    variables = [item.strip() for item in match.group("variables").split(",") if item.strip()]
    shape = match.group("shape").casefold()
    if shape == "triaxial":
        shape = "triaxial-ellipsoid"
    if match.group("table_type").casefold() != "cd":
        result.diagnostics.append(_diagnostic("analytical-unsupported-table-type", "Analytical generation currently supports only the cd table type.", path, line_number))
        return result
    if not variables or any(not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", item) for item in variables):
        result.diagnostics.append(_diagnostic("analytical-invalid-axis-list", "Analytical table axes must be identifier names.", path, line_number))
        return result
    ####
    parameters: dict[str, float] = {}
    model = "projected-area"
    axes: list[AnalyticalAxis] = []
    no_extrap = True
    for current_line, raw_line in enumerate(lines[line_number:], line_number + 1):
        stripped = raw_line.strip()
        if not stripped:
            continue
        if stripped.casefold() == "no-extrap":
            no_extrap = True
            continue
        axis_match = re.match(r"^(?P<axis>[A-Za-z_][A-Za-z0-9_]*)\s+(?P<rest>.+)$", stripped)
        assignments = dict(_ASSIGNMENT.findall(stripped))
        if axis_match and axis_match.group("axis").casefold() in {item.casefold() for item in variables}:
            values = {key: _parse_float(value) for key, value in assignments.items()}
            if any(value is None for value in values.values()) or not all(key in values for key in ("start", "stop", "step")):
                result.diagnostics.append(_diagnostic("analytical-invalid-axis", "An analytical axis requires numeric start, stop, and step values.", path, current_line))
                continue
            axes.append(AnalyticalAxis(name=axis_match.group("axis"), start=values["start"], stop=values["stop"], step=values["step"]))  # type: ignore[arg-type]
            continue
        for key, value in assignments.items():
            parsed = _parse_float(value)
            if key == "model":
                model = value.casefold()
            elif key == "axes":
                axis_parameters = [_parse_float(item) for item in value.split(",")]
                if len(axis_parameters) != 3 or any(item is None for item in axis_parameters):
                    result.diagnostics.append(_diagnostic("analytical-invalid-axes", "triaxial axes requires three finite numeric values.", path, current_line))
                else:
                    parameters.update(dict(zip(("a_m", "b_m", "c_m"), axis_parameters, strict=True)))  # type: ignore[arg-type]
            elif parsed is None:
                result.diagnostics.append(_diagnostic("analytical-invalid-parameter", f"Analytical parameter {key!r} requires a finite numeric value.", path, current_line))
            else:
                parameters[key] = parsed
    ####
    if not axes:
        result.diagnostics.append(_diagnostic("analytical-missing-axis", "Analytical generation requires at least one axis range.", path, line_number))
    if len(axes) != len(variables):
        result.diagnostics.append(_diagnostic("analytical-axis-mismatch", "Every declared independent variable requires exactly one axis range.", path, line_number))
    expected_variables = ["alphat", "phi"] if shape == "triaxial-ellipsoid" else ["alphat"]
    if [item.casefold() for item in variables] != expected_variables:
        result.diagnostics.append(_diagnostic("analytical-axis-shape-mismatch", f"Shape {shape!r} requires axes {expected_variables!r}.", path, line_number))
    if any(axis.step <= 0.0 or axis.stop <= axis.start for axis in axes):
        result.diagnostics.append(_diagnostic("analytical-invalid-axis-range", "Analytical axis ranges require stop > start and step > 0.", path, line_number))
    if shape not in {"sphere", "spheroid", "cylinder", "cone", "triaxial-ellipsoid"}:
        result.diagnostics.append(_diagnostic("analytical-unsupported-shape", f"Unsupported analytical shape {shape!r}.", path, line_number))
    required_parameters = {
        "sphere": ("radius",),
        "spheroid": ("axial-semi-axis", "transverse-semi-axis"),
        "cylinder": ("radius", "length"),
        "cone": ("radius", "height"),
        "triaxial-ellipsoid": ("a_m", "b_m", "c_m"),
    }.get(shape, ())
    for parameter in required_parameters:
        if parameter not in parameters:
            result.diagnostics.append(_diagnostic("analytical-missing-parameter", f"Analytical shape {shape!r} requires parameter {parameter!r}.", path, line_number))
        elif parameters[parameter] <= 0.0:
            result.diagnostics.append(_diagnostic("analytical-invalid-parameter", f"Analytical parameter {parameter!r} must be positive.", path, line_number))
    if "reference-area" not in parameters:
        result.diagnostics.append(_diagnostic("analytical-missing-reference-area", "Analytical declarations require a positive reference-area.", path, line_number))
    elif parameters["reference-area"] <= 0.0:
        result.diagnostics.append(_diagnostic("analytical-invalid-reference-area", "reference-area must be positive.", path, line_number))
    if result.diagnostics:
        return result
    ####
    result.declaration = AnalyticalTableDeclaration(
        name=header[1].group("name"),
        table_type="cd",
        independent_variables=variables,
        shape=shape,  # type: ignore[arg-type]
        parameters=parameters,
        model=model,
        axes=axes,
        no_extrap=no_extrap,
        source_text=text,
        location=SourceLocation(path=path, line=header[0]),
    )
    return result
####


def _format_values(values: np.ndarray) -> str:
    return ", ".join(format(float(value), ".12g") for value in values.ravel())
####


def lower_analytical_table(declaration: AnalyticalTableDeclaration) -> LoweredAnalyticalTable:
    """Generate ordinary table text and a stable provenance record."""

    parameters = dict(declaration.parameters)
    reference_area = parameters.pop("reference-area")
    model_parameters = {key.replace("-", "_"): value for key, value in parameters.items()}
    for short_name, runtime_name in {
        "radius": "radius_m",
        "height": "height_m",
        "length": "length_m",
        "axial-semi-axis": "axial_semi_axis_m",
        "transverse-semi-axis": "transverse_semi_axis_m",
    }.items():
        if short_name in parameters:
            model_parameters[runtime_name] = parameters[short_name]
    axis_values = [_axis_values(axis) for axis in declaration.axes]
    if declaration.shape == "triaxial-ellipsoid":
        alpha, phi = axis_values
        semi_axes = tuple(model_parameters[key] for key in ("a_m", "b_m", "c_m"))
        alpha_grid, phi_grid = np.meshgrid(alpha, phi, indexing="ij")
        areas = triaxial_projected_area(alpha_grid, phi_grid, semi_axes)  # type: ignore[arg-type]
        values = areas / reference_area
    else:
        angles = axis_values[0]
        model = DragModel(declaration.name, declaration.shape, reference_area, model_parameters)
        areas = projected_area(declaration.shape, angles, model_parameters)
        if declaration.model == "projected-area":
            values = model_parameters.get("projected_cd", 1.0) * areas / reference_area
        else:
            values = drag_coefficient(model, angles, areas)
    ####
    option = " no-extrap" if declaration.no_extrap else ""
    header = f"({declaration.name})\n  table cd({','.join(declaration.independent_variables)}){option} sref={reference_area:.12g}\n"
    assignments = "".join(f"  {axis.name} = {_format_values(axis_values[index])}\n" for index, axis in enumerate(declaration.axes))
    table_text = header + assignments + f"  cd = {_format_values(values)}\n"
    fingerprint = hashlib.sha256(declaration.source_text.encode("utf-8")).hexdigest()
    provenance: dict[str, str | float | list[str]] = {
        "generator": "taoryx.aero_drag_tables",
        "generator_version": "1",
        "name": declaration.name,
        "shape": declaration.shape,
        "model": declaration.model,
        "reference_area": reference_area,
        "axes": declaration.independent_variables,
        "parameters_json": json.dumps(declaration.parameters, sort_keys=True, separators=(",", ":")),
        "axis_grids_json": json.dumps([_axis_values(axis).tolist() for axis in declaration.axes], separators=(",", ":")),
        "source_fingerprint": fingerprint,
    }
    return LoweredAnalyticalTable(source_text=declaration.source_text, table_text=table_text, provenance=provenance, provenance_fingerprint=fingerprint)
####
