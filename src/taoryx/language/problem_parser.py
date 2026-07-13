from __future__ import annotations

import re
from pathlib import Path
from typing import Any

from taoryx.language.diagnostics import Diagnostic, Severity, SourceLocation
from taoryx.language.expressions import (
    BinaryExpression,
    ExpressionSyntaxError,
    NameExpression,
    TableReferenceExpression,
    parse_expression,
)
from taoryx.language.grammar_contracts import (
    SUPPORTED_PROBLEM_BLOCKS,
    SUPPORTED_SEGMENT_BLOCKS,
    SUPPORTED_TRAJECTORY_BLOCKS,
)
from taoryx.language.models import (
    AeroBlock,
    Assignment,
    AtmosBlock,
    CgBlock,
    ConstantsBlock,
    DefineAssignmentStatement,
    DefineBlock,
    DefineControlStatement,
    DownrangeCrossrangeBlock,
    EarthBlock,
    EgsBlock,
    FileBlock,
    FlyBlock,
    FlyPoint,
    IipBlock,
    IncrementBlock,
    InertialBlock,
    InitialBlock,
    IntegrationBlock,
    Limit,
    LimitsBlock,
    OptimizeBlock,
    OptimizeConstraint,
    OptimizeEndpoint,
    PrintBlock,
    Problem,
    ProblemDocument,
    PropulsionBlock,
    RadarBlock,
    RailBlock,
    RawStatement,
    RecoveredRecord,
    ResetBlock,
    SearchBlock,
    SearchObjective,
    Segment,
    SummarizeBlock,
    SummaryOperand,
    SummaryOperation,
    SurveyBlock,
    SurveySetting,
    TangentBlock,
    TitleBlock,
    Trajectory,
    UnitFormatSetting,
    UnitsFormatBlock,
    WhenBlock,
    WindBlock,
)

_BLOCK_RE = re.compile(r"^\s*\*(?P<keyword>[A-Za-z0-9_/]+)\b(?P<header>.*)$")
_PROBLEM_RE = re.compile(r"^\s*\((?P<name>[^()]+)\)\s*$")
_ASSIGN_RE = re.compile(r"(?P<name>[A-Za-z_][A-Za-z0-9_./-]*(?:\[\d+\])?)\s*(?P<op>\+=|-=|\*=|/=|<=|>=|==|!=|=|<|>)\s*(?P<value>\([^()]+\)|\*|[^\s,;]+)")
_STATEMENT_ASSIGN_RE = re.compile(r"^\s*(?P<name>[A-Za-z_][A-Za-z0-9_./-]*(?:\[\d+\])?)\s*(?P<op>\+=|-=|\*=|/=|<=|>=|==|!=|=|<|>)\s*(?P<value>.+?)\s*;?\s*$")
_IF_ASSIGN_RE = re.compile(r"^\s*if\s*(?:\((?P<parenthesized>.+?)\)|(?P<condition>.+?))\s+(?:then\s+)?(?P<assignment>.+?)\s*;?\s*$", re.IGNORECASE)
_ELSE_ASSIGN_RE = re.compile(r"^\s*else\s+(?P<assignment>.+?)\s*;?\s*$", re.IGNORECASE)
_ELSE_IF_RE = re.compile(r"^\s*else\s+(?P<statement>if\b.+)$", re.IGNORECASE)
_IF_BRACE_RE = re.compile(r"^\s*if\s*\((?P<condition>.+?)\)\s*\{\s*$", re.IGNORECASE)
_ELSE_BRACE_RE = re.compile(r"^\s*\}\s*else\s*\{\s*$", re.IGNORECASE)
_CLOSE_BRACE_RE = re.compile(r"^\s*\}\s*$")
_LIMIT_RE = re.compile(r"(?P<variable>[A-Za-z_][A-Za-z0-9_.-]*)\s*(?P<operator>[<>])\s*(?P<value>[^\s,]+)")
_FLY_ASSIGN_RE = re.compile(r"^\s*(?P<variable>[A-Za-z_][A-Za-z0-9_./-]*)\s*=\s*(?P<value>.+?)\s*$", re.IGNORECASE)
_FLY_VRS_RE = re.compile(
    r"^\s*(?P<variable>[A-Za-z_][A-Za-z0-9_./-]*)\s+vrs\s+(?P<reference>[A-Za-z_][A-Za-z0-9_./-]*)(?:\s+(?P<interpolation>interp-[0-9]+))?\s*$",
    re.IGNORECASE,
)
_OPTIMIZE_ENDPOINT_RE = re.compile(
    r"^(?P<text>[A-Za-z_][A-Za-z0-9_.-]*(?:\[\d+\])?|[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[EeDd][+-]?\d+)?|\*)"
    r"(?:\s+on\s+segment\s+(?P<segment>\d+)(?:\s*,?\s*trajectory\s+(?P<trajectory>\d+))?)?\s*$",
    re.IGNORECASE,
)
_FREE_FIELD_SPLIT_RE = re.compile(r"[\s,=():<>]+")
_OPTIMIZE_CONTROL_NAMES = {"fref", "derivs", "tol", "dx", "maxitr", "adjust", "integ", "surveys", "restarts", "print"}
_SURVEY_SETTING_RE = re.compile(r"(?P<name>lo|hi|inc|vals)\s*=\s*(?P<values>.*?)(?=(?:\s+[A-Za-z_][A-Za-z0-9_./-]*\s*=)|$)", re.IGNORECASE)
_SURVEY_NUMBER_RE = re.compile(r"^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[EeDd][+-]?\d+)?$")
_SEARCH_CONTROL_NAMES = {"xlo", "xhi", "xest", "dx", "tol", "xref", "fref", "maxitr", "integ", "print"}
_RADAR_PARAMETER_NAMES = {"alt", "long", "latgd", "diste", "distn", "distd", "reqtr", "rpolr", "flat", "ecc"}
_RAIL_PARAMETER_NAMES = {"cfstat", "cfslid"}
_INTEGRATION_PARAMETER_NAMES = {"dt", "dtprnt", "dtguid"}
_RESET_INCREMENT_VARIABLE_NAMES = {
    "alt", "long", "latgd", "rcm", "latgc", "xecfc", "yecfc", "zecfc", "xecic", "yecic", "zecic",
    "dxb", "dyb", "dzb", "vel", "gamgc", "psigc", "gamgd", "psigd", "xecfcdt", "yecfcdt", "zecfcdt",
    "xecicdt", "yecicdt", "zecicdt", "wt", "mass", "fuel", "time", "tseg", "tmark", "range", "grseg",
    "grmark", "plength", "plseg", "plmark", "iip_beta", "velibx",
}
_AERO_COEFFICIENT_SETS = (
    {"ca", "cn"},
    {"cl", "cd", "cs"},
    {"cx", "cy", "cz"},
)
_FORMAT_RE = re.compile(r"^[ef]\.\d+$", re.IGNORECASE)
_ATMOS_STANDARD_MODELS = {"none", "standard", *{str(number) for number in range(21)}}
_EARTH_MODELS = {"spherical", "wgs-72", "wgs-84", "tsap-72", "tsap-84", "wgs-84-full", "gem-t1-full"}
_EARTH_PARAMETER_NAMES = {"reqtr", "rpolr", "ecc", "flat", "omega", "g", "gm", "j2", "j3", "j4", "c20", "c22", "c30", "c31", "c32", "c33", "c40", "c41", "c42", "c43", "c44", "s22", "s31", "s32", "s33", "s41", "s42", "s43", "s44"}
_OUTPUT_VARIABLE_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_.-]*(?:\[\d+\])*$")
_WIND_SPEED_NAMES = {"windv", "windh", "windd"}
_WIND_COMPONENT_NAMES = {"winde", "windn", "windd"}
_SUMMARY_OPERATIONS_WITH_OPERAND = {"add", "sub", "mult", "div", "idiv", "exp", "iexp"}
_SUMMARY_OPERATIONS_WITHOUT_OPERAND = {"abs", "neg", "sqr", "sqrt", "ln", "log", "e", "sin", "cos", "tan", "asin", "acos", "atan"}
_SUMMARY_FUNCTION_RE = re.compile(r"^(?P<function>max|min|first|last)\((?P<value>[^()]+)\)(?:\s+trajectory\s+(?P<trajectory>\d+))?$", re.IGNORECASE)
_SUMMARY_SEGMENT_RE = re.compile(r"^(?P<value>\S+)\s+on\s+segment\s+(?P<segment>\d+)(?:\s*,?\s*trajectory\s+(?P<trajectory>\d+))?$", re.IGNORECASE)
_INITIAL_VARIABLES = {
    "geodetic": {"alt", "long", "lat", "vel", "gamma", "gama", "psi", "mach", "wt", "mass", "time", "range", "path", "t_0", "t_epoch", "omega_0"},
    "geocentric": {"rcm", "long", "lat", "vel", "gamma", "gama", "psi", "mach", "wt", "mass", "time", "range", "path", "t_0", "t_epoch", "omega_0"},
    "ecfc": {"x", "y", "z", "xdt", "ydt", "zdt", "mach", "wt", "mass", "time", "range", "path", "t_0", "t_epoch", "omega_0"},
    "ecic": {"x", "y", "z", "xdt", "ydt", "zdt", "mach", "wt", "mass", "time", "range", "path", "t_0", "t_epoch", "omega_0"},
}
_FLY_DIRECT_VARIABLES = {
    "alpha",
    "alphat",
    "bankgc",
    "bankgd",
    "beta",
    "betae",
    "phi",
    "pitchgc",
    "pitchgd",
    "pitchi",
    "rollgc",
    "rollgd",
    "rolli",
    "yawgc",
    "yawgd",
    "yawi",
    "power",
}
_FLY_CONDITION_VARIABLES = {
    "alt",
    "cl",
    "cs",
    "downria",
    "dynprs",
    "gamgc",
    "gamgd",
    "intercept",
    "l/d",
    "mach",
    "nx",
    "ny",
    "nz",
    "propnav",
    "psigc",
    "psigd",
    "thrust",
    "upria",
    "vel",
}
_FLY_VARIABLES = _FLY_DIRECT_VARIABLES | _FLY_CONDITION_VARIABLES
_DOCUMENTED_UNITS = {
    "ft", "in", "mi", "nm", "m", "km", "sec", "min", "hr", "deg", "rad",
    "ft/sec", "ft/min", "ft/hr", "in/sec", "in/min", "in/hr", "mi/sec", "mi/min", "mi/hr", "knots",
    "m/sec", "m/min", "m/hr", "km/sec", "km/min", "km/hr", "deg/sec", "deg/min", "deg/hr",
    "rad/sec", "rad/min", "rad/hr", "rev/sec", "rpm",
    "ft/sec2", "ft/min2", "ft/hr2", "in/sec2", "in/min2", "in/hr2", "mi/sec2", "mi/min2", "mi/hr2", "nm/hr2",
    "m/sec2", "m/min2", "m/hr2", "km/sec2", "km/min2", "km/hr2", "deg/sec2", "deg/min2", "deg/hr2",
    "rad/sec2", "rad/min2", "rad/hr2", "rev/sec2", "rev/min2", "rev/hr2",
    "lb", "slugs", "gm", "kg", "lb/sec", "lb/min", "lb/hr", "slugs/sec", "slugs/min", "slugs/hr",
    "g/sec", "g/min", "g/hr", "kg/sec", "kg/min", "kg/hr", "lbf", "n", "kn",
    "lbf/ft2", "psi", "pascal", "kpascal", "1/in", "1/ft", "1/m", "g", "ft2/sec", "m2/sec",
    "lb/ft3", "lb/m3", "g/cm3", "kg/m3",
}
_PROP_THRUST_UNITS = {"lb", "n", "kn"}
_PROP_MASS_FLOW_UNITS = {"lb/sec", "lb/min", "lb/hr", "slugs/sec", "slugs/min", "slugs/hr", "g/sec", "g/min", "g/hr", "kg/sec", "kg/min", "kg/hr"}


def _location(path: str, line: int, column: int = 1) -> SourceLocation:
    return SourceLocation(path=path, line=line, column=column)
####


def _parse_value(value: str):
    stripped = value.strip()
    if stripped.startswith("(") and stripped.endswith(")") and stripped.count("(") == 1:
        return TableReferenceExpression(name=stripped[1:-1].strip())
    ####
    return parse_expression(stripped)
####


def _assignments(text: str, path: str, line: int, diagnostics: list[Diagnostic]) -> list[Assignment]:
    if ";" in text:
        segments = [segment.strip() for segment in text.split(";") if segment.strip()]
        if len(segments) > 1:
            result: list[Assignment] = []
            for segment in segments:
                result.extend(_assignments(segment, path, line, diagnostics))
            return result
        ####
        statement = _STATEMENT_ASSIGN_RE.fullmatch(text)
        if statement is not None:
            try:
                value = _parse_value(statement.group("value"))
            except ExpressionSyntaxError as exc:
                diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-expression", message=str(exc), location=_location(path, line)))
                return []
            return [Assignment(name=statement.group("name"), operator=statement.group("op"), value=value, location=_location(path, line))]
        ####
    result: list[Assignment] = []
    for match in _ASSIGN_RE.finditer(text):
        try:
            value = _parse_value(match.group("value"))
        except ExpressionSyntaxError as exc:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-expression", message=str(exc), location=_location(path, line, match.start("value") + 1)))
            continue
        ####
        result.append(Assignment(name=match.group("name"), operator=match.group("op"), value=value, location=_location(path, line, match.start("name") + 1)))
    ####
    return result
####


def _define_control(text: str, path: str, line: int, diagnostics: list[Diagnostic]) -> DefineControlStatement | None:
    if _IF_BRACE_RE.fullmatch(text) or _ELSE_BRACE_RE.fullmatch(text) or _CLOSE_BRACE_RE.fullmatch(text):
        return None
    if match := _ELSE_IF_RE.fullmatch(text):
        nested = _define_control(match.group("statement"), path, line, diagnostics)
        return DefineControlStatement(kind="else", nested=nested, location=_location(path, line))
    if match := _IF_ASSIGN_RE.fullmatch(text):
        condition_text = match.group("parenthesized") or match.group("condition")
        try:
            condition = parse_expression(condition_text.strip())
            body_text = match.group("assignment").strip()
            nested = _define_control(body_text, path, line, diagnostics) if body_text.lower().startswith(("if ", "if(", "else ")) else None
            assignments = [] if nested else _assignments(body_text, path, line, diagnostics)
        except ExpressionSyntaxError as exc:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-define-control", message=str(exc), location=_location(path, line)))
            return None
        if nested is None and not assignments:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="invalid-define-control",
                    message=f"Expected an assignment after *define if condition: {text.strip()!r}.",
                    location=_location(path, line),
                )
            )
            return None
        return DefineControlStatement(kind="if", condition=condition, assignment=assignments[0] if assignments else None, nested=nested, location=_location(path, line))
    if match := _ELSE_ASSIGN_RE.fullmatch(text):
        assignments = _assignments(match.group("assignment"), path, line, diagnostics)
        return DefineControlStatement(kind="else", assignment=assignments[0] if assignments else None, location=_location(path, line))
    ####
    return None
####


def _define_statement(text: str, path: str, line: int, diagnostics: list[Diagnostic]) -> DefineAssignmentStatement | DefineControlStatement | None:
    if control := _define_control(text, path, line, diagnostics):
        return control
    if statement := _STATEMENT_ASSIGN_RE.fullmatch(text):
        try:
            value = _parse_value(statement.group("value"))
        except ExpressionSyntaxError as exc:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-expression", message=str(exc), location=_location(path, line)))
            return None
        return DefineAssignmentStatement(
            assignment=Assignment(name=statement.group("name"), operator=statement.group("op"), value=value, location=_location(path, line))
        )
    if text.strip().lower().startswith(("if ", "if(", "else ")):
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-define-control", message=f"Unsupported *define control statement: {text.strip()!r}.", location=_location(path, line)))
        return None
    assignments = _assignments(text, path, line, diagnostics)
    if len(assignments) == 1:
        return DefineAssignmentStatement(assignment=assignments[0])
    if text.strip():
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-define-statement", message=f"Unsupported *define statement: {text.strip()!r}.", location=_location(path, line)))
    ####
    return None
####


def _free_fields(text: str) -> list[str]:
    return [field for field in _FREE_FIELD_SPLIT_RE.split(text.strip()) if field]
####


def _validate_named_header(
    keyword: str,
    header: str,
    assignments: list[Assignment],
    allowed_names: set[str] | None,
    path: str,
    line: int,
    diagnostics: list[Diagnostic],
) -> None:
    """Validate an assignment-only header without discarding its source text."""

    if not header.strip():
        return
    ####
    spans = [match.span() for match in _ASSIGN_RE.finditer(header)]
    residual = header
    for start, end in reversed(spans):
        residual = f"{residual[:start]} {residual[end:]}"
    ####
    if re.sub(r"[\s,]+", "", residual):
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="invalid-block-header",
                message=f"Expected named assignments in '*{keyword}' header; unparsed text was preserved.",
                location=_location(path, line),
            )
        )
    ####
    for assignment in assignments:
        if allowed_names is not None and assignment.name.casefold() not in allowed_names:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="unsupported-block-parameter",
                    message=f"Parameter {assignment.name!r} is not documented for '*{keyword}'.",
                    location=assignment.location,
                )
            )
    ####
####


def _validate_output_variables(
    keyword: str,
    tokens: list[str],
    path: str,
    line: int,
    diagnostics: list[Diagnostic],
) -> None:
    for token in tokens:
        if not _OUTPUT_VARIABLE_RE.fullmatch(token):
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="invalid-output-variable",
                    message=f"Output variable {token!r} is not a documented variable/subscript token for '*{keyword}'.",
                    location=_location(path, line),
                )
            )
        ####
    ####


def _validate_wind(
    assignments: list[Assignment],
    path: str,
    line: int,
    diagnostics: list[Diagnostic],
) -> None:
    names = [assignment.name.casefold() for assignment in assignments]
    if len(names) != 3:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="invalid-wind-components",
                message="A '*wind' block requires exactly windd plus either windv/windh or winde/windn.",
                location=_location(path, line),
            )
        )
        return
    ####
    if len(set(names)) != len(names) or set(names) not in (_WIND_SPEED_NAMES, _WIND_COMPONENT_NAMES):
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="mixed-wind-components",
                message="Wind speed/heading variables cannot be mixed with east/north components, and each must appear once.",
                location=_location(path, line),
            )
        )
    ####


def _parse_limits(header: str, path: str, line: int, diagnostics: list[Diagnostic]) -> list[Limit]:
    limits: list[Limit] = []
    matches = list(_LIMIT_RE.finditer(header))
    residual = header
    for match in reversed(matches):
        start, end = match.span()
        residual = f"{residual[:start]} {residual[end:]}"
    ####
    for match in matches:
        try:
            value = _parse_value(match.group("value"))
        except ExpressionSyntaxError as exc:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="invalid-limit-value",
                    message=str(exc),
                    location=_location(path, line, match.start("value") + 1),
                )
            )
            continue
        ####
        limits.append(
            Limit(
                variable=match.group("variable"),
                operator=match.group("operator"),
                value=value,
                location=_location(path, line, match.start("variable") + 1),
            )
        )
    ####
    if re.sub(r"[\s,]+", "", residual):
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="invalid-limits-header",
                message="Expected one or more variable limits such as 'alpha<20'.",
                location=_location(path, line),
            )
        )
    ####
    return limits
####


def _parse_summary_operand(text: str, path: str, line: int, diagnostics: list[Diagnostic]) -> SummaryOperand | None:
    body = text.strip()
    function_match = _SUMMARY_FUNCTION_RE.fullmatch(body)
    if function_match:
        try:
            expression = _parse_value(function_match.group("value"))
        except ExpressionSyntaxError as exc:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-summary-operand", message=str(exc), location=_location(path, line)))
            return None
        ####
        return SummaryOperand(
            text=body,
            expression=expression,
            function=function_match.group("function").casefold(),
            trajectory=int(function_match.group("trajectory")) if function_match.group("trajectory") else None,
        )
    ####
    segment_match = _SUMMARY_SEGMENT_RE.fullmatch(body)
    if segment_match:
        try:
            expression = _parse_value(segment_match.group("value"))
        except ExpressionSyntaxError as exc:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-summary-operand", message=str(exc), location=_location(path, line)))
            return None
        ####
        return SummaryOperand(
            text=body,
            expression=expression,
            segment=int(segment_match.group("segment")),
            trajectory=int(segment_match.group("trajectory")) if segment_match.group("trajectory") else None,
        )
    ####
    try:
        expression = _parse_value(body)
    except ExpressionSyntaxError as exc:
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-summary-operand", message=str(exc), location=_location(path, line)))
        return None
    ####
    return SummaryOperand(text=body, expression=expression)


def _parse_summary_operation(text: str, path: str, line: int, diagnostics: list[Diagnostic]) -> SummaryOperation | None:
    match = re.match(r"^\s*(?P<operation>[A-Za-z]+)(?:\s+(?P<operand>.*?))?\s*$", text)
    if match is None:
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-summary-operation", message="Expected a documented summary math operation.", location=_location(path, line)))
        return None
    ####
    operation = match.group("operation").casefold()
    operand_text = (match.group("operand") or "").strip()
    if operation not in _SUMMARY_OPERATIONS_WITH_OPERAND | _SUMMARY_OPERATIONS_WITHOUT_OPERAND:
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="unsupported-summary-operation", message=f"Operation {operation!r} is not documented for '*summarize'.", location=_location(path, line)))
        return None
    ####
    if operation in _SUMMARY_OPERATIONS_WITH_OPERAND and not operand_text:
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="missing-summary-operand", message=f"Summary operation '{operation}' requires an operand.", location=_location(path, line)))
        return None
    ####
    if operation in _SUMMARY_OPERATIONS_WITHOUT_OPERAND and operand_text:
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="unexpected-summary-operand", message=f"Summary operation '{operation}' does not accept an operand.", location=_location(path, line)))
        return None
    ####
    return SummaryOperation(operation=operation, operand=_parse_summary_operand(operand_text, path, line, diagnostics) if operand_text else None, location=_location(path, line))


def _validate_initial_assignments(block: InitialBlock, path: str, line: int, diagnostics: list[Diagnostic]) -> None:
    coordinate = block.coordinate_system or "geodetic"
    allowed = _INITIAL_VARIABLES[coordinate]
    for assignment in block.assignments:
        name = assignment.name.casefold()
        if name not in allowed:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="unsupported-initial-parameter", message=f"Parameter {assignment.name!r} is not documented for '*initial {coordinate}'.", location=assignment.location))
        ####
    names = {assignment.name.casefold() for assignment in block.assignments}
    if {"wt", "mass"} <= names:
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="conflicting-initial-mass", message="'*initial' cannot specify both wt and mass.", location=_location(path, line)))
    ####
    if {"vel", "mach"} <= names:
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="conflicting-initial-velocity", message="'*initial' cannot specify both vel and mach.", location=_location(path, line)))
    ####


def _parse_fly(header: str, path: str, line: int, diagnostics: list[Diagnostic]) -> dict[str, Any]:
    if match := _FLY_ASSIGN_RE.fullmatch(header):
        variable = match.group("variable").casefold()
        if variable not in _FLY_VARIABLES and not any(variable == base + "dt" for base in _FLY_VARIABLES):
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="unsupported-fly-variable",
                    message=f"Guidance variable {match.group('variable')!r} is not documented by the TAOS manual.",
                    location=_location(path, line),
                )
            )
        try:
            value = _parse_value(match.group("value"))
        except ExpressionSyntaxError as exc:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-fly-value", message=str(exc), location=_location(path, line)))
            return {"guidance_variable": match.group("variable")}
        ####
        return {"guidance_variable": match.group("variable"), "value": value}
    if match := _FLY_VRS_RE.fullmatch(header):
        variable = match.group("variable").casefold()
        if variable not in _FLY_DIRECT_VARIABLES:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="unsupported-fly-variable",
                    message=f"Guidance-table variable {match.group('variable')!r} is not a documented direct guidance variable.",
                    location=_location(path, line),
                )
            )
        if match.group("interpolation") and match.group("interpolation").casefold() not in {"interp-1", "interp-2", "interp-3"}:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="unsupported-fly-interpolation",
                    message="Guidance-table interpolation must be interp-1, interp-2, or interp-3.",
                    location=_location(path, line),
                )
            )
        return {
            "guidance_variable": match.group("variable"),
            "reference": match.group("reference"),
            "interpolation": match.group("interpolation"),
        }
    if header.strip().casefold() == "l/d-max":
        return {"guidance_variable": "l/d-max"}
    ####
    diagnostics.append(
        Diagnostic(
            severity=Severity.ERROR,
            code="invalid-fly-statement",
            message="Expected '*fly variable=value', '*fly variable vrs variable [interp-N]', or '*fly l/d-max'.",
            location=_location(path, line),
        )
    )
    return {"guidance_variable": header.strip().split()[0] if header.strip() else None}
####


def _parse_optimize_endpoint(text: str, path: str, line: int, diagnostics: list[Diagnostic]) -> OptimizeEndpoint | None:
    match = _OPTIMIZE_ENDPOINT_RE.fullmatch(text.strip())
    if match is None:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="invalid-optimize-endpoint",
                message=f"Unsupported optimization endpoint {text.strip()!r}; source text was preserved.",
                location=_location(path, line),
            )
        )
        return None
    ####
    endpoint_text = match.group("text")
    trajectory_subscript = None
    subscript = re.search(r"\[(\d+)\]$", endpoint_text)
    if subscript:
        trajectory_subscript = int(subscript.group(1))
    ####
    expression = None
    try:
        expression = _parse_value(endpoint_text)
    except ExpressionSyntaxError:
        pass
    ####
    return OptimizeEndpoint(
        text=endpoint_text,
        expression=expression,
        segment=int(match.group("segment")) if match.group("segment") else None,
        trajectory=int(match.group("trajectory")) if match.group("trajectory") else None,
        trajectory_subscript=trajectory_subscript,
    )
####


def _parse_optimize_constraint(text: str, path: str, line: int, diagnostics: list[Diagnostic]) -> OptimizeConstraint | None:
    body = text.strip()
    if not body.casefold().startswith("constrain"):
        return None
    ####
    body = body[len("constrain") :].strip()
    reference = None
    reference_match = re.search(r"(?:,\s*|\s+)ref\s*=\s*(\S+)\s*$", body, re.IGNORECASE)
    if reference_match:
        try:
            reference = _parse_value(reference_match.group(1))
        except ExpressionSyntaxError as exc:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-optimize-reference", message=str(exc), location=_location(path, line)))
        body = body[: reference_match.start()].rstrip(" ,")
    ####
    relation = re.search(r"(?P<operator>[<>=])", body)
    if relation is None:
        return None
    ####
    left = _parse_optimize_endpoint(body[: relation.start()], path, line, diagnostics)
    right = _parse_optimize_endpoint(body[relation.end() :], path, line, diagnostics)
    if left is None or right is None:
        return None
    ####
    return OptimizeConstraint(left=left, operator=relation.group("operator"), right=right, reference=reference, location=_location(path, line))
####


def _parse_search_objective(text: str, path: str, line: int, diagnostics: list[Diagnostic], *, report_incomplete: bool = False) -> SearchObjective | None:
    body = text.strip()
    relation = re.search(r"(?P<operator>[<>=])", body)
    if relation is None:
        left = _parse_optimize_endpoint(body, path, line, diagnostics)
        if left is None:
            return None
        ####
        return SearchObjective(left=left)
    ####
    left = _parse_optimize_endpoint(body[: relation.start()], path, line, diagnostics)
    right = _parse_optimize_endpoint(body[relation.end() :], path, line, diagnostics)
    if left is None or right is None:
        return None
    ####
    return SearchObjective(left=left, operator=relation.group("operator"), right=right)
####


def _validate_search_controls(assignments: list[Assignment], path: str, line: int, diagnostics: list[Diagnostic]) -> None:
    for assignment in assignments:
        if assignment.name.casefold() not in _SEARCH_CONTROL_NAMES:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="unsupported-search-control", message=f"Control {assignment.name!r} is not documented in a '*search' body.", location=assignment.location))
    ####
####


def _validate_optimize_controls(assignments: list[Assignment], path: str, line: int, diagnostics: list[Diagnostic]) -> None:
    for assignment in assignments:
        name = assignment.name.casefold()
        if name in _OPTIMIZE_CONTROL_NAMES or re.fullmatch(r"(?:par|lo|hi|ref)-\d+", name):
            continue
        ####
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="unsupported-optimize-control",
                message=f"Control {assignment.name!r} is not documented in an '*optimize' body.",
                location=assignment.location,
            )
        )
    ####


def _validate_propulsion_units(block: PropulsionBlock, diagnostics: list[Diagnostic]) -> None:
    for assignment in block.assignments:
        name = assignment.name.casefold()
        allowed = _PROP_THRUST_UNITS if name == "thr_units" else _PROP_MASS_FLOW_UNITS if name == "mdt_units" else None
        if allowed is None:
            continue
        ####
        value = _unit_expression_name(assignment.value)
        if not isinstance(value, str) or value.casefold() not in allowed:
            category = "thrust" if name == "thr_units" else "mass-flow"
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="unsupported-propulsion-unit",
                    message=f"{assignment.name} must name a documented {category} unit.",
                    location=assignment.location,
                )
            )
    ####
####


def _unit_expression_name(value: Any) -> str | None:
    if isinstance(value, NameExpression):
        return value.name
    if isinstance(value, BinaryExpression) and value.operator == "/":
        left = _unit_expression_name(value.left)
        right = _unit_expression_name(value.right)
        if left is not None and right is not None:
            return f"{left}/{right}"
    ####
    return None
####


def _parse_survey_settings(text: str, path: str, line: int, diagnostics: list[Diagnostic]) -> list[SurveySetting]:
    settings: list[SurveySetting] = []
    matches = list(_SURVEY_SETTING_RE.finditer(text))
    residual = text
    for match in reversed(matches):
        start, end = match.span()
        residual = f"{residual[:start]} {residual[end:]}"
    ####
    if re.sub(r"[\s,]+", "", residual):
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-survey-setting", message="Expected lo, hi, inc, or vals assignments in a *survey body.", location=_location(path, line)))
    ####
    for match in matches:
        name = match.group("name").casefold()
        values = [value for value in re.split(r"[\s,]+", match.group("values").strip()) if value]
        if not values or any(_SURVEY_NUMBER_RE.fullmatch(value) is None for value in values):
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-survey-value", message=f"Survey setting {name!r} requires numeric value(s).", location=_location(path, line, match.start("values") + 1)))
            continue
        ####
        if name != "vals" and len(values) != 1:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-survey-value-count", message=f"Survey setting {name!r} accepts exactly one value.", location=_location(path, line, match.start("values") + 1)))
            continue
        ####
        settings.append(SurveySetting(name=name, values=values, location=_location(path, line, match.start("name") + 1)))
    ####
    return settings
####


def _parse_units_format_settings(text: str, path: str, line: int, diagnostics: list[Diagnostic]) -> list[UnitFormatSetting]:
    tokens = text.split()
    settings: list[UnitFormatSetting] = []
    index = 0
    while index < len(tokens):
        if index + 1 >= len(tokens):
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-units-format-setting", message="Expected variable and unit in *units/fmt setting.", location=_location(path, line)))
            break
        ####
        variable, second = tokens[index], tokens[index + 1]
        index += 2
        unit = None if _FORMAT_RE.fullmatch(second) else second
        format_value = second if unit is None else None
        if unit is not None and unit.casefold() not in _DOCUMENTED_UNITS:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="unsupported-unit",
                    message=f"Unit {unit!r} is not listed in the TAOS manual's allowable units.",
                    location=_location(path, line),
                )
            )
        if index < len(tokens) and _FORMAT_RE.fullmatch(tokens[index]):
            format_value = tokens[index]
            index += 1
        elif index < len(tokens) and tokens[index].lower().startswith(("e.", "f.")):
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-units-format", message=f"Invalid output format {tokens[index]!r}; expected e.N or f.N.", location=_location(path, line)))
            index += 1
        ####
        settings.append(UnitFormatSetting(variable=variable, unit=unit, format=format_value, location=_location(path, line)))
    ####
    return settings
####


def _parse_atmos_body(block: AtmosBlock, text: str, path: str, line: int, diagnostics: list[Diagnostic]) -> None:
    expected = ["alt", "temp", "pres", "rho", "sndspd", "visc"] if block.model == "user" else ["alt", "pres", "rho"]
    tokens = text.split()
    if not block.columns:
        if len(tokens) != len(expected) or {token.casefold() for token in tokens} != set(expected):
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-atmos-columns", message=f"Expected atmosphere columns {expected!r}.", location=_location(path, line)))
            return
        ####
        block.columns.extend(token.casefold() for token in tokens)
        return
    ####
    if len(tokens) != len(block.columns):
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-atmos-row", message=f"Expected {len(block.columns)} numeric atmosphere values.", location=_location(path, line)))
        return
    ####
    try:
        values = [float(token.replace("D", "E").replace("d", "e")) for token in tokens]
    except ValueError:
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-atmos-row", message="Atmosphere data rows must contain numeric values.", location=_location(path, line)))
        return
    ####
    block.rows.append(values)
####


def _make_block(keyword: str, header: str, scope: str, path: str, line: int, diagnostics: list[Diagnostic]):
    common: dict[str, Any] = {"scope": scope, "location": _location(path, line), "header": header.strip(), "assignments": _assignments(header, path, line, diagnostics)}
    if keyword == "when":
        common["assignments"] = []
    elif keyword == "limits":
        common["assignments"] = []
    elif keyword == "fly":
        common["assignments"] = []
    words = _free_fields(header)
    mapping = {
        "atmos": AtmosBlock,
        "earth": EarthBlock,
        "title": TitleBlock,
        "define": DefineBlock,
        "egs": EgsBlock,
        "file": FileBlock,
        "print": PrintBlock,
        "radar": RadarBlock,
        "optimize": OptimizeBlock,
        "search": SearchBlock,
        "summarize": SummarizeBlock,
        "survey": SurveyBlock,
        "units/fmt": UnitsFormatBlock,
        "wind": WindBlock,
        "dwn/crs": DownrangeCrossrangeBlock,
        "iip": IipBlock,
        "initial": InitialBlock,
        "tangent": TangentBlock,
        "aero": AeroBlock,
        "constants": ConstantsBlock,
        "cg": CgBlock,
        "fly": FlyBlock,
        "increment": IncrementBlock,
        "inertial": InertialBlock,
        "integ": IntegrationBlock,
        "limits": LimitsBlock,
        "prop": PropulsionBlock,
        "rail": RailBlock,
        "reset": ResetBlock,
        "when": WhenBlock,
    }
    cls = mapping.get(keyword)
    if cls is None:
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="unknown-block", message=f"Unknown TAOS block '*{keyword}'.", location=_location(path, line)))
        return None
    ####
    extra: dict[str, Any] = {}
    if keyword in {"atmos", "earth"}:
        extra["model"] = words[0] if words else None
        if keyword == "atmos":
            model = (words[0] if words else "").casefold()
            if model not in _ATMOS_STANDARD_MODELS | {"user", "site"} or len(words) != 1:
                diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-atmos-header", message="Expected '*atmos 0..20', '*atmos standard|none', '*atmos user', or '*atmos site'.", location=_location(path, line)))
        else:
            model = (words[0] if words else "").casefold()
            if model not in _EARTH_MODELS:
                diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-earth-header", message="Expected a documented *earth model family.", location=_location(path, line)))
            else:
                _validate_named_header("earth", header[len(words[0]) :].strip(), common["assignments"], _EARTH_PARAMETER_NAMES, path, line, diagnostics)
    elif keyword == "title":
        extra["title"] = header.strip()
    elif keyword == "define":
        integral_match = re.fullmatch(r"\s*integral\s+([A-Za-z_][A-Za-z0-9_.-]*)\s*=\s*(.+?)\s*", header, re.IGNORECASE)
        if integral_match:
            extra["integral"] = True
            extra["variable"] = integral_match.group(1)
            try:
                extra["initial_value"] = _parse_value(integral_match.group(2))
            except ExpressionSyntaxError as exc:
                diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-define-integral-value", message=str(exc), location=_location(path, line)))
            if scope != "trajectory":
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="invalid-integral-define-scope",
                        message="Integral *define blocks are documented only inside a trajectory.",
                        location=_location(path, line),
                    )
                )
        else:
            extra["variable"] = words[0] if words else None
    elif keyword in {"egs", "file"}:
        if keyword == "egs" and words and words[0].casefold() == "summary":
            extra["summary"] = True
            words = words[1:]
        extra["filename"] = words[0] if words else None
        extra["variables"] = words[1:] if len(words) > 1 else []
        if extra["filename"] is None:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="missing-output-filename", message=f"'*{keyword}' requires an output filename.", location=_location(path, line)))
        elif extra.get("summary") and keyword == "egs" and extra["variables"]:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-egs-summary", message="The '*egs summary file' form does not accept output variable names.", location=_location(path, line)))
        else:
            _validate_output_variables(keyword, extra["variables"], path, line, diagnostics)
    elif keyword == "print":
        extra["variables"] = words
        _validate_output_variables(keyword, words, path, line, diagnostics)
    elif keyword == "survey":
        match = re.fullmatch(r"\s*(\d+)\s+(\S+)(?:\s+(.*?))?\s*", header)
        if match:
            extra["survey_id"] = int(match.group(1))
            extra["name"] = match.group(2)
            extra["settings"] = _parse_survey_settings(match.group(3) or "", path, line, diagnostics)
        else:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-survey-header", message="Expected '*survey N name'.", location=_location(path, line)))
    elif keyword == "summarize":
        extra["name"] = words[0] if words else None
        if len(words) != 1:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-summarize-header", message="Expected '*summarize name'.", location=_location(path, line)))
    elif keyword == "radar":
        match = re.fullmatch(r"\s*(\d+)\s+(\S+)(?:\s+(.*?))?\s*", header)
        if match:
            extra["radar_id"] = int(match.group(1))
            extra["station_name"] = match.group(2)
            remainder = match.group(3) or ""
            shape_match = re.search(r"\b(wgs-72|wgs-84)\b", remainder, re.IGNORECASE)
            if shape_match:
                extra["earth_shape"] = shape_match.group(1).lower()
                remainder = f"{remainder[:shape_match.start()]} {remainder[shape_match.end():]}"
            _validate_named_header("radar", remainder, common["assignments"], _RADAR_PARAMETER_NAMES, path, line, diagnostics)
        else:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-radar-header", message="Expected '*radar N station-name [parameters]'.", location=_location(path, line)))
    elif keyword == "wind":
        coordinate_match = re.match(r"\s*(geocentric|geodetic)\b", header, re.IGNORECASE)
        coordinate = coordinate_match.group(1).casefold() if coordinate_match else None
        extra["coordinate_system"] = coordinate if coordinate in {"geocentric", "geodetic"} else None
        if coordinate is None:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-wind-header", message="Expected '*wind geocentric' or '*wind geodetic'.", location=_location(path, line)))
        assignment_header = header[coordinate_match.end() :].strip() if coordinate_match else header
        _validate_named_header("wind", assignment_header, common["assignments"], _WIND_SPEED_NAMES | _WIND_COMPONENT_NAMES, path, line, diagnostics)
        if common["assignments"]:
            _validate_wind(common["assignments"], path, line, diagnostics)
    elif keyword == "fly":
        extra.update(_parse_fly(header, path, line, diagnostics))
    elif keyword == "rail":
        extra["mode"] = words[0] if words else None
        if extra["mode"] is None or extra["mode"].casefold() not in {"launch", "sled"}:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-rail-header", message="Expected '*rail launch ...' or '*rail sled ...'.", location=_location(path, line)))
        else:
            assignment_header = header[len(words[0]) :].strip()
            _validate_named_header("rail", assignment_header, common["assignments"], _RAIL_PARAMETER_NAMES, path, line, diagnostics)
    elif keyword == "initial":
        coordinate = words[0].casefold() if words and words[0].casefold() in {"geodetic", "geocentric", "ecfc", "ecic"} else None
        copied = re.fullmatch(r"\s*from\s+segment\s+(\d+)\s*,?\s*trajectory\s+(\d+)\s*", header, re.IGNORECASE)
        legacy_copied = re.fullmatch(r"\s*from\s+trajectory\s+(\d+)\s*,?\s*segment\s+(\d+)\s*", header, re.IGNORECASE)
        if copied:
            extra["mode"] = "from"
            extra["source_segment"] = int(copied.group(1))
            extra["source_trajectory"] = int(copied.group(2))
        elif legacy_copied:
            extra["mode"] = "from"
            extra["source_trajectory"] = int(legacy_copied.group(1))
            extra["source_segment"] = int(legacy_copied.group(2))
        elif words and coordinate is None and not re.match(r"[A-Za-z_][A-Za-z0-9_.-]*\s*(?:=|<|>)", header):
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-initial-header", message="Expected a coordinate system or 'from segment N, trajectory M'.", location=_location(path, line)))
        else:
            extra["mode"] = coordinate
            extra["coordinate_system"] = coordinate
        ####
    elif keyword == "dwn/crs":
        _validate_named_header(keyword, header, common["assignments"], {"latgd", "long", "azm"}, path, line, diagnostics)
    elif keyword == "iip":
        _validate_named_header(keyword, header, common["assignments"], {"iip_beta", "iip_alt"}, path, line, diagnostics)
    elif keyword == "tangent":
        _validate_named_header(keyword, header, common["assignments"], {"latgd", "long", "alt", "azm"}, path, line, diagnostics)
    elif keyword in {"aero", "constants", "cg", "prop"}:
        _validate_named_header(keyword, header, common["assignments"], None, path, line, diagnostics)
    elif keyword == "integ":
        _validate_named_header(keyword, header, common["assignments"], _INTEGRATION_PARAMETER_NAMES, path, line, diagnostics)
    elif keyword in {"reset", "increment"}:
        _validate_named_header(keyword, header, common["assignments"], _RESET_INCREMENT_VARIABLE_NAMES, path, line, diagnostics)
    elif keyword == "limits":
        extra["limits"] = _parse_limits(header, path, line, diagnostics)
    elif keyword == "units/fmt":
        extra["settings"] = _parse_units_format_settings(header, path, line, diagnostics)
    elif keyword == "inertial":
        inertial_header = re.sub(r"^\s*(?:platform\s+)?(?:alignment\s+)?", "", header, flags=re.IGNORECASE)
        alignment_match = re.match(r"(?P<alignment>body|ecfc|geocentric|geodetic|velocity|wind)\b", inertial_header, re.IGNORECASE)
        if alignment_match:
            alignment = alignment_match.group("alignment").casefold()
            extra["alignment"] = alignment
            if alignment in {"ecfc", "geocentric", "geodetic"}:
                extra["coordinate_system"] = alignment
            assignment_header = inertial_header[alignment_match.end() :].strip()
        elif re.match(r"[A-Za-z_][A-Za-z0-9_.-]*\s*=", inertial_header):
            extra["alignment"] = "geodetic"
            extra["coordinate_system"] = "geodetic"
            assignment_header = inertial_header
        else:
            assignment_header = ""
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-inertial-header", message="Expected a documented inertial alignment or named position assignments.", location=_location(path, line)))
        ####
        allowed = {
            "body": {"time"},
            "ecfc": {"eastx", "easty", "eastz", "downx", "downy", "downz", "time"},
            "geocentric": {"long", "lat", "rcm", "time"},
            "geodetic": {"long", "lat", "alt", "time"},
            "velocity": {"time"},
            "wind": {"time"},
        }.get(extra.get("alignment"), {"long", "lat", "alt", "time"})
        _validate_named_header("inertial", assignment_header, common["assignments"], allowed, path, line, diagnostics)
    elif keyword == "optimize":
        match = re.search(r"^\s*([a-e])\s+for\s+(\S+)\s*=\s*(min|max)\s+on\s+segment\s+(\d+)(?:\s*,?\s*trajectory\s+(\d+))?", header, re.IGNORECASE)
        if match:
            extra.update(loop=match.group(1).lower(), objective_variable=match.group(2), objective_mode=match.group(3).lower(), segment=int(match.group(4)), trajectory=int(match.group(5)) if match.group(5) else None)
        else:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-optimize-header", message="Expected '*optimize loop for variable=min|max on segment N, trajectory N'.", location=_location(path, line)))
        ####
    elif keyword == "search":
        match = re.fullmatch(r"\s*(\d+)\s+vary\s+(.+?)\s+until\s+(.+)\s*", header, re.IGNORECASE)
        if match:
            extra["search_id"] = int(match.group(1))
            extra["variable"] = match.group(2).strip()
            extra["objective"] = _parse_search_objective(match.group(3), path, line, diagnostics)
        else:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-search-header", message="Expected '*search N vary name until objective'.", location=_location(path, line)))
    elif keyword == "when":
        match = re.match(r"\s*(.*?)\s+(goto\s+(\d+)|stop)\s*$", header, re.IGNORECASE)
        if match is None:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-when-statement", message="Expected '*when condition goto N' or '*when condition stop'.", location=_location(path, line)))
        else:
            try:
                extra["condition"] = parse_expression(match.group(1))
            except ExpressionSyntaxError as exc:
                diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-when-condition", message=str(exc), location=_location(path, line)))
            ####
            extra["action"] = "goto" if match.group(3) else "stop"
            extra["target_segment"] = int(match.group(3)) if match.group(3) else None
        ####
    return cls(**common, **extra)
####


def parse_problem_text(text: str, path: str = "<memory>") -> ProblemDocument:
    document = ProblemDocument()
    current_problem: Problem | None = None
    current_trajectory: Trajectory | None = None
    current_segment: Segment | None = None
    current_block = None
    problem_closed = False
    pending_define = ""
    pending_define_line = 0
    define_brace_stack: list[tuple[DefineControlStatement, bool]] = []
    pending_optimize = ""
    pending_optimize_line = 0
    pending_search = ""
    pending_search_line = 0
    lines = text.splitlines()

    def recover(raw_line: str, code: str, line_number: int) -> None:
        document.recovered_records.append(RecoveredRecord(text=raw_line.strip(), code=code, location=_location(path, line_number)))
    ####

    def flush_unclosed_define_braces(line_number: int) -> None:
        if not define_brace_stack:
            return
        ####
        document.diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="unclosed-define-brace",
                message=f"*define ended with {len(define_brace_stack)} unclosed braced control block(s).",
                location=_location(path, line_number),
            )
        )
        recover("{", "unclosed-define-brace", line_number)
        define_brace_stack.clear()
    ####

    def flush_pending_define(line_number: int) -> None:
        nonlocal pending_define, pending_define_line
        if not pending_define:
            return
        ####
        document.diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="incomplete-define-statement",
                message="*define input ended before a semicolon terminated the statement; source text was preserved.",
                location=_location(path, pending_define_line or line_number),
            )
        )
        recover(pending_define, "incomplete-define-statement", pending_define_line or line_number)
        pending_define = ""
        pending_define_line = 0
    ####

    def flush_pending_optimize() -> None:
        nonlocal pending_optimize, pending_optimize_line
        if not pending_optimize:
            return
        ####
        document.diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="invalid-optimize-constraint",
                message="Incomplete optimization constraint; source text was preserved.",
                location=_location(path, pending_optimize_line),
            )
        )
        recover(pending_optimize, "invalid-optimize-constraint", pending_optimize_line)
        pending_optimize = ""
        pending_optimize_line = 0
    ####

    def flush_pending_search() -> None:
        nonlocal pending_search, pending_search_line
        if not pending_search:
            return
        ####
        document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-search-objective", message="Incomplete search objective; source text was preserved.", location=_location(path, pending_search_line)))
        recover(pending_search, "invalid-search-objective", pending_search_line)
        pending_search = ""
        pending_search_line = 0
    ####

    for number, original in enumerate(lines, start=1):
        line = original.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        ####
        problem_match = _PROBLEM_RE.match(line)
        if problem_match:
            if isinstance(current_block, DefineBlock):
                flush_pending_define(number)
                flush_unclosed_define_braces(number)
            ####
            current_problem = Problem(name=problem_match.group("name").strip(), location=_location(path, number))
            document.problems.append(current_problem)
            current_trajectory = None
            current_segment = None
            current_block = None
            problem_closed = False
            pending_define = ""
            define_brace_stack.clear()
            continue
        ####
        block_match = _BLOCK_RE.match(line)
        if block_match:
            if isinstance(current_block, DefineBlock):
                flush_pending_define(number)
                flush_unclosed_define_braces(number)
            ####
            flush_pending_optimize()
            flush_pending_search()
            keyword = block_match.group("keyword").lower()
            header = block_match.group("header").strip()
            if keyword == "end":
                if current_problem is None:
                    document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="end-before-problem", message="'*end' appears before a problem name.", location=_location(path, number)))
                    recover(line, "end-before-problem", number)
                elif problem_closed:
                    document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="duplicate-end", message="A problem cannot contain more than one '*end'.", location=_location(path, number)))
                    recover(line, "duplicate-end", number)
                else:
                    current_problem.ended = True
                    problem_closed = True
                ####
                current_block = None
                current_trajectory = None
                current_segment = None
                pending_define = ""
                define_brace_stack.clear()
                continue
            ####
            if current_problem is None:
                document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="block-before-problem", message=f"Block '*{keyword}' appears before a problem name.", location=_location(path, number)))
                recover(line, "block-before-problem", number)
                continue
            ####
            if problem_closed:
                document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="block-after-end", message=f"Block '*{keyword}' appears after '*end'.", location=_location(path, number)))
                recover(line, "block-after-end", number)
                continue
            ####
            if keyword == "trajectory":
                match = re.match(r"(\d+)\s+(.*?)\s+start\s+on\s+(\d+)\s*$", header, re.IGNORECASE)
                if not match:
                    document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-trajectory-header", message="Expected '*trajectory N name start on S'.", location=_location(path, number)))
                    recover(line, "invalid-trajectory-header", number)
                    continue
                ####
                current_trajectory = Trajectory(number=int(match.group(1)), name=match.group(2).strip(), start_segment=int(match.group(3)), location=_location(path, number))
                current_problem.trajectories.append(current_trajectory)
                current_segment = None
                current_block = None
                continue
            ####
            if keyword == "segment":
                if current_trajectory is None:
                    document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="segment-outside-trajectory", message="Segment appears outside a trajectory.", location=_location(path, number)))
                    recover(line, "segment-outside-trajectory", number)
                    continue
                ####
                match = re.match(r"(\d+)(?:\s+(.*))?$", header)
                if not match:
                    document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-segment-header", message="Expected '*segment N [title]'.", location=_location(path, number)))
                    recover(line, "invalid-segment-header", number)
                    continue
                ####
                current_segment = Segment(number=int(match.group(1)), title=(match.group(2) or "").strip(), location=_location(path, number))
                current_trajectory.segments.append(current_segment)
                current_block = None
                continue
            ####
            problem_keywords = SUPPORTED_PROBLEM_BLOCKS - {"define", "file", "print"}
            trajectory_keywords = SUPPORTED_TRAJECTORY_BLOCKS - {"define", "file", "print"}
            if keyword in problem_keywords:
                scope = "problem"
                current_segment = None
                current_trajectory = None
            elif keyword in trajectory_keywords:
                scope = "trajectory"
                current_segment = None
            elif keyword in {"define", "file", "print"}:
                scope = "segment" if current_segment is not None else "trajectory" if current_trajectory is not None else "problem"
            elif keyword in SUPPORTED_SEGMENT_BLOCKS:
                scope = "segment" if current_segment is not None else "trajectory" if current_trajectory is not None else "problem"
            else:
                scope = "segment" if current_segment is not None else "trajectory" if current_trajectory is not None else "problem"
            ####
            diagnostic_start = len(document.diagnostics)
            block = _make_block(keyword, header, scope, path, number, document.diagnostics)
            if block is None:
                recover(line, "unknown-block", number)
                continue
            ####
            if len(document.diagnostics) > diagnostic_start:
                for diagnostic in document.diagnostics[diagnostic_start:]:
                    recover(line, diagnostic.code, number)
                ####
            current_block = block
            pending_define = ""
            pending_define_line = 0
            define_brace_stack.clear()
            if isinstance(block, SearchBlock) and block.objective is not None and block.objective.operator is None:
                search_header = re.fullmatch(r"\s*\d+\s+vary\s+.+?\s+until\s+(.+)\s*", block.header, re.IGNORECASE)
                if search_header:
                    pending_search = search_header.group(1).strip()
                    pending_search_line = number
            if scope == "segment":
                if current_segment is None:
                    document.diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="block-outside-segment",
                            message=f"Block '*{keyword}' appears without an active segment.",
                            location=_location(path, number),
                        )
                    )
                    recover(line, "block-outside-segment", number)
                    current_block = None
                    continue
                ####
                current_segment.blocks.append(block)
            elif scope == "trajectory":
                if current_trajectory is None:
                    document.diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="block-outside-trajectory",
                            message=f"Block '*{keyword}' appears without an active trajectory.",
                            location=_location(path, number),
                        )
                    )
                    recover(line, "block-outside-trajectory", number)
                    current_block = None
                    continue
                ####
                current_trajectory.blocks.append(block)
            else:
                current_problem.blocks.append(block)
            ####
            continue
        ####
        if current_block is not None:
            current_block.statements.append(RawStatement(text=line.strip(), location=_location(path, number)))
            if isinstance(current_block, DefineBlock):
                stripped = line.strip()
                if match := _IF_BRACE_RE.fullmatch(stripped):
                    try:
                        condition = parse_expression(match.group("condition").strip())
                    except ExpressionSyntaxError as exc:
                        document.diagnostics.append(
                            Diagnostic(
                                severity=Severity.ERROR,
                                code="invalid-define-control",
                                message=str(exc),
                                location=_location(path, number),
                            )
                        )
                        recover(line, "invalid-define-control", number)
                    else:
                        control = DefineControlStatement(kind="if", condition=condition, location=_location(path, number))
                        current_block.typed_statements.append(control)
                        current_block.control_statements.append(control)
                        define_brace_stack.append((control, False))
                    pending_define = ""
                    continue
                if _ELSE_BRACE_RE.fullmatch(stripped):
                    if not define_brace_stack:
                        document.diagnostics.append(
                            Diagnostic(
                                severity=Severity.ERROR,
                                code="unmatched-define-brace",
                                message="'*define else' has no open braced if statement.",
                                location=_location(path, number),
                            )
                        )
                        recover(line, "unmatched-define-brace", number)
                    else:
                        control, _ = define_brace_stack[-1]
                        define_brace_stack[-1] = (control, True)
                    pending_define = ""
                    continue
                if _CLOSE_BRACE_RE.fullmatch(stripped):
                    if not define_brace_stack:
                        document.diagnostics.append(
                            Diagnostic(
                                severity=Severity.ERROR,
                                code="unmatched-define-brace",
                                message="'*define' contains a closing brace without an open control block.",
                                location=_location(path, number),
                            )
                        )
                        recover(line, "unmatched-define-brace", number)
                    else:
                        define_brace_stack.pop()
                    pending_define = ""
                    continue
                if not pending_define:
                    pending_define_line = number
                pending_define = f"{pending_define} {stripped}".strip()
                if ";" in pending_define:
                    statements = pending_define.split(";")
                    pending_define = statements.pop()
                    for statement_text in statements:
                        typed = _define_statement(statement_text, path, pending_define_line or number, document.diagnostics)
                        if typed is None:
                            continue
                        if define_brace_stack:
                            control, in_else = define_brace_stack[-1]
                            if isinstance(typed, DefineAssignmentStatement):
                                (control.else_body if in_else else control.body).append(typed.assignment)
                            else:
                                document.diagnostics.append(
                                    Diagnostic(
                                        severity=Severity.ERROR,
                                        code="unsupported-nested-define-control",
                                        message="Nested *define controls inside braces are not yet represented.",
                                        location=_location(path, pending_define_line or number),
                                    )
                                )
                                recover(statement_text, "unsupported-nested-define-control", pending_define_line or number)
                        else:
                            current_block.typed_statements.append(typed)
                            if isinstance(typed, DefineControlStatement):
                                current_block.control_statements.append(typed)
                            else:
                                current_block.assignments.append(typed.assignment)
            elif isinstance(current_block, OptimizeBlock):
                stripped = line.strip()
                if pending_optimize and not stripped.startswith("="):
                    flush_pending_optimize()
                if pending_optimize and stripped.startswith("="):
                    candidate = f"{pending_optimize} {stripped}".strip()
                    constraint = _parse_optimize_constraint(candidate, path, pending_optimize_line, document.diagnostics)
                    if constraint is None:
                        document.diagnostics.append(
                            Diagnostic(
                                severity=Severity.ERROR,
                                code="invalid-optimize-constraint",
                                message="Malformed optimization constraint; source text was preserved.",
                                location=_location(path, pending_optimize_line),
                            )
                        )
                        recover(candidate, "invalid-optimize-constraint", pending_optimize_line)
                    else:
                        current_block.constraints.append(constraint)
                    pending_optimize = ""
                    pending_optimize_line = 0
                    continue
                if stripped.casefold().startswith("constrain"):
                    candidate = f"{pending_optimize} {stripped}".strip() if pending_optimize else stripped
                    constraint = _parse_optimize_constraint(candidate, path, pending_optimize_line or number, document.diagnostics)
                    if constraint is None:
                        if "=" not in candidate and "<" not in candidate and ">" not in candidate:
                            pending_optimize = candidate
                            pending_optimize_line = pending_optimize_line or number
                        else:
                            document.diagnostics.append(
                                Diagnostic(
                                    severity=Severity.ERROR,
                                    code="invalid-optimize-constraint",
                                    message="Malformed optimization constraint; source text was preserved.",
                                    location=_location(path, number),
                                )
                            )
                            recover(stripped, "invalid-optimize-constraint", number)
                            pending_optimize = ""
                            pending_optimize_line = 0
                    else:
                        current_block.constraints.append(constraint)
                        pending_optimize = ""
                        pending_optimize_line = 0
                    continue
                ####
                assignments = _assignments(line, path, number, document.diagnostics)
                _validate_optimize_controls(assignments, path, number, document.diagnostics)
                current_block.controls.extend(assignments)
                current_block.assignments.extend(assignments)
            elif isinstance(current_block, SearchBlock):
                stripped = line.strip()
                if pending_search and not stripped.startswith("="):
                    flush_pending_search()
                if pending_search and stripped.startswith("="):
                    objective = _parse_search_objective(f"{pending_search} {stripped}", path, pending_search_line, document.diagnostics)
                    if objective is None or objective.operator is None:
                        document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-search-objective", message="Malformed search objective relationship; source text was preserved.", location=_location(path, pending_search_line)))
                        recover(f"{pending_search} {stripped}", "invalid-search-objective", pending_search_line)
                    else:
                        current_block.objective = objective
                    pending_search = ""
                    pending_search_line = 0
                    continue
                ####
                if stripped.startswith("="):
                    document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="unexpected-search-continuation", message="Search objective continuation has no pending objective.", location=_location(path, number)))
                    recover(stripped, "unexpected-search-continuation", number)
                    continue
                ####
                assignments = _assignments(line, path, number, document.diagnostics)
                _validate_search_controls(assignments, path, number, document.diagnostics)
                current_block.controls.extend(assignments)
                current_block.assignments.extend(assignments)
            elif isinstance(current_block, SurveyBlock):
                settings = _parse_survey_settings(line, path, number, document.diagnostics)
                if not settings:
                    recover(line, "invalid-survey-setting", number)
                current_block.settings.extend(settings)
            elif isinstance(current_block, UnitsFormatBlock):
                settings = _parse_units_format_settings(line, path, number, document.diagnostics)
                if not settings:
                    recover(line, "invalid-units-format-setting", number)
                current_block.settings.extend(settings)
            elif isinstance(current_block, (FileBlock, EgsBlock, PrintBlock)):
                tokens = _free_fields(line.strip())
                if isinstance(current_block, EgsBlock) and current_block.summary:
                    document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-egs-summary", message="The '*egs summary file' form does not accept output variable lines.", location=_location(path, number)))
                    recover(line, "invalid-egs-summary", number)
                else:
                    _validate_output_variables(current_block.keyword, tokens, path, number, document.diagnostics)
                    current_block.variables.extend(tokens)
                    if any(diagnostic.location.line == number and diagnostic.code == "invalid-output-variable" for diagnostic in document.diagnostics):
                        recover(line, "invalid-output-variable", number)
            elif isinstance(current_block, WindBlock):
                assignments = _assignments(line, path, number, document.diagnostics)
                _validate_named_header("wind", line, assignments, _WIND_SPEED_NAMES | _WIND_COMPONENT_NAMES, path, number, document.diagnostics)
                current_block.assignments.extend(assignments)
                if len(current_block.assignments) >= 3:
                    before = len(document.diagnostics)
                    _validate_wind(current_block.assignments, path, number, document.diagnostics)
                    if len(document.diagnostics) > before:
                        recover(line, document.diagnostics[-1].code, number)
            elif isinstance(current_block, FlyBlock):
                if current_block.reference is None:
                    document.diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="invalid-fly-body",
                            message="Only a '*fly variable vrs state-variable' form may contain continuation rows.",
                            location=_location(path, number),
                        )
                    )
                    recover(line, "invalid-fly-body", number)
                else:
                    fields = _free_fields(line.strip())
                    if len(fields) != 2:
                        document.diagnostics.append(
                            Diagnostic(
                                severity=Severity.ERROR,
                                code="invalid-fly-data",
                                message="A guidance-table row requires exactly an independent value and a guidance value.",
                                location=_location(path, number),
                            )
                        )
                        recover(line, "invalid-fly-data", number)
                    else:
                        try:
                            independent = _parse_value(fields[0])
                            value = _parse_value(fields[1])
                        except ExpressionSyntaxError as exc:
                            document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-fly-data", message=str(exc), location=_location(path, number)))
                            recover(line, "invalid-fly-data", number)
                        else:
                            current_block.points.append(FlyPoint(independent=independent, value=value, location=_location(path, number)))
            elif isinstance(current_block, TitleBlock):
                current_block.title += "\n" + line.strip()
            elif isinstance(current_block, SummarizeBlock):
                before = len(document.diagnostics)
                operation = _parse_summary_operation(line.strip(), path, number, document.diagnostics)
                if operation is not None:
                    current_block.operations.append(operation)
                if len(document.diagnostics) > before:
                    recover(line, document.diagnostics[-1].code, number)
            elif isinstance(current_block, InitialBlock):
                assignments = _assignments(line, path, number, document.diagnostics)
                if not assignments:
                    document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-initial-assignment", message="Expected an assignment in an *initial block.", location=_location(path, number)))
                    recover(line, "invalid-initial-assignment", number)
                current_block.assignments.extend(assignments)
            elif isinstance(current_block, AtmosBlock) and current_block.model in {"user", "site"}:
                before = len(document.diagnostics)
                _parse_atmos_body(current_block, line.strip(), path, number, document.diagnostics)
                if len(document.diagnostics) > before:
                    recover(line, document.diagnostics[-1].code, number)
            else:
                assignments = _assignments(line, path, number, document.diagnostics)
                if isinstance(current_block, InitialBlock) and not assignments:
                    document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-initial-assignment", message="Expected an assignment in an *initial block.", location=_location(path, number)))
                    recover(line, "invalid-initial-assignment", number)
                elif not assignments:
                    document.diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="invalid-block-body",
                            message=f"Line is not a documented body assignment for '*{current_block.keyword}'.",
                            location=_location(path, number),
                        )
                    )
                    recover(line, "invalid-block-body", number)
                current_block.assignments.extend(assignments)
        elif current_problem is not None and current_problem.blocks and isinstance(current_problem.blocks[-1], TitleBlock):
            current_problem.blocks[-1].title += "\n" + line.strip()
        else:
            document.diagnostics.append(Diagnostic(severity=Severity.WARNING, code="orphan-line", message=f"Line is not attached to a block: {line.strip()!r}", location=_location(path, number)))
            recover(line, "orphan-line", number)
        ####
    ####
    flush_pending_define(len(lines) or 1)
    flush_pending_optimize()
    flush_pending_search()
    for problem in document.problems:
        for block in problem.blocks + [child for trajectory in problem.trajectories for child in trajectory.blocks] + [child for trajectory in problem.trajectories for segment in trajectory.segments for child in segment.blocks]:
            if isinstance(block, AeroBlock):
                coefficient_names = {assignment.name.casefold() for assignment in block.assignments}
                present_sets = [coefficient_set & coefficient_names for coefficient_set in _AERO_COEFFICIENT_SETS if coefficient_set & coefficient_names]
                if len(present_sets) > 1:
                    document.diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="inconsistent-aero-coefficients",
                            message="An *aero block cannot mix coefficient sets CA/CN, CL/CD/CS, and CX/CY/CZ.",
                            location=block.location,
                        )
                    )
                    recover(block.header or "*aero", "inconsistent-aero-coefficients", block.location.line)
            if isinstance(block, PropulsionBlock):
                before = len(document.diagnostics)
                _validate_propulsion_units(block, document.diagnostics)
                for diagnostic in document.diagnostics[before:]:
                    recover(block.header or "*prop", diagnostic.code, diagnostic.location.line)
            if isinstance(block, WindBlock) and block.coordinate_system is not None and len(block.assignments) != 3:
                document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-wind-components", message="A '*wind' block requires exactly windd plus either windv/windh or winde/windn.", location=block.location))
                recover(block.header or "*wind", "invalid-wind-components", block.location.line)
            if isinstance(block, InitialBlock) and block.mode != "from":
                before = len(document.diagnostics)
                _validate_initial_assignments(block, path, block.location.line, document.diagnostics)
                names = {assignment.name.casefold() for assignment in block.assignments}
                if not names & {"wt", "mass"}:
                    document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="missing-initial-mass", message="A direct '*initial' block requires wt or mass.", location=block.location))
                if len(document.diagnostics) > before:
                    for diagnostic in document.diagnostics[before:]:
                        recover(block.header or "*initial", diagnostic.code, block.location.line)
            if isinstance(block, (FileBlock, EgsBlock)) and block.filename is not None and not getattr(block, "summary", False) and not block.variables:
                document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="missing-output-variables", message=f"'*{block.keyword}' requires at least one output variable.", location=block.location))
                recover(block.header or f"*{block.keyword}", "missing-output-variables", block.location.line)
        ####
    ####
    if not document.problems:
        document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="missing-problem", message="No '(problem-name)' declaration was found.", location=_location(path, 1)))
    ####
    return document
####


def parse_problem_file(path: str | Path) -> ProblemDocument:
    source = Path(path)
    return parse_problem_text(source.read_text(encoding="utf-8"), str(source))
####
