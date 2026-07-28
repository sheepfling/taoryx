from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Any

from taoryx.language.diagnostics import Diagnostic, Severity, SourceLocation
from taoryx.language.expressions import (
    BinaryExpression,
    CallExpression,
    ExpressionSyntaxError,
    IndexedExpression,
    NameExpression,
    NumberExpression,
    ParameterExpression,
    TableReferenceExpression,
    UnaryExpression,
    WildcardExpression,
    parse_expression,
)
from taoryx.language.grammar_contracts import (
    DOCUMENTED_STATE_VARIABLES,
    SUPPORTED_PROBLEM_BLOCKS,
    SUPPORTED_SEGMENT_BLOCKS,
    SUPPORTED_TAORYX_PROBLEM_BLOCKS,
    SUPPORTED_TAORYX_SEGMENT_BLOCKS,
    SUPPORTED_TAORYX_TRAJECTORY_BLOCKS,
    SUPPORTED_TRAJECTORY_BLOCKS,
    GrammarProfile,
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
    DofDirectiveBlock,
    DownrangeCrossrangeBlock,
    EarthBlock,
    EgsBlock,
    ExtensionBlock,
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
    ModeBlock,
    OptimizeBlock,
    OptimizeConstraint,
    OptimizeEndpoint,
    PrintBlock,
    Problem,
    ProblemDocument,
    PropulsionBlock,
    RadarBlock,
    RailBlock,
    RandomBlock,
    RawStatement,
    RecoveredRecord,
    ResetBlock,
    RuntimeBlock,
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
_PROBLEM_NAME_RE = re.compile(r"^[A-Za-z0-9_.-]+$")
_ASSIGN_RE = re.compile(r"(?P<name>[A-Za-z_][A-Za-z0-9_./-]*(?:\[\d+\])?)\s*(?P<op>\+=|-=|\*=|/=|<=|>=|==|!=|=|<|>)\s*(?P<value>\([^()]+\)|\*|[^\s,;]+)")
_STATEMENT_ASSIGN_RE = re.compile(r"^\s*(?P<name>[A-Za-z_][A-Za-z0-9_./-]*(?:\[\d+\])?)\s*(?P<op>\+=|-=|\*=|/=|<=|>=|==|!=|=|<|>)\s*(?P<value>.+?)\s*;?\s*$")
_IF_ASSIGN_RE = re.compile(r"^\s*if\s*(?:\((?P<parenthesized>.+?)\)|(?P<condition>.+?))\s+(?:then\s+)?(?P<assignment>.+?)\s*;?\s*$", re.IGNORECASE)
_ELSE_ASSIGN_RE = re.compile(r"^\s*else\s+(?P<assignment>.+?)\s*;?\s*$", re.IGNORECASE)
_ELSE_IF_RE = re.compile(r"^\s*else\s+(?P<statement>if\b.+)$", re.IGNORECASE)
_IF_BRACE_RE = re.compile(r"^\s*if\s*\((?P<condition>.+?)\)\s*\{\s*$", re.IGNORECASE)
_ELSE_BRACE_RE = re.compile(r"^\s*\}\s*else\s*\{\s*$", re.IGNORECASE)
_CLOSE_BRACE_RE = re.compile(r"^\s*\}\s*$")
_LIMIT_RE = re.compile(r"(?P<variable>[A-Za-z_][A-Za-z0-9_./-]*)\s*(?P<operator>[<>])\s*(?P<value>[^\s,]+)")
_FLY_ASSIGN_RE = re.compile(r"^\s*(?P<variable>[A-Za-z_][A-Za-z0-9_./-]*)\s*=\s*(?P<value>.+?)\s*$", re.IGNORECASE)
_FLY_VRS_RE = re.compile(
    r"^\s*(?P<variable>[A-Za-z_][A-Za-z0-9_./-]*)\s+vrs\s+(?P<reference>[A-Za-z_][A-Za-z0-9_./-]*)(?:\s+(?P<interpolation>interp-[0-9]+))?\s*$",
    re.IGNORECASE,
)
_OPTIMIZE_ENDPOINT_RE = re.compile(
    r"^(?P<text>[A-Za-z_][A-Za-z0-9_.-]*(?:\[\d+\])?|[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[EeDd][+-]?[0-9]+)?)"
    r"(?:\s+on\s+segment\s+(?P<segment>\d+)(?:\s*,?\s*trajectory\s+(?P<trajectory>\d+))?)?\s*$",
    re.IGNORECASE,
)
_FREE_FIELD_SPLIT_RE = re.compile(r"[\s,=():<>]+")
_ASSIGNMENT_BODY_BLOCKS = frozenset({"aero", "constants", "cg", "prop", "integ", "inertial", "reset", "increment"})
_OPTIMIZE_CONTROL_NAMES = {"fref", "derivs", "tol", "dx", "maxitr", "adjust", "integ", "surveys", "restarts", "print"}
_SURVEY_SETTING_RE = re.compile(r"(?P<name>lo|hi|inc|vals)\s*=\s*(?P<values>.*?)(?=(?:\s+[A-Za-z_][A-Za-z0-9_./-]*\s*=)|$)", re.IGNORECASE)
_SURVEY_NUMBER_RE = re.compile(r"^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[EeDd][+-]?\d+)?$")
_SEARCH_CONTROL_NAMES = {"xlo", "xhi", "xest", "dx", "tol", "xref", "fref", "maxitr", "integ", "print"}
_RADAR_PARAMETER_NAMES = {"alt", "long", "latgd", "diste", "distn", "distd", "reqtr", "rpolr", "flat", "ecc"}
_RAIL_PARAMETER_NAMES = {"cfstat", "cfslid", "azm", "elev"}
_INTEGRATION_PARAMETER_NAMES = {"dt", "dtprnt", "dtguid"}
_RESET_INCREMENT_VARIABLE_NAMES = {
    "alt", "long", "latgd", "rcm", "latgc", "xecfc", "yecfc", "zecfc", "xecic", "yecic", "zecic",
    "dxb", "dyb", "dzb", "vel", "gamgc", "psigc", "gamgd", "psigd", "xecfcdt", "yecfcdt", "zecfcdt",
    "xecicdt", "yecicdt", "zecicdt", "wt", "mass", "fuel", "time", "tseg", "tmark", "range", "grseg",
    "grmark", "plength", "plseg", "plmark", "iip_beta", "velibx",
    "q_rail", "azm", "elev",
}
_RESET_INCREMENT_COORDINATE_FAMILIES = {
    "geodetic": {"alt", "latgd", "gamgd", "psigd"},
    "geocentric": {"rcm", "latgc", "gamgc", "psigc"},
    "ecfc": {"xecfc", "yecfc", "zecfc", "xecfcdt", "yecfcdt", "zecfcdt"},
    "ecic": {"xecic", "yecic", "zecic", "xecicdt", "yecicdt", "zecicdt"},
}
_AERO_COEFFICIENT_SETS = (
    {"ca", "cn"},
    {"cl", "cd", "cs"},
    {"cx", "cy", "cz"},
)
_FORMAT_RE = re.compile(r"^[ef]\.\d+$", re.IGNORECASE)
_RUNTIME_ATTRIBUTE_RE = re.compile(r"(?P<name>[A-Za-z_][A-Za-z0-9_.-]*)=(?P<value>\"[^\"]*\"|'[^']*'|[^\s]+)")
_SENSOR_KINDS = {"imu", "accelerometer", "gyroscope", "magnetometer", "gps", "camera", "radar", "custom"}
_SENSOR_ATTRIBUTES = {"kind", "cadence-s", "phase-s", "sample", "delivery-s", "truth", "rate-policy", "frame", "mount", "source", "enabled", "units"}
_SENSOR_SAMPLE_MODES = {"instantaneous", "interval"}
_SENSOR_TRUTH_POLICIES = {"boundary", "accepted-segment"}
_SENSOR_RATE_POLICIES = {"split", "accumulate"}
_ATMOS_STANDARD_MODELS = {"none", "standard", "rcc", *{str(number) for number in range(21)}}
_EARTH_MODELS = {"spherical", "wgs-72", "wgs-84", "tsap-72", "tsap-84", "wgs-84-full", "gem-t1-full"}
_EARTH_PARAMETER_NAMES = {"reqtr", "rpolr", "ecc", "flat", "omega", "g", "gm", "j2", "j3", "j4", "c20", "c22", "c30", "c31", "c32", "c33", "c40", "c41", "c42", "c43", "c44", "s22", "s31", "s32", "s33", "s41", "s42", "s43", "s44"}
_OUTPUT_VARIABLE_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_.-]*(?:\[\d+\])*$")
_WIND_SPEED_NAMES = {"windv", "windh", "windd"}
_WIND_COMPONENT_NAMES = {"winde", "windn", "windd"}
_SUMMARY_OPERATIONS_WITH_OPERAND = {"add", "sub", "mult", "div", "idiv", "exp", "iexp"}
_SUMMARY_OPERATIONS_WITHOUT_OPERAND = {"abs", "neg", "sqr", "sqrt", "ln", "log", "e", "sin", "cos", "tan", "asin", "acos", "atan"}
_FORBIDDEN_LIMIT_VARIABLES = {"intercept", "prop", "propnav", "downria", "upria", "l/d-max", "l/d_max"}
_SUMMARY_FUNCTION_RE = re.compile(r"^(?P<function>max|min|first|last|maxfit|minfit)\((?P<value>[^()]+)\)(?:\s+trajectory\s+(?P<trajectory>\d+))?$", re.IGNORECASE)
_SUMMARY_SEGMENT_RE = re.compile(r"^(?P<value>[^,\s]+)\s*,?\s+on\s+segment\s+(?P<segment>\d+)(?:\s*,?\s*trajectory\s+(?P<trajectory>\d+))?$", re.IGNORECASE)
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
_FLY_BODY_ATTITUDE_ANGLE_SETS = (
    {"alpha", "betae", "bankgc"},
    {"alpha", "beta", "bankgc"},
    {"alphat", "phi", "bankgc"},
    {"alpha", "betae", "bankgd"},
    {"alpha", "beta", "bankgd"},
    {"alphat", "phi", "bankgd"},
    {"yawgc", "pitchgc", "rollgc"},
    {"yawgd", "pitchgd", "rollgd"},
    {"yawi", "pitchi", "rolli"},
)
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
_UNIT_DIMENSIONS = {
    **{unit: "length" for unit in {"ft", "in", "mi", "nm", "m", "km"}},
    **{unit: "time" for unit in {"sec", "min", "hr"}},
    **{unit: "angle" for unit in {"deg", "rad"}},
    **{unit: "speed" for unit in {"ft/sec", "ft/min", "ft/hr", "in/sec", "in/min", "in/hr", "mi/sec", "mi/min", "mi/hr", "knots", "m/sec", "m/min", "m/hr", "km/sec", "km/min", "km/hr"}},
    **{unit: "angular_rate" for unit in {"deg/sec", "deg/min", "deg/hr", "rad/sec", "rad/min", "rad/hr", "rev/sec", "rpm"}},
    **{unit: "acceleration" for unit in {"ft/sec2", "ft/min2", "ft/hr2", "in/sec2", "in/min2", "in/hr2", "mi/sec2", "mi/min2", "mi/hr2", "nm/hr2", "m/sec2", "m/min2", "m/hr2", "km/sec2", "km/min2", "km/hr2", "deg/sec2", "deg/min2", "deg/hr2", "rad/sec2", "rad/min2", "rad/hr2", "rev/sec2", "rev/min2", "rev/hr2", "g"}},
    **{unit: "mass" for unit in {"lb", "slugs", "gm", "kg"}},
    **{unit: "mass_rate" for unit in {"lb/sec", "lb/min", "lb/hr", "slugs/sec", "slugs/min", "slugs/hr", "g/sec", "g/min", "g/hr", "kg/sec", "kg/min", "kg/hr"}},
    **{unit: "force" for unit in {"lbf", "n", "kn"}},
    **{unit: "pressure" for unit in {"lbf/ft2", "psi", "pascal", "kpascal"}},
    **{unit: "inverse_length" for unit in {"1/in", "1/ft", "1/m"}},
    **{unit: "kinematic_viscosity" for unit in {"ft2/sec", "m2/sec"}},
    **{unit: "density" for unit in {"lb/ft3", "lb/m3", "g/cm3", "kg/m3"}},
}
_VARIABLE_DIMENSIONS = {
    **{name: "length" for name in {"alt", "range", "rcm", "grmark", "grseg", "plength", "plmark", "plseg"}},
    **{name: "speed" for name in {"altdt", "latgcdt", "latgddt", "rcmdt", "vel", "vair", "vgr", "xdt", "ydt", "zdt"}},
    **{name: "time" for name in {"time", "tmark", "tseg"}},
    **{name: "angle" for name in {"alpha", "alphat", "bankgc", "bankgd", "beta", "betae", "gamgc", "gamgd", "latgc", "latgd", "long", "phi", "pitchgc", "pitchgd", "pitchi", "psigc", "psigd", "rollgc", "rollgd", "rolli", "yawgc", "yawgd", "yawi", "ep1", "ep2"}},
    **{name: "angular_rate" for name in {"longdt"}},
    **{name: "acceleration" for name in {"nx", "ny", "nz"}},
    **{name: "mass" for name in {"mass", "wt"}},
    "mdot": "mass_rate",
    **{name: "force" for name in {"thrust"}},
    **{name: "pressure" for name in {"pres", "dynprs"}},
    **{name: "density" for name in {"rho"}},
    **{name: "temperature" for name in {"temp"}},
    **{name: "kinematic_viscosity" for name in {"nu"}},
    **{name: "dimensionless" for name in {"mach", "cg", "power", "segment"}},
    "sref": "area",
}
_PROP_THRUST_UNITS = {"lb", "n", "kn"}
_PROP_MASS_FLOW_UNITS = {"lb/sec", "lb/min", "lb/hr", "slugs/sec", "slugs/min", "slugs/hr", "g/sec", "g/min", "g/hr", "kg/sec", "kg/min", "kg/hr"}
_DOCUMENTED_DEFINE_FUNCTIONS = {
    "abs", "acos", "asin", "atan", "atan2", "ceil", "cos", "cosh", "exp", "floor",
    "log", "log10", "max", "sin", "sinh", "sqrt", "surface_azm", "surface_dist", "table", "tan", "tanh",
}
_DOCUMENTED_DEFINE_FUNCTION_ARITY = {name: 1 for name in _DOCUMENTED_DEFINE_FUNCTIONS} | {"atan2": 2, "surface_azm": 2, "surface_dist": 2}
_CONTINUATION_PARAMETER_NAMES = {
    "dwn/crs": {"latgd", "long", "azm"},
    "earth": _EARTH_PARAMETER_NAMES,
    "iip": {"iip_beta", "iip_alt"},
    "integ": _INTEGRATION_PARAMETER_NAMES,
    "radar": _RADAR_PARAMETER_NAMES,
    "rail": _RAIL_PARAMETER_NAMES,
    "reset": _RESET_INCREMENT_VARIABLE_NAMES,
    "increment": _RESET_INCREMENT_VARIABLE_NAMES,
    "tangent": {"latgd", "long", "alt", "azm"},
}
_NUMERIC_PARAMETER_BLOCKS = {
    "dwn/crs",
    "earth",
    "iip",
    "integ",
    "inertial",
    "radar",
    "rail",
    "tangent",
}
_UNIQUE_PARAMETER_BLOCKS = _NUMERIC_PARAMETER_BLOCKS


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


def _validate_expression_functions(expression: Any, path: str, line: int, diagnostics: list[Diagnostic]) -> None:
    if isinstance(expression, CallExpression):
        function = expression.function.casefold()
        if function not in _DOCUMENTED_DEFINE_FUNCTIONS:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="unsupported-define-function",
                    message=f"Function {expression.function!r} is not documented for *define expressions; source text was preserved.",
                    location=_location(path, line),
                )
            )
        else:
            expected_arity = _DOCUMENTED_DEFINE_FUNCTION_ARITY[function]
            valid_arity = len(expression.arguments) >= 1 if function == "table" else len(expression.arguments) == expected_arity
            if not valid_arity:
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="invalid-define-function-arity",
                        message=(
                            f"Function {expression.function!r} requires at least one argument(s); source text was preserved."
                            if function == "table"
                            else f"Function {expression.function!r} requires {expected_arity} argument(s); source text was preserved."
                        ),
                        location=_location(path, line),
                    )
                )
            elif function == "table" and not isinstance(expression.arguments[0], NameExpression):
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="invalid-define-table-argument",
                        message="The documented table(id, query...) form requires a table-identification name first; source text was preserved.",
                        location=_location(path, line),
                    )
                )
        ####
        for argument in expression.arguments:
            _validate_expression_functions(argument, path, line, diagnostics)
        ####
        return
    ####
    if isinstance(expression, BinaryExpression):
        _validate_expression_functions(expression.left, path, line, diagnostics)
        _validate_expression_functions(expression.right, path, line, diagnostics)
    elif hasattr(expression, "operand"):
        _validate_expression_functions(expression.operand, path, line, diagnostics)
    ####
####


def _contains_call_expression(expression: Any) -> bool:
    if isinstance(expression, CallExpression):
        return True
    if isinstance(expression, BinaryExpression):
        return _contains_call_expression(expression.left) or _contains_call_expression(expression.right)
    if hasattr(expression, "operand"):
        return _contains_call_expression(expression.operand)
    return False
####


def _contains_nested_relationship(expression: Any) -> bool:
    """Return whether an expression contains a relationship below its root."""

    if isinstance(expression, BinaryExpression):
        if expression.operator in {"=", "==", "!=", "<", ">", "<=", ">="}:
            return True
        return _contains_nested_relationship(expression.left) or _contains_nested_relationship(expression.right)
    if isinstance(expression, UnaryExpression):
        return _contains_nested_relationship(expression.operand)
    if isinstance(expression, CallExpression):
        return any(_contains_nested_relationship(argument) for argument in expression.arguments)
    ####
    return False
####


def _is_documented_when_relationship(expression: Any) -> bool:
    """Check the manual's single relationship form used by ``*when``."""

    return isinstance(expression, BinaryExpression) and expression.operator in {"=", "==", "!=", "<", "<=", ">", ">="} and not _contains_nested_relationship(expression.left) and not _contains_nested_relationship(expression.right)
####


def _validate_problem_define_expression(expression: Any, path: str, line: int, diagnostics: list[Diagnostic]) -> None:
    """Reject trajectory-dependent table calls in problem-scope definitions."""
    if isinstance(expression, CallExpression):
        if expression.function.casefold() == "table":
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="table-call-outside-trajectory",
                    message="The documented table() function cannot be evaluated in a problem-scope *define block; source text was preserved.",
                    location=_location(path, line),
                )
            )
        for argument in expression.arguments:
            _validate_problem_define_expression(argument, path, line, diagnostics)
        ####
        return
    ####
    if isinstance(expression, BinaryExpression):
        _validate_problem_define_expression(expression.left, path, line, diagnostics)
        _validate_problem_define_expression(expression.right, path, line, diagnostics)
    elif hasattr(expression, "operand"):
        _validate_problem_define_expression(expression.operand, path, line, diagnostics)
    ####


def _validate_problem_define_control(control: DefineControlStatement, diagnostics: list[Diagnostic]) -> None:
    if control.condition is not None:
        _validate_problem_define_expression(control.condition, control.location.path, control.location.line, diagnostics)
    if control.assignment is not None:
        _validate_problem_define_expression(control.assignment.value, control.assignment.location.path, control.assignment.location.line, diagnostics)
    for assignment in control.body + control.else_body:
        _validate_problem_define_expression(assignment.value, assignment.location.path, assignment.location.line, diagnostics)
    for nested_control in control.body_controls + control.else_controls:
        _validate_problem_define_control(nested_control, diagnostics)
    if control.nested is not None:
        _validate_problem_define_control(control.nested, diagnostics)
    ####


def _validate_problem_define_table_calls(block: DefineBlock, diagnostics: list[Diagnostic]) -> None:
    if block.scope != "problem":
        return
    ####
    if block.initial_value is not None:
        _validate_problem_define_expression(block.initial_value, block.location.path, block.location.line, diagnostics)
    for assignment in block.assignments:
        _validate_problem_define_expression(assignment.value, assignment.location.path, assignment.location.line, diagnostics)
    for control in block.control_statements:
        _validate_problem_define_control(control, diagnostics)
    ####


def _expression_names(expression: Any) -> set[str]:
    if isinstance(expression, NameExpression):
        return {expression.name.casefold()}
    if isinstance(expression, IndexedExpression):
        return {expression.name.casefold()}
    if isinstance(expression, CallExpression):
        arguments = expression.arguments[1:] if expression.function.casefold() == "table" else expression.arguments
        return {name for argument in arguments for name in _expression_names(argument)}
    if isinstance(expression, BinaryExpression):
        return _expression_names(expression.left) | _expression_names(expression.right)
    if hasattr(expression, "operand"):
        return _expression_names(expression.operand)
    return set()
####


def _validate_define_assignment_order(block: DefineBlock, diagnostics: list[Diagnostic]) -> None:
    """Diagnose definite forward references to same-definition temporaries."""
    assignments = block.assignments
    for index, assignment in enumerate(assignments):
        assigned_later = {item.name.casefold() for item in assignments[index + 1 :]}
        for name in sorted(_expression_names(assignment.value) & assigned_later):
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="define-variable-used-before-assignment",
                    message=f"Temporary variable {name!r} is used before its assignment in this *define block; source text was preserved.",
                    location=assignment.location,
                )
            )
    ####


def _assignments(
    text: str,
    path: str,
    line: int,
    diagnostics: list[Diagnostic],
    *,
    allow_functions: bool = False,
) -> list[Assignment]:
    if ";" in text:
        segments = [segment.strip() for segment in text.split(";") if segment.strip()]
        if len(segments) > 1:
            result: list[Assignment] = []
            for segment in segments:
                result.extend(_assignments(segment, path, line, diagnostics, allow_functions=allow_functions))
            return result
        ####
        statement = _STATEMENT_ASSIGN_RE.fullmatch(text)
        if statement is not None:
            try:
                value = _parse_value(statement.group("value"))
            except ExpressionSyntaxError as exc:
                diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-expression", message=str(exc), location=_location(path, line)))
                return []
            if not allow_functions and _contains_call_expression(value):
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="unsupported-assignment-call",
                        message="Function calls are documented only in *define expressions; source text was preserved.",
                        location=_location(path, line),
                    )
                )
            ####
            return [Assignment(name=statement.group("name"), operator=statement.group("op"), value=value, location=_location(path, line))]
        ####
    elif allow_functions:
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
        if not allow_functions and _contains_call_expression(value):
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="unsupported-assignment-call",
                    message="Function calls are documented only in *define expressions; source text was preserved.",
                    location=_location(path, line, match.start("value") + 1),
                )
            )
        ####
        result.append(Assignment(name=match.group("name"), operator=match.group("op"), value=value, location=_location(path, line, match.start("name") + 1)))
    ####
    return result
####


def _assignment_residual(text: str) -> tuple[int, str] | None:
    covered = [False] * len(text)
    for match in _ASSIGN_RE.finditer(text):
        for index in range(*match.span()):
            covered[index] = True
    ####
    for index, character in enumerate(text):
        if not covered[index] and not character.isspace() and character != ",":
            residual = text[index:].strip()
            return index + 1, residual
    ####
    return None
####


def _diagnose_assignment_residual(
    text: str,
    keyword: str,
    path: str,
    line: int,
    diagnostics: list[Diagnostic],
) -> bool:
    residual = _assignment_residual(text)
    if residual is None:
        return False
    ####
    column, residual_text = residual
    diagnostics.append(
        Diagnostic(
            severity=Severity.ERROR,
            code="invalid-assignment-line",
            message=f"Unparsed assignment text {residual_text!r} remains in '*{keyword}'.",
            location=_location(path, line, column),
        )
    )
    return True
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
            _validate_expression_functions(condition, path, line, diagnostics)
            body_text = match.group("assignment").strip()
            nested = _define_control(body_text, path, line, diagnostics) if body_text.lower().startswith(("if ", "if(", "else ")) else None
            assignments = [] if nested else _assignments(body_text, path, line, diagnostics, allow_functions=True)
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
        for assignment in assignments:
            _validate_expression_functions(assignment.value, path, line, diagnostics)
        ####
        return DefineControlStatement(kind="if", condition=condition, assignment=assignments[0] if assignments else None, nested=nested, location=_location(path, line))
    if match := _ELSE_ASSIGN_RE.fullmatch(text):
        assignments = _assignments(match.group("assignment"), path, line, diagnostics, allow_functions=True)
        for assignment in assignments:
            _validate_expression_functions(assignment.value, path, line, diagnostics)
        ####
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
        _validate_expression_functions(value, path, line, diagnostics)
        return DefineAssignmentStatement(
            assignment=Assignment(name=statement.group("name"), operator=statement.group("op"), value=value, location=_location(path, line))
        )
    if text.strip().lower().startswith(("if ", "if(", "else ")):
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-define-control", message=f"Unsupported *define control statement: {text.strip()!r}.", location=_location(path, line)))
        return None
    assignments = _assignments(text, path, line, diagnostics, allow_functions=True)
    if len(assignments) == 1:
        _validate_expression_functions(assignments[0].value, path, line, diagnostics)
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


def _validate_segment_termination(trajectory: Trajectory, diagnostics: list[Diagnostic]) -> None:
    """Require each documented segment to contain a final-condition block."""
    for segment in trajectory.segments:
        if not any(isinstance(block, WhenBlock) for block in segment.blocks):
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="missing-segment-when",
                    message=f"Segment {segment.number} must contain at least one '*when' final-condition block; source text was preserved.",
                    location=segment.location,
                )
            )
    ####


def _validate_trajectory_structure(trajectory: Trajectory, diagnostics: list[Diagnostic]) -> None:
    """Require the documented initial-condition and segment children."""
    if not any(isinstance(block, InitialBlock) for block in trajectory.blocks) and not any(block.keyword == "deployed" for block in trajectory.blocks):
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="missing-trajectory-initial",
                message=f"Trajectory {trajectory.number} must contain an '*initial' block; source text was preserved.",
                location=trajectory.location,
            )
        )
    if not trajectory.segments:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="missing-trajectory-segment",
                message=f"Trajectory {trajectory.number} must contain at least one '*segment' block; source text was preserved.",
                location=trajectory.location,
            )
        )
    ####
####


def _validate_continuation_parameters(
    keyword: str,
    assignments: list[Assignment],
    path: str,
    line: int,
    diagnostics: list[Diagnostic],
    *,
    allowed_names: set[str] | None = None,
) -> None:
    """Apply a fixed block vocabulary to assignments on continuation lines."""

    allowed = allowed_names if allowed_names is not None else _CONTINUATION_PARAMETER_NAMES.get(keyword)
    if allowed is None:
        return
    ####
    for assignment in assignments:
        if assignment.name.casefold() not in allowed:
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
    *,
    scope: str | None = None,
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
        else:
            base_name = token.split("[", 1)[0].casefold()
            required_subscripts = 2 if scope == "problem" else 1
            if base_name.startswith(("rad", "rel")) and token.count("[") != required_subscripts:
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="invalid-related-output-subscript",
                        message=f"Output variable {token!r} requires exactly {required_subscripts} subscript(s) in a {scope or 'trajectory'}-scope '*{keyword}' block; source text was preserved.",
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


def _wind_form(assignments: list[Assignment]) -> str | None:
    names = {assignment.name.casefold() for assignment in assignments}
    if len(assignments) == 3 and names == _WIND_SPEED_NAMES:
        return "speed-heading"
    ####
    if len(assignments) == 3 and names == _WIND_COMPONENT_NAMES:
        return "east-north"
    ####
    return None
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
        if _contains_call_expression(value):
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="unsupported-limit-call",
                    message="Limit values may not contain function calls; source text was preserved.",
                    location=_location(path, line, match.start("value") + 1),
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


def _validate_limits(block: LimitsBlock, diagnostics: list[Diagnostic]) -> None:
    """Reject guidance rules that the manual explicitly excludes from limits."""
    for limit in block.limits:
        if limit.variable.casefold() in _FORBIDDEN_LIMIT_VARIABLES:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="unlimit-able-guidance-variable",
                    message=f"Guidance variable {limit.variable!r} cannot be constrained by '*limits'.",
                    location=limit.location,
                )
            )
        ####
    ####
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
        if not isinstance(expression, (NameExpression, IndexedExpression)):
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="invalid-summary-function-operand",
                    message="Summary special functions require a variable or indexed variable operand; source text was preserved.",
                    location=_location(path, line),
                )
            )
        if isinstance(expression, IndexedExpression) and function_match.group("trajectory") is not None:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="conflicting-summary-trajectory",
                    message="A summary special-function operand cannot use both a bracketed trajectory subscript and a 'trajectory' qualifier.",
                    location=_location(path, line),
                )
            )
        if _contains_call_expression(expression):
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="unsupported-summary-operand-call", message="Summary special functions may only contain a variable or indexed variable; source text was preserved.", location=_location(path, line)))
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
        if _contains_call_expression(expression):
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="unsupported-summary-operand-call", message="Summary operands may not contain function calls; source text was preserved.", location=_location(path, line)))
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
    if _contains_call_expression(expression):
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="unsupported-summary-operand-call",
                message="Summary operands may be variables, constants, or documented special functions; source text was preserved.",
                location=_location(path, line),
            )
        )
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


def _validate_initial_assignments(block: InitialBlock, path: str, line: int, diagnostics: list[Diagnostic], *, profile: GrammarProfile = GrammarProfile.TAOS96) -> None:
    coordinate = block.coordinate_system or "geodetic"
    allowed = set(_INITIAL_VARIABLES[coordinate])
    if profile is GrammarProfile.TAORYX and coordinate == "ecic":
        allowed.update({"qw", "qx", "qy", "qz", "wx", "wy", "wz", "propellant_mass", "heat_load", "peak_heat_rate"})
    seen: set[str] = set()
    for assignment in block.assignments:
        name = assignment.name.casefold()
        if name not in allowed:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="unsupported-initial-parameter", message=f"Parameter {assignment.name!r} is not documented for '*initial {coordinate}'.", location=assignment.location))
        elif name in seen:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="duplicate-initial-parameter",
                    message=f"Parameter {assignment.name!r} is assigned more than once in '*initial'; source text was preserved.",
                    location=assignment.location,
                )
            )
        seen.add(name)
        ####
    names = {assignment.name.casefold() for assignment in block.assignments}
    if {"wt", "mass"} <= names:
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="conflicting-initial-mass", message="'*initial' cannot specify both wt and mass.", location=_location(path, line)))
    ####
    if {"vel", "mach"} <= names:
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="conflicting-initial-velocity", message="'*initial' cannot specify both vel and mach.", location=_location(path, line)))
    ####


def _validate_initial_copy_assignments(assignments: list[Assignment], path: str, line: int, diagnostics: list[Diagnostic]) -> None:
    """Allow only the manual-documented ECFC/ECIC settings after a copied state."""
    for assignment in assignments:
        if assignment.name.casefold() not in {"t_0", "omega_0"}:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="unsupported-initial-copy-parameter",
                    message=f"Parameter {assignment.name!r} is not documented after a copied '*initial' state; source text was preserved.",
                    location=assignment.location,
                )
            )
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
    if trajectory_subscript is not None and match.group("trajectory") is not None:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="conflicting-endpoint-trajectory",
                message="An endpoint cannot use both a bracketed trajectory subscript and a 'trajectory' qualifier.",
                location=_location(path, line),
            )
        )
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
    if body.casefold() in {"min", "max"}:
        return SearchObjective(left=OptimizeEndpoint(text=body, expression=NameExpression(name=body)))
    ####
    relation = re.search(r"(?P<operator>[<>=])", body)
    if relation is None:
        left = _parse_optimize_endpoint(body, path, line, diagnostics)
        if left is None:
            return None
        if isinstance(left.expression, NumberExpression):
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="invalid-search-endpoint",
                    message="A search objective's first term must be an output variable, not a numeric value.",
                    location=_location(path, line),
                )
            )
        ####
        if report_incomplete and (left.segment is None or (left.trajectory is None and left.trajectory_subscript is None)):
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="invalid-search-endpoint",
                    message="A search objective's first endpoint requires a segment and trajectory qualifier (or trajectory subscript).",
                    location=_location(path, line),
                )
            )
        ####
        return SearchObjective(left=left)
    ####
    left = _parse_optimize_endpoint(body[: relation.start()], path, line, diagnostics)
    right = _parse_optimize_endpoint(body[relation.end() :], path, line, diagnostics)
    if left is None or right is None:
        return None
    if isinstance(left.expression, NumberExpression):
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="invalid-search-endpoint",
                message="A search objective's first term must be an output variable, not a numeric value.",
                location=_location(path, line),
            )
        )
    ####
    if left.segment is None and right.segment is not None:
        left.segment = right.segment
    elif right.segment is None and left.segment is not None:
        right.segment = left.segment
    ####
    left_trajectory = left.trajectory if left.trajectory is not None else left.trajectory_subscript
    right_trajectory = right.trajectory if right.trajectory is not None else right.trajectory_subscript
    if left_trajectory is None and right_trajectory is not None:
        left.trajectory = right_trajectory
    elif right_trajectory is None and left_trajectory is not None:
        right.trajectory = left_trajectory
    elif left_trajectory is None and right_trajectory is None:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="invalid-search-endpoint",
                message="A search objective relationship requires a trajectory qualifier or trajectory subscript.",
                location=_location(path, line),
            )
        )
    ####
    if left.segment is None or (left.trajectory is None and left.trajectory_subscript is None):
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="invalid-search-endpoint",
                message="A search objective's first endpoint requires a segment and trajectory qualifier (or trajectory subscript).",
                location=_location(path, line),
            )
        )
    ####
    return SearchObjective(left=left, operator=relation.group("operator"), right=right)
####


def _validate_search_controls(assignments: list[Assignment], path: str, line: int, diagnostics: list[Diagnostic]) -> None:
    seen: set[str] = set()
    for assignment in assignments:
        name = assignment.name.casefold()
        if name in seen:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="duplicate-search-control",
                    message=f"Search control {assignment.name!r} is assigned more than once; source text was preserved.",
                    location=assignment.location,
                )
            )
        seen.add(name)
        if name not in _SEARCH_CONTROL_NAMES:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="unsupported-search-control", message=f"Control {assignment.name!r} is not documented in a '*search' body.", location=assignment.location))
        elif not _is_numeric_parameter_value(assignment.value):
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="nonnumeric-search-control",
                    message=f"Search control {assignment.name!r} requires a numeric value; source text was preserved.",
                    location=assignment.location,
                )
            )
    ####
####


def _validate_optimize_controls(assignments: list[Assignment], path: str, line: int, diagnostics: list[Diagnostic], *, allow_symbolic: bool = False) -> None:
    seen: set[str] = set()
    for assignment in assignments:
        name = assignment.name.casefold()
        if name in seen:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="duplicate-optimize-control",
                    message=f"Optimization control {assignment.name!r} is assigned more than once; source text was preserved.",
                    location=assignment.location,
                )
            )
        seen.add(name)
        if name in _OPTIMIZE_CONTROL_NAMES or re.fullmatch(r"(?:par|lo|hi|ref)-\d+", name):
            if not _is_numeric_parameter_value(assignment.value) and not (allow_symbolic and isinstance(assignment.value, NameExpression)):
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="nonnumeric-optimize-control",
                        message=f"Optimization control {assignment.name!r} requires a numeric value; source text was preserved.",
                        location=assignment.location,
                    )
                )
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


def _validate_reset_increment(block: Any, problem: Problem, diagnostics: list[Diagnostic]) -> None:
    """Validate the manual's state-coordinate restrictions for reset/increment."""
    names = {assignment.name.casefold() for assignment in block.assignments}
    if {"wt", "mass"} <= names:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="conflicting-reset-mass",
                message="A '*reset' or '*increment' block cannot specify both wt and mass.",
                location=block.location,
            )
        )
    ####
    families = {
        family
        for family, variables in _RESET_INCREMENT_COORDINATE_FAMILIES.items()
        if names & variables
    }
    if len(families) > 1:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="mixed-reset-coordinate-systems",
                message="Position and velocity variables in one '*reset' or '*increment' block must use one coordinate system.",
                location=block.location,
            )
        )
    ####
    if "time" in names and len(problem.trajectories) > 1:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="absolute-time-discontinuity-multiple-trajectories",
                message="A multiple-trajectory problem cannot reset or increment absolute time.",
                location=block.location,
            )
        )
    ####
####


def _validate_fly_angle_set(segment: Any, diagnostics: list[Diagnostic]) -> None:
    angle_blocks = [
        block
        for block in segment.blocks
        if isinstance(block, FlyBlock) and block.guidance_variable is not None and block.guidance_variable.casefold() in {name for angle_set in _FLY_BODY_ATTITUDE_ANGLE_SETS for name in angle_set}
    ]
    specified = {block.guidance_variable.casefold() for block in angle_blocks if block.guidance_variable is not None}
    if not specified or any(specified <= angle_set for angle_set in _FLY_BODY_ATTITUDE_ANGLE_SETS):
        return
    ####
    location = angle_blocks[-1].location if angle_blocks else segment.location
    diagnostics.append(
        Diagnostic(
            severity=Severity.ERROR,
            code="inconsistent-fly-angle-set",
            message="Body-attitude *fly angles do not form one of the nine consistent Table 4-3 angle sets.",
            location=location,
        )
    )
####


def _validate_fly_blocks(trajectory: Trajectory, diagnostics: list[Diagnostic]) -> None:
    """Validate per-trajectory structural rules for guidance blocks."""
    for index, segment in enumerate(trajectory.segments):
        fly_blocks = [block for block in segment.blocks if isinstance(block, FlyBlock)]
        if len(fly_blocks) > 4:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="too-many-fly-blocks",
                    message="A segment may contain no more than four '*fly' blocks.",
                    location=fly_blocks[4].location,
                )
            )
        ####
        special_blocks = [
            block
            for block in fly_blocks
            if block.guidance_variable is not None
            and block.guidance_variable.casefold() in {"intercept", "propnav"}
        ]
        if special_blocks and len(fly_blocks) > 3:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="too-many-special-fly-blocks",
                    message="A segment using '*fly intercept' or '*fly propnav' may contain no more than three '*fly' blocks.",
                    location=fly_blocks[3].location,
                )
            )
        ####
        if index == 0:
            for block in fly_blocks:
                if isinstance(block.value, WildcardExpression):
                    diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="wildcard-fly-in-first-segment",
                            message="A '*fly' wildcard value requires a preceding trajectory segment.",
                            location=block.location,
                        )
                    )
                ####
            ####
        ####
    ####


def _validate_inertial_blocks(trajectory: Trajectory, diagnostics: list[Diagnostic]) -> None:
    """Reject alignments that require attitude before the first segment runs."""
    if not trajectory.segments:
        return
    ####
    first_segment = trajectory.segments[0]
    for block in first_segment.blocks:
        if not isinstance(block, InertialBlock) or block.alignment not in {"body", "wind", "velocity"}:
            continue
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="inertial-body-first-segment",
                message=f"Inertial platform alignment {block.alignment!r} requires a preceding segment because attitude is unavailable at the start of the first segment.",
                location=block.location,
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


def _validate_survey_configuration(block: SurveyBlock, diagnostics: list[Diagnostic]) -> None:
    """Validate the documented incremental/explicit survey setting forms."""
    seen: set[str] = set()
    for setting in block.settings:
        if setting.name in seen:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="duplicate-survey-setting",
                    message=f"Survey setting {setting.name!r} is assigned more than once; source text was preserved.",
                    location=setting.location,
                )
            )
        seen.add(setting.name)
    ####
    settings = {setting.name: setting for setting in block.settings}
    incremental_names = {"lo", "hi", "inc"}
    present_incremental = incremental_names & settings.keys()
    has_explicit_values = "vals" in settings

    if not block.settings:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="missing-survey-values",
                message="A '*survey' block requires lo/hi/inc settings, vals settings, or both.",
                location=block.location,
            )
        )
        return
    ####
    if present_incremental and present_incremental != incremental_names:
        missing = ", ".join(sorted(incremental_names - present_incremental))
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="incomplete-survey-increment",
                message=f"An incremental survey requires lo, hi, and inc; missing {missing}.",
                location=block.location,
            )
        )
    ####
    if not present_incremental and not has_explicit_values:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="missing-survey-values",
                message="A '*survey' block requires lo/hi/inc settings or at least one vals setting.",
                location=block.location,
            )
        )
    ####
####


def _validate_survey_output_names(problem: Problem, diagnostics: list[Diagnostic]) -> None:
    """Reject survey names that collide with documented output-variable names."""
    blocks = list(problem.blocks)
    for trajectory in problem.trajectories:
        blocks.extend(trajectory.blocks)
        for segment in trajectory.segments:
            blocks.extend(segment.blocks)
    output_names = {
        variable.casefold()
        for block in blocks
        if isinstance(block, (FileBlock, EgsBlock, PrintBlock))
        for variable in block.variables
    }
    for block in blocks:
        if isinstance(block, SurveyBlock) and block.name is not None and block.name.casefold() in output_names:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="conflicting-survey-output-name",
                    message=f"Survey name {block.name!r} must not duplicate an output-variable name.",
                    location=block.location,
                )
            )
    ####


def _validate_units_format_user_variables(problem: Problem, diagnostics: list[Diagnostic]) -> None:
    """Reject unit changes for user variables declared by a ``*define`` block."""
    blocks = list(problem.blocks)
    for trajectory in problem.trajectories:
        blocks.extend(trajectory.blocks)
        for segment in trajectory.segments:
            blocks.extend(segment.blocks)
    ####
    defined_names = {
        block.variable.casefold()
        for block in blocks
        if isinstance(block, DefineBlock) and block.variable is not None
    }
    if not defined_names:
        return
    ####
    for block in blocks:
        if not isinstance(block, UnitsFormatBlock):
            continue
        for setting in block.settings:
            if setting.unit is not None and setting.variable.casefold() in defined_names:
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="units-on-user-defined-variable",
                        message=f"*units/fmt may set a format for user-defined variable {setting.variable!r}, but cannot change its units; source text was preserved.",
                        location=setting.location,
                    )
                )
        ####
    ####


def _validate_define_target_names(problem: Problem, diagnostics: list[Diagnostic]) -> None:
    """Keep declared user variables distinct from the manual's state vocabulary."""
    blocks = list(problem.blocks)
    for trajectory in problem.trajectories:
        blocks.extend(trajectory.blocks)
    ####
    for block in blocks:
        if not isinstance(block, DefineBlock) or block.variable is None:
            continue
        if block.variable.casefold() in DOCUMENTED_STATE_VARIABLES:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="reserved-define-variable",
                    message=f"*define target {block.variable!r} is reserved for a documented TAOS state variable; source text was preserved.",
                    location=block.location,
                )
            )
        ####
    ####
    scope_groups: list[list[DefineBlock]] = [
        [block for block in problem.blocks if isinstance(block, DefineBlock)]
    ]
    scope_groups.extend(
        [block for block in trajectory.blocks if isinstance(block, DefineBlock)]
        for trajectory in problem.trajectories
    )

    def collect_assignments(block: DefineBlock) -> list[Assignment]:
        assignments = list(block.assignments)

        def collect_control(control: DefineControlStatement) -> None:
            if control.assignment is not None:
                assignments.append(control.assignment)
            assignments.extend(control.body)
            assignments.extend(control.else_body)
            for nested in control.body_controls + control.else_controls:
                collect_control(nested)
            if control.nested is not None:
                collect_control(control.nested)
            ####

        for control in block.control_statements:
            collect_control(control)
        return assignments
    ####

    for define_blocks in scope_groups:
        target_blocks: dict[str, list[DefineBlock]] = {}
        for block in define_blocks:
            if block.variable is not None:
                target_blocks.setdefault(block.variable.casefold(), []).append(block)
            ####
        ####
        for target_blocks_for_name in target_blocks.values():
            for block in target_blocks_for_name[1:]:
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="duplicate-define-variable",
                        message=f"User-defined variable {block.variable!r} is declared by more than one *define block in the same scope; source text was preserved.",
                        location=block.location,
                    )
                )
            ####
        ####
        target_names = set(target_blocks)
        for block in define_blocks:
            temporary_assignments = [
                assignment
                for assignment in collect_assignments(block)
                if block.variable is None or assignment.name.casefold() != block.variable.casefold()
            ]
            seen_temporaries: set[str] = set()
            for assignment in temporary_assignments:
                name = assignment.name.casefold()
                if name in DOCUMENTED_STATE_VARIABLES:
                    diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="reserved-define-temporary",
                            message=f"Temporary variable {assignment.name!r} is reserved for a documented TAOS state variable; source text was preserved.",
                            location=assignment.location,
                        )
                    )
                ####
                if name in target_names and name != (block.variable or "").casefold():
                    diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="define-variable-collision",
                            message=f"Temporary variable {assignment.name!r} conflicts with another user-defined *define target in the same scope; source text was preserved.",
                            location=assignment.location,
                        )
                    )
                ####
                if name in seen_temporaries:
                    diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="duplicate-define-temporary",
                            message=f"Temporary variable {assignment.name!r} is assigned more than once in this *define block; source text was preserved.",
                            location=assignment.location,
                        )
                    )
                ####
                seen_temporaries.add(name)
            ####
        ####
    ####


def _validate_egs_summary_prerequisites(problem: Problem, diagnostics: list[Diagnostic]) -> None:
    """Require the survey and summary inputs documented for ``*egs summary``."""
    surveys = [block for block in problem.blocks if isinstance(block, SurveyBlock)]
    summaries = [block for block in problem.blocks if isinstance(block, SummarizeBlock) and block.operations]
    for block in problem.blocks:
        if not isinstance(block, EgsBlock) or not block.summary:
            continue
        if not surveys:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="missing-egs-summary-survey",
                    message="'*egs summary' requires at least one '*survey' loop; source text was preserved.",
                    location=block.location,
                )
            )
        if not summaries:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="missing-egs-summary-variable",
                    message="'*egs summary' requires at least one populated '*summarize' block; source text was preserved.",
                    location=block.location,
                )
            )
    ####
####


def _validate_define_target(block: DefineBlock, diagnostics: list[Diagnostic]) -> None:
    """Ensure a populated definition assigns its declared user-defined variable."""
    if block.variable is None:
        return
    ####
    names = {assignment.name.casefold() for assignment in block.assignments}

    def collect_control_names(control: DefineControlStatement) -> None:
        if control.assignment is not None:
            names.add(control.assignment.name.casefold())
        names.update(assignment.name.casefold() for assignment in control.body)
        names.update(assignment.name.casefold() for assignment in control.else_body)
        for nested_control in control.body_controls + control.else_controls:
            collect_control_names(nested_control)
        if control.nested is not None:
            collect_control_names(control.nested)
        ####

    for control in block.control_statements:
        collect_control_names(control)
    if names and block.variable.casefold() not in names:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="missing-define-target",
                message=f"*define {block.variable!r} must assign its declared variable on the left side of an equation.",
                location=block.location,
            )
        )
    ####
####


def _validate_define_control_structure(block: DefineBlock, diagnostics: list[Diagnostic]) -> None:
    """Reject a top-level ``else`` that has no preceding top-level ``if``."""
    previous_control: DefineControlStatement | None = None
    for statement in block.typed_statements:
        if not isinstance(statement, DefineControlStatement):
            previous_control = None
            continue
        ####
        if statement.kind == "else" and (previous_control is None or previous_control.kind != "if"):
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="orphan-define-else",
                    message="A top-level *define else statement must follow a top-level if statement; source text was preserved.",
                    location=statement.location,
                )
            )
        ####
        previous_control = statement
    ####
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
        if not _OUTPUT_VARIABLE_RE.fullmatch(variable):
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="invalid-units-format-variable",
                    message=f"Variable {variable!r} is not a valid output or user-defined variable name.",
                    location=_location(path, line),
                )
            )
        ####
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
        elif unit is not None:
            variable_dimension = _VARIABLE_DIMENSIONS.get(variable.casefold().split("[", 1)[0])
            unit_dimension = _UNIT_DIMENSIONS.get(unit.casefold())
            compatible = (
                variable_dimension is None
                or unit_dimension is None
                or variable_dimension == unit_dimension
                or variable_dimension == "area" and unit_dimension == "length"
            )
            if not compatible:
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="incompatible-unit-dimension",
                        message=f"Unit {unit!r} is dimensionally incompatible with variable {variable!r}; source text was preserved.",
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
        normalized = [token.casefold() for token in tokens]
        valid_user_nu = block.model == "user" and len(normalized) == 6 and set(normalized) == {"alt", "temp", "pres", "rho", "sndspd", "nu"}
        if len(tokens) != len(expected) or (set(normalized) != set(expected) and not valid_user_nu):
            expected_text = "alt, temp, pres, rho, sndspd, and either visc or nu" if block.model == "user" else ", ".join(expected)
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-atmos-columns", message=f"Expected atmosphere columns {expected_text}.", location=_location(path, line)))
            return
        ####
        block.columns.extend(normalized)
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
    block.row_locations.append(_location(path, line))
    ####


def _validate_atmos_rows(block: AtmosBlock, diagnostics: list[Diagnostic]) -> None:
    if not block.rows or "alt" not in block.columns:
        return
    ####
    altitude_index = block.columns.index("alt")
    altitudes = [row[altitude_index] for row in block.rows]
    for index, (previous, current) in enumerate(zip(altitudes, altitudes[1:]), start=1):
        if current == previous:
            code = "duplicate-atmos-altitude"
            message = "Atmosphere altitude samples must not contain duplicate values."
        elif current < previous:
            code = "unordered-atmos-altitude"
            message = "Atmosphere altitude samples must be strictly increasing."
        else:
            continue
        ####
        location = block.row_locations[index] if index < len(block.row_locations) else block.location
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code=code, message=message, location=location))
    ####
####


def _validate_earth_shape_parameters(block: EarthBlock, diagnostics: list[Diagnostic]) -> None:
    shape_assignments = [assignment for assignment in block.assignments if assignment.name.casefold() in {"rpolr", "ecc", "flat"}]
    if len(shape_assignments) <= 1:
        return
    ####
    diagnostics.append(
        Diagnostic(
            severity=Severity.ERROR,
            code="conflicting-earth-shape-parameters",
            message="An earth model may specify at most one of rpolr, ecc, or flat as the polar-shape parameter.",
            location=shape_assignments[1].location,
        )
    )
####


def _validate_radar_shape_parameters(block: RadarBlock, diagnostics: list[Diagnostic]) -> None:
    """Validate the alternative direct-earth-shape form documented for radar stations."""
    shape_assignments = [
        assignment
        for assignment in block.assignments
        if assignment.name.casefold() in {"reqtr", "rpolr", "ecc", "flat"}
    ]
    polar_assignments = [assignment for assignment in shape_assignments if assignment.name.casefold() != "reqtr"]
    if not shape_assignments:
        return
    ####
    if block.earth_shape is not None:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="conflicting-radar-earth-shape-definition",
                message="A '*radar' station must select WGS-72/WGS-84 or provide direct earth-shape values, not both.",
                location=shape_assignments[0].location,
            )
        )
        return
    ####
    if not any(assignment.name.casefold() == "reqtr" for assignment in shape_assignments):
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="missing-radar-equatorial-radius",
                message="Direct radar earth-shape input requires the equatorial radius parameter 'reqtr'.",
                location=polar_assignments[0].location if polar_assignments else block.location,
            )
        )
    ####
    if not polar_assignments:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="missing-radar-polar-shape",
                message="Direct radar earth-shape input requires one of 'rpolr', 'ecc', or 'flat'.",
                location=shape_assignments[0].location,
            )
        )
    elif len(polar_assignments) > 1:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="conflicting-radar-shape-parameters",
                message="Direct radar earth-shape input may specify only one of 'rpolr', 'ecc', or 'flat'.",
                location=polar_assignments[1].location,
            )
        )
    ####
####


def _validate_unique_block_assignments(block: EarthBlock | RadarBlock, code: str, diagnostics: list[Diagnostic]) -> None:
    seen: set[str] = set()
    for assignment in block.assignments:
        name = assignment.name.casefold()
        if name in seen:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code=code,
                    message=f"Parameter {assignment.name!r} is assigned more than once; source text was preserved.",
                    location=assignment.location,
                )
            )
        seen.add(name)
    ####
####


def _validate_assignment_operators(block: Any, diagnostics: list[Diagnostic]) -> None:
    for assignment in block.assignments:
        if assignment.operator != "=":
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="unsupported-assignment-operator",
                    message=f"Operator {assignment.operator!r} is not documented for '*{block.keyword}' data-block assignments; source text was preserved.",
                    location=assignment.location,
                )
            )
        ####
    ####
####


def _validate_fixed_parameter_uniqueness(block: Any, diagnostics: list[Diagnostic]) -> None:
    if block.keyword not in _UNIQUE_PARAMETER_BLOCKS or block.keyword in {"earth", "radar"}:
        return
    ####
    seen: set[str] = set()
    for assignment in block.assignments:
        name = assignment.name.casefold()
        if name in seen:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="duplicate-block-parameter",
                    message=f"Parameter {assignment.name!r} is assigned more than once in '*{block.keyword}'; source text was preserved.",
                    location=assignment.location,
                )
            )
        seen.add(name)
    ####
####


def _is_numeric_parameter_value(value: Any) -> bool:
    if isinstance(value, (NumberExpression, ParameterExpression)):
        return True
    ####
    return isinstance(value, UnaryExpression) and _is_numeric_parameter_value(value.operand)
####


def _validate_numeric_parameters(block: Any, diagnostics: list[Diagnostic]) -> None:
    if block.keyword not in _NUMERIC_PARAMETER_BLOCKS:
        return
    ####
    for assignment in block.assignments:
        if not _is_numeric_parameter_value(assignment.value):
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="nonnumeric-block-parameter",
                    message=f"Parameter {assignment.name!r} in '*{block.keyword}' requires a numeric value or numeric survey placeholder; source text was preserved.",
                    location=assignment.location,
                )
            )
        ####
    ####
####


def _validate_problem_global_blocks(problem: Problem, diagnostics: list[Diagnostic]) -> None:
    earth_blocks = [block for block in problem.blocks if isinstance(block, EarthBlock)]
    for block in earth_blocks[1:]:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="duplicate-earth-model",
                message="A problem may define only one global earth model; source text was preserved.",
                location=block.location,
            )
        )
    atmos_blocks = [block for block in problem.blocks if isinstance(block, AtmosBlock)]
    for block in atmos_blocks[1:]:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="duplicate-atmosphere-model",
                message="A problem may define only one global atmosphere model; source text was preserved.",
                location=block.location,
            )
        )
    seen_radar_ids: set[int] = set()
    for block in (block for block in problem.blocks if isinstance(block, RadarBlock)):
        if block.radar_id is None or block.radar_id < 1:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="invalid-radar-id",
                    message="Radar station numbers must begin at 1.",
                    location=block.location,
                )
            )
        elif block.radar_id in seen_radar_ids:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="duplicate-radar-id",
                    message=f"Radar station number {block.radar_id} is declared more than once; source text was preserved.",
                    location=block.location,
                )
            )
        else:
            seen_radar_ids.add(block.radar_id)
    ####
####


def _validate_runtime_sensor(
    name: str | None,
    attributes: dict[str, str],
    path: str,
    line: int,
    diagnostics: list[Diagnostic],
) -> None:
    """Validate the provider-neutral sensor clock contract."""

    location = _location(path, line)
    if name is None:
        return
    for required in ("kind", "cadence-s", "sample", "truth", "rate-policy"):
        if required not in attributes:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="missing-sensor-attribute",
                    message=f"Runtime sensor {name!r} requires {required}=.",
                    location=location,
                )
            )
    unknown = sorted(set(attributes) - _SENSOR_ATTRIBUTES - {attribute for attribute in attributes if attribute.startswith("provider-")})
    for attribute in unknown:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="unsupported-sensor-attribute",
                message=f"Runtime sensor attribute {attribute!r} is not in the common sensor contract; use provider-* for provider-specific metadata.",
                location=location,
            )
        )
    kind = attributes.get("kind", "").casefold()
    if kind and kind not in _SENSOR_KINDS:
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-sensor-kind", message=f"Unknown runtime sensor kind {kind!r}.", location=location))
    sample = attributes.get("sample", "").casefold()
    if sample and sample not in _SENSOR_SAMPLE_MODES:
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-sensor-sample-mode", message="Sensor sample must be instantaneous or interval.", location=location))
    truth = attributes.get("truth", "").casefold()
    if truth and truth not in _SENSOR_TRUTH_POLICIES:
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-sensor-truth-policy", message="Sensor truth must be boundary or accepted-segment.", location=location))
    rate_policy = attributes.get("rate-policy", "").casefold()
    if rate_policy and rate_policy not in _SENSOR_RATE_POLICIES:
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-sensor-rate-policy", message="Sensor rate-policy must be split or accumulate.", location=location))
    for attribute, minimum, strict in (("cadence-s", 0.0, True), ("phase-s", 0.0, False), ("delivery-s", 0.0, False)):
        value = attributes.get(attribute)
        if value is None:
            continue
        try:
            parsed = float(value)
        except ValueError:
            parsed = math.nan
        if not math.isfinite(parsed) or (parsed <= minimum if strict else parsed < minimum):
            relation = "positive" if strict else "non-negative"
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-sensor-timing", message=f"Sensor {attribute} must be finite and {relation}.", location=location))
    if sample == "instantaneous" and truth and truth != "boundary":
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="inconsistent-sensor-timing-policy", message="Instantaneous sensors must sample committed truth boundaries.", location=location))
    if sample == "interval" and truth and truth != "accepted-segment":
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="inconsistent-sensor-timing-policy", message="Interval sensors must consume accepted truth segments.", location=location))
    if sample == "instantaneous" and rate_policy and rate_policy != "split":
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="inconsistent-sensor-rate-policy", message="Instantaneous sensors require rate-policy=split.", location=location))
    if sample == "interval" and rate_policy and rate_policy != "accumulate":
        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="inconsistent-sensor-rate-policy", message="Interval sensors require rate-policy=accumulate.", location=location))
    ####


def _validate_runtime_model_binding(
    declaration: str,
    name: str | None,
    attributes: dict[str, str],
    path: str,
    line: int,
    diagnostics: list[Diagnostic],
) -> None:
    """Validate provider-neutral observation, navigation, and feedback references."""

    if name is None:
        return
    required = {"observation": "model", "navigation": "estimator", "feedback": "source"}[declaration]
    if required not in attributes:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="missing-runtime-binding-attribute",
                message=f"Runtime {declaration} {name!r} requires {required}=.",
                location=_location(path, line),
            )
        )
    allowed = {
        "model", "profile", "seed", "input", "estimator", "source", "availability",
        "stale-policy", "max-age-s", "latency-s",
    }
    unknown = sorted(set(attributes) - allowed)
    for attribute in unknown:
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="unsupported-runtime-binding-attribute",
                message=f"Runtime {declaration} attribute {attribute!r} is not supported.",
                location=_location(path, line),
            )
        )
    ####


def _make_block(
    keyword: str,
    header: str,
    scope: str,
    path: str,
    line: int,
    diagnostics: list[Diagnostic],
    *,
    source_text: str | None = None,
):
    common: dict[str, Any] = {
        "keyword": keyword,
        "scope": scope,
        "location": _location(path, line),
        "header": header.strip(),
        "source_text": source_text,
        "assignments": _assignments(header, path, line, diagnostics),
    }
    if keyword == "when":
        common["assignments"] = []
    elif keyword == "search":
        # The relationship in a search header is an objective, not a named
        # assignment.  Keep it out of the generic assignment validator so
        # documented '<' and '>' objectives are not rejected as assignments.
        common["assignments"] = []
    elif keyword == "limits":
        common["assignments"] = []
    elif keyword == "fly":
        common["assignments"] = []
    words = _free_fields(header)
    mapping = {
        "method": ExtensionBlock,
        "mass": ExtensionBlock,
        "ptmass": ExtensionBlock,
        "deployed": ExtensionBlock,
        "cases": ExtensionBlock,
        "atmos": AtmosBlock,
        "earth": EarthBlock,
        "title": TitleBlock,
        "mode": ModeBlock,
        "3dof": DofDirectiveBlock,
        "6dof": DofDirectiveBlock,
        "sixdof": DofDirectiveBlock,
        "define": DefineBlock,
        "egs": EgsBlock,
        "file": FileBlock,
        "print": PrintBlock,
        "radar": RadarBlock,
        "random": RandomBlock,
        "runtime": RuntimeBlock,
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
    if cls is ExtensionBlock:
        if keyword == "method":
            fields = _free_fields(header)
            extra["method"] = fields[0].casefold() if fields else None
            extra["options"] = [field.casefold() for field in fields[1:]]
        elif keyword == "deployed":
            match = re.fullmatch(
                r"\s*from\s+trajectory\s+(?P<trajectory>\d+)\s*,?\s*segment\s+(?P<segment>\d+)\s*(?:,?\s*wt\s*=\s*(?P<weight>.+))?\s*",
                header,
                re.IGNORECASE,
            )
            if match:
                extra["source_trajectory"] = int(match.group("trajectory"))
                extra["source_segment"] = int(match.group("segment"))
                if match.group("weight"):
                    try:
                        extra["deployed_weight"] = _parse_value(match.group("weight"))
                    except ExpressionSyntaxError as exc:
                        diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-deployed-weight", message=str(exc), location=_location(path, line)))
            else:
                diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-deployed-header", message="Expected '*deployed from trajectory N, segment N, wt=value'.", location=_location(path, line)))
        return cls(**common, **extra)
    if keyword == "runtime":
        fields = _free_fields(header)
        declaration = fields[0].casefold() if fields else None
        if declaration not in {"parameter", "control", "status", "event", "output", "lqr", "sensor", "observation", "navigation", "feedback"}:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-runtime-declaration", message="Expected '*runtime parameter|control|status|event|output|lqr|sensor ...'.", location=_location(path, line)))
        else:
            extra["declaration"] = declaration
            extra["name"] = fields[1] if len(fields) > 1 and "=" not in fields[1] else None
            attributes = {match.group("name").casefold(): match.group("value").strip("\"'") for match in _RUNTIME_ATTRIBUTE_RE.finditer(header)}
            extra["attributes"] = attributes
            if declaration in {"parameter", "control", "status", "event", "lqr", "sensor", "observation", "navigation", "feedback"} and extra["name"] is None:
                diagnostics.append(Diagnostic(severity=Severity.ERROR, code="missing-runtime-name", message=f"Runtime {declaration} declarations require a name.", location=_location(path, line)))
            if declaration == "lqr":
                if not attributes.get("states"):
                    diagnostics.append(Diagnostic(severity=Severity.ERROR, code="missing-lqr-states", message="Runtime lqr declarations require states=.", location=_location(path, line)))
                if not attributes.get("controls"):
                    diagnostics.append(Diagnostic(severity=Severity.ERROR, code="missing-lqr-controls", message="Runtime lqr declarations require controls=.", location=_location(path, line)))
                if attributes.get("method", "continuous").casefold() != "continuous":
                    diagnostics.append(Diagnostic(severity=Severity.ERROR, code="unsupported-lqr-method", message="TAORYX currently supports only method=continuous for runtime lqr.", location=_location(path, line)))
            if declaration == "sensor":
                _validate_runtime_sensor(extra["name"], attributes, path, line, diagnostics)
            elif declaration in {"observation", "navigation", "feedback"}:
                _validate_runtime_model_binding(declaration, extra["name"], attributes, path, line, diagnostics)
    elif keyword in {"atmos", "earth"}:
        extra["model"] = words[0] if words else None
        if keyword == "atmos":
            model = (words[0] if words else "").casefold()
            if model not in _ATMOS_STANDARD_MODELS | {"user", "site"} or (model != "rcc" and len(words) != 1) or (model == "rcc" and len(words) not in {1, 2}):
                diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-atmos-header", message="Expected '*atmos 0..20', '*atmos standard|none', '*atmos user', or '*atmos site'.", location=_location(path, line)))
            else:
                extra["options"] = [word.casefold() for word in words[1:]]
        else:
            model = (words[0] if words else "").casefold()
            if model not in _EARTH_MODELS:
                diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-earth-header", message="Expected a documented *earth model family.", location=_location(path, line)))
            else:
                _validate_named_header("earth", header[len(words[0]) :].strip(), common["assignments"], _EARTH_PARAMETER_NAMES, path, line, diagnostics)
    elif keyword == "title":
        extra["title"] = header.strip()
    elif keyword == "mode":
        extra["mode"] = words[0].casefold() if len(words) == 1 else None
        if extra["mode"] not in {"point-mass", "kinematic-6dof", "rigid-body-6dof"}:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="invalid-dynamics-mode",
                    message="Expected '*mode point-mass', '*mode kinematic-6dof', or '*mode rigid-body-6dof'.",
                    location=_location(path, line),
                )
            )
    elif keyword in {"3dof", "6dof", "sixdof"}:
        extra["mode"] = "point-mass" if keyword == "3dof" else "rigid-body-6dof"
    elif keyword == "define":
        integral_match = re.fullmatch(r"\s*integral\s+([A-Za-z_][A-Za-z0-9_.-]*)\s*=\s*(.+?)\s*", header, re.IGNORECASE)
        if integral_match:
            extra["integral"] = True
            extra["variable"] = integral_match.group(1)
            try:
                extra["initial_value"] = _parse_value(integral_match.group(2))
                _validate_expression_functions(extra["initial_value"], path, line, diagnostics)
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
            if re.fullmatch(r"\s*initial\s+variables\s*", header, re.IGNORECASE):
                extra["declaration"] = "initial variables"
                extra["variable"] = None
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
            _validate_output_variables(keyword, extra["variables"], path, line, diagnostics, scope=scope)
    elif keyword == "print":
        extra["variables"] = words
        _validate_output_variables(keyword, words, path, line, diagnostics, scope=scope)
    elif keyword == "random":
        _validate_named_header("random", header, common["assignments"], {"seed"}, path, line, diagnostics)
        if common["assignments"]:
            seed_assignments = [assignment for assignment in common["assignments"] if assignment.name.casefold() == "seed"]
            if len(seed_assignments) > 1:
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="duplicate-random-seed",
                        message="The '*random' header may declare at most one seed value.",
                        location=seed_assignments[-1].location,
                    )
                )
            elif seed_assignments:
                seed_assignment = seed_assignments[0]
                if isinstance(seed_assignment.value, NumberExpression) and float(seed_assignment.value.value).is_integer():
                    extra["seed"] = int(seed_assignment.value.value)
                else:
                    diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="invalid-random-seed",
                            message="The '*random seed' setting must be an integer literal.",
                            location=seed_assignment.location,
                        )
                    )
        common["assignments"] = []
    elif keyword == "survey":
        match = re.fullmatch(r"\s*(\d+)\s+(\S+)(?:\s+(.*?))?\s*", header)
        if match:
            extra["survey_id"] = int(match.group(1))
            extra["name"] = match.group(2)
            if extra["survey_id"] < 1:
                diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-survey-id", message="Survey numbers must begin at 1.", location=_location(path, line)))
            extra["settings"] = _parse_survey_settings(match.group(3) or "", path, line, diagnostics)
        else:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-survey-header", message="Expected '*survey N name'.", location=_location(path, line)))
    elif keyword == "summarize":
        summary_match = re.fullmatch(r"\s*(?P<name>\S+)(?:\s+decm\s*=\s*(?P<decm>\d+))?\s*", header, re.IGNORECASE)
        if summary_match:
            extra["name"] = summary_match.group("name")
            extra["options"] = {"decm": summary_match.group("decm")} if summary_match.group("decm") is not None else {}
        else:
            extra["name"] = words[0] if words else None
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
        file_match = re.search(r"\bfile\s*=\s*([^\s]+)", assignment_header, re.IGNORECASE)
        units_match = re.search(r"\bunits\s*=\s*([^\s]+)", assignment_header, re.IGNORECASE)
        if file_match:
            extra["filename"] = file_match.group(1)
            extra["units"] = units_match.group(1) if units_match else None
            common["assignments"] = []
        else:
            _validate_named_header("wind", assignment_header, common["assignments"], _WIND_SPEED_NAMES | _WIND_COMPONENT_NAMES, path, line, diagnostics)
            if common["assignments"]:
                _validate_wind(common["assignments"], path, line, diagnostics)
            extra["wind_form"] = _wind_form(common["assignments"])
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
            # Assignment-form *initial uses the documented geodetic default;
            # retain it in the typed model so runtime lowering applies it too.
            resolved_coordinate = coordinate or "geodetic"
            extra["mode"] = resolved_coordinate
            extra["coordinate_system"] = resolved_coordinate
        ####
    elif keyword == "dwn/crs":
        _validate_named_header(keyword, header, common["assignments"], {"latgd", "long", "azm"}, path, line, diagnostics)
    elif keyword == "iip":
        _validate_named_header(keyword, header, common["assignments"], {"iip_beta", "iip_alt"}, path, line, diagnostics)
    elif keyword == "tangent":
        _validate_named_header(keyword, header, common["assignments"], {"latgd", "long", "alt", "azm"}, path, line, diagnostics)
    elif keyword in {"aero", "constants", "cg", "prop"}:
        if keyword == "prop" and re.match(r"\s*vacuum\b", header, re.IGNORECASE):
            common["assignments"] = _assignments(re.sub(r"^\s*vacuum\b", "", header, flags=re.IGNORECASE), path, line, diagnostics)
            extra["variant"] = "vacuum"
        else:
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
            if match.group(5) is None:
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="invalid-optimize-header",
                        message="An '*optimize' objective requires both a segment and trajectory number.",
                        location=_location(path, line),
                    )
                )
        else:
            diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-optimize-header", message="Expected '*optimize loop for variable=min|max on segment N, trajectory N'.", location=_location(path, line)))
        ####
    elif keyword == "search":
        match = re.fullmatch(r"\s*(\d+)\s+vary\s+(.+?)\s+until\s+(.+)\s*", header, re.IGNORECASE)
        if match:
            extra["search_id"] = int(match.group(1))
            extra["variable"] = match.group(2).strip()
            if extra["search_id"] < 1:
                diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-search-id", message="Search numbers must begin at 1.", location=_location(path, line)))
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
                if _contains_call_expression(extra["condition"]):
                    diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="unsupported-condition-call",
                            message="*when conditions may not contain function calls; source text was preserved.",
                            location=_location(path, line),
                        )
                    )
                elif not _is_documented_when_relationship(extra["condition"]):
                    diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="unsupported-when-condition",
                            message="*when requires one documented relationship using '=', '!=', '<', '<=', '>', or '>='; compound conditions were preserved.",
                            location=_location(path, line),
                        )
                    )
            except ExpressionSyntaxError as exc:
                diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-when-condition", message=str(exc), location=_location(path, line)))
            ####
            extra["action"] = "goto" if match.group(3) else "stop"
            extra["target_segment"] = int(match.group(3)) if match.group(3) else None
        ####
    return cls(**common, **extra)
####


def parse_problem_text(text: str, path: str = "<memory>", *, profile: GrammarProfile | str = GrammarProfile.TAOS96) -> ProblemDocument:
    document = ProblemDocument(grammar_profile=GrammarProfile(profile))
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

    def recover(raw_line: str, code: str, line_number: int, column: int = 1) -> None:
        source_line = lines[line_number - 1] if 0 < line_number <= len(lines) else None
        semantic_source = source_line.split("#", 1)[0].rstrip() if source_line is not None else None
        preserved = source_line if source_line is not None and raw_line.strip() == semantic_source.strip() else raw_line
        document.recovered_records.append(RecoveredRecord(text=preserved, code=code, location=_location(path, line_number, column)))
    ####

    def recover_assignment_diagnostics(diagnostic_start: int, line_number: int, raw_line: str) -> None:
        for diagnostic in document.diagnostics[diagnostic_start:]:
            if diagnostic.code == "unsupported-assignment-call":
                recover(raw_line, diagnostic.code, line_number)
        ####
    ####

    def recover_diagnostic(diagnostic: Diagnostic) -> None:
        line_number = diagnostic.location.line if diagnostic.location is not None else 1
        column = diagnostic.location.column if diagnostic.location is not None else 1
        source_line = lines[line_number - 1] if 0 < line_number <= len(lines) else ""
        recover(source_line, diagnostic.code, line_number, column)
    ####

    def report_missing_end(problem: Problem, line_number: int) -> None:
        document.diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="missing-end",
                message=f"Problem {problem.name!r} ended without the required '*end' block.",
                location=_location(path, line_number),
            )
        )
        recover_diagnostic(document.diagnostics[-1])
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
        recover_diagnostic(document.diagnostics[-1])
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
        recover_diagnostic(document.diagnostics[-1])
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
        recover_diagnostic(document.diagnostics[-1])
        pending_optimize = ""
        pending_optimize_line = 0
    ####

    def flush_pending_search() -> None:
        nonlocal pending_search, pending_search_line
        if not pending_search:
            return
        ####
        diagnostic_start = len(document.diagnostics)
        _parse_search_objective(pending_search, path, pending_search_line, document.diagnostics, report_incomplete=True)
        for diagnostic in document.diagnostics[diagnostic_start:]:
            recover_diagnostic(diagnostic)
        document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-search-objective", message="Incomplete search objective; source text was preserved.", location=_location(path, pending_search_line)))
        recover_diagnostic(document.diagnostics[-1])
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
            if current_problem is not None and not problem_closed:
                report_missing_end(current_problem, number)
            ####
            problem_name = problem_match.group("name").strip()
            if _PROBLEM_NAME_RE.fullmatch(problem_name) is None:
                document.diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="invalid-problem-name",
                        message=f"Problem name {problem_name!r} must be a nonempty identifier.",
                        location=_location(path, number),
                    )
                )
                recover(line, "invalid-problem-name", number)
            ####
            current_problem = Problem(name=problem_name, location=_location(path, number), source_text=original)
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
                    current_trajectory = None
                    current_segment = None
                    current_block = None
                    recover(line, "invalid-trajectory-header", number)
                    continue
                ####
                trajectory_name = match.group(2).strip()
                if not trajectory_name:
                    document.diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="invalid-trajectory-header",
                            message="A trajectory header requires a nonempty title between its number and 'start on' clause.",
                            location=_location(path, number),
                        )
                    )
                    recover(line, "invalid-trajectory-header", number)
                ####
                trajectory_number = int(match.group(1))
                trajectory_name_key = trajectory_name.casefold()
                if any(item.number == trajectory_number for item in current_problem.trajectories):
                    document.diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="duplicate-trajectory-number",
                            message=f"Trajectory number {trajectory_number} is not unique within this problem; source text was preserved.",
                            location=_location(path, number),
                        )
                    )
                    recover(line, "duplicate-trajectory-number", number)
                if any(item.name.casefold() == trajectory_name_key for item in current_problem.trajectories):
                    document.diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="duplicate-trajectory-name",
                            message=f"Trajectory name {trajectory_name!r} is not unique within this problem; source text was preserved.",
                            location=_location(path, number),
                        )
                    )
                    recover(line, "duplicate-trajectory-name", number)
                current_trajectory = Trajectory(
                    number=trajectory_number,
                    name=trajectory_name,
                    start_segment=int(match.group(3)),
                    location=_location(path, number),
                    source_text=original,
                )
                current_problem.trajectories.append(current_trajectory)
                current_segment = None
                current_block = None
                continue
            ####
            if keyword == "segment":
                if current_trajectory is None:
                    document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="segment-outside-trajectory", message="Segment appears outside a trajectory.", location=_location(path, number)))
                    current_segment = None
                    current_block = None
                    recover(line, "segment-outside-trajectory", number)
                    continue
                ####
                match = re.match(r"(\d+)(?:\s+(.*))?$", header)
                if not match:
                    document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-segment-header", message="Expected '*segment N [title]'.", location=_location(path, number)))
                    current_segment = None
                    current_block = None
                    recover(line, "invalid-segment-header", number)
                    continue
                ####
                segment_number = int(match.group(1))
                if any(item.number == segment_number for item in current_trajectory.segments):
                    document.diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="duplicate-segment-number",
                            message=f"Segment number {segment_number} is not unique within trajectory {current_trajectory.number}; source text was preserved.",
                            location=_location(path, number),
                        )
                    )
                    recover(line, "duplicate-segment-number", number)
                current_segment = Segment(
                    number=segment_number,
                    title=(match.group(2) or "").strip(),
                    location=_location(path, number),
                    source_text=original,
                )
                current_trajectory.segments.append(current_segment)
                current_block = None
                continue
            ####
            problem_keywords = (SUPPORTED_PROBLEM_BLOCKS | SUPPORTED_TAORYX_PROBLEM_BLOCKS) - {"define", "file", "print"}
            # Keep successor directives in the dispatch set even when a
            # profile catalog is narrowed by an embedding application.
            problem_keywords |= {"3dof", "6dof", "sixdof"}
            trajectory_catalog = SUPPORTED_TRAJECTORY_BLOCKS | (SUPPORTED_TAORYX_TRAJECTORY_BLOCKS if document.grammar_profile is GrammarProfile.TAORYX else set())
            segment_catalog = SUPPORTED_SEGMENT_BLOCKS | (SUPPORTED_TAORYX_SEGMENT_BLOCKS if document.grammar_profile is GrammarProfile.TAORYX else set())
            trajectory_keywords = trajectory_catalog - {"define", "file", "print"}
            if keyword in problem_keywords:
                scope = "problem"
                current_segment = None
                current_trajectory = None
                if keyword in {"3dof", "6dof", "sixdof", "random", "runtime", "method", "cases"} and document.grammar_profile is GrammarProfile.TAOS96:
                    document.diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="taoryx-extension-requires-profile",
                            message=f"*{keyword} is a Taoryx extension; parse with profile=taoryx.",
                            location=_location(path, number),
                        )
                    )
                    recover(line, "taoryx-extension-requires-profile", number)
            elif keyword in trajectory_keywords:
                scope = "trajectory"
                current_segment = None
            elif keyword == "define":
                # *define is documented only at problem or trajectory scope;
                # it terminates the current segment context rather than
                # acquiring false segment semantics.
                scope = "trajectory" if current_trajectory is not None else "problem"
                current_segment = None
            elif keyword == "file":
                # File blocks are trajectory-scoped before the first segment
                # and problem-scoped outside trajectory definitions. A
                # completed segment must not leak stale segment semantics.
                scope = "trajectory" if current_trajectory is not None and current_segment is None else "problem"
            elif keyword == "print":
                # Preserve the established segment-level compatibility form
                # for *print; *file has a distinct manual-level boundary.
                scope = "segment" if current_segment is not None else "trajectory" if current_trajectory is not None else "problem"
            elif keyword in segment_catalog:
                # Segment blocks never change meaning based on the current
                # enclosing scope.  Keep the declared scope so the attachment
                # check below can reject a misplaced block instead of giving
                # it false problem/trajectory semantics.
                # Historical *inertial forms are also accepted before the
                # first segment; the typed tests preserve that manual form.
                scope = "trajectory" if keyword == "inertial" and current_trajectory is not None and current_segment is None else "segment"
            else:
                scope = "segment" if current_segment is not None else "trajectory" if current_trajectory is not None else "problem"
            ####
            dual_scope_after_segment = (
                current_trajectory is not None
                and current_trajectory.segments
                and (
                    keyword == "file"
                    or keyword in {"define", "print"} and current_segment is None
                )
            )
            if dual_scope_after_segment:
                # These keywords are documented at both problem and trajectory
                # scope. Once a trajectory has segments, the line alone does
                # not establish whether the author is continuing that
                # trajectory or beginning the problem-level tail. Retain the
                # established trajectory attachment, but make the ambiguity
                # explicit instead of silently claiming problem-level meaning.
                document.diagnostics.append(
                    Diagnostic(
                        severity=Severity.WARNING,
                        code="ambiguous-dual-scope-block",
                        message=f"*{keyword} is documented at both problem and trajectory scope after a completed segment; trajectory scope was retained without inferring problem-level semantics.",
                        location=_location(path, number),
                    )
                )
                recover(line, "ambiguous-dual-scope-block", number)
            ####
            diagnostic_start = len(document.diagnostics)
            block = _make_block(keyword, header, scope, path, number, document.diagnostics, source_text=original)
            if block is None:
                # An unrecognized header is a synchronization boundary.  Do
                # not let following text inherit the previous block's typed
                # meaning; it will be retained as orphan text below.
                current_block = None
                pending_define = ""
                pending_define_line = 0
                define_brace_stack.clear()
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
            if (
                isinstance(block, SearchBlock)
                and block.objective is not None
                and block.objective.operator is None
                and block.objective.left.text.casefold() not in {"min", "max"}
            ):
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
            current_block.statements.append(RawStatement(text=original, location=_location(path, number)))
            if isinstance(current_block, DefineBlock):
                stripped = line.strip()
                helper_match = re.fullmatch(r"(surface_ref)\s*\(([^()]*)\)\s*;?", stripped, re.IGNORECASE)
                if helper_match:
                    current_block.helper_calls.append(stripped)
                    pending_define = ""
                    continue
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
                        if define_brace_stack:
                            parent, in_else = define_brace_stack[-1]
                            (parent.else_controls if in_else else parent.body_controls).append(control)
                        else:
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
                        diagnostic_start = len(document.diagnostics)
                        typed = _define_statement(statement_text, path, pending_define_line or number, document.diagnostics)
                        for diagnostic in document.diagnostics[diagnostic_start:]:
                            recover_diagnostic(diagnostic)
                        ####
                        if typed is None:
                            continue
                        if define_brace_stack:
                            control, in_else = define_brace_stack[-1]
                            if isinstance(typed, DefineAssignmentStatement):
                                (control.else_body if in_else else control.body).append(typed.assignment)
                            else:
                                (control.else_controls if in_else else control.body_controls).append(typed)
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
                    constraint_diagnostic_start = len(document.diagnostics)
                    constraint = _parse_optimize_constraint(candidate, path, pending_optimize_line, document.diagnostics)
                    for diagnostic in document.diagnostics[constraint_diagnostic_start:]:
                        recover_diagnostic(diagnostic)
                    if constraint is None:
                        document.diagnostics.append(
                            Diagnostic(
                                severity=Severity.ERROR,
                                code="invalid-optimize-constraint",
                                message="Malformed optimization constraint; source text was preserved.",
                                location=_location(path, pending_optimize_line),
                            )
                        )
                        recover_diagnostic(document.diagnostics[-1])
                    else:
                        current_block.constraints.append(constraint)
                    pending_optimize = ""
                    pending_optimize_line = 0
                    continue
                if stripped.casefold().startswith("constrain"):
                    candidate = f"{pending_optimize} {stripped}".strip() if pending_optimize else stripped
                    constraint_diagnostic_start = len(document.diagnostics)
                    constraint = _parse_optimize_constraint(candidate, path, pending_optimize_line or number, document.diagnostics)
                    for diagnostic in document.diagnostics[constraint_diagnostic_start:]:
                        recover_diagnostic(diagnostic)
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
                            recover_diagnostic(document.diagnostics[-1])
                            pending_optimize = ""
                            pending_optimize_line = 0
                    else:
                        current_block.constraints.append(constraint)
                        pending_optimize = ""
                        pending_optimize_line = 0
                    continue
                ####
                diagnostic_start = len(document.diagnostics)
                assignments = _assignments(line, path, number, document.diagnostics)
                recover_assignment_diagnostics(diagnostic_start, number, line)
                _validate_optimize_controls(assignments, path, number, document.diagnostics, allow_symbolic=document.grammar_profile is GrammarProfile.TAORYX)
                if assignments and _diagnose_assignment_residual(line, "optimize", path, number, document.diagnostics):
                    recover_diagnostic(document.diagnostics[-1])
                elif not assignments and len(document.diagnostics) == diagnostic_start:
                    document.diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="invalid-optimize-control-line",
                            message="Expected a documented optimization control assignment; source text was preserved.",
                            location=_location(path, number),
                        )
                    )
                    recover(line, "invalid-optimize-control-line", number)
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
                        recover_diagnostic(document.diagnostics[-1])
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
                diagnostic_start = len(document.diagnostics)
                assignments = _assignments(line, path, number, document.diagnostics)
                recover_assignment_diagnostics(diagnostic_start, number, line)
                _validate_search_controls(assignments, path, number, document.diagnostics)
                if assignments and _diagnose_assignment_residual(line, "search", path, number, document.diagnostics):
                    recover_diagnostic(document.diagnostics[-1])
                elif not assignments and len(document.diagnostics) == diagnostic_start:
                    document.diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="invalid-search-control-line",
                            message="Expected a documented search control assignment; source text was preserved.",
                            location=_location(path, number),
                        )
                    )
                    recover(line, "invalid-search-control-line", number)
                current_block.controls.extend(assignments)
                current_block.assignments.extend(assignments)
            elif isinstance(current_block, SurveyBlock):
                before = len(document.diagnostics)
                settings = _parse_survey_settings(line, path, number, document.diagnostics)
                for diagnostic in document.diagnostics[before:]:
                    recover(line, diagnostic.code, number)
                if not settings and len(document.diagnostics) == before:
                    recover(line, "invalid-survey-setting", number)
                current_block.settings.extend(settings)
            elif isinstance(current_block, UnitsFormatBlock):
                before = len(document.diagnostics)
                settings = _parse_units_format_settings(line, path, number, document.diagnostics)
                if len(document.diagnostics) > before:
                    for diagnostic in document.diagnostics[before:]:
                        recover(line, diagnostic.code, number)
                elif not settings:
                    recover(line, "invalid-units-format-setting", number)
                current_block.settings.extend(settings)
            elif isinstance(current_block, (FileBlock, EgsBlock, PrintBlock)):
                tokens = _free_fields(line.strip())
                if isinstance(current_block, EgsBlock) and current_block.summary:
                    document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-egs-summary", message="The '*egs summary file' form does not accept output variable lines.", location=_location(path, number)))
                    recover(line, "invalid-egs-summary", number)
                else:
                    _validate_output_variables(current_block.keyword, tokens, path, number, document.diagnostics, scope=current_block.scope)
                    current_block.variables.extend(tokens)
                    for diagnostic in document.diagnostics:
                        if diagnostic.location.line == number and diagnostic.code in {"invalid-output-variable", "invalid-related-output-subscript"}:
                            recover(line, diagnostic.code, number)
            elif isinstance(current_block, WindBlock):
                diagnostic_start = len(document.diagnostics)
                assignments = _assignments(line, path, number, document.diagnostics)
                recover_assignment_diagnostics(diagnostic_start, number, line)
                _validate_named_header("wind", line, assignments, _WIND_SPEED_NAMES | _WIND_COMPONENT_NAMES, path, number, document.diagnostics)
                current_block.assignments.extend(assignments)
                current_block.wind_form = _wind_form(current_block.assignments)
                if len(current_block.assignments) >= 3:
                    before = len(document.diagnostics)
                    _validate_wind(current_block.assignments, path, number, document.diagnostics)
                    if len(document.diagnostics) > before:
                        recover(line, document.diagnostics[-1].code, number)
            elif isinstance(current_block, RandomBlock):
                diagnostic_start = len(document.diagnostics)
                assignments = _assignments(line, path, number, document.diagnostics, allow_functions=True)
                recover_assignment_diagnostics(diagnostic_start, number, line)
                if not assignments and len(document.diagnostics) == diagnostic_start:
                    document.diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="invalid-random-body",
                            message="Expected one or more random-variable assignments in '*random'.",
                            location=_location(path, number),
                        )
                    )
                    recover(line, "invalid-random-body", number)
                current_block.assignments.extend(assignments)
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
                if current_block.mode == "from":
                    diagnostic_start = len(document.diagnostics)
                    assignments = _assignments(line, path, number, document.diagnostics)
                    recover_assignment_diagnostics(diagnostic_start, number, line)
                    if not assignments:
                        document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-initial-copy-assignment", message="Expected t_0 or omega_0 assignment after a copied '*initial' state.", location=_location(path, number)))
                        recover(line, "invalid-initial-copy-assignment", number)
                    elif _diagnose_assignment_residual(line, "initial", path, number, document.diagnostics):
                        recover_diagnostic(document.diagnostics[-1])
                    current_block.assignments.extend(assignments)
                    before = len(document.diagnostics)
                    _validate_initial_copy_assignments(assignments, path, number, document.diagnostics)
                    for diagnostic in document.diagnostics[before:]:
                        recover(line, diagnostic.code, number)
                    continue
                ####
                diagnostic_start = len(document.diagnostics)
                assignments = _assignments(line, path, number, document.diagnostics)
                recover_assignment_diagnostics(diagnostic_start, number, line)
                if not assignments:
                    document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-initial-assignment", message="Expected an assignment in an *initial block.", location=_location(path, number)))
                    recover(line, "invalid-initial-assignment", number)
                elif _diagnose_assignment_residual(line, "initial", path, number, document.diagnostics):
                    recover_diagnostic(document.diagnostics[-1])
                current_block.assignments.extend(assignments)
            elif isinstance(current_block, LimitsBlock):
                before = len(document.diagnostics)
                limits = _parse_limits(line.strip(), path, number, document.diagnostics)
                if len(document.diagnostics) > before:
                    for diagnostic in document.diagnostics[before:]:
                        recover(line, diagnostic.code, number)
                if not limits:
                    document.diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="invalid-limits-body",
                            message="A continued '*limits' line must contain one or more documented relationships.",
                            location=_location(path, number),
                        )
                    )
                    recover(line, "invalid-limits-body", number)
                current_block.limits.extend(limits)
            elif isinstance(current_block, WhenBlock):
                document.diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="invalid-block-body",
                        message=f"'*{current_block.keyword}' is a header-only block; continuation text was preserved without assignment semantics.",
                        location=_location(path, number),
                    )
                )
                recover(line, "invalid-block-body", number)
            elif isinstance(current_block, RadarBlock):
                body = line.strip()
                shape_match = re.match(r"^(?P<shape>wgs-72|wgs-84)\b", body, re.IGNORECASE)
                if shape_match:
                    shape = shape_match.group("shape").casefold()
                    if current_block.earth_shape is not None and current_block.earth_shape != shape:
                        document.diagnostics.append(
                            Diagnostic(
                                severity=Severity.ERROR,
                                code="conflicting-radar-earth-shape",
                                message="A '*radar' station cannot specify both wgs-72 and wgs-84.",
                                location=_location(path, number),
                            )
                        )
                        recover(line, "conflicting-radar-earth-shape", number)
                    else:
                        current_block.earth_shape = shape
                    body = body[shape_match.end() :].strip()
                ####
                diagnostic_start = len(document.diagnostics)
                assignments = _assignments(body, path, number, document.diagnostics)
                recover_assignment_diagnostics(diagnostic_start, number, line)
                validation_start = len(document.diagnostics)
                _validate_continuation_parameters("radar", assignments, path, number, document.diagnostics)
                for diagnostic in document.diagnostics[validation_start:]:
                    recover(line, diagnostic.code, number)
                if assignments and _diagnose_assignment_residual(body, "radar", path, number, document.diagnostics):
                    recover(line, "invalid-assignment-line", number)
                elif not assignments and not shape_match:
                    document.diagnostics.append(
                        Diagnostic(
                            severity=Severity.ERROR,
                            code="invalid-radar-setting",
                            message="Expected a documented radar parameter or wgs-72/wgs-84 earth-shape selector.",
                            location=_location(path, number),
                        )
                    )
                    recover(line, "invalid-radar-setting", number)
                current_block.assignments.extend(assignments)
            elif isinstance(current_block, AtmosBlock) and current_block.model in {"user", "site"}:
                before = len(document.diagnostics)
                _parse_atmos_body(current_block, line.strip(), path, number, document.diagnostics)
                if len(document.diagnostics) > before:
                    recover(line, document.diagnostics[-1].code, number)
            elif isinstance(current_block, ExtensionBlock):
                current_block.raw_lines.append(line)
                fields = line.strip().split()
                if current_block.keyword == "cases" and fields:
                    if not current_block.columns:
                        current_block.columns.extend(fields)
                    else:
                        current_block.rows.append(fields)
            else:
                diagnostic_start = len(document.diagnostics)
                assignments = _assignments(line, path, number, document.diagnostics)
                recover_assignment_diagnostics(diagnostic_start, number, line)
                validation_start = len(document.diagnostics)
                allowed_names = _CONTINUATION_PARAMETER_NAMES.get(current_block.keyword)
                if current_block.keyword == "inertial":
                    allowed_names = {
                        "body": {"time"},
                        "ecfc": {"eastx", "easty", "eastz", "downx", "downy", "downz", "time"},
                        "geocentric": {"long", "lat", "rcm", "time"},
                        "geodetic": {"long", "lat", "alt", "time"},
                        "velocity": {"time"},
                        "wind": {"time"},
                    }.get(getattr(current_block, "alignment", None))
                _validate_continuation_parameters(current_block.keyword, assignments, path, number, document.diagnostics, allowed_names=allowed_names)
                for diagnostic in document.diagnostics[validation_start:]:
                    recover(line, diagnostic.code, number)
                if assignments and current_block.keyword in _ASSIGNMENT_BODY_BLOCKS:
                    if _diagnose_assignment_residual(line, current_block.keyword, path, number, document.diagnostics):
                        recover_diagnostic(document.diagnostics[-1])
                ####
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
        else:
            document.diagnostics.append(Diagnostic(severity=Severity.WARNING, code="orphan-line", message=f"Line is not attached to a block: {line.strip()!r}", location=_location(path, number)))
            recover(line, "orphan-line", number)
        ####
    ####
    flush_pending_define(len(lines) or 1)
    flush_pending_optimize()
    flush_pending_search()
    if current_problem is not None and not problem_closed:
        report_missing_end(current_problem, len(lines) or current_problem.location.line)
    for problem in document.problems:
        before = len(document.diagnostics)
        _validate_problem_global_blocks(problem, document.diagnostics)
        for diagnostic in document.diagnostics[before:]:
            recover_diagnostic(diagnostic)
        before = len(document.diagnostics)
        _validate_survey_output_names(problem, document.diagnostics)
        for diagnostic in document.diagnostics[before:]:
            recover_diagnostic(diagnostic)
        before = len(document.diagnostics)
        _validate_units_format_user_variables(problem, document.diagnostics)
        for diagnostic in document.diagnostics[before:]:
            recover_diagnostic(diagnostic)
        before = len(document.diagnostics)
        _validate_define_target_names(problem, document.diagnostics)
        for diagnostic in document.diagnostics[before:]:
            recover_diagnostic(diagnostic)
        before = len(document.diagnostics)
        _validate_egs_summary_prerequisites(problem, document.diagnostics)
        for diagnostic in document.diagnostics[before:]:
            recover_diagnostic(diagnostic)
        for trajectory in problem.trajectories:
            before = len(document.diagnostics)
            _validate_trajectory_structure(trajectory, document.diagnostics)
            for diagnostic in document.diagnostics[before:]:
                recover_diagnostic(diagnostic)
            before = len(document.diagnostics)
            _validate_segment_termination(trajectory, document.diagnostics)
            for diagnostic in document.diagnostics[before:]:
                recover_diagnostic(diagnostic)
            before = len(document.diagnostics)
            _validate_fly_blocks(trajectory, document.diagnostics)
            for diagnostic in document.diagnostics[before:]:
                recover_diagnostic(diagnostic)
            before = len(document.diagnostics)
            _validate_inertial_blocks(trajectory, document.diagnostics)
            for diagnostic in document.diagnostics[before:]:
                recover_diagnostic(diagnostic)
            for segment in trajectory.segments:
                before = len(document.diagnostics)
                _validate_fly_angle_set(segment, document.diagnostics)
                if len(document.diagnostics) > before:
                    diagnostic = document.diagnostics[-1]
                    recover_diagnostic(diagnostic)
        ####
        for block in problem.blocks + [child for trajectory in problem.trajectories for child in trajectory.blocks] + [child for trajectory in problem.trajectories for segment in trajectory.segments for child in segment.blocks]:
            before = len(document.diagnostics)
            _validate_assignment_operators(block, document.diagnostics)
            for diagnostic in document.diagnostics[before:]:
                recover_diagnostic(diagnostic)
            before = len(document.diagnostics)
            _validate_fixed_parameter_uniqueness(block, document.diagnostics)
            for diagnostic in document.diagnostics[before:]:
                recover_diagnostic(diagnostic)
            before = len(document.diagnostics)
            _validate_numeric_parameters(block, document.diagnostics)
            for diagnostic in document.diagnostics[before:]:
                recover_diagnostic(diagnostic)
            if isinstance(block, DefineBlock):
                before = len(document.diagnostics)
                _validate_define_target(block, document.diagnostics)
                for diagnostic in document.diagnostics[before:]:
                    recover_diagnostic(diagnostic)
                before = len(document.diagnostics)
                _validate_define_control_structure(block, document.diagnostics)
                for diagnostic in document.diagnostics[before:]:
                    recover_diagnostic(diagnostic)
                before = len(document.diagnostics)
                _validate_problem_define_table_calls(block, document.diagnostics)
                for diagnostic in document.diagnostics[before:]:
                    recover_diagnostic(diagnostic)
                before = len(document.diagnostics)
                _validate_define_assignment_order(block, document.diagnostics)
                for diagnostic in document.diagnostics[before:]:
                    recover_diagnostic(diagnostic)
            if isinstance(block, AtmosBlock) and block.model in {"user", "site"}:
                before = len(document.diagnostics)
                _validate_atmos_rows(block, document.diagnostics)
                for diagnostic in document.diagnostics[before:]:
                    recover_diagnostic(diagnostic)
            if isinstance(block, SurveyBlock):
                before = len(document.diagnostics)
                _validate_survey_configuration(block, document.diagnostics)
                for diagnostic in document.diagnostics[before:]:
                    recover_diagnostic(diagnostic)
            if isinstance(block, EarthBlock):
                before = len(document.diagnostics)
                _validate_earth_shape_parameters(block, document.diagnostics)
                for diagnostic in document.diagnostics[before:]:
                    recover_diagnostic(diagnostic)
                before = len(document.diagnostics)
                _validate_unique_block_assignments(block, "duplicate-earth-parameter", document.diagnostics)
                for diagnostic in document.diagnostics[before:]:
                    recover_diagnostic(diagnostic)
            if isinstance(block, RadarBlock):
                before = len(document.diagnostics)
                _validate_radar_shape_parameters(block, document.diagnostics)
                for diagnostic in document.diagnostics[before:]:
                    recover_diagnostic(diagnostic)
                before = len(document.diagnostics)
                _validate_unique_block_assignments(block, "duplicate-radar-parameter", document.diagnostics)
                for diagnostic in document.diagnostics[before:]:
                    recover_diagnostic(diagnostic)
            if isinstance(block, LimitsBlock):
                before = len(document.diagnostics)
                _validate_limits(block, document.diagnostics)
                for diagnostic in document.diagnostics[before:]:
                    recover_diagnostic(diagnostic)
            if isinstance(block, AeroBlock) and document.grammar_profile is GrammarProfile.TAOS96:
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
                    recover_diagnostic(document.diagnostics[-1])
            if isinstance(block, PropulsionBlock):
                before = len(document.diagnostics)
                _validate_propulsion_units(block, document.diagnostics)
                for diagnostic in document.diagnostics[before:]:
                    recover_diagnostic(diagnostic)
            if block.keyword in {"reset", "increment"}:
                before = len(document.diagnostics)
                _validate_reset_increment(block, problem, document.diagnostics)
                for diagnostic in document.diagnostics[before:]:
                    recover_diagnostic(diagnostic)
            if isinstance(block, WindBlock) and block.coordinate_system is not None and block.filename is None and len(block.assignments) != 3:
                document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-wind-components", message="A '*wind' block requires exactly windd plus either windv/windh or winde/windn.", location=block.location))
                recover_diagnostic(document.diagnostics[-1])
            if isinstance(block, WindBlock) and block.wind_form is None and len(block.assignments) == 3:
                block.wind_form = _wind_form(block.assignments)
            if isinstance(block, InitialBlock) and block.mode != "from":
                before = len(document.diagnostics)
                _validate_initial_assignments(block, path, block.location.line, document.diagnostics, profile=document.grammar_profile)
                names = {assignment.name.casefold() for assignment in block.assignments}
                if not names & {"wt", "mass"}:
                    document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="missing-initial-mass", message="A direct '*initial' block requires wt or mass.", location=block.location))
                if len(document.diagnostics) > before:
                    for diagnostic in document.diagnostics[before:]:
                        recover_diagnostic(diagnostic)
            if isinstance(block, (FileBlock, EgsBlock)) and block.filename is not None and not getattr(block, "summary", False) and not block.variables:
                document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="missing-output-variables", message=f"'*{block.keyword}' requires at least one output variable.", location=block.location))
                recover_diagnostic(document.diagnostics[-1])
            if isinstance(block, PrintBlock) and len(block.variables) > 10 and document.grammar_profile is GrammarProfile.TAOS96:
                document.diagnostics.append(
                    Diagnostic(
                        severity=Severity.ERROR,
                        code="too-many-print-variables",
                        message="A '*print' block may contain no more than ten output variables.",
                        location=block.location,
                    )
                )
                recover_diagnostic(document.diagnostics[-1])
        ####
    ####
    if not document.problems:
        document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="missing-problem", message="No '(problem-name)' declaration was found.", location=_location(path, 1)))
    ####
    recovered_keys = {(record.code, record.location.line, record.location.column) for record in document.recovered_records}
    for diagnostic in document.diagnostics:
        location = diagnostic.location
        if location is None:
            continue
        key = (diagnostic.code, location.line, location.column)
        if key in recovered_keys:
            continue
        same_line = next(
            (
                record
                for record in document.recovered_records
                if record.code == diagnostic.code and record.location.line == location.line
            ),
            None,
        )
        if same_line is not None:
            recovered_keys.discard((same_line.code, same_line.location.line, same_line.location.column))
            same_line.location = location
            recovered_keys.add(key)
            continue
        source_line = lines[location.line - 1] if 0 < location.line <= len(lines) else ""
        document.recovered_records.append(
            RecoveredRecord(
                text=source_line,
                code=diagnostic.code,
                location=location,
            )
        )
        recovered_keys.add(key)
    ####
    document.diagnostics.sort(
        key=lambda diagnostic: (
            diagnostic.location.line if diagnostic.location is not None else len(lines) + 1,
            diagnostic.location.column if diagnostic.location is not None else 1,
        )
    )
    ####
    return document
####


def parse_problem_file(path: str | Path, *, profile: GrammarProfile | str = GrammarProfile.TAOS96) -> ProblemDocument:
    source = Path(path)
    text = source.read_bytes().decode("utf-8", errors="surrogateescape")
    return parse_problem_text(text, str(source), profile=profile)
####
