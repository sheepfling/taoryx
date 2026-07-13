from __future__ import annotations

from typing import Annotated, Literal

from pydantic import BaseModel, Field

from taoryx.language.diagnostics import Diagnostic, SourceLocation
from taoryx.language.expressions import ExpressionType


class Assignment(BaseModel):
    name: str
    operator: Literal["=", "<", ">", "<=", ">=", "==", "!=", "+=", "-=", "*=", "/="] = "="
    value: ExpressionType
    location: SourceLocation
####


class RawStatement(BaseModel):
    text: str
    location: SourceLocation
####


class RecoveredRecord(BaseModel):
    """Source text retained after a syntax error or unsupported construct."""

    text: str
    code: str
    location: SourceLocation
####


class DefineControlStatement(BaseModel):
    kind: Literal["if", "else"]
    location: SourceLocation
    condition: ExpressionType | None = None
    assignment: Assignment | None = None
    nested: "DefineControlStatement | None" = None
####


class DefineAssignmentStatement(BaseModel):
    kind: Literal["assignment"] = "assignment"
    assignment: Assignment
####


class BlockBase(BaseModel):
    keyword: str
    scope: Literal["problem", "trajectory", "segment"]
    location: SourceLocation
    header: str = ""
    assignments: list[Assignment] = Field(default_factory=list)
    statements: list[RawStatement] = Field(default_factory=list)
####


class AtmosBlock(BlockBase):
    keyword: Literal["atmos"] = "atmos"
    model: str | None = None
    columns: list[str] = Field(default_factory=list)
    rows: list[list[float]] = Field(default_factory=list)
####


class EarthBlock(BlockBase):
    keyword: Literal["earth"] = "earth"
    model: str | None = None
####


class TitleBlock(BlockBase):
    keyword: Literal["title"] = "title"
    title: str = ""
####


class DefineBlock(BlockBase):
    keyword: Literal["define"] = "define"
    variable: str | None = None
    control_statements: list[DefineControlStatement] = Field(default_factory=list)
    typed_statements: list[DefineAssignmentStatement | DefineControlStatement] = Field(default_factory=list)
####


class EgsBlock(BlockBase):
    keyword: Literal["egs"] = "egs"
    filename: str | None = None
    summary: bool = False
    variables: list[str] = Field(default_factory=list)
####


class FileBlock(BlockBase):
    keyword: Literal["file"] = "file"
    filename: str | None = None
    variables: list[str] = Field(default_factory=list)
####


class PrintBlock(BlockBase):
    keyword: Literal["print"] = "print"
    variables: list[str] = Field(default_factory=list)
####


class RadarBlock(BlockBase):
    keyword: Literal["radar"] = "radar"
    radar_id: int | None = None
    station_name: str | None = None
    earth_shape: Literal["wgs-72", "wgs-84"] | None = None
####


class OptimizeEndpoint(BaseModel):
    text: str
    expression: ExpressionType | None = None
    segment: int | None = None
    trajectory: int | None = None
    trajectory_subscript: int | None = None
####


class OptimizeConstraint(BaseModel):
    left: OptimizeEndpoint
    operator: Literal["=", "<", ">"]
    right: OptimizeEndpoint
    reference: ExpressionType | None = None
    location: SourceLocation
####


class OptimizeBlock(BlockBase):
    keyword: Literal["optimize"] = "optimize"
    loop: str | None = None
    objective_variable: str | None = None
    objective_mode: Literal["min", "max"] | None = None
    segment: int | None = None
    trajectory: int | None = None
    constraints: list[OptimizeConstraint] = Field(default_factory=list)
    controls: list[Assignment] = Field(default_factory=list)
####


class SearchObjective(BaseModel):
    left: OptimizeEndpoint
    operator: Literal["=", "<", ">"] | None = None
    right: OptimizeEndpoint | None = None
####


class SearchBlock(BlockBase):
    keyword: Literal["search"] = "search"
    search_id: int | None = None
    variable: str | None = None
    objective: SearchObjective | None = None
    controls: list[Assignment] = Field(default_factory=list)
####


class SummarizeBlock(BlockBase):
    keyword: Literal["summarize"] = "summarize"
    name: str | None = None
    operations: list["SummaryOperation"] = Field(default_factory=list)


class SummaryOperand(BaseModel):
    text: str
    expression: ExpressionType | None = None
    function: Literal["max", "min", "first", "last"] | None = None
    segment: int | None = None
    trajectory: int | None = None


class SummaryOperation(BaseModel):
    operation: Literal[
        "add", "sub", "mult", "div", "idiv", "exp", "iexp",
        "abs", "neg", "sqr", "sqrt", "ln", "log", "e", "sin",
        "cos", "tan", "asin", "acos", "atan",
    ]
    operand: SummaryOperand | None = None
    location: SourceLocation
####


class SurveyBlock(BlockBase):
    keyword: Literal["survey"] = "survey"
    survey_id: int | None = None
    name: str | None = None
    settings: list["SurveySetting"] = Field(default_factory=list)
####


class SurveySetting(BaseModel):
    name: Literal["lo", "hi", "inc", "vals"]
    values: list[str]
    location: SourceLocation
####


class UnitsFormatBlock(BlockBase):
    keyword: Literal["units/fmt"] = "units/fmt"
    settings: list["UnitFormatSetting"] = Field(default_factory=list)
####


class UnitFormatSetting(BaseModel):
    variable: str
    unit: str
    format: str | None = None
    location: SourceLocation
####


class WindBlock(BlockBase):
    keyword: Literal["wind"] = "wind"
    coordinate_system: Literal["geocentric", "geodetic"] | None = None
####


class DownrangeCrossrangeBlock(BlockBase):
    keyword: Literal["dwn/crs"] = "dwn/crs"
####


class IipBlock(BlockBase):
    keyword: Literal["iip"] = "iip"
####


class InitialBlock(BlockBase):
    keyword: Literal["initial"] = "initial"
    mode: str | None = None
    coordinate_system: Literal["geodetic", "geocentric", "ecfc", "ecic"] | None = None
    source_trajectory: int | None = None
    source_segment: int | None = None
####


class TangentBlock(BlockBase):
    keyword: Literal["tangent"] = "tangent"
####


class AeroBlock(BlockBase):
    keyword: Literal["aero"] = "aero"
####


class ConstantsBlock(BlockBase):
    keyword: Literal["constants"] = "constants"
####


class CgBlock(BlockBase):
    keyword: Literal["cg"] = "cg"
####


class FlyBlock(BlockBase):
    keyword: Literal["fly"] = "fly"
    guidance_variable: str | None = None
    value: ExpressionType | None = None
    reference: str | None = None
    interpolation: str | None = None
####


class IncrementBlock(BlockBase):
    keyword: Literal["increment"] = "increment"
####


class InertialBlock(BlockBase):
    keyword: Literal["inertial"] = "inertial"
    alignment: Literal["body", "ecfc", "geocentric", "geodetic", "velocity", "wind"] | None = None
    coordinate_system: Literal["ecfc", "geocentric", "geodetic"] | None = None
####


class IntegrationBlock(BlockBase):
    keyword: Literal["integ"] = "integ"
####


class Limit(BaseModel):
    variable: str
    operator: Literal["<", ">"]
    value: ExpressionType
    location: SourceLocation
####


class LimitsBlock(BlockBase):
    keyword: Literal["limits"] = "limits"
    limits: list[Limit] = Field(default_factory=list)
####


class PropulsionBlock(BlockBase):
    keyword: Literal["prop"] = "prop"
####


class RailBlock(BlockBase):
    keyword: Literal["rail"] = "rail"
    mode: str | None = None
####


class ResetBlock(BlockBase):
    keyword: Literal["reset"] = "reset"
####


class WhenBlock(BlockBase):
    keyword: Literal["when"] = "when"
    condition: ExpressionType | None = None
    action: Literal["goto", "stop"] | None = None
    target_segment: int | None = None
####


ProblemBlock = Annotated[
    AtmosBlock
    | DefineBlock
    | EarthBlock
    | EgsBlock
    | FileBlock
    | OptimizeBlock
    | PrintBlock
    | RadarBlock
    | SearchBlock
    | SummarizeBlock
    | SurveyBlock
    | TitleBlock
    | UnitsFormatBlock
    | WindBlock,
    Field(discriminator="keyword"),
]

TrajectoryBlock = Annotated[
    DefineBlock | DownrangeCrossrangeBlock | FileBlock | IipBlock | InitialBlock | PrintBlock | TangentBlock,
    Field(discriminator="keyword"),
]

SegmentBlock = Annotated[
    AeroBlock
    | ConstantsBlock
    | CgBlock
    | FlyBlock
    | IncrementBlock
    | InertialBlock
    | IntegrationBlock
    | LimitsBlock
    | PropulsionBlock
    | RailBlock
    | ResetBlock
    | WhenBlock,
    Field(discriminator="keyword"),
]

AnyBlock = ProblemBlock | TrajectoryBlock | SegmentBlock


class Segment(BaseModel):
    number: int
    title: str
    location: SourceLocation
    blocks: list[SegmentBlock] = Field(default_factory=list)
####


class Trajectory(BaseModel):
    number: int
    name: str
    start_segment: int
    location: SourceLocation
    blocks: list[TrajectoryBlock] = Field(default_factory=list)
    segments: list[Segment] = Field(default_factory=list)
####


class Problem(BaseModel):
    name: str
    location: SourceLocation
    blocks: list[ProblemBlock] = Field(default_factory=list)
    trajectories: list[Trajectory] = Field(default_factory=list)
    ended: bool = False
####


class ProblemDocument(BaseModel):
    problems: list[Problem] = Field(default_factory=list)
    diagnostics: list[Diagnostic] = Field(default_factory=list)
    recovered_records: list[RecoveredRecord] = Field(default_factory=list)
####


class TableAssignment(BaseModel):
    name: str
    values: list[float]
    location: SourceLocation
####


class TableCall(BaseModel):
    name: str
    arguments: list[str] = Field(default_factory=list)
####


class TableOperation(BaseModel):
    operator: str
    location: SourceLocation
    operand: str | float | TableCall | None = None
    label: str | None = None
    condition: str | None = None
    extrapolation: str | None = None
    assignments: list[TableAssignment] = Field(default_factory=list)
    nested: "TableOperation | None" = None
####


class TableDefinition(BaseModel):
    name: str
    table_type: str
    format: Literal["simple", "full"]
    location: SourceLocation
    independent_variables: list[str] = Field(default_factory=list)
    options: dict[str, str | float] = Field(default_factory=dict)
    assignments: list[TableAssignment] = Field(default_factory=list)
    operations: list[TableOperation] = Field(default_factory=list)
    omissions: list[int] = Field(default_factory=list)
####


class TableDocument(BaseModel):
    tables: list[TableDefinition] = Field(default_factory=list)
    diagnostics: list[Diagnostic] = Field(default_factory=list)
    recovered_records: list[RecoveredRecord] = Field(default_factory=list)
    executable_complete: bool = True
####
