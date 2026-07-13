from __future__ import annotations

import re
from pathlib import Path
from typing import Any

from taoryx.language.diagnostics import Diagnostic, Severity, SourceLocation
from taoryx.language.expressions import (
    ExpressionSyntaxError,
    TableReferenceExpression,
    parse_expression,
)
from taoryx.language.models import (
    AeroBlock,
    Assignment,
    AtmosBlock,
    CgBlock,
    ConstantsBlock,
    DefineBlock,
    DownrangeCrossrangeBlock,
    EarthBlock,
    EgsBlock,
    FileBlock,
    FlyBlock,
    IipBlock,
    IncrementBlock,
    InertialBlock,
    InitialBlock,
    IntegrationBlock,
    LimitsBlock,
    OptimizeBlock,
    PrintBlock,
    Problem,
    ProblemDocument,
    PropulsionBlock,
    RadarBlock,
    RailBlock,
    RawStatement,
    ResetBlock,
    SearchBlock,
    Segment,
    SummarizeBlock,
    SurveyBlock,
    TangentBlock,
    TitleBlock,
    Trajectory,
    UnitsFormatBlock,
    WhenBlock,
    WindBlock,
)

_BLOCK_RE = re.compile(r"^\s*\*(?P<keyword>[A-Za-z0-9_/]+)\b(?P<header>.*)$")
_PROBLEM_RE = re.compile(r"^\s*\((?P<name>[^()]+)\)\s*$")
_ASSIGN_RE = re.compile(r"(?P<name>[A-Za-z_][A-Za-z0-9_./-]*(?:\[\d+\])?)\s*(?P<op><=|>=|==|!=|=|<|>)\s*(?P<value>\([^()]+\)|\*|[^\s,;]+)")


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


def _make_block(keyword: str, header: str, scope: str, path: str, line: int, diagnostics: list[Diagnostic]):
    common: dict[str, Any] = {"scope": scope, "location": _location(path, line), "header": header.strip(), "assignments": _assignments(header, path, line, diagnostics)}
    words = header.strip().split()
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
    elif keyword == "title":
        extra["title"] = header.strip()
    elif keyword == "define":
        extra["variable"] = words[0] if words else None
    elif keyword in {"egs", "file"}:
        extra["filename"] = words[0] if words else None
        extra["variables"] = words[1:] if len(words) > 1 else []
    elif keyword == "print":
        extra["variables"] = words
    elif keyword == "survey":
        if words and words[0].isdigit():
            extra["survey_id"] = int(words[0])
            extra["name"] = words[1] if len(words) > 1 else None
    elif keyword == "summarize":
        extra["name"] = words[0] if words else None
    elif keyword == "radar":
        extra["radar_id"] = int(words[0]) if words and words[0].isdigit() else None
    elif keyword == "wind":
        extra["coordinate_system"] = words[0] if words else None
    elif keyword == "fly":
        extra["guidance_variable"] = words[0].split("=", 1)[0] if words else None
    elif keyword == "rail":
        extra["mode"] = words[0] if words else None
    elif keyword == "initial":
        extra["mode"] = words[0] if words else None
        match = re.search(r"from\s+trajectory\s+(\d+)\s*,?\s*segment\s+(\d+)", header, re.IGNORECASE)
        if match:
            extra["source_trajectory"] = int(match.group(1))
            extra["source_segment"] = int(match.group(2))
        ####
    elif keyword == "optimize":
        match = re.search(r"^\s*([a-e])\s+for\s+(\S+)\s*=\s*(min|max)\s+on\s+segment\s+(\d+)(?:\s*,?\s*trajectory\s+(\d+))?", header, re.IGNORECASE)
        if match:
            extra.update(loop=match.group(1).lower(), objective_variable=match.group(2), objective_mode=match.group(3).lower(), segment=int(match.group(4)), trajectory=int(match.group(5)) if match.group(5) else None)
        ####
    elif keyword == "search":
        extra["search_id"] = int(words[0]) if words and words[0].isdigit() else None
    elif keyword == "when":
        match = re.match(r"\s*(.*?)\s+(goto\s+(\d+)|stop)\s*$", header, re.IGNORECASE)
        if match:
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
    lines = text.splitlines()
    for number, original in enumerate(lines, start=1):
        line = original.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        ####
        problem_match = _PROBLEM_RE.match(line)
        if problem_match:
            current_problem = Problem(name=problem_match.group("name").strip(), location=_location(path, number))
            document.problems.append(current_problem)
            current_trajectory = None
            current_segment = None
            current_block = None
            continue
        ####
        block_match = _BLOCK_RE.match(line)
        if block_match:
            keyword = block_match.group("keyword").lower()
            header = block_match.group("header").strip()
            if keyword == "end":
                if current_problem is not None:
                    current_problem.ended = True
                ####
                current_block = None
                continue
            ####
            if current_problem is None:
                document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="block-before-problem", message=f"Block '*{keyword}' appears before a problem name.", location=_location(path, number)))
                continue
            ####
            if keyword == "trajectory":
                match = re.match(r"(\d+)\s+(.*?)\s+start\s+on\s+(\d+)\s*$", header, re.IGNORECASE)
                if not match:
                    document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-trajectory-header", message="Expected '*trajectory N name start on S'.", location=_location(path, number)))
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
                    continue
                ####
                match = re.match(r"(\d+)(?:\s+(.*))?$", header)
                if not match:
                    document.diagnostics.append(Diagnostic(severity=Severity.ERROR, code="invalid-segment-header", message="Expected '*segment N [title]'.", location=_location(path, number)))
                    continue
                ####
                current_segment = Segment(number=int(match.group(1)), title=(match.group(2) or "").strip(), location=_location(path, number))
                current_trajectory.segments.append(current_segment)
                current_block = None
                continue
            ####
            problem_keywords = {"atmos", "earth", "egs", "optimize", "radar", "search", "summarize", "survey", "title", "units/fmt", "wind"}
            trajectory_keywords = {"dwn/crs", "iip", "initial", "tangent"}
            if keyword in problem_keywords:
                scope = "problem"
                current_segment = None
                current_trajectory = None
            elif keyword in trajectory_keywords:
                scope = "trajectory"
                current_segment = None
            elif keyword in {"define", "file", "print"}:
                scope = "segment" if current_segment is not None else "trajectory" if current_trajectory is not None else "problem"
            else:
                scope = "segment" if current_segment is not None else "trajectory" if current_trajectory is not None else "problem"
            ####
            block = _make_block(keyword, header, scope, path, number, document.diagnostics)
            if block is None:
                continue
            ####
            current_block = block
            if scope == "segment":
                current_segment.blocks.append(block)
            elif scope == "trajectory":
                current_trajectory.blocks.append(block)
            else:
                current_problem.blocks.append(block)
            ####
            continue
        ####
        if current_block is not None:
            current_block.statements.append(RawStatement(text=line.strip(), location=_location(path, number)))
            current_block.assignments.extend(_assignments(line, path, number, document.diagnostics))
        elif current_problem is not None and current_problem.blocks and isinstance(current_problem.blocks[-1], TitleBlock):
            current_problem.blocks[-1].title += "\n" + line.strip()
        else:
            document.diagnostics.append(Diagnostic(severity=Severity.WARNING, code="orphan-line", message=f"Line is not attached to a block: {line.strip()!r}", location=_location(path, number)))
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
