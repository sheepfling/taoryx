from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any

import yaml

from .simple_aero_dumps import SimpleAeroSegmentSpec


@dataclass(frozen=True, slots=True)
class SimpleAeroTrajectorySpec:
    """A rendered TAOS trajectory constructed from SimpleAero source metadata."""

    trajectory_line: str
    initial_line: str
    file_line: str
    segments: tuple[SimpleAeroSegmentSpec, ...]


@dataclass(frozen=True, slots=True)
class SimpleAeroSolutionMetadata:
    """Structured SimpleAero solution metadata preserved for traceability."""

    family: str
    direction: str | None = None
    maneuver_altitude_start_km: float | None = None
    maneuver_duration_s: float | None = None
    min_time_to_go_s: float | None = None
    initial_heading_error_deg: float | None = None
    maneuver_begin_time_to_go_s: float | None = None
    start_range_to_go_km: float | None = None
    end_range_to_go_km: float | None = None
    weave_end_range_to_go_km: float | None = None
    phugoid_amplitude_deg: float | None = None
    phugoid_frequency_Hz: float | None = None
    maneuver_roll_deg: float | None = None
    terminal_handoff_range_km: float | None = None
    notes: tuple[str, ...] = ()
####


@dataclass(frozen=True, slots=True)
class SimpleAeroTrajectoryProblemSpec:
    """A synthetic TAOS problem-file translation with one or more trajectories."""

    path: str
    problem_name: str
    title: str
    source_bundle: str
    comments: tuple[str, ...]
    simple_aero_metadata: SimpleAeroSolutionMetadata
    atmosphere: str
    earth: str
    trajectories: tuple[SimpleAeroTrajectorySpec, ...]


@dataclass(frozen=True, slots=True)
class SimpleAeroTrajectoryWorkspaceSpec:
    """A workspace-level bundle of generated SimpleAero trajectory problem files."""

    schema_version: int
    source_bundle: str
    translation_style: str
    problems: tuple[SimpleAeroTrajectoryProblemSpec, ...]


def _load_mapping(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise TypeError(f"expected mapping at {path}")
    ####
    return payload
####


def load_spec(path: Path) -> SimpleAeroTrajectoryWorkspaceSpec:
    """Load a SimpleAero trajectory workspace specification from YAML."""

    payload = _load_mapping(path)
    problems: list[SimpleAeroTrajectoryProblemSpec] = []
    for item in payload.get("problems", []):
        if not isinstance(item, dict):
            raise TypeError(f"problem entry must be a mapping: {item!r}")
        ####
        trajectories = tuple(
            SimpleAeroTrajectorySpec(
                trajectory_line=str(trajectory["trajectory_line"]),
                initial_line=str(trajectory["initial_line"]),
                file_line=str(trajectory["file_line"]),
                segments=tuple(
                    SimpleAeroSegmentSpec(
                        number=int(segment["number"]),
                        title=str(segment["title"]),
                        phase=str(segment["phase"]),
                        body=tuple(str(line) for line in segment.get("body", [])),
                    )
                    for segment in trajectory.get("segments", [])
                ),
            )
            for trajectory in item.get("trajectories", [])
        )
        problems.append(
            SimpleAeroTrajectoryProblemSpec(
                path=str(item["path"]),
                problem_name=str(item["problem_name"]),
                title=str(item["title"]),
                source_bundle=str(item["source_bundle"]),
                comments=tuple(str(line) for line in item.get("comments", [])),
                simple_aero_metadata=_load_simple_aero_metadata(item.get("simple_aero_metadata", {}), path=path),
                atmosphere=str(item["atmosphere"]),
                earth=str(item["earth"]),
                trajectories=trajectories,
            )
        )
    ####
    return SimpleAeroTrajectoryWorkspaceSpec(
        schema_version=int(payload["schema_version"]),
        source_bundle=str(payload["source_bundle"]),
        translation_style=str(payload["translation_style"]),
        problems=tuple(problems),
    )
####


def render_problem_file(spec: SimpleAeroTrajectoryProblemSpec) -> str:
    """Render one synthetic TAOS trajectory problem file."""

    lines: list[str] = [f"({spec.problem_name})", ""]
    for line in spec.comments:
        lines.append(f"# {line}")
    if spec.comments:
        lines.append("")
    if spec.simple_aero_metadata:
        lines.extend(_render_metadata_comments(spec.simple_aero_metadata))
        lines.append("")
    lines.extend(
        [
            f"*title {spec.title}",
            "",
            f"*atmos {spec.atmosphere}",
            f"*earth {spec.earth}",
            "",
        ]
    )
    for trajectory in spec.trajectories:
        lines.append(trajectory.trajectory_line)
        lines.append("")
        lines.append(f"  {trajectory.initial_line}")
        lines.append("")
        lines.append(f"  {trajectory.file_line}")
        for segment in trajectory.segments:
            lines.append(f"  *segment {segment.number} {segment.title}")
            for body_line in segment.body:
                lines.append(f"    {body_line}")
        lines.append("")
    lines.extend(["*end", ""])
    ####
    return "\n".join(lines)
####


def _load_simple_aero_metadata(value: Any, *, path: Path) -> SimpleAeroSolutionMetadata:
    if value is None:
        return SimpleAeroSolutionMetadata(family="unknown")
    ####
    if not isinstance(value, dict):
        raise TypeError(f"simple_aero_metadata must be a mapping at {path}")
    ####
    return SimpleAeroSolutionMetadata(
        family=str(value["family"]),
        direction=_optional_str(value.get("direction")),
        maneuver_altitude_start_km=_optional_float(value.get("maneuver_altitude_start_km")),
        maneuver_duration_s=_optional_float(value.get("maneuver_duration_s")),
        min_time_to_go_s=_optional_float(value.get("min_time_to_go_s")),
        initial_heading_error_deg=_optional_float(value.get("initial_heading_error_deg")),
        maneuver_begin_time_to_go_s=_optional_float(value.get("maneuver_begin_time_to_go_s")),
        start_range_to_go_km=_optional_float(value.get("start_range_to_go_km")),
        end_range_to_go_km=_optional_float(value.get("end_range_to_go_km")),
        weave_end_range_to_go_km=_optional_float(value.get("weave_end_range_to_go_km")),
        phugoid_amplitude_deg=_optional_float(value.get("phugoid_amplitude_deg")),
        phugoid_frequency_Hz=_optional_float(value.get("phugoid_frequency_Hz")),
        maneuver_roll_deg=_optional_float(value.get("maneuver_roll_deg")),
        terminal_handoff_range_km=_optional_float(value.get("terminal_handoff_range_km")),
        notes=tuple(str(line) for line in value.get("notes", [])),
    )
####


def _render_metadata_comments(metadata: SimpleAeroSolutionMetadata, *, indent: str = "") -> list[str]:
    lines: list[str] = [f"{indent}# Simple Aero metadata:"]
    for field in fields(metadata):
        key = field.name
        value = getattr(metadata, key)
        if value in (None, (), ""):
            continue
        lines.extend(_render_metadata_entry(key, value, indent=indent + "# "))
    ####
    return lines
####


def _render_metadata_entry(key: str, value: Any, *, indent: str) -> list[str]:
    if isinstance(value, dict):
        lines = [f"{indent}{key}:"]
        for child_key, child_value in value.items():
            lines.extend(_render_metadata_entry(child_key, child_value, indent=f"{indent}  "))
        return lines
    ####
    if isinstance(value, (list, tuple)):
        lines = [f"{indent}{key}:"]
        for item in value:
            lines.append(f"{indent}  - {item}")
        return lines
    ####
    return [f"{indent}{key}: {value}"]
####


def _optional_str(value: Any) -> str | None:
    if value is None:
        return None
    ####
    return str(value)
####


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    ####
    return float(value)
####


def render_workspace(spec: SimpleAeroTrajectoryWorkspaceSpec) -> dict[str, str]:
    """Render every trajectory problem file in a workspace specification."""

    return {problem.path: render_problem_file(problem) for problem in spec.problems}
####


def write_workspace(spec: SimpleAeroTrajectoryWorkspaceSpec, destination: Path) -> None:
    """Write a rendered trajectory workspace to disk."""

    destination.mkdir(parents=True, exist_ok=True)
    for path, text in render_workspace(spec).items():
        target = destination / path
        target.write_text(text, encoding="utf-8")
    ####
####
