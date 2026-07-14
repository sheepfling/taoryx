from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True, slots=True)
class SpectreSegmentSpec:
    """A rendered TAOS segment constructed from Spectre source metadata."""

    number: int
    title: str
    phase: str
    body: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class SpectreProblemSpec:
    """A synthetic TAOS problem-file translation for one Spectre family."""

    path: str
    problem_name: str
    title: str
    source_bundle: str
    comments: tuple[str, ...]
    atmosphere: str
    earth: str
    trajectory_line: str
    initial_line: str
    file_line: str
    segments: tuple[SpectreSegmentSpec, ...]


@dataclass(frozen=True, slots=True)
class SpectreWorkspaceSpec:
    """A workspace-level bundle of generated Spectre problem files."""

    schema_version: int
    source_bundle: str
    translation_style: str
    problems: tuple[SpectreProblemSpec, ...]


@dataclass(frozen=True, slots=True)
class SpectrePhaseCatalogSpec:
    """A phase bucket in the higher-level Spectre family index."""

    name: str
    description: str
    introduced_controls: tuple[str, ...]
    families: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class SpectreFamilyCatalogSpec:
    """A family entry in the higher-level Spectre family index."""

    family: str
    path: str
    phase: str
    problem_name: str
    source_bundle: str
    source_snippet: str
    source_controls: tuple[str, ...]
    stop_controls: tuple[str, ...]
    notes: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class SpectreCatalogSpec:
    """A machine-readable index of phase buckets and family metadata."""

    schema_version: int
    source_bundle: str
    translation_style: str
    description: str
    phases: tuple[SpectrePhaseCatalogSpec, ...]
    families: tuple[SpectreFamilyCatalogSpec, ...]


def _load_mapping(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise TypeError(f"expected mapping at {path}")
    ####
    return payload
####


def load_spec(path: Path) -> SpectreWorkspaceSpec:
    """Load a Spectre dump workspace specification from YAML."""

    payload = _load_mapping(path)
    problems: list[SpectreProblemSpec] = []
    for item in payload.get("problems", []):
        if not isinstance(item, dict):
            raise TypeError(f"problem entry must be a mapping: {item!r}")
        ####
        segments = tuple(
            SpectreSegmentSpec(
                number=int(segment["number"]),
                title=str(segment["title"]),
                phase=str(segment["phase"]),
                body=tuple(str(line) for line in segment.get("body", [])),
            )
            for segment in item.get("segments", [])
        )
        problems.append(
            SpectreProblemSpec(
                path=str(item["path"]),
                problem_name=str(item["problem_name"]),
                title=str(item["title"]),
                source_bundle=str(item["source_bundle"]),
                comments=tuple(str(line) for line in item.get("comments", [])),
                atmosphere=str(item["atmosphere"]),
                earth=str(item["earth"]),
                trajectory_line=str(item["trajectory_line"]),
                initial_line=str(item["initial_line"]),
                file_line=str(item["file_line"]),
                segments=segments,
            )
        )
    ####
    return SpectreWorkspaceSpec(
        schema_version=int(payload["schema_version"]),
        source_bundle=str(payload["source_bundle"]),
        translation_style=str(payload["translation_style"]),
        problems=tuple(problems),
    )
####


def load_catalog(path: Path) -> SpectreCatalogSpec:
    """Load a higher-level Spectre family catalog from YAML."""

    payload = _load_mapping(path)
    phases = tuple(
        SpectrePhaseCatalogSpec(
            name=str(item["name"]),
            description=str(item["description"]),
            introduced_controls=tuple(str(line) for line in item.get("introduced_controls", [])),
            families=tuple(str(name) for name in item.get("families", [])),
        )
        for item in payload.get("phases", [])
    )
    families = tuple(
        SpectreFamilyCatalogSpec(
            family=str(item["family"]),
            path=str(item["path"]),
            phase=str(item["phase"]),
            problem_name=str(item["problem_name"]),
            source_bundle=str(item["source_bundle"]),
            source_snippet=str(item["source_snippet"]),
            source_controls=tuple(str(line) for line in item.get("source_controls", [])),
            stop_controls=tuple(str(line) for line in item.get("stop_controls", [])),
            notes=tuple(str(line) for line in item.get("notes", [])),
        )
        for item in payload.get("families", [])
    )
    return SpectreCatalogSpec(
        schema_version=int(payload["schema_version"]),
        source_bundle=str(payload["source_bundle"]),
        translation_style=str(payload["translation_style"]),
        description=str(payload["description"]),
        phases=tuple(phases),
        families=tuple(families),
    )
####


def group_families_by_phase(catalog: SpectreCatalogSpec) -> dict[str, tuple[SpectreFamilyCatalogSpec, ...]]:
    """Group cataloged families by their inferred phase bucket."""

    buckets: dict[str, list[SpectreFamilyCatalogSpec]] = {}
    for family in catalog.families:
        buckets.setdefault(family.phase, []).append(family)
    ####
    return {phase: tuple(families) for phase, families in buckets.items()}
####


def validate_catalog_against_workspace(catalog: SpectreCatalogSpec, workspace: SpectreWorkspaceSpec) -> None:
    """Check that the phase catalog is aligned with the rendered workspace."""

    rendered_paths = [problem.path for problem in workspace.problems]
    catalog_paths = [family.path for family in catalog.families]
    if rendered_paths != catalog_paths:
        raise ValueError(
            f"catalog workspace mismatch: {catalog_paths!r} != {rendered_paths!r}"
        )
    ####
    rendered_by_name = {Path(problem.path).stem: problem for problem in workspace.problems}
    for family in catalog.families:
        problem = rendered_by_name.get(family.family)
        if problem is None:
            raise ValueError(f"catalog family not rendered: {family.family}")
        ####
        if problem.problem_name != family.problem_name:
            raise ValueError(
                f"catalog problem name mismatch for {family.family}: "
                f"{family.problem_name!r} != {problem.problem_name!r}"
            )
        ####
        if not problem.segments:
            raise ValueError(f"workspace problem has no segments: {family.family}")
        ####
        if problem.segments[0].phase != family.phase:
            raise ValueError(
                f"catalog phase mismatch for {family.family}: "
                f"{family.phase!r} != {problem.segments[0].phase!r}"
            )
        ####
    ####
####


def render_problem_file(spec: SpectreProblemSpec) -> str:
    """Render one synthetic TAOS problem file."""

    lines: list[str] = [f"({spec.problem_name})", ""]
    for line in spec.comments:
        lines.append(f"# {line}")
    if spec.comments:
        lines.append("")
    lines.extend(
        [
            f"*title {spec.title}",
            "",
            f"*atmos {spec.atmosphere}",
            f"*earth {spec.earth}",
            "",
            spec.trajectory_line,
            f"  {spec.initial_line}",
            f"  {spec.file_line}",
        ]
    )
    for segment in spec.segments:
        lines.append(f"  *segment {segment.number} {segment.title}")
        for body_line in segment.body:
            lines.append(f"    {body_line}")
    lines.extend(["", "*end", ""])
    ####
    return "\n".join(lines)
####


def render_workspace(spec: SpectreWorkspaceSpec) -> dict[str, str]:
    """Render every problem file in a workspace specification."""

    return {problem.path: render_problem_file(problem) for problem in spec.problems}
####


def write_workspace(spec: SpectreWorkspaceSpec, destination: Path) -> None:
    """Write a rendered workspace to disk."""

    destination.mkdir(parents=True, exist_ok=True)
    for path, text in render_workspace(spec).items():
        target = destination / path
        target.write_text(text, encoding="utf-8")
    ####
####
