"""File-oriented TAOS runtime entrypoint."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

from taoryx.language.diagnostics import Diagnostic, Severity, SourceLocation
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.ingest import FileKind, ingest_file
from taoryx.language.models import ProblemDocument, TableDocument
from taoryx.outputs import RunArtifact, build_run_artifact

from .engine import ExecutionResult
from .lowering import execute_lowered, lower_problem_document, problem_unit_settings
from .table_binding import bind_runtime_tables


@dataclass(frozen=True, slots=True)
class RunReport:
    """Machine-readable result of one file-oriented runtime invocation."""

    problem: str
    tables: tuple[str, ...]
    cases: int
    results: tuple[ExecutionResult, ...]
    diagnostics: tuple[Diagnostic, ...]
    outputs: tuple[str, ...]
    artifacts: tuple[RunArtifact, ...] = ()
    metadata: tuple[dict[str, object], ...] = ()

    @property
    def exit_code(self) -> int:
        if any(item.severity is Severity.ERROR for item in self.diagnostics):
            return 2
        if any(not result.completed for result in self.results):
            return 1
        return 0
    ####

    def as_dict(self) -> dict[str, Any]:
        return {
            "problem": self.problem,
            "tables": list(self.tables),
            "cases": self.cases,
            "results": [{"completed": result.completed, "stop_reason": result.stop_reason, "vehicles": list(result.states)} for result in self.results],
            "diagnostics": [item.model_dump(mode="json") for item in self.diagnostics],
            "outputs": list(self.outputs),
            "artifacts": [artifact.model_dump(mode="json") for artifact in self.artifacts],
            "metadata": list(self.metadata),
            "exit_code": self.exit_code,
        }
    ####
####


def run_files(
    problem_path: str | Path,
    table_paths: tuple[str | Path, ...] = (),
    *,
    output_dir: str | Path = ".",
    max_steps: int = 100000,
    integrator: str | None = None,
    seed: int | None = None,
    profile: GrammarProfile | str = GrammarProfile.TAOS96,
) -> RunReport:
    """Ingest, lower, execute, and write products for one `.prb` file."""

    problem = Path(problem_path)
    destination = Path(output_dir)
    diagnostics: list[Diagnostic] = []
    try:
        destination.mkdir(parents=True, exist_ok=True)
        before_outputs = {path: path.stat().st_mtime_ns for path in destination.iterdir() if path.is_file()}
    except OSError as error:
        diagnostics.append(_error(destination, "output-dir-failed", str(error)))
        return RunReport(str(problem), tuple(map(str, table_paths)), 0, (), tuple(diagnostics), ())
    ####
    table_documents: list[TableDocument] = []
    for path in table_paths:
        try:
            ingested = ingest_file(path)
            diagnostics.extend(ingested.diagnostics)
            if ingested.kind is not FileKind.TABLE:
                diagnostics.append(_error(path, "expected-table-file", "runtime table arguments must use .tbl files"))
            else:
                table_documents.append(cast(TableDocument, ingested.document))
        except (OSError, UnicodeError, ValueError) as error:
            diagnostics.append(_error(path, "table-ingest-failed", str(error)))
    ####
    try:
        available_tables, tables = bind_runtime_tables(table_documents, {})
    except (KeyError, TypeError, ValueError) as error:
        diagnostics.append(_error(problem, "runtime-table-binding-failed", str(error)))
        return RunReport(str(problem), tuple(map(str, table_paths)), 0, (), tuple(diagnostics), ())
    ####
    try:
        problem_ingested = ingest_file(
            problem,
            available_tables=available_tables,
            profile=profile,
        )
        problem_document = cast(ProblemDocument, problem_ingested.document)
        diagnostics.extend(_runtime_diagnostics(problem_ingested.diagnostics))
        if problem_ingested.kind is not FileKind.PROBLEM:
            diagnostics.append(_error(problem, "expected-problem-file", "runtime problem argument must use a .prb file"))
    except (OSError, UnicodeError, ValueError) as error:
        diagnostics.append(_error(problem, "problem-ingest-failed", str(error)))
        return RunReport(str(problem), tuple(map(str, table_paths)), 0, (), tuple(diagnostics), ())
    ####
    if any(item.severity is Severity.ERROR for item in diagnostics):
        return RunReport(str(problem), tuple(map(str, table_paths)), 0, (), tuple(diagnostics), ())
    try:
        unit_settings, _ = problem_unit_settings(problem_document)
        available_tables, tables = bind_runtime_tables(table_documents, unit_settings)
        lowered = lower_problem_document(problem_document, tables, seed=seed)
        unsafe_output = _unsafe_output_path(lowered, destination)
        if unsafe_output is not None:
            diagnostics.append(_error(problem, "unsafe-output-path", unsafe_output))
            return RunReport(str(problem), tuple(map(str, table_paths)), len(lowered.cases), (), tuple(diagnostics), ())
        if lowered.unsupported_features:
            for feature in lowered.unsupported_features:
                diagnostics.append(_error(problem, "unsupported-runtime-feature", f"runtime execution does not yet implement *{feature} semantics"))
            return RunReport(str(problem), tuple(map(str, table_paths)), len(lowered.cases), (), tuple(diagnostics), ())
        results = execute_lowered(lowered, output_dir=str(destination), max_steps=max_steps, integrator=integrator)
        incomplete = tuple(result.stop_reason for result in results if not result.completed)
        if incomplete:
            diagnostics.append(
                _warning(
                    problem,
                    "runtime-incomplete",
                    f"{len(incomplete)} case(s) reached the runtime step limit ({max_steps}) before completion",
                )
            )
    except (KeyError, OSError, RuntimeError, TypeError, ValueError) as error:
        diagnostics.append(_error(problem, "runtime-execution-failed", str(error)))
        case_count = len(lowered.cases) if "lowered" in locals() else 0
        return RunReport(str(problem), tuple(map(str, table_paths)), case_count, (), tuple(diagnostics), ())
    ####
    try:
        outputs = tuple(
            str(path)
            for path in sorted(destination.iterdir(), key=lambda item: item.name)
            if path.is_file() and (path not in before_outputs or path.stat().st_mtime_ns != before_outputs[path])
        )
    except OSError as error:
        diagnostics.append(_error(destination, "output-discovery-failed", str(error)))
        return RunReport(str(problem), tuple(map(str, table_paths)), len(lowered.cases), results, tuple(diagnostics), ())
    ####
    artifacts = tuple(
        build_run_artifact(
            str(problem),
            case.problem,
            result,
            vehicle_kinds={vehicle_id: vehicle.vehicle_kind for vehicle_id, vehicle in case.problem.vehicles.items()},
            events=case.problem.event_history,
        )
        for case, result in zip(lowered.cases, results, strict=True)
    )
    metadata = tuple(
        {
            key: value
            for key, value in case.problem.metadata.items()
            if key in {"dynamics_mode", "parameters", "vehicle", "actuator", "target", "route", "controls", "thermal", "telemetry", "native_pipeline"}
        }
        for case in lowered.cases
    )
    return RunReport(str(problem), tuple(map(str, table_paths)), len(lowered.cases), results, tuple(diagnostics), outputs, artifacts, metadata)
####


def _error(path: str | Path, code: str, message: str) -> Diagnostic:
    return Diagnostic(severity=Severity.ERROR, code=code, message=message, location=SourceLocation(path=str(path), line=1))
####


def _warning(path: str | Path, code: str, message: str) -> Diagnostic:
    return Diagnostic(severity=Severity.WARNING, code=code, message=message, location=SourceLocation(path=str(path), line=1))


def _runtime_diagnostics(diagnostics: tuple[Diagnostic, ...]) -> tuple[Diagnostic, ...]:
    """Turn unresolved table references into execution-blocking diagnostics."""

    resolved: list[Diagnostic] = []
    for diagnostic in diagnostics:
        if diagnostic.code != "external-table-reference":
            resolved.append(diagnostic)
            continue
        resolved.append(
            diagnostic.model_copy(
                update={
                    "severity": Severity.ERROR,
                    "code": "missing-runtime-table",
                    "message": diagnostic.message.replace(
                        "is not present in the supplied table set",
                        "is required by runtime execution but is not present in the supplied table set",
                    ),
                }
            )
        )
    return tuple(resolved)
####
####


def _unsafe_output_path(document: object, destination: Path) -> str | None:
    """Reject declared products that would write outside the output directory."""

    root = destination.resolve()
    filenames: list[str] = []
    for filename, _, _, _ in getattr(document, "output_files", ()):
        filenames.append(filename)
    for filename, _, _ in getattr(document, "summary_egs_files", ()):
        filenames.append(filename)
    for filename in filenames:
        candidate = (destination / filename).resolve()
        try:
            candidate.relative_to(root)
        except ValueError:
            return f"declared output {filename!r} is outside the selected output directory"
    return None
####
