"""File-oriented TAOS runtime entrypoint."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

from taoryx.language.diagnostics import Diagnostic, Severity, SourceLocation
from taoryx.language.ingest import FileKind, ingest_file
from taoryx.language.models import ProblemDocument, TableDocument
from taoryx.language.table_parser import table_type_catalog

from .engine import ExecutionResult
from .lowering import execute_lowered, lower_problem_document, lower_tables


@dataclass(frozen=True, slots=True)
class RunReport:
    """Machine-readable result of one file-oriented runtime invocation."""

    problem: str
    tables: tuple[str, ...]
    cases: int
    results: tuple[ExecutionResult, ...]
    diagnostics: tuple[Diagnostic, ...]
    outputs: tuple[str, ...]

    @property
    def exit_code(self) -> int:
        if any(item.severity is Severity.ERROR for item in self.diagnostics):
            return 2
        if any(not result.completed and result.stop_reason == "max_steps" for result in self.results):
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
            "exit_code": self.exit_code,
        }
    ####
####


def run_files(problem_path: str | Path, table_paths: tuple[str | Path, ...] = (), *, output_dir: str | Path = ".", max_steps: int = 100000) -> RunReport:
    """Ingest, lower, execute, and write products for one `.prb` file."""

    problem = Path(problem_path)
    diagnostics: list[Diagnostic] = []
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
        problem_ingested = ingest_file(problem, available_tables={name for document in table_documents for name in table_type_catalog(document)})
        problem_document = cast(ProblemDocument, problem_ingested.document)
        diagnostics.extend(problem_ingested.diagnostics)
        if problem_ingested.kind is not FileKind.PROBLEM:
            diagnostics.append(_error(problem, "expected-problem-file", "runtime problem argument must use a .prb file"))
    except (OSError, UnicodeError, ValueError) as error:
        diagnostics.append(_error(problem, "problem-ingest-failed", str(error)))
        return RunReport(str(problem), tuple(map(str, table_paths)), 0, (), tuple(diagnostics), ())
    ####
    if any(item.severity is Severity.ERROR for item in diagnostics):
        return RunReport(str(problem), tuple(map(str, table_paths)), 0, (), tuple(diagnostics), ())
    try:
        tables = {name: table for document in table_documents for name, table in lower_tables(document).items()}
        lowered = lower_problem_document(problem_document, tables)
        if lowered.unsupported_features:
            for feature in lowered.unsupported_features:
                diagnostics.append(_error(problem, "unsupported-runtime-feature", f"runtime execution does not yet implement *{feature} semantics"))
            return RunReport(str(problem), tuple(map(str, table_paths)), len(lowered.cases), (), tuple(diagnostics), ())
        results = execute_lowered(lowered, output_dir=str(output_dir), max_steps=max_steps)
    except (KeyError, RuntimeError, TypeError, ValueError) as error:
        diagnostics.append(_error(problem, "runtime-execution-failed", str(error)))
        return RunReport(str(problem), tuple(map(str, table_paths)), 0, (), tuple(diagnostics), ())
    ####
    outputs = tuple(str(path) for path in Path(output_dir).iterdir() if path.is_file())
    return RunReport(str(problem), tuple(map(str, table_paths)), len(lowered.cases), results, tuple(diagnostics), outputs)
####


def _error(path: str | Path, code: str, message: str) -> Diagnostic:
    return Diagnostic(severity=Severity.ERROR, code=code, message=message, location=SourceLocation(path=str(path), line=1))
####
