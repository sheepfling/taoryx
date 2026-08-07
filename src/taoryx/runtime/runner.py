"""File-oriented TAOS runtime entrypoint."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, cast

from taoryx.language.diagnostics import Diagnostic, Severity, SourceLocation
from taoryx.language.expressions import NumberExpression
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.ingest import FileKind, ingest_file
from taoryx.language.models import Assignment, EarthBlock, ProblemDocument, TableDocument
from taoryx.outputs import RunArtifact, build_run_artifact
from taoryx.simulation_runtime_contracts import SimulationRuntimeStatus, classify_runtime_outcome

from .engine import ExecutionResult
from .lowering import LoweredDocument, execute_lowered, lower_problem_document, problem_unit_settings
from .sensor_scenario import SensorScenarioRuntime, SensorScenarioSpec, attach_sensor_scenario, render_sensor_scenario_plots
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
    def status(self) -> SimulationRuntimeStatus:
        """Return the closed Simulation Runtime outcome status for this source run."""

        return classify_runtime_outcome(
            has_errors=any(item.severity is Severity.ERROR for item in self.diagnostics),
            execution_started=bool(self.results),
            completed=all(result.completed for result in self.results),
        )
    ####

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
            "status": self.status.value,
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
    sensor_spec: str | Path | SensorScenarioSpec | None = None,
    control_provenance: Literal["none", "summary", "intervals", "full"] = "none",
) -> RunReport:
    """Ingest, lower, execute, and write products for one `.prb` file.

    ``sensor_spec`` is an explicit optional sidecar binding. Omitting it keeps
    the historical clock-only and unsensorized execution path unchanged.

    ``control_provenance`` optionally writes an accepted-interval ledger from
    the native runtime.  It is deliberately not a Mission Composition semantic-action
    trace: solver-stage controller mutation makes an interval ineligible for a
    held-command claim.  ``summary`` exposes only counts and the claim
    boundary; ``intervals`` retains accepted-interval records without
    solver-stage detail; ``full`` also retains every underlying control
    evaluation for diagnosis.
    """

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
        resolved_sensor_spec: SensorScenarioSpec | None
        if sensor_spec is None:
            resolved_sensor_spec = None
        elif isinstance(sensor_spec, SensorScenarioSpec):
            resolved_sensor_spec = sensor_spec
        else:
            resolved_sensor_spec = SensorScenarioSpec.from_file(sensor_spec)
        lowered_document = _apply_sensor_earth_rate(problem_document, resolved_sensor_spec)
        unit_settings, _ = problem_unit_settings(lowered_document)
        available_tables, tables = bind_runtime_tables(table_documents, unit_settings)
        lowered = lower_problem_document(lowered_document, tables, seed=seed)
        sensor_runtimes: list[SensorScenarioRuntime | None] = []
        if resolved_sensor_spec is not None:
            for case in lowered.cases:
                attached_runtime = attach_sensor_scenario(case.problem, resolved_sensor_spec)
                attached_runtime.source_inputs = _sensor_source_inputs(problem, table_paths, resolved_sensor_spec)
                case.problem.metadata["sensor_scenario"] = {
                    "scenario_id": resolved_sensor_spec.scenario_id,
                    "source_inputs": dict(attached_runtime.source_inputs),
                }
                sensor_runtimes.append(attached_runtime)
        else:
            sensor_runtimes = [None for _ in lowered.cases]
        sensor_outputs: list[str] = []
        unsafe_output = _unsafe_output_path(lowered, destination)
        if unsafe_output is not None:
            diagnostics.append(_error(problem, "unsafe-output-path", unsafe_output))
            return RunReport(str(problem), tuple(map(str, table_paths)), len(lowered.cases), (), tuple(diagnostics), ())
        if lowered.unsupported_features:
            for feature in lowered.unsupported_features:
                diagnostics.append(_error(problem, "unsupported-runtime-feature", f"runtime execution does not yet implement *{feature} semantics"))
            return RunReport(str(problem), tuple(map(str, table_paths)), len(lowered.cases), (), tuple(diagnostics), ())
        results = execute_lowered(lowered, output_dir=str(destination), max_steps=max_steps, integrator=integrator)
        if control_provenance != "none":
            _write_control_provenance(destination / "control_provenance.json", lowered, detail=control_provenance)
        incomplete = tuple(result.stop_reason for result in results if not result.completed)
        if incomplete:
            diagnostics.append(
                _warning(
                    problem,
                    "runtime-incomplete",
                    f"{len(incomplete)} case(s) reached the runtime step limit ({max_steps}) before completion",
                )
            )
        for case, runtime, result in zip(lowered.cases, sensor_runtimes, results, strict=True):
            if runtime is not None:
                runtime.finalize(completed=result.completed, stop_reason=result.stop_reason, max_steps=max_steps)
                sensor_outputs.extend(str(path) for path in runtime.write_artifacts(destination, case.index))
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
    artifact_values: list[RunArtifact] = []
    for index, (case, result) in enumerate(zip(lowered.cases, results, strict=True)):
        base_artifact = build_run_artifact(
            str(problem),
            case.problem,
            result,
            vehicle_kinds={vehicle_id: vehicle.vehicle_kind for vehicle_id, vehicle in case.problem.vehicles.items()},
            events=case.problem.event_history,
        )
        runtime = sensor_runtimes[index]
        if runtime is None:
            artifact_values.append(base_artifact)
            continue
        execution = runtime.artifact()
        plot_directory = destination / "sensor" / f"case-{case.index}" / "plots"
        plot_manifest = render_sensor_scenario_plots(base_artifact, execution, plot_directory)
        runtime.plot_manifest = {
            "manifest_path": "plots/plot-manifest.json",
            "plots": plot_manifest.get("plots", []),
        }
        runtime.write_manifest(destination, case.index)
        sensor_outputs.extend(str(path) for path in sorted(plot_directory.iterdir()) if path.is_file())
        artifact_values.append(base_artifact.model_copy(update={"sensor_execution": runtime.artifact()}))
    outputs = outputs + tuple(sorted(set(sensor_outputs)))
    artifacts = tuple(artifact_values)
    metadata = tuple(
        {
            key: value
            for key, value in case.problem.metadata.items()
            if key in {
                "dynamics_mode",
                "parameters",
                "vehicle",
                "actuator",
                "target",
                "route",
                "controls",
                "thermal",
                "telemetry",
                "native_pipeline",
                "lqr",
                "lqr_by_role",
                "controller_realization",
                "controller_realizations",
                "sensor_execution",
                "sensor_scenario",
                "runtime_model_bindings",
            }
        }
        for case in lowered.cases
    )
    return RunReport(str(problem), tuple(map(str, table_paths)), len(lowered.cases), results, tuple(diagnostics), outputs, artifacts, metadata)
####


def _write_control_provenance(
    destination: Path,
    lowered: LoweredDocument,
    *,
    detail: Literal["summary", "intervals", "full"],
) -> None:
    """Write native control-evaluation evidence without overstating it.

    This intentionally lives beside runtime capabilities rather than the
    composition action-trace artifact.  A public semantic trace requires a
    controller to resolve commands at accepted truth boundaries and hold those
    commands across every solver stage.  Resolver-owned native commands now
    have that runtime guarantee, while legacy mutable controls remain
    diagnostic-only.  The artifact is useful for making that distinction
    visible and for demonstrating exactly why a trace is withheld.
    """

    cases: list[dict[str, object]] = []
    for case in lowered.cases:
        vehicles: dict[str, object] = {}
        for vehicle_id, vehicle in sorted(case.problem.vehicles.items()):
            intervals = vehicle.control_interval_history
            evaluations = vehicle.control_evaluation_history
            solver_stage_mutation_count = sum(item.solver_stage_control_mutation_detected for item in intervals)
            summary: dict[str, object] = {
                "accepted_interval_count": len(intervals),
                "solver_stage_mutation_interval_count": solver_stage_mutation_count,
                "held_command_eligible_interval_count": len(intervals) - solver_stage_mutation_count,
                "control_evaluation_count": len(evaluations),
                "solver_stage_evaluation_count": sum(item.phase == "solver_stage" for item in evaluations),
                "committed_truth_evaluation_count": sum(item.phase == "committed_truth" for item in evaluations),
                "declared_native_control_names": sorted(vehicle.control_values),
                "committed_resolver_control_names": sorted(vehicle.committed_control_names),
            }
            if detail in {"intervals", "full"}:
                summary["control_interval_records"] = [item.as_dict() for item in intervals]
            if detail == "full":
                summary["control_evaluation_records"] = [item.as_dict() for item in evaluations]
            vehicles[vehicle_id] = summary
        cases.append({"case_index": case.index, "vehicles": vehicles})
    payload = {
        "schema": "taoryx.runtime-control-provenance/v1alpha1",
        "detail": detail,
        "cases": cases,
        "claim_boundary": (
            "This artifact preserves native runtime control-evaluation and accepted-interval provenance. "
            "It is not a semantic_action_trace and does not establish that a command was resolved at an "
            "accepted truth boundary and held unchanged throughout its interval."
        ),
    }
    destination.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
####


def _error(path: str | Path, code: str, message: str) -> Diagnostic:
    return Diagnostic(severity=Severity.ERROR, code=code, message=message, location=SourceLocation(path=str(path), line=1))
####


def _warning(path: str | Path, code: str, message: str) -> Diagnostic:
    return Diagnostic(severity=Severity.WARNING, code=code, message=message, location=SourceLocation(path=str(path), line=1))


def _sensor_artifact(runtime: SensorScenarioRuntime | None) -> dict[str, object]:
    return {} if runtime is None else runtime.artifact()


def _sensor_source_inputs(problem: Path, table_paths: tuple[str | Path, ...], spec: SensorScenarioSpec) -> dict[str, object]:
    profile = spec.profile_path
    packaged_profile = None
    if spec.profile_name is not None:
        packaged_profile = {
            "package": f"{spec.profile_category}/{spec.profile_name}",
            "package_name": "imu-error-model",
            "package_version": "0.1.3",
        }
    return {
        "problem": {"path": str(problem.resolve()), "sha256": _file_sha256(problem)},
        "tables": [{"path": str(Path(path).resolve()), "sha256": _file_sha256(Path(path))} for path in table_paths],
        "sidecar": None if spec.source_path is None else {"path": str(spec.source_path), "sha256": _file_sha256(spec.source_path)},
        "profile": packaged_profile if packaged_profile is not None else None if profile is None else {"path": str(profile), "sha256": _file_sha256(profile)},
    }


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _apply_sensor_earth_rate(document: ProblemDocument, spec: SensorScenarioSpec | None) -> ProblemDocument:
    """Apply only an explicit sensor-side Earth-rate override to a copy."""

    if spec is None or spec.earth_rate_mode == "source":
        return document
    omega = 7.2921151467e-5 if spec.earth_rate_mode == "nominal" else spec.earth_omega_rad_s
    if omega is None:
        raise ValueError("sensor Earth-rate override did not resolve to a finite value")
    copied = document.model_copy(deep=True)
    for problem in copied.problems:
        for block in problem.blocks:
            if not isinstance(block, EarthBlock):
                continue
            assignment = Assignment(name="omega", value=NumberExpression(value=omega), location=block.location)
            existing = next((index for index, item in enumerate(block.assignments) if item.name.casefold() == "omega"), None)
            if existing is None:
                block.assignments.append(assignment)
            else:
                block.assignments[existing] = assignment
    return copied


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
