"""Loaded source-program boundary for the TAORYX runtime emulator."""

from __future__ import annotations

import hashlib
import json
import math
import os
import tempfile
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import cast

from taoryx.contracts import Frame
from taoryx.language.diagnostics import Diagnostic, Severity
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.ingest import FileKind, ingest_file
from taoryx.language.models import ProblemDocument, RuntimeBlock, TableDocument, TitleBlock
from taoryx.language.table_parser import table_type_catalog

from .common import RuntimeProblem, RuntimeState
from .lowering import LoweredDocument, lower_problem_document, lower_tables, problem_unit_settings


class ProgramLoadError(ValueError):
    """Raised when source files cannot form an executable loaded program."""

    def __init__(self, diagnostics: Sequence[Diagnostic]) -> None:
        self.diagnostics = tuple(diagnostics)
        message = "; ".join(f"{item.code}: {item.message}" for item in self.diagnostics)
        super().__init__(message or "program loading failed")
    ####
####


@dataclass(frozen=True, slots=True)
class LoadedProgram:
    """Parsed source plus its lowered executable cases and table inventory."""

    problem_path: str
    table_paths: tuple[str, ...]
    profile: GrammarProfile
    source: ProblemDocument
    table_documents: tuple[TableDocument, ...]
    lowered: LoweredDocument
    diagnostics: tuple[Diagnostic, ...] = ()

    @classmethod
    def load(
        cls,
        problem_path: str | Path,
        table_paths: Sequence[str | Path] = (),
        *,
        profile: GrammarProfile | str = GrammarProfile.TAOS96,
        seed: int | None = None,
        parameter_overrides: Mapping[str, float] | None = None,
    ) -> LoadedProgram:
        """Ingest, validate, and lower a `.prb` plus its optional `.tbl` files."""

        selected_profile = GrammarProfile(profile)
        diagnostics: list[Diagnostic] = []
        table_documents: list[TableDocument] = []
        normalized_table_paths = tuple(str(Path(path)) for path in table_paths)
        for path in normalized_table_paths:
            ingested = ingest_file(path, profile=selected_profile)
            diagnostics.extend(ingested.diagnostics)
            if ingested.kind is not FileKind.TABLE or not isinstance(ingested.document, TableDocument):
                continue
            table_documents.append(ingested.document)
        available_tables = {
            name: table_type
            for document in table_documents
            for name, table_type in table_type_catalog(document).items()
        }
        problem_ingested = ingest_file(problem_path, available_tables=available_tables, profile=selected_profile)
        diagnostics.extend(problem_ingested.diagnostics)
        if problem_ingested.kind is not FileKind.PROBLEM or not isinstance(problem_ingested.document, ProblemDocument):
            raise ProgramLoadError(diagnostics)
        if any(item.severity is Severity.ERROR for item in diagnostics):
            raise ProgramLoadError(diagnostics)
        unit_settings, _ = problem_unit_settings(problem_ingested.document)
        tables = {
            name: table
            for document in table_documents
            for name, table in lower_tables(document, unit_settings).items()
        }
        lowered = lower_problem_document(problem_ingested.document, tables, seed=seed, parameter_overrides=parameter_overrides)
        return cls(str(problem_path), normalized_table_paths, selected_profile, problem_ingested.document, tuple(table_documents), lowered, tuple(diagnostics))
    ####

    def case(self, index: int = 0) -> RuntimeProblem:
        """Return the selected executable case."""

        try:
            return self.lowered.cases[index].problem
        except IndexError as error:
            raise IndexError(f"loaded program has no case at index {index}") from error
    ####

    def copy_case(self, index: int = 0) -> RuntimeProblem:
        """Copy a case at its current loaded state for independent execution."""

        problem = self.case(index)
        return problem.clone_at(max(vehicle.state.time for vehicle in problem.vehicles.values()), resume=True)
    ####

    def clone_case_at(self, time: float, index: int = 0) -> RuntimeProblem:
        """Clone a case at an exact or interpolated recorded flight time."""

        return self.case(index).clone_at(time)
    ####

    def inspect(self) -> dict[str, object]:
        """Return a JSON-compatible summary of source and executable state."""

        problems = []
        for problem in self.source.problems:
            problems.append({
                "name": problem.name,
                "blocks": [block.keyword for block in problem.blocks],
                "trajectories": [
                    {
                        "number": trajectory.number,
                        "name": trajectory.name,
                        "start_segment": trajectory.start_segment,
                        "segments": [{"number": segment.number, "title": segment.title, "blocks": [block.keyword for block in segment.blocks]} for segment in trajectory.segments],
                    }
                    for trajectory in problem.trajectories
                ],
            })
        return {
            "problem_path": self.problem_path,
            "table_paths": list(self.table_paths),
            "grammar_profile": self.profile.value,
            "title": next((block.title for block in self.source.problems[0].blocks if isinstance(block, TitleBlock)), self.source.problems[0].name),
            "metadata": {
                "problem_name": self.source.problems[0].name,
                "source_path": self.source.problems[0].location.path,
                "source_line": self.source.problems[0].location.line,
                "defaults": self.source.problems[0].defaults.model_dump(mode="json"),
            },
            "problems": problems,
            "tables": {name: table.table_type for name, table in self.lowered.tables.items()},
            "cases": len(self.lowered.cases),
            "searches": [search.search_id for search in self.lowered.searches],
            "optimizations": [optimize.loop for optimize in self.lowered.optimizations],
            "controls": self.inspect_controls(),
            "parameters": self.inspect_parameters(),
            "lqr": self.inspect_lqr(),
            "vehicles": [
                {
                    "name": name,
                    "state_names": list(vehicle.state.value_names),
                    "time": vehicle.state.time,
                    "controls": sorted(vehicle.control_values),
                    "segment": vehicle.segment_number,
                }
                for name, vehicle in self.case().vehicles.items()
            ],
            "unsupported_features": list(self.lowered.unsupported_features),
        }
    ####

    def inspect_controls(self, index: int = 0) -> list[dict[str, object]]:
        """List every declared control and its live value for each vehicle."""

        problem = self.case(index)
        controls: list[dict[str, object]] = []
        for block in self.source.problems[0].blocks:
            if not isinstance(block, RuntimeBlock) or block.declaration != "control" or block.name is None:
                continue
            attributes = block.attributes
            target = attributes.get("vehicle")
            vehicles = tuple(problem.vehicles) if target is None else (target,)
            controls.append({
                "name": block.name,
                "unit": attributes.get("unit"),
                "default": float(attributes.get("default", "0")),
                "lower": float(attributes["lower"]) if "lower" in attributes else None,
                "upper": float(attributes["upper"]) if "upper" in attributes else None,
                "slew_rate": float(attributes["slew"]) if "slew" in attributes else None,
                "modes": tuple(item for item in attributes.get("modes", "point-mass,kinematic-6dof,rigid-body-6dof").split(",") if item),
                "vehicle": target,
                "current_values": {vehicle: problem.vehicles[vehicle].control_values.get(block.name.casefold()) for vehicle in vehicles if vehicle in problem.vehicles},
            })
        return controls
    ####

    def inspect_parameters(self, index: int = 0) -> list[dict[str, object]]:
        """List problem-file parameters and their resolved values."""

        problem = self.case(index)
        raw_parameters = problem.metadata.get("parameters", {})
        resolved = raw_parameters if isinstance(raw_parameters, Mapping) else {}
        return [
            {
                "name": block.name,
                "unit": block.attributes.get("unit"),
                "value": resolved.get(block.name.casefold()),
                "mutable": block.attributes.get("mutable", "false").casefold() in {"1", "true", "yes", "on"},
            }
            for block in self.source.problems[0].blocks
            if isinstance(block, RuntimeBlock) and block.declaration == "parameter" and block.name is not None
        ]
    ####

    def inspect_lqr(self) -> list[dict[str, object]]:
        """List declarative LQR configurations without solving them."""

        return [
            {
                "name": block.name,
                "states": tuple(block.attributes.get("states", "").split(",")) if block.attributes.get("states") else (),
                "controls": tuple(block.attributes.get("controls", "").split(",")) if block.attributes.get("controls") else (),
                "q_source": block.attributes.get("q", block.attributes.get("q-table")),
                "r_source": block.attributes.get("r", block.attributes.get("r-table")),
                "linearization_source": block.attributes.get("linearization", block.attributes.get("ab")),
                "method": block.attributes.get("method", "continuous"),
                "update": block.attributes.get("update", "initial"),
            }
            for block in self.source.problems[0].blocks
            if isinstance(block, RuntimeBlock) and block.declaration == "lqr" and block.name is not None
        ]
    ####

    def inspect_case(self, index: int = 0) -> dict[str, object]:
        """Return live emulator state, controls, parameters, and histories."""

        problem = self.case(index)
        raw_parameters = problem.metadata.get("parameters", {})
        parameters = {str(name): value for name, value in raw_parameters.items()} if isinstance(raw_parameters, Mapping) else {}
        return {
            "case_index": index,
            "parameters": parameters,
            "event_history": list(problem.event_history),
            "vehicles": {
                name: {
                    "time": vehicle.state.time,
                    "active": vehicle.active,
                    "segment": vehicle.segment_number,
                    "source_segment": self._source_segment(vehicle_name=name, number=vehicle.segment_number),
                    "state_names": list(vehicle.state.value_names),
                    "values": list(vehicle.state.values),
                    "named": dict(vehicle.state.named),
                    "history_length": len(vehicle.history),
                    "controls": dict(vehicle.control_values),
                }
                for name, vehicle in problem.vehicles.items()
            },
        }
    ####

    def observe(self, *, index: int = 0, vehicle: str | None = None, include_deep: bool = False) -> object:
        """Return standard, declared-status, and optional deep observations."""

        status_names = tuple(
            block.attributes.get("source", block.name or "")
            for block in self.source.problems[0].blocks
            if isinstance(block, RuntimeBlock) and block.declaration == "status" and block.name is not None
        )
        return self.case(index).observe(vehicle, status_names=status_names, include_deep=include_deep)
    ####

    def _source_segment(self, *, vehicle_name: str, number: int) -> dict[str, object] | None:
        """Resolve a live segment number back to its source-file record."""

        trajectory = next((item for item in self.source.problems[0].trajectories if str(item.number) == vehicle_name), None)
        if trajectory is None:
            return None
        segment = next((item for item in trajectory.segments if item.number == number), None)
        if segment is None:
            return {"number": number, "title": None, "source_path": trajectory.location.path, "source_line": trajectory.location.line}
        return {"number": segment.number, "title": segment.title, "source_path": segment.location.path, "source_line": segment.location.line}
    ####

    def set_control(self, name: str, value: float, *, vehicle: str | None = None, index: int = 0) -> float:
        """Modify a declared runtime control in the live emulator case."""

        if not math.isfinite(value):
            raise ValueError("runtime control value must be finite")
        controls = {
            block.name.casefold(): block
            for block in self.source.problems[0].blocks
            if isinstance(block, RuntimeBlock) and block.declaration == "control" and block.name is not None
        }
        control = controls.get(name.casefold())
        if control is None:
            raise KeyError(f"unknown runtime control: {name}")
        lower = float(control.attributes["lower"]) if "lower" in control.attributes else -math.inf
        upper = float(control.attributes["upper"]) if "upper" in control.attributes else math.inf
        applied = min(upper, max(lower, float(value)))
        problem = self.case(index)
        selected = tuple(problem.vehicles.values()) if vehicle is None else (problem.vehicles.get(vehicle),)
        if any(item is None for item in selected):
            raise KeyError(f"unknown runtime vehicle: {vehicle}")
        for item in selected:
            assert item is not None
            item.control_values = {**item.control_values, name.casefold(): applied}
            refreshed = item.state.with_values(item.state.values)
            item.state = RuntimeState(
                refreshed.time,
                refreshed.values,
                refreshed.frame,
                {**refreshed.named, name.casefold(): applied},
                refreshed.value_names,
                refreshed.segment_endpoints,
            )
        return applied
    ####

    def set_parameter(self, name: str, value: float, *, index: int = 0) -> None:
        """Modify a mutable numeric parameter without rewriting source text."""

        if not math.isfinite(value):
            raise ValueError("runtime parameter value must be finite")
        declaration = next(
            (
                block
                for block in self.source.problems[0].blocks
                if isinstance(block, RuntimeBlock)
                and block.declaration == "parameter"
                and block.name is not None
                and block.name.casefold() == name.casefold()
            ),
            None,
        )
        if declaration is not None and declaration.attributes.get("mutable", "false").casefold() not in {"1", "true", "yes", "on"}:
            raise ValueError(f"runtime parameter {name!r} is setup-only; pass parameter_overrides to load()")
        problem = self.case(index)
        parameters = problem.metadata.setdefault("parameters", {})
        if not isinstance(parameters, dict):
            raise TypeError("loaded program parameter store is not mutable")
        parameters[name.casefold()] = float(value)
        for vehicle in problem.vehicles.values():
            vehicle.parameters = parameters
            refreshed = vehicle.state.with_values(vehicle.state.values)
            vehicle.state = RuntimeState(
                refreshed.time,
                refreshed.values,
                refreshed.frame,
                {**refreshed.named, name.casefold(): float(value)},
                refreshed.value_names,
                refreshed.segment_endpoints,
            )
    ####

    def save_checkpoint(self, path: str | Path, *, index: int = 0) -> Path:
        """Persist source, lowered-case identity, and live emulator state.

        The executable callbacks are rebuilt on load from the serialized source
        documents; they are intentionally not pickled as Python closures.
        """

        problem = self.case(index)
        if any(vehicle.kinematic_state is not None for vehicle in problem.vehicles.values()):
            raise NotImplementedError("checkpoint serialization for kinematic sidecars is not implemented")
        payload = {
            "schema_version": 2,
            "problem_path": self.problem_path,
            "table_paths": list(self.table_paths),
            "profile": self.profile.value,
            "source": self.source.model_dump(mode="json"),
            "table_documents": [document.model_dump(mode="json") for document in self.table_documents],
            "integrity": {
                "source_sha256": _fingerprint(self.source.model_dump(mode="json")),
                "tables_sha256": [_fingerprint(document.model_dump(mode="json")) for document in self.table_documents],
            },
            "case_index": index,
            "problem": {
                "print_times": list(problem.print_times),
                "table_knots": list(problem.table_knots),
                "final_time": problem.final_time,
                "metadata": _json_safe({key: value for key, value in problem.metadata.items() if key != "tables"}),
                "event_history": _json_safe(problem.event_history),
                "vehicles": {
                    name: {
                        "state": _state_payload(vehicle.state),
                        "history": [_state_payload(state) for state in vehicle.history],
                        "active": vehicle.active,
                        "activation_pending": vehicle.activation_pending,
                        "segment_number": vehicle.segment_number,
                        "fired_events": sorted(vehicle.fired_events),
                        "control_values": _json_safe(vehicle.control_values),
                        "parameters": _json_safe(vehicle.parameters),
                        "step_size": vehicle.step_size,
                        "integrator": vehicle.integrator,
                        "absolute_tolerance": vehicle.absolute_tolerance,
                        "relative_tolerance": vehicle.relative_tolerance,
                        "max_step_size": vehicle.max_step_size,
                        "publish_derived_rates": vehicle.publish_derived_rates,
                    }
                    for name, vehicle in problem.vehicles.items()
                },
            },
        }
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        serialized = json.dumps(payload, indent=2, sort_keys=True) + "\n"
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=destination.parent, delete=False) as temporary:
            temporary.write(serialized)
            temporary_path = Path(temporary.name)
        os.replace(temporary_path, destination)
        return destination
    ####

    @classmethod
    def load_checkpoint(cls, path: str | Path) -> LoadedProgram:
        """Rebuild a loaded program and restore its saved emulator state."""

        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if payload.get("schema_version") != 2:
            raise ValueError(f"unsupported TAORYX checkpoint schema: {payload.get('schema_version')!r}")
        source_payload = payload["source"]
        table_payloads = payload.get("table_documents", ())
        integrity = payload.get("integrity", {})
        if integrity.get("source_sha256") != _fingerprint(source_payload):
            raise ValueError("checkpoint source integrity verification failed")
        if integrity.get("tables_sha256", []) != [_fingerprint(item) for item in table_payloads]:
            raise ValueError("checkpoint table integrity verification failed")
        source = ProblemDocument.model_validate(source_payload)
        table_documents = tuple(TableDocument.model_validate(item) for item in table_payloads)
        unit_settings, _ = problem_unit_settings(source)
        tables = {name: table for document in table_documents for name, table in lower_tables(document, unit_settings).items()}
        lowered = lower_problem_document(source, tables)
        program = cls(
            str(payload["problem_path"]),
            tuple(str(item) for item in payload.get("table_paths", ())),
            GrammarProfile(payload["profile"]),
            source,
            table_documents,
            lowered,
        )
        case = program.case(int(payload.get("case_index", 0)))
        saved = payload["problem"]
        case.print_times = tuple(float(value) for value in saved.get("print_times", ()))
        case.table_knots = tuple(float(value) for value in saved.get("table_knots", ()))
        case.final_time = float(saved["final_time"]) if saved.get("final_time") is not None else None
        case.metadata.update(saved.get("metadata", {}))
        case.event_history[:] = list(saved.get("event_history", ()))
        for name, vehicle_payload in saved["vehicles"].items():
            vehicle = case.vehicles[name]
            vehicle.state = _state_from_payload(vehicle_payload["state"], vehicle.state.frame)
            vehicle.history = [_state_from_payload(item, vehicle.state.frame) for item in vehicle_payload["history"]]
            vehicle.active = bool(vehicle_payload["active"])
            vehicle.activation_pending = bool(vehicle_payload["activation_pending"])
            vehicle.segment_number = int(vehicle_payload["segment_number"])
            vehicle.fired_events = set(vehicle_payload.get("fired_events", ()))
            vehicle.control_values = {str(key): float(value) for key, value in vehicle_payload.get("control_values", {}).items()}
            vehicle.parameters = {str(key): float(value) for key, value in vehicle_payload.get("parameters", {}).items()}
            vehicle.step_size = float(vehicle_payload["step_size"])
            vehicle.integrator = str(vehicle_payload["integrator"])
            vehicle.absolute_tolerance = float(vehicle_payload["absolute_tolerance"])
            vehicle.relative_tolerance = float(vehicle_payload["relative_tolerance"])
            vehicle.max_step_size = float(vehicle_payload["max_step_size"]) if vehicle_payload.get("max_step_size") is not None else None
            vehicle.publish_derived_rates = bool(vehicle_payload["publish_derived_rates"])
        return program
    ####
####


def _state_payload(state: RuntimeState) -> dict[str, object]:
    """Serialize the portable portion of one runtime state."""

    return {
        "time": state.time,
        "values": list(state.values),
        "frame": getattr(state.frame, "value", str(state.frame)),
        "named": _json_safe(state.named),
        "value_names": list(state.value_names),
    }
####


def _state_from_payload(payload: Mapping[str, object], frame: Frame | str) -> RuntimeState:
    """Restore a runtime state using the rebuilt case frame."""

    values = cast(Sequence[float | int | str], payload["values"])
    named = cast(Mapping[str, object], payload.get("named", {}))
    value_names = cast(Sequence[object], payload.get("value_names", ()))
    return RuntimeState(
        float(cast(float | int | str, payload["time"])),
        tuple(float(value) for value in values),
        frame,
        {str(key): float(cast(float | int | str, value)) for key, value in named.items()},
        tuple(str(name) for name in value_names),
    )
####


def _json_safe(value: object) -> object:
    """Keep checkpoint metadata JSON-compatible without serializing callbacks."""

    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set, frozenset)):
        return [_json_safe(item) for item in value]
    return str(value)
####


def _fingerprint(value: object) -> str:
    """Hash canonical JSON so checkpoints detect tampering or drift."""

    canonical = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()
####
