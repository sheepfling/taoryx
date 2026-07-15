"""Validated, serializable scenario identity and compilation boundary."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Callable
from pathlib import Path
from typing import Annotated, Any, Literal, cast

from pydantic import BaseModel, ConfigDict, Field, model_validator

from taoryx.language.diagnostics import Diagnostic, Severity
from taoryx.language.expressions import parse_expression
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.ingest import FileKind, ingest_file
from taoryx.language.models import ProblemDocument, RuntimeBlock, TableDocument
from taoryx.language.table_parser import table_type_catalog

SCENARIO_SCHEMA_VERSION = 1


class ScenarioCompileError(ValueError):
    """Compilation failed validation and retained the source diagnostics."""

    def __init__(self, message: str, diagnostics: tuple[Diagnostic, ...]) -> None:
        super().__init__(message)
        self.diagnostics = diagnostics
        ####
    ####


class ScenarioSource(BaseModel):
    """One source file included in a resolved scenario identity."""

    model_config = ConfigDict(frozen=True)

    path: str
    kind: FileKind
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    size: int = Field(ge=0)
    ####


class ParameterOverride(BaseModel):
    """Replace one named scenario parameter before initial-state lowering."""

    model_config = ConfigDict(frozen=True)

    kind: Literal["parameter"] = "parameter"
    name: str = Field(min_length=1)
    value: float
    unit: str | None = None
    ####


class InitialValueOverride(BaseModel):
    """Replace one named initial-state value for a vehicle."""

    model_config = ConfigDict(frozen=True)

    kind: Literal["initial-value"] = "initial-value"
    vehicle: str = Field(min_length=1)
    field: str = Field(min_length=1)
    value: float
    unit: str | None = None
    ####


class InitialFrameState(BaseModel):
    """Frame-labeled position and velocity initialization."""

    model_config = ConfigDict(frozen=True)

    kind: Literal["initial-frame"] = "initial-frame"
    vehicle: str = Field(min_length=1)
    frame: Literal["ecfc", "geodetic"]
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    position_units: tuple[str | None, str | None, str | None] = (None, None, None)
    velocity_units: tuple[str | None, str | None, str | None] = (None, None, None)
    ####


class VelocityImpulse(BaseModel):
    """A velocity increment expressed in one explicit frame convention."""

    model_config = ConfigDict(frozen=True)

    kind: Literal["velocity-impulse"] = "velocity-impulse"
    vehicle: str = Field(min_length=1)
    frame: Literal["ecfc", "geodetic", "body"]
    delta_velocity: tuple[float, float, float]
    unit: str | None = None
    ####


class MassAdjustment(BaseModel):
    """Set or offset the initial mass of one vehicle."""

    model_config = ConfigDict(frozen=True)

    kind: Literal["mass"] = "mass"
    vehicle: str = Field(min_length=1)
    absolute_mass: float | None = None
    delta_mass: float | None = None
    unit: str | None = None

    @model_validator(mode="after")
    def exactly_one_mode(self) -> MassAdjustment:
        if (self.absolute_mass is None) == (self.delta_mass is None):
            raise ValueError("mass adjustment requires exactly one of absolute_mass or delta_mass")
        return self
    ####


class InheritanceOverride(BaseModel):
    """Declare an explicit source trajectory/segment for inherited state."""

    model_config = ConfigDict(frozen=True)

    kind: Literal["inheritance"] = "inheritance"
    vehicle: str = Field(min_length=1)
    source_vehicle: str = Field(min_length=1)
    source_segment: int = Field(ge=1)
    ####


class IntegratorOverride(BaseModel):
    """Numerical configuration attached to a scenario request."""

    model_config = ConfigDict(frozen=True)

    kind: Literal["integrator"] = "integrator"
    name: str = Field(min_length=1)
    vehicle: str | None = None
    absolute_tolerance: float | None = None
    relative_tolerance: float | None = None
    max_step: float | None = None
    ####


class RandomSeed(BaseModel):
    """Set the deterministic random seed as an ordered composition patch."""

    model_config = ConfigDict(frozen=True)

    kind: Literal["random-seed"] = "random-seed"
    seed: int
    ####


class ResolutionRecord(BaseModel):
    """One applied composition change with explicit before/after evidence."""

    model_config = ConfigDict(frozen=True)

    order: int = Field(ge=0)
    kind: str
    target: str
    before: object | None = None
    after: object | None = None
    unit: str | None = None
    frame: str | None = None
    source: str
    reason: str
    ####


class ControlContract(BaseModel):
    """Serializable control declaration shared by batch and interactive consumers."""

    model_config = ConfigDict(frozen=True)

    name: str = Field(min_length=1)
    unit: str | None = None
    default: float = 0.0
    lower: float | None = None
    upper: float | None = None
    slew_rate: float | None = None
    modes: tuple[str, ...] = ("point-mass", "kinematic-6dof", "rigid-body-6dof")
    ####


class StatusContract(BaseModel):
    """Serializable status declaration shared by runtime projections."""

    model_config = ConfigDict(frozen=True)

    name: str = Field(min_length=1)
    source: str | None = None
    unit: str | None = None
    modes: tuple[str, ...] = ("point-mass", "kinematic-6dof", "rigid-body-6dof")
    ####


class EventContract(BaseModel):
    """Serializable event rule evaluated at accepted runtime boundaries."""

    model_config = ConfigDict(frozen=True)

    name: str = Field(min_length=1)
    condition: str = Field(min_length=1)
    action: Literal["stop", "transition", "signal"] = "signal"
    signal: str | None = None
    once: bool = True
    ####


class OutputContract(BaseModel):
    """Serializable semantic output subscription."""

    model_config = ConfigDict(frozen=True)

    channels: tuple[str, ...] = ()
    sample_interval: float | None = None
    include_events: bool = True
    ####


class ScenarioRuntimeContract(BaseModel):
    """Shared runtime declarations projected into batch and step mode."""

    model_config = ConfigDict(frozen=True)

    schema_version: int = 1
    controls: tuple[ControlContract, ...] = ()
    statuses: tuple[StatusContract, ...] = ()
    events: tuple[EventContract, ...] = ()
    outputs: tuple[OutputContract, ...] = ()

    @model_validator(mode="after")
    def unique_names(self) -> ScenarioRuntimeContract:
        control_names = [item.name for item in self.controls]
        status_names = [item.name for item in self.statuses]
        if len(control_names) != len(set(control_names)):
            raise ValueError("runtime control names must be unique")
        if len(status_names) != len(set(status_names)):
            raise ValueError("runtime status names must be unique")
        event_names = [item.name for item in self.events]
        if len(event_names) != len(set(event_names)):
            raise ValueError("runtime event names must be unique")
        return self
    ####


CompositionPatch = Annotated[
    ParameterOverride | InitialValueOverride | InitialFrameState | VelocityImpulse | MassAdjustment | InheritanceOverride | IntegratorOverride | RandomSeed,
    Field(discriminator="kind"),
]


class ScenarioRequest(BaseModel):
    """Serializable inputs that affect scenario resolution."""

    model_config = ConfigDict(frozen=True)

    problem_path: str
    table_paths: tuple[str, ...] = ()
    profile: GrammarProfile = GrammarProfile.TAOS96
    seed: int | None = None
    integrator: str | None = None
    parameter_overrides: dict[str, float] = Field(default_factory=dict)
    patches: tuple[CompositionPatch, ...] = ()
    runtime: ScenarioRuntimeContract = Field(default_factory=ScenarioRuntimeContract)
    ####


class ResolvedScenario(BaseModel):
    """Immutable, cacheable result of source validation and resolution."""

    model_config = ConfigDict(frozen=True)

    schema_version: int = SCENARIO_SCHEMA_VERSION
    identity: str = Field(pattern=r"^[0-9a-f]{64}$")
    request: ScenarioRequest
    sources: tuple[ScenarioSource, ...]
    table_names: tuple[str, ...] = ()
    diagnostics: tuple[dict[str, Any], ...] = ()
    composition: tuple[dict[str, Any], ...] = ()
    resolution_records: tuple[ResolutionRecord, ...] = ()
    parsed_problem: dict[str, Any] | None = None
    parsed_tables: tuple[dict[str, Any], ...] = ()
    ####

    def write_json(self, path: str | Path) -> Path:
        """Write the canonical scenario envelope to JSON."""

        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(self.model_dump_json(indent=2) + "\n", encoding="utf-8")
        return destination
    ####

    @classmethod
    def read_json(cls, path: str | Path) -> ResolvedScenario:
        """Read and verify a serialized scenario envelope."""

        scenario = cls.model_validate_json(Path(path).read_text(encoding="utf-8"))
        expected = _scenario_identity(scenario.request, scenario.sources)
        if scenario.identity != expected:
            raise ValueError("resolved scenario identity does not match its request and source manifest")
        return scenario
    ####

    def lower(self) -> Any:
        """Lower the validated source and apply its semantic composition patches."""

        from taoryx.runtime.lowering import lower_problem_document, lower_tables

        table_documents = [TableDocument.model_validate(document) for document in self.parsed_tables] if self.parsed_tables else [
            cast(TableDocument, ingest_file(path, profile=self.request.profile).document)
            for path in self.request.table_paths
        ]
        problem_document: Any = (
            ProblemDocument.model_validate(self.parsed_problem)
            if self.parsed_problem is not None
            else cast(
                ProblemDocument,
                ingest_file(
                    self.request.problem_path,
                    available_tables={name for document in table_documents for name in table_type_catalog(document)},
                    profile=self.request.profile,
                ).document,
            )
        )
        unit_settings: dict[str, str | None] = {}
        for source_problem in problem_document.problems:
            for block in source_problem.blocks:
                if getattr(block, "keyword", "") != "units/fmt":
                    continue
                for setting in block.settings:
                    unit_settings[setting.variable.casefold()] = setting.unit
        tables = {name: table for document in table_documents for name, table in lower_tables(document, unit_settings).items()}
        lowered = lower_problem_document(
            problem_document,
            tables,
            seed=self.request.seed,
            parameter_overrides=self.request.parameter_overrides,
        )
        for case in lowered.cases:
            records = _apply_runtime_patches(case.problem, self.request.patches)
            case.problem.metadata["composition_records"] = records
        return lowered
    ####

    def interactive_session(self, *, case_index: int = 0) -> Any:
        """Project one lowered case into the shared InteractiveSession contract."""

        from taoryx.runtime.interactive import ControlSpec, InteractiveSession, OutputSubscription, StatusSpec

        lowered = self.lower()
        if case_index < 0 or case_index >= len(lowered.cases):
            raise IndexError(f"scenario case index out of range: {case_index}")
        runtime = self.request.runtime
        controls = tuple(
            ControlSpec(
                item.name,
                item.unit,
                item.default,
                float("-inf") if item.lower is None else item.lower,
                float("inf") if item.upper is None else item.upper,
                item.slew_rate,
                item.modes,
            )
            for item in runtime.controls
        )
        statuses = tuple(StatusSpec(item.name, item.source, item.unit, item.modes) for item in runtime.statuses)
        from taoryx.runtime.interactive import EventAction, EventSpec

        events = tuple(
            EventSpec(
                item.name,
                _runtime_predicate(item.condition),
                EventAction(item.action),
                item.signal,
                item.once,
                "problem.*runtime",
            )
            for item in runtime.events
        )
        outputs = tuple(OutputSubscription(item.channels, item.sample_interval, item.include_events) for item in runtime.outputs)
        return InteractiveSession(lowered.cases[case_index].problem, controls=controls, status_specs=statuses, event_specs=events, output_subscriptions=outputs)
    ####

    def run(self, *, output_dir: str | Path = ".", max_steps: int = 100000) -> tuple[Any, ...]:
        """Execute this resolved scenario and return identity-linked artifacts."""

        from taoryx.outputs import apply_output_subscriptions, build_run_artifact
        from taoryx.runtime.lowering import execute_lowered

        lowered = self.lower()
        results = execute_lowered(
            lowered,
            output_dir=str(output_dir),
            max_steps=max_steps,
            integrator=self.request.integrator,
        )
        artifacts = []
        for case, result in zip(lowered.cases, results, strict=True):
            artifact = build_run_artifact(
                str(self.request.problem_path),
                case.problem,
                result,
                scenario_identity=self.identity,
                composition=self.composition,
                resolution_records=tuple(case.problem.metadata.get("composition_records", ())),
                events=case.problem.event_history,
                visualization={"source": "RunArtifact", "schema_version": 1, "runtime": self.request.runtime.model_dump(mode="json")},
            )
            artifacts.append(apply_output_subscriptions(artifact, self.request.runtime.outputs))
        return tuple(artifacts)
    ####


class ScenarioCompiler:
    """Compile validated source files into a deterministic scenario envelope."""

    def compile(
        self,
        problem_path: str | Path,
        *,
        table_paths: tuple[str | Path, ...] = (),
        profile: GrammarProfile | str = GrammarProfile.TAOS96,
        seed: int | None = None,
        integrator: str | None = None,
        parameter_overrides: dict[str, float] | None = None,
        patches: tuple[CompositionPatch, ...] = (),
        runtime: ScenarioRuntimeContract | None = None,
    ) -> ResolvedScenario:
        problem = Path(problem_path)
        tables = tuple(Path(path) for path in table_paths)
        selected_profile = GrammarProfile(profile)
        diagnostics: list[Diagnostic] = []
        table_documents: list[TableDocument] = []
        sources = [_source_record(problem, FileKind.PROBLEM)]
        for table_path in tables:
            ingested = ingest_file(table_path, profile=selected_profile)
            diagnostics.extend(ingested.diagnostics)
            sources.append(_source_record(table_path, FileKind.TABLE))
            if ingested.kind is FileKind.TABLE:
                table_documents.append(cast(TableDocument, ingested.document))
            else:
                diagnostics.append(_error(table_path, "expected-table-file", "scenario table inputs must use .tbl files"))
        available_tables = {name for document in table_documents for name in table_type_catalog(document)}
        problem_ingested = ingest_file(problem, available_tables=available_tables, profile=selected_profile)
        diagnostics.extend(problem_ingested.diagnostics)
        if problem_ingested.kind is not FileKind.PROBLEM:
            diagnostics.append(_error(problem, "expected-problem-file", "scenario input must use a .prb file"))
        if any(item.severity is Severity.ERROR for item in diagnostics):
            raise ScenarioCompileError("scenario validation failed", tuple(diagnostics))
        resolved_parameter_overrides = {name.casefold(): value for name, value in (parameter_overrides or {}).items()}
        resolved_seed = seed
        for patch in patches:
            if isinstance(patch, ParameterOverride):
                resolved_parameter_overrides[patch.name.casefold()] = _to_internal(patch.value, patch.name, patch.unit)
            elif isinstance(patch, RandomSeed):
                resolved_seed = patch.seed
        declared_runtime = _runtime_contract(cast(ProblemDocument, problem_ingested.document))
        request = ScenarioRequest(
            problem_path=str(problem),
            table_paths=tuple(str(path) for path in tables),
            profile=selected_profile,
            seed=resolved_seed,
            integrator=integrator,
            parameter_overrides=resolved_parameter_overrides,
            patches=patches,
            runtime=runtime or declared_runtime,
        )
        normalized_sources = tuple(sources)
        return ResolvedScenario(
            identity=_scenario_identity(request, normalized_sources),
            request=request,
            sources=normalized_sources,
            table_names=tuple(sorted(available_tables)),
            diagnostics=tuple(item.model_dump(mode="json") for item in diagnostics),
            composition=tuple(patch.model_dump(mode="json") for patch in patches),
            parsed_problem=cast(ProblemDocument, problem_ingested.document).model_dump(mode="json"),
            parsed_tables=tuple(document.model_dump(mode="json") for document in table_documents),
        )
    ####

    def load_or_compile(
        self,
        cache_path: str | Path,
        problem_path: str | Path,
        *,
        table_paths: tuple[str | Path, ...] = (),
        profile: GrammarProfile | str = GrammarProfile.TAOS96,
        seed: int | None = None,
        integrator: str | None = None,
        parameter_overrides: dict[str, float] | None = None,
        patches: tuple[CompositionPatch, ...] = (),
        runtime: ScenarioRuntimeContract | None = None,
    ) -> ResolvedScenario:
        """Reuse a valid cache entry, invalidating it when source identity changes."""

        destination = Path(cache_path)
        cached: ResolvedScenario | None = None
        try:
            cached = ResolvedScenario.read_json(destination)
        except (OSError, ValueError, TypeError, ScenarioCompileError):
            pass
        if cached is not None:
            try:
                selected_profile = GrammarProfile(profile)
                resolved_parameters = {name.casefold(): value for name, value in (parameter_overrides or {}).items()}
                resolved_seed = seed
                for patch in patches:
                    if isinstance(patch, ParameterOverride):
                        resolved_parameters[patch.name.casefold()] = _to_internal(patch.value, patch.name, patch.unit)
                    elif isinstance(patch, RandomSeed):
                        resolved_seed = patch.seed
                expected_request = ScenarioRequest(
                    problem_path=str(problem_path),
                    table_paths=tuple(str(path) for path in table_paths),
                    profile=selected_profile,
                    seed=resolved_seed,
                    integrator=integrator,
                    parameter_overrides=resolved_parameters,
                    patches=patches,
                    runtime=runtime or ScenarioRuntimeContract(),
                )
                expected_sources = tuple(
                    [_source_record(Path(problem_path), FileKind.PROBLEM)]
                    + [_source_record(Path(path), FileKind.TABLE) for path in table_paths]
                )
                if cached.identity == _scenario_identity(expected_request, expected_sources):
                    return cached
            except (OSError, TypeError, ValueError):
                pass
        requested = self.compile(
            problem_path,
            table_paths=table_paths,
            profile=profile,
            seed=seed,
            integrator=integrator,
            parameter_overrides=parameter_overrides,
            patches=patches,
            runtime=runtime,
        )
        if cached is not None and cached.identity == requested.identity:
            return cached
        requested.write_json(destination)
        return requested
    ####


def _source_record(path: Path, kind: FileKind) -> ScenarioSource:
    data = path.read_bytes()
    return ScenarioSource(path=str(path), kind=kind, sha256=hashlib.sha256(data).hexdigest(), size=len(data))
    ####


def _runtime_contract(document: ProblemDocument) -> ScenarioRuntimeContract:
    """Lower TAORYX ``*runtime`` declarations into the shared contract."""

    controls: list[ControlContract] = []
    statuses: list[StatusContract] = []
    events: list[EventContract] = []
    outputs: list[OutputContract] = []
    for problem in document.problems:
        for block in problem.blocks:
            if not isinstance(block, RuntimeBlock) or block.declaration is None:
                continue
            attributes = block.attributes
            declaration = block.declaration
            if declaration == "control":
                controls.append(ControlContract(
                    name=block.name or "",
                    unit=attributes.get("unit"),
                    default=_runtime_float(attributes, "default", 0.0),
                    lower=_runtime_optional_float(attributes, "lower"),
                    upper=_runtime_optional_float(attributes, "upper"),
                    slew_rate=_runtime_optional_float(attributes, "slew"),
                    modes=_runtime_modes(attributes),
                ))
            elif declaration == "status":
                statuses.append(StatusContract(name=block.name or "", source=attributes.get("source"), unit=attributes.get("unit"), modes=_runtime_modes(attributes)))
            elif declaration == "event":
                events.append(EventContract(name=block.name or "", condition=attributes.get("condition", ""), action=_runtime_action(attributes), signal=attributes.get("signal"), once=_runtime_bool(attributes, "once", True)))
            elif declaration == "output":
                channels = tuple(item for item in attributes.get("channels", "").split(",") if item)
                outputs.append(OutputContract(channels=channels, sample_interval=_runtime_optional_float(attributes, "interval"), include_events=_runtime_bool(attributes, "events", True)))
    return ScenarioRuntimeContract(controls=tuple(controls), statuses=tuple(statuses), events=tuple(events), outputs=tuple(outputs))
    ####


def _runtime_modes(attributes: dict[str, str]) -> tuple[str, ...]:
    return tuple(item for item in attributes.get("modes", "point-mass,kinematic-6dof,rigid-body-6dof").split(",") if item)


def _runtime_float(attributes: dict[str, str], name: str, default: float) -> float:
    value = attributes.get(name)
    return default if value is None else float(value)


def _runtime_optional_float(attributes: dict[str, str], name: str) -> float | None:
    value = attributes.get(name)
    return None if value is None else float(value)


def _runtime_bool(attributes: dict[str, str], name: str, default: bool) -> bool:
    value = attributes.get(name)
    return default if value is None else value.casefold() in {"1", "true", "yes", "on"}


def _runtime_action(attributes: dict[str, str]) -> Literal["stop", "transition", "signal"]:
    value = attributes.get("action", "signal")
    if value not in {"stop", "transition", "signal"}:
        raise ValueError(f"unsupported runtime event action {value!r}")
    return cast(Literal["stop", "transition", "signal"], value)


def _runtime_predicate(condition: str) -> Callable[[Any], bool]:
    expression = parse_expression(condition)

    def predicate(state: Any) -> bool:
        from taoryx.runtime.expressions import evaluate_expression

        return evaluate_expression(expression, {**state.named, "time": state.time}, {}) != 0.0

    return predicate
    ####


def _scenario_identity(request: ScenarioRequest, sources: tuple[ScenarioSource, ...]) -> str:
    payload = {
        "schema_version": SCENARIO_SCHEMA_VERSION,
        "request": {
            "profile": request.profile.value,
            "seed": request.seed,
            "integrator": request.integrator,
            "parameter_overrides": dict(sorted(request.parameter_overrides.items())),
            "patches": [patch.model_dump(mode="json") for patch in request.patches],
            "runtime": request.runtime.model_dump(mode="json"),
        },
        "sources": [{"kind": source.kind.value, "sha256": source.sha256, "size": source.size} for source in sources],
    }
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()
    ####


def _apply_runtime_patches(problem: Any, patches: tuple[CompositionPatch, ...]) -> tuple[dict[str, object], ...]:
    """Apply state-only patches after source lowering and coordinate normalization."""

    from taoryx.runtime.common import RuntimeState
    from taoryx.runtime.lowering import _normalize_initial_coordinates

    records: list[dict[str, object]] = []
    for order, patch in enumerate(patches):
        source = f"scenario.patch[{order}]"
        if isinstance(patch, ParameterOverride):
            parameters = problem.metadata.setdefault("parameters", {})
            if not isinstance(parameters, dict):
                raise TypeError("problem parameter store must be a dictionary")
            before = parameters.get(patch.name.casefold())
            value = _to_internal(patch.value, patch.name, patch.unit)
            parameters[patch.name.casefold()] = value
            records.append(ResolutionRecord(order=order, kind=patch.kind, target=patch.name.casefold(), before=before, after=value, unit=patch.unit, source=source, reason="parameter override").model_dump(mode="json"))
            continue
        if isinstance(patch, RandomSeed):
            records.append(ResolutionRecord(order=order, kind=patch.kind, target="seed", after=patch.seed, source=source, reason="random seed override").model_dump(mode="json"))
            continue
        if isinstance(patch, IntegratorOverride):
            selected = tuple(problem.vehicles.values()) if patch.vehicle is None else (problem.vehicles.get(patch.vehicle),)
            if any(vehicle is None for vehicle in selected):
                raise ValueError(f"integrator patch references unknown vehicle {patch.vehicle!r}")
            for vehicle in selected:
                assert vehicle is not None
                vehicle.integrator = patch.name
                if patch.absolute_tolerance is not None:
                    vehicle.absolute_tolerance = patch.absolute_tolerance
                if patch.relative_tolerance is not None:
                    vehicle.relative_tolerance = patch.relative_tolerance
                if patch.max_step is not None:
                    vehicle.max_step_size = patch.max_step
            records.append(ResolutionRecord(order=order, kind=patch.kind, target=patch.vehicle or "*", after=patch.name, source=source, reason="integrator override").model_dump(mode="json"))
            continue
        if isinstance(patch, InheritanceOverride):
            target = problem.vehicles.get(patch.vehicle)
            if target is None:
                raise ValueError(f"inheritance patch references unknown vehicle {patch.vehicle!r}")
            if patch.source_vehicle not in problem.vehicles:
                raise ValueError(f"inheritance patch references unknown source vehicle {patch.source_vehicle!r}")
            target.dependencies = (patch.source_vehicle,)
            target.dependency_segments = {patch.source_vehicle: patch.source_segment}
            target.active = False
            target.activation_pending = True
            records.append(ResolutionRecord(order=order, kind=patch.kind, target=patch.vehicle, after={"source_vehicle": patch.source_vehicle, "source_segment": patch.source_segment}, source=source, reason="inheritance override").model_dump(mode="json"))
            continue
        vehicle = problem.vehicles.get(patch.vehicle)
        if vehicle is None:
            raise ValueError(f"composition patch references unknown vehicle {patch.vehicle!r}")
        named = dict(vehicle.state.named)
        if isinstance(patch, InitialValueOverride):
            before = named.get(patch.field.casefold())
            value = _to_internal(patch.value, patch.field, patch.unit)
            named[patch.field.casefold()] = value
            records.append(ResolutionRecord(order=order, kind=patch.kind, target=f"{patch.vehicle}.{patch.field.casefold()}", before=before, after=value, unit=patch.unit, source=source, reason="initial value override").model_dump(mode="json"))
        elif isinstance(patch, InitialFrameState):
            before = {name: named.get(name) for name in ("x", "y", "z", "xdt", "ydt", "zdt")}
            if patch.frame == "ecfc":
                names = ("x", "y", "z", "xdt", "ydt", "zdt")
                variables = ("x", "y", "z", "xdt", "ydt", "zdt")
                values = (*patch.position, *patch.velocity)
                units = (*patch.position_units, *patch.velocity_units)
                for name, variable, value, unit in zip(names, variables, values, units, strict=True):
                    named[name] = _to_internal(value, variable, unit)
                _normalize_initial_coordinates(named, "ecfc")
            else:
                names = ("long", "lat", "alt", "vel", "gama", "psi")
                values = (*patch.position, *patch.velocity)
                units = (*patch.position_units, *patch.velocity_units)
                for name, value, unit in zip(names, values, units, strict=True):
                    named[name] = _to_internal(value, name, unit)
                _normalize_initial_coordinates(named, "geodetic")
            records.append(ResolutionRecord(order=order, kind=patch.kind, target=patch.vehicle, before=before, after={name: named.get(name) for name in ("x", "y", "z", "xdt", "ydt", "zdt")}, frame=patch.frame, source=source, reason="initial frame state").model_dump(mode="json"))
        elif isinstance(patch, VelocityImpulse):
            if patch.frame == "body":
                raise ValueError("body-frame velocity impulses require an attitude provider")
            if patch.frame == "ecfc":
                delta = tuple(_to_internal(value, "vel", patch.unit) for value in patch.delta_velocity)
            else:
                longitude = math.radians(named.get("long", 0.0))
                latitude = math.radians(named.get("lat", 0.0))
                east, north, up = tuple(_to_internal(value, "vel", patch.unit) for value in patch.delta_velocity)
                delta = (
                    up * math.cos(latitude) * math.cos(longitude) - north * math.sin(latitude) * math.cos(longitude) - east * math.sin(longitude),
                    up * math.cos(latitude) * math.sin(longitude) - north * math.sin(latitude) * math.sin(longitude) + east * math.cos(longitude),
                    up * math.sin(latitude) + north * math.cos(latitude),
                )
            before = {name: named.get(name) for name in ("xdt", "ydt", "zdt")}
            for name, value in zip(("xdt", "ydt", "zdt"), delta, strict=True):
                named[name] = named.get(name, 0.0) + value
            named["vel"] = math.sqrt(sum(named.get(name, 0.0) ** 2 for name in ("xdt", "ydt", "zdt")))
            records.append(ResolutionRecord(order=order, kind=patch.kind, target=f"{patch.vehicle}.velocity", before=before, after={name: named.get(name) for name in ("xdt", "ydt", "zdt")}, frame=patch.frame, source=source, reason="velocity impulse").model_dump(mode="json"))
        elif isinstance(patch, MassAdjustment):
            field = "mass" if "mass" in named else "wt"
            current = named.get(field, 0.0)
            if patch.absolute_mass is not None:
                updated = _to_internal(patch.absolute_mass, "mass", patch.unit)
            else:
                assert patch.delta_mass is not None
                updated = current + _to_internal(patch.delta_mass, "mass", patch.unit)
            named[field] = updated
            records.append(ResolutionRecord(order=order, kind=patch.kind, target=f"{patch.vehicle}.{field}", before=current, after=named[field], unit=patch.unit, source=source, reason="mass adjustment").model_dump(mode="json"))
        else:
            raise TypeError(f"unsupported composition patch {type(patch).__name__}")
        values = tuple(named[name] for name in vehicle.state.value_names)
        vehicle.state = RuntimeState(vehicle.state.time, values, vehicle.state.frame, named, vehicle.state.value_names, vehicle.state.segment_endpoints)
        vehicle.history[0] = vehicle.state
    return tuple(records)
    ####


def _to_internal(value: float | None, variable: str, unit: str | None) -> float:
    """Convert an explicit composition value to the runtime canonical unit."""

    if value is None:
        raise ValueError(f"missing value for composition variable {variable!r}")
    if unit is None:
        return float(value)
    from taoryx.runtime.units import to_internal

    return to_internal(float(value), variable, {variable.casefold(): unit})


def _error(path: str | Path, code: str, message: str) -> Diagnostic:
    from taoryx.language.diagnostics import SourceLocation

    return Diagnostic(severity=Severity.ERROR, code=code, message=message, location=SourceLocation(path=str(path), line=1))
    ####
####
