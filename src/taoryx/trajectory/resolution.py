"""Alpha 2 case resolution, canonical units, and provenance."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping
from pathlib import Path
from typing import Any, Literal

import yaml

from .contracts import (
    CaseIntent,
    CaseValue,
    DerivedParameter,
    FamilyCatalog,
    FamilyPackage,
    ParameterSchema,
    ProvenanceRecord,
    ResolvedCase,
    ResolvedValue,
    ResolvedVariant,
    VariantResolutionReport,
)


class ResolutionError(ValueError):
    """A fail-closed case resolution error with a stable diagnostic code."""

    def __init__(self, code: str, message: str, *, field: str | None = None) -> None:
        self.code = code
        self.field = field
        prefix = f"{code}: "
        if field is not None:
            prefix = f"{prefix}{field}: "
        super().__init__(prefix + message)
        ####
    ####


_UNIT_FACTORS: dict[str, tuple[str, float, float]] = {
    "": ("", 1.0, 0.0),
    "1": ("", 1.0, 0.0),
    "dimensionless": ("", 1.0, 0.0),
    "m": ("m", 1.0, 0.0),
    "km": ("m", 1000.0, 0.0),
    "ft": ("m", 0.3048, 0.0),
    "s": ("s", 1.0, 0.0),
    "min": ("s", 60.0, 0.0),
    "kg": ("kg", 1.0, 0.0),
    "kg/s": ("kg/s", 1.0, 0.0),
    "lbm": ("kg", 0.45359237, 0.0),
    "m/s": ("m/s", 1.0, 0.0),
    "km/s": ("m/s", 1000.0, 0.0),
    "ft/s": ("m/s", 0.3048, 0.0),
    "km/h": ("m/s", 1000.0 / 3600.0, 0.0),
    "N": ("N", 1.0, 0.0),
    "kN": ("N", 1000.0, 0.0),
    "m^2": ("m^2", 1.0, 0.0),
    "kg*m^2": ("kg*m^2", 1.0, 0.0),
    "deg": ("deg", 1.0, 0.0),
    "rad": ("deg", 180.0 / math.pi, 0.0),
}


def _canonical_unit(unit: str | None) -> str:
    """Normalize a unit spelling for lookup and diagnostics."""

    return "" if unit is None else unit.strip()
    ####


def _convert(value: Any, input_unit: str | None, canonical_unit: str | None, field: str) -> Any:
    """Convert a scalar input to its declared canonical unit."""

    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return value
    source = _canonical_unit(input_unit or canonical_unit)
    target = _canonical_unit(canonical_unit)
    if source == target or target == "":
        return value
    source_record = _UNIT_FACTORS.get(source)
    target_record = _UNIT_FACTORS.get(target)
    if source_record is None or target_record is None or source_record[0] != target_record[0]:
        raise ResolutionError("unit-mismatch", f"cannot convert {source!r} to {target!r}", field=field)
    base = float(value) * source_record[1] + source_record[2]
    return base / target_record[1] - target_record[2] / target_record[1]
    ####


def _as_case_value(raw: Any) -> CaseValue:
    """Normalize a YAML scalar or mapping into a typed input value."""

    if isinstance(raw, CaseValue):
        return raw
    if isinstance(raw, Mapping) and "value" in raw:
        return CaseValue.model_validate(raw)
    return CaseValue(value=raw)
    ####


def _check_value(schema: Any, value: Any, field: str) -> None:
    """Check the resolved value against the declared parameter schema."""

    if schema.kind == "number" and (not isinstance(value, (int, float)) or isinstance(value, bool)):
        raise ResolutionError("type-mismatch", "expected a numeric value", field=field)
    if schema.kind == "integer" and (not isinstance(value, int) or isinstance(value, bool)):
        raise ResolutionError("type-mismatch", "expected an integer value", field=field)
    if schema.kind == "boolean" and not isinstance(value, bool):
        raise ResolutionError("type-mismatch", "expected a boolean value", field=field)
    if schema.kind == "string" and not isinstance(value, str):
        raise ResolutionError("type-mismatch", "expected a string value", field=field)
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if schema.minimum is not None and value < schema.minimum:
            raise ResolutionError("out-of-range", f"value must be >= {schema.minimum}", field=field)
        if schema.maximum is not None and value > schema.maximum:
            raise ResolutionError("out-of-range", f"value must be <= {schema.maximum}", field=field)
    ####


def _apply_layer(
    values: dict[str, tuple[Any, str, str | None, Any]],
    layer: Mapping[str, Any],
    source: str,
    schemas: Mapping[str, Any],
) -> None:
    """Apply one preset layer and retain its source/unit metadata."""

    for parameter_id, raw in layer.items():
        if parameter_id not in schemas:
            raise ResolutionError("unknown-override", "parameter is not declared by the family", field=parameter_id)
        if schemas[parameter_id].role == "derived":
            raise ResolutionError("derived-override", "derived parameters are computed and cannot be overridden", field=parameter_id)
        supplied = _as_case_value(raw)
        canonical = _convert(supplied.value, supplied.unit, schemas[parameter_id].canonical_unit, parameter_id)
        _check_value(schemas[parameter_id], canonical, parameter_id)
        values[parameter_id] = (canonical, source, supplied.unit, supplied.value)
    ####


def _bound_variant_value(
    value: float,
    minimum: float | None,
    maximum: float | None,
    *,
    policy: str,
    field: str,
    report: dict[str, Any],
) -> float:
    """Apply an explicit reject/project policy to one candidate value."""

    projected = value
    if minimum is not None and projected < minimum:
        if policy == "reject":
            raise ResolutionError("variant-out-of-range", f"value must be >= {minimum}", field=field)
        projected = minimum
    if maximum is not None and projected > maximum:
        if policy == "reject":
            raise ResolutionError("variant-out-of-range", f"value must be <= {maximum}", field=field)
        projected = maximum
    if projected != value:
        report["projection_distance"] = float(report.get("projection_distance", 0.0)) + abs(projected - value)
    return projected
    ####


def _apply_variant_modifiers(
    values: dict[str, tuple[Any, str, str | None, Any]],
    intent: CaseIntent,
    family: FamilyPackage,
    schemas: Mapping[str, ParameterSchema],
) -> VariantResolutionReport:
    """Apply bounded semantic modifiers and return their immutable audit report."""

    space = family.variant_space
    modifiers = {modifier.id: modifier for modifier in space.modifiers}
    if len(modifiers) != len(space.modifiers):
        raise ResolutionError("duplicate-variant-modifier", "variant modifier IDs must be unique", field="variant_parameters")
    unknown = sorted(set(intent.variant_parameters) - set(modifiers))
    if unknown:
        raise ResolutionError(
            "unknown-variant-parameter",
            f"variant parameter(s) are not declared: {', '.join(unknown)}",
            field="variant_parameters",
        )

    original: dict[str, Any] = {
        key: _as_case_value(value).model_dump(mode="json") for key, value in sorted(intent.variant_parameters.items())
    }
    applied: dict[str, Any] = {}
    invalidations: set[str] = set()
    diagnostics: list[str] = []
    report_state: dict[str, Any] = {"projection_distance": 0.0}
    applied_ids: list[str] = []
    extended = False

    for modifier_id, raw in sorted(intent.variant_parameters.items()):
        modifier = modifiers[modifier_id]
        if modifier.target not in schemas:
            raise ResolutionError("unknown-variant-target", "modifier target is not declared by the family", field=modifier.target)
        target_schema = schemas[modifier.target]
        if target_schema.role == "derived":
            raise ResolutionError("derived-variant-target", "variant modifiers cannot target derived parameters", field=modifier.target)
        supplied = _as_case_value(raw)
        candidate = _convert(supplied.value, supplied.unit, modifier.canonical_unit, modifier_id)
        if not isinstance(candidate, (int, float)) or isinstance(candidate, bool):
            raise ResolutionError("variant-type-mismatch", "variant modifier candidates must be numeric", field=modifier_id)
        candidate_float = float(candidate)
        bounded = _bound_variant_value(
            candidate_float,
            modifier.minimum,
            modifier.maximum,
            policy=space.policy,
            field=modifier_id,
            report=report_state,
        )
        if modifier.qualified_minimum is not None and bounded < modifier.qualified_minimum:
            extended = True
            diagnostics.append(f"{modifier_id} below qualified minimum")
        if modifier.qualified_maximum is not None and bounded > modifier.qualified_maximum:
            extended = True
            diagnostics.append(f"{modifier_id} above qualified maximum")
        if modifier.target not in values:
            raise ResolutionError("missing-variant-target", "modifier target has no resolved base value", field=modifier.target)
        current = values[modifier.target][0]
        if not isinstance(current, (int, float)) or isinstance(current, bool):
            raise ResolutionError("variant-target-type", "modifier target must resolve to a numeric value", field=modifier.target)
        if modifier.operation == "set":
            updated = bounded
        elif modifier.operation == "add":
            updated = float(current) + bounded
        else:
            updated = float(current) * bounded
        updated = _bound_variant_value(
            float(updated),
            target_schema.minimum,
            target_schema.maximum,
            policy=space.policy,
            field=modifier.target,
            report=report_state,
        )
        if target_schema.qualified_minimum is not None and updated < target_schema.qualified_minimum:
            extended = True
            diagnostics.append(f"{modifier.target} below qualified minimum")
        if target_schema.qualified_maximum is not None and updated > target_schema.qualified_maximum:
            extended = True
            diagnostics.append(f"{modifier.target} above qualified maximum")
        values[modifier.target] = (updated, f"modifier:{modifier_id}", modifier.canonical_unit, supplied.value)
        applied[modifier_id] = bounded
        applied_ids.append(modifier_id)
        if modifier.requires_retrim or target_schema.requires_retrim:
            invalidations.add(f"retrim:{modifier.target}")
        if modifier.requires_requalification or target_schema.requires_requalification:
            invalidations.add(f"requalification:{modifier.target}")

    status: Literal["qualified", "extended", "projected"] = (
        "projected" if report_state["projection_distance"] > 0.0 else ("extended" if extended else "qualified")
    )
    fingerprint_payload = {
        "family": family.family_id,
        "family_version": family.version,
        "fidelity": intent.fidelity,
        "variant": intent.variant,
        "loadout": intent.loadout,
        "candidate_original": original,
        "candidate_applied": applied,
    }
    return VariantResolutionReport(
        status=status,
        candidate_original=original,
        candidate_applied=applied,
        projection_distance=float(report_state["projection_distance"]),
        modifiers_applied=tuple(applied_ids),
        invalidations=tuple(sorted(invalidations)),
        diagnostics=tuple(diagnostics),
        fingerprint=_identity_payload(fingerprint_payload),
    )
    ####


def _derive_variant_parameters(
    values: dict[str, tuple[Any, str, str | None, Any]],
    family: FamilyPackage,
    schemas: Mapping[str, ParameterSchema],
) -> tuple[dict[str, Any], dict[str, tuple[str, tuple[str, ...]]]]:
    """Resolve the small declarative derived-parameter graph."""

    pending = list(family.variant_space.derived)
    derived_values: dict[str, Any] = {}
    derivations: dict[str, tuple[str, tuple[str, ...]]] = {}
    while pending:
        progressed = False
        remaining: list[DerivedParameter] = []
        for definition in pending:
            if definition.id not in schemas:
                raise ResolutionError("unknown-derived-parameter", "derived parameter is not declared by the family", field=definition.id)
            if schemas[definition.id].role != "derived":
                raise ResolutionError("derived-role-mismatch", "derived graph target must have role=derived", field=definition.id)
            if any(dependency not in values for dependency in definition.dependencies):
                remaining.append(definition)
                continue
            operands = [values[dependency][0] for dependency in definition.dependencies]
            if not all(isinstance(item, (int, float)) and not isinstance(item, bool) for item in operands):
                raise ResolutionError("derived-type-mismatch", "derived operands must be numeric", field=definition.id)
            if definition.operation == "sum":
                result = sum(float(item) for item in operands) + definition.constant
            elif definition.operation == "difference":
                result = float(operands[0]) - sum(float(item) for item in operands[1:])
            elif definition.operation == "product":
                result = math.prod(float(item) for item in operands)
            elif definition.operation == "ratio":
                result = float(operands[0])
                for denominator in operands[1:]:
                    if float(denominator) == 0.0:
                        raise ResolutionError("derived-zero-division", "derived ratio denominator is zero", field=definition.id)
                    result /= float(denominator)
            else:
                result = float(operands[0]) * definition.constant
            schema = schemas[definition.id]
            _check_value(schema, result, definition.id)
            values[definition.id] = (result, f"derived:{definition.id}", schema.canonical_unit, result)
            derived_values[definition.id] = result
            derivations[definition.id] = (f"{definition.operation}({', '.join(definition.dependencies)})", definition.dependencies)
            progressed = True
        if not progressed:
            unresolved = ", ".join(item.id for item in remaining)
            raise ResolutionError("derived-cycle", f"derived parameters cannot be resolved: {unresolved}")
        pending = remaining
    return derived_values, derivations
    ####


def _select_named_layer(family: FamilyPackage, attribute: str, name: str, *, kind: str) -> Mapping[str, Any]:
    """Select a named preset and make missing names fail explicitly."""

    layers = getattr(family, attribute)
    if name not in layers:
        raise ResolutionError("unknown-preset", f"unknown {kind} preset {name!r}", field=kind)
    return layers[name]
    ####


def _identity_payload(payload: Mapping[str, Any]) -> str:
    """Hash a canonical JSON payload with stable ordering."""

    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
    ####


def load_case_intent(path: str | Path) -> CaseIntent:
    """Load a provider-neutral case intent from YAML."""

    source = Path(path)
    try:
        payload: Any = yaml.safe_load(source.read_text(encoding="utf-8"))
    except OSError as error:
        raise ResolutionError("case-read-failed", str(error)) from error
    if not isinstance(payload, Mapping):
        raise ResolutionError("case-shape", "case intent must be a YAML mapping")
    try:
        return CaseIntent.model_validate(payload)
    except ValueError as error:
        raise ResolutionError("case-invalid", str(error)) from error
    ####


def diff_resolved_cases(left: ResolvedCase, right: ResolvedCase) -> dict[str, Any]:
    """Return a stable semantic diff between two resolved cases."""

    left_payload = left.canonical_payload()
    right_payload = right.canonical_payload()
    keys = sorted(set(left_payload) | set(right_payload))
    return {
        "equal": left.identity_sha256 == right.identity_sha256,
        "left_sha256": left.identity_sha256,
        "right_sha256": right.identity_sha256,
        "mismatches": {
            key: {"left": left_payload.get(key), "right": right_payload.get(key)}
            for key in keys
            if left_payload.get(key) != right_payload.get(key)
        },
    }
    ####


def resolve_case(intent: CaseIntent, catalog: FamilyCatalog) -> ResolvedCase:
    """Resolve one case intent into canonical values and complete provenance."""

    family = catalog.family(intent.family)
    if intent.fidelity not in family.fidelities:
        raise ResolutionError("unsupported-fidelity", f"family does not advertise {intent.fidelity!r}", field="fidelity")
    if family.capabilities.fidelities and intent.fidelity not in family.capabilities.fidelities:
        raise ResolutionError("capability-fidelity", f"family capability contract does not support {intent.fidelity!r}", field="fidelity")
    schemas = family.parameter_map()
    values: dict[str, tuple[Any, str, str | None, Any]] = {}
    provenance: list[ProvenanceRecord] = []
    derivations: dict[str, tuple[str, tuple[str, ...]]] = {}

    for schema in family.parameters:
        if schema.required and schema.default is None:
            raise ResolutionError("missing-required-default", "required parameter has no default", field=schema.id)
        if schema.default is not None:
            canonical = _convert(schema.default, schema.canonical_unit, schema.canonical_unit, schema.id)
            _check_value(schema, canonical, schema.id)
            values[schema.id] = (canonical, "family.default", schema.canonical_unit, schema.default)
    _apply_layer(values, _select_named_layer(family, "variants", intent.variant, kind="variant"), f"variant:{intent.variant}", schemas)
    _apply_layer(values, _select_named_layer(family, "loadouts", intent.loadout, kind="loadout"), f"loadout:{intent.loadout}", schemas)
    _apply_layer(values, _select_named_layer(family, "missions", intent.mission, kind="mission"), f"mission:{intent.mission}", schemas)
    _apply_layer(values, _select_named_layer(family, "segment_plans", intent.segment_plan, kind="segment-plan"), f"segment-plan:{intent.segment_plan}", schemas)
    segment_graph = _select_named_layer(family, "segment_graphs", intent.segment_plan, kind="segment-graph") if family.segment_graphs else {}
    _apply_layer(values, intent.overrides, "case.override", schemas)
    variant_report = _apply_variant_modifiers(values, intent, family, schemas)
    derived_values, derivations = _derive_variant_parameters(values, family, schemas)

    control_ids = set(family.control_map())
    unknown_controls = sorted(set(intent.requested_controls) - control_ids)
    if unknown_controls:
        raise ResolutionError(
            "unknown-control",
            f"requested control(s) are not declared by the family: {', '.join(unknown_controls)}",
            field="requested_controls",
        )
    unavailable_controls = sorted(
        control.id for control in family.controls if control.id in intent.requested_controls and control.availability == "unavailable"
    )
    if unavailable_controls:
        raise ResolutionError(
            "unsupported-control",
            f"requested control(s) are unavailable for this family: {', '.join(unavailable_controls)}",
            field="requested_controls",
        )
    observation_ids = set(family.observation_map())
    unknown_observations = sorted(set(intent.requested_observations) - observation_ids)
    if unknown_observations:
        raise ResolutionError(
            "unknown-observation",
            f"requested observation(s) are not declared by the family: {', '.join(unknown_observations)}",
            field="requested_observations",
        )
    unavailable_observations = sorted(
        observation.id
        for observation in family.observations
        if observation.id in intent.requested_observations and observation.availability == "unavailable"
    )
    if unavailable_observations:
        raise ResolutionError(
            "unsupported-observation",
            f"requested observation(s) are unavailable for this family: {', '.join(unavailable_observations)}",
            field="requested_observations",
        )

    missing = sorted(schema.id for schema in family.parameters if schema.required and schema.id not in values)
    if missing:
        raise ResolutionError("missing-required", f"required parameters are not resolved: {', '.join(missing)}")
    final_parameters: dict[str, ResolvedValue] = {}
    for schema in family.parameters:
        if schema.id not in values:
            continue
        value, source, input_unit, input_value = values[schema.id]
        derivation = derivations.get(schema.id)
        final_parameters[schema.id] = ResolvedValue(value=value, unit=schema.canonical_unit, source=source)
        provenance.append(
            ProvenanceRecord(
                parameter_id=schema.id,
                source=source,
                input_unit=input_unit,
                canonical_unit=schema.canonical_unit,
                input_value=input_value,
                canonical_value=value,
                derivation=derivation[0] if derivation is not None else None,
                dependencies=derivation[1] if derivation is not None else (),
            )
        )
    variant_report = variant_report.model_copy(update={"derived_values": derived_values})
    resolved_variant = ResolvedVariant(
        family=family.family_id,
        family_version=family.version,
        fidelity=intent.fidelity,
        variant=intent.variant,
        loadout=intent.loadout,
        parameters=final_parameters,
        resolution=variant_report,
        fingerprint=variant_report.fingerprint,
    )
    payload: dict[str, Any] = {
        "schema_version": 1,
        "case_id": intent.case_id,
        "family": family.family_id,
        "family_version": family.version,
        "fidelity": intent.fidelity,
        "variant": intent.variant,
        "loadout": intent.loadout,
        "mission": intent.mission,
        "segment_plan": intent.segment_plan,
        "controller": intent.controller,
        "parameters": {key: value.model_dump(mode="json") for key, value in sorted(final_parameters.items())},
        "controls": [item.model_dump(mode="json") for item in family.controls],
        "observations": [item.model_dump(mode="json") for item in family.observations],
        "capabilities": family.capabilities.model_dump(mode="json"),
        "component_slots": [item.model_dump(mode="json") for item in family.component_slots],
        "resources": [item.model_dump(mode="json") for item in family.resources],
        "allocations": [item.model_dump(mode="json") for item in family.allocations],
        "mode_transitions": [item.model_dump(mode="json") for item in family.mode_transitions],
        "evidence_grade": family.evidence_grade,
        "uncertainty": family.uncertainty,
        "provenance": [item.model_dump(mode="json") for item in provenance],
        "segment_graph": segment_graph,
        "extensions": intent.extensions,
        "resolved_variant": resolved_variant.model_dump(mode="json"),
    }
    identity = _identity_payload(payload)
    return ResolvedCase(
        **payload,
        identity_sha256=identity,
    )
    ####


__all__ = ["ResolutionError", "diff_resolved_cases", "load_case_intent", "resolve_case"]
####
