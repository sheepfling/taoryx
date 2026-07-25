"""Alpha 2 case resolution, canonical units, and provenance."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import yaml

from .contracts import CaseIntent, CaseValue, FamilyCatalog, FamilyPackage, ProvenanceRecord, ResolvedCase, ResolvedValue


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
        supplied = _as_case_value(raw)
        canonical = _convert(supplied.value, supplied.unit, schemas[parameter_id].canonical_unit, parameter_id)
        _check_value(schemas[parameter_id], canonical, parameter_id)
        values[parameter_id] = (canonical, source, supplied.unit, supplied.value)
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
    schemas = family.parameter_map()
    values: dict[str, tuple[Any, str, str | None, Any]] = {}
    provenance: list[ProvenanceRecord] = []

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

    control_ids = set(family.control_map())
    unknown_controls = sorted(set(intent.requested_controls) - control_ids)
    if unknown_controls:
        raise ResolutionError(
            "unknown-control",
            f"requested control(s) are not declared by the family: {', '.join(unknown_controls)}",
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

    missing = sorted(schema.id for schema in family.parameters if schema.required and schema.id not in values)
    if missing:
        raise ResolutionError("missing-required", f"required parameters are not resolved: {', '.join(missing)}")
    final_parameters: dict[str, ResolvedValue] = {}
    for schema in family.parameters:
        if schema.id not in values:
            continue
        value, source, input_unit, input_value = values[schema.id]
        final_parameters[schema.id] = ResolvedValue(value=value, unit=schema.canonical_unit, source=source)
        provenance.append(
            ProvenanceRecord(
                parameter_id=schema.id,
                source=source,
                input_unit=input_unit,
                canonical_unit=schema.canonical_unit,
                input_value=input_value,
                canonical_value=value,
                derivation=None,
            )
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
        "provenance": [item.model_dump(mode="json") for item in provenance],
        "segment_graph": segment_graph,
        "extensions": intent.extensions,
    }
    identity = _identity_payload(payload)
    return ResolvedCase(
        **payload,
        identity_sha256=identity,
    )
    ####


__all__ = ["ResolutionError", "diff_resolved_cases", "load_case_intent", "resolve_case"]
####
