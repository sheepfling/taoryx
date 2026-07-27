"""Deterministic, loss-aware semantic IR and exporter for DAVE-ML XML."""

from __future__ import annotations

import hashlib
import json
import re
import xml.etree.ElementTree as ET
from collections.abc import Iterator
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True, slots=True)
class DAVEMLUnitRecord:
    """Typed source-anchored unit declaration."""

    source_path: str
    identifier: str
    unit: str
    dimension: str

    def to_dict(self) -> dict[str, object]:
        return {
            "source_path": self.source_path,
            "identifier": self.identifier,
            "unit": self.unit,
            "dimension": self.dimension,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class DAVEMLVariableRecord:
    """Typed variable definition with parsed initial values."""

    source_path: str
    tag: str
    identifier: str
    units: str | None
    dimension: str | None
    initial_values: tuple[float, ...]
    description: str | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "source_path": self.source_path,
            "tag": self.tag,
            "identifier": self.identifier,
            "units": self.units,
            "dimension": self.dimension,
            "initial_values": list(self.initial_values),
            "description": self.description,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class DAVEMLReferenceRecord:
    """Typed local or external reference disposition."""

    source_path: str
    tag: str
    identifier: str | None
    status: str
    target_path: str | None = None
    target_tag: str | None = None
    resolved_identifier_kind: str | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "source_path": self.source_path,
            "tag": self.tag,
            "identifier": self.identifier,
            "status": self.status,
            "target_path": self.target_path,
            "target_tag": self.target_tag,
            "resolved_identifier_kind": self.resolved_identifier_kind,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class DAVEMLFunctionRecord:
    """Typed function boundary and its reference dispositions."""

    source_path: str
    tag: str
    references: tuple[DAVEMLReferenceRecord, ...]
    input_ids: tuple[str, ...] = ()
    output_ids: tuple[str, ...] = ()
    dependency_ids: tuple[str, ...] = ()
    expression_tags: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, object]:
        return {
            "source_path": self.source_path,
            "tag": self.tag,
            "references": [reference.to_dict() for reference in self.references],
            "input_ids": list(self.input_ids),
            "output_ids": list(self.output_ids),
            "dependency_ids": list(self.dependency_ids),
            "expression_tags": list(self.expression_tags),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class DAVEMLTableRecord:
    """Typed table definition boundary with source identity."""

    source_path: str
    tag: str
    identifier: str | None
    independent_variable_ids: tuple[str, ...]
    dependent_variable_ids: tuple[str, ...]
    axes: tuple[tuple[str, tuple[float, ...]], ...] = ()
    interpolation: str | None = None
    extrapolation: str | None = None
    boundary_policy: str | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "source_path": self.source_path,
            "tag": self.tag,
            "identifier": self.identifier,
            "independent_variable_ids": list(self.independent_variable_ids),
            "dependent_variable_ids": list(self.dependent_variable_ids),
            "axes": [{"identifier": identifier, "values": list(values)} for identifier, values in self.axes],
            "interpolation": self.interpolation,
            "extrapolation": self.extrapolation,
            "boundary_policy": self.boundary_policy,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class DAVEMLCheckSignalRecord:
    """Typed expected signal values from one check case."""

    source_path: str
    signal_id: str
    values: tuple[float, ...]
    tolerance: float | None

    def to_dict(self) -> dict[str, object]:
        return {
            "source_path": self.source_path,
            "signal_id": self.signal_id,
            "values": list(self.values),
            "tolerance": self.tolerance,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class DAVEMLCheckRecord:
    """Typed checkData/check-case signal inventory."""

    source_path: str
    tag: str
    case_name: str | None
    signals: tuple[DAVEMLCheckSignalRecord, ...]

    @property
    def signal_ids(self) -> tuple[str, ...]:
        """Return signal IDs in source order."""

        return tuple(signal.signal_id for signal in self.signals)

    def to_dict(self) -> dict[str, object]:
        return {
            "source_path": self.source_path,
            "tag": self.tag,
            "case_name": self.case_name,
            "signals": [signal.to_dict() for signal in self.signals],
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class DAVEMLOpaqueFeatureRecord:
    """Typed diagnostic for an extension retained only by the lossless tree."""

    source_path: str
    tag: str
    disposition: str = "retained_lossless_not_executable"

    def to_dict(self) -> dict[str, object]:
        return {
            "source_path": self.source_path,
            "tag": self.tag,
            "disposition": self.disposition,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class DAVEMLComponentRecord:
    """Typed component boundary inferred from DAVE-ML input/output markers."""

    source_path: str
    component_id: str | None
    input_ids: tuple[str, ...]
    output_ids: tuple[str, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "source_path": self.source_path,
            "component_id": self.component_id,
            "input_ids": list(self.input_ids),
            "output_ids": list(self.output_ids),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class DAVEMLTypedSemantic:
    """Deterministic typed index over one source-anchored DAVE-ML document."""

    variables: tuple[DAVEMLVariableRecord, ...]
    units: tuple[DAVEMLUnitRecord, ...]
    functions: tuple[DAVEMLFunctionRecord, ...]
    tables: tuple[DAVEMLTableRecord, ...]
    references: tuple[DAVEMLReferenceRecord, ...]
    checks: tuple[DAVEMLCheckRecord, ...]
    opaque_features: tuple[DAVEMLOpaqueFeatureRecord, ...]
    components: tuple[DAVEMLComponentRecord, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "variables": [item.to_dict() for item in self.variables],
            "units": [item.to_dict() for item in self.units],
            "functions": [item.to_dict() for item in self.functions],
            "tables": [item.to_dict() for item in self.tables],
            "references": [item.to_dict() for item in self.references],
            "checks": [item.to_dict() for item in self.checks],
            "opaque_features": [item.to_dict() for item in self.opaque_features],
            "components": [item.to_dict() for item in self.components],
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class DAVEMLDiff:
    """One typed difference between two semantic IR documents."""

    kind: str
    path: str
    expected: object
    actual: object
    disposition: str

    def to_dict(self) -> dict[str, object]:
        """Return a JSON-safe difference record."""

        return {
            "kind": self.kind,
            "path": self.path,
            "expected": self.expected,
            "actual": self.actual,
            "disposition": self.disposition,
        }
        ####
####


@dataclass(frozen=True, slots=True)
class DAVEMLIR:
    """Canonical semantic representation of one DAVE-ML document."""

    schema_version: str
    document_id: str
    source_sha256: str
    root: dict[str, object]
    semantic: dict[str, object]
    typed: DAVEMLTypedSemantic
    opaque_paths: tuple[str, ...]

    def to_dict(self) -> dict[str, object]:
        """Return the deterministic IR payload."""

        return {
            "schema_version": self.schema_version,
            "document_id": self.document_id,
            "source_sha256": self.source_sha256,
            "root": self.root,
            "semantic": self.semantic,
            "typed": self.typed.to_dict(),
            "opaque_paths": list(self.opaque_paths),
        }
        ####

    def canonical_json(self) -> bytes:
        """Return stable UTF-8 JSON bytes."""

        return (json.dumps(self.to_dict(), indent=2, sort_keys=True) + "\n").encode("utf-8")
        ####
####


def build_daveml_ir(payload: bytes, *, document_id: str) -> DAVEMLIR:
    """Parse XML into a deterministic, source-anchored semantic tree."""

    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.fromstring(payload, parser=parser)
    opaque: list[str] = []
    semantic_root = _node(root, "/0", opaque)
    semantic = _semantic_projection(semantic_root)
    typed = _typed_projection(semantic_root, semantic, tuple(opaque))
    return DAVEMLIR(
        schema_version="taoryx.daveml-ir/v1",
        document_id=document_id,
        source_sha256=hashlib.sha256(payload).hexdigest(),
        root=semantic_root,
        semantic=semantic,
        typed=typed,
        opaque_paths=tuple(opaque),
    )
    ####


def export_daveml_ir(ir: DAVEMLIR) -> bytes:
    """Export IR as deterministic canonical XML bytes."""

    root = _element(ir.root)
    payload = ET.tostring(root, encoding="utf-8", short_empty_elements=True)
    return b'<?xml version="1.0" encoding="UTF-8"?>\n' + payload + b"\n"
    ####


def compare_daveml_ir(expected: DAVEMLIR, actual: DAVEMLIR) -> tuple[DAVEMLDiff, ...]:
    """Compare two IR trees while treating generated source hashes as metadata."""

    differences: list[DAVEMLDiff] = []
    _compare_node(expected.root, actual.root, "/0", differences)
    if expected.semantic != actual.semantic:
        differences.append(DAVEMLDiff("semantic", "/semantic", expected.semantic, actual.semantic, "review"))
    if expected.typed != actual.typed:
        differences.append(DAVEMLDiff("semantic", "/typed", expected.typed.to_dict(), actual.typed.to_dict(), "review"))
    if expected.document_id != actual.document_id:
        differences.append(DAVEMLDiff("provenance", "/document_id", expected.document_id, actual.document_id, "review"))
    return tuple(differences)
    ####


def compare_daveml_numeric(
    expected: DAVEMLIR,
    actual: DAVEMLIR,
    *,
    absolute_tolerance: float = 1.0e-12,
    relative_tolerance: float = 1.0e-10,
) -> tuple[DAVEMLDiff, ...]:
    """Compare numeric attributes and text vectors by source path."""

    if absolute_tolerance < 0.0 or relative_tolerance < 0.0:
        raise ValueError("numeric tolerances must be nonnegative")
    left = _numeric_values(expected.root)
    right = _numeric_values(actual.root)
    differences: list[DAVEMLDiff] = []
    for path in sorted(set(left) | set(right)):
        expected_values = left.get(path, ())
        actual_values = right.get(path, ())
        if len(expected_values) != len(actual_values):
            differences.append(DAVEMLDiff("numeric", path, list(expected_values), list(actual_values), "review"))
            continue
        for index, (expected_value, actual_value) in enumerate(zip(expected_values, actual_values, strict=True)):
            tolerance = absolute_tolerance + relative_tolerance * max(abs(expected_value), abs(actual_value))
            if abs(expected_value - actual_value) > tolerance:
                differences.append(
                    DAVEMLDiff(
                        "numeric",
                        f"{path}/{index}",
                        expected_value,
                        actual_value,
                        "review",
                    )
                )
    return tuple(differences)
    ####


def _node(element: ET.Element, path: str, opaque: list[str]) -> dict[str, object]:
    """Convert one XML element into a stable IR node."""

    tag = _tag(element.tag)
    if _is_opaque_tag(tag):
        opaque.append(path)
    attributes = {str(key): str(value) for key, value in sorted(element.attrib.items())}
    children = [_node(child, f"{path}/{index}", opaque) for index, child in enumerate(list(element))]
    text = _normalize_text(element.text)
    return {
        "source_path": path,
        "tag": tag,
        "attributes": attributes,
        "text": text,
        "children": children,
    }
    ####


def _semantic_projection(root: dict[str, object]) -> dict[str, object]:
    """Project source-anchored nodes into a stable, vocabulary-level index."""

    groups = {
        "variables": {"variableDef", "variable"},
        "breakpoints": {"breakpointDef", "breakpoint"},
        "functions": {"function", "functionDefn", "calculation"},
        "tables": {"griddedTable", "griddedTableDef", "ungriddedTable", "ungriddedTableDef"},
        "checks": {"checkData", "checkCase", "check"},
    }
    result: dict[str, list[dict[str, object]]] = {name: [] for name in groups}
    result["units"] = []
    result["vectors"] = []

    identifiers: dict[tuple[str, str], list[str]] = {}

    def index(node: dict[str, object]) -> None:
        tag = str(node.get("tag", ""))
        attributes = node.get("attributes", {})
        if not tag.endswith("Ref") and isinstance(attributes, dict):
            for attribute in ("varID", "gtID", "utID", "bpID", "name"):
                value = attributes.get(attribute)
                if value:
                    identifiers.setdefault((attribute, str(value).strip()), []).append(str(node.get("source_path", "")))
        children = node.get("children", [])
        if isinstance(children, list):
            for child in children:
                if isinstance(child, dict):
                    index(child)

    index(root)

    def descendants(node: dict[str, object]) -> list[dict[str, object]]:
        found: list[dict[str, object]] = []
        children = node.get("children", [])
        if not isinstance(children, list):
            return found
        for child in children:
            if isinstance(child, dict):
                found.append(child)
                found.extend(descendants(child))
        return found

    def node_at_path(path: str) -> dict[str, object] | None:
        """Find one source-anchored node by its deterministic path."""

        if path == "/0":
            return root
        node = root
        for component in path.removeprefix("/0/").split("/"):
            children = node.get("children", [])
            if not isinstance(children, list) or not component.isdigit() or int(component) >= len(children):
                return None
            child = children[int(component)]
            if not isinstance(child, dict):
                return None
            node = child
        return node

    def references(node: dict[str, object]) -> list[dict[str, object]]:
        resolved: list[dict[str, object]] = []
        for child in descendants(node):
            tag = str(child.get("tag", ""))
            attributes = child.get("attributes", {})
            if not tag.endswith("Ref") or not isinstance(attributes, dict):
                continue
            if tag in {"documentRef", "modificationRef"} or attributes.get("refID"):
                resolved.append(
                    {
                        "source_path": child.get("source_path"),
                        "tag": tag,
                        "identifier": str(attributes.get("refID", "")).strip() or None,
                        "status": "external_provenance",
                    }
                )
                continue
            key_attribute = next((name for name in ("varID", "gtID", "utID", "bpID", "name") if attributes.get(name)), None)
            if key_attribute is None:
                resolved.append({"source_path": child.get("source_path"), "tag": tag, "status": "unresolved"})
                continue
            key = (key_attribute, str(attributes[key_attribute]).strip())
            targets = identifiers.get(key, [])
            legacy_key = None
            if not targets and key_attribute == "gtID":
                legacy_key = ("utID", key[1])
                targets = identifiers.get(legacy_key, [])
            target_tag = None
            if len(targets) == 1:
                target = node_at_path(targets[0])
                target_tag = str(target.get("tag", "")) if target is not None else None
            type_mismatch = len(targets) == 1 and (
                tag == "griddedTableRef" and target_tag == "ungriddedTableDef"
                or tag == "ungriddedTableRef" and target_tag in {"griddedTableDef", "griddedTable"}
            )
            resolved.append(
                {
                    "source_path": child.get("source_path"),
                    "tag": tag,
                    "identifier": key[1],
                    "resolved_identifier_kind": legacy_key[0] if legacy_key is not None and targets else key_attribute,
                    "target_path": targets[0] if len(targets) == 1 else None,
                    "status": "type_mismatch" if type_mismatch else "resolved" if len(targets) == 1 else "ambiguous" if targets else "unresolved",
                    "target_tag": target_tag,
                }
            )
        return resolved

    def visit(node: dict[str, object]) -> None:
        tag = str(node.get("tag", ""))
        raw_attributes = node.get("attributes", {})
        attributes = dict(raw_attributes) if isinstance(raw_attributes, dict) else {}
        raw_children = node.get("children", [])
        children = [child for child in raw_children if isinstance(child, dict)] if isinstance(raw_children, list) else []
        for name, tags in groups.items():
            if tag in tags:
                entry = {
                    "source_path": node.get("source_path"),
                    "tag": tag,
                    "attributes": attributes,
                    "text": node.get("text"),
                    "child_tags": [str(child.get("tag", "")) for child in children],
                }
                if tag == "function":
                    entry["references"] = references(node)
                result[name].append(entry)
                break
        unit_name = attributes.get("units", attributes.get("unit"))
        identifier = attributes.get("varID", attributes.get("name"))
        if unit_name and identifier:
            result["units"].append(
                {
                    "source_path": node.get("source_path"),
                    "identifier": identifier,
                    "unit": unit_name,
                    "dimension": _dimension_for_unit(unit_name),
                }
            )
        initial = attributes.get("initialValue")
        if initial and identifier:
            values = _parse_numbers(initial)
            if len(values) > 1:
                result["vectors"].append(
                    {
                        "source_path": node.get("source_path"),
                        "identifier": identifier,
                        "values": list(values),
                        "unit": unit_name,
                    }
                )
        for child in children:
            visit(child)

    visit(root)
    return {name: entries for name, entries in result.items()}


def _typed_projection(
    root: dict[str, object],
    semantic: dict[str, object],
    opaque_paths: tuple[str, ...],
) -> DAVEMLTypedSemantic:
    """Build typed records from the source-anchored semantic tree."""

    nodes = tuple(_walk_nodes(root))
    raw_functions = semantic.get("functions", [])
    if not isinstance(raw_functions, list):
        raw_functions = []
    function_entries = {
        str(entry.get("source_path")): entry
        for entry in raw_functions
        if isinstance(entry, dict)
    }
    variables: list[DAVEMLVariableRecord] = []
    units: list[DAVEMLUnitRecord] = []
    functions: list[DAVEMLFunctionRecord] = []
    tables: list[DAVEMLTableRecord] = []
    references: list[DAVEMLReferenceRecord] = []
    checks: list[DAVEMLCheckRecord] = []
    breakpoint_values = {
        _node_identifier(node): _first_values(node, {"bpVals", "breakpointValues"})
        for node in nodes
        if str(node.get("tag", "")) == "breakpointDef" and _node_identifier(node)
    }

    for node in nodes:
        tag = str(node.get("tag", ""))
        path = str(node.get("source_path", ""))
        attributes = node.get("attributes", {})
        if not isinstance(attributes, dict):
            attributes = {}
        identifier = str(attributes.get("varID") or attributes.get("name") or "").strip()
        unit = str(attributes.get("units") or attributes.get("unit") or "").strip() or None
        if tag in {"variableDef", "variable"} and identifier:
            initial = str(attributes.get("initialValue", ""))
            variables.append(
                DAVEMLVariableRecord(
                    source_path=path,
                    tag=tag,
                    identifier=identifier,
                    units=unit,
                    dimension=_dimension_for_unit(unit) if unit else None,
                    initial_values=_parse_numbers(initial) if initial else (),
                    description=_first_text(node, {"description", "varDescription"}),
                )
            )
        if unit and identifier:
            units.append(DAVEMLUnitRecord(path, identifier, unit, _dimension_for_unit(unit)))
        if tag in {"function", "functionDefn", "calculation"}:
            entry = function_entries.get(path, {})
            raw_references = entry.get("references", []) if isinstance(entry, dict) else []
            typed_references = tuple(
                _reference_record(item)
                for item in raw_references
                if isinstance(item, dict)
            )
            input_ids: list[str] = []
            output_ids: list[str] = []
            expression_tags: list[str] = []
            for child in _walk_nodes(node):
                child_tag = str(child.get("tag", ""))
                child_id = _node_identifier(child)
                if child_id and ("independent" in child_tag.casefold() or child_tag in {"inputVar", "input"}):
                    input_ids.append(child_id)
                elif child_id and ("dependent" in child_tag.casefold() or child_tag in {"outputVar", "output"}):
                    output_ids.append(child_id)
                if child_tag in {"math", "sum", "product", "quotient", "difference", "negate"}:
                    expression_tags.append(child_tag)
            dependency_ids = [reference.identifier for reference in typed_references if reference.identifier]
            functions.append(
                DAVEMLFunctionRecord(
                    path,
                    tag,
                    typed_references,
                    tuple(dict.fromkeys(input_ids)),
                    tuple(dict.fromkeys(output_ids)),
                    tuple(dict.fromkeys(str(value) for value in dependency_ids)),
                    tuple(dict.fromkeys(expression_tags)),
                )
            )
            references.extend(typed_references)
        if tag in {"griddedTable", "griddedTableDef", "ungriddedTable", "ungriddedTableDef"}:
            table_id = str(attributes.get("gtID") or attributes.get("utID") or attributes.get("name") or "").strip() or None
            independent: list[str] = []
            dependent: list[str] = []
            axes: list[tuple[str, tuple[float, ...]]] = []
            for child in _walk_nodes(node):
                child_tag = str(child.get("tag", ""))
                child_attributes = child.get("attributes", {})
                if not isinstance(child_attributes, dict):
                    continue
                child_id = str(
                    child_attributes.get("varID")
                    or child_attributes.get("bpID")
                    or child_attributes.get("independentVarID")
                    or child_attributes.get("dependentVarID")
                    or ""
                ).strip()
                if not child_id:
                    continue
                if "independent" in child_tag.casefold() or child_tag in {"bpRef", "breakpointRef", "independentVarPts"}:
                    independent.append(child_id)
                elif "dependent" in child_tag.casefold() or child_tag == "dependentVarPts":
                    dependent.append(child_id)
                values = breakpoint_values.get(child_id, ()) or _node_values(child)
                if values and (
                    "independent" in child_tag.casefold()
                    or "breakpoint" in child_tag.casefold()
                    or child_tag in {"bpRef", "breakpointRef"}
                ):
                    axes.append((child_id, values))
            tables.append(
                DAVEMLTableRecord(
                    path,
                    tag,
                    table_id,
                    tuple(dict.fromkeys(independent)),
                    tuple(dict.fromkeys(dependent)),
                    tuple(axes),
                    _first_policy(node, "interpolation"),
                    _first_policy(node, "extrapolation"),
                    _first_policy(node, "boundary", "limit", "limiting"),
                )
            )
        if tag in {"checkData", "checkCase", "check", "staticShot"}:
            signals: list[DAVEMLCheckSignalRecord] = []
            for child in _walk_nodes(node):
                if str(child.get("tag", "")) != "signal":
                    continue
                signal_id = _first_text(child, {"signalID", "varID"}) or _node_attribute(child, "signalID") or _node_attribute(child, "varID")
                if not signal_id:
                    continue
                values = _first_values(child, {"signalValue"})
                tolerance_values = _first_values(child, {"tol", "tolerance"})
                signals.append(
                    DAVEMLCheckSignalRecord(
                        source_path=str(child.get("source_path", "")),
                        signal_id=signal_id,
                        values=values,
                        tolerance=tolerance_values[0] if tolerance_values else None,
                    )
                )
            checks.append(
                DAVEMLCheckRecord(
                    source_path=path,
                    tag=tag,
                    case_name=str(attributes.get("name") or attributes.get("caseID") or "").strip() or None,
                    signals=tuple(signals),
                )
            )

    opaque = []
    for path in opaque_paths:
        opaque_node = _node_at_path(root, path)
        opaque.append(DAVEMLOpaqueFeatureRecord(path, str(opaque_node.get("tag", "#unknown")) if opaque_node else "#unknown"))
    root_attributes = root.get("attributes", {})
    component_id = str(root_attributes.get("name", "")).strip() if isinstance(root_attributes, dict) else ""
    root_input_ids: list[str] = []
    root_output_ids: list[str] = []
    for node in nodes:
        if str(node.get("tag", "")) not in {"variableDef", "variable"}:
            continue
        identifier = _node_identifier(node) or ""
        if not identifier:
            continue
        if _has_descendant_tag(node, "isInput"):
            root_input_ids.append(identifier)
        if _has_descendant_tag(node, "isOutput"):
            root_output_ids.append(identifier)
    components = (
        DAVEMLComponentRecord(
            "/0",
            component_id or None,
            tuple(dict.fromkeys(root_input_ids)),
            tuple(dict.fromkeys(root_output_ids)),
        ),
    )
    return DAVEMLTypedSemantic(
        variables=tuple(variables),
        units=tuple(units),
        functions=tuple(functions),
        tables=tuple(tables),
        references=tuple(references),
        checks=tuple(checks),
        opaque_features=tuple(opaque),
        components=components,
    )


def _walk_nodes(node: dict[str, object]) -> Iterator[dict[str, object]]:
    """Yield one IR node and all descendants in source order."""

    yield node
    children = node.get("children", [])
    if isinstance(children, list):
        for child in children:
            if isinstance(child, dict):
                yield from _walk_nodes(child)


def _node_at_path(root: dict[str, object], path: str) -> dict[str, object] | None:
    """Resolve one source path in the deterministic IR tree."""

    if path == "/0":
        return root
    node = root
    for component in path.removeprefix("/0/").split("/"):
        children = node.get("children", [])
        if not isinstance(children, list) or not component.isdigit() or int(component) >= len(children):
            return None
        child = children[int(component)]
        if not isinstance(child, dict):
            return None
        node = child
    return node


def _reference_record(value: dict[str, object]) -> DAVEMLReferenceRecord:
    """Convert one dictionary reference projection into a typed record."""

    return DAVEMLReferenceRecord(
        source_path=str(value.get("source_path", "")),
        tag=str(value.get("tag", "")),
        identifier=str(value["identifier"]) if value.get("identifier") is not None else None,
        status=str(value.get("status", "unresolved")),
        target_path=str(value["target_path"]) if value.get("target_path") is not None else None,
        target_tag=str(value["target_tag"]) if value.get("target_tag") is not None else None,
        resolved_identifier_kind=str(value["resolved_identifier_kind"])
        if value.get("resolved_identifier_kind") is not None
        else None,
    )


def _node_attribute(node: dict[str, object], name: str) -> str | None:
    """Return one trimmed node attribute when present."""

    attributes = node.get("attributes", {})
    if not isinstance(attributes, dict) or attributes.get(name) is None:
        return None
    value = str(attributes[name]).strip()
    return value or None


def _node_identifier(node: dict[str, object]) -> str | None:
    """Return the first typed identifier carried by one node."""

    for name in ("varID", "bpID", "gtID", "utID", "signalID", "independentVarID", "dependentVarID", "name"):
        value = _node_attribute(node, name)
        if value:
            return value
    return None


def _first_text(node: dict[str, object], tags: set[str]) -> str | None:
    """Return the first non-empty text from a descendant with one of the tags."""

    for child in _walk_nodes(node):
        if str(child.get("tag", "")) in tags and child.get("text"):
            value = str(child["text"]).strip()
            if value:
                return value
    return None


def _first_values(node: dict[str, object], tags: set[str]) -> tuple[float, ...]:
    """Return numeric values from the first matching descendant."""

    for child in _walk_nodes(node):
        if str(child.get("tag", "")) in tags:
            text = str(child.get("text", ""))
            values = _parse_numbers(text)
            if values:
                return values
            for attribute in ("value", "values", "tol", "tolerance"):
                value = _node_attribute(child, attribute)
                if value:
                    parsed = _parse_numbers(value)
                    if parsed:
                        return parsed
    return ()


def _node_values(node: dict[str, object]) -> tuple[float, ...]:
    """Return knot values carried by a table-axis node."""

    text = str(node.get("text", ""))
    if text:
        values = _parse_numbers(text)
        if values:
            return values
    return _first_values(node, {"values", "breakpointValues", "independentVarPts"})


def _first_policy(node: dict[str, object], *names: str) -> str | None:
    """Return the first interpolation or boundary policy marker."""

    for child in _walk_nodes(node):
        attributes = child.get("attributes", {})
        if isinstance(attributes, dict):
            for name in names:
                for key in (name, f"{name}Policy", f"{name}Method"):
                    value = attributes.get(key)
                    if value:
                        return str(value).strip()
        tag = str(child.get("tag", ""))
        if any(name.casefold() in tag.casefold() for name in names) and child.get("text"):
            return str(child["text"]).strip()
    return None


def _has_descendant_tag(node: dict[str, object], tag: str) -> bool:
    """Return whether one node contains a marker tag."""

    return any(str(child.get("tag", "")) == tag for child in _walk_nodes(node))


def _dimension_for_unit(unit: str) -> str:
    """Return a stable dimension signature for common DAVE-ML source units."""

    dimensions = {
        "nd": "1",
        "deg": "angle",
        "rad": "angle",
        "deg_rad": "angle",
        "rad_s": "angle/time",
        "f_s": "length/time",
        "ft_s": "length/time",
        "f": "length",
        "ft": "length",
        "f2": "length^2",
        "fracMAC": "1",
        "slug": "mass",
        "slug_ft2": "mass*length^2",
        "lb": "force",
        "lbf": "force",
        "s": "time",
        "s_rad": "time/angle",
    }
    return dimensions.get(unit.strip(), "unknown")


def _element(node: dict[str, object]) -> Any:
    """Recreate one XML element from IR."""

    tag = str(node["tag"])
    if tag == "#comment":
        return ET.Comment(str(node.get("text") or ""))
    raw_attributes = node.get("attributes", {})
    attributes = raw_attributes if isinstance(raw_attributes, dict) else {}
    element = ET.Element(tag, {str(key): str(value) for key, value in attributes.items()})
    text = node.get("text")
    if text:
        element.text = str(text)
    children = node.get("children", [])
    if isinstance(children, list):
        for child in children:
            if isinstance(child, dict):
                element.append(_element(child))
    return element
    ####


def _compare_node(expected: Any, actual: Any, path: str, differences: list[DAVEMLDiff]) -> None:
    """Compare node identity and recursively compare children."""

    for field in ("tag", "attributes", "text"):
        if expected.get(field) != actual.get(field):
            differences.append(DAVEMLDiff("structure", f"{path}/{field}", expected.get(field), actual.get(field), "review"))
    expected_children = list(expected.get("children", []))
    actual_children = list(actual.get("children", []))
    if len(expected_children) != len(actual_children):
        differences.append(DAVEMLDiff("structure", f"{path}/children", len(expected_children), len(actual_children), "review"))
    for index, (left, right) in enumerate(zip(expected_children, actual_children, strict=False)):
        _compare_node(dict(left), dict(right), f"{path}/{index}", differences)
    ####


def _numeric_values(node: Any) -> dict[str, tuple[float, ...]]:
    """Collect numeric values from one IR tree without changing source text."""

    values: dict[str, tuple[float, ...]] = {}
    path = str(node["source_path"])
    for name, value in dict(node.get("attributes", {})).items():
        numbers = _parse_numbers(str(value))
        if numbers:
            values[f"{path}/attributes/{name}"] = numbers
    text = node.get("text")
    if text:
        numbers = _parse_numbers(str(text))
        if numbers:
            values[f"{path}/text"] = numbers
    for child in node.get("children", []):
        values.update(_numeric_values(dict(child)))
    return values
    ####


def _parse_numbers(value: str) -> tuple[float, ...]:
    """Parse standalone decimal/scientific numbers from an XML value."""

    result: list[float] = []
    for token in re.findall(r"(?<![A-Za-z])[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?(?![A-Za-z])", value):
        try:
            result.append(float(token))
        except ValueError:
            continue
    return tuple(result)
    ####


def _tag(tag: object) -> str:
    """Convert ElementTree namespace notation to a stable local tag."""

    if not isinstance(tag, str):
        return "#comment"
    return re.sub(r"^\{[^}]+\}", "", tag)
    ####


def _normalize_text(text: str | None) -> str | None:
    """Normalize insignificant whitespace while retaining meaningful text."""

    if text is None:
        return None
    normalized = " ".join(text.split())
    return normalized or None
    ####


def _is_opaque_tag(tag: str) -> bool:
    """Flag explicitly extension-like nodes without guessing DAVE-ML vocabulary."""

    lowered = tag.casefold()
    return any(marker in lowered for marker in ("extension", "vendor", "unknown"))
    ####


__all__ = [
    "DAVEMLCheckRecord",
    "DAVEMLCheckSignalRecord",
    "DAVEMLComponentRecord",
    "DAVEMLDiff",
    "DAVEMLFunctionRecord",
    "DAVEMLIR",
    "DAVEMLOpaqueFeatureRecord",
    "DAVEMLReferenceRecord",
    "DAVEMLTableRecord",
    "DAVEMLTypedSemantic",
    "DAVEMLUnitRecord",
    "DAVEMLVariableRecord",
    "build_daveml_ir",
    "compare_daveml_ir",
    "compare_daveml_numeric",
    "export_daveml_ir",
]
