"""Deterministic, loss-aware semantic IR and exporter for DAVE-ML XML."""

from __future__ import annotations

import hashlib
import json
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Any


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
    opaque_paths: tuple[str, ...]

    def to_dict(self) -> dict[str, object]:
        """Return the deterministic IR payload."""

        return {
            "schema_version": self.schema_version,
            "document_id": self.document_id,
            "source_sha256": self.source_sha256,
            "root": self.root,
            "semantic": self.semantic,
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
    return DAVEMLIR(
        schema_version="taoryx.daveml-ir/v1",
        document_id=document_id,
        source_sha256=hashlib.sha256(payload).hexdigest(),
        root=semantic_root,
        semantic=semantic,
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

    def visit(node: dict[str, object]) -> None:
        tag = str(node.get("tag", ""))
        raw_attributes = node.get("attributes", {})
        attributes = dict(raw_attributes) if isinstance(raw_attributes, dict) else {}
        raw_children = node.get("children", [])
        children = [child for child in raw_children if isinstance(child, dict)] if isinstance(raw_children, list) else []
        for name, tags in groups.items():
            if tag in tags:
                result[name].append(
                    {
                        "source_path": node.get("source_path"),
                        "tag": tag,
                        "attributes": attributes,
                        "text": node.get("text"),
                        "child_tags": [str(child.get("tag", "")) for child in children],
                    }
                )
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


def _dimension_for_unit(unit: str) -> str:
    """Return a stable dimension signature for common DAVE-ML source units."""

    dimensions = {
        "nd": "1",
        "deg": "angle",
        "rad": "angle",
        "deg_rad": "angle",
        "rad_s": "angle/time",
        "f_s": "length/time",
        "f": "length",
        "fracMAC": "1",
        "slug": "mass",
        "slug_ft2": "mass*length^2",
        "lb": "force",
        "lbf": "force",
        "s": "time",
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
    "DAVEMLDiff",
    "DAVEMLIR",
    "build_daveml_ir",
    "compare_daveml_ir",
    "compare_daveml_numeric",
    "export_daveml_ir",
]
