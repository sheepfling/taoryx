from __future__ import annotations

import io
import zipfile

import pytest

from taoryx.trajectory import (
    build_daveml_ir,
    compare_daveml_ir,
    compare_daveml_numeric,
    export_daveml_ir,
)

CORPUS = "resources/aerospace/daveml/taoryx-corpus-v1.1/corpus.zip"


def test_semantic_ir_is_deterministic_and_retains_unknown_nodes() -> None:
    """Canonical IR retains comments and unsupported extension paths."""

    source = b"<?xml version='1.0'?><DAVEfunc><!-- keep --><vendorExtension answer='42'> x </vendorExtension><variable varID='A' /></DAVEfunc>"
    first = build_daveml_ir(source, document_id="fixture")
    second = build_daveml_ir(source, document_id="fixture")

    assert first.canonical_json() == second.canonical_json()
    assert first.opaque_paths == ("/0/1",)
    assert first.semantic["variables"][0]["attributes"]["varID"] == "A"
    exported = export_daveml_ir(first)
    assert compare_daveml_ir(first, build_daveml_ir(exported, document_id="fixture")) == ()
    assert b"vendorExtension" in exported
    assert b"keep" in exported
    ####


def test_numeric_comparison_reports_changed_seed_value() -> None:
    """Numeric comparison catches a changed literal independently of XML shape."""

    expected = build_daveml_ir(b"<DAVEfunc><check value='1.0'/></DAVEfunc>", document_id="fixture")
    actual = build_daveml_ir(b"<DAVEfunc><check value='1.1'/></DAVEfunc>", document_id="fixture")

    differences = compare_daveml_numeric(expected, actual)

    assert len(differences) == 1
    assert differences[0].kind == "numeric"
    assert differences[0].path.endswith("/attributes/value/0")
    ####


def test_semantic_ir_preserves_vector_initial_values_as_typed_metadata() -> None:
    ir = build_daveml_ir(
        b'<DAVEfunc><variableDef varID="v" units="nd" initialValue="1 2 3"/></DAVEfunc>',
        document_id="vector",
    )
    assert ir.semantic["vectors"] == [
        {
            "source_path": "/0/0",
            "identifier": "v",
            "values": [1.0, 2.0, 3.0],
            "unit": "nd",
        }
    ]


def test_semantic_ir_resolves_typed_table_references() -> None:
    ir = build_daveml_ir(
        b"""
        <DAVEfunc>
          <griddedTableDef gtID="table"/>
          <function>
            <functionDefn><griddedTableRef gtID="table"/></functionDefn>
          </function>
        </DAVEfunc>
        """,
        document_id="references",
    )
    reference = ir.semantic["functions"][0]["references"][0]
    assert reference["tag"] == "griddedTableRef"
    assert reference["status"] == "resolved"
    assert reference["target_path"] == "/0/0"


def test_semantic_ir_marks_legacy_table_reference_type_mismatch() -> None:
    ir = build_daveml_ir(
        b"""
        <DAVEfunc>
          <ungriddedTableDef utID="table"/>
          <function>
            <functionDefn><griddedTableRef gtID="table"/></functionDefn>
          </function>
        </DAVEfunc>
        """,
        document_id="legacy-reference",
    )
    reference = ir.semantic["functions"][0]["references"][0]
    assert reference["status"] == "type_mismatch"
    assert reference["target_tag"] == "ungriddedTableDef"


def test_semantic_ir_classifies_modification_reference_as_external_provenance() -> None:
    ir = build_daveml_ir(
        b"""
        <DAVEfunc>
          <function><provenance><modificationRef refID="prior"/></provenance></function>
        </DAVEfunc>
        """,
        document_id="provenance-reference",
    )
    reference = ir.semantic["functions"][0]["references"][0]
    assert reference["status"] == "external_provenance"
    assert reference["identifier"] == "prior"


@pytest.mark.parametrize(
    "package_member",
    (
        "taoryx-aerospace-data-corpus-v1.1/qualified-models/f16-s119/taoryx-f16-s119-reference-v0.7.txair",
        "taoryx-aerospace-data-corpus-v1.1/qualified-models/hl20-mod-k/taoryx-hl20-mod-k-unpowered-v0.10.txair",
    ),
)
def test_real_reference_sources_roundtrip_through_canonical_ir(package_member: str) -> None:
    """The two promoted vehicle source documents structurally re-import."""

    with zipfile.ZipFile(CORPUS) as corpus:
        package_payload = corpus.read(package_member)
    with zipfile.ZipFile(io.BytesIO(package_payload)) as package:
        source_member = next(name for name in package.namelist() if name.startswith("models/") and name.endswith(".dml"))
        source = package.read(source_member)
    ir = build_daveml_ir(source, document_id=source_member)
    fresh = build_daveml_ir(export_daveml_ir(ir), document_id=source_member)

    assert compare_daveml_ir(ir, fresh) == ()
    assert ir.source_sha256
    assert ir.root["tag"] == "DAVEfunc"
    assert ir.semantic["functions"]
    assert ir.semantic["units"]
    assert all("dimension" in item for item in ir.semantic["units"])
    ####
