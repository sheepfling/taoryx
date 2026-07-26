from __future__ import annotations

import hashlib
import io
import json
import zipfile
from pathlib import Path

import pytest

from taoryx.trajectory import (
    CollectionManifest,
    SourceDocument,
    export_source_preserving_collection,
    read_collection_archive,
    replay_reference_collection,
    write_deterministic_collection,
)


def test_deterministic_collection_archive_repeats_byte_for_byte(tmp_path: Path) -> None:
    """M0 archives have stable ordering, timestamps, and checksum ledgers."""

    files = {
        "canonical/variables.json": b'{"id":"aero.alpha"}\n',
        "manifest.json": b'{"collection_id":"fixture"}\n',
        "source/aerodynamics.dml": b"<DAVE-ML/>\n",
    }
    first = write_deterministic_collection(files, tmp_path / "first.txcollection")
    second = write_deterministic_collection(files, tmp_path / "second.txcollection")
    assert first.sha256 == second.sha256
    assert (tmp_path / "first.txcollection").read_bytes() == (tmp_path / "second.txcollection").read_bytes()
    assert "checksums.sha256" in first.members
    ####


def test_deterministic_collection_rejects_unsafe_member_names(tmp_path: Path) -> None:
    """Collection members cannot escape the archive root."""

    with pytest.raises(ValueError, match="unsafe"):
        write_deterministic_collection({"../outside.txt": b"nope"}, tmp_path / "bad.txcollection")
    ####


def test_collection_reader_validates_ledger_and_manifest(tmp_path: Path) -> None:
    """The collection reader reloads the same semantic manifest from disk."""

    manifest = CollectionManifest(
        collection_type="taoryx.txcollection/v1alpha1",
        collection_id="fixture",
        collection_version="1.0.0",
        family_id="fixture-family",
        canonical_authority="taoryx_canonical",
        source_documents=(
            SourceDocument(
                document_id="fixture.aerodynamics",
                role="aerodynamics",
                source_path="fixture/aerodynamics.dml",
                package_member="models/aerodynamics.dml",
                sha256="" + "a" * 64,
                byte_identical_to_upstream=True,
            ),
        ),
        component_bindings=(),
        contribution_authority=(),
        transforms=(),
        stateful_components=(),
        runtime_artifact="runtime/aircraft.txair",
        validation_artifact="validation/source-check-cases.json",
        source_payload_external=False,
    )
    archive = write_deterministic_collection(
        {
            "manifest.json": manifest.canonical_json(),
            "source/aerodynamics.dml": b"<DAVE-ML/>\n",
        },
        tmp_path / "fixture.txcollection",
    )
    loaded = read_collection_archive(archive.path)
    assert loaded.manifest.collection_id == "fixture"
    assert loaded.files["source/aerodynamics.dml"] == b"<DAVE-ML/>\n"
    ####


def test_source_preserving_export_reports_verified_hash_reload(tmp_path: Path) -> None:
    """Source export and a fresh byte reload produce an explicit L0 report."""

    source = b"<DAVE-ML id=\"fixture\"/>\n"
    source_hash = hashlib.sha256(source).hexdigest()
    manifest = CollectionManifest(
        collection_type="taoryx.txcollection/v1alpha1",
        collection_id="fixture",
        collection_version="1.0.0",
        family_id="fixture-family",
        canonical_authority="taoryx_canonical",
        source_documents=(
            SourceDocument(
                document_id="fixture.aerodynamics",
                role="aerodynamics",
                source_path="fixture/aerodynamics.dml",
                package_member="models/aerodynamics.dml",
                sha256=source_hash,
                byte_identical_to_upstream=True,
            ),
        ),
        component_bindings=(),
        contribution_authority=(),
        transforms=(),
        stateful_components=(),
        runtime_artifact="runtime/aircraft.txair",
        validation_artifact="validation/source-check-cases.json",
        source_payload_external=False,
    )
    collection = tmp_path / "fixture.txcollection"
    write_deterministic_collection(
        {"manifest.json": manifest.canonical_json(), "source/aerodynamics.dml": source}, collection
    )
    report = export_source_preserving_collection(collection, tmp_path / "exported")
    assert report["status"] == "verified_source_preserving"
    assert report["levels"]["L0"] == "verified"
    assert (tmp_path / "exported/aerodynamics.dml").read_bytes() == source
    ####


def test_collection_replay_resolves_and_replays_declared_runtime_artifact(tmp_path: Path) -> None:
    """A collection replay verifies the outer archive before the runtime seam."""

    corpus = Path(__file__).parents[2] / "resources/aerospace/daveml/taoryx-corpus-v1.1/corpus.zip"
    package_member = "taoryx-aerospace-data-corpus-v1.1/qualified-models/f16-s119/taoryx-f16-s119-reference-v0.7.txair"
    with zipfile.ZipFile(corpus) as archive:
        runtime = archive.read(package_member)
    with zipfile.ZipFile(io.BytesIO(runtime)) as package:
        source_name = next(name for name in package.namelist() if name.endswith("aerodynamics.dml"))
        source = package.read(source_name)
    source_hash = hashlib.sha256(source).hexdigest()
    manifest = CollectionManifest(
        collection_type="taoryx.txcollection/v1alpha1",
        collection_id="f16-s119-fixture",
        collection_version="1.0.0",
        family_id="reference_f16_s119",
        canonical_authority="taoryx_canonical",
        source_documents=(SourceDocument(
            document_id="f16-s119.aerodynamics",
            role="aerodynamics",
            source_path=source_name,
            package_member=source_name,
            sha256=source_hash,
            byte_identical_to_upstream=True,
        ),),
        component_bindings=(),
        contribution_authority=(),
        transforms=(),
        stateful_components=(),
        runtime_artifact="runtime/aircraft.txair",
        validation_artifact="validation/source-check-cases.json",
        source_payload_external=False,
    )
    collection = tmp_path / "f16.txcollection"
    write_deterministic_collection(
        {
            "manifest.json": manifest.canonical_json(),
            "source/" + Path(source_name).name: source,
            "runtime/aircraft.txair": runtime,
        },
        collection,
    )

    report = replay_reference_collection(collection)

    assert report.status == "runtime_replay_qualification_passed"
    assert report.collection_id == "f16-s119-fixture"
    assert report.family_id == "reference_f16_s119"
    assert report.runtime_member == "runtime/aircraft.txair"
    assert json.dumps(report.to_dict())
