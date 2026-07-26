"""Loss-aware Taoryx collection contracts and deterministic containers."""

from __future__ import annotations

import hashlib
import json
import zipfile
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

from .daveml_evaluator import evaluate_daveml_checkdata
from .daveml_semantic import build_daveml_ir, compare_daveml_ir, compare_daveml_numeric, export_daveml_ir


class SourceDocument(BaseModel):
    """One immutable source document in a multi-document collection."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    document_id: str = Field(min_length=1)
    role: str = Field(min_length=1)
    source_path: str = Field(min_length=1)
    package_member: str = Field(min_length=1)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    byte_identical_to_upstream: bool
    upstream_repository: str | None = None
    upstream_path: str | None = None
    upstream_revision: str | None = None
####


class ComponentBinding(BaseModel):
    """Binding between canonical producers and a runtime component."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    component_id: str = Field(min_length=1)
    role: str = Field(min_length=1)
    source_document_ids: tuple[str, ...] = ()
    inputs: tuple[str, ...] = ()
    outputs: tuple[str, ...] = ()
    runtime_member: str | None = None
####


class ContributionAuthority(BaseModel):
    """Declares the single producer for one physical contribution."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    contribution: str = Field(min_length=1)
    producer: str = Field(min_length=1)
    status: Literal["authoritative", "derived", "unresolved"]
    rationale: str = ""
####


class TransformRecord(BaseModel):
    """Explicit source-to-canonical unit or frame transformation."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    transform_id: str = Field(min_length=1)
    kind: Literal["unit", "frame", "axis", "sign"]
    source: str = Field(min_length=1)
    canonical: str = Field(min_length=1)
    scale: float = 1.0
    offset: float = 0.0
    sign: int = Field(default=1, ge=-1, le=1)
    reason: str = Field(min_length=1)
####


class StatefulComponentContract(BaseModel):
    """Preserves dynamics that static DAVE-ML cannot own by itself."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    component_id: str = Field(min_length=1)
    classification: Literal["external_exact", "external_required", "exported_approximation", "not_exported"]
    states: tuple[str, ...] = ()
    source: str = Field(min_length=1)
    notes: str = ""
####


class CollectionManifest(BaseModel):
    """M0 collection identity and M1/M2 component boundary."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = Field(default=1, ge=1)
    collection_type: Literal["taoryx.txcollection/v1alpha1"]
    collection_id: str = Field(min_length=1)
    collection_version: str = Field(min_length=1)
    family_id: str = Field(min_length=1)
    qualification_class: Literal["reference_exact", "derived_exact", "surrogate_composite", "synthetic"] = "reference_exact"
    manufacturer_validated: bool = False
    source_exact: bool = False
    canonical_authority: Literal["taoryx_canonical"]
    source_documents: tuple[SourceDocument, ...]
    component_bindings: tuple[ComponentBinding, ...]
    contribution_authority: tuple[ContributionAuthority, ...]
    authority_map: Mapping[str, str] = Field(default_factory=dict)
    disabled_contributions: tuple[str, ...] = ()
    transforms: tuple[TransformRecord, ...]
    stateful_components: tuple[StatefulComponentContract, ...]
    runtime_artifact: str = Field(min_length=1)
    validation_artifact: str = Field(min_length=1)
    source_payload_external: bool = True
    roundtrip_status: Literal["not_started", "source_preserving_pending", "verified"] = "not_started"

    def canonical_json(self) -> bytes:
        """Return deterministic manifest JSON bytes."""

        return (json.dumps(self.model_dump(mode="json"), indent=2, sort_keys=True) + "\n").encode("utf-8")
        ####

    def source_map(self) -> dict[str, SourceDocument]:
        """Return source documents indexed by stable collection ID."""

        result = {item.document_id: item for item in self.source_documents}
        if len(result) != len(self.source_documents):
            raise ValueError(f"collection {self.collection_id!r} has duplicate source document IDs")
        return result
        ####


@dataclass(frozen=True, slots=True)
class CollectionArchive:
    """Output of a deterministic collection archive build."""

    path: Path
    sha256: str
    members: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class CollectionContents:
    """Validated contents of one ``.txcollection`` archive."""

    path: Path
    manifest: CollectionManifest
    files: dict[str, bytes]


def read_collection_archive(path: str | Path) -> CollectionContents:
    """Read and validate a deterministic collection archive.

    The checksum ledger is checked before the manifest is returned.  This is
    deliberately a container-level operation: it does not claim that the
    embedded DAVE-ML has been semantically parsed or that the runtime artifact
    has been replayed.
    """

    archive_path = Path(path)
    try:
        with zipfile.ZipFile(archive_path) as archive:
            names = archive.namelist()
            if len(names) != len(set(names)):
                raise ValueError(f"collection contains duplicate members: {archive_path}")
            for name in names:
                relative = Path(name)
                if relative.is_absolute() or ".." in relative.parts or not name:
                    raise ValueError(f"collection contains unsafe member: {name!r}")
            files = {name: archive.read(name) for name in names}
    except (OSError, zipfile.BadZipFile) as error:
        raise ValueError(f"invalid collection archive {archive_path}: {error}") from error
    if "manifest.json" not in files or "checksums.sha256" not in files:
        raise ValueError(f"collection is missing manifest.json or checksums.sha256: {archive_path}")

    ledger = files["checksums.sha256"].decode("utf-8")
    expected: dict[str, str] = {}
    for line in ledger.splitlines():
        digest, separator, name = line.partition("  ")
        if not separator or not name or name in expected:
            raise ValueError(f"invalid collection checksum ledger entry: {line!r}")
        expected[name] = digest
    actual_names = set(files) - {"checksums.sha256"}
    if set(expected) != actual_names:
        raise ValueError("collection checksum ledger does not match archive members")
    for name, digest in expected.items():
        observed = hashlib.sha256(files[name]).hexdigest()
        if observed != digest:
            raise ValueError(f"collection checksum mismatch for {name!r}: {observed} != {digest}")

    try:
        manifest = CollectionManifest.model_validate(json.loads(files["manifest.json"]))
    except (json.JSONDecodeError, ValueError) as error:
        raise ValueError(f"invalid collection manifest: {archive_path}") from error
    return CollectionContents(path=archive_path, manifest=manifest, files=files)
    ####


def _source_member_name(document: SourceDocument, files: dict[str, bytes]) -> str:
    """Resolve the collection member for one source document."""

    package_name = Path(document.package_member).name
    candidates = (
        f"source/{document.package_member.removeprefix('models/')}",
        f"source/{package_name}",
        f"source/{Path(document.source_path).name}",
    )
    for candidate in candidates:
        if candidate in files:
            return candidate
    raise ValueError(f"source payload for {document.document_id!r} is absent from collection")
    ####


def export_source_preserving_collection(
    collection: str | Path,
    output_directory: str | Path,
) -> dict[str, object]:
    """Export embedded source bytes and verify a fresh source-layer import.

    This implements the lossless/source-preserving part of the round trip. It
    intentionally does not label canonical DAVE-ML regeneration or runtime
    replay as complete; those require a semantic DAVE-ML serializer and a
    provider replay adapter respectively.
    """

    contents = read_collection_archive(collection)
    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    documents: list[dict[str, object]] = []
    for document in contents.manifest.source_documents:
        source_member = _source_member_name(document, contents.files)
        payload = contents.files[source_member]
        exported_path = destination / Path(source_member).name
        exported_path.write_bytes(payload)
        exported = exported_path.read_bytes()
        source_hash = hashlib.sha256(payload).hexdigest()
        fresh_import_hash = hashlib.sha256(exported).hexdigest()
        documents.append(
            {
                "document_id": document.document_id,
                "role": document.role,
                "collection_member": source_member,
                "exported_path": str(exported_path),
                "declared_sha256": document.sha256,
                "exported_sha256": source_hash,
                "fresh_import_sha256": fresh_import_hash,
                "byte_identical": payload == exported,
            }
        )

    l0 = all(
        item["byte_identical"] is True
        and item["declared_sha256"] == item["exported_sha256"]
        and item["exported_sha256"] == item["fresh_import_sha256"]
        for item in documents
    )
    l1 = l0 and bool(contents.manifest.source_map())
    canonical_documents: list[dict[str, object]] = []
    for item in documents:
        member = str(item["collection_member"])
        if not member.casefold().endswith((".dml", ".xml")):
            continue
        payload = contents.files[member]
        ir = build_daveml_ir(payload, document_id=str(item["document_id"]))
        exported = export_daveml_ir(ir)
        fresh = build_daveml_ir(exported, document_id=str(item["document_id"]))
        structural = compare_daveml_ir(ir, fresh)
        numeric = compare_daveml_numeric(ir, fresh)
        checks = evaluate_daveml_checkdata(payload)
        canonical_documents.append(
            {
                "document_id": item["document_id"],
                "structural_diff_count": len(structural),
                "numeric_diff_count": len(numeric),
                "checkdata_count": len(checks),
                "checkdata_failed": sum(result.status == "failed" for result in checks),
                "checkdata_unsupported": sum(result.status == "unsupported" for result in checks),
                "canonical_export_sha256": hashlib.sha256(exported).hexdigest(),
            }
        )
    l2 = bool(canonical_documents) and all(item["structural_diff_count"] == 0 for item in canonical_documents)
    l3 = l2 and all(
        item["numeric_diff_count"] == 0 and item["checkdata_failed"] == 0 for item in canonical_documents
    )
    report: dict[str, object] = {
        "status": "verified_source_preserving" if l0 else "failed",
        "collection": contents.manifest.collection_id,
        "collection_archive": str(contents.path),
        "fresh_import": "lossless_source_hash_reload",
        "levels": {
            "L0": "verified" if l0 else "failed",
            "L1": "verified_source_identity" if l1 else "failed",
            "L2": "verified_canonical_structure" if l2 else "failed",
            "L3": "verified_canonical_numeric_and_checkdata" if l3 else "failed",
            "L4": "pending_runtime_replay",
        },
        "source_documents": documents,
        "canonical_documents": canonical_documents,
        "limitations": [
            "Canonical semantic regeneration and fresh structural/numeric import are verified for embedded DAVE-ML documents.",
            "L4 runtime replay remains a separate collection/package evidence gate.",
        ],
    }
    (destination / "roundtrip-report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return report
    ####


def write_deterministic_collection(files: dict[str, bytes], output: str | Path) -> CollectionArchive:
    """Write a reproducible ZIP-form ``.txcollection``.

    The caller supplies all members except ``checksums.sha256``.  Member names
    are normalized and sorted, timestamps and permissions are fixed, and the
    checksum ledger is generated from the exact member bytes.
    """

    destination = Path(output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    normalized: dict[str, bytes] = {}
    for name, content in files.items():
        relative = Path(name)
        if relative.is_absolute() or ".." in relative.parts or relative.as_posix() in normalized:
            raise ValueError(f"unsafe or duplicate collection member: {name}")
        normalized[relative.as_posix()] = bytes(content)
    ledger = "".join(
        f"{hashlib.sha256(normalized[name]).hexdigest()}  {name}\n" for name in sorted(normalized)
    ).encode("utf-8")
    normalized["checksums.sha256"] = ledger
    with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for name in sorted(normalized):
            info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o100644 << 16
            archive.writestr(info, normalized[name])
    digest = hashlib.sha256(destination.read_bytes()).hexdigest()
    return CollectionArchive(destination, digest, tuple(sorted(normalized)))
    ####


__all__ = [
    "CollectionArchive",
    "CollectionContents",
    "CollectionManifest",
    "ComponentBinding",
    "ContributionAuthority",
    "SourceDocument",
    "StatefulComponentContract",
    "TransformRecord",
    "export_source_preserving_collection",
    "read_collection_archive",
    "write_deterministic_collection",
]
