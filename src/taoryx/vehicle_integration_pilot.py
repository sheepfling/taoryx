"""Provider-neutral integration pilots for collection and reference manifests.

The main vehicle-integration pipeline consumes the richer typed reference-family
manifest used by the F-16 and HL-20.  A320 is intentionally represented as a
derived-exact OpenAP collection and a surrogate-composite OpenAP/JSBSim
collection, while NESC is a source-grounded open-loop family.  This module
exercises the same staged contract without pretending those inputs are the
same manifest kind or have the same control obligations.
"""

from __future__ import annotations

import hashlib
import json
import shutil
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import yaml

from .trajectory.collections import CollectionManifest
from .vehicle_controller_mission_preflight import validate_vehicle_controller_mission_preflight
from .vehicle_registry import ROOT

PilotStatus = Literal["passed", "development", "blocked", "not_applicable", "planned"]
EvidenceKind = Literal["local_path", "external_path", "inline_status"]
STAGE_ORDER: tuple[str, ...] = (
    "intake",
    "conventions",
    "plant",
    "effectivity",
    "trim",
    "operating_points",
    "fidelity",
    "controller",
    "mission",
)
PILOT_FAMILIES: tuple[str, ...] = (
    "reference_nesc_two_stage_rocket",
    "a320_openap_3dof",
    "a320_openap_jsbsim_pseudo6dof",
)


@dataclass(frozen=True, slots=True)
class IntegrationPilotFinding:
    """One deterministic pilot diagnostic."""

    severity: Literal["error", "warning", "info"]
    code: str
    path: str
    message: str
    hint: str

    def as_dict(self) -> dict[str, str]:
        """Return a JSON-compatible finding."""

        return {
            "severity": self.severity,
            "code": self.code,
            "path": self.path,
            "message": self.message,
            "hint": self.hint,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class IntegrationPilotStage:
    """One stage in a collection/reference integration pilot."""

    stage_id: str
    status: PilotStatus
    claim: str
    evidence: tuple[str, ...]
    findings: tuple[IntegrationPilotFinding, ...]
    metrics: Mapping[str, object]

    @property
    def blockers(self) -> tuple[IntegrationPilotFinding, ...]:
        """Return errors for this stage."""

        return tuple(item for item in self.findings if item.severity == "error")
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a stable stage record."""

        return {
            "stage_id": self.stage_id,
            "status": self.status,
            "claim": self.claim,
            "evidence": list(self.evidence),
            "metrics": dict(self.metrics),
            "error_count": len(self.blockers),
            "findings": [item.as_dict() for item in self.findings],
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class VehicleIntegrationPilotReport:
    """Complete diagnostic report for one non-reference-family pilot."""

    family_id: str
    manifest_kind: Literal["collection", "reference"]
    qualification_class: str
    status: str
    current_tier: str
    next_gate: str
    promotion_eligible: bool
    claim_boundary: str
    snapshot: IntegrationSnapshot
    stages: tuple[IntegrationPilotStage, ...]
    blockers: tuple[IntegrationPilotFinding, ...]
    caveats: tuple[IntegrationPilotFinding, ...]

    def as_dict(self) -> dict[str, object]:
        """Return a self-contained machine-readable report."""

        return {
            "schema_version": "taoryx.vehicle-integration-pilot/v1",
            "family_id": self.family_id,
            "manifest_kind": self.manifest_kind,
            "qualification_class": self.qualification_class,
            "status": self.status,
            "current_tier": self.current_tier,
            "next_gate": self.next_gate,
            "promotion_eligible": self.promotion_eligible,
            "claim_boundary": self.claim_boundary,
            "snapshot": self.snapshot.as_dict(),
            "stages": [stage.as_dict() for stage in self.stages],
            "blockers": [item.as_dict() for item in self.blockers],
            "caveats": [item.as_dict() for item in self.caveats],
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class EvidenceReference:
    """One normalized evidence or dependency reference.

    ``local_path`` means the referenced bytes are available in this checkout.
    ``external_path`` means the manifest intentionally points outside the
    checkout and must remain a handoff dependency.  ``inline_status`` covers
    prose, ratios, status labels, or other evidence that has no file identity.
    """

    label: str
    kind: EvidenceKind
    reference: str
    path: str | None
    declared_sha256: str | None
    observed_sha256: str | None
    available: bool

    def as_dict(self) -> dict[str, object]:
        """Return a stable JSON representation."""

        return {
            "label": self.label,
            "kind": self.kind,
            "reference": self.reference,
            "path": self.path,
            "declared_sha256": self.declared_sha256,
            "observed_sha256": self.observed_sha256,
            "available": self.available,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class IntegrationSnapshot:
    """Normalized input view shared by all automatic integration stages."""

    family_id: str
    manifest_kind: Literal["collection", "reference"]
    manifest_path: str
    record_path: str
    contract_path: str
    catalog_path: str
    qualification_class: str
    source_kind: str
    runtime_fidelity: str
    frames: Mapping[str, object]
    applicability: Mapping[str, str]
    evidence: tuple[EvidenceReference, ...]
    dependency_hashes: Mapping[str, str]
    source_payload_external: bool
    manifest: Mapping[str, object]
    record: Mapping[str, object]
    contract: Mapping[str, object] | None
    catalog_entry: Mapping[str, object] | None

    @property
    def external_dependencies(self) -> tuple[EvidenceReference, ...]:
        """Return references that must be supplied outside this checkout."""

        return tuple(item for item in self.evidence if item.kind == "external_path")
        ####

    @property
    def local_evidence(self) -> tuple[EvidenceReference, ...]:
        """Return evidence files available for packet inclusion."""

        return tuple(item for item in self.evidence if item.kind == "local_path")
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a self-contained normalized snapshot description."""

        return {
            "schema_version": "taoryx.vehicle-integration-snapshot/v1",
            "family_id": self.family_id,
            "manifest_kind": self.manifest_kind,
            "manifest_path": self.manifest_path,
            "record_path": self.record_path,
            "contract_path": self.contract_path,
            "catalog_path": self.catalog_path,
            "qualification_class": self.qualification_class,
            "source_kind": self.source_kind,
            "runtime_fidelity": self.runtime_fidelity,
            "frames": dict(self.frames),
            "applicability": dict(self.applicability),
            "evidence": [item.as_dict() for item in self.evidence],
            "dependency_hashes": dict(self.dependency_hashes),
            "source_payload_external": self.source_payload_external,
            "external_dependency_count": len(self.external_dependencies),
            "local_evidence_count": len(self.local_evidence),
        }
        ####
    ####


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)
    ####


def _read_yaml(path: Path) -> Mapping[str, object]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{path} must contain a mapping")
    return payload
    ####


def _read_json(path: Path) -> Mapping[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{path} must contain a mapping")
    return payload
    ####


def _finding(
    severity: Literal["error", "warning", "info"],
    code: str,
    path: str,
    message: str,
    hint: str,
) -> IntegrationPilotFinding:
    return IntegrationPilotFinding(severity, code, path, message, hint)
    ####


def _stage(
    stage_id: str,
    status: PilotStatus,
    claim: str,
    evidence: list[str],
    findings: list[IntegrationPilotFinding],
    metrics: Mapping[str, object] | None = None,
) -> IntegrationPilotStage:
    return IntegrationPilotStage(stage_id, status, claim, tuple(evidence), tuple(findings), metrics or {})
    ####


def _contract(family_id: str) -> tuple[Path, Mapping[str, object] | None]:
    path = ROOT / "verification/daveml_operational_contracts.yaml"
    if not path.is_file():
        return path, None
    payload = _read_yaml(path)
    families = payload.get("families")
    if not isinstance(families, list):
        return path, None
    return path, next((item for item in families if isinstance(item, Mapping) and item.get("id") == family_id), None)
    ####


def _manifest_and_record(family_id: str) -> tuple[Path, Mapping[str, object], Path, Mapping[str, object], Literal["collection", "reference"]]:
    family_dir = ROOT / "families" / family_id
    record_path = family_dir / "qualification/integration-record.yaml"
    manifest_path = family_dir / "collection-manifest.json"
    kind: Literal["collection", "reference"] = "collection"
    if not manifest_path.is_file():
        manifest_path = family_dir / "family.yaml"
        kind = "reference"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"manifest not found for {family_id}")
    if not record_path.is_file():
        raise FileNotFoundError(f"integration record not found for {family_id}")
    manifest = _read_json(manifest_path) if kind == "collection" else _read_yaml(manifest_path)
    record = _read_yaml(record_path)
    return manifest_path, manifest, record_path, record, kind
    ####


def _applicability(contract: Mapping[str, object] | None, key: str, record: Mapping[str, object]) -> str:
    applicability = contract.get("applicability") if isinstance(contract, Mapping) else None
    if isinstance(applicability, Mapping):
        return str(applicability.get(key, "required"))
    value = record.get("layers")
    if isinstance(value, Mapping):
        layer = value.get(key)
        if isinstance(layer, Mapping) and "evidence" in layer and "not_applicable" in str(layer["evidence"]).lower():
            return "not_applicable"
    return "required"
    ####


def _path_evidence(value: object) -> Path | None:
    if not isinstance(value, str):
        return None
    value = value.split("#", 1)[0].strip()
    if not value or "/" not in value:
        return None
    path = Path(value)
    return path if path.is_absolute() else ROOT / path
    ####


def _collection(manifest_path: Path, manifest: Mapping[str, object]) -> CollectionManifest | None:
    if manifest_path.suffix != ".json":
        return None
    return CollectionManifest.model_validate(manifest)
    ####


def _sha256_file(path: Path) -> str:
    """Return the SHA-256 digest of one local dependency."""

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
    ####


def _looks_like_path(value: str) -> bool:
    """Distinguish file references from inline status/prose evidence."""

    base = value.split("#", 1)[0].strip()
    if not base or base in {"none", "not_applicable", "not_available"}:
        return False
    if base.count("/") == 1:
        left, right = (part.strip() for part in base.split("/"))
        if left.isdigit() and (right.split(maxsplit=1)[0] if right else "").isdigit():
            return False
    return Path(base).is_absolute() or "/" in base or Path(base).suffix in {
        ".json",
        ".yaml",
        ".yml",
        ".csv",
        ".dml",
        ".xml",
        ".zip",
        ".txair",
        ".tbl",
        ".prb",
    }
    ####


def _resolve_path_reference(value: str) -> tuple[str, Path | None]:
    """Return the original reference and its local checkout candidate."""

    base = value.split("#", 1)[0].strip()
    path = Path(base)
    candidate = path if path.is_absolute() else ROOT / path
    return base, candidate
    ####


def _evidence_reference(
    label: str,
    value: object,
    *,
    declared_sha256: str | None = None,
) -> EvidenceReference:
    """Normalize a path, external dependency, or inline evidence value."""

    reference = str(value)
    if not _looks_like_path(reference):
        return EvidenceReference(label, "inline_status", reference, None, declared_sha256, None, True)
    base, candidate = _resolve_path_reference(reference)
    if candidate is not None and candidate.is_file():
        observed = _sha256_file(candidate)
        return EvidenceReference(label, "local_path", reference, _relative(candidate), declared_sha256, observed, True)
    return EvidenceReference(label, "external_path", reference, base, declared_sha256, None, False)
    ####


def _add_evidence(
    result: list[EvidenceReference],
    seen: set[tuple[str, str]],
    label: str,
    value: object,
    *,
    declared_sha256: str | None = None,
) -> None:
    """Append one normalized reference once."""

    item = _evidence_reference(label, value, declared_sha256=declared_sha256)
    key = (item.label, item.reference)
    if key not in seen:
        result.append(item)
        seen.add(key)
    ####


def _string_value(value: object) -> str | None:
    return value if isinstance(value, str) and value.strip() else None
    ####


def _collect_snapshot_evidence(
    family_id: str,
    manifest_path: Path,
    manifest: Mapping[str, object],
    record_path: Path,
    record: Mapping[str, object],
    contract_path: Path,
    catalog_path: Path,
    catalog_entry: Mapping[str, object] | None,
) -> tuple[tuple[EvidenceReference, ...], dict[str, str]]:
    evidence: list[EvidenceReference] = []
    seen: set[tuple[str, str]] = set()
    dependency_hashes: dict[str, str] = {}

    for label, path in (
        ("manifest", manifest_path),
        ("integration_record", record_path),
        ("operational_contract", contract_path),
        ("operating_point_catalog", catalog_path),
    ):
        _add_evidence(evidence, seen, label, _relative(path))
        if path.is_file():
            dependency_hashes[label] = _sha256_file(path)

    collection = _collection(manifest_path, manifest)
    if collection is not None:
        for document in collection.source_documents:
            for field, value in (("source_path", document.source_path), ("package_member", document.package_member)):
                label = f"source_documents.{document.document_id}.{field}"
                _add_evidence(evidence, seen, label, value, declared_sha256=document.sha256)
                dependency_hashes[label] = document.sha256
        for label, raw_value in (
            ("runtime_artifact", collection.runtime_artifact),
            ("validation_artifact", collection.validation_artifact),
            ("runtime_qualification_artifact", collection.runtime_qualification_artifact),
        ):
            resolved_runtime = _string_value(raw_value)
            if resolved_runtime is not None:
                _add_evidence(evidence, seen, label, resolved_runtime)
    else:
        raw_source = manifest.get("source")
        if isinstance(raw_source, Mapping):
            for key in ("package", "package_relative_path", "aerodynamic_source", "corpus_archive"):
                raw_value = _string_value(raw_source.get(key))
                resolved_source = raw_value
                if resolved_source is None:
                    continue
                declared = _string_value(raw_source.get(f"{key}_sha256"))
                label = f"manifest.source.{key}"
                _add_evidence(evidence, seen, label, resolved_source, declared_sha256=declared)
                if declared:
                    dependency_hashes[label] = declared
        for key in ("daveml_import", "bindings"):
            manifest_value = manifest.get(key)
            if isinstance(manifest_value, str):
                _add_evidence(evidence, seen, f"manifest.{key}", manifest_value)
            elif isinstance(manifest_value, list):
                for index, item in enumerate(manifest_value):
                    _add_evidence(evidence, seen, f"manifest.{key}[{index}]", item)

    raw_record_source = record.get("source")
    if isinstance(raw_record_source, Mapping):
        for key, value in raw_record_source.items():
            if key.endswith("_sha256") and isinstance(value, str):
                dependency_hashes[f"record.source.{key.removesuffix('_sha256')}"] = value
            elif key in {"package", "corpus_archive", "source_package", "source_path", "package_relative_path"}:
                if isinstance(value, str):
                    _add_evidence(evidence, seen, f"record.source.{key}", value)

    raw_layers = record.get("layers")
    if isinstance(raw_layers, Mapping):
        for layer_id, layer in raw_layers.items():
            if not isinstance(layer, Mapping):
                continue
            for field in ("evidence", "artifact", "implementation", "version"):
                layer_value = layer.get(field)
                if isinstance(layer_value, str):
                    _add_evidence(evidence, seen, f"record.layers.{layer_id}.{field}", layer_value)

    if catalog_entry is not None:
        catalog_value = catalog_entry.get("evidence")
        if catalog_value is not None:
            _add_evidence(evidence, seen, "operating_point.evidence", catalog_value)

    family_dir = manifest_path.parent
    for path in sorted(family_dir.rglob("*")):
        if not path.is_file() or path.suffix.lower() not in {".json", ".yaml", ".yml", ".dml"}:
            continue
        relative = _relative(path)
        label = f"family_file.{relative}"
        _add_evidence(evidence, seen, label, relative)
        dependency_hashes[label] = _sha256_file(path)

    if family_id == "reference_nesc_two_stage_rocket":
        for relative in (
            "verification/daveml_nesc_reduction_qualification.json",
            "verification/daveml_nesc_staging_lineage.json",
            "verification/daveml_alpha3_deployment_evidence.json",
            "verification/daveml_nesc_replay_evidence.json",
            "verification/daveml_nesc_objectives_evidence.json",
        ):
            path = ROOT / relative
            _add_evidence(evidence, seen, f"nesc_lineage.{Path(relative).stem}", relative)
            if path.is_file():
                dependency_hashes[f"nesc_lineage.{Path(relative).stem}"] = _sha256_file(path)

    a320_mission_artifacts = {
        "a320_openap_3dof": ("point_mass_3dof", "a320-openap-point-mass-racetrack"),
        "a320_openap_jsbsim_pseudo6dof": ("pseudo_6dof_kinematic_bridge", "a320-openap-pseudo6dof-racetrack"),
    }
    if family_id in a320_mission_artifacts:
        mode, label = a320_mission_artifacts[family_id]
        for suffix in ("evidence.json", "telemetry.csv", "board.png"):
            relative = f"verification/a320_racetrack/{mode}_{suffix}"
            path = ROOT / relative
            _add_evidence(evidence, seen, f"mission.{label}.{suffix}", relative)
            if path.is_file():
                dependency_hashes[f"mission.{label}.{suffix}"] = _sha256_file(path)

    return tuple(evidence), dependency_hashes
    ####


def load_integration_snapshot(family_id: str) -> IntegrationSnapshot:
    """Load a normalized snapshot for a collection or reference family."""

    manifest_path, manifest, record_path, record, kind = _manifest_and_record(family_id)
    contract_path, contract = _contract(family_id)
    catalog_path, catalog_entry = _catalog_entry(family_id)
    collection = _collection(manifest_path, manifest)
    plant = record.get("plant")
    frames: Mapping[str, object] = {}
    if isinstance(plant, Mapping):
        candidate = plant.get("frames")
        if isinstance(candidate, Mapping):
            frames = dict(candidate)
        else:
            frames = {
                key.removesuffix("_frame"): value
                for key in ("body_frame", "navigation_frame", "quaternion_order")
                if (value := plant.get(key)) is not None
            }
    contract_frames = contract.get("frames") if isinstance(contract, Mapping) else None
    if not frames and isinstance(contract_frames, Mapping):
        frames = {str(key): value for key, value in contract_frames.items()}
    applicability: dict[str, str] = {}
    for key in ("trim", "linearization", "tuning", "objectives", "scenarios"):
        applicability[key] = _applicability(contract, key, record)
    qualification_class: str
    if collection is not None:
        qualification_class = collection.qualification_class
        source_kind = str(collection.source_documents[0].upstream_repository or "collection") if collection.source_documents else "collection"
        runtime_fidelity = str(plant.get("fidelity", "unknown")) if isinstance(plant, Mapping) else "unknown"
        source_payload_external = collection.source_payload_external
    else:
        qualification_class = str(manifest.get("qualification_class", "reference_exact"))
        raw_source = manifest.get("source")
        source_kind = str(raw_source.get("kind", "reference")) if isinstance(raw_source, Mapping) else "reference"
        runtime_fidelity = str(plant.get("fidelity", "unknown")) if isinstance(plant, Mapping) else "unknown"
        source_payload_external = bool(plant.get("source_payload_required_for_execution", False)) if isinstance(plant, Mapping) else False
    evidence, dependency_hashes = _collect_snapshot_evidence(
        family_id,
        manifest_path,
        manifest,
        record_path,
        record,
        contract_path,
        catalog_path,
        catalog_entry,
    )
    return IntegrationSnapshot(
        family_id=family_id,
        manifest_kind=kind,
        manifest_path=_relative(manifest_path),
        record_path=_relative(record_path),
        contract_path=_relative(contract_path),
        catalog_path=_relative(catalog_path),
        qualification_class=qualification_class,
        source_kind=source_kind,
        runtime_fidelity=runtime_fidelity,
        frames=frames,
        applicability=applicability,
        evidence=evidence,
        dependency_hashes=dependency_hashes,
        source_payload_external=source_payload_external,
        manifest=manifest,
        record=record,
        contract=contract,
        catalog_entry=catalog_entry,
    )
    ####


def _intake_stage(
    family_id: str,
    manifest_path: Path,
    manifest: Mapping[str, object],
    record_path: Path,
    record: Mapping[str, object],
    kind: Literal["collection", "reference"],
    snapshot: IntegrationSnapshot | None = None,
) -> IntegrationPilotStage:
    findings: list[IntegrationPilotFinding] = []
    if record.get("family") not in {None, family_id}:
        findings.append(_finding("error", "record-family-mismatch", _relative(record_path), "integration record family does not match the requested pilot", "Keep the record and manifest family identifiers identical."))
    source = record.get("source")
    if not isinstance(source, Mapping):
        findings.append(_finding("error", "source-record-missing", _relative(record_path), "integration record has no source block", "Declare immutable source type, revision, and hashes."))
    if kind == "collection":
        try:
            parsed = _collection(manifest_path, manifest)
        except ValueError as error:
            parsed = None
            findings.append(_finding("error", "collection-manifest-invalid", _relative(manifest_path), str(error), "Repair the collection manifest before runtime integration."))
        if parsed is not None and not parsed.source_documents:
            findings.append(_finding("error", "collection-sources-empty", _relative(manifest_path), "collection declares no source documents", "Pin at least one source document even for a derived or composite lane."))
    if snapshot is not None:
        for reference in snapshot.local_evidence:
            if reference.declared_sha256 and reference.observed_sha256 and reference.declared_sha256 != reference.observed_sha256:
                findings.append(_finding("error", "dependency-hash-mismatch", reference.path or reference.reference, "locally available evidence does not match its declared SHA-256", "Restore the pinned artifact or update its manifest through the source-ingestion workflow; never accept a silent hash drift."))
    evidence = [_relative(manifest_path), _relative(record_path)]
    status: PilotStatus = "blocked" if findings else "passed"
    metrics: dict[str, object] = {"manifest_kind": kind}
    if snapshot is not None:
        metrics.update({"evidence_reference_count": len(snapshot.evidence), "local_evidence_count": len(snapshot.local_evidence), "external_dependency_count": len(snapshot.external_dependencies)})
    return _stage("intake", status, "manifest and integration record are discoverable, typed, and provenance-bearing", evidence, findings, metrics)
    ####


def _conventions_stage(
    manifest_path: Path,
    manifest: Mapping[str, object],
    contract_path: Path,
    contract: Mapping[str, object] | None,
    record_path: Path,
    record: Mapping[str, object],
) -> IntegrationPilotStage:
    findings: list[IntegrationPilotFinding] = []
    evidence = [_relative(manifest_path), _relative(contract_path), _relative(record_path)]
    plant = record.get("plant")
    record_frames = plant.get("frames") if isinstance(plant, Mapping) else None
    if not isinstance(record_frames, Mapping) and isinstance(plant, Mapping):
        record_frames = {key: plant.get(key) for key in ("body_frame", "navigation_frame", "quaternion_order")}
        record_frames = {key.replace("_frame", ""): value for key, value in record_frames.items() if value is not None}
    contract_frames = contract.get("frames") if isinstance(contract, Mapping) else None
    if isinstance(record_frames, Mapping) and isinstance(contract_frames, Mapping):
        for key in ("body", "navigation", "quaternion_order"):
            if key in record_frames and key in contract_frames and record_frames[key] != contract_frames[key]:
                findings.append(_finding("error", "frame-convention-mismatch", _relative(record_path), f"{key} convention differs between record and operational contract", "Correct the adapter boundary; never silently rotate or reorder state."))
    collection = _collection(manifest_path, manifest)
    if collection is not None:
        transforms = {(item.kind, item.transform_id): item for item in collection.transforms}
        if not any(key[0] == "frame" and "body" in key[1] for key in transforms):
            findings.append(_finding("error", "collection-body-frame-transform-missing", _relative(manifest_path), "collection does not declare a body-frame transform", "Declare source-to-canonical frame treatment explicitly."))
    status: PilotStatus = "blocked" if findings else "passed"
    return _stage("conventions", status, "frames and source-to-canonical transforms are explicit at the collection boundary", evidence, findings, {"contract_checked": contract is not None})
    ####


def _plant_stage(
    family_id: str,
    manifest_path: Path,
    manifest: Mapping[str, object],
    record_path: Path,
    record: Mapping[str, object],
) -> IntegrationPilotStage:
    findings: list[IntegrationPilotFinding] = []
    evidence = [_relative(manifest_path), _relative(record_path)]
    metrics: dict[str, object] = {"family_id": family_id}
    plant = record.get("plant")
    qualification = str(plant.get("qualification", "")) if isinstance(plant, Mapping) else ""
    collection = _collection(manifest_path, manifest)
    roundtrip = collection.roundtrip_status if collection is not None else "verified"
    if not any(token in qualification.lower() for token in ("passed", "verified", "qualified")):
        findings.append(_finding("error", "plant-qualification-missing", _relative(record_path), f"plant qualification is {qualification!r}", "Retain source/oracle or runtime replay evidence before promoting the plant."))
    if roundtrip != "verified":
        findings.append(_finding("error", "collection-roundtrip-not-verified", _relative(manifest_path), f"collection roundtrip is {roundtrip!r}", "Complete source-preserving collection validation before runtime use."))
    if collection is not None and collection.source_payload_external:
        findings.append(_finding("info", "source-payload-external", _relative(manifest_path), "collection depends on an external pinned source payload", "Keep the external hash and retrieval instructions in the handoff packet."))
    if family_id == "reference_nesc_two_stage_rocket":
        lineage_paths = (
            ROOT / "verification/daveml_nesc_reduction_qualification.json",
            ROOT / "verification/daveml_nesc_staging_lineage.json",
            ROOT / "verification/daveml_alpha3_deployment_evidence.json",
            ROOT / "verification/daveml_nesc_replay_evidence.json",
            ROOT / "verification/daveml_nesc_objectives_evidence.json",
        )
        lineage_payloads: dict[str, Mapping[str, object]] = {}
        for path in lineage_paths:
            evidence.append(_relative(path))
            if not path.is_file():
                findings.append(_finding("error", "nesc-lineage-evidence-missing", _relative(path), "NESC staging/reduction/deployment lineage evidence is missing", "Regenerate the lineage artifact before promoting automatic reductions or deployment witnesses."))
                continue
            try:
                lineage_payloads[path.stem] = _read_json(path)
            except (OSError, ValueError, json.JSONDecodeError) as error:
                findings.append(_finding("error", "nesc-lineage-evidence-invalid", _relative(path), str(error), "Repair the lineage evidence JSON before promotion."))
        staging: Mapping[str, object] = lineage_payloads.get("daveml_nesc_staging_lineage", {})
        reduction: Mapping[str, object] = lineage_payloads.get("daveml_nesc_reduction_qualification", {})
        deployment: Mapping[str, object] = lineage_payloads.get("daveml_alpha3_deployment_evidence", {})
        staging_events = staging.get("events")
        staging_lineage = staging.get("lineage")
        reduction_payload = reduction.get("reduction")
        lineage: Mapping[str, object] = staging_lineage if isinstance(staging_lineage, Mapping) else {}
        reduction_result: Mapping[str, object] = reduction_payload if isinstance(reduction_payload, Mapping) else {}
        metrics["nesc_lineage"] = {
            "staging_event_count": len(staging_events) if isinstance(staging_events, list) else 0,
            "staging_status": staging.get("status"),
            "reduction_id": reduction_result.get("id"),
            "reduction_status": reduction_result.get("status"),
            "reduction_pass": reduction_result.get("pass"),
            "deployment_status": deployment.get("status"),
            "deployment_child_id": deployment.get("child"),
            "parent_qualification_unchanged_by_child": lineage.get("parent_qualification_unchanged_by_child"),
        }
        findings.append(_finding("info", "nesc-lineage-verified", "verification/daveml_nesc_staging_lineage.json", "staging, reduction, replay, and synthetic deployment lineage are included in the pilot", "Retain each artifact and its hash in the integration packet."))
    status: PilotStatus = "blocked" if findings and any(item.severity == "error" for item in findings) else "passed"
    metrics.update({"qualification": qualification, "roundtrip_status": roundtrip})
    return _stage("plant", status, "the declared source or composite plant has a verified package/roundtrip boundary", evidence, findings, metrics)
    ####


def _effectivity_stage(
    family_id: str,
    manifest_path: Path,
    manifest: Mapping[str, object],
    record: Mapping[str, object],
) -> IntegrationPilotStage:
    findings: list[IntegrationPilotFinding] = []
    evidence = [_relative(manifest_path)]
    collection = _collection(manifest_path, manifest)
    controls = []
    if collection is not None:
        controls = [item for item in collection.component_bindings if any("control" in output or "surface" in output or "moment" in output for output in item.outputs)]
    raw_layers = record.get("layers")
    actuator_layer = raw_layers.get("actuators") if isinstance(raw_layers, Mapping) else None
    if family_id == "reference_nesc_two_stage_rocket" or (not controls and not actuator_layer):
        findings.append(_finding("info", "effectivity-not-applicable", _relative(manifest_path), "the pilot declares no physical control/effectivity path", "Keep the model open-loop and do not invent a controller or actuator tier."))
        return _stage("effectivity", "not_applicable", "control-effectivity validation is not applicable to this open-loop or point-mass lane", evidence, findings, {"physical_control_bindings": len(controls)})
    effectivity_contract = manifest_path.parent / "qualification/effector-contract.yaml"
    if effectivity_contract.is_file():
        evidence.append(_relative(effectivity_contract))
        try:
            payload = _read_yaml(effectivity_contract)
        except (OSError, ValueError, yaml.YAMLError) as error:
            return _stage(
                "effectivity",
                "blocked",
                "effectivity contract exists but cannot be parsed",
                evidence,
                [_finding("error", "effectivity-contract-invalid", _relative(effectivity_contract), str(error), "Repair the typed effector contract before running the integration pilot.")],
                {"physical_control_bindings": len(controls)},
            )
        declared_controls = payload.get("controls") if isinstance(payload, Mapping) else None
        invalid_controls: list[str] = []
        if not isinstance(declared_controls, list) or not declared_controls:
            invalid_controls.append("controls must be a non-empty list")
        else:
            for item in declared_controls:
                if not isinstance(item, Mapping) or not item.get("id") or not item.get("unit"):
                    invalid_controls.append("each control requires id and unit")
                    continue
                for key in ("lower", "upper"):
                    if not isinstance(item.get(key), (int, float)):
                        invalid_controls.append(f"{item.get('id', '<unknown>')} missing numeric {key}")
        if invalid_controls:
            return _stage(
                "effectivity",
                "blocked",
                "typed effectivity contract is incomplete",
                evidence,
                [_finding("error", "effectivity-contract-incomplete", _relative(effectivity_contract), "; ".join(invalid_controls), "Declare bounded controls with units before promotion.")],
                {"physical_control_bindings": len(controls)},
            )
        declared_effectors = declared_controls if isinstance(declared_controls, list) else []
        allocation = payload.get("allocation")
        physical_allocator = bool(allocation.get("physical_allocator", False)) if isinstance(allocation, Mapping) else False
        findings.append(_finding("warning", "surrogate-effectivity-overlay", _relative(effectivity_contract), "bounded semantic effectors are declared, but their actuator behavior remains a Taoryx policy overlay", "Keep coefficient authority, actuator limits, and pseudo-6DOF response evidence separate from source-exact flight-control claims."))
        return _stage(
            "effectivity",
            "development",
            "typed pseudo-6DOF effectivity contract is present with explicit surrogate boundaries",
            evidence,
            findings,
            {"physical_control_bindings": len(controls), "declared_effectors": len(declared_effectors), "contract": _relative(effectivity_contract), "physical_allocator": physical_allocator},
        )
    has_overlay_actuators = collection is not None and (
        any("actuator" in item.role or "control" in item.role for item in collection.component_bindings)
        or any(item.component_id == "actuators" for item in collection.stateful_components)
        or collection.authority_map.get("actuators") == "taoryx-policy"
    )
    if has_overlay_actuators:
        findings.append(_finding("warning", "surrogate-effectivity-overlay", _relative(manifest_path), "control authority is supplied by a Taoryx policy or composite overlay", "Keep allocation and actuator evidence separate from source performance authority."))
        return _stage("effectivity", "development", "composite control/effectivity authority is declared but remains overlay evidence", evidence, findings, {"physical_control_bindings": len(controls)})
    return _stage("effectivity", "planned", "physical effectivity has not yet been declared", evidence, [_finding("warning", "effectivity-pending", _relative(manifest_path), "no reusable effectivity contract was discovered", "Add an actuator/effectivity layer before controller promotion.")], {"physical_control_bindings": len(controls)})
    ####


def _catalog_entry(family_id: str) -> tuple[Path, Mapping[str, object] | None]:
    path = ROOT / "verification/daveml_operating_point_catalog.yaml"
    if not path.is_file():
        return path, None
    payload = _read_yaml(path)
    families = payload.get("families")
    if not isinstance(families, list):
        return path, None
    return path, next((item for item in families if isinstance(item, Mapping) and item.get("id") == family_id), None)
    ####


def _trim_and_points_stage(
    family_id: str,
    record_path: Path,
    record: Mapping[str, object],
    contract: Mapping[str, object] | None,
) -> tuple[IntegrationPilotStage, IntegrationPilotStage]:
    catalog_path, entry = _catalog_entry(family_id)
    applicability = _applicability(contract, "trim", record)
    if applicability == "not_applicable":
        trim = _stage("trim", "not_applicable", "trim is explicitly not applicable for the declared open-loop contract", [_relative(record_path)], [_finding("info", "trim-not-applicable", _relative(record_path), "the operational contract declares no equilibrium trim requirement", "Use trajectory checkpoints and event evidence instead of inventing a trim solve.")])
    elif entry is None:
        trim = _stage("trim", "blocked", "trim requires a verified operating-point record", [_relative(catalog_path)], [_finding("error", "trim-evidence-missing", _relative(catalog_path), "no operating-point entry exists for this family", "Add a source-backed trim or steady-performance point.")])
    else:
        trim = _stage("trim", "passed", "trim/steady-performance evidence is discoverable in the shared operating-point catalog", [_relative(catalog_path), str(entry.get("evidence"))], [], {"point_id": entry.get("point_id"), "applicability": entry.get("applicability")})
    if entry is None:
        points = _stage("operating_points", "blocked", "operating-point evidence is required for automatic integration", [_relative(catalog_path)], [_finding("error", "operating-point-entry-missing", _relative(catalog_path), "family has no catalog entry", "Add a typed checkpoint or trim point.")])
    else:
        evidence_value = entry.get("evidence")
        evidence_path = _path_evidence(evidence_value)
        findings: list[IntegrationPilotFinding] = []
        evidence = [_relative(catalog_path), str(evidence_value)]
        if evidence_path is not None and not evidence_path.is_file():
            findings.append(_finding("error", "operating-point-evidence-missing", _relative(evidence_path), "catalog evidence path does not exist", "Regenerate the source-backed evidence artifact."))
        points = _stage("operating_points", "blocked" if findings else "passed", "the family has a typed operating-point or trajectory-checkpoint entry", evidence, findings, {"point_id": entry.get("point_id"), "point_type": entry.get("point_type")})
    return trim, points
    ####


def _fidelity_stage(
    manifest_path: Path,
    manifest: Mapping[str, object],
    record: Mapping[str, object],
) -> IntegrationPilotStage:
    collection = _collection(manifest_path, manifest)
    plant = record.get("plant")
    runtime_fidelity = str(plant.get("fidelity", "unknown")) if isinstance(plant, Mapping) else "unknown"
    qualification_class = collection.qualification_class if collection is not None else "reference_exact"
    findings: list[IntegrationPilotFinding] = []
    profile_statuses: list[str] = []
    profile_records: list[dict[str, object]] = []
    family_path = manifest_path.parent / "family.yaml"
    if family_path.is_file():
        family = _read_yaml(family_path)
        raw_profiles = family.get("fidelity_profiles")
        if isinstance(raw_profiles, list):
            for item in raw_profiles:
                if not isinstance(item, Mapping):
                    continue
                status_value = str(item.get("status", ""))
                profile_statuses.append(status_value)
                profile_records.append(
                    {
                        "profile_id": str(item.get("profile_id", "")),
                        "runtime_fidelity": str(item.get("runtime_fidelity", "")),
                        "status": status_value,
                    }
                )
    deferred_profiles = [status for status in profile_statuses if any(token in status.lower() for token in ("deferred", "planned", "development", "pending"))]
    eligible_profiles = [
        item["profile_id"]
        for item in profile_records
        if item["profile_id"]
        and any(token in str(item["status"]).lower() for token in ("qualified", "passed", "verified"))
    ]
    if qualification_class == "surrogate_composite":
        findings.append(_finding("warning", "surrogate-fidelity-boundary", _relative(manifest_path), "this fidelity combines authorities and is not source-exact", "Do not lower or promote it as manufacturer-authoritative rigid-body evidence."))
        status: PilotStatus = "development"
    elif deferred_profiles:
        findings.append(_finding("warning", "fidelity-profile-deferred", _relative(family_path), f"one or more declared fidelity profiles remain deferred: {deferred_profiles}", "Promote only the profiles with their own evidence and retain deferred tiers as explicit gates."))
        status = "development"
    else:
        status = "passed"
    return _stage("fidelity", status, "the runtime fidelity and evidence class are explicit", [_relative(manifest_path)], findings, {"runtime_fidelity": runtime_fidelity, "qualification_class": qualification_class, "profile_statuses": profile_statuses, "profiles": profile_records, "automatic_lowering_eligible": (qualification_class == "derived_exact" and not deferred_profiles) or bool(eligible_profiles), "automatic_lowering_eligible_profiles": eligible_profiles})
    ####


def _controller_stage(
    family_id: str,
    manifest_path: Path,
    record: Mapping[str, object],
    contract: Mapping[str, object] | None,
) -> IntegrationPilotStage:
    if _applicability(contract, "tuning", record) == "not_applicable" or family_id == "reference_nesc_two_stage_rocket":
        return _stage("controller", "not_applicable", "controller synthesis is outside the declared open-loop or performance-only contract", [_relative(manifest_path)], [_finding("info", "controller-not-applicable", _relative(manifest_path), "no controller is declared for this pilot", "Do not infer controller evidence from source replay.")])
    raw_layers = record.get("layers")
    controller = raw_layers.get("controller", raw_layers.get("controllers")) if isinstance(raw_layers, Mapping) else None
    if controller is None and family_id == "a320_openap_3dof":
        return _stage("controller", "not_applicable", "point-mass OpenAP lane does not expose a physical attitude controller", [_relative(manifest_path)], [_finding("info", "controller-not-applicable", _relative(manifest_path), "3DOF performance lane has no attitude/effectors contract", "Keep controller claims out of this lane.")])
    return _stage("controller", "development", "controller metadata exists only as a Taoryx overlay pending independent nonlinear qualification", [_relative(manifest_path)], [_finding("warning", "controller-overlay-development", _relative(manifest_path), "controller authority is not source-exact for this pilot", "Retain command/achieved and nonlinear response evidence before promotion.")])
    ####


def _mission_stage(
    family_id: str,
    manifest_path: Path,
    manifest: Mapping[str, object],
    record: Mapping[str, object],
) -> IntegrationPilotStage:
    family_dir = manifest_path.parent
    binding_path = family_dir / "qualification/racetrack-binding.yaml"
    if binding_path.is_file():
        preflight = validate_vehicle_controller_mission_preflight(family_id)
        findings = [
            _finding(item.severity, item.code, item.path, item.message, item.hint)
            for item in (*preflight.controller.findings, *preflight.mission.findings)
        ]
        if preflight.blockers:
            return _stage(
                "mission",
                "blocked",
                "mission binding is present but its structural preflight is blocked",
                [_relative(binding_path), *preflight.mission.evidence],
                findings,
                dict(preflight.mission.metrics),
            )
        mission_artifacts = {
            "a320_openap_3dof": ROOT / "verification/a320_racetrack/point_mass_3dof_evidence.json",
            "a320_openap_jsbsim_pseudo6dof": ROOT / "verification/a320_racetrack/pseudo_6dof_kinematic_bridge_evidence.json",
        }
        mission_artifact = mission_artifacts.get(family_id)
        if mission_artifact is not None and mission_artifact.is_file():
            try:
                mission_payload = _read_json(mission_artifact)
            except (OSError, ValueError, json.JSONDecodeError) as error:
                return _stage(
                    "mission",
                    "blocked",
                    "mission evidence exists but cannot be parsed",
                    [_relative(binding_path), _relative(mission_artifact)],
                    [_finding("error", "mission-evidence-invalid", _relative(mission_artifact), str(error), "Regenerate the A320 mission evidence with the pinned runner.")],
                    dict(preflight.mission.metrics),
                )
            evaluation = mission_payload.get("evaluation")
            if isinstance(evaluation, Mapping) and evaluation.get("mission_pass") is True:
                metrics = dict(preflight.mission.metrics)
                metrics.update({"runtime_mission_pass": True, "runtime_evidence": _relative(mission_artifact)})
                return _stage(
                    "mission",
                    "passed",
                    "shared A320 racetrack executed and all required truth gates passed",
                    [_relative(binding_path), _relative(mission_artifact)],
                    [],
                    metrics,
                )
            metrics = dict(preflight.mission.metrics)
            metrics.update({"runtime_mission_pass": False, "runtime_evidence": _relative(mission_artifact)})
            findings.append(_finding("warning", "mission-execution-failed", _relative(mission_artifact), "the A320 runner produced evidence but one or more truth objectives failed", "Inspect the independent objective table before promoting the mission lane."))
            return _stage("mission", "development", "A320 mission runner exists but its truth-gate packet is not passing", [_relative(binding_path), _relative(mission_artifact), *preflight.mission.evidence], findings, metrics)
        findings.append(_finding("warning", "mission-execution-pending", _relative(binding_path), "mission geometry and timing preflight pass, but this pilot does not execute the vehicle mission", "Run the family mission runner and independent truth evaluator before promotion."))
        return _stage(
            "mission",
            "development",
            "reusable mission binding and geometry/timing preflight are present",
            [_relative(binding_path), *preflight.mission.evidence],
            findings,
            dict(preflight.mission.metrics),
        )
    if family_id == "reference_nesc_two_stage_rocket" and (family_dir / "family.yaml").is_file():
        graph = _read_yaml(family_dir / "family.yaml").get("segment_graphs")
        if graph:
            return _stage("mission", "passed", "source Scenario 17 segment graph and terminal event contract are discoverable", [_relative(family_dir / "family.yaml")], [], {"source_segment_graph": True})
    return _stage("mission", "blocked", "mission execution requires an explicit family mission binding", [_relative(binding_path)], [_finding("error", "mission-binding-missing", _relative(binding_path), "no reusable mission binding is declared for this pilot", "Add a family mission binding before showcase or controller claims.")])
    ####


def validate_vehicle_integration_pilot(family_id: str) -> VehicleIntegrationPilotReport:
    """Run the collection/reference pilot without synthesizing missing physics."""

    snapshot = load_integration_snapshot(family_id)
    manifest_path = ROOT / snapshot.manifest_path
    record_path = ROOT / snapshot.record_path
    manifest = snapshot.manifest
    record = snapshot.record
    kind = snapshot.manifest_kind
    contract_path = ROOT / snapshot.contract_path
    contract = snapshot.contract
    stages_list = [
        _intake_stage(family_id, manifest_path, manifest, record_path, record, kind, snapshot),
        _conventions_stage(manifest_path, manifest, contract_path, contract, record_path, record),
        _plant_stage(family_id, manifest_path, manifest, record_path, record),
        _effectivity_stage(family_id, manifest_path, manifest, record),
    ]
    trim, points = _trim_and_points_stage(family_id, record_path, record, contract)
    stages_list.extend((trim, points, _fidelity_stage(manifest_path, manifest, record), _controller_stage(family_id, manifest_path, record, contract), _mission_stage(family_id, manifest_path, manifest, record)))
    stages = tuple(stages_list)
    blockers = tuple(item for stage in stages for item in stage.blockers)
    caveats = tuple(item for stage in stages for item in stage.findings if item.severity != "error")
    if blockers:
        status = "blocked"
    elif any(stage.status in {"development", "planned"} for stage in stages):
        status = "development_ready_with_gates"
    else:
        status = "promotion_ready"
    completed = {stage.stage_id for stage in stages if stage.status == "passed"}
    current_tier = "T2_plant_ready" if "plant" in completed else "T0_metadata_blocked"
    next_gate = str(record.get("next_gate", "add_explicit_next_gate"))
    return VehicleIntegrationPilotReport(
        family_id=family_id,
        manifest_kind=kind,
        qualification_class=snapshot.qualification_class,
        status=status,
        current_tier=current_tier,
        next_gate=next_gate,
        promotion_eligible=not blockers and all(stage.status in {"passed", "not_applicable"} for stage in stages),
        claim_boundary="provider-neutral integration pilot evidence; not flight or operational qualification",
        snapshot=snapshot,
        stages=stages,
        blockers=blockers,
        caveats=caveats,
    )
    ####


def validate_all_vehicle_integration_pilots() -> tuple[VehicleIntegrationPilotReport, ...]:
    """Run the A320/NESC integration pilots."""

    return tuple(validate_vehicle_integration_pilot(family_id) for family_id in PILOT_FAMILIES)
    ####


def _write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def write_vehicle_integration_packet(
    family_id: str,
    output_directory: str | Path,
    *,
    report: VehicleIntegrationPilotReport | None = None,
) -> Path:
    """Write a reproducible packet for one collection/reference pilot.

    Local evidence is copied with its repository-relative path.  External
    source packages are deliberately not fabricated or copied from nowhere;
    they remain listed with their declared hash and availability state.
    """

    snapshot = load_integration_snapshot(family_id)
    resolved_report = report or validate_vehicle_integration_pilot(family_id)
    destination = Path(output_directory) / family_id
    inputs = destination / "inputs"
    destination.mkdir(parents=True, exist_ok=True)
    inputs.mkdir(parents=True, exist_ok=True)

    copied: list[dict[str, object]] = []
    copied_destinations: set[str] = set()
    for reference in snapshot.local_evidence:
        if reference.path is None:
            continue
        source = ROOT / reference.path
        if not source.is_file():
            continue
        relative = Path(reference.path)
        target = inputs / relative
        if str(relative) in copied_destinations:
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        observed = _sha256_file(target)
        copied.append(
            {
                "source": reference.path,
                "packet_path": str(Path("inputs") / relative),
                "sha256": observed,
                "declared_sha256": reference.declared_sha256,
            }
        )
        copied_destinations.add(str(relative))

    packet_manifest: dict[str, object] = {
        "schema_version": "taoryx.vehicle-integration-packet/v1",
        "family_id": family_id,
        "manifest_kind": snapshot.manifest_kind,
        "qualification_class": snapshot.qualification_class,
        "copied_inputs": copied,
        "external_dependencies": [item.as_dict() for item in snapshot.external_dependencies],
        "dependency_hashes": dict(snapshot.dependency_hashes),
    }
    _write_json(destination / "packet-manifest.json", packet_manifest)
    _write_json(destination / "snapshot.json", snapshot.as_dict())
    _write_json(destination / "integration-report.json", resolved_report.as_dict())
    (destination / "reproduction.txt").write_text(
        "python tools/validate_vehicle_integration_pilots.py "
        f"--family {family_id} --json {family_id}-report.json\n",
        encoding="utf-8",
    )
    return destination
    ####


__all__ = [
    "IntegrationPilotFinding",
    "IntegrationPilotStage",
    "EvidenceReference",
    "IntegrationSnapshot",
    "PILOT_FAMILIES",
    "STAGE_ORDER",
    "VehicleIntegrationPilotReport",
    "validate_all_vehicle_integration_pilots",
    "validate_vehicle_integration_pilot",
    "load_integration_snapshot",
    "write_vehicle_integration_packet",
]
####
