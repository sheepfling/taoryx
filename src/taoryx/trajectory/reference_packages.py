"""Preflight binding checks for immutable ``.txair`` reference packages."""

from __future__ import annotations

import hashlib
import json
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .reference_families import ReferenceFamilyManifest


class ReferencePackageBindingError(ValueError):
    """Raised when a package does not match its family manifest."""
####


@dataclass(frozen=True, slots=True)
class ReferencePackageInspection:
    """Machine-readable result of one package-to-family comparison."""

    family_id: str
    package_path: str
    package_sha256: str
    model_id: str
    package_fidelity: str
    package_schema_version: str
    artifact_count: int
    status: str

    def to_dict(self) -> dict[str, Any]:
        """Return a stable JSON-compatible report."""

        return {
            "family_id": self.family_id,
            "package_path": self.package_path,
            "package_sha256": self.package_sha256,
            "model_id": self.model_id,
            "package_fidelity": self.package_fidelity,
            "package_schema_version": self.package_schema_version,
            "artifact_count": self.artifact_count,
            "status": self.status,
        }
        ####
####


def _sha256(path: Path) -> str:
    """Hash one package without loading it twice into memory."""

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
    ####


def _json_member(package: zipfile.ZipFile, name: str) -> dict[str, Any]:
    """Load one required JSON package member."""

    try:
        value: Any = json.loads(package.read(name))
    except (KeyError, json.JSONDecodeError) as error:
        raise ReferencePackageBindingError(f"package is missing or has invalid JSON member {name!r}") from error
    if not isinstance(value, dict):
        raise ReferencePackageBindingError(f"package member {name!r} must contain a JSON object")
    return value
    ####


def _require_equal(actual: Any, expected: Any, label: str) -> None:
    """Fail with a useful field-level mismatch."""

    if actual != expected:
        raise ReferencePackageBindingError(f"{label} mismatch: package={actual!r}, family={expected!r}")
    ####


def inspect_reference_package(manifest: ReferenceFamilyManifest, package_path: str | Path) -> ReferencePackageInspection:
    """Verify an immutable package against its standardized family manifest.

    This is a package-binding check only.  It does not parse DAVE-ML or
    execute a force/moment evaluation; those remain separate replay gates.
    """

    path = Path(package_path)
    if not path.is_file():
        raise ReferencePackageBindingError(f"package does not exist: {path}")
    observed_hash = _sha256(path)
    _require_equal(observed_hash, manifest.source.package_sha256, "package_sha256")
    try:
        with zipfile.ZipFile(path) as package:
            package_manifest = _json_member(package, "manifest.json")
            _require_equal(package_manifest.get("frames", {}).get("body"), manifest.plant.body_frame, "body frame")
            _require_equal(
                package_manifest.get("frames", {}).get("navigation"),
                manifest.plant.navigation_frame,
                "navigation frame",
            )
            _require_equal(package_manifest.get("frames", {}).get("quaternion_order"), manifest.plant.quaternion_order, "quaternion order")
            _require_equal(package_manifest.get("fidelity"), "6dof", "package fidelity")
            _require_equal(package_manifest.get("schema_version"), manifest.plant.package_schema_version, "package schema version")
            _require_equal(package_manifest.get("reference_geometry"), manifest.plant.reference_geometry.model_dump(mode="json"), "reference geometry")
            _require_equal(package_manifest.get("validity_envelope"), manifest.plant.validity_envelope.model_dump(mode="json"), "validity envelope")

            artifacts = package_manifest.get("artifacts")
            if not isinstance(artifacts, list):
                raise ReferencePackageBindingError("package manifest artifacts must be a list")
            names = set(package.namelist())
            for artifact in artifacts:
                if not isinstance(artifact, dict) or not isinstance(artifact.get("path"), str):
                    raise ReferencePackageBindingError("package manifest contains an invalid artifact record")
                artifact_path = artifact["path"]
                if artifact_path not in names:
                    raise ReferencePackageBindingError(f"package artifact is absent: {artifact_path}")
                actual_artifact_hash = hashlib.sha256(package.read(artifact_path)).hexdigest()
                _require_equal(actual_artifact_hash, artifact.get("sha256"), f"artifact {artifact_path} sha256")
    except zipfile.BadZipFile as error:
        raise ReferencePackageBindingError(f"invalid txair package: {path}") from error
    return ReferencePackageInspection(
        family_id=manifest.family_id,
        package_path=str(path),
        package_sha256=observed_hash,
        model_id=str(package_manifest.get("model_id", "")),
        package_fidelity=str(package_manifest.get("fidelity", "")),
        package_schema_version=str(package_manifest.get("schema_version", "")),
        artifact_count=len(artifacts),
        status="verified_package_binding",
    )
    ####


__all__ = ["ReferencePackageBindingError", "ReferencePackageInspection", "inspect_reference_package"]
