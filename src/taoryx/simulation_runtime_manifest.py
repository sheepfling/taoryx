"""Portable Simulation Runtime run-manifest creation and compatibility validation."""

from __future__ import annotations

import hashlib
import json
import platform
import subprocess
import sys
from collections.abc import Iterable, Mapping, Sequence
from pathlib import Path
from typing import Any

from pydantic import BaseModel, ConfigDict, Field

from taoryx.simulation_runtime_contracts import (
    SIMULATION_RUNTIME_RUN_MANIFEST_SCHEMA,
    SIMULATION_RUNTIME_RUN_MANIFEST_SCHEMA_VERSION,
    SimulationRuntimeStatus,
)

ROOT = Path(__file__).resolve().parents[2]


class SimulationRuntimeManifestCompatibilityError(ValueError):
    """The manifest is not readable by this Simulation Runtime manifest version."""


class SimulationRuntimeManifestSourceInput(BaseModel):
    """Hash-bound source input record."""

    model_config = ConfigDict(extra="allow")

    path: str = Field(min_length=1)
    role: str = Field(min_length=1)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    bytes: int = Field(ge=0)
    ####


class SimulationRuntimeManifestArtifact(BaseModel):
    """Hash-bound output artifact record."""

    model_config = ConfigDict(extra="allow")

    path: str = Field(min_length=1)
    kind: str = Field(min_length=1)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    bytes: int = Field(ge=0)
    ####


class SimulationRuntimeRunManifest(BaseModel):
    """Minimum portable Simulation Runtime run identity and evidence envelope."""

    model_config = ConfigDict(extra="allow")

    schema_id: str = Field(
        default=SIMULATION_RUNTIME_RUN_MANIFEST_SCHEMA,
        alias="schema",
        serialization_alias="schema",
    )
    schema_version: int = SIMULATION_RUNTIME_RUN_MANIFEST_SCHEMA_VERSION
    scenario_id: str = Field(min_length=1)
    status: SimulationRuntimeStatus
    expected_disposition: SimulationRuntimeStatus
    operation: str = Field(min_length=1)
    fidelity: str = Field(min_length=1)
    realization: str = Field(min_length=1)
    source_inputs: tuple[SimulationRuntimeManifestSourceInput, ...]
    run_identity: str = Field(pattern=r"^[0-9a-f]{64}$")
    runtime: dict[str, Any]
    integration: dict[str, Any]
    time: dict[str, Any]
    termination: dict[str, Any]
    artifacts: tuple[SimulationRuntimeManifestArtifact, ...]
    claim_boundary: str = Field(min_length=1)
    ####

    def canonical_identity_payload(self) -> dict[str, object]:
        """Return the input/configuration fields that define one run identity."""

        return {
            "schema": self.schema_id,
            "schema_version": self.schema_version,
            "scenario_id": self.scenario_id,
            "expected_disposition": self.expected_disposition.value,
            "operation": self.operation,
            "fidelity": self.fidelity,
            "realization": self.realization,
            "source_inputs": [item.model_dump(mode="json") for item in self.source_inputs],
            "integration": self.integration,
            "time": self.time,
        }
        ####

    def verify_identity(self) -> SimulationRuntimeRunManifest:
        """Reject a manifest whose reproducibility identity was tampered with."""

        expected = _identity_digest(self.canonical_identity_payload())
        if self.run_identity != expected:
            raise SimulationRuntimeManifestCompatibilityError(
                f"run_identity mismatch: received {self.run_identity}, expected {expected}"
            )
        return self
        ####

    def write_json(self, path: str | Path) -> Path:
        """Write the canonical manifest without path-dependent formatting."""

        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(self.model_dump(mode="json", by_alias=True), indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return destination
        ####


def read_simulation_runtime_run_manifest(path: str | Path, *, verify_identity: bool = True) -> SimulationRuntimeRunManifest:
    """Read a manifest and enforce the M0 schema compatibility policy."""

    source = Path(path)
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise SimulationRuntimeManifestCompatibilityError(f"could not read run manifest {source}: {error}") from error
    if not isinstance(payload, Mapping):
        raise SimulationRuntimeManifestCompatibilityError("run manifest must contain one JSON object")
    schema = payload.get("schema")
    version = payload.get("schema_version")
    if schema != SIMULATION_RUNTIME_RUN_MANIFEST_SCHEMA:
        raise SimulationRuntimeManifestCompatibilityError(
            f"unsupported run-manifest schema: received {schema!r}, supported {SIMULATION_RUNTIME_RUN_MANIFEST_SCHEMA!r}"
        )
    if version != SIMULATION_RUNTIME_RUN_MANIFEST_SCHEMA_VERSION:
        raise SimulationRuntimeManifestCompatibilityError(
            f"unsupported run-manifest schema_version: received {version!r}, supported {SIMULATION_RUNTIME_RUN_MANIFEST_SCHEMA_VERSION}"
        )
    try:
        manifest = SimulationRuntimeRunManifest.model_validate(payload)
    except ValueError as error:
        raise SimulationRuntimeManifestCompatibilityError(f"invalid run manifest: {error}") from error
    return manifest.verify_identity() if verify_identity else manifest
    ####


def build_simulation_runtime_run_manifest(
    *,
    scenario_id: str,
    status: SimulationRuntimeStatus,
    expected_disposition: SimulationRuntimeStatus,
    operation: str,
    fidelity: str,
    realization: str,
    source_inputs: Sequence[SimulationRuntimeManifestSourceInput],
    runtime: Mapping[str, Any] | None = None,
    integration: Mapping[str, Any] | None = None,
    time: Mapping[str, Any] | None = None,
    termination: Mapping[str, Any] | None = None,
    artifacts: Sequence[SimulationRuntimeManifestArtifact] = (),
    claim_boundary: str,
    reproduction_command: Sequence[str] = (),
) -> SimulationRuntimeRunManifest:
    """Build a validated manifest from a scenario result and its evidence."""

    runtime_payload = dict(runtime or default_runtime_identity())
    integration_payload = dict(integration or {})
    time_payload = dict(time or {})
    payload = {
        "schema": SIMULATION_RUNTIME_RUN_MANIFEST_SCHEMA,
        "schema_version": SIMULATION_RUNTIME_RUN_MANIFEST_SCHEMA_VERSION,
        "scenario_id": scenario_id,
        "expected_disposition": expected_disposition.value,
        "operation": operation,
        "fidelity": fidelity,
        "realization": realization,
        "source_inputs": [item.model_dump(mode="json") for item in source_inputs],
        "integration": integration_payload,
        "time": time_payload,
    }
    extra: dict[str, Any] = {}
    if reproduction_command:
        extra["reproduction_command"] = list(reproduction_command)
    manifest_payload: dict[str, Any] = {
        "schema": SIMULATION_RUNTIME_RUN_MANIFEST_SCHEMA,
        "scenario_id": scenario_id,
        "status": status,
        "expected_disposition": expected_disposition,
        "operation": operation,
        "fidelity": fidelity,
        "realization": realization,
        "source_inputs": tuple(source_inputs),
        "run_identity": _identity_digest(payload),
        "runtime": runtime_payload,
        "integration": integration_payload,
        "time": time_payload,
        "termination": dict(termination or {}),
        "artifacts": tuple(artifacts),
        "claim_boundary": claim_boundary,
        **extra,
    }
    manifest = SimulationRuntimeRunManifest.model_validate(manifest_payload)
    return manifest.verify_identity()
    ####


def source_input_record(path: str | Path, *, role: str, root: Path = ROOT) -> SimulationRuntimeManifestSourceInput:
    """Hash one input using a portable repository-relative path where possible."""

    source = Path(path)
    data = source.read_bytes()
    return SimulationRuntimeManifestSourceInput(path=_portable_path(source, root), role=role, sha256=_sha256_bytes(data), bytes=len(data))
    ####


def artifact_inventory(root: str | Path, *, exclude: Iterable[str] = ("run-manifest.json",)) -> tuple[SimulationRuntimeManifestArtifact, ...]:
    """Inventory files beneath an output root, excluding self-referential files."""

    directory = Path(root)
    excluded = set(exclude)
    records: list[SimulationRuntimeManifestArtifact] = []
    for path in sorted(item for item in directory.rglob("*") if item.is_file() and item.name not in excluded and item.name != ".DS_Store"):
        relative = path.relative_to(directory)
        records.append(
            SimulationRuntimeManifestArtifact(
                path=str(relative),
                kind=_artifact_kind(path),
                sha256=_sha256_file(path),
                bytes=path.stat().st_size,
            )
        )
    return tuple(records)
    ####


def default_runtime_identity() -> dict[str, str]:
    """Return host and source identity needed to interpret a run."""

    try:
        revision = subprocess.check_output(("git", "rev-parse", "HEAD"), cwd=ROOT, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        revision = "unknown"
    return {
        "python": platform.python_version(),
        "platform": platform.platform(),
        "executable": str(sys.executable),
        "git_revision": revision,
    }
    ####


def _identity_digest(payload: Mapping[str, object]) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
    ####


def _portable_path(path: Path, root: Path) -> str:
    resolved = path.resolve()
    try:
        return str(resolved.relative_to(root.resolve()))
    except ValueError:
        return str(path)
    ####


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()
    ####


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
    ####


def _artifact_kind(path: Path) -> str:
    suffix = path.suffix.casefold()
    return {
        ".json": "json",
        ".csv": "telemetry_csv",
        ".sqlite": "sqlite",
        ".png": "plot",
        ".txt": "text",
    }.get(suffix, "file")
    ####


__all__ = [
    "SimulationRuntimeManifestArtifact",
    "SimulationRuntimeManifestCompatibilityError",
    "SimulationRuntimeManifestSourceInput",
    "SimulationRuntimeRunManifest",
    "artifact_inventory",
    "build_simulation_runtime_run_manifest",
    "default_runtime_identity",
    "read_simulation_runtime_run_manifest",
    "source_input_record",
]
####
