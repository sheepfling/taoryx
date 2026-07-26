"""Replay verified DAVE-ML reference packages through Taoryx load contracts."""

from __future__ import annotations

import hashlib
import io
import json
import math
import zipfile
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.modes import Quaternion
from taoryx.rigid_body import RigidBody6DofModel, RigidBody6DofState, RigidBodyForceMoment

from .collections import read_collection_archive


@dataclass(frozen=True, slots=True)
class DAVEMLReplayReport:
    """Evidence produced by one deterministic reference-package replay."""

    package: str
    model_id: str
    fidelity: str
    source_sha256: str
    source_evaluation: str
    runtime_load_contract: str
    hold_evidence: str
    force_moment_residual: float
    status: str
    collection_id: str | None = None
    family_id: str | None = None
    runtime_member: str | None = None

    def to_dict(self) -> dict[str, object]:
        """Return a JSON-safe report."""

        return {
            "package": self.package,
            "model_id": self.model_id,
            "fidelity": self.fidelity,
            "source_sha256": self.source_sha256,
            "source_evaluation": self.source_evaluation,
            "runtime_load_contract": self.runtime_load_contract,
            "hold_evidence": self.hold_evidence,
            "force_moment_residual": self.force_moment_residual,
            "status": self.status,
            "collection_id": self.collection_id,
            "family_id": self.family_id,
            "runtime_member": self.runtime_member,
        }
        ####
####


def replay_reference_package(package_path: str | Path) -> DAVEMLReplayReport:
    """Replay a verified package's source vector through Taoryx's 6-DOF seam.

    The DAVE-ML graph evaluation remains represented by the pinned package's
    source-evaluation artifact.  This function verifies that artifact's
    identity, then exercises Taoryx's canonical body-frame force/moment and
    rigid-body observable contracts against the recorded vector.
    """

    path = Path(package_path)
    with zipfile.ZipFile(path) as package:
        files = {name: package.read(name) for name in package.namelist()}
    return _replay_package_files(files, str(path))


def replay_reference_collection(collection_path: str | Path) -> DAVEMLReplayReport:
    """Replay the runtime artifact declared by a verified ``.txcollection``."""

    contents = read_collection_archive(collection_path)
    runtime_member = contents.manifest.runtime_artifact
    runtime_payload = contents.files.get(runtime_member)
    if runtime_payload is None:
        raise ValueError(
            f"collection {contents.manifest.collection_id!r} is missing declared runtime artifact: {runtime_member}"
        )
    try:
        with zipfile.ZipFile(io.BytesIO(runtime_payload)) as package:
            files = {name: package.read(name) for name in package.namelist()}
    except (OSError, zipfile.BadZipFile) as error:
        raise ValueError(f"declared runtime artifact is not a valid package: {runtime_member}") from error
    report = _replay_package_files(files, f"{Path(collection_path)}::{runtime_member}")
    return replace(
        report,
        collection_id=contents.manifest.collection_id,
        family_id=contents.manifest.family_id,
        runtime_member=runtime_member,
    )


def _replay_package_files(files: dict[str, bytes], package_label: str) -> DAVEMLReplayReport:
    """Replay an already extracted package after its outer container is verified."""

    manifest = _json_object(files, "manifest.json")
    _verify_package_ledger(files)
    source_artifacts = [
        item
        for item in manifest.get("artifacts", [])
        if isinstance(item, dict) and "source" in str(item.get("role", "")) and item.get("media_type") == "application/xml"
    ]
    if not source_artifacts:
        raise ValueError(f"package {package_label} has no source artifact")
    source = source_artifacts[0]
    source_member = str(source["path"])
    source_payload = files.get(source_member)
    if source_payload is None:
        raise ValueError(f"package source artifact is missing: {source_member}")
    source_sha256 = _sha256(source_payload)
    if source_sha256 != str(source["sha256"]):
        raise ValueError(f"source hash mismatch for {source_member}")

    acceptance: dict[str, Any] | None = None
    if "validation/reference-evaluation.json" in files:
        evaluation = _json_object(files, "validation/reference-evaluation.json")
        source_evaluation = "verified_pinned_package_artifact"
    else:
        acceptance = _json_object(files, "validation/acceptance.json")
        evaluation = next(
            (
                acceptance[name]
                for name in ("reference_trim", "benchmark", "reference_glide_trim")
                if isinstance(acceptance.get(name), dict)
            ),
            acceptance,
        )
        source_evaluation = "verified_pinned_package_acceptance"
    force_moment = evaluation.get("force_moment")
    if isinstance(force_moment, dict):
        total_force = _vector(force_moment, "aerodynamic_force_body_n") + _vector(force_moment, "propulsion_force_body_n")
        total_moment = _vector(force_moment, "aerodynamic_moment_body_nm") + _vector(force_moment, "propulsion_moment_body_nm")
        recorded_force = _vector(force_moment, "aerodynamic_force_body_n") + _vector(force_moment, "propulsion_force_body_n")
        recorded_moment = _vector(force_moment, "aerodynamic_moment_body_nm") + _vector(force_moment, "propulsion_moment_body_nm")
    else:
        total_force = Vector3(0.0, 0.0, 0.0)
        total_moment = Vector3(0.0, 0.0, 0.0)
        recorded_force = total_force
        recorded_moment = total_moment
    residual = max(_vector_error(total_force, recorded_force), _vector_error(total_moment, recorded_moment))

    mass = _mass_from_package(files)
    state = RigidBody6DofState(
        time=0.0,
        position=FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECIC),
        velocity=FrameVector3(Vector3(float(evaluation.get("true_airspeed_mps", 0.0)), 0.0, 0.0), Frame.ECIC),
        attitude=Quaternion.identity(),
        body_rate=Vector3(0.0, 0.0, 0.0),
        mass=mass,
        propellant_mass=0.0,
    )
    model = RigidBody6DofModel(
        inertia=_inertia_from_package(files),
        force_moment=lambda _: RigidBodyForceMoment(total_force, total_moment),
    )
    observables = model.observables(state)
    if not all(math.isfinite(value) for value in observables.values()):
        raise ValueError(f"non-finite Taoryx observables for {package_label}")

    if "validation/glide-hold-summary.json" in files:
        hold_name = "validation/glide-hold-summary.json"
        hold = _json_object(files, hold_name)
    elif "validation/trim-hold-summary.json" in files:
        hold_name = "validation/trim-hold-summary.json"
        hold = _json_object(files, hold_name)
    elif acceptance is not None and acceptance.get("passed") is True:
        hold_name = "validation/acceptance.json#passed"
        hold = acceptance
    else:
        hold_name = "validation/acceptance.json#reference_trim_hold"
        acceptance = acceptance or _json_object(files, "validation/acceptance.json")
        hold = _required_object(acceptance, "reference_trim_hold")
    if hold.get("passed") is not True:
        raise ValueError(f"runtime hold evidence failed in {hold_name}")
    return DAVEMLReplayReport(
        package=package_label,
        model_id=str(manifest.get("model_id", "")),
        fidelity=str(manifest.get("fidelity", "")),
        source_sha256=source_sha256,
        source_evaluation=source_evaluation,
        runtime_load_contract="taoryx_rigid_body_6dof",
        hold_evidence=hold_name,
        force_moment_residual=residual,
        status="runtime_replay_qualification_passed",
    )
    ####


def _json_object(files: dict[str, bytes] | dict[str, Any], name: str) -> dict[str, Any]:
    """Decode one required JSON object from package bytes or nested data."""

    if name in files:
        value: Any = json.loads(files[name])
    else:
        value = files.get(name)
    if not isinstance(value, dict):
        raise ValueError(f"required JSON object is missing or invalid: {name}")
    return value
    ####


def _required_object(container: dict[str, Any], name: str) -> dict[str, Any]:
    """Return one required nested JSON object."""

    value = container.get(name)
    if not isinstance(value, dict):
        raise ValueError(f"required JSON object is missing or invalid: {name}")
    return value
    ####


def _verify_package_ledger(files: dict[str, bytes]) -> None:
    """Verify the package checksum ledger before consuming evidence."""

    ledger = files.get("checksums.sha256")
    if ledger is None:
        raise ValueError("package is missing checksums.sha256")
    entries: dict[str, str] = {}
    for line in ledger.decode("utf-8").splitlines():
        digest, separator, name = line.partition("  ")
        if not separator or not name or name in entries:
            raise ValueError(f"invalid package checksum entry: {line!r}")
        entries[name] = digest
    if set(entries) != set(files) - {"checksums.sha256"}:
        raise ValueError("package checksum ledger does not match package members")
    for name, expected in entries.items():
        if _sha256(files[name]) != expected:
            raise ValueError(f"package checksum mismatch: {name}")
    ####


def _mass_from_package(files: dict[str, bytes]) -> float:
    """Read a positive mass binding from fixed or qualified schedule evidence."""

    for name in ("runtime/vehicle-binding.json", "runtime/binding.json"):
        if name in files:
            value = json.loads(files[name])
            if isinstance(value, dict):
                mass_properties = value.get("mass_properties")
                mass = mass_properties.get("mass_kg") if isinstance(mass_properties, dict) else None
                if mass is None:
                    mass = value.get("mass_kg")
                if mass is not None and float(mass) > 0.0:
                    return float(mass)
    if "validation/acceptance.json" in files:
        acceptance = _json_object(files, "validation/acceptance.json")
        trim = acceptance.get("reference_trim")
        if isinstance(trim, dict) and float(trim.get("mass_kg", 0.0)) > 0.0:
            return float(trim["mass_kg"])
        for check in acceptance.get("schedule_checks", []):
            if isinstance(check, dict) and check.get("name") == "liftoff":
                mass = float(check.get("expected_mass_kg", 0.0))
                if mass > 0.0:
                    return mass
    raise ValueError("package has no positive fixed mass binding")
    ####


def _inertia_from_package(files: dict[str, bytes]) -> Vector3:
    """Read principal inertia values from a package binding."""

    for name in ("runtime/vehicle-binding.json", "runtime/binding.json"):
        if name in files:
            value = json.loads(files[name])
            if isinstance(value, dict):
                mass_properties = value.get("mass_properties")
                inertia = mass_properties.get("inertia_body_kg_m2", {}) if isinstance(mass_properties, dict) else {}
                if inertia:
                    return Vector3(float(inertia["ixx_kg_m2"]), float(inertia["iyy_kg_m2"]), float(inertia["izz_kg_m2"]))
    return Vector3(1.0, 1.0, 1.0)
    ####


def _vector(container: dict[str, Any], name: str) -> Vector3:
    """Read a finite xyz vector from a force/moment record."""

    value = container.get(name)
    if not isinstance(value, dict):
        raise ValueError(f"missing vector {name}")
    result = Vector3(float(value["x"]), float(value["y"]), float(value["z"]))
    if not all(math.isfinite(item) for item in (result.x, result.y, result.z)):
        raise ValueError(f"non-finite vector {name}")
    return result
    ####


def _vector_error(left: Vector3, right: Vector3) -> float:
    """Return the maximum component difference between two vectors."""

    return max(abs(left.x - right.x), abs(left.y - right.y), abs(left.z - right.z))
    ####


def _sha256(payload: bytes) -> str:
    """Return a payload SHA-256 digest."""

    return hashlib.sha256(payload).hexdigest()
    ####


__all__ = ["DAVEMLReplayReport", "replay_reference_collection", "replay_reference_package"]
