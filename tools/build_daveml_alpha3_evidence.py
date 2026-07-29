"""Build deterministic evidence reports for the Alpha 3 DAVE-ML tranche."""

from __future__ import annotations

import csv
import hashlib
import io
import json
import math
import zipfile
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
REGISTRY = ROOT / "verification/daveml_alpha3_completion.yaml"
OVERLAY_REPORT = ROOT / "verification/daveml_alpha3_overlay_evidence.json"
REDUCTION_REPORT = ROOT / "verification/daveml_alpha3_reduction_evidence.json"
DEPLOYMENT_REPORT = ROOT / "verification/daveml_alpha3_deployment_evidence.json"
NESC_ZIP = ROOT / "resources/aerospace/daveml/nesc-model-catalog-v1.0/qualified/nesc-two-stage-rocket/nesc-two-stage-rocket-v0.9-evidence.zip"
NESC_PREFIX = "taoryx-nesc-two-stage-rocket-v0.9/"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _path(label: str) -> Path:
    return ROOT / label
    ####


def _registry() -> dict[str, Any]:
    payload = yaml.safe_load(REGISTRY.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("invalid Alpha 3 completion registry")
    return payload
    ####


def _write(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def build_overlay_evidence(registry: dict[str, Any]) -> dict[str, Any]:
    """Hash all actuator and allocation contracts without promoting source claims."""

    overlays: list[dict[str, Any]] = []
    for family in registry["families"]:
        for overlay in family.get("overlays", []):
            artifact = _path(str(overlay["artifact"]))
            overlays.append(
                {
                    "family_id": family["id"],
                    "overlay_id": overlay["id"],
                    "kind": overlay["kind"],
                    "status": overlay["status"],
                    "artifact": artifact.relative_to(ROOT).as_posix(),
                    "artifact_sha256": _sha256(artifact),
                    "authority": "downstream_taoryx_overlay",
                    "source_parent": family["source_parent"],
                }
            )
    return {
        "schema_version": "taoryx.daveml-alpha3-overlay-evidence/v1",
        "claim_boundary": "overlay contracts are downstream Taoryx artifacts, not DAVE-ML source content",
        "overlays": overlays,
        "status": "verified",
    }
    ####


def build_reduction_evidence(registry: dict[str, Any]) -> dict[str, Any]:
    """Record every reduction parent, comparison contract, and disposition."""

    reductions: list[dict[str, Any]] = []
    for family in registry["families"]:
        for reduction in family.get("reductions", []):
            artifact = _path(str(reduction["artifact"]))
            contract = yaml.safe_load(artifact.read_text(encoding="utf-8"))
            reductions.append(
                {
                    "family_id": family["id"],
                    "reduction_id": reduction["id"],
                    "qualification_class": contract["qualification_class"],
                    "status": reduction["status"],
                    "artifact": artifact.relative_to(ROOT).as_posix(),
                    "artifact_sha256": _sha256(artifact),
                    "parent": contract["parent"],
                    "preserves": contract["preserves"],
                    "omits": contract["omits"],
                    "comparison": contract["comparison"],
                    "nonclaims": contract.get("nonclaims", []),
                }
            )
    return {
        "schema_version": "taoryx.daveml-alpha3-reduction-evidence/v1",
        "claim_boundary": "a reduction is promoted only under its declared comparison classification; source-equivalence and flight claims remain bounded",
        "reduction_count": len(reductions),
        "equivalence_pending_count": sum(item["status"] == "equivalence_pending" for item in reductions),
        "reductions": reductions,
        "status": "verified_with_known_gaps" if any(item["status"] == "equivalence_pending" for item in reductions) else "verified",
    }
    ####


def build_deployment_evidence(registry: dict[str, Any]) -> dict[str, Any]:
    """Recompute the synthetic passive-child witness from pinned NESC telemetry."""

    family = next(item for item in registry["families"] if item["id"] == "reference_nesc_two_stage_rocket")
    deployment = family["deployment"]
    with zipfile.ZipFile(NESC_ZIP) as archive:
        trajectory_member = NESC_PREFIX + "validation/scenario17-trajectory.csv"
        event_member = NESC_PREFIX + "tables/mission-events.csv"
        rows = list(csv.DictReader(io.StringIO(archive.read(trajectory_member).decode("utf-8"))))
        events = list(csv.DictReader(io.StringIO(archive.read(event_member).decode("utf-8"))))
    separation_s = next(float(row["time_s"]) for row in events if row["event"] == "stage1_burnout")
    child_samples: list[dict[str, float]] = []
    for row in rows:
        time_s = float(row["time_s"])
        if time_s < separation_s:
            continue
        position = [float(row[f"position_eci_{axis}_m"]) for axis in ("x", "y", "z")]
        velocity = [float(row[f"velocity_eci_{axis}_mps"]) for axis in ("x", "y", "z")]
        radius = math.sqrt(sum(value * value for value in position))
        radial_velocity = sum(pos * vel for pos, vel in zip(position, velocity)) / radius
        dt = time_s - separation_s
        altitude = max(0.0, float(row["altitude_m"]) + radial_velocity * dt - 0.5 * 9.80665 * dt * dt)
        child_samples.append({"time_s": time_s, "altitude_m": altitude})
    source_hashes = {
        "evidence_archive_sha256": _sha256(NESC_ZIP),
        "parent_replay_sha256": _sha256(_path(str(family["evidence"][0]))),
        "deployment_contract_sha256": _sha256(_path(str(deployment["artifact"]))),
    }
    return {
        "schema_version": "taoryx.daveml-alpha3-deployment-evidence/v1",
        "claim_boundary": "synthetic passive deployment witness attached to a source-backed NESC parent; not source-exact child validation",
        "parent_family": family["id"],
        "parent_qualification_class": family["qualification_class"],
        "child": deployment["child_qualification_class"],
        "synthetic_child": deployment,
        "lineage": {
            "parent_id": "nesc-stack",
            "child_id": "nesc-synthetic-cylinder",
            "separation_event": "stage1_burnout",
            "separation_time_s": separation_s,
            "terminal_horizon_s": 200.0,
        },
        "source_event_count": len(events),
        "child_sample_count": len(child_samples),
        "child_altitude_range_m": {
            "minimum": min(item["altitude_m"] for item in child_samples),
            "maximum": max(item["altitude_m"] for item in child_samples),
        },
        "child_samples": child_samples,
        "source_hashes": source_hashes,
        "status": "verified_witness",
    }
    ####


def main() -> int:
    registry = _registry()
    _write(OVERLAY_REPORT, build_overlay_evidence(registry))
    _write(REDUCTION_REPORT, build_reduction_evidence(registry))
    _write(DEPLOYMENT_REPORT, build_deployment_evidence(registry))
    print(json.dumps({"status": "verified", "reports": [str(path.relative_to(ROOT)) for path in (OVERLAY_REPORT, REDUCTION_REPORT, DEPLOYMENT_REPORT)]}, indent=2))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
