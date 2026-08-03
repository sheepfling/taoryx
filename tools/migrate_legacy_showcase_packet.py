#!/usr/bin/env python3
"""Wrap an existing showcase packet in the canonical run-artifact contract.

Migration is for retained evidence whose telemetry already exists but whose
older manifest predates the horizontal fidelity contract. It never reruns or
reinterprets the vehicle. The migrated realization records the selected
fidelity/control boundary, and the packet receives a nonclaim that the
migration did not regenerate the underlying run.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import mimetypes
from pathlib import Path
from typing import Any

from taoryx.showcase import (
    ArtifactFile,
    EvidenceBoardSpec,
    FidelityShowcaseRealization,
    ShowcaseOutcome,
    build_showcase_run_artifact,
)

ROOT = Path(__file__).resolve().parents[1]
####


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()
    ####


def _payload_sha256(value: object) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()
    ####


def _load(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object in {path}")
    return payload
    ####


def _outcome(summary: dict[str, Any]) -> ShowcaseOutcome:
    if bool(summary.get("mission_pass")):
        return "completed"
    if summary.get("numerical_valid") is False:
        return "numerical_failure"
    return "partial"
    ####


def _artifact_files(packet: Path, scenario_hash: str) -> tuple[ArtifactFile, ...]:
    names = [
        path.relative_to(packet).as_posix()
        for path in sorted(packet.rglob("*"))
        if path.is_file() and path.name != "manifest.json"
    ]
    names.append("manifest.json")
    return tuple(
        ArtifactFile(
            path=name,
            sha256=scenario_hash if name == "manifest.json" else _sha256(packet / name),
            media_type=mimetypes.guess_type(name)[0] or "application/octet-stream",
        )
        for name in names
    )
    ####


def migrate(
    packet: Path,
    *,
    family: str,
    vehicle_binding_id: str,
    fidelity: str,
    control_realization: str,
    state_schema: tuple[str, ...],
    evidence_grade: str,
    additional_nonclaims: tuple[str, ...] = (),
) -> Path:
    """Add canonical metadata to one retained packet in place."""

    packet = packet.resolve()
    summary_path = packet / "summary.json"
    if not summary_path.is_file():
        raise FileNotFoundError(f"legacy showcase packet has no summary.json: {packet}")
    summary = _load(summary_path)
    mission_id = str(summary.get("mission_id", packet.name))
    claim = str(summary.get("claim", f"Retained evidence for {mission_id}"))
    nonclaims = tuple(str(item) for item in summary.get("nonclaims", ())) + tuple(additional_nonclaims)
    source_files = {
        path.relative_to(packet).as_posix(): _sha256(path)
        for path in sorted(packet.rglob("*"))
        if path.is_file() and path.name != "manifest.json"
    }
    scenario_hash = _payload_sha256(
        {
            "mission_id": mission_id,
            "family": family,
            "fidelity": fidelity,
            "control_realization": control_realization,
            "source_files": source_files,
        }
    )
    realization = FidelityShowcaseRealization(
        fidelity=fidelity,
        control_realization=control_realization,
        realization_id=f"legacy.{family}.{mission_id}.canonical-wrapper-v1",
        state_schema=state_schema,
        semantic_command_mapping={"legacy_packet": "retained telemetry; no migration-time rerun"},
        physical_effectors=(),
        available_physics=("retained packet telemetry", "retained packet objective evaluation"),
        claim=claim,
        nonclaims=nonclaims,
        evidence_grade=evidence_grade,
    )
    (packet / "realized_fidelity.json").write_text(json.dumps(realization.model_dump(mode="json"), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (packet / "claim.json").write_text(
        json.dumps(
            {
                "claim": realization.claim,
                "nonclaims": list(realization.nonclaims),
                "fidelity": realization.fidelity,
                "control_realization": realization.control_realization,
                "evidence_grade": realization.evidence_grade,
                "outcome": _outcome(summary),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    showcase_run = build_showcase_run_artifact(
        realization=realization,
        run_id=f"{mission_id}-{fidelity}",
        showcase_id=f"org.taoryx.showcase.legacy.{mission_id}",
        vehicle_binding_id=vehicle_binding_id,
        scenario_contract_sha256=scenario_hash,
        outcome=_outcome(summary),
        files=_artifact_files(packet, scenario_hash),
        board=EvidenceBoardSpec(
            profile="family-evidence-board-v1",
            modules=("trajectory_3d", "mission_timeline", "dynamics_and_resources", "envelope_margins"),
        ),
        archetypes=("mission_geometry", "mission_timeline", "dynamics_and_resources", "envelope_and_qualification"),
    )
    manifest = {
        "schema_version": 1,
        "migration": {
            "kind": "legacy_showcase_packet_wrapper",
            "source_manifest": "manifest.json",
            "rerun_performed": False,
            "note": "Canonical metadata was added around retained telemetry; no vehicle simulation was executed by this migration.",
        },
        "mission_id": mission_id,
        "family": family,
        "realized_fidelity": "realized_fidelity.json",
        "run_artifacts": [showcase_run.model_dump(mode="json")],
        "files": {},
    }
    manifest_path = packet / "manifest.json"
    manifest["files"] = {
        path.relative_to(packet).as_posix(): _sha256(path)
        for path in sorted(packet.rglob("*"))
        if path.is_file() and path != manifest_path
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest_path
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--packet", type=Path, required=True)
    parser.add_argument("--family", required=True)
    parser.add_argument("--vehicle-binding-id", required=True)
    parser.add_argument("--fidelity", default="rigid_body_6dof_direct_wrench")
    parser.add_argument("--control-realization", default="direct_wrench")
    parser.add_argument("--state-schema", action="append", default=["retained_rigid_body_6dof_telemetry"])
    parser.add_argument("--evidence-grade", default="mixed")
    parser.add_argument("--nonclaim", action="append", default=[])
    arguments = parser.parse_args()
    print(
        migrate(
            arguments.packet,
            family=arguments.family,
            vehicle_binding_id=arguments.vehicle_binding_id,
            fidelity=arguments.fidelity,
            control_realization=arguments.control_realization,
            state_schema=tuple(arguments.state_schema),
            evidence_grade=arguments.evidence_grade,
            additional_nonclaims=tuple(arguments.nonclaim),
        )
    )
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
    ####
