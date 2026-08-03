from __future__ import annotations

import json
from pathlib import Path

from taoryx.showcase.artifact_binding import validate_showcase_run_artifact_boundary
from tools.migrate_legacy_showcase_packet import migrate


def test_legacy_packet_migration_preserves_retained_evidence_boundary(tmp_path: Path) -> None:
    packet = tmp_path / "retained-packet"
    packet.mkdir()
    (packet / "summary.json").write_text(
        json.dumps(
            {
                "mission_id": "retained-v1",
                "claim": "Retained nominal evidence",
                "mission_pass": True,
                "nonclaims": ["not a family qualification"],
            }
        )
        + "\n",
        encoding="utf-8",
    )
    (packet / "truth_telemetry.csv").write_text("time_s,altitude_m\n0,100\n", encoding="utf-8")

    manifest_path = migrate(
        packet,
        family="synthetic_fixed_wing",
        vehicle_binding_id="synthetic-fixed-wing-v1",
        fidelity="rigid_body_6dof_direct_wrench",
        control_realization="direct_wrench",
        state_schema=("retained_rigid_body_6dof_telemetry",),
        evidence_grade="mixed",
        additional_nonclaims=("migration did not rerun the vehicle",),
    )

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["migration"]["rerun_performed"] is False
    artifact = validate_showcase_run_artifact_boundary(manifest["run_artifacts"][0])
    assert artifact.fidelity == "rigid_body_6dof_direct_wrench"
    assert artifact.control_realization == "direct_wrench"
    assert artifact.realization.physical_effectors == ()
    assert "migration did not rerun the vehicle" in artifact.nonclaims
    ####
