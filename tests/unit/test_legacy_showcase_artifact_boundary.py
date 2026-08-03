"""Boundary tests for the legacy B747/X8 evidence builder."""

from __future__ import annotations

from taoryx.showcase import ArtifactFile, EvidenceBoardSpec, build_showcase_run_artifact, validate_showcase_run_artifact_boundary
from tools.build_b747_x8_evidence_packet import CASES, _case_realization


def _artifact(realization):
    return build_showcase_run_artifact(
        realization=realization,
        run_id=f"test-{realization.realization_id}",
        showcase_id="org.taoryx.test.legacy-evidence",
        vehicle_binding_id="test-binding",
        scenario_contract_sha256="a" * 64,
        outcome="completed",
        files=(ArtifactFile(path="manifest.json", sha256="b" * 64, media_type="application/json"),),
        board=EvidenceBoardSpec(profile="family-evidence-board-v1", modules=("trajectory_3d",)),
    )


def test_legacy_point_mass_case_declares_force_model() -> None:
    realization = _case_realization(CASES[4])
    assert realization.fidelity == "point_mass_3dof"
    assert realization.control_realization == "force_model"
    assert realization.physical_effectors == ()
    assert validate_showcase_run_artifact_boundary(_artifact(realization)) == _artifact(realization)


def test_legacy_surface_case_declares_active_effectors() -> None:
    realization = _case_realization(CASES[6])
    assert realization.fidelity == "rigid_body_6dof_surface_allocated"
    assert realization.control_realization == "surface_allocated"
    assert realization.physical_effectors == (
        "collective_elevon_deg",
        "differential_elevon_deg",
        "throttle_fraction",
    )
    assert validate_showcase_run_artifact_boundary(_artifact(realization)) == _artifact(realization)
