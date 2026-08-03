"""Boundary tests for Hummingbird aggregate and individual-rotor packets."""

from __future__ import annotations

from taoryx.showcase import ArtifactFile, EvidenceBoardSpec, build_showcase_run_artifact, validate_showcase_run_artifact_boundary
from tools.build_hummingbird_pad_to_pad_packet import _showcase_realization

SUMMARY = {
    "claim": "Synthetic Hummingbird evidence claim.",
    "nonclaims": ["family-wide qualification"],
}


def _artifact(realization):
    return build_showcase_run_artifact(
        realization=realization,
        run_id=f"test-{realization.realization_id}",
        showcase_id="org.taoryx.test.hummingbird",
        vehicle_binding_id="test-hummingbird",
        scenario_contract_sha256="a" * 64,
        outcome="partial",
        files=(ArtifactFile(path="manifest.json", sha256="b" * 64, media_type="application/json"),),
        board=EvidenceBoardSpec(profile="family-evidence-board-v1", modules=("trajectory_3d",)),
    )


def test_hummingbird_aggregate_bridge_is_direct_wrench() -> None:
    realization = _showcase_realization(SUMMARY, False)
    assert realization.fidelity == "rigid_body_6dof_direct_wrench"
    assert realization.control_realization == "direct_wrench"
    assert realization.physical_effectors == ()
    validate_showcase_run_artifact_boundary(_artifact(realization))


def test_hummingbird_individual_source_declares_rotor_effectors() -> None:
    realization = _showcase_realization(SUMMARY, True)
    assert realization.fidelity == "rigid_body_6dof_surface_allocated"
    assert realization.control_realization == "surface_allocated"
    assert realization.physical_effectors == (
        "rotor_1_speed",
        "rotor_2_speed",
        "rotor_3_speed",
        "rotor_4_speed",
    )
    validate_showcase_run_artifact_boundary(_artifact(realization))
