from __future__ import annotations

import hashlib

import pytest

from taoryx.showcase import (
    ArtifactFile,
    EvidenceBoardSpec,
    FamilyShowcaseTemplate,
    FidelityShowcaseRealization,
    MissionSegmentSpec,
    ShowcaseRunArtifact,
    StartContract,
    TerminalContract,
    VehicleShowcaseBinding,
)


def _template() -> FamilyShowcaseTemplate:
    return FamilyShowcaseTemplate(
        id="org.taoryx.showcase.synthetic.route",
        version="0.1.0",
        family="powered_fixed_wing",
        display_name="Synthetic route flagship",
        claim="The declared synthetic vehicle completes its route contract.",
        start_contract=StartContract(type="trimmed_airborne"),
        segments=(
            MissionSegmentSpec(id="outbound", objective="capture-outbound", success_event="outbound-captured"),
            MissionSegmentSpec(id="return", objective="capture-return", success_event="return-captured"),
        ),
        required_events=("outbound-captured", "return-captured"),
        terminal_contract=TerminalContract(success_event="return-captured", corridor={"position_m": 2.0}),
        required_semantic_controls=("command.bank", "command.throttle"),
        plot_modules=("trajectory_3d", "attitude_and_rates", "terminal_corridor"),
    )


def test_showcase_template_has_family_local_segments_and_explicit_terminal() -> None:
    template = _template()

    assert [segment.id for segment in template.segments] == ["outbound", "return"]
    assert template.terminal_contract.timeout_is_success is False
    assert "command.bank" in template.required_semantic_controls


def test_showcase_template_rejects_duplicate_segment_ids() -> None:
    with pytest.raises(ValueError, match="segment IDs must be unique"):
        FamilyShowcaseTemplate(
            **_template().model_dump(mode="python") | {
                "segments": (_template().segments[0], _template().segments[0])
            }
        )


def test_showcase_run_artifact_requires_hashed_manifest_and_declares_fidelity() -> None:
    digest = hashlib.sha256(b"manifest").hexdigest()
    artifact = ShowcaseRunArtifact(
        run_id="run-001",
        showcase_id="org.taoryx.showcase.synthetic.route",
        vehicle_binding_id="synthetic-v1",
        fidelity="pseudo_6dof",
        scenario_contract_sha256=digest,
        outcome="completed",
        claim="Declared synthetic route completed.",
        files=(ArtifactFile(path="manifest.json", sha256=digest, media_type="application/json"),),
        board=EvidenceBoardSpec(profile="evidence-board-v1", modules=("trajectory_3d", "terminal_corridor")),
    )

    assert artifact.board.event_marker_policy == "family_local"
    assert artifact.files[0].path == "manifest.json"


def test_showcase_binding_and_realization_preserve_claim_boundary() -> None:
    binding = VehicleShowcaseBinding(
        id="synthetic-v1",
        family="powered_fixed_wing",
        version="1.0.0",
        vehicle_package="synthetic-research-surrogate",
        start_state_factory="synthetic.trim",
        evidence_grade="synthetic",
        unsupported_behaviors=("stall",),
    )
    realization = FidelityShowcaseRealization(
        fidelity="rigid_body_6dof",
        realization_id="synthetic-rigid-v1",
        state_schema=("position_ecef_m", "quaternion_xyzw", "body_rates_rad_s"),
        semantic_command_mapping={"command.bank": "aileron_deg"},
        physical_effectors=("aileron_deg",),
        available_physics=("forces", "moments", "actuator_rate_limits"),
        claim="Rigid-body response inside the synthetic declared envelope.",
        nonclaims=("flight qualification",),
        evidence_grade="synthetic",
    )

    assert binding.evidence_grade == "synthetic"
    assert "flight qualification" in realization.nonclaims
