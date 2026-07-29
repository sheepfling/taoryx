from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from taoryx.vehicle_integration_pilot import (
    PILOT_FAMILIES,
    STAGE_ORDER,
    EvidenceReference,
    _intake_stage,
    load_integration_snapshot,
    validate_all_vehicle_integration_pilots,
    validate_vehicle_integration_pilot,
    write_vehicle_integration_packet,
)


def test_a320_and_nesc_pilots_share_a_stable_stage_contract() -> None:
    reports = validate_all_vehicle_integration_pilots()

    assert tuple(report.family_id for report in reports) == PILOT_FAMILIES
    for report in reports:
        assert tuple(stage.stage_id for stage in report.stages) == STAGE_ORDER
        assert report.claim_boundary
        assert report.as_dict()["schema_version"] == "taoryx.vehicle-integration-pilot/v1"
    ####


def test_nesc_open_loop_pilot_does_not_invent_trim_or_controls() -> None:
    report = validate_vehicle_integration_pilot("reference_nesc_two_stage_rocket")
    stages = {stage.stage_id: stage for stage in report.stages}

    assert report.manifest_kind == "reference"
    assert report.status == "development_ready_with_gates"
    assert stages["intake"].status == "passed"
    assert stages["conventions"].status == "passed"
    assert stages["plant"].status == "passed"
    assert stages["trim"].status == "not_applicable"
    assert stages["effectivity"].status == "not_applicable"
    assert stages["controller"].status == "not_applicable"
    assert stages["mission"].status == "passed"
    assert stages["plant"].metrics["nesc_lineage"]["staging_event_count"] == 5
    assert stages["plant"].metrics["nesc_lineage"]["reduction_pass"] is True
    assert stages["plant"].metrics["nesc_lineage"]["parent_qualification_unchanged_by_child"] is True
    assert stages["fidelity"].status == "development"
    assert stages["fidelity"].metrics["automatic_lowering_eligible_profiles"] == [
        "nesc_rocket.performance_3dof.v1",
        "nesc_rocket.variable_mass_6dof.v1",
    ]
    assert report.promotion_eligible is False
    ####


def test_a320_derived_exact_lane_has_runtime_truth_mission_gate() -> None:
    report = validate_vehicle_integration_pilot("a320_openap_3dof")
    stages = {stage.stage_id: stage for stage in report.stages}

    assert report.manifest_kind == "collection"
    assert report.qualification_class == "derived_exact"
    assert report.status == "promotion_ready"
    assert stages["plant"].status == "passed"
    assert stages["trim"].status == "passed"
    assert stages["operating_points"].status == "passed"
    assert stages["effectivity"].status == "not_applicable"
    assert stages["fidelity"].status == "passed"
    assert stages["fidelity"].metrics["automatic_lowering_eligible"] is True
    assert stages["controller"].status == "not_applicable"
    assert stages["mission"].status == "passed"
    assert stages["mission"].metrics["realization_count"] == 1
    assert stages["mission"].metrics["runtime_mission_pass"] is True
    assert not any(item.code == "mission-execution-pending" for item in report.caveats)
    ####


def test_a320_pseudo6dof_retains_surrogate_and_controller_boundaries() -> None:
    report = validate_vehicle_integration_pilot("a320_openap_jsbsim_pseudo6dof")
    stages = {stage.stage_id: stage for stage in report.stages}

    assert report.qualification_class == "surrogate_composite"
    assert stages["plant"].status == "passed"
    assert stages["effectivity"].status == "development"
    assert stages["fidelity"].status == "development"
    assert stages["controller"].status == "development"
    assert stages["fidelity"].metrics["automatic_lowering_eligible"] is False
    assert any(item.code == "surrogate-fidelity-boundary" for item in report.caveats)
    assert report.promotion_eligible is False
    ####


def test_unknown_pilot_family_fails_closed() -> None:
    with pytest.raises(FileNotFoundError):
        validate_vehicle_integration_pilot("not_a_vehicle")
    ####


def test_snapshot_distinguishes_external_and_inline_evidence() -> None:
    a320 = load_integration_snapshot("a320_openap_3dof")
    nesc = load_integration_snapshot("reference_nesc_two_stage_rocket")

    a320_external = {item.reference for item in a320.external_dependencies}
    assert "qualified-models/a320-openap/runtime/openap-a320.json" in a320_external
    assert any(item.kind == "inline_status" and item.reference == "272/272 oracle comparisons" for item in a320.evidence)
    assert a320.dependency_hashes["source_documents.a320-openap.runtime.source_path"] == "987464f4a7ba818ca9b727201b550cdec57df8673c30329c82237f602d13da63"
    assert nesc.applicability["trim"] == "not_applicable"
    assert nesc.applicability["tuning"] == "not_applicable"
    assert any(item.kind == "local_path" and item.path == "families/reference_nesc_two_stage_rocket/family.yaml" for item in nesc.local_evidence)
    ####


def test_collection_packet_copies_local_inputs_and_retains_external_hashes(tmp_path) -> None:
    packet = write_vehicle_integration_packet("a320_openap_3dof", tmp_path)

    assert (packet / "packet-manifest.json").is_file()
    assert (packet / "snapshot.json").is_file()
    assert (packet / "integration-report.json").is_file()
    assert (packet / "inputs/families/a320_openap_3dof/collection-manifest.json").is_file()
    manifest = (packet / "packet-manifest.json").read_text(encoding="utf-8")
    assert "qualified-models/a320-openap/runtime/openap-a320.json" in manifest
    assert "987464f4a7ba818ca9b727201b550cdec57df8673c30329c82237f602d13da63" in manifest
    ####


def test_local_dependency_hash_drift_blocks_intake() -> None:
    snapshot = load_integration_snapshot("a320_openap_3dof")
    drifted = replace(
        snapshot,
        evidence=snapshot.evidence
        + (
            EvidenceReference(
                label="negative-control",
                kind="local_path",
                reference="verification/daveml_a320_openap_integration.json",
                path="verification/daveml_a320_openap_integration.json",
                declared_sha256="0" * 64,
                observed_sha256="1" * 64,
                available=True,
            ),
        ),
    )
    stage = _intake_stage(
        "a320_openap_3dof",
        Path(snapshot.manifest_path),
        snapshot.manifest,
        Path(snapshot.record_path),
        snapshot.record,
        snapshot.manifest_kind,
        drifted,
    )
    assert stage.status == "blocked"
    assert any(item.code == "dependency-hash-mismatch" for item in stage.blockers)
    ####
