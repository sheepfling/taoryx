"""Tests for the staged provider-neutral vehicle integration report."""

from __future__ import annotations

from pathlib import Path

from taoryx.vehicle_integration_pipeline import (
    STAGE_ORDER,
    validate_all_vehicle_integration_pipelines,
    validate_vehicle_integration_pipeline,
    write_vehicle_integration_packet,
)


def test_f16_pipeline_separates_verified_plant_from_development_overlays() -> None:
    """F-16 source replay passes while effectivity and mission remain development evidence."""

    report = validate_vehicle_integration_pipeline("reference_f16_s119")
    stages = {stage.stage_id: stage for stage in report.stages}

    assert tuple(stage.stage_id for stage in report.stages) == STAGE_ORDER
    assert report.status == "development_ready_with_gates"
    assert report.current_tier == "T2_source_plant_ready"
    assert stages["intake"].status == "passed"
    assert stages["conventions"].status == "passed"
    assert stages["plant"].status == "passed"
    assert stages["effectivity"].status == "development"
    assert stages["trim"].status == "passed"
    assert stages["operating_points"].metrics["work_item_count"] == 7
    assert stages["controller"].status == "development"
    assert stages["mission"].metrics["estimated_duration_s_min"] == 1408.1865661383833
    assert not report.promotion_eligible
    assert report.blockers == ()
    ####


def test_hl20_pipeline_fails_closed_for_missing_expected_direct_profile() -> None:
    """HL-20 does not inherit a missing direct-wrench profile by association."""

    report = validate_vehicle_integration_pipeline("reference_hl20_mod_k")
    stages = {stage.stage_id: stage for stage in report.stages}

    assert report.status == "blocked"
    assert stages["conventions"].status == "passed"
    assert stages["plant"].status == "passed"
    assert stages["effectivity"].status == "development"
    assert stages["trim"].status == "passed"
    assert stages["operating_points"].metrics["work_item_count"] == 1
    assert stages["controller"].status == "not_applicable"
    assert stages["mission"].status == "blocked"
    assert any(item.code == "mission-binding-missing" for item in report.blockers)
    assert any(item.code == "profile-not-declared" for item in report.blockers)
    ####


def test_all_supported_pipelines_have_stable_stage_order_and_json_shape() -> None:
    """The report contract is uniform across the two conformance families."""

    reports = validate_all_vehicle_integration_pipelines()

    assert {report.family_id for report in reports} == {"reference_f16_s119", "reference_hl20_mod_k"}
    for report in reports:
        payload = report.as_dict()
        assert payload["schema_version"] == "taoryx.vehicle-integration-pipeline/v1"
        assert [stage["stage_id"] for stage in payload["stages"]] == list(STAGE_ORDER)
        assert payload["claim_boundary"]
    ####


def test_packet_writer_copies_hashed_inputs_and_report(tmp_path: Path) -> None:
    """A pilot report can be handed off as a reproducible evidence packet."""

    packet = tmp_path / "f16-packet"
    report = write_vehicle_integration_packet("reference_f16_s119", packet)

    assert report.family_id == "reference_f16_s119"
    manifest = (packet / "manifest.json").read_text(encoding="utf-8")
    assert '"source_package_included": true' in manifest
    assert (packet / "integration_report.json").is_file()
    assert (packet / "effectivity_preflight.json").is_file()
    assert (packet / "controller_mission_preflight.json").is_file()
    assert (packet / "vehicle_trim_solve_evidence.json").is_file()
    assert any(item.name.endswith(".txair") for item in (packet / "inputs").rglob("*"))
    assert (packet / "inputs/families/reference_f16_s119/qualification/trim-recipe.yaml").is_file()
    assert (packet / "inputs/verification/daveml_f16_equilibrium_trim_evidence.json").is_file()
    assert (packet / "inputs/families/reference_f16_s119/controllers/local-physical-wrench-lqr-v1.yaml").is_file()
    assert (packet / "inputs/verification/f16_runtime_linearization_evidence.json").is_file()
    assert (packet / "inputs/verification/f16_physical_allocation_evidence.json").is_file()
    assert (packet / "inputs/verification/racetrack_templates.yaml").is_file()
    ####
