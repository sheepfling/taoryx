"""Tests for the staged provider-neutral vehicle integration report."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from taoryx.runtime.cli import main
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
    assert report.next_gate == "multi_point_scheduled_control_and_r1_robustness_review"
    assert stages["intake"].status == "passed"
    assert stages["conventions"].status == "passed"
    assert stages["plant"].status == "passed"
    assert stages["effectivity"].status == "development"
    assert stages["trim"].status == "passed"
    assert stages["operating_points"].metrics["work_item_count"] == 7
    assert stages["controller"].status == "development"
    controller_authority = stages["controller"].metrics["authority_preflights"]
    assert all(item["status"] == "passed" for item in controller_authority.values())
    assert stages["mission"].metrics["estimated_duration_s_min"] == pytest.approx(1308.1865661383836)
    assert not report.promotion_eligible
    assert report.blockers == ()
    ####


def test_hl20_pipeline_accepts_semantic_mission_binding_but_preserves_runtime_gap() -> None:
    """HL-20 semantic lowering is distinct from its still-unbound glide runtime."""

    report = validate_vehicle_integration_pipeline("reference_hl20_mod_k")
    stages = {stage.stage_id: stage for stage in report.stages}

    assert report.status == "development_ready_with_gates"
    assert stages["conventions"].status == "passed"
    assert stages["plant"].status == "passed"
    assert stages["effectivity"].status == "development"
    assert stages["trim"].status == "passed"
    assert stages["operating_points"].metrics["work_item_count"] == 1
    assert stages["controller"].status == "not_applicable"
    assert stages["mission"].status == "development"
    assert stages["mission"].metrics["preflight_status_by_fidelity"] == {
        "point_mass_3dof": "translation_ready",
        "pseudo_6dof": "translation_ready",
    }
    assert any(item.code == "mission-runtime-planned" for item in stages["mission"].findings)
    assert not report.blockers
    assert not any(item.code == "profile-not-declared" for item in report.blockers)
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


def test_hl20_packet_retains_generic_semantic_mission_binding_and_witnesses(tmp_path: Path) -> None:
    """A non-racetrack mission keeps its exact semantic preflight inputs in the handoff packet."""

    packet = tmp_path / "hl20-packet"
    report = write_vehicle_integration_packet("reference_hl20_mod_k", packet)

    assert report.status == "development_ready_with_gates"
    assert (packet / "inputs/families/reference_hl20_mod_k/qualification/mission-binding.yaml").is_file()
    assert (packet / "inputs/examples/vehicle_composition/hl20_glide_energy_capability_3dof_compose.yaml").is_file()
    assert (packet / "inputs/examples/vehicle_composition/hl20_glide_energy_capability_pseudo6dof_compose.yaml").is_file()
    ####


def test_mission_composition_cli_reports_semantic_hl20_mission_without_promoting_runtime(
    capsys: pytest.CaptureFixture[str],
) -> None:
    """A semantic mission remains development evidence when no runtime is bound."""

    exit_code = main(["vehicle", "integration", "pipeline", "reference_hl20_mod_k"])

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["schema"] == "taoryx.vehicle-integration-command/v1alpha1"
    assert payload["command"] == "pipeline"
    assert "does not execute a mission" in payload["claim_boundary"]
    report = payload["reports"][0]
    assert report["status"] == "development_ready_with_gates"
    mission = next(item for item in report["stages"] if item["stage_id"] == "mission")
    assert mission["status"] == "development"
    assert any(item["code"] == "mission-runtime-planned" for item in mission["findings"])
    ####


def test_mission_composition_cli_can_write_a_hash_bound_source_integration_packet(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """The author-facing command uses the same evidence-packet writer as CI."""

    packet = tmp_path / "f16-packet"
    exit_code = main(
        [
            "vehicle",
            "integration",
            "pipeline",
            "reference_f16_s119",
            "--packet-dir",
            str(packet),
        ]
    )

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["reports"][0]["family_id"] == "reference_f16_s119"
    assert (packet / "manifest.json").is_file()
    assert (packet / "integration_report.json").is_file()
    ####
