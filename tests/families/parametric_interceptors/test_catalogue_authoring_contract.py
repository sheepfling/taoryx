"""Strict, discoverable authoring contract for catalogue-backed interceptors."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from pydantic import ValidationError
from taoryx_parametric_interceptors import (
    interceptor_authoring_schema_bundle,
    interceptor_from_catalogue_record,
)
from taoryx_parametric_interceptors.__main__ import main


def test_catalogue_contract_rejects_unknown_fields_at_each_structural_level() -> None:
    with pytest.raises(ValidationError, match="launch_mass_kg"):
        interceptor_from_catalogue_record(
            {
                "interceptor_id": "int:misplaced-field",
                "launch_mass_kg": 100.0,
            }
        )

    with pytest.raises(ValidationError, match="confidnce"):
        interceptor_from_catalogue_record(
            {
                "interceptor_id": "int:misspelled-evidence-field",
                "evidence": {
                    "launch_mass": {
                        "value": 100.0,
                        "unit": "kg",
                        "origin": "observed",
                        "confidnce": "high",
                    }
                },
            }
        )

    with pytest.raises(ValidationError, match="archetyep"):
        interceptor_from_catalogue_record(
            {
                "interceptor_id": "int:misspelled-assumption-field",
                "resolver_assumptions": {"archetyep": "generic_slender_sam_v1"},
            }
        )
    ####


def test_catalogue_contract_rejects_contradictory_gap_and_value_records() -> None:
    with pytest.raises(ValidationError, match="populated evidence cannot also supply a gap status"):
        interceptor_from_catalogue_record(
            {
                "interceptor_id": "int:value-and-gap",
                "evidence": {
                    "launch_mass": {
                        "value": 100.0,
                        "unit": "kg",
                        "origin": "observed",
                        "status": "not_resolved",
                    }
                },
            }
        )

    with pytest.raises(ValidationError, match="null evidence cannot also supply origin"):
        interceptor_from_catalogue_record(
            {
                "interceptor_id": "int:gap-and-origin",
                "evidence": {
                    "launch_mass": {
                        "value": None,
                        "unit": "kg",
                        "origin": "observed",
                        "status": "not_resolved",
                    }
                },
            }
        )
    ####


def test_catalogue_contract_rejects_alias_collisions_and_unknown_status() -> None:
    with pytest.raises(ValueError, match=r"supplied more than once at evidence\.length and derived\.length_m"):
        interceptor_from_catalogue_record(
            {
                "interceptor_id": "int:duplicate-length",
                "evidence": {
                    "length": {
                        "value": 4.0,
                        "unit": "m",
                        "origin": "observed",
                    }
                },
                "derived": {
                    "length_m": {
                        "value": 4.0,
                        "unit": "m",
                        "method": "copied_by_mistake",
                    }
                },
            }
        )

    with pytest.raises(ValueError, match="unsupported catalogue resolution_status"):
        interceptor_from_catalogue_record(
            {
                "interceptor_id": "int:misspelled-status",
                "resolution_status": "source_domniant",
            }
        )
    ####


def test_catalogue_contract_preserves_profile_version_and_calibration_identity() -> None:
    profile = interceptor_from_catalogue_record(
        {
            "kind": "interceptor_evidence_record",
            "interceptor_id": "int:versioned-model",
            "parameter_set_version": "2026.08-prototype.3",
            "calibration_reference_model_id": "cadac:template:sam6",
        }
    )

    assert profile.parameter_set_version == "2026.08-prototype.3"
    assert profile.calibration_reference_model_id == "cadac:template:sam6"
    ####


def test_catalogue_contract_accepts_a_provenance_bearing_thrust_schedule() -> None:
    profile = interceptor_from_catalogue_record(
        {
            "kind": "interceptor_evidence_record",
            "interceptor_id": "int:static-test-witness",
            "source_record_ids": ["source:public-static-test"],
            "thrust_profile_schedule": {
                "schedule_id": "static-test-shape-v1",
                "points": [
                    {"burn_fraction": 0.0, "multiplier": 1.5},
                    {"burn_fraction": 0.3, "multiplier": 1.0},
                    {"burn_fraction": 1.0, "multiplier": 0.7},
                ],
                "origin": "reported",
                "source_record_ids": ["source:public-static-test"],
                "confidence": "medium",
                "method": "relative curve transcribed from public figure",
            },
        }
    )

    assert profile.thrust_profile_schedule is not None
    assert profile.thrust_profile_schedule.origin == "reported"
    assert profile.thrust_profile_schedule.source_record_ids == ("source:public-static-test",)
    ####


def test_schema_bundle_describes_both_actual_file_surfaces() -> None:
    bundle = interceptor_authoring_schema_bundle()

    assert bundle["schema"] == "taoryx.parametric-interceptor-authoring-schema-bundle/v1"
    assert bundle["provider_id"] == "taoryx.parametric-interceptors.mission-composition"
    assert set(bundle["formats"]) == {"flat", "catalogue"}
    flat = bundle["formats"]["flat"]
    catalogue = bundle["formats"]["catalogue"]
    assert flat["additionalProperties"] is False
    assert catalogue["additionalProperties"] is False
    mass_variants = flat["properties"]["launch_mass_kg"]["anyOf"]
    assert {item.get("type") for item in mass_variants} >= {"number", "null"}
    assert all(item.get("type") != "string" for item in mass_variants)
    guidance_variants = flat["properties"]["guidance_family"]["anyOf"]
    assert {item.get("type") for item in guidance_variants} >= {"string", "null"}
    evidence_field = catalogue["$defs"]["CatalogueEvidenceField"]
    assert evidence_field["additionalProperties"] is False
    assert "ThrustProfileSchedule" in catalogue["properties"]["thrust_profile_schedule"]["anyOf"][0]["$ref"]
    ####


def test_cli_writes_selected_authoring_schema_without_loading_a_model(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    output = tmp_path / "catalogue-schema.json"

    assert main(["schema", "--format", "catalogue", "--output", str(output)]) == 0
    assert capsys.readouterr().out == f"wrote {output}\n"
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert set(payload["formats"]) == {"catalogue"}
    assert payload["formats"]["catalogue"]["$id"].endswith("parametric-interceptor-catalogue/v1")
    ####
