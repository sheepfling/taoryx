from __future__ import annotations

from pathlib import Path

import yaml
from taoryx_hl20.resources import model_resource_root

from taoryx.vehicle_trim_orchestration import orchestrate_trim_recipe


def test_f16_recipe_builds_source_ordered_worklist() -> None:
    report = orchestrate_trim_recipe("reference_f16_s119")

    assert report.status == "ready_for_adapter"
    assert report.recipe_id == "f16-s119-force-moment-equilibrium-v1"
    assert len(report.work_items) == 7
    assert report.work_items[0].trim_spec.control_initial["power_pct"] == 8.93225837337992
    assert report.work_items[-1].point_id == "f16-7p5km-152mps"
    assert any(item.code == "operating-point-residuals-pending" for item in report.findings)
    ####


def test_hl20_recipe_builds_control_free_pitch_worklist() -> None:
    report = orchestrate_trim_recipe("reference_hl20_mod_k")

    assert report.status == "ready_for_adapter"
    assert report.recipe_id == "hl20-mod-k-pitch-channel-trim-v1"
    assert len(report.work_items) == 1
    spec = report.work_items[0].trim_spec
    assert spec.control_names == ()
    assert spec.state_initial["alpha_deg"] == 6.457652069267908
    ####


def test_hl20_package_recipe_validates_its_own_evidence_root() -> None:
    """A wheel-installed family must not need the checkout root for trim data."""

    root = model_resource_root()
    report = orchestrate_trim_recipe(
        "reference_hl20_mod_k",
        root / "families/reference_hl20_mod_k/qualification/trim-recipe.yaml",
        resource_root=root,
    )

    assert report.status == "ready_for_adapter"
    assert len(report.work_items) == 1
    ####


def test_invalid_recipe_blocks_without_invoking_solver(tmp_path: Path) -> None:
    recipe_path = tmp_path / "trim-recipe.yaml"
    catalog_path = tmp_path / "operating-points.yaml"
    recipe_path.write_text(
        yaml.safe_dump(
            {
                "kind": "taoryx.trim-recipe/v1alpha1",
                "family": "test-family",
                "recipe_id": "invalid",
                "adapter": "test.adapter",
                "claim_boundary": "test only",
                "operating_point_catalog": catalog_path.name,
                "variables": [],
                "residuals": [],
            }
        ),
        encoding="utf-8",
    )

    report = orchestrate_trim_recipe("test-family", recipe_path)

    assert report.status == "blocked"
    assert not report.work_items
    assert any(item.code == "recipe-variables-missing" for item in report.findings)
    ####
