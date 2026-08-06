from __future__ import annotations

import json
from pathlib import Path

import pytest

from taoryx.product_two_bundle import build_product_two_bundle
from taoryx.product_two_catalog import load_product_two_catalog
from taoryx.product_two_contracts import ProductTwoStatus
from taoryx.product_two_doctor import doctor_scenario
from taoryx.product_two_manifest import (
    ProductTwoManifestCompatibilityError,
    ProductTwoManifestSourceInput,
    build_product_two_run_manifest,
    read_product_two_run_manifest,
)
from taoryx.runtime.cli import main

ROOT = Path(__file__).resolve().parents[2]


def test_product_two_catalog_resolves_canonical_entries_and_aliases() -> None:
    catalog = load_product_two_catalog()

    assert len(catalog.scenarios) == 8
    assert catalog.find("hl20-pseudo6dof").id == "hl20-source-release-pseudo6dof"
    assert [item.id for item in catalog.search("hawaii")] == [
        "hl20-four-fidelity",
        "synthetic-california-hawaii",
        "interactive-california-hawaii",
    ]
    ####


def test_product_two_doctor_covers_every_canonical_scenario() -> None:
    catalog = load_product_two_catalog()

    reports = [doctor_scenario(scenario) for scenario in catalog.scenarios]

    assert all(report["status"] == "passed" for report in reports)
    assert all(not report["diagnostics"] for report in reports)
    assert all(report["source_inputs"] for report in reports)
    ####


def test_product_two_cli_discovery_commands_are_machine_readable(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["scenario", "list", "--json"]) == 0
    listed = json.loads(capsys.readouterr().out)
    assert listed["schema"] == "taoryx.product-two-scenario-catalog/v1alpha1"
    assert len(listed["scenarios"]) == 8

    assert main(["scenario", "show", "hl20-pseudo6dof", "--json"]) == 0
    shown = json.loads(capsys.readouterr().out)
    assert shown["id"] == "hl20-source-release-pseudo6dof"
    assert shown["expected_disposition"] == "development"
    ####


def test_product_two_manifest_round_trip_preserves_additive_fields(tmp_path: Path) -> None:
    source = ProductTwoManifestSourceInput(path="inputs/mission.prb", role="problem", sha256="a" * 64, bytes=12)
    manifest = build_product_two_run_manifest(
        scenario_id="manifest-fixture",
        status=ProductTwoStatus.PASSED,
        expected_disposition=ProductTwoStatus.DEVELOPMENT,
        operation="batch",
        fidelity="point_mass_3dof",
        realization="source_runtime",
        source_inputs=(source,),
        integration={"integrator": "rk4", "max_steps": 20},
        time={"requested_duration_s": 1.0, "accepted_start_s": 0.0, "accepted_end_s": 1.0},
        termination={"completed": True, "reason": "terminal"},
        claim_boundary="fixture only",
    ).model_copy(update={"future_field": {"preserved": True}})
    path = manifest.write_json(tmp_path / "run-manifest.json")

    loaded = read_product_two_run_manifest(path)

    assert loaded.run_identity == manifest.run_identity
    assert loaded.model_extra == {"future_field": {"preserved": True}}
    ####


def test_product_two_manifest_rejects_unknown_schema(tmp_path: Path) -> None:
    source = ProductTwoManifestSourceInput(path="inputs/mission.prb", role="problem", sha256="b" * 64, bytes=12)
    manifest = build_product_two_run_manifest(
        scenario_id="schema-fixture",
        status=ProductTwoStatus.PASSED,
        expected_disposition=ProductTwoStatus.PASSED,
        operation="batch",
        fidelity="point_mass_3dof",
        realization="source_runtime",
        source_inputs=(source,),
        claim_boundary="fixture only",
    )
    payload = manifest.model_dump(mode="json", by_alias=True)
    payload["schema"] = "taoryx.product-two-run-manifest/v2alpha1"
    path = tmp_path / "unknown.json"
    path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ProductTwoManifestCompatibilityError, match="unsupported run-manifest schema"):
        read_product_two_run_manifest(path)
    ####


def test_product_two_bundle_collects_inputs_manifest_and_reproduction(tmp_path: Path) -> None:
    scenario = load_product_two_catalog().find("two-stage-ballistic")

    result = build_product_two_bundle(scenario, tmp_path / "bundle", run=False, plots=False)
    manifest = read_product_two_run_manifest(tmp_path / "bundle/run-manifest.json")

    assert result["status"] == "incomplete"
    assert (tmp_path / "bundle/inputs/examples/mission_families/two_stage_ballistic_rocket/mission.prb").is_file()
    assert (tmp_path / "bundle/reproduction.txt").is_file()
    assert manifest.status is ProductTwoStatus.INCOMPLETE
    assert manifest.source_inputs[0].path.startswith("inputs/")
    ####
