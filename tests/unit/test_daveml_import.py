from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

from taoryx.trajectory import DAVEMLTrimBinding, load_daveml_family_graph, load_daveml_family_import, load_reference_family_catalog

ROOT = Path(__file__).resolve().parents[2]
CATALOG_ROOT = Path(
    os.environ.get(
        "TAORYX_DAVEML_CATALOG_ROOT",
        ROOT / "INBOX/taoryx-daveml-nesc-model-catalog-v1.0",
    )
)


def test_qualified_family_sidecars_are_bound_to_the_reference_catalog() -> None:
    """Each promoted family exposes a validated DAVE-ML import boundary."""

    manifests = load_reference_family_catalog(ROOT / "verification/reference_family_catalog.yaml")
    for manifest in manifests:
        assert manifest.daveml_import == "plant/daveml-import.json"
        record = load_daveml_family_import(ROOT / "families" / manifest.family_id / manifest.daveml_import)
        assert record.family_id == manifest.family_id
        assert record.package.sha256 == manifest.source.package_sha256
        assert record.roundtrip.status == "verified"
        assert record.roundtrip.checkdata_status in {"verified", "not_present"}
        assert record.replay.status == "runtime_replay_qualification_passed"
        assert all(document.structural_diff_count == 0 for document in record.package.source_documents)
        assert all(document.numeric_diff_count == 0 for document in record.package.source_documents)


def test_family_graph_binding_verifies_package_and_source_member_before_execution() -> None:
    binding = load_daveml_family_graph(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        role="propulsion",
    )
    values = binding.evaluate(
        {"powerLeverAngle": 0.0, "altitudeMSL": 0.0, "mach": 0.0},
        ("thrustBodyForce_X",),
    )
    assert binding.family_id == "reference_f16_s119"
    assert binding.package_member == "models/propulsion.dml"
    assert binding.unit_for("thrustBodyForce_X") == "lbf"
    assert values["thrustBodyForce_X"] == 1060.0

    trim_binding = DAVEMLTrimBinding(
        graph=binding,
        state_inputs={"altitude_ft": "altitudeMSL"},
        control_inputs={"power_pct": "powerLeverAngle"},
        residual_outputs={"thrust_lbf": "thrustBodyForce_X"},
        fixed_inputs={"mach": 0.0},
    )
    assert trim_binding.evaluate({"altitude_ft": 0.0}, {"power_pct": 0.0}) == {"thrust_lbf": 1060.0}


@pytest.mark.skipif(not CATALOG_ROOT.is_dir(), reason="local DAVE-ML catalog is external to the repository")
def test_full_catalog_import_report_covers_sources_packages_and_library_targets() -> None:
    """The importer keeps source-ready records distinct from runtime families."""

    report = json.loads((ROOT / "verification/daveml_catalog_import.json").read_text(encoding="utf-8"))
    assert report["status"] == "verified"
    assert report["source_document_count"] == 22
    assert report["qualified_package_count"] == 3
    assert report["model_family_count"] == 9
    assert len(report["sources"]) == 22
    assert len(report["qualified_packages"]) == 3
    assert sum(item["library_status"] == "qualified_reference_family" for item in report["family_library"]) == 3
    assert all(item["roundtrip"]["status"] == "verified" for item in report["qualified_packages"])
    assert all(item["replay"]["status"] == "runtime_replay_qualification_passed" for item in report["qualified_packages"])
