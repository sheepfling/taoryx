from pathlib import Path

import pytest

from taoryx.trajectory import (
    CollectionManifest,
    load_compatibility_overlay,
    load_quarantine_for_payload,
)
from taoryx.trajectory.daveml_evaluator import evaluate_daveml_checkdata

SOURCE = Path("resources/aerospace/daveml/official-conformance-v1/twoD_ungridded.dml")
OVERLAY = Path("resources/aerospace/daveml/compatibility-overlays/2c6c66492f30560a631c7905e07cc806931c3eae1d8a630f0e5168951ac3a46d.json")
QUARANTINE_DIR = Path("resources/aerospace/daveml/compatibility-overlays/quarantines")


def test_legacy_fixture_uses_only_the_pinned_compatibility_lane() -> None:
    payload = SOURCE.read_bytes()
    overlay = load_compatibility_overlay(OVERLAY, payload)
    quarantine = load_quarantine_for_payload(payload, QUARANTINE_DIR, "janus_delaunay_linear_qhull_v1")
    results = evaluate_daveml_checkdata(
        payload,
        ungridded_policy="janus_delaunay_linear_qhull_v1",
        compatibility_overlay=overlay,
        quarantine=quarantine,
    )
    assert [result.status for result in results] == ["passed", "quarantined", "passed", "passed"]
    assert results[1].actual == pytest.approx(0.235)
    assert results[1].reason_code == "legacy_triangulation_ambiguity"


def test_collection_qualification_classes_are_strict() -> None:
    manifest = CollectionManifest(
        collection_type="taoryx.txcollection/v1alpha1",
        collection_id="a320-surrogate",
        collection_version="1.0.0",
        family_id="a320_openap_jsbsim_pseudo6dof",
        qualification_class="surrogate_composite",
        manufacturer_validated=False,
        source_exact=False,
        canonical_authority="taoryx_canonical",
        source_documents=(),
        component_bindings=(),
        contribution_authority=(),
        transforms=(),
        stateful_components=(),
        runtime_artifact="runtime/aircraft.txair",
        validation_artifact="validation/report.json",
    )
    assert manifest.qualification_class == "surrogate_composite"
    with pytest.raises(ValueError):
        manifest.__class__(**{**manifest.model_dump(), "qualification_class": "manufacturer_exact"})
