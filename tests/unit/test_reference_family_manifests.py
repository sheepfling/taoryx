from __future__ import annotations

import os
from pathlib import Path

import pytest
import yaml

from taoryx.trajectory import CaseIntent, as_family_catalog, inspect_reference_package, load_reference_family_catalog, resolve_case

ROOT = Path(__file__).resolve().parents[2]
CATALOG_PATH = ROOT / "verification" / "reference_family_catalog.yaml"
CORPUS_ROOT = Path(
    os.environ.get(
        "TAORYX_DAVEML_CORPUS_ROOT",
        ROOT / "resources/aerospace/daveml/taoryx-corpus-v1.1/extracted/taoryx-aerospace-data-corpus-v1.1",
    )
)


def test_reference_family_manifests_project_into_common_catalog() -> None:
    """F-16 and HL-20 use the same provider-neutral family contract."""

    manifests = load_reference_family_catalog(CATALOG_PATH)
    catalog = as_family_catalog(manifests)
    assert {manifest.family_id for manifest in manifests} == {"reference_f16_s119", "reference_hl20_mod_k"}
    assert {family.family_id for family in catalog.families} == {
        "reference_f16_s119",
        "reference_hl20_mod_k",
    }
    assert all(
        set(family.fidelities) == {"point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"}
        for family in catalog.families
    )
    ####


def test_reference_family_source_locks_match_existing_intake_records() -> None:
    """The standardized family shape cannot silently drift from intake hashes."""

    manifests = {manifest.family_id: manifest for manifest in load_reference_family_catalog(CATALOG_PATH)}
    records = {
        "reference_f16_s119": ROOT / "families/reference_f16_s119/qualification/integration-record.yaml",
        "reference_hl20_mod_k": ROOT / "families/reference_hl20_mod_k/qualification/integration-record.yaml",
    }
    for family_id, path in records.items():
        record = yaml.safe_load(path.read_text(encoding="utf-8"))
        manifest = manifests[family_id]
        assert record["source"]["package_sha256"] == manifest.source.package_sha256
        assert record["source"].get("aerodynamics_sha256", record["source"].get("sha256")) == manifest.source.aerodynamics_sha256
        assert record["source"]["source_corpus_archive_sha256"] == manifest.source.corpus_archive_sha256
        assert record["plant"]["body_frame"] == manifest.plant.body_frame
        assert record["plant"]["navigation_frame"] == manifest.plant.navigation_frame
    ####


def test_reference_family_plant_metadata_matches_verified_package_manifests() -> None:
    """Geometry, quaternion order, and envelope are part of the library shape."""

    manifests = {manifest.family_id: manifest for manifest in load_reference_family_catalog(CATALOG_PATH)}
    f16 = manifests["reference_f16_s119"].plant
    assert f16.package_schema_version == "0.2.0"
    assert f16.quaternion_order == "wxyz"
    assert f16.reference_geometry.area_m2 == 27.870912
    assert f16.validity_envelope.mach_max == 1.0
    hl20 = manifests["reference_hl20_mod_k"].plant
    assert hl20.package_schema_version == "0.3.0"
    assert hl20.reference_geometry.mean_aerodynamic_chord_m == 8.607552
    assert hl20.validity_envelope.mach_max == 4.0
    assert hl20.validity_envelope.altitude_min_m == -1000.0
    ####


@pytest.mark.skipif(not CORPUS_ROOT.is_dir(), reason="local DAVE-ML corpus is external to the repository")
def test_local_reference_packages_match_standardized_family_manifests() -> None:
    """When the local corpus is present, bind both packages before replay."""

    for manifest in load_reference_family_catalog(CATALOG_PATH):
        report = inspect_reference_package(manifest, CORPUS_ROOT / manifest.source.package_relative_path)
        assert report.status == "verified_package_binding"
        assert report.package_fidelity == "6dof"
        assert report.artifact_count > 0
    ####


def test_reference_family_case_resolution_preserves_source_provenance() -> None:
    """A standardized source family is selectable before native replay exists."""

    catalog = as_family_catalog(tuple(load_reference_family_catalog(CATALOG_PATH)))
    for family_id, control_id, observation_id in (
        ("reference_f16_s119", "control.elevator", "aero.mach"),
        ("reference_hl20_mod_k", "control.surface.rudder", "aero.angle_of_attack"),
    ):
        intent = CaseIntent(
            case_id=f"{family_id}-baseline-contract",
            family=family_id,
            fidelity="rigid_body_6dof",
            requested_controls=(control_id,),
            requested_observations=(observation_id,),
        )
        resolved = resolve_case(intent, catalog)
        assert resolved.recompute_identity() == resolved.identity_sha256
        assert resolved.parameters["vehicle.source_package"].source == "family.default"
        assert "package_sha256=" in catalog.family(family_id).provenance
    ####


def test_reference_sidecar_bindings_are_declared_and_family_specific() -> None:
    """The directory layout keeps plant, controls, observations, and overlays separate."""

    for family_id in ("reference_f16_s119", "reference_hl20_mod_k"):
        family_root = ROOT / "families" / family_id
        manifest = yaml.safe_load((family_root / "family.yaml").read_text(encoding="utf-8"))
        for relative_path in manifest["bindings"]:
            sidecar = family_root / relative_path
            payload = yaml.safe_load(sidecar.read_text(encoding="utf-8"))
            assert payload["family"] == family_id
            assert payload["schema_version"] == 1
        assert (family_root / "actuators/ideal-direct.yaml").is_file()
        assert (family_root / "actuators/reference-first-order-v1.yaml").is_file()
    ####
