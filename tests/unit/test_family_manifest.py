from __future__ import annotations

from taoryx.family_manifest import load_unified_family_manifest_catalog


def test_unified_manifest_joins_all_nine_families() -> None:
    catalog = load_unified_family_manifest_catalog()

    assert len(catalog.families) == 9
    assert not catalog.errors
    assert {family.family_id for family in catalog.families} == {
        "skywalker_x8",
        "b747",
        "a320_openap_3dof",
        "f16_s119",
        "x15",
        "hummingbird",
        "hl20_mod_k",
        "reference_nesc_two_stage_rocket",
        "tumbling_body",
    }
    ####


def test_unified_manifest_preserves_source_and_vehicle_authorities() -> None:
    catalog = load_unified_family_manifest_catalog()

    f16 = next(family for family in catalog.families if family.family_id == "f16_s119")
    assert f16.source_manifest is not None
    assert f16.source_manifest.family_id == "reference_f16_s119"
    x8 = next(family for family in catalog.families if family.family_id == "skywalker_x8")
    assert x8.vehicle_definition is not None
    assert x8.source_manifest is None
    a320 = next(family for family in catalog.families if family.family_id == "a320_openap_3dof")
    assert set(a320.family.data_evidence) == {"point_mass_3dof", "pseudo_6dof"}
    assert a320.family.data_evidence["point_mass_3dof"].paths == (
        "verification/daveml_a320_openap_integration.json",
    )
    assert a320.family.data_evidence["point_mass_3dof"].sha256 == {
        "verification/daveml_a320_openap_integration.json": "0fc54c2a92fe2798e75edef807d589c843e69b6cb5a49f3c20f3cf93a09d26af",
    }
    ####


def test_unified_manifest_exposes_passive_null_control_tiers() -> None:
    catalog = load_unified_family_manifest_catalog()
    tumbling = next(family for family in catalog.families if family.family_id == "tumbling_body")

    assert tumbling.direct_profile is None
    assert tumbling.surface_profile is None
    assert tumbling.family.automatic_lowering is False
    ####
