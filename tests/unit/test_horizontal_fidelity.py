from __future__ import annotations

import pytest

from taoryx.family_adapter import ADAPTER_OPERATIONS
from taoryx.fidelity_contracts import CANONICAL_FIDELITY_TIERS
from taoryx.horizontal_fidelity import (
    PROMOTION_OPERATIONS,
    HorizontalTierBinding,
    load_horizontal_registry,
    validate_horizontal_fidelity,
)


def test_horizontal_registry_declares_all_current_families_and_tiers() -> None:
    registry = load_horizontal_registry()
    assert len(registry.families) == 9
    assert {item.family_id for item in registry.families} == {
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
    assert all(set(item.tiers) == set(CANONICAL_FIDELITY_TIERS) for item in registry.families)


def test_horizontal_registry_matches_catalog_and_lowers_fail_closed() -> None:
    report = validate_horizontal_fidelity()
    assert report.status == "pass"
    assert report.family_count == 9
    assert not report.errors
    tumbling = next(item for item in report.families if item.family_id == "tumbling_body")
    assert tumbling.tiers["rigid_body_6dof_surface_allocated"] is None
    assert tumbling.lowering.selected is None
    assert all(item.lowering.selected != "rigid_body_6dof_surface_allocated" for item in report.families)


def test_horizontal_promotion_gate_rejects_unknown_operations_and_ambiguous_nulls() -> None:
    with pytest.raises(ValueError, match="unknown promotion operations"):
        HorizontalTierBinding(profile_id="fixture.v1", required_operations=("invented_operation",))
    with pytest.raises(ValueError, match="null profile must be planned"):
        HorizontalTierBinding(profile_id=None, promotion_status="development")


def test_horizontal_promotion_operations_use_the_common_adapter_vocabulary() -> None:
    assert PROMOTION_OPERATIONS == set(ADAPTER_OPERATIONS)
