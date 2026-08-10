"""Focused integrity checks for executable vehicle maturity claims."""

from __future__ import annotations

import yaml

from tools.validate_vehicle_maturity_registry import MATURITY_REGISTRY, validate


def test_composition_managed_maturity_records_retain_public_batch_witnesses() -> None:
    """M4 family claims stay tied to exact runnable Composition entries."""

    validate()
    records = yaml.safe_load(MATURITY_REGISTRY.read_text(encoding="utf-8"))["records"]
    maturity = {record["id"]: record for record in records}
    assert maturity["a320_openap_3dof"]["maturity"] == "M4"
    assert maturity["a320_openap_3dof"]["composition_family_id"] == "a320_openap_3dof"
    assert maturity["x15"]["composition_family_id"] == "x15"
    assert maturity["reference_hl20_mod_k"]["composition_family_id"] == "hl20_mod_k"
    assert maturity["reference_nesc_two_stage_rocket"]["maturity"] == "M4"
    assert maturity["tumbling_body"]["composition_family_id"] == "tumbling_body"
    ####
