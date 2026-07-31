"""Regression checks for the normalized Alpha 3 direct-wrench contract ledger."""

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_direct_wrench_contract_covers_all_controlled_families_and_passive_exception() -> None:
    payload = json.loads((ROOT / "verification/alpha3_direct_wrench_contract/manifest.json").read_text(encoding="utf-8"))
    assert payload["status"] == "contract_verified"
    assert payload["required_dimensions"] == ["trim", "force_moment", "resources", "envelope", "claim_boundary"]
    records = {record["family_id"]: record for record in payload["records"]}
    assert len(records) == 9
    for family_id, record in records.items():
        assert record["trim"]["status"]
        assert record["force_moment"]["composition"]
        assert record["resources"]["status"]
        assert record["envelope"]["status"]
        assert record["nonclaims"]
    assert records["tumbling_body"]["control_realization"] == "native_uncontrolled"
    assert "direct_wrench_not_applicable" in records["tumbling_body"]["promotion_status"]
    ####
