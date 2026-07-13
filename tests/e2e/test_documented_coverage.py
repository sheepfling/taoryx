from __future__ import annotations

from tools.build_e2e_documented_coverage import collect


def test_every_documented_surface_has_positive_case_coverage() -> None:
    payload = collect()

    assert payload["summary"] == {
        "block_scopes_covered": 33,
        "block_scopes_total": 33,
        "table_types_covered": 19,
        "table_types_total": 19,
        "table_operations_covered": 28,
        "table_operations_total": 28,
    }
    assert not [item for category in ("block_scopes", "table_types", "table_operations") for item in payload[category] if not item["covered"]]
    ####
