"""Tests for the Alpha 3 paired-fidelity artifact index."""

from __future__ import annotations

from tools.validate_alpha3_fidelity_ladder import build_summary


def test_alpha3_index_has_all_nine_target_families_and_paired_tiers() -> None:
    summary = build_summary()

    assert summary["status"] == "pass"
    assert summary["family_count"] == 9
    assert summary["missing_families"] == []
    assert summary["incomplete_pairs"] == []
    assert summary["cross_fidelity_manifest"] == "verification/alpha3_cross_fidelity/manifest.json"
    assert all(record["mission_pass"] is True for record in summary["records"])
    assert len(summary["records"]) == 26
    assert len({(record["family_id"], record["fidelity"]) for record in summary["records"]}) == 26
    ####
