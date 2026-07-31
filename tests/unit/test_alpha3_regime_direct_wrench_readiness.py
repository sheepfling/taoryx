from __future__ import annotations

from tools.build_alpha3_regime_direct_wrench_readiness import build


def test_regime_direct_wrench_readiness_is_fail_closed() -> None:
    report = build()
    assert report["status"] == "development_fail_closed"
    assert report["summary"] == {
        "blocked_or_debug_count": 4,
        "direct_wrench_bridge_count": 4,
        "promoted_direct_wrench_count": 0,
        "record_count": 4,
    }
    records = {record["family_id"]: record for record in report["records"]}
    assert records["x15"]["status"] == "direct_wrench_bridge_available_source_trim_blocked"
    assert "verification/alpha3_x15_direct_wrench/manifest.json" in records["x15"]["available_evidence"]
    assert records["reference_nesc_two_stage_rocket"]["status"] == "direct_wrench_bridge_available_gimbal_blocked"
    assert "verification/alpha3_nesc_direct_wrench/manifest.json" in records["reference_nesc_two_stage_rocket"]["available_evidence"]
    assert records["hummingbird"]["direct_wrench_mode"] == "physical_preferred"
    assert records["hl20_mod_k"]["mission_pass"] is None
    assert "verification/alpha3_hl20_direct_wrench/manifest.json" in records["hl20_mod_k"]["available_evidence"]
    ####
