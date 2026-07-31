from __future__ import annotations

from tools.validate_hummingbird_direct_wrench_debug import build_artifact


def test_hummingbird_direct_wrench_is_only_a_debug_comparator() -> None:
    artifact = build_artifact()
    assert artifact["status"] == "debug_comparator_pass"
    assert artifact["claim"]["evidence_tier"] == "T3_direct_wrench_bridge"
    assert artifact["claim"]["direct_body_moment_injection"] is True
    assert artifact["claim"]["physical_effector_allocation"] is False
    assert artifact["native_reference"]
    assert "motor or rotor allocation" in artifact["nonclaims"]
    ####
