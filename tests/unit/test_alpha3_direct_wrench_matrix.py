"""Tests for the Alpha 3 direct-wrench family matrix."""

from __future__ import annotations

from tools.build_alpha3_direct_wrench_matrix import build


def test_direct_wrench_matrix_is_fail_closed() -> None:
    report = build()
    assert report["summary"]["family_count"] == 9
    assert report["summary"]["direct_wrench_artifact_count"] == 8
    assert report["summary"]["nominal_direct_wrench_pass_count"] == 8
    assert report["summary"]["direct_wrench_bridge_pass_count"] == 8
    records = {record["family_id"]: record for record in report["records"]}
    assert records["reference_nesc_two_stage_rocket"]["status"] == "nominal_case_pass"
    assert records["tumbling_body"]["status"] == "not_applicable_native_uncontrolled"
    ####
