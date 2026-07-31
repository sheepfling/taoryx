from __future__ import annotations

import json
from pathlib import Path

from tools.qualify_f16_physical_schedule_interior import qualify


def test_f16_schedule_interior_is_conservative_and_keeps_boundaries(tmp_path: Path) -> None:
    report = qualify(tmp_path / "interior.json")

    assert report["status"] == "F16_physical_schedule_interior_qualified"
    assert report["direct_body_moment_injection"] is False
    assert report["validated_interior"]["case_ids"] == ["alpha_minus_1mps", "alpha_plus_1mps"]
    assert report["validated_interior"]["case_count"] == 8
    assert report["validated_interior"]["all_cases_pass_at_all_nodes"] is True
    assert report["boundary_witnesses"]["schedule_wide_case_count"] == 24
    assert "high_yaw_rate_plus" in report["boundary_witnesses"]["case_ids"]
    persisted = json.loads((tmp_path / "interior.json").read_text(encoding="utf-8"))
    assert persisted["summary"] == report["summary"]
    ####
