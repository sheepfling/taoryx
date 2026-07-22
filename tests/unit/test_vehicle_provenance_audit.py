from __future__ import annotations

from tools.audit_vehicle_provenance import audit


def test_scoped_vehicle_problem_files_match_registry() -> None:
    """Every audited vehicle problem uses the registry physical contract."""

    report = audit()

    assert report["status"] == "pass", report["findings"]
    assert report["files_checked"] > 0
    assert set(report["vehicles"]) == {"b747", "skywalker_x8", "hummingbird", "x15"}
    assert all(count > 0 for count in report["vehicles"].values())
####
