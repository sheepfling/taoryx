"""Regression coverage for reusable reduced-tier campaign witnesses."""

from __future__ import annotations

from collections.abc import Mapping

from tools.validate_reduced_tuning_campaigns import build_report


def test_cross_topology_reduced_campaigns_are_candidate_ready_without_manual_gains() -> None:
    report = build_report()

    assert report["status"] == "candidate_ready"
    assert report["campaign_count"] == 2
    payloads = report["campaigns"]
    assert isinstance(payloads, list)
    assert all(isinstance(item, Mapping) for item in payloads)
    campaigns: dict[str, Mapping[str, object]] = {}
    for item in payloads:
        assert isinstance(item, Mapping)
        campaign = item["campaign"]
        assert isinstance(campaign, Mapping)
        family_id = campaign["family_id"]
        assert isinstance(family_id, str)
        campaigns[family_id] = item
    assert set(campaigns) == {"hummingbird", "a320_openap_3dof"}
    for item in campaigns.values():
        assert item["status"] == "candidate_ready"
        nodes = item["nodes"]
        assert isinstance(nodes, list) and len(nodes) == 1
        node = nodes[0]
        assert isinstance(node, Mapping)
        assert node["status"] == "candidate_ready"
        lqr = node["lqr"]
        assert isinstance(lqr, Mapping)
        best_profile_id = lqr["best_profile_id"]
        assert isinstance(best_profile_id, str)
        assert best_profile_id.startswith(("hummingbird-pseudo-hover.", "a320-pseudo-cruise."))
        candidates = lqr["candidates"]
        assert isinstance(candidates, list)
        assert len(candidates) == 9
    ####
