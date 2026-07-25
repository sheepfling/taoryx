from __future__ import annotations

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
BACKLOG = ROOT / "verification" / "alpha2_post_release_backlog.yaml"
MATURITY = ROOT / "verification" / "vehicle_maturity_registry.yaml"
RELEASE_PLAN = ROOT / "verification" / "alpha2_release_plan.yaml"
####


def test_alpha2_post_release_backlog_has_unique_ordered_items() -> None:
    """The expanded backlog has stable IDs and resolvable dependencies."""

    payload = yaml.safe_load(BACKLOG.read_text(encoding="utf-8"))
    items = payload["items"]
    ids = [item["id"] for item in items]
    assert len(ids) == len(set(ids))
    assert payload["baseline"]["completion_signal"] == "A2-RELEASE-PASS"
    boundaries = payload["release_boundaries"]
    assert boundaries["alpha2_closeout"]["required_maturity"] == "M4"
    required = set(boundaries["alpha2_closeout"]["required_items"])
    deferred = set(boundaries["alpha3_breadth"]["deferred_items"])
    assert required.isdisjoint(deferred)
    assert "A2-POST-P0-1" in required
    assert "A2-POST-P0-5" in required
    assert "A2-POST-P2-5" in deferred
    assert {item["priority"] for item in items} == {"P0", "P1", "P2", "P3", "P4"}
    known = set(ids) | {"A2-RELEASE-PASS"}
    assert all(dependency in known for item in items for dependency in item.get("depends_on", []))
    assert payload["ranked_sequence"] == ids[:10] + [
        "A2-POST-P2-4",
        "A2-POST-P2-5",
        "A2-POST-P3-2",
        "A2-POST-P2-6",
        "A2-POST-P2-3",
        "A2-POST-P2-1",
        "A2-POST-P2-2",
        "A2-POST-P3-1",
        "A2-POST-P3-2A",
        "A2-POST-P3-3",
        "A2-POST-P4-1",
    ]
    by_name = {item["name"]: item for item in items}
    assert by_name["reusable-trim-generation-and-operating-point-procedure"]["target_release"] == "alpha2"
    assert by_name["passive-tumbling-deployable-body-geometry-qualification"]["target_release"] == "alpha2"
    assert by_name["guidance-search-and-reachability-workbench"]["target_release"] == "alpha3"
    assert by_name["sensor-weather-and-measurement-layer"]["target_release"] == "alpha3"
    assert by_name["light-propeller-aircraft-family"]["priority"] == "P2"
    assert by_name["rotorcraft-vtol-transition-family"]["priority"] == "P2"
    assert by_name["small-business-jet-family"]["priority"] == "P2"
    assert by_name["spacecraft-reference-family"]["priority"] == "P2"
    assert by_name["flight-dynamics-surrogate-library"]["priority"] == "P2"
    assert by_name["spacecraft-reference-family"]["lane"] == "domain-pilot"
    assert by_name["light-propeller-aircraft-family"]["research_intake"].startswith(
        "verification/small_aircraft_research_intake_v1.yaml#"
    )
    assert by_name["small-business-jet-family"]["research_intake"].startswith(
        "verification/small_aircraft_research_intake_v1.yaml#"
    )
    rotorcraft = by_name["rotorcraft-vtol-transition-family"]
    assert rotorcraft["research_intake"].endswith(
        "verification/rotorcraft_tiltrotor_research_intake_v1.yaml"
    )
    assert "tiltrotor.xv15_class.p6dof_conversion.v1" in rotorcraft[
        "catalog_variants"
    ]
    assert "tiltrotor.v22_class.p6dof_scaled.v1" in rotorcraft["catalog_variants"]
    assert "rotorcraft.uh1h.p6dof_scheduled.v1" in rotorcraft["catalog_variants"]
    assert "rotorcraft.uh60a.p6dof_scheduled.v1" in rotorcraft["catalog_variants"]
    assert any("transition continuity" in entry for entry in rotorcraft["scope"])
    assert items[-1]["status"] == "blocked_external"
    ####


def test_alpha2_release_plan_moves_lab_and_flagship_out_of_core_release() -> None:
    """The release inventory does not silently claim unfinished post-release gates."""

    payload = yaml.safe_load(RELEASE_PLAN.read_text(encoding="utf-8"))
    gates = {gate["id"]: gate for gate in payload["gates"]}
    assert payload["status"] == "complete"
    assert payload["post_release_backlog"] == "verification/alpha2_post_release_backlog.yaml"
    assert gates["A2-R12"]["required"] is False
    assert gates["A2-R12"]["post_release"] is True
    assert gates["A2-R15"]["required"] is False
    assert gates["A2-R15"]["post_release"] is True
    ####


def test_vehicle_maturity_registry_keeps_intake_separate_from_alpha2() -> None:
    """New breadth is cataloged without inheriting proof-family maturity."""

    payload = yaml.safe_load(MATURITY.read_text(encoding="utf-8"))
    records = {record["id"]: record for record in payload["records"]}
    assert payload["release_policy"]["alpha2_finish_line"].startswith("M4")
    assert records["b747"]["maturity"] == "M4"
    assert records["skywalker_x8"]["maturity"] == "M3"
    assert records["r44_class"]["maturity"] == "M0"
    assert records["uh1h_scheduled"]["maturity"] == "M0"
    assert records["xv15_class"]["maturity"] == "M0"
    assert records["v22_class_scaled"]["evidence_strength"].endswith(
        "engineering_estimate"
    )
    ####
