from __future__ import annotations

import pytest
from taoryx.x15_maneuvers import X15ManeuverSpec, load_x15_maneuver_catalog, settled_x15_maneuvers
from taoryx_x15.resources import model_resource_root

ROOT = model_resource_root()
CATALOG_PATH = ROOT / "verification/x15_maneuver_catalog.yaml"

pytestmark = pytest.mark.segment


def test_x15_catalog_exposes_settled_menu_and_candidate() -> None:
    catalog = load_x15_maneuver_catalog(CATALOG_PATH)

    assert [maneuver.id for maneuver in catalog.select("settled")] == [
        "powered-ascent",
        "release-coast",
        "bank-energy-management",
        "phugoid-alpha-profile",
        "weave",
    ]
    assert catalog.get("weave").kind == "weave"
    assert catalog.get("terminal-pronav").status == "candidate"
    assert len(settled_x15_maneuvers(CATALOG_PATH)) == 5
    ####


def test_every_settled_x15_maneuver_has_passing_focused_gates() -> None:
    catalog = load_x15_maneuver_catalog(CATALOG_PATH)

    for maneuver in catalog.select("settled"):
        assert maneuver.quality_gates_pass
        assert all(gate.status == "pass" for gate in maneuver.quality_gates)
        assert maneuver.focused_test.startswith("tests/")
        assert "::test_" in maneuver.focused_test
    ####


def test_x15_catalog_sources_and_tables_resolve() -> None:
    catalog = load_x15_maneuver_catalog(CATALOG_PATH)

    for maneuver in catalog.maneuvers:
        assert (ROOT / maneuver.source_problem).is_file(), maneuver.id
        assert all((ROOT / table).is_file() for table in maneuver.tables), maneuver.id
    ####


def test_x15_catalog_rejects_settled_pending_gate() -> None:
    with pytest.raises(ValueError, match="settled maneuver"):
        X15ManeuverSpec(
            id="invalid",
            display_name="Invalid",
            kind="weave",
            mode="rigid_body_6dof",
            source_problem="fixture.prb",
            tables=("fixture.tbl",),
            focused_test="tests/test_example.py::test_example",
            status="settled",
            quality_gates=(
                {
                    "name": "pending-gate",
                    "status": "pending",
                    "evidence": "not-yet-proven",
                },
            ),
            claim_boundary="test fixture only",
        )
    ####
