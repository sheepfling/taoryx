from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.controller_inventory import ControllerInventory, ControllerInventoryEntry
from tools.audit_controller_inventory import audit

ROOT = Path(__file__).resolve().parents[2]


def test_controller_inventory_covers_active_four_family_paths_and_designs() -> None:
    report = audit()

    assert report["status"] == "pass"
    assert report["active_entry_count"] == 7
    assert report["baseline_entry_count"] == 4
    assert report["missing_design_inventory"] == []


def test_inventory_rejects_qualification_eligible_scenario_override() -> None:
    with pytest.raises(ValueError, match="cannot qualify"):
        ControllerInventoryEntry(
            id="invalid",
            family="synthetic",
            vehicle_id="synthetic",
            classification="primary_regulator",
            implementation="lqr",
            source="test",
            path=("lqr", "plant"),
            active_runtime=True,
            scenario_gain_overrides=True,
            qualification_eligible=True,
            gain_owner="test",
        )


def test_inventory_requires_an_active_runtime_path() -> None:
    with pytest.raises(ValueError, match="at least one active"):
        ControllerInventory(
            id="empty",
            claim_boundary="test",
            entries=(
                ControllerInventoryEntry(
                    id="candidate",
                    family="synthetic",
                    vehicle_id="synthetic",
                    classification="primary_regulator",
                    implementation="lqr",
                    source="test",
                    path=("lqr", "plant"),
                    gain_owner="test",
                ),
            ),
        )
