from __future__ import annotations

from pathlib import Path

import pytest
import yaml

SHOWCASE_ROOT = Path(__file__).resolve().parents[1].parent / "examples" / "showcases"


def test_stage_coast_entry_showcase_manifest_is_explicitly_scaffolded() -> None:
    root = SHOWCASE_ROOT / "6dof_stage_coast_entry"
    showcase = yaml.safe_load((root / "showcase.yaml").read_text(encoding="utf-8"))
    expected = yaml.safe_load((root / "expected.yaml").read_text(encoding="utf-8"))
    plots = yaml.safe_load((root / "plot.yaml").read_text(encoding="utf-8"))

    assert showcase["schema_version"] == 1
    assert showcase["status"] == "scaffolded"
    assert showcase["claim_boundary"]["manual_bounded"] is False
    assert showcase["claim_boundary"]["taoryx_extension"] is True
    assert expected["showcase_id"] == showcase["id"]
    assert expected["invariants"]
    assert plots["showcase_id"] == showcase["id"]
    assert {plot["id"] for plot in plots["plots"]} == {
        "phase_timeline",
        "mass_thrust",
        "ascent_apogee",
        "thermal_corridor",
        "attitude_rates",
    }
    ####


@pytest.mark.parametrize(
    "showcase_id",
    ["orbital_insertion_coast_reentry", "suborbital_ballistic_return", "quadcopter_drone_racetrack"],
)
def test_extended_vehicle_family_showcases_declare_plot_and_claim_contracts(showcase_id: str) -> None:
    root = SHOWCASE_ROOT / showcase_id
    showcase = yaml.safe_load((root / "showcase.yaml").read_text(encoding="utf-8"))
    expected = yaml.safe_load((root / "expected.yaml").read_text(encoding="utf-8"))
    plots = yaml.safe_load((root / "plot.yaml").read_text(encoding="utf-8"))

    assert showcase["id"] == showcase_id
    assert showcase["status"] == "scaffolded"
    assert showcase["claim_boundary"]["engineering_validity"] is False
    assert expected["showcase_id"] == showcase_id
    assert expected["invariants"]
    assert plots["showcase_id"] == showcase_id
    assert plots["plots"]
    assert all("id" in plot and "x" in plot and "y" in plot for plot in plots["plots"])
    ####
