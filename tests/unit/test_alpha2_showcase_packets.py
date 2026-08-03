"""Contract tests for the Alpha 2 showcase closeout artifacts."""

from __future__ import annotations

import json
from pathlib import Path

import yaml

from tools.build_cahi_showcase_packet import _evaluate

ROOT = Path(__file__).resolve().parents[2]
####


def _row(time_s: float, *, altitude_m: float = 120_000.0, range_m: float = 3_900_000.0) -> dict[str, float]:
    return {
        "time_s": time_s,
        "latitude_deg": 30.0,
        "longitude_deg": -130.0,
        "altitude_m": altitude_m,
        "range_to_target_m": range_m,
        "speed_m_s": 4_000.0,
        "mass_kg": 10_000.0 - time_s,
        "propellant_mass_kg": 2_000.0 - time_s,
        "propellant_mass_rate_kg_s": 1.0,
        "thrust_n": 100_000.0,
        "pro_nav_active": 1.0 if time_s >= 1_650.0 else 0.0,
        "aero_mach": 8.0,
        "aero_table_operational_margin": 0.5,
        "translation_equation_residual_normalized": 1.0e-12,
        "rotation_equation_residual_normalized": 1.0e-12,
        "total_force_n": 100_000.0,
    }
####


def test_cahi_endpoint_is_independent_of_phase_completion() -> None:
    rows = [_row(0.0), _row(180.0), _row(360.0), _row(700.0), _row(1_650.0)]
    evaluation = _evaluate(
        rows,
        {
            "trajectory-1-when-1-4": 180.0,
            "trajectory-1-when-2-4": 360.0,
            "trajectory-1-when-3-4": 700.0,
            "trajectory-1-when-4-4": 1_650.0,
        },
    )
    assert evaluation["mission_pass"] is False
    assert evaluation["objectives"][-1]["id"] == "honolulu-terminal-endpoint"
    assert evaluation["objectives"][-1]["status"] == "fail"
####


def test_alpha2_catalog_declares_four_requested_packs() -> None:
    catalog = yaml.safe_load((ROOT / "verification/alpha2_showcase_catalog.yaml").read_text(encoding="utf-8"))
    assert [item["family"] for item in catalog["showcases"]] == ["skywalker_x8", "b747", "hummingbird", "synthetic_x8_plus_boosters"]
    assert catalog["showcases"][-1]["objective_status"] == "fail_terminal_endpoint"
####


def test_generated_alpha2_catalog_has_honest_statuses_when_present() -> None:
    path = ROOT / "artifacts/showcases/alpha2/final-catalog-v1/catalog.json"
    if not path.exists():
        return
    catalog = json.loads(path.read_text(encoding="utf-8"))
    records = {item["id"]: item for item in catalog["families"]}
    assert records["cahi-x8-plus-boosters"]["mission_pass"] is False
    # The canonical surface-allocation racetrack is intentionally fail-closed
    # when the two-elevon witness reaches the source beta boundary.  A
    # numerical failure must remain visible in the catalog rather than being
    # mistaken for the prior direct-wrench or aggregate nominal result.
    assert records["x8-racetrack"]["mission_pass"] is False
    assert records["x8-racetrack"]["status"] == "integration_failure_qualification_blocked"
    assert records["hummingbird-pad-to-pad"]["mission_pass"] is True
####
