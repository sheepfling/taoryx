from __future__ import annotations

from pathlib import Path

import yaml

ROOT = Path(__file__).parents[2]


def test_angle_convention_registry_separates_bank_roll_and_query_coordinates() -> None:
    document = yaml.safe_load((ROOT / "verification/spec/angle_conventions.yaml").read_text(encoding="utf-8"))

    assert document["angles"]["bankgc"]["channel"] is None
    assert document["angles"]["rollgc"]["channel"] == "local_roll_deg"
    assert document["angles"]["flight_path_angle"]["reference"] == "local_horizon"
    assert document["angles"]["heading"]["reference"] == "local_horizon"
    assert document["query_channels"]["aero_query_bank-deg"]["physical_attitude"] is False
    assert document["query_channels"]["aero_query_bank-deg"]["plot_as_bank_by_default"] is False
    ####


def test_angle_convention_registry_keeps_euler_and_projected_sideslip_distinct() -> None:
    document = yaml.safe_load((ROOT / "verification/spec/angle_conventions.yaml").read_text(encoding="utf-8"))

    assert document["angles"]["beta"]["channel"] == "aero_sideslip_deg"
    assert document["angles"]["betae"]["must_not_alias"] == "aero_sideslip_deg"
    assert document["angles"]["alpha"]["positive_direction"] == "body_positive_z_down"
    ####
