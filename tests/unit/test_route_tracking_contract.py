from pathlib import Path

import yaml

from taoryx.output_catalog import output_channel_spec

ROOT = Path(__file__).resolve().parents[2]


def test_route_tracking_contract_declares_common_channels() -> None:
    contract = yaml.safe_load((ROOT / "verification/spec/route_tracking.yaml").read_text(encoding="utf-8"))
    channels = contract["channels"]

    assert set(channels) >= {
        "route_target_error_m",
        "route_cross_track_error_m",
        "route_along_track_error_m",
        "route_heading_error_deg",
        "route_bank_tracking_error_deg",
        "route_leg_index",
        "route_phase_index",
    }
    assert channels["route_cross_track_error_m"]["signed"] is True
    assert channels["route_target_error_m"]["signed"] is False
    ####


def test_route_tracking_channels_are_in_output_catalog() -> None:
    for name in (
        "route_target_error_m",
        "route_cross_track_error_m",
        "route_along_track_error_m",
        "route_heading_error_deg",
        "route_bank_tracking_error_deg",
        "route_leg_index",
        "route_phase_index",
    ):
        assert output_channel_spec(name) is not None
    ####
