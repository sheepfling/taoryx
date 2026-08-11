from __future__ import annotations

import numpy as np
import pytest
from taoryx.families.cadac import (
    build_cadac_sensor_integration_contract,
    cadac_local_ned_relative_state_track,
)
from taoryx.families.cadac.aim5 import mat2tr

from taoryx.sensor_plugins.relative_state import RelativeStateTrack


def test_cadac_local_ned_adapter_uses_native_typed_relative_state_projection() -> None:
    body_from_local = mat2tr(np.deg2rad(30.0), np.deg2rad(10.0))
    local_relative_position = np.asarray((800.0, -200.0, 100.0))
    local_relative_velocity = np.asarray((-80.0, 15.0, 4.0))

    track = cadac_local_ned_relative_state_track(
        time_s=4.0,
        host_position_ned_m=np.asarray((10.0, 20.0, -30.0)),
        host_velocity_ned_mps=np.asarray((120.0, 5.0, -2.0)),
        target_id="target",
        target_position_ned_m=np.asarray((10.0, 20.0, -30.0)) + local_relative_position,
        target_velocity_ned_mps=np.asarray((120.0, 5.0, -2.0)) + local_relative_velocity,
        body_from_local=body_from_local,
    )

    assert isinstance(track, RelativeStateTrack)
    expected_range = float(np.linalg.norm(local_relative_position))
    expected_unit_body = body_from_local @ (local_relative_position / expected_range)
    expected_velocity_body = body_from_local @ local_relative_velocity
    assert track.range_m == pytest.approx(expected_range)
    assert track.unit_los_sensor == pytest.approx(tuple(expected_unit_body))
    assert track.relative_velocity_sensor_mps == pytest.approx(tuple(expected_velocity_body))
    assert track.line_of_sight_rate_sensor_rad_s == pytest.approx(tuple(np.cross(expected_unit_body, expected_velocity_body) / expected_range))
    ####


def test_cadac_sensor_contract_keeps_source_gimbal_state_visible_and_sensor_bus_unclaimed() -> None:
    contract = build_cadac_sensor_integration_contract("cadac.sraam6.missile")

    assert contract.status == "available"
    assert contract.native_sensor_provider == "relative-state-track"
    assert contract.execution == "native_projection_in_source_module"
    assert contract.sensor_bus_status == "blocked"
    assert any("gimbal" in value for value in contract.source_specialised_state)

    absent = build_cadac_sensor_integration_contract("cadac.rocket6g.vehicle")
    assert absent.status == "not_applicable"
    assert absent.execution == "none"
    ####
