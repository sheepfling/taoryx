"""Regression coverage for normalized public transport contracts."""

from __future__ import annotations

from types import MappingProxyType

import pytest

from taoryx.composition_episode import NativeActionValues
from taoryx.composition_policy import parse_composition_policy_trace_record
from taoryx.racetrack_template import RacetrackBinding, resolve_racetrack_binding
from taoryx.sensor_api import packet_from_record, parse_measurement_packet_record
from taoryx.vehicle_interface import CommittedNativeStatusValues


def _racetrack_values() -> dict[str, object]:
    """Return the smallest valid typed racetrack binding source record."""

    return {
        "vehicle_id": "test-vehicle",
        "fidelity": "point_mass_3dof",
        "straight_length_m": 500.0,
        "turn_radius_m": 100.0,
        "speed_m_s": 50.0,
        "low_altitude_m": 100.0,
        "high_altitude_m": 200.0,
        "climb_rate_m_s": 10.0,
        "descent_rate_m_s": 10.0,
        "left_turn_bank_deg": -20.0,
        "right_turn_bank_deg": 20.0,
    }
    ####


def test_racetrack_resolver_accepts_only_a_validated_binding() -> None:
    """The route resolver no longer indexes a raw unbounded configuration map."""

    binding = RacetrackBinding.model_validate(_racetrack_values())
    route = resolve_racetrack_binding("powered_fixed_wing_racetrack_v1", "test", binding)

    assert route.vehicle_id == "test-vehicle"
    with pytest.raises(ValueError, match="extra_forbidden"):
        RacetrackBinding.model_validate({**_racetrack_values(), "unexpected": "bag"})
    ####


def test_committed_native_status_values_snapshot_its_top_level_mapping() -> None:
    """Committed native rows cannot change after crossing the projection seam."""

    source = {"velocity_m_s": 12.5}
    values = CommittedNativeStatusValues(source)
    source["velocity_m_s"] = 99.0

    assert values["velocity_m_s"] == 12.5
    with pytest.raises(ValueError, match="string keys"):
        CommittedNativeStatusValues({1: "not a native channel"})
    ####


def test_native_action_values_snapshot_the_lowered_control_map() -> None:
    """A plug-in cannot alter a lowered native action after boundary validation."""

    source = {"elevator": 0.25}
    values = NativeActionValues(source)
    source["elevator"] = -0.25

    assert values["elevator"] == 0.25
    ####


def test_sensor_packet_parser_normalizes_checkpoint_envelopes() -> None:
    """Checkpoint maps become a stable packet record before codec restoration."""

    record = parse_measurement_packet_record(
        MappingProxyType(
            {
                "sampled_at_s": 1.0,
                "available_at_s": 1.5,
                "valid": False,
                "payload": None,
            }
        )
    )

    assert record["payload_contract"] == {}
    assert packet_from_record(record).valid is False
    with pytest.raises(ValueError, match="payload must be a mapping"):
        parse_measurement_packet_record(
            {
                "sampled_at_s": 1.0,
                "available_at_s": 1.5,
                "payload": "opaque",
            }
        )
    ####


def test_policy_trace_parser_normalizes_untrusted_parity_evidence() -> None:
    """Parity adapters receive a fresh, portable trace envelope rather than a foreign map."""

    record = parse_composition_policy_trace_record(
        MappingProxyType(
            {
                "schema": "taoryx.composition-policy-trace/v1alpha1",
                "composition_id": "candidate",
                "composition_identity_sha256": "a" * 64,
                "interface_id": "test-interface",
                "interface_fingerprint_sha256": "b" * 64,
                "authority_profile_id": "guidance",
                "observation_profile_id": "state",
                "initial_observation": MappingProxyType({"values": MappingProxyType({"altitude_m": 10.0})}),
                "steps": (MappingProxyType({"action_frame": MappingProxyType({"values": {}})}),),
                "final_observation": MappingProxyType({"values": {}}),
                "final_status": MappingProxyType({"values": {}}),
                "stopped_by_policy": True,
                "integration_step_s": 0.02,
            }
        )
    )

    assert record["steps"] == [{"action_frame": {"values": {}}}]
    assert record["integration_step_s"] == 0.02
    with pytest.raises(ValueError, match="only JSON-shaped values"):
        parse_composition_policy_trace_record(
            {
                "schema": "taoryx.composition-policy-trace/v1alpha1",
                "composition_id": "candidate",
                "composition_identity_sha256": "a" * 64,
                "interface_id": "test-interface",
                "interface_fingerprint_sha256": "b" * 64,
                "authority_profile_id": "guidance",
                "observation_profile_id": "state",
                "initial_observation": {"values": object()},
                "steps": [],
                "final_observation": {"values": {}},
                "final_status": {"values": {}},
                "stopped_by_policy": True,
                "integration_step_s": None,
            }
        )
    ####
