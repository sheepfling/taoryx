from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import pytest

from taoryx.powered_fixed_wing_mission_compiler import (
    PoweredFixedWingCapability,
    PoweredFixedWingRacetrackIntent,
    compare_compiled_racetrack_to_baseline,
    compile_powered_fixed_wing_racetrack,
    load_powered_fixed_wing_mission_profiles,
    resolve_powered_fixed_wing_mission_profile,
)
from taoryx.racetrack_template import load_racetrack_template_catalog

ROOT = Path(__file__).resolve().parents[2]
PROFILES = ROOT / "verification/powered_fixed_wing_mission_profiles.yaml"


def test_compiler_reserves_turn_and_level_geometry_from_capability() -> None:
    capability, intent = load_powered_fixed_wing_mission_profiles(PROFILES)["x8-cruise"]

    compiled = compile_powered_fixed_wing_racetrack(
        capability,
        intent,
        binding_id="x8-compiled",
        fidelity="pseudo_6dof_kinematic_bridge",
    )

    expected_radius = intent.turn_radius_margin * capability.nominal_speed_m_s**2 / (9.80665 * math.tan(math.radians(10.0)))
    expected_straight = capability.nominal_speed_m_s * ((intent.high_altitude_m - intent.low_altitude_m) / capability.maximum_descent_rate_m_s + intent.level_dwell_s)
    assert compiled.status == "capability_feasible"
    assert compiled.route.turn_radius_m == pytest.approx(expected_radius)
    assert compiled.minimum_straight_length_m == pytest.approx(expected_straight)
    assert compiled.route.straight_length_m == pytest.approx(max(intent.minimum_straight_length_m, expected_straight))
    assert compiled.route.left_turn_bank_deg == pytest.approx(-10.0)
    assert compiled.route.right_turn_bank_deg == pytest.approx(10.0)
    assert compiled.route.timing.outbound_level_time_s >= intent.level_dwell_s
    assert compiled.route.timing.inbound_level_time_s >= intent.level_dwell_s
    ####


def test_compiler_reports_clipping_instead_of_silently_exceeding_capability() -> None:
    capability = PoweredFixedWingCapability(
        vehicle_id="test",
        operating_point_id="test-op",
        minimum_speed_m_s=10.0,
        nominal_speed_m_s=20.0,
        maximum_speed_m_s=30.0,
        maximum_bank_deg=20.0,
        maximum_climb_rate_m_s=2.0,
        maximum_descent_rate_m_s=2.0,
        provenance="test",
        evidence_class="synthetic",
    )
    intent = PoweredFixedWingRacetrackIntent(
        id="test-racetrack",
        low_altitude_m=100.0,
        high_altitude_m=200.0,
        requested_speed_m_s=40.0,
        requested_bank_deg=45.0,
        minimum_straight_length_m=100.0,
        level_dwell_s=10.0,
    )

    compiled = compile_powered_fixed_wing_racetrack(capability, intent)

    assert compiled.status == "capability_clipped"
    assert compiled.route.speed_m_s == pytest.approx(30.0)
    assert abs(compiled.route.right_turn_bank_deg) == pytest.approx(20.0)
    assert compiled.route.straight_length_m > intent.minimum_straight_length_m
    assert len(compiled.diagnostics) == 3
    assert "does not validate controller" in compiled.manifest()["nonclaim"]
    ####


def test_compiler_preserves_a_larger_requested_turn_radius_and_reports_a_tighter_one() -> None:
    capability, baseline_intent = load_powered_fixed_wing_mission_profiles(PROFILES)["x8-cruise"]

    larger_radius = compile_powered_fixed_wing_racetrack(
        capability,
        replace(baseline_intent, requested_turn_radius_m=600.0),
    )
    tighter_radius = compile_powered_fixed_wing_racetrack(
        capability,
        replace(baseline_intent, requested_turn_radius_m=20.0),
    )

    assert larger_radius.route.turn_radius_m == pytest.approx(600.0)
    assert tighter_radius.route.turn_radius_m == pytest.approx(
        baseline_intent.turn_radius_margin
        * capability.nominal_speed_m_s**2
        / (9.80665 * math.tan(math.radians(10.0)))
    )
    assert any("requested turn radius" in item for item in tighter_radius.diagnostics)
    ####


def test_all_four_reference_airbreathers_compile_with_explicit_limits() -> None:
    profiles = load_powered_fixed_wing_mission_profiles(PROFILES)

    assert set(profiles) == {"x8-cruise", "b747-cruise", "a320-cruise", "f16-subsonic"}
    for profile_id, (capability, intent) in profiles.items():
        compiled = compile_powered_fixed_wing_racetrack(capability, intent, binding_id=f"{profile_id}-compiled")
        assert compiled.route.vehicle_id == capability.vehicle_id
        assert capability.minimum_speed_m_s <= compiled.route.speed_m_s <= capability.maximum_speed_m_s
        assert abs(compiled.route.left_turn_bank_deg) <= capability.maximum_bank_deg
        assert abs(compiled.route.right_turn_bank_deg) <= capability.maximum_bank_deg
        assert compiled.route.turn_radius_m >= compiled.minimum_turn_radius_m
        assert compiled.route.horizon_s > compiled.route.declared_duration_s
        assert compiled.route.timing.outbound_level_time_s >= intent.level_dwell_s
        assert compiled.route.timing.inbound_level_time_s >= intent.level_dwell_s
    ####


def test_baseline_comparison_exposes_route_assumptions_without_replacing_baseline() -> None:
    capability, intent = load_powered_fixed_wing_mission_profiles(PROFILES)["a320-cruise"]
    compiled = compile_powered_fixed_wing_racetrack(capability, intent)
    baseline = load_racetrack_template_catalog(ROOT / "verification/racetrack_templates.yaml").get("a320-openap-3dof")

    comparison = compare_compiled_racetrack_to_baseline(compiled, baseline)

    assert comparison["baseline_binding_id"] == "a320-openap-3dof"
    assert comparison["status"] == "baseline_review_required"
    assert any("turn radius" in item for item in comparison["findings"])
    assert baseline.turn_radius_m == pytest.approx(6000.0)
    ####


def test_fidelity_specific_intent_prevents_b747_reduced_lanes_inheriting_rigid_altitude() -> None:
    """A route compiler resolves a tier-specific operating corridor explicitly."""

    _, reduced_intent = resolve_powered_fixed_wing_mission_profile(
        PROFILES,
        "b747-cruise",
        "pseudo_6dof_kinematic_bridge",
    )
    _, rigid_intent = resolve_powered_fixed_wing_mission_profile(
        PROFILES,
        "b747-cruise",
        "rigid_body_6dof_direct_wrench",
    )

    assert (reduced_intent.low_altitude_m, reduced_intent.high_altitude_m) == (100.0, 250.0)
    assert (rigid_intent.low_altitude_m, rigid_intent.high_altitude_m) == (500.0, 650.0)
    ####
