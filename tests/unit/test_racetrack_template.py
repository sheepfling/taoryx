from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from taoryx.racetrack_template import RACETRACK_FIDELITIES, RacetrackBinding, load_racetrack_template_catalog

ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / "verification/racetrack_templates.yaml"


def test_racetrack_catalog_resolves_x8_reference_and_phase_contract() -> None:
    catalog = load_racetrack_template_catalog(CATALOG)
    resolved = catalog.get("x8-reference")

    assert catalog.template_id == "powered_fixed_wing_racetrack_v1"
    assert tuple(window.name for window in resolved.phase_windows) == (
        "outbound-climb",
        "outbound-level",
        "left-turn",
        "inbound-descent",
        "inbound-level",
        "right-turn",
    )
    assert resolved.route_attributes()["duration-s"] == format(resolved.declared_duration_s, ".16g")
    assert resolved.gates[-1].north_m == pytest.approx(0.0)
    assert resolved.gates[-1].east_m == pytest.approx(0.0)
    assert resolved.gates[-1].gate_normal() == [0.0, 1.0, 0.0]
    ####


def test_racetrack_catalog_names_four_claim_tiers() -> None:
    assert RACETRACK_FIDELITIES == (
        "point_mass_3dof",
        "pseudo_6dof_kinematic_bridge",
        "rigid_body_6dof_direct_wrench",
        "rigid_body_6dof_surface_allocated",
    )
    catalog = load_racetrack_template_catalog(CATALOG)
    assert catalog.get("x8-point-mass").fidelity == "point_mass_3dof"
    assert catalog.get("x8-kinematic-bridge").fidelity == "pseudo_6dof_kinematic_bridge"
    assert catalog.get("x8-direct-wrench").fidelity == "rigid_body_6dof_direct_wrench"
    assert catalog.get("x8-reference").fidelity == "rigid_body_6dof_surface_allocated"
    ####


def test_transport_binding_is_scaled_without_changing_semantic_phase_order() -> None:
    catalog = load_racetrack_template_catalog(CATALOG)
    resolved = catalog.get("b747-transport-scaled")

    assert resolved.vehicle_id == "b747"
    assert resolved.fidelity == "point_mass_3dof"
    assert resolved.source_realization == "source_table_point_mass_route"
    assert resolved.status == "binding_defined_nominal_run_pending"
    assert resolved.straight_length_m > 20.0 * catalog.get("x8-reference").straight_length_m
    assert resolved.turn_radius_m > 20.0 * catalog.get("x8-reference").turn_radius_m
    assert resolved.timing.outbound_level_time_s > 0.0
    assert resolved.timing.inbound_level_time_s > 0.0
    assert resolved.horizon_s > resolved.declared_duration_s
    assert [gate.id for gate in resolved.gates] == [
        "high-altitude-level-gate",
        "left-turn-exit-gate",
        "low-altitude-level-gate",
        "terminal-start-finish-gate",
    ]
    ####


def test_racetrack_template_rejects_vertical_phase_that_does_not_fit() -> None:
    catalog = load_racetrack_template_catalog(CATALOG)
    values = {
        "vehicle_id": "test",
        "fidelity": "point_mass_3dof",
        "straight_length_m": 100.0,
        "turn_radius_m": 20.0,
        "speed_m_s": 10.0,
        "low_altitude_m": 0.0,
        "high_altitude_m": 100.0,
        "climb_rate_m_s": 1.0,
        "descent_rate_m_s": 1.0,
        "left_turn_bank_deg": -10.0,
        "right_turn_bank_deg": 10.0,
    }
    from taoryx.racetrack_template import resolve_racetrack_binding

    with pytest.raises(ValueError, match="straight length"):
        resolve_racetrack_binding(catalog.template_id, "invalid", RacetrackBinding.model_validate(values))
    ####


def test_timing_phase_windows_close_without_using_observation_margin() -> None:
    resolved = load_racetrack_template_catalog(CATALOG).get("x8-reference")
    assert resolved.phase_windows[-1].end_s == pytest.approx(resolved.declared_duration_s)
    assert resolved.horizon_s == pytest.approx(resolved.declared_duration_s + 4.0)
    assert math.isfinite(resolved.timing_manifest()["simulation_horizon_s"])
    ####


def test_qualification_missions_reference_template_gates_instead_of_copying_geometry() -> None:
    mission_catalog = yaml.safe_load((ROOT / "verification/family_qualification_missions.yaml").read_text(encoding="utf-8"))
    missions = {item["id"]: item for item in mission_catalog["missions"]}

    for mission_id in ("x8-racetrack-altitude-turns-v1", "b747-racetrack-altitude-turns-3dof-v1"):
        mission = missions[mission_id]
        assert mission["racetrack_binding"]
        assert "racetrack_gate_id" in mission["terminal"]
        assert all("racetrack_gate_id" in objective or objective["objective_type"] == "event" for objective in mission["objectives"])
        assert all("target" not in objective for objective in mission["objectives"] if objective["objective_type"] == "fly_by_gate")
    ####
