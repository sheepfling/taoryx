"""Contract tests for the airbreathing racetrack fidelity ladder."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]


def _ladder_module():
    path = ROOT / "tools/build_airbreathing_racetrack_ladder.py"
    spec = importlib.util.spec_from_file_location("airbreathing_racetrack_ladder", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_ladder_exposes_each_airbreathing_vehicle_at_each_fidelity() -> None:
    module = _ladder_module()
    selected = module._selected_missions("all", "all")
    assert len(selected) == 12
    assert {(vehicle, fidelity) for vehicle, fidelity, _ in selected} == {
        (vehicle, fidelity)
        for vehicle in ("x8", "b747", "f16")
        for fidelity in module.FIDELITIES
    }
    ####


def test_every_ladder_mission_resolves_a_declared_template_binding() -> None:
    missions = yaml.safe_load((ROOT / "verification/family_qualification_missions.yaml").read_text(encoding="utf-8"))["missions"]
    mission_by_id = {str(item["id"]): item for item in missions}
    module = _ladder_module()
    for mission_id in (
        mission_id
        for bindings in module.MISSION_IDS.values()
        for mission_id in bindings.values()
        if mission_id is not None
    ):
        mission = mission_by_id[mission_id]
        assert mission["racetrack_binding"]
        assert Path(ROOT / mission["problem"]).exists()
        assert "racetrack_gate_id" in mission["terminal"]
        for objective in mission["objectives"]:
            if objective["objective_type"] == "event":
                continue
            if objective["objective_type"] == "dwell":
                assert objective["racetrack_phase"]
                assert objective["target"]
                assert objective["tolerance"]
                continue
            assert "racetrack_gate_id" in objective
    ####


def test_qualification_missions_declare_canonical_fidelity_explicitly() -> None:
    missions = yaml.safe_load((ROOT / "verification/family_qualification_missions.yaml").read_text(encoding="utf-8"))["missions"]
    canonical = {
        "point_mass_3dof",
        "pseudo_6dof",
        "rigid_body_6dof_direct_wrench",
        "rigid_body_6dof_surface_allocated",
    }

    assert missions
    assert {str(item["fidelity_tier"]) for item in missions} <= canonical
    assert all("fidelity_tier" in item for item in missions)
    ####
