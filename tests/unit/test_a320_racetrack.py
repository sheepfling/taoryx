"""Truth-gated A320 shared-racetrack integration tests."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from taoryx.mission_objectives import TruthObjectiveSpec, evaluate_truth_objectives
from taoryx.mission_promotion import assess_mission_promotion, mission_proposal_fingerprint
from taoryx.powered_fixed_wing_mission_compiler import (
    compile_powered_fixed_wing_racetrack,
    load_powered_fixed_wing_mission_profiles,
)
from taoryx.racetrack_template import load_racetrack_template_catalog
from taoryx.trajectory import (
    A320OpenAPModel,
    A320OpenAPOperatingPoint,
    A320Pseudo6DOFModel,
    A320RacetrackRunner,
)
from taoryx.trajectory.pseudo6dof_profiles import load_pseudo6dof_catalog
from tools.validate_a320_racetrack import run_case

ROOT = Path(__file__).resolve().parents[2]


def _specs(route) -> tuple[TruthObjectiveSpec, ...]:
    windows = {item.name: item for item in route.phase_windows}
    return tuple(
        TruthObjectiveSpec(
            id=gate.id,
            objective_type="fly_by_gate",
            target=gate.target(route.speed_m_s),
            tolerance={
                "corridor_m": route.gate_corridor_m,
                "altitude_m": route.gate_altitude_tolerance_m,
                "speed_m_s": route.gate_speed_tolerance_mps,
            },
            gate_normal=tuple(gate.gate_normal()),
            window_start_s=max(0.0, windows[gate.phase].start_s - 12.0),
            window_end_s=route.horizon_s if gate.id == "terminal-start-finish-gate" else windows[gate.phase].end_s + 12.0,
        )
        for gate in route.gates
    )
    ####


def test_a320_point_mass_racetrack_passes_all_truth_gates() -> None:
    catalog = load_racetrack_template_catalog(ROOT / "verification/racetrack_templates.yaml")
    route = catalog.get("a320-openap-3dof")
    model = A320OpenAPModel.from_repository(ROOT)
    trim = model.trim_level_flight(A320OpenAPOperatingPoint(10500.0, 0.78, 60000.0))
    run = A320RacetrackRunner(model, trim, route, "point_mass_3dof", dt_s=0.2).run()
    report = evaluate_truth_objectives(_specs(route), run.rows, hard_gates_passed=run.numerical_valid)

    assert run.numerical_valid is True
    assert report["mission_pass"] is True
    assert report["required_passed"] == report["required_objectives"] == 4
    ####


def test_a320_pseudo6dof_racetrack_passes_without_physical_actuator_claim() -> None:
    catalog = load_racetrack_template_catalog(ROOT / "verification/racetrack_templates.yaml")
    route = catalog.get("a320-openap-pseudo6dof")
    model = A320Pseudo6DOFModel.from_repository(ROOT)
    trim = model.trim_pseudo6dof(A320OpenAPOperatingPoint(10500.0, 0.78, 60000.0))
    run = A320RacetrackRunner(model, trim, route, "pseudo_6dof_kinematic_bridge", dt_s=0.2).run()
    report = evaluate_truth_objectives(_specs(route), run.rows, hard_gates_passed=run.numerical_valid)

    assert run.numerical_valid is True
    assert report["mission_pass"] is True
    assert all(row["allocation_status"] == "surrogate_policy_overlay" for row in run.rows)
    assert all(row["control_effectivity_source"] == "jsbsim-1.3.1-a320-coefficient-surrogate" for row in run.rows)
    ####


def test_a320_pseudo6dof_can_use_shared_catalog_response_law() -> None:
    catalog = load_racetrack_template_catalog(ROOT / "verification/racetrack_templates.yaml")
    route = catalog.get("a320-openap-pseudo6dof")
    model = A320Pseudo6DOFModel.from_repository(ROOT)
    trim = model.trim_pseudo6dof(A320OpenAPOperatingPoint(10500.0, 0.78, 60000.0))
    _, profile = load_pseudo6dof_catalog(ROOT / "verification/pseudo6dof_profiles.yaml").for_family("a320_openap_3dof")
    run = A320RacetrackRunner(
        model,
        trim,
        route,
        "pseudo_6dof_kinematic_bridge",
        dt_s=0.2,
        response_profile=profile,
    ).run()

    assert run.numerical_valid is True
    assert {row["response_profile_id"] for row in run.rows} == {profile.id}
    ####


def test_a320_truth_gate_negative_control_cannot_be_rescued_by_nominal_telemetry() -> None:
    catalog = load_racetrack_template_catalog(ROOT / "verification/racetrack_templates.yaml")
    route = catalog.get("a320-openap-3dof")
    model = A320OpenAPModel.from_repository(ROOT)
    trim = model.trim_level_flight(A320OpenAPOperatingPoint(10500.0, 0.78, 60000.0))
    run = A320RacetrackRunner(model, trim, route, "point_mass_3dof", dt_s=0.2).run()
    specs = list(_specs(route))
    specs[0] = replace(specs[0], target={**specs[0].target, "east_m": 35000.0})
    report = evaluate_truth_objectives(tuple(specs), run.rows, hard_gates_passed=run.numerical_valid)

    assert report["mission_pass"] is False
    assert report["results"][0]["status"] == "fail"
    ####


def test_a320_candidate_route_is_truth_evaluated_against_its_fingerprint() -> None:
    """A capability-scaled route must execute before it is promotion-eligible."""

    profiles = load_powered_fixed_wing_mission_profiles(
        ROOT / "verification/powered_fixed_wing_mission_profiles.yaml"
    )
    proposal = compile_powered_fixed_wing_racetrack(
        *profiles["a320-cruise"],
        binding_id="a320-cruise-pseudo_6dof_kinematic_bridge-candidate",
        fidelity="pseudo_6dof_kinematic_bridge",
    )
    fingerprint = mission_proposal_fingerprint(proposal)
    packet, _ = run_case(
        "pseudo_6dof_kinematic_bridge",
        1.0,
        route_override=proposal.route,
        mission_proposal_fingerprint=fingerprint,
    )

    assert packet["evaluation"]["mission_pass"] is True
    assert packet["mission_proposal_fingerprint"] == fingerprint
    assert assess_mission_promotion(proposal, packet).status == "promotion_eligible"
    ####
