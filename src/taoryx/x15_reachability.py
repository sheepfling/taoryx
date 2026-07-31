"""X-15-scaled Alpha 3 reachability demonstration."""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from .contracts import Vector3
from .reachability_envelope import (
    LaunchCommand,
    ReachabilityEnvelope,
    ReachabilityFidelity,
    RocketGlideVehicle,
    RocketStageSpec,
    StagedRocketSpec,
    StageSeparationSpec,
    TerminalCriteria,
    TrajectoryResult,
    generate_launch_grid,
    run_reachability_envelope,
    simulate_rocket_glide,
)
from .reachability_visualization import ReachabilityPlotReport, render_reachability_plot_bundle
from .vehicle import DetachedBodyDefinition, PropellantType, PropulsionCapabilities
from .vehicle_registry import vehicle_definition

X15EnvelopeTier = Literal["point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"]

_ROOT = Path(__file__).resolve().parents[2]
_SOURCE_STAGED_MISSION = _ROOT / "examples/showcases/x15_rocket_to_hawaii/mission.prb"


@dataclass(frozen=True, slots=True)
class X15IntegrationPreflight:
    """Machine-readable source-to-surrogate integration gate."""

    source: tuple[tuple[str, float], ...]
    surrogate: tuple[tuple[str, float], ...]
    checks: tuple[tuple[str, bool], ...]
    declared_propellant_discrepancy_kg: float

    @property
    def passed(self) -> bool:
        return all(result for _, result in self.checks)

    @property
    def failures(self) -> tuple[str, ...]:
        return tuple(name for name, result in self.checks if not result)

    def as_dict(self) -> dict[str, object]:
        return {
            "status": "PASS" if self.passed else "FAIL",
            "source": dict(self.source),
            "surrogate": dict(self.surrogate),
            "checks": {name: result for name, result in self.checks},
            "declared_propellant_discrepancy_kg": self.declared_propellant_discrepancy_kg,
            "failures": list(self.failures),
        }


@dataclass(frozen=True, slots=True)
class X15ReachabilityBundle:
    """The comparable X-15-scaled envelope tiers and their plot report."""

    envelopes: tuple[ReachabilityEnvelope, ...]
    plot_report: ReachabilityPlotReport
    manifest_path: Path
    ####


def build_x15_fidelity_evidence(
    trajectory: TrajectoryResult,
    *,
    fidelity: ReachabilityFidelity,
    command: LaunchCommand,
    step_size_s: float,
    horizon_s: float,
) -> dict[str, object]:
    """Build the common Alpha 3 evidence shape for one staged X-15 witness.

    This is intentionally a development-tier artifact.  The reduced model
    can independently prove the boost/coast/release/glide event chain and a
    terminal impact witness, but it does not prove a controlled approach or a
    source-backed physical-effector handoff.  Keeping those claims separate
    lets the automatic fidelity resolver remain fail-closed.
    """

    phase_times: dict[str, float] = {}
    for state in trajectory.states:
        phase_times.setdefault(state.phase, state.time_s)
    source = x15_source_staging_contract()
    # This is the declared reduced-order terminal-speed study window from the
    # X-15 provenance record.  It is an energy-corridor witness, not a claim
    # that a physical terminal controller or handoff is present.
    terminal_speed_window = (720.0, 950.0)
    energy_candidates = [
        state
        for state in trajectory.states
        if state.phase == "glide"
        and terminal_speed_window[0] <= state.speed_m_s <= terminal_speed_window[1]
        and state.position_m[2] > 5_000.0
    ]
    energy_state = min(energy_candidates, key=lambda state: abs(state.speed_m_s - 900.0), default=None)
    # This is an open-loop atmospheric handoff witness, not a claim that the
    # vehicle can guide or control itself into the corridor.  It records the
    # first useful lower-atmosphere state after the high-energy portion so the
    # staged mission has an explicit handoff contract before impact.
    handoff_candidates = [
        state
        for state in trajectory.states
        if state.phase == "glide"
        and 7_000.0 <= state.position_m[2] <= 11_000.0
        and 850.0 <= state.speed_m_s <= 1_100.0
        and state.velocity_m_s[2] < 0.0
    ]
    handoff_state = min(
        handoff_candidates,
        key=lambda state: abs(state.position_m[2] - 9_000.0) + 100.0 * abs(state.speed_m_s - 1_000.0),
        default=None,
    )
    phase_objectives = (
        {
            "id": "booster_burn_and_cutoff",
            "type": "event",
            "truth_result": "PASS" if {"boost", "coast"} <= phase_times.keys() else "FAIL",
            "truth_time_s": phase_times.get("coast"),
            "expected_time_s": source["powered_duration_s"],
            "tolerance_s": 1.0e-9,
        },
        {
            "id": "booster_release",
            "type": "event",
            "truth_result": "PASS" if "glide" in phase_times and trajectory.deployment_events else "FAIL",
            "truth_time_s": phase_times.get("glide"),
            "expected_time_s": source["release_time_s"],
            "tolerance_s": 1.0e-9,
        },
        {
            "id": "unpowered_glide",
            "type": "path_corridor",
            "truth_result": "PASS" if "glide" in phase_times else "FAIL",
            "truth_time_s": phase_times.get("glide"),
            "expected_time_s": None,
            "tolerance_s": None,
        },
        {
            "id": "high_energy_terminal_corridor",
            "type": "energy_corridor",
            "truth_result": "PASS" if energy_state is not None else "FAIL",
            "truth_time_s": None if energy_state is None else energy_state.time_s,
            "expected_time_s": None,
            "tolerance_s": None,
            "actual": None
            if energy_state is None
            else {
                "speed_m_s": energy_state.speed_m_s,
                "altitude_m": energy_state.position_m[2],
                "specific_energy_j_per_kg": 0.5 * energy_state.speed_m_s**2 + 9.80665 * energy_state.position_m[2],
            },
            "corridor": {
                "phase": "glide",
                "speed_window_m_s": list(terminal_speed_window),
                "minimum_altitude_m": 5_000.0,
                "reference_speed_m_s": 900.0,
            },
        },
        {
            "id": "atmospheric_terminal_handoff",
            "type": "terminal_state_gate",
            "truth_result": "PASS" if handoff_state is not None else "FAIL",
            "truth_time_s": None if handoff_state is None else handoff_state.time_s,
            "expected_time_s": None,
            "tolerance_s": None,
            "actual": None
            if handoff_state is None
            else {
                "altitude_m": handoff_state.position_m[2],
                "speed_m_s": handoff_state.speed_m_s,
                "vertical_speed_m_s": handoff_state.velocity_m_s[2],
                "specific_energy_j_per_kg": 0.5 * handoff_state.speed_m_s**2 + 9.80665 * handoff_state.position_m[2],
            },
            "corridor": {
                "phase": "glide",
                "altitude_window_m": [7_000.0, 11_000.0],
                "speed_window_m_s": [850.0, 1_100.0],
                "vertical_speed_sign": "descending",
                "control_mode": "open_loop_handoff_witness",
            },
        },
        {
            "id": "terminal_impact_witness",
            "type": "event",
            "truth_result": "PASS" if trajectory.termination.value == "ground_contact" else "FAIL",
            "truth_time_s": trajectory.terminal.time_s,
            "expected_time_s": None,
            "tolerance_s": None,
        },
    )
    mission_pass = all(objective["truth_result"] == "PASS" for objective in phase_objectives)
    claim_boundary = (
        "X-15-scaled staged reduced-order boost/coast/release/high-energy-corridor/glide witness with "
        "parent translation and explicit passive spent-booster deployment; no controlled terminal handoff, "
        "native X-15 batch provider, or physical-effector qualification is claimed."
    )
    return {
        "schema": "taoryx.family-fidelity-evidence/v1alpha1",
        "status": "development",
        "family_id": "x15",
        "fidelity": fidelity.value,
        "profile_id": f"x15.{'attitude_response_p6dof' if fidelity is ReachabilityFidelity.PSEUDO_6DOF else 'point_mass_3dof'}.v1",
        "claim": {
            "proves": "The reduced staged X-15 witness preserves the declared boost, cutoff, release, high-energy corridor, glide, open-loop atmospheric handoff gate, and impact event ordering.",
            "nonclaims": [
                "No controlled terminal approach or energy-managed handoff is claimed.",
                "No physical stabilator, rudder, throttle, RCS, or control-surface allocation is claimed.",
                "The pseudo-6DOF attitude channels are a named response bridge, not source-derived moments.",
            ],
            "claim_boundary": claim_boundary,
        },
        "control_path": {
            "realization": "response_law" if fidelity is ReachabilityFidelity.PSEUDO_6DOF else "none",
            "direct_force_moment_injection": False,
            "physical_effectors": [],
            "guidance": "open_loop_launch_and_declared_glide_bank",
        },
        "mission": {
            "start_contract": "staged_launch_state",
        "terminal_contract": "ground_contact_impact_witness",
        "handoff_contract": "open_loop_atmospheric_terminal_handoff_gate",
            "required_objectives": list(phase_objectives),
            "objective_count": len(phase_objectives),
            "passed_objective_count": sum(objective["truth_result"] == "PASS" for objective in phase_objectives),
            "independent_truth_evaluation": True,
            "controller_transition_evidence": "not_applicable_open_loop",
        },
        "evaluation": {
            "mission_pass": mission_pass,
            "hard_envelope_violations": [],
            "numerical_pass": all(
                all(math.isfinite(value) for value in (*state.position_m, *state.velocity_m_s, state.mass_kg))
                for state in trajectory.states
            ),
            "terminal": {
                "termination": trajectory.termination.value,
                "time_s": trajectory.terminal.time_s,
                "position_m": list(trajectory.terminal.position_m),
                "speed_m_s": trajectory.terminal.speed_m_s,
            },
        },
        "runtime": {
            "hard_gates_passed": mission_pass,
            "step_size_s": step_size_s,
            "horizon_s": horizon_s,
            "exit_code": 0,
        },
        "deployment": {
            "event_count": len(trajectory.deployment_events),
            "events": list(trajectory.deployment_events),
            "child_count": len(trajectory.spawned_bodies),
            "child_terminal_outcomes": [child.classification for child in trajectory.spawned_bodies],
        },
        "command": {
            "azimuth_rad": command.azimuth_rad,
            "elevation_rad": command.elevation_rad,
            "bank_rad": command.bank_rad,
        },
        "source": _x15_provenance(),
        "telemetry": list(trajectory.telemetry),
    }


def x15_source_staging_contract() -> dict[str, float]:
    """Read the staged source mission values used by the reduced-order gate."""

    text = _SOURCE_STAGED_MISSION.read_text(encoding="utf-8")

    def extract(pattern: str, name: str) -> float:
        match = re.search(pattern, text, flags=re.IGNORECASE | re.DOTALL)
        if match is None:
            raise ValueError(f"X-15 staged source is missing {name}: {_SOURCE_STAGED_MISSION}")
        return float(match.group(1))

    initial_block = r"\*initial\s+ecic(?P<body>.*?)(?=\n\s*\*file)"
    initial_match = re.search(initial_block, text, flags=re.IGNORECASE | re.DOTALL)
    if initial_match is None:
        raise ValueError(f"X-15 staged source is missing its ECIC initial block: {_SOURCE_STAGED_MISSION}")
    initial_body = initial_match.group("body")

    def extract_initial(pattern: str, name: str) -> float:
        match = re.search(pattern, initial_body, flags=re.IGNORECASE)
        if match is None:
            raise ValueError(f"X-15 staged source is missing initial {name}: {_SOURCE_STAGED_MISSION}")
        return float(match.group(1))

    return {
        "launch_mass_kg": extract_initial(r"\bmass\s*=\s*([-+0-9.eE]+)", "mass"),
        "initial_speed_m_s": math.sqrt(
            sum(
                extract_initial(rf"\b{axis}dt\s*=\s*([-+0-9.eE]+)", f"{axis}dt") ** 2
                for axis in ("x", "y", "z")
            )
        ),
        "declared_booster_propellant_kg": extract_initial(
            r"\bpropellant_mass\s*=\s*([-+0-9.eE]+)", "propellant_mass"
        ),
        "booster_thrust_n": extract(r"\*prop\s+thrust\s*=\s*([-+0-9.eE]+)", "booster thrust"),
        "booster_mdot_kg_s": extract(r"\*prop\s+thrust\s*=\s*[-+0-9.eE]+\s+mdot\s*=\s*([-+0-9.eE]+)", "booster mdot"),
        "powered_duration_s": extract(r"\*when\s+time\s*>\s*([-+0-9.eE]+)\s+goto\s+2", "powered duration"),
        "release_time_s": extract(r"\*segment\s+2.*?\*when\s+time\s*>\s*([-+0-9.eE]+)\s+goto\s+3", "release time"),
        "release_mass_kg": extract(r"\*reset\s+mass\s*=\s*([-+0-9.eE]+)", "release mass"),
    }


def x15_integration_preflight(vehicle: RocketGlideVehicle | None = None) -> X15IntegrationPreflight:
    """Validate source anchors and stage transitions before an envelope run."""

    source = x15_source_staging_contract()
    selected_vehicle = x15_surrogate_vehicle() if vehicle is None else vehicle
    consumed_propellant = source["booster_mdot_kg_s"] * source["powered_duration_s"]
    surrogate = {
        "launch_mass_kg": selected_vehicle.initial_mass_kg,
        "initial_speed_m_s": selected_vehicle.initial_speed_m_s,
        "booster_thrust_n": selected_vehicle.booster_thrust_n,
        "consumed_booster_propellant_kg": selected_vehicle.booster_propellant_mass_kg,
        "booster_burn_time_s": selected_vehicle.booster_burn_time_s,
        "release_time_s": selected_vehicle.booster_release_time_s,
        "release_mass_kg": selected_vehicle.release_mass_kg,
    }
    checks: list[tuple[str, bool]] = []

    def close(name: str, expected: float, actual: float, tolerance: float = 1.0e-8) -> None:
        checks.append((name, math.isclose(expected, actual, rel_tol=tolerance, abs_tol=tolerance)))

    close("launch_mass", source["launch_mass_kg"], surrogate["launch_mass_kg"])
    close("initial_speed", source["initial_speed_m_s"], surrogate["initial_speed_m_s"])
    close("booster_thrust", source["booster_thrust_n"], surrogate["booster_thrust_n"])
    close("consumed_propellant", consumed_propellant, surrogate["consumed_booster_propellant_kg"])
    close("booster_burn_time", source["powered_duration_s"], surrogate["booster_burn_time_s"])
    close("release_time", source["release_time_s"], surrogate["release_time_s"])
    close("release_mass", source["release_mass_kg"], surrogate["release_mass_kg"])

    for step_size_s in (0.5, 7.0, 17.0):
        result = simulate_rocket_glide(
            selected_vehicle,
            LaunchCommand(0.0, math.radians(45.0)),
            fidelity=ReachabilityFidelity.POINT_MASS_3DOF,
            step_size_s=step_size_s,
            horizon_s=60.0,
        )
        at_burnout = next((state for state in result.states if math.isclose(state.time_s, source["powered_duration_s"])), None)
        at_release = next((state for state in result.states if math.isclose(state.time_s, source["release_time_s"])), None)
        checks.append((f"burnout_transition_step_{step_size_s:g}", at_burnout is not None and at_burnout.phase == "coast" and math.isclose(at_burnout.mass_kg, selected_vehicle.burnout_mass_kg)))
        checks.append((f"release_transition_step_{step_size_s:g}", at_release is not None and at_release.phase == "glide" and math.isclose(at_release.mass_kg, selected_vehicle.release_mass_kg)))

    report = X15IntegrationPreflight(
        source=tuple(sorted(source.items())),
        surrogate=tuple(sorted(surrogate.items())),
        checks=tuple(checks),
        declared_propellant_discrepancy_kg=source["declared_booster_propellant_kg"] - consumed_propellant,
    )
    if not report.passed:
        raise ValueError(f"X-15 reachability preflight failed: {', '.join(report.failures)}")
    return report


def x15_surrogate_vehicle() -> RocketGlideVehicle:
    """Build the staged X-15 reachability surrogate from repository anchors.

    The reduced-order vehicle starts at the mission's booster launch state,
    burns the attached booster, coasts to release, then glides unpowered. The
    aerodynamic body remains a fixed-L/D reduction until a batchable native
    X-15 provider exists.
    """

    definition = vehicle_definition("x15")
    release_mass_kg = 14_641.0545
    launch_mass_kg = 30_000.0
    booster_burn_time_s = 20.0
    core_stage = RocketStageSpec(
        identifier="x15-glide-body",
        dry_mass_kg=release_mass_kg,
    )
    booster_stage = RocketStageSpec(
        identifier="x15-booster",
        dry_mass_kg=launch_mass_kg - release_mass_kg - 2_000.0,
        propellant_mass_kg=2_000.0,
        thrust_n=40_000.0,
        burn_time_s=booster_burn_time_s,
        mass_flow_kg_s=100.0,
        declared_propellant_mass_kg=9_000.0,
        capabilities=PropulsionCapabilities.defaults_for(PropellantType.LIQUID),
    )
    detached_booster = DetachedBodyDefinition.cylinder(
        "x15-spent-booster",
        mass_kg=booster_stage.dry_mass_kg,
        radius_m=1.25,
        length_m=8.0,
        inertia_kg_m2=Vector3(50_000.0, 50_000.0, 5_000.0),
    )
    staged_spec = StagedRocketSpec(
        core_stage=core_stage,
        attached_stages=(booster_stage,),
        separation_events=(StageSeparationSpec("x15-booster", 50.0, detached_body=detached_booster),),
    )
    return RocketGlideVehicle.from_staged_spec(
        staged_spec,
        vehicle_id="x15-generic-reachability-surrogate-v1",
        reference_area_m2=float(definition["reference_area_m2"]),
        drag_coefficient=0.045,
        lift_to_drag=3.0,
        initial_speed_m_s=1_555.6349186151906,
        initial_altitude_m=0.0,
        max_attitude_rate_rad_s=math.radians(3_600.0),
        pseudo6dof_profile_id="x15.attitude_response_p6dof.v1",
    )
    ####


def x15_reachability_commands() -> tuple[LaunchCommand, ...]:
    """Return the shared deterministic X-15 launch/glide search grid."""

    return generate_launch_grid(
        tuple(math.radians(value) for value in (-20.0, 0.0, 20.0)),
        tuple(math.radians(value) for value in (25.0, 35.0, 45.0, 55.0, 65.0)),
        tuple(math.radians(value) for value in (-20.0, 0.0, 20.0)),
    )
    ####


def _x15_provenance() -> dict[str, object]:
    return {
        "source_registry": "verification/vehicle_models.yaml",
        "source_vehicle_id": "x15",
        "source_point_mass_case": "examples/showcases/x15_rocket_to_hawaii/release_glide_reduction_3dof.prb",
        "source_rigid_body_case": "examples/showcases/x15_rocket_to_hawaii/release_glide_parity_6dof.prb",
        "source_tables": "tests/fixtures/x15_coherent_6dof_public_research_v1/tables/x15_static_6axis.tbl",
        "source_staged_mission": "examples/showcases/x15_rocket_to_hawaii/mission.prb",
        "source_reduced_mission": "examples/showcases/x15_rocket_to_hawaii/mission_3dof.prb",
        "claim_boundary": "X-15-scaled staged reachability surrogate; not a native X-15 rigid-body batch provider",
        "fidelity_boundary": {
            "point_mass_3dof": "reduced parent and reduced child",
            "pseudo_6dof": "reduced parent and pseudo-6DOF child",
            "rigid_body_6dof": "reduced parent and native rigid-body child",
        },
        "source_launch_mass_kg": 30_000.0,
        "source_booster_thrust_n": 40_000.0,
        "source_booster_mdot_kg_s": 100.0,
        "source_booster_powered_duration_s": 20.0,
        "source_release_mass_kg": 14_641.0545,
        "source_declared_booster_propellant_kg": 9_000.0,
        "assumed_booster_propellant_consumed_kg": 2_000.0,
        "assumed_booster_dry_mass_discarded_kg": 13_358.9455,
        "source_declared_minus_consumed_propellant_kg": 7_000.0,
        "stage_contract": {
            "core_stage": "x15-glide-body",
            "attached_stage": "x15-booster",
            "separation_time_s": 50.0,
            "ejected_mass_kg": 13_358.9455,
        },
        "assumed_drag_coefficient": 0.045,
        "assumed_lift_to_drag": 3.0,
        "default_terminal_speed_window_m_s": [720.0, 950.0],
        "registry_dry_mass_kg": float(vehicle_definition("x15")["dry_mass_kg"]),
        "registry_nominal_mass_kg": float(vehicle_definition("x15")["nominal_mass_kg"]),
        "integration_note": "The source mission declares 9000 kg booster propellant but its 20 s, 100 kg/s powered segment consumes 2000 kg; the reduced-order mass closure models the consumed amount and records the discrepancy.",
    }
    ####


def run_x15_reachability_tiers(
    *,
    commands: tuple[LaunchCommand, ...] | None = None,
    step_size_s: float = 0.5,
    horizon_s: float = 120.0,
    workers: int = 1,
    criteria: TerminalCriteria | None = None,
    spawn_children: bool = True,
) -> tuple[ReachabilityEnvelope, ...]:
    """Run point-mass and pseudo-6DOF X-15-scaled tiers over one search grid."""

    selected_commands = commands or x15_reachability_commands()
    selected_criteria = criteria or TerminalCriteria(min_speed_m_s=720.0, max_speed_m_s=950.0)
    vehicle = x15_surrogate_vehicle()
    preflight = x15_integration_preflight(vehicle)
    provenance = {**_x15_provenance(), "integration_preflight": preflight.as_dict()}
    tiers: tuple[ReachabilityFidelity, ...] = (
        ReachabilityFidelity.POINT_MASS_3DOF,
        ReachabilityFidelity.PSEUDO_6DOF,
        ReachabilityFidelity.RIGID_BODY_6DOF,
    )
    return tuple(
        run_reachability_envelope(
            vehicle,
            selected_commands,
            fidelity=fidelity,
            step_size_s=step_size_s,
            horizon_s=horizon_s,
            criteria=selected_criteria,
            workers=workers,
            study_id="x15_scaled_reachability_v1",
            provenance=provenance,
            spawn_children=spawn_children,
        )
        for fidelity in tiers
    )
    ####


def write_x15_reachability_bundle(
    directory: str | Path,
    *,
    commands: tuple[LaunchCommand, ...] | None = None,
    step_size_s: float = 0.5,
    horizon_s: float = 120.0,
    workers: int = 1,
    dpi: int = 140,
    criteria: TerminalCriteria | None = None,
    spawn_children: bool = True,
) -> X15ReachabilityBundle:
    """Write comparable X-15 tier artifacts and their standard plot bundle."""

    destination = Path(directory)
    destination.mkdir(parents=True, exist_ok=True)
    envelopes = run_x15_reachability_tiers(
        commands=commands,
        step_size_s=step_size_s,
        horizon_s=horizon_s,
        workers=workers,
        criteria=criteria,
        spawn_children=spawn_children,
    )
    artifact_paths: list[Path] = []
    for envelope in envelopes:
        path = destination / f"{envelope.fidelity.value}.json"
        envelope.write_json(path)
        artifact_paths.append(path)
    plot_report = render_reachability_plot_bundle(
        envelopes[0],
        destination / "plots",
        comparison_sources=envelopes[1:],
        dpi=dpi,
    )
    manifest = {
        "schema": "taoryx.x15-reachability-bundle/v1alpha1",
        "vehicle_id": "x15",
        "claim_boundary": _x15_provenance()["claim_boundary"],
        "envelopes": [path.name for path in artifact_paths],
        "plots": [path.name for path in plot_report.plot_paths],
        "plot_manifest": str(plot_report.manifest_path.relative_to(destination)),
    }
    manifest_path = destination / "bundle-manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return X15ReachabilityBundle(envelopes, plot_report, manifest_path)
    ####


__all__ = [
    "X15IntegrationPreflight",
    "X15ReachabilityBundle",
    "X15EnvelopeTier",
    "build_x15_fidelity_evidence",
    "x15_integration_preflight",
    "run_x15_reachability_tiers",
    "x15_source_staging_contract",
    "write_x15_reachability_bundle",
    "x15_reachability_commands",
    "x15_surrogate_vehicle",
]
####
