"""Tests for the F-16 reduced-fidelity shared racetrack realizations."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from taoryx.racetrack_template import load_racetrack_template_catalog
from taoryx.trajectory import (
    F16AttitudeResponsePseudo6DOFModel,
    F16GuidanceOverride,
    F16PointMass3DOFModel,
    F16ReducedRacetrackRunner,
    F16ReducedRacetrackStepper,
)
from taoryx.trajectory.pseudo6dof_profiles import load_pseudo6dof_catalog
from tools.validate_f16_reductions import _build_case

ROOT = Path(__file__).resolve().parents[2]


def test_f16_point_mass_racetrack_is_truth_evaluable_and_repeatable() -> None:
    """The point-mass route uses bounded kinematics, not physical moments."""

    source, trim, trim_pitch_rad = _build_case()
    route = load_racetrack_template_catalog(ROOT / "verification/racetrack_templates.yaml").get("f16-s119-point-mass")
    runner = F16ReducedRacetrackRunner(
        F16PointMass3DOFModel(source, trim, trim_pitch_rad),
        trim,
        route,
        "point_mass_3dof",
        dt_s=2.0,
    )
    first = runner.run()
    second = runner.run()

    assert first.numerical_valid is True
    assert first.failure is None
    assert first.rows == second.rows
    assert first.rows
    assert {row["control_path"] for row in first.rows} == {"point_mass_3dof"}
    assert {row["allocation_status"] for row in first.rows} == {"reduced_response_law"}
    assert all(np.isfinite(float(row["north_m"])) for row in first.rows)
    ####


def test_f16_pseudo6dof_racetrack_exposes_named_response_channels() -> None:
    """The pseudo-6DOF route adds attitude/rate response without effector claims."""

    source, trim, trim_pitch_rad = _build_case()
    linearization = source.linearize_local(
        trim.state,
        trim.controls,
        trim_pitch_rad=trim_pitch_rad,
        altitude_m=0.0,
        state_step=1.0e-5,
        control_step=1.0e-5,
    )
    route = load_racetrack_template_catalog(ROOT / "verification/racetrack_templates.yaml").get("f16-s119-pseudo-6dof")
    run = F16ReducedRacetrackRunner(
        F16AttitudeResponsePseudo6DOFModel(source, trim, linearization, trim_pitch_rad),
        trim,
        route,
        "pseudo_6dof_kinematic_bridge",
        dt_s=2.0,
    ).run(duration_s=route.declared_duration_s)

    assert run.numerical_valid is True
    assert run.rows
    assert {row["control_path"] for row in run.rows} == {"pseudo_6dof_kinematic_bridge"}
    assert all(np.isfinite(float(row["p_rad_s"])) for row in run.rows)
    assert all(np.isfinite(float(row["route_pitch_achieved_deg"])) for row in run.rows)
    assert all(row["allocation_status"] == "reduced_response_law" for row in run.rows)
    ####


def test_f16_pseudo6dof_can_use_shared_catalog_response_law() -> None:
    """The F-16 runner can execute through the common Alpha 3 profile."""

    source, trim, trim_pitch_rad = _build_case()
    linearization = source.linearize_local(
        trim.state,
        trim.controls,
        trim_pitch_rad=trim_pitch_rad,
        altitude_m=0.0,
        state_step=1.0e-5,
        control_step=1.0e-5,
    )
    route = load_racetrack_template_catalog(ROOT / "verification/racetrack_templates.yaml").get("f16-s119-pseudo-6dof")
    _, profile = load_pseudo6dof_catalog(ROOT / "verification/pseudo6dof_profiles.yaml").for_family("f16_s119")
    run = F16ReducedRacetrackRunner(
        F16AttitudeResponsePseudo6DOFModel(source, trim, linearization, trim_pitch_rad),
        trim,
        route,
        "pseudo_6dof_kinematic_bridge",
        dt_s=2.0,
        response_profile=profile,
    ).run(duration_s=route.declared_duration_s)

    assert run.numerical_valid is True
    assert {row["response_profile_id"] for row in run.rows} == {profile.id}
    ####


def test_f16_stepper_owns_committed_state_and_matches_point_mass_batch_boundaries() -> None:
    source, trim, trim_pitch_rad = _build_case()
    route = load_racetrack_template_catalog(ROOT / "verification/racetrack_templates.yaml").get("f16-s119-point-mass")
    runner = F16ReducedRacetrackRunner(
        F16PointMass3DOFModel(source, trim, trim_pitch_rad),
        trim,
        route,
        "point_mass_3dof",
        dt_s=2.0,
    )
    batch_rows = runner.run(duration_s=4.0).rows
    stepper = F16ReducedRacetrackStepper(runner)
    stepper_rows = stepper.step(4.0)

    assert len(batch_rows) >= len(stepper_rows)
    assert math.isclose(stepper.state.time_s, 4.0, abs_tol=1.0e-12)
    assert stepper.state.numerical_valid is True
    for batch, stepped in zip(batch_rows, stepper_rows):
        for channel in ("time_s", "north_m", "east_m", "altitude_m", "speed_m_s"):
            assert math.isclose(float(batch[channel]), float(stepped[channel]), abs_tol=1.0e-12)
    ####


def test_f16_pseudo_stepper_applies_held_guidance_without_surface_claim() -> None:
    source, trim, trim_pitch_rad = _build_case()
    linearization = source.linearize_local(
        trim.state,
        trim.controls,
        trim_pitch_rad=trim_pitch_rad,
        altitude_m=0.0,
        state_step=1.0e-5,
        control_step=1.0e-5,
    )
    route = load_racetrack_template_catalog(ROOT / "verification/racetrack_templates.yaml").get("f16-s119-pseudo-6dof")
    stepper = F16ReducedRacetrackStepper(
        F16ReducedRacetrackRunner(
            F16AttitudeResponsePseudo6DOFModel(source, trim, linearization, trim_pitch_rad),
            trim,
            route,
            "pseudo_6dof_kinematic_bridge",
            dt_s=2.0,
        )
    )

    rows = stepper.step(4.0, F16GuidanceOverride(heading_rad=0.0, bank_angle_rad=0.15))
    committed = stepper.current_row(F16GuidanceOverride(heading_rad=0.0, bank_angle_rad=0.15))

    assert len(rows) == 2
    assert float(committed["route_heading_command_deg"]) == 0.0
    assert float(committed["route_bank_achieved_deg"]) != 0.0
    assert committed["allocation_status"] == "reduced_response_law"
    assert stepper.state.numerical_valid is True
    ####
