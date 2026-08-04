"""Regression checks for the X-15 local direct-wrench screen."""

from __future__ import annotations

import json
from pathlib import Path
from typing import cast

import numpy as np

from taoryx.local_direct_wrench import run_local_direct_wrench_screen
from taoryx.x15_adapter import (
    build_x15_local_direct_wrench_screen_config,
    build_x15_source_direct_wrench_adapter,
    build_x15_source_direct_wrench_plant,
)

ROOT = Path(__file__).resolve().parents[2]


def test_x15_direct_wrench_screen_is_auditable_but_not_physical_effector_control() -> None:
    payload = json.loads((ROOT / "verification/alpha3_x15_direct_wrench/manifest.json").read_text(encoding="utf-8"))
    assert payload["status"] == "nominal_case_pass"
    assert payload["evaluation"]["mission_pass"] is True
    assert payload["claim"]["evidence_tier"] == "T3_direct_wrench_bridge"
    assert payload["claim"]["direct_body_moment_injection"] is True
    assert payload["claim"]["physical_effector_allocation"] is False
    assert payload["source_trim_boundary"]["direct_bias_is_not_source_trim"] is True
    assert payload["evaluation"]["wrench_statuses_observed"] == ["feasible"]
    ####


def test_x15_direct_bridge_trims_and_linearizes_the_same_local_nonlinear_plant() -> None:
    plant = build_x15_source_direct_wrench_plant()
    trim = plant.trim(plant.reference_state, {name: 0.0 for name in plant.control_names})

    assert trim.success is True
    assert trim.spec.operating_point["control_realization"] == "direct_wrench"
    assert trim.spec.operating_point["source_trim_claim"] == "not_source_physical_effector_trim"
    derivative = plant.state_derivative(trim.state, trim.controls, {})
    assert max(abs(float(value)) for value in derivative.values()) < 1.0e-6

    linearization = plant.linearize(trim, {"state_step": 1.0e-5, "control_step": 1.0e-5})
    assert linearization.primary.a_matrix.shape == (6, 6)
    assert linearization.primary.b_matrix.shape == (6, 6)
    assert np.isfinite(linearization.primary.a_matrix).all()
    assert np.isfinite(linearization.primary.b_matrix).all()
    assert linearization.provenance.derivative_consistent is True


def test_x15_direct_bridge_facade_exposes_trim_and_linearize_but_not_surface_allocation() -> None:
    adapter = build_x15_source_direct_wrench_adapter("rigid_body_6dof_direct_wrench")
    capabilities = {item.operation: item.status for item in adapter.capability_report().capabilities}

    assert capabilities["state_derivative"] == "available"
    assert capabilities["trim"] == "available"
    assert capabilities["linearize"] == "available"
    assert capabilities["effectiveness"] == "not_applicable"
    assert capabilities["allocate"] == "not_applicable"
    ####


def test_x15_local_screen_uses_the_shared_direct_wrench_execution_seam() -> None:
    """The source-owned X-15 screen must not require a private tool loop."""

    screen = run_local_direct_wrench_screen(build_x15_local_direct_wrench_screen_config())

    assert screen.mission_pass is True
    assert screen.observed_statuses == ("feasible",)
    assert screen.equilibrium_pass is True
    assert screen.equilibrium_projection.status == "feasible"
    assert screen.equilibrium_derivative_norm <= screen.config.equilibrium_derivative_norm_limit
    assert screen.lqr.hurwitz is True
    assert screen.final_error_norm < screen.initial_error_norm * 0.25
    payload = screen.as_dict()
    assert payload["physical_effector_allocation"] is False
    assert payload["evaluation"]["equilibrium"]["passed"] is True
    ####


def test_x15_local_screen_configuration_is_reused_within_one_source_revision() -> None:
    """Product 3 preflight and execution must reuse one immutable source setup."""

    first = build_x15_local_direct_wrench_screen_config()
    second = build_x15_local_direct_wrench_screen_config()

    assert first is second
    assert first.id == "x15-source-release-glide-local-direct-wrench-v1"
    ####


def test_x15_direct_wrench_plant_reuses_source_setup_but_not_caller_mappings() -> None:
    """Cached source loading must not make a caller's control map globally mutable."""

    first = build_x15_source_direct_wrench_plant()
    second = build_x15_source_direct_wrench_plant()
    controls = cast(dict[str, float], first.source_controls)
    controls["rudder_deg"] = 17.0

    assert first is not second
    assert second.source_controls["rudder_deg"] == 0.0
    assert first.reference_state == second.reference_state
    ####
