"""Focused tests for the F-16 source-aware racetrack execution seam."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from taoryx.racetrack_template import load_racetrack_template_catalog
from taoryx.trajectory import F16RacetrackRunner
from tools.validate_f16_physical_wrench_perturbations import _build_case
from tools.validate_f16_racetrack import _envelope_violations

ROOT = Path(__file__).resolve().parents[2]


def test_f16_racetrack_direct_wrench_is_a_finite_comparison_run() -> None:
    """Direct-wrench execution remains reproducible but is explicitly labelled."""

    adapter, trim, design = _build_case()
    route = load_racetrack_template_catalog(ROOT / "verification/racetrack_templates.yaml").get("f16-s119-direct-wrench")
    run = F16RacetrackRunner(adapter.source, trim, design, route, "direct_wrench", dt_s=0.2).run(duration_s=1.0)

    assert run.numerical_valid is True
    assert run.failure is None
    assert run.rows
    assert all(row["allocation_status"] == "direct_wrench" for row in run.rows)
    assert all(np.isfinite(float(row["requested_moment_x_nm"])) for row in run.rows)
    ####


def test_f16_racetrack_surface_run_logs_physical_effectors_and_residuals() -> None:
    """Surface execution reaches the adapter and exposes physical evidence."""

    adapter, trim, design = _build_case()
    route = load_racetrack_template_catalog(ROOT / "verification/racetrack_templates.yaml").get("f16-s119-surfaces")
    run = F16RacetrackRunner(adapter.source, trim, design, route, "surface_allocated", adapter, dt_s=0.2).run(duration_s=1.0)

    assert run.numerical_valid is True
    assert run.rows
    assert {row["allocation_status"] for row in run.rows} <= {"feasible", "feasible_near_limit", "partially_achievable"}
    assert all(np.isfinite(float(row["allocation_residual_norm"])) for row in run.rows)
    assert all(np.isfinite(float(row["elevator_deg"])) for row in run.rows)
    assert all(np.isfinite(float(row["throttle_fraction"])) for row in run.rows)
    assert all(np.isfinite(float(row["commanded_elevator_deg"])) for row in run.rows)
    assert all(np.isfinite(float(row["elevator_position_error_deg"])) for row in run.rows)
    assert all("source_total_moment_y_nm" in row for row in run.rows)
    ####


def test_f16_effectiveness_is_defined_at_throttle_lower_limit() -> None:
    """Bounded finite differences must not probe an invalid negative throttle."""

    adapter, trim, _ = _build_case()
    controls = dict(trim.controls)
    controls["throttle_fraction"] = 0.0
    effectiveness = adapter.effectiveness(trim.state, controls)

    assert np.isfinite(effectiveness.array).all()
    assert effectiveness.array.shape == (4, 4)
    ####


def test_f16_racetrack_envelope_check_rejects_out_of_domain_truth() -> None:
    """The packet cannot silently treat source-domain excursions as valid."""

    rows = [{"time_s": 1.0, "altitude_m": 0.0, "mach": 1.1, "aero_alpha_deg": 0.0, "aero_sideslip_deg": 0.0}]
    violations = _envelope_violations(rows)
    assert {item["channel"] for item in violations} == {"mach"}
    ####


def test_f16_racetrack_envelope_check_ignores_roundoff_at_ground() -> None:
    """A tiny negative ground altitude from floating point is not an excursion."""

    rows = [{"time_s": 1.0, "altitude_m": -1.0e-12, "mach": 0.8, "aero_alpha_deg": 0.0, "aero_sideslip_deg": 0.0}]
    assert _envelope_violations(rows) == []
    ####
