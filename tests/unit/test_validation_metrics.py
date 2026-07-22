from __future__ import annotations

import math

import pytest

from taoryx.validation import (
    actuator_saturation_fraction,
    capture_time,
    dwell_in_band,
    energy_balance_residual,
    independent_force_closure,
    independent_moment_closure,
    integral_mass_balance_error,
    phase_slice,
    settling_time,
    specific_energy,
    timestep_convergence_error,
    wrapped_angle_error,
)


def test_phase_and_angle_metrics_are_reusable() -> None:
    history = tuple({"time_s": float(index), "heading_deg": value} for index, value in enumerate((179.0, 180.0, -179.0, -178.0)))

    assert [sample["time_s"] for sample in phase_slice(history, 1.0, 2.0)] == [1.0, 2.0]
    assert wrapped_angle_error(-179.0, 179.0) == pytest.approx(2.0)


def test_capture_settling_and_dwell_require_a_persistent_band() -> None:
    history = tuple(
        {"time_s": float(index), "error": value}
        for index, value in enumerate((4.0, 1.5, 0.4, 0.2, 0.1))
    )

    assert capture_time(history, "error", 0.0, tolerance=0.5) == pytest.approx(2.0)
    assert settling_time(history, "error", 0.0, tolerance=0.5) == pytest.approx(2.0)
    assert dwell_in_band(history, "error", lower=-0.5, upper=0.5) == pytest.approx(2.0)


def test_specific_energy_is_conserved_by_ballistic_trade() -> None:
    gravity = 9.80665
    initial_speed = 100.0
    history = tuple(
        {
            "time_s": float(index),
            "altitude_m": 1_000.0 - float(index),
            "speed_m_s": math.sqrt(initial_speed**2 + 2.0 * gravity * index),
            "drag_force_n": 0.0,
            "thrust_n": 0.0,
            "mass_kg": 10.0,
        }
        for index in range(3)
    )

    energies = specific_energy(history)
    assert energies[0] == pytest.approx(energies[-1])
    assert energy_balance_residual(history) == pytest.approx(0.0, abs=1.0e-10)


def test_mass_balance_and_saturation_fraction_are_explicit() -> None:
    history = tuple(
        {
            "time_s": float(index),
            "mass_kg": 10.0 - 0.5 * index,
            "mass_rate_kg_s": 0.5,
            "actuator_saturated": float(index in (1, 2)),
        }
        for index in range(3)
    )

    assert integral_mass_balance_error(history) == pytest.approx(0.0)
    assert actuator_saturation_fraction(history) == pytest.approx(2.0 / 3.0)


def test_independent_force_closure_uses_saved_velocity_and_force() -> None:
    history = tuple(
        {
            "time_s": float(index),
            "mass_kg": 10.0,
            "xdt": float(index),
            "ydt": 0.0,
            "zdt": 0.0,
            "total_force_ecic_x_n": 10.0,
            "total_force_ecic_y_n": 0.0,
            "total_force_ecic_z_n": 0.0,
        }
        for index in range(3)
    )

    report = independent_force_closure(history)
    assert report["sample_count"] == 1.0
    assert report["maximum_normalized_residual"] == pytest.approx(0.0)


def test_independent_force_closure_reports_event_excluded_stencils() -> None:
    history = tuple(
        {
            "time_s": float(index),
            "mass_kg": 10.0,
            "xdt": float(index),
            "ydt": 0.0,
            "zdt": 0.0,
            "total_force_ecic_x_n": 10.0,
            "total_force_ecic_y_n": 0.0,
            "total_force_ecic_z_n": 0.0,
        }
        for index in range(9)
    )

    report = independent_force_closure(history, event_times=(7.0,))
    assert report["event_times"] == [7.0]
    assert report["event_excluded_sample_count"] == 2.0
    assert report["sample_count"] == 3.0
    assert report["maximum_time_s"] == 2.0


def test_independent_moment_closure_uses_saved_rates_and_moments() -> None:
    history = tuple(
        {
            "time_s": float(index),
            "inertia_x_kg_m2": 2.0,
            "inertia_y_kg_m2": 3.0,
            "inertia_z_kg_m2": 4.0,
            "wx": float(index),
            "wy": 0.0,
            "wz": 0.0,
            "total_moment_body_x_nm": 2.0,
            "total_moment_body_y_nm": 0.0,
            "total_moment_body_z_nm": 0.0,
        }
        for index in range(5)
    )

    report = independent_moment_closure(history)
    assert report["sample_count"] == 1.0
    assert report["maximum_normalized_residual"] == pytest.approx(0.0)


def test_convergence_metric_requires_aligned_histories() -> None:
    coarse = tuple({"time_s": float(index), "altitude_m": float(index)} for index in range(3))
    fine = tuple({"time_s": float(index), "altitude_m": float(index) + 0.25} for index in range(3))

    assert timestep_convergence_error(coarse, fine, "altitude_m") == pytest.approx(0.25)
    with pytest.raises(AssertionError, match="aligned"):
        timestep_convergence_error(coarse, (*fine, {"time_s": 3.0, "altitude_m": 3.25}), "altitude_m")
