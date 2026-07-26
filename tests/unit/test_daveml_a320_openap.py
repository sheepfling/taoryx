"""Executable family-library checks for the derived-exact A320 product."""

from __future__ import annotations

import pytest

from taoryx.trajectory import A320OpenAPEnvelopeError, A320OpenAPModel, A320OpenAPOperatingPoint


@pytest.fixture(scope="module")
def a320() -> A320OpenAPModel:
    return A320OpenAPModel.from_repository()


def test_a320_openap_reference_cruise_matches_pinned_evidence(a320: A320OpenAPModel) -> None:
    result = a320.evaluate(A320OpenAPOperatingPoint(11000.0, 0.78, 60000.0))

    assert result.drag_n == pytest.approx(32807.716586087394, abs=1.0e-9)
    assert result.drag_coefficient == pytest.approx(0.02745764589804906, abs=1.0e-14)
    assert result.lift_coefficient == pytest.approx(0.49244668846040285, abs=1.0e-14)
    assert result.required_throttle_ratio == pytest.approx(0.717480693769065, abs=1.0e-14)
    assert result.fuel_flow_kg_s == pytest.approx(0.696755122612149, abs=1.0e-12)
    assert result.feasible
    assert result.qualification_class == "derived_exact"
    assert result.package_sha256 == a320.provenance["package_sha256"]


def test_a320_openap_binds_trim_tuning_objectives_and_linearization(a320: A320OpenAPModel) -> None:
    point = A320OpenAPOperatingPoint(11000.0, 0.78, 60000.0)
    result = a320.evaluate(point)
    trim = a320.trim_level_flight(point)
    tuning = a320.tune_cruise_throttle(point)
    linearization = a320.linearize_point_mass(point, trim)
    objectives = a320.score_level_flight_objectives(result)

    assert trim.success
    assert trim.controls["flight_path_angle_rad"] == pytest.approx(0.0, abs=1.0e-10)
    assert trim.controls["throttle_ratio"] == pytest.approx(result.required_throttle_ratio, abs=1.0e-10)
    assert tuning.converged
    assert tuning.parameters[0] == pytest.approx(result.required_throttle_ratio, abs=1.0e-5)
    assert linearization.a_matrix.shape == (4, 4)
    assert linearization.b_matrix.shape == (4, 2)
    assert linearization.metadata_dict["claim_boundary"] == "point_mass_3dof; no rigid_body_moments"
    assert objectives["status"] == "pass"


def test_a320_openap_fails_closed_outside_source_envelope(a320: A320OpenAPModel) -> None:
    with pytest.raises(A320OpenAPEnvelopeError):
        a320.evaluate(A320OpenAPOperatingPoint(11000.0, 0.9, 60000.0))
