"""Regression checks for the bounded A320 surrogate runtime evidence."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from taoryx.trajectory import A320OpenAPOperatingPoint, A320Pseudo6DOFModel, A320Pseudo6DOFOperatingPoint

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.artifact
def test_a320_pseudo6dof_runtime_report_is_verified() -> None:
    report = json.loads((ROOT / "families/a320_openap_jsbsim_pseudo6dof/validation/runtime-qualification.json").read_text(encoding="utf-8"))

    assert report["status"] == "verified"
    assert report["qualification_class"] == "surrogate_composite"
    assert all(item["pass"] for item in report["trim"].values())
    assert report["coordinated_turn"]["pass"] is True
    assert all(item["pass"] for item in report["control_pulses"].values())
    assert report["authority_checks"]["pass"] is True


def test_a320_pseudo6dof_source_moment_derivatives_remain_distinct_from_policy() -> None:
    model = A320Pseudo6DOFModel.from_repository()
    operating_point = A320OpenAPOperatingPoint(11000.0, 0.78, 60000.0)
    source_point = model.evaluate(
        A320Pseudo6DOFOperatingPoint(
            operating_point,
            beta_rad=0.02,
            aileron_rad=0.01,
            elevator_rad=-0.01,
            rudder_rad=0.01,
        )
    )
    derivatives = model.rotational_derivatives(source_point.input)

    assert derivatives["roll_rate_rad_s"] == source_point.roll_moment_nm / model.inertia_for_mass(60000.0)[0]
    assert derivatives["pitch_rate_rad_s"] == source_point.pitch_moment_nm / model.inertia_for_mass(60000.0)[1]
    assert derivatives["yaw_rate_rad_s"] == source_point.yaw_moment_nm / model.inertia_for_mass(60000.0)[2]
