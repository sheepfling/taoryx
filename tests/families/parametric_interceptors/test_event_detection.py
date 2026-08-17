"""Focused within-step interceptor event-localization witnesses."""

from __future__ import annotations

import pytest
from taoryx_parametric_interceptors.event_detection import (
    first_capture_time_within_step,
    first_ground_contact_time_within_step,
    localize_translation_step_event,
)


def test_linear_fly_through_finds_first_capture_boundary() -> None:
    event_time = first_capture_time_within_step(
        relative_position_m=(100.0, 0.0, 0.0),
        relative_velocity_mps=(-200.0, 0.0, 0.0),
        interceptor_acceleration_mps2=(0.0, 0.0, 0.0),
        capture_radius_m=10.0,
        duration_s=1.0,
    )

    assert event_time == pytest.approx(0.45)
    ####


def test_semi_implicit_quadratic_path_and_tangent_contact_are_localized() -> None:
    accelerated = first_capture_time_within_step(
        relative_position_m=(100.0, 0.0, 0.0),
        relative_velocity_mps=(0.0, 0.0, 0.0),
        interceptor_acceleration_mps2=(10.0, 0.0, 0.0),
        capture_radius_m=10.0,
        duration_s=4.0,
    )
    tangent = first_capture_time_within_step(
        relative_position_m=(20.0, 10.0, 0.0),
        relative_velocity_mps=(-20.0, 0.0, 0.0),
        interceptor_acceleration_mps2=(0.0, 0.0, 0.0),
        capture_radius_m=10.0,
        duration_s=2.0,
    )

    assert accelerated == pytest.approx(3.0)
    assert tangent == pytest.approx(1.0)
    ####


def test_ground_contact_precedes_later_capture_and_uses_committed_descent() -> None:
    ground = first_ground_contact_time_within_step(
        altitude_m=100.0,
        vertical_velocity_mps=-200.0,
        vertical_acceleration_mps2=0.0,
        duration_s=1.0,
    )
    event = localize_translation_step_event(
        relative_position_m=(190.0, 0.0, 0.0),
        relative_velocity_mps=(-200.0, 0.0, 0.0),
        interceptor_acceleration_mps2=(0.0, 0.0, 0.0),
        capture_radius_m=10.0,
        altitude_m=100.0,
        vertical_velocity_mps=-200.0,
        duration_s=1.0,
    )

    assert ground == pytest.approx(0.5)
    assert event is not None
    assert event.kind == "ground_contact"
    assert event.time_from_step_start_s == pytest.approx(0.5)
    ####


def test_localizer_returns_none_without_an_intersection() -> None:
    assert (
        first_capture_time_within_step(
            relative_position_m=(100.0, 100.0, 0.0),
            relative_velocity_mps=(10.0, 0.0, 0.0),
            interceptor_acceleration_mps2=(0.0, 0.0, 0.0),
            capture_radius_m=1.0,
            duration_s=1.0,
        )
        is None
    )
    ####
