"""Tests for explicit bilateral surface mapping hypotheses."""

from __future__ import annotations

import pytest

from taoryx.airbreathing_control_mapping import BilateralSurfaceMapping, x8_mapping_hypotheses


def test_bilateral_mapping_round_trips_both_sign_hypotheses() -> None:
    for mapping in x8_mapping_hypotheses():
        physical = mapping.virtual_to_physical(3.0, 1.5)
        assert mapping.physical_to_virtual(*physical) == pytest.approx((3.0, 1.5))
        assert mapping.as_dict()["status"] == "hypothesis"
    ####


def test_bilateral_mapping_rejects_invalid_sign() -> None:
    with pytest.raises(ValueError, match="differential sign"):
        BilateralSurfaceMapping("invalid", 0, "test")
    ####
