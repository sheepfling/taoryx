from __future__ import annotations

import math

import pytest

from taoryx.output_catalog import output_channel_spec
from taoryx.runtime.common import RuntimeState
from taoryx.runtime.lowering import _interpolate_output_state


def test_output_catalog_normalizes_private_segment_alias() -> None:
    spec = output_channel_spec("segment")

    assert spec is not None
    assert spec.semantic_name == "phase.segment"
    assert spec.interpolation == "step"
    assert spec.visualization_roles == ("all",)
    ####


def test_scheduled_output_interpolation_steps_segment_values() -> None:
    start = RuntimeState(0.0, (0.0,), named={"_segment": 1.0})
    end = RuntimeState(1.0, (1.0,), named={"_segment": 2.0})

    sample = _interpolate_output_state(start, end, 0.5, 0.5)

    assert sample.named["_segment"] == 1.0
    ####


def test_scheduled_output_interpolation_wraps_angles_on_shortest_path() -> None:
    start_angle = math.radians(179.0)
    end_angle = math.radians(-179.0)
    start = RuntimeState(0.0, (0.0,), named={"alpha": start_angle})
    end = RuntimeState(1.0, (1.0,), named={"alpha": end_angle})

    sample = _interpolate_output_state(start, end, 0.5, 0.5)

    assert sample.named["alpha"] == pytest.approx(math.pi, abs=1.0e-12)
    ####
