from __future__ import annotations

import random
from pathlib import Path

from tools.tune_rectangle_controller import _sample_candidate, inject_controls, load_cases


def test_inject_controls_replaces_existing_and_adds_missing_runtime_attributes() -> None:
    source = (
        "*runtime status route mode=rectangle rectangle-bank-deg=15\n"
        "*runtime status guidance rectangle-coordinated-turn=false\n"
    )

    tuned = inject_controls(
        source,
        {
            "rectangle-bank-deg": 22.5,
            "rectangle-heading-gain-nm-per-rad": 125000.0,
        },
    )

    assert "rectangle-bank-deg=22.5" in tuned
    assert "rectangle-heading-gain-nm-per-rad=125000" in tuned
    assert "rectangle-coordinated-turn=true" in tuned
    ####


def test_inject_controls_requires_a_status_line_for_missing_attributes() -> None:
    source = "*title no-runtime-status\n"

    try:
        inject_controls(source, {"rectangle-bank-gain-nm-per-rad": 10.0})
    except ValueError as error:
        assert "guidance or route line" in str(error)
    else:
        raise AssertionError("missing guidance status line should fail closed")
    ####


def test_rectangle_tuner_samples_lqr_weights_logarithmically() -> None:
    """Positive Q/R weights use the declared logarithmic search domain."""

    case = next(
        item
        for item in load_cases(Path("verification/rectangle_tuning.yaml"))
        if item.identifier == "b747"
    )
    sample = _sample_candidate(case, random.Random(1995))

    assert 1.0e8 <= sample["q-angle"] <= 1.0e13
    assert 1.0e-2 <= sample["q-rate"] <= 1.0e2
    assert 1.0e-3 <= sample["r-moment"] <= 1.0e1
    assert sample["q-angle"] > 0.0
    assert sample["r-moment"] > 0.0
    ####
