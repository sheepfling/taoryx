from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.simple_aero_validation import validate_simple_aero_segment_fixture

ROOT = Path(__file__).resolve().parents[2]
SEGMENT_ROOT = ROOT / "tests/fixtures/problem_file_dumps/simple_aero_segments"

pytestmark = [pytest.mark.simple_aero, pytest.mark.segment]

FAMILIES = (
    "ballistic",
    "cbcr",
    "crossrange",
    "marv",
    "phugoid",
    "range_extension",
    "skip",
    "slalom",
    "weave",
)


@pytest.mark.parametrize("family", FAMILIES)
def test_simple_aero_segment_passes_fixture_quality_ladder(family: str, tmp_path: Path) -> None:
    result = validate_simple_aero_segment_fixture(
        SEGMENT_ROOT / f"{family}.prb",
        family=family,
        output_dir=tmp_path / family,
        max_steps=5_000,
    )

    assert result.passed, [(check.name, check.message) for check in result.checks]
    assert [check.name for check in result.checks] == [
        "grammar",
        "runtime-completion",
        "required-telemetry",
        "finite-telemetry",
        "monotonic-time",
        "single-segment-span",
    ]
    assert result.physical_quality_pending
    assert result.claim_boundary
    ####


def test_simple_aero_segment_validator_does_not_claim_vehicle_quality(tmp_path: Path) -> None:
    result = validate_simple_aero_segment_fixture(
        SEGMENT_ROOT / "phugoid.prb",
        family="phugoid",
        output_dir=tmp_path / "phugoid",
        max_steps=5_000,
    )

    assert result.passed
    assert "not a vehicle-wide aero validation" in result.claim_boundary
    assert result.deferred_quality_gates == (
        "alpha profile interpolation",
        "aero table bounds",
        "energy closure",
        "time-step convergence",
    )
    ####

