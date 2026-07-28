from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.imu_profile_comparison import compare_imu_profiles

ROOT = Path(__file__).resolve().parents[2]
UPSTREAM = ROOT.parent.parent / "imu-error-model"
BASELINE = ROOT / "tests/fixtures/imu_profiles/taoryx_demo.yaml"
CANDIDATE = UPSTREAM / "examples/imu_profiles/hardware-estimates/hg1700ag58.yaml"


def test_source_pinned_profile_comparison_is_reproducible() -> None:
    pytest.importorskip("imu_error_model")
    if not CANDIDATE.is_file():
        pytest.skip("local source-pinned imu-error-model checkout is not available")

    first = compare_imu_profiles(BASELINE, CANDIDATE, seed=41, horizon_s=0.2).as_dict()
    second = compare_imu_profiles(BASELINE, CANDIDATE, seed=41, horizon_s=0.2).as_dict()

    assert first == second
    assert first["baseline"]["profile"]["model_name"] == "TaoryxDemoIMU"
    assert first["candidate"]["profile"]["model_name"] == "HG1700AG58"
    assert first["baseline"]["valid_packet_count"] == 2
    assert first["candidate"]["valid_packet_count"] == 20
    assert first["baseline"]["packet_sha256"] != first["candidate"]["packet_sha256"]
