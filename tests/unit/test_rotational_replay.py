from __future__ import annotations

import json

import numpy as np
import pytest

from taoryx.runtime import RotationalTruthRecorder, RotationalTruthReplay, RotationTruth, RuntimeState


def _frame(time_s: float) -> dict[str, object]:
    return {
        "time_s": time_s,
        "orientation_eci_from_body": np.eye(3).tolist(),
        "angular_rate_body_radps": [0.0, 0.0, 0.25],
    }


def test_rotational_replay_requires_exact_accepted_timestamp(tmp_path) -> None:
    source = tmp_path / "rotation.jsonl"
    source.write_text("\n".join(json.dumps(_frame(time)) for time in (0.0, 0.1, 0.2)) + "\n", encoding="utf-8")
    replay = RotationalTruthReplay.from_jsonl(source)

    result = replay(RuntimeState(0.1, (0.0,)))

    assert result.time_s == pytest.approx(0.1)
    assert replay.request_log[-1]["status"] == "accepted"
    with pytest.raises(ValueError, match="no frame for accepted time"):
        replay(RuntimeState(0.15, (0.0,)))
    assert replay.request_log[-1]["status"] == "rejected"


def test_rotational_replay_rejects_duplicate_or_unsorted_frames(tmp_path) -> None:
    duplicate = tmp_path / "duplicate.jsonl"
    duplicate.write_text("\n".join(json.dumps(_frame(time)) for time in (0.0, 0.0)) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="strictly increasing"):
        RotationalTruthReplay.from_jsonl(duplicate)


def test_rotational_recorder_validates_provider_timestamp_and_writes_log(tmp_path) -> None:
    def provider(state: RuntimeState) -> RotationTruth:
        return RotationTruth(state.time, np.eye(3), np.array([0.0, 0.0, 0.25]))

    recorder = RotationalTruthRecorder(provider, source="unit-test")
    recorder(RuntimeState(0.2, (0.0,)))
    destination = recorder.write_request_log(tmp_path / "requests.jsonl")
    records = [json.loads(line) for line in destination.read_text(encoding="utf-8").splitlines()]

    assert records == [
        {
            "angular_rate_body_radps": [0.0, 0.0, 0.25],
            "requested_time_s": 0.2,
            "returned_time_s": 0.2,
            "status": "accepted",
            "timestamp_error_s": 0.0,
        }
    ]


def test_rotational_recorder_fails_closed_on_timestamp_mismatch() -> None:
    def provider(state: RuntimeState) -> RotationTruth:
        return RotationTruth(state.time + 0.01, np.eye(3), np.zeros(3))

    recorder = RotationalTruthRecorder(provider)
    with pytest.raises(ValueError, match="returned time"):
        recorder(RuntimeState(0.2, (0.0,)))
    assert recorder.records[-1]["status"] == "rejected"
