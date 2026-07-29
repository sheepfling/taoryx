"""Regression checks for the fresh-process NESC replay."""

import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.artifact


def test_nesc_replay_is_verified() -> None:
    report = json.loads((ROOT / "verification/daveml_nesc_replay_evidence.json").read_text(encoding="utf-8"))
    assert report["status"] == "verified"
    assert report["subprocess_returncode"] == 0
    assert report["replay"]["model_id"] == "nasa-nesc-two-stage-rocket-scenario17"
