from __future__ import annotations

from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "spectre_simple_aero_v1"
pytestmark = pytest.mark.spectre


def test_spectre_simple_aero_manifest_matches_bundled_files() -> None:
    manifest = yaml.safe_load((ROOT / "manifest.yaml").read_text(encoding="utf-8"))
    listed = manifest["files"]
    actual = sorted(path.name for path in ROOT.glob("*.yaml") if path.name != "manifest.yaml")

    assert listed == actual
    assert manifest["bundle"] == "simple_aero"
    assert manifest["source_project"] == "darts-pre-v0.2.13"


def test_spectre_simple_aero_examples_share_the_expected_scaffold() -> None:
    for path in sorted(ROOT.glob("*.yaml")):
        if path.name == "manifest.yaml":
            continue
        ####
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
        assert payload["integration_timestep"] == 0.1
        assert payload["trajectory"]["initial_altitude_m"] == 0.0
        assert payload["trajectory"]["initial_speed_mps"] == 10.0
        assert payload["trajectory"]["launch_point"]["latitude_deg"] == 35.8766
        assert "solution" in payload
        assert payload["solution"]["type"]
        assert payload["segment_family"] == path.stem
        ####
    ####
