from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
DEMO = ROOT / "examples/taoryx/rotating_earth_sensor_demo.py"

pytestmark = pytest.mark.plot


def _load_demo():
    spec = importlib.util.spec_from_file_location("rotating_earth_sensor_demo", DEMO)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_rotating_earth_demo_produces_nominal_eci_sensor_evidence(tmp_path: Path) -> None:
    report = _load_demo().run(tmp_path)

    assert report["completed"] is True
    assert report["baseline_policy"] == "does-not-modify-omega-zero-family-fixtures"
    contract = report["truth_contract"]
    assert contract["frame"] == "ECI"
    assert contract["earth_rate"]["omega_rad_s"] == 7.2921151467e-5
    execution = report["sensor_execution"]
    assert execution["measurement_summary"]["valid"] == 10
    assert execution["truth_contract"]["support_mode"] == "transport-only"
    assert (tmp_path / "rotating-earth-report.json").is_file()
    assert (tmp_path / "rotating-earth-sensor.png").is_file()
