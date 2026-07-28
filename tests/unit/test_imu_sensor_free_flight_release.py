import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "verification/imu_sensor_free_flight_release.yaml"
CATALOG = ROOT / "resources/sensors/imu_profiles/catalog.json"


def test_free_flight_release_manifest_covers_required_evidence() -> None:
    payload = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    assert payload["status"] == "verified_free_flight_tranche"
    assert payload["source_profile"]["deterministic_fixture"] == "tests/fixtures/imu_profiles/taoryx_demo.yaml"
    assert payload["source_profile"]["upstream_commit"] == json.loads(CATALOG.read_text(encoding="utf-8"))["source_commit"]

    scenarios = {scenario["id"]: scenario for scenario in payload["scenarios"]}
    assert {
        "hummingbird-hover",
        "hummingbird-rate",
        "hummingbird-translation",
        "hummingbird-pseudo6dof",
        "hummingbird-rotation",
        "x8-powered",
        "rotating-earth-transport",
        "rotational-replay",
        "profile-comparison",
    } <= scenarios.keys()
    assert scenarios["x8-powered"]["vehicle"] == "skywalker_x8"
    assert scenarios["rotating-earth-transport"]["fidelity"] == "transport_only"
    assert scenarios["rotational-replay"]["source_truth"] == "timestamped_external_rotation"
    assert "contact" in " ".join(payload["known_limitations"]).lower()
