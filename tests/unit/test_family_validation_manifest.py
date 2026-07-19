from __future__ import annotations

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "verification/family_validation_execution.yaml"


def test_family_validation_manifest_has_distinct_semantic_families() -> None:
    document = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    scenarios = document["scenarios"]

    assert {scenario["family"] for scenario in scenarios} == {
        "unpowered_hypersonic_glider",
        "powered_fixed_wing",
        "quadrotor",
    }
    assert all(scenario["status"] in document["statuses"] for scenario in scenarios)
    assert all(scenario["phases"] for scenario in scenarios)


def test_manifest_does_not_promote_missing_final_problems() -> None:
    document = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    for scenario in document["scenarios"]:
        required_problem = scenario.get("required_problem")
        if required_problem and not (ROOT / required_problem).exists():
            assert scenario["status"] == "blocked"
            assert scenario.get("blocking_reason")


def test_candidate_problems_exist() -> None:
    document = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    for scenario in document["scenarios"]:
        candidate = scenario.get("candidate_problem")
        assert candidate
        assert (ROOT / candidate).exists(), candidate
        for supporting in scenario.get("supporting_problems", ()):
            assert (ROOT / str(supporting)).exists(), supporting
