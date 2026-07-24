from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("build_alpha1_feature_matrix", ROOT / "tools/build_alpha1_feature_matrix.py")
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_alpha1_matrix_covers_the_documented_surface() -> None:
    coverage = json.loads(
        (ROOT / "tests/fixtures/taos_e2e_v23/coverage/documented_surface_coverage.json").read_text(encoding="utf-8")
    )
    matrix = MODULE.build(coverage)

    assert matrix["summary"] == {
        "feature_count": 80,
        "covered_count": 80,
        "block_scopes": 33,
        "table_types": 19,
        "table_operations": 28,
    }
    assert len({item["feature_id"] for item in matrix["features"]}) == 80
    assert all(item["fixtures"] for item in matrix["features"])
    assert all(item["tests"] for item in matrix["features"])
    assert all(item["requirement_ids"] for item in matrix["features"])
####
