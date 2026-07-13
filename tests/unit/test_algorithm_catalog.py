from __future__ import annotations

import json
from pathlib import Path

import pytest

from taoryx.catalog import load_catalog

ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / "tests" / "fixtures" / "algorithm_catalog" / "minimal.json"


def test_catalog_indexes_dependencies_and_equations(tmp_path: Path) -> None:
    equation_map = tmp_path / "equations.csv"
    equation_map.write_text(
        'equation,algorithm_ids\n2-1,"[""A-001""]"\n2-2,"[""A-002""]"\n',
        encoding="utf-8",
    )

    catalog = load_catalog(FIXTURE, equation_map)

    assert catalog.get("A-002").implementation_target.symbol == "transform"
    assert [item.id for item in catalog.dependency_order()] == ["A-001", "A-002"]
    assert [item.id for item in catalog.for_equation("2-1")] == ["A-001"]
    assert [item.id for item in catalog.by_phase("M1")] == ["A-002"]
####


def test_catalog_rejects_dependency_cycles(tmp_path: Path) -> None:
    payload = json.loads(FIXTURE.read_text(encoding="utf-8"))
    payload["algorithms"][0]["dependencies"] = ["A-002"]
    path = tmp_path / "cycle.json"
    path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="cycle"):
        load_catalog(path)
####
