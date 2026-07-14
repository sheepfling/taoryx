from __future__ import annotations

import csv
import importlib
import json
from pathlib import Path

import pytest

from taoryx.catalog import load_catalog
from tools.check_algorithm_bindings import audit

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


def test_restored_catalog_has_a_binding_for_every_algorithm() -> None:
    count, missing = audit()
    assert count == 107
    assert missing == ()
####


def test_generated_status_requires_importable_targets_and_verification_paths() -> None:
    status = ROOT / "metadata" / "algorithm_catalog" / "algorithm_status.csv"
    with status.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))

    assert len(rows) == 107
    assert all(row["binding_documented"] == "true" for row in rows)
    assert all(row["target_importable"] == "true" for row in rows)
    assert all(row["verification_paths_exist"] == "true" for row in rows)
    assert all(row["roadmap_status"] in {"planned", "complete"} for row in rows)
    assert all(row["implementation_stage"] == "unit_verified" for row in rows)
    assert all(row["historical_equivalence"] == "unverified" for row in rows)
    for row in rows:
        module_name, _, symbol = row["documented_binding"].rpartition(".")
        assert hasattr(importlib.import_module(module_name), symbol)
        assert all((ROOT / path).exists() for path in row["verification_tests"].split(";") if path)
####


def test_status_distinguishes_roadmap_completion_from_unit_verification() -> None:
    status = ROOT / "metadata" / "algorithm_catalog" / "algorithm_status.csv"
    with status.open(newline="", encoding="utf-8") as handle:
        rows = {row["id"]: row for row in csv.DictReader(handle)}

    assert rows["TAOS-ALG-GUID-008"]["roadmap_status"] == "complete"
    promoted = {
        "TAOS-ALG-COORD-001",
        "TAOS-ALG-COORD-002",
        "TAOS-ALG-COORD-003",
        "TAOS-ALG-COORD-004",
        "TAOS-ALG-COORD-005",
        "TAOS-ALG-COORD-006",
        "TAOS-ALG-COORD-007",
        "TAOS-ALG-COORD-008",
        "TAOS-ALG-COORD-009",
        "TAOS-ALG-COORD-010",
        "TAOS-ALG-COORD-011",
        "TAOS-ALG-COORD-012",
        "TAOS-ALG-COORD-013",
        "TAOS-ALG-COORD-014",
        "TAOS-ALG-COORD-015",
        "TAOS-ALG-COORD-016",
        "TAOS-ALG-COORD-017",
        "TAOS-ALG-COORD-020",
        "TAOS-ALG-DYN-001",
        "TAOS-ALG-DYN-002",
        "TAOS-ALG-DYN-003",
        "TAOS-ALG-DYN-004",
        "TAOS-ALG-DYN-005",
        "TAOS-ALG-DYN-006",
        "TAOS-ALG-DYN-007",
        "TAOS-ALG-DYN-008",
        "TAOS-ALG-DYN-009",
        "TAOS-ALG-DYN-010",
        "TAOS-ALG-DYN-011",
        "TAOS-ALG-DYN-012",
        "TAOS-ALG-ENV-002",
        "TAOS-ALG-ENV-003",
        "TAOS-ALG-ENV-004",
        "TAOS-ALG-FORCE-001",
        "TAOS-ALG-FORCE-002",
        "TAOS-ALG-FORCE-003",
        "TAOS-ALG-EXEC-001",
        "TAOS-ALG-EXEC-002",
        "TAOS-ALG-EXEC-003",
        "TAOS-ALG-EXEC-004",
        "TAOS-ALG-EXEC-005",
        "TAOS-ALG-EXEC-006",
        "TAOS-ALG-EXEC-007",
        "TAOS-ALG-EXEC-008",
        "TAOS-ALG-EXEC-009",
        "TAOS-ALG-PRB-001",
        "TAOS-ALG-PRB-002",
        "TAOS-ALG-PRB-003",
        "TAOS-ALG-PRB-004",
        "TAOS-ALG-PRB-005",
        "TAOS-ALG-PRB-006",
        "TAOS-ALG-PRB-007",
        "TAOS-ALG-PRB-008",
        "TAOS-ALG-PRB-009",
        "TAOS-ALG-PRB-010",
        "TAOS-ALG-PRB-011",
        "TAOS-ALG-PRB-012",
        "TAOS-ALG-TABLE-001",
        "TAOS-ALG-TABLE-002",
        "TAOS-ALG-TABLE-003",
        "TAOS-ALG-TABLE-004",
        "TAOS-ALG-TABLE-005",
        "TAOS-ALG-TABLE-006",
        "TAOS-ALG-TABLE-007",
        "TAOS-ALG-TABLE-008",
        "TAOS-ALG-TABLE-009",
        "TAOS-ALG-GUID-008",
        "TAOS-ALG-GUID-003",
        "TAOS-ALG-GUID-004",
        "TAOS-ALG-GUID-005",
        "TAOS-ALG-GRAV-002",
        "TAOS-ALG-GRAV-003",
        "TAOS-ALG-GRAV-005",
        "TAOS-ALG-OPT-004",
        "TAOS-ALG-OPT-001",
        "TAOS-ALG-SEARCH-001",
        "TAOS-ALG-SEARCH-002",
        "TAOS-ALG-SEARCH-003",
        "TAOS-ALG-SEARCH-004",
        "TAOS-ALG-SEARCH-005",
        "TAOS-ALG-COORD-018",
        "TAOS-ALG-COORD-019",
        "TAOS-ALG-ENV-005",
        "TAOS-ALG-ENV-001",
        "TAOS-ALG-FORCE-004",
        "TAOS-ALG-FORCE-005",
        "TAOS-ALG-GRAV-001",
        "TAOS-ALG-GRAV-004",
        "TAOS-ALG-OUT-001",
        "TAOS-ALG-GEO-001",
        "TAOS-ALG-GEO-002",
        "TAOS-ALG-GEO-003",
        "TAOS-ALG-RADAR-001",
        "TAOS-ALG-REL-001",
        "TAOS-ALG-IIP-001",
        "TAOS-ALG-IIP-002",
        "TAOS-ALG-IIP-003",
        "TAOS-ALG-AERO-001",
        "TAOS-ALG-AERO-002",
        "TAOS-ALG-GUID-001",
        "TAOS-ALG-GUID-002",
        "TAOS-ALG-GUID-006",
        "TAOS-ALG-GUID-007",
        "TAOS-ALG-GUID-009",
        "TAOS-ALG-OPT-002",
        "TAOS-ALG-OPT-003",
        "TAOS-ALG-OPT-005",
    }
    assert {row["id"] for row in rows.values() if row["roadmap_status"] == "complete"} == promoted
    assert sum(row["roadmap_status"] == "complete" for row in rows.values()) == 107
    assert sum(row["roadmap_status"] == "planned" for row in rows.values()) == 0
    assert rows["TAOS-ALG-TABLE-006"]["roadmap_status"] == "complete"
    assert rows["TAOS-ALG-TABLE-008"]["roadmap_status"] == "complete"
    assert all(row["implementation_stage"] == "unit_verified" for row in rows.values())
####
