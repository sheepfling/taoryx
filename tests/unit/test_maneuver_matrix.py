from __future__ import annotations

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]


def test_maneuver_evidence_bindings_resolve_to_matrix_cases_and_files() -> None:
    matrix = yaml.safe_load((ROOT / "verification/vehicle_maneuver_matrix.yaml").read_text(encoding="utf-8"))
    bindings = yaml.safe_load((ROOT / "verification/maneuver_evidence.yaml").read_text(encoding="utf-8"))
    case_ids = {f"{vehicle['id']}/{case['id']}" for vehicle in matrix["vehicles"] for case in vehicle["cases"]}

    assert bindings["cases"]
    for binding in bindings["cases"]:
        assert str(binding.get("case_id", binding["id"])) in case_ids
        assert binding["dimension"] in {"3dof", "6dof"}
        assert (ROOT / binding["problem"]).is_file()
        assert all((ROOT / table).is_file() for table in binding.get("tables", []))
    ####


def test_every_matrix_case_has_all_declared_dimension_bindings() -> None:
    matrix = yaml.safe_load((ROOT / "verification/vehicle_maneuver_matrix.yaml").read_text(encoding="utf-8"))
    bindings = yaml.safe_load((ROOT / "verification/maneuver_evidence.yaml").read_text(encoding="utf-8"))
    bound_dimensions: dict[str, set[str]] = {}
    for binding in bindings["cases"]:
        identifier = str(binding.get("case_id", binding["id"]))
        bound_dimensions.setdefault(identifier, set()).add(str(binding["dimension"]))

    for vehicle in matrix["vehicles"]:
        for case in vehicle["cases"]:
            identifier = f"{vehicle['id']}/{case['id']}"
            required = {str(case["dimension"])} if isinstance(case["dimension"], str) else {str(item) for item in case["dimension"]}
            assert bound_dimensions.get(identifier, set()) >= required, identifier
    ####
