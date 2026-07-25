from __future__ import annotations

import ast
import json
from pathlib import Path

import pytest

from taoryx.trajectory import CaseIntent, CaseValue, ObservationSchema, ResolutionError, load_family_catalog, resolve_case

ROOT = Path(__file__).resolve().parents[2]
FIXTURES = ROOT / "tests" / "fixtures" / "alpha2_case_contracts"
CATALOG = load_family_catalog(ROOT / "verification" / "alpha2_family_catalog.yaml")


def _load_case(name: str) -> CaseIntent:
    import yaml

    return CaseIntent.model_validate(yaml.safe_load((FIXTURES / name).read_text(encoding="utf-8")))


def test_resolved_case_is_canonical_and_provenance_complete() -> None:
    resolved = resolve_case(_load_case("case-light.yaml"), CATALOG)

    assert resolved.parameters["vehicle.mass.initial"].value == pytest.approx(850.0)
    assert resolved.parameters["mission.initial_speed"].value == pytest.approx(50.0)
    assert resolved.parameters["mission.target_range"].value == pytest.approx(80_000.0)
    assert resolved.parameters["vehicle.booster.thrust"].value == pytest.approx(120_000.0)
    assert resolved.recompute_identity() == resolved.identity_sha256
    assert {row["parameter_id"] for row in resolved.explain()} == set(resolved.parameters)
    assert all(row["source"] for row in resolved.explain())
    speed_provenance = next(row for row in resolved.explain() if row["parameter_id"] == "mission.initial_speed")
    assert speed_provenance["input_value"] == pytest.approx(180.0)
    assert speed_provenance["canonical_value"] == pytest.approx(50.0)


def test_identical_case_resolution_has_stable_identity_and_json(tmp_path: Path) -> None:
    intent = _load_case("case-heavy.yaml")
    first = resolve_case(intent, CATALOG)
    second = resolve_case(intent, CATALOG)

    assert first.identity_sha256 == second.identity_sha256
    first_path = tmp_path / "first.json"
    second_path = tmp_path / "second.json"
    first.write_json(str(first_path))
    second.write_json(str(second_path))
    assert first_path.read_bytes() == second_path.read_bytes()
    assert json.loads(first_path.read_text())["identity_sha256"] == first.identity_sha256


def test_resolver_fails_closed_for_unknown_and_incompatible_inputs() -> None:
    base = _load_case("case-heavy.yaml")
    with pytest.raises(ResolutionError, match="unknown-override"):
        resolve_case(base.model_copy(update={"overrides": {"vehicle.missing": CaseValue(value=1.0)}}), CATALOG)
    with pytest.raises(ResolutionError, match="unit-mismatch"):
        resolve_case(
            base.model_copy(
                update={
                    "overrides": {
                        "mission.initial_speed": CaseValue(value=1.0, unit="kg"),
                    }
                }
            ),
            CATALOG,
        )
    with pytest.raises(ResolutionError, match="unsupported-fidelity"):
        resolve_case(base.model_copy(update={"fidelity": "rigid_body_6dof"}), CATALOG)


def test_unknown_requested_schema_channels_fail_closed() -> None:
    base = _load_case("case-heavy.yaml")
    with pytest.raises(ResolutionError, match="unknown-control"):
        resolve_case(base.model_copy(update={"requested_controls": ("command.missing",)}), CATALOG)
    with pytest.raises(ResolutionError, match="unknown-observation"):
        resolve_case(base.model_copy(update={"requested_observations": ("state.missing",)}), CATALOG)


def test_neutral_contract_module_has_no_native_runtime_imports() -> None:
    source = (ROOT / "src" / "taoryx" / "trajectory" / "contracts.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    imports = [node.module or "" for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)]
    assert not any(module.startswith("taoryx.runtime") for module in imports)


def test_observation_schema_distinguishes_achieved_resources_and_event_predictions() -> None:
    observations = (
        ObservationSchema(id="actuator.bank.achieved", unit="deg", kind="actuator_achieved"),
        ObservationSchema(id="propellant.fuel_remaining", unit="kg", kind="resource"),
        ObservationSchema(id="propulsion.time_to_burnout", unit="s", kind="event_prediction"),
    )

    assert [item.kind for item in observations] == ["actuator_achieved", "resource", "event_prediction"]
