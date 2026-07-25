from __future__ import annotations

import ast
import json
from pathlib import Path

import pytest

from taoryx.trajectory import (
    CaseIntent,
    CaseValue,
    DerivedParameter,
    FamilyCatalog,
    FamilyPackage,
    ObservationSchema,
    ParameterSchema,
    ResolutionError,
    VariantModifier,
    VariantSpace,
    load_family_catalog,
    resolve_case,
)

ROOT = Path(__file__).resolve().parents[2]
FIXTURES = ROOT / "tests" / "fixtures" / "alpha2_case_contracts"
CATALOG = load_family_catalog(ROOT / "verification" / "alpha2_family_catalog.yaml")


def _load_case(name: str) -> CaseIntent:
    import yaml

    return CaseIntent.model_validate(yaml.safe_load((FIXTURES / name).read_text(encoding="utf-8")))


def _variant_catalog(*, policy: str = "reject") -> FamilyCatalog:
    """Build a small variantized view of the checked-in Simple Aero family."""

    family = CATALOG.family("simple_aero")
    parameters = family.parameters + (
        ParameterSchema(id="vehicle.mass.propellant", canonical_unit="kg", default=100.0, minimum=0.0),
        ParameterSchema(id="vehicle.mass.wet", canonical_unit="kg", role="derived", minimum=1.0),
    )
    variant_space = VariantSpace(
        policy=policy,
        modifiers=(
            VariantModifier(
                id="propulsion.thrust_scale",
                target="vehicle.booster.thrust",
                operation="scale",
                canonical_unit="dimensionless",
                minimum=0.5,
                maximum=1.5,
                qualified_minimum=0.8,
                qualified_maximum=1.2,
                requires_retrim=True,
            ),
        ),
        derived=(
            DerivedParameter(
                id="vehicle.mass.wet",
                operation="sum",
                dependencies=("vehicle.mass.initial", "vehicle.mass.propellant"),
                canonical_unit="kg",
            ),
        ),
    )
    return FamilyCatalog(families=(family.model_copy(update={"parameters": parameters, "variant_space": variant_space}),))


def _archetype_variant_catalog(archetype: str) -> FamilyCatalog:
    """Build a synthetic fixed-wing/rotorcraft contract fixture.

    These are resolver fixtures, not vehicle models.  Their purpose is to
    prove that distinct control/parameter vocabularies can use the same
    bounded variant compiler without a family-specific resolver.
    """

    base = CATALOG.family("simple_aero")
    if archetype == "fixed_wing":
        parameter_id = "vehicle.aero.lift_scale"
        modifier_id = "aero.lift_scale"
        default = 1.0
    elif archetype == "rotorcraft":
        parameter_id = "vehicle.rotor.collective_authority"
        modifier_id = "rotor.collective_authority_scale"
        default = 1.0
    else:
        raise AssertionError(f"unknown contract fixture archetype: {archetype}")
    parameters = base.parameters + (
        ParameterSchema(id=parameter_id, canonical_unit="dimensionless", default=default, minimum=0.1, maximum=2.0),
        ParameterSchema(id="vehicle.mass.propellant", canonical_unit="kg", default=100.0, minimum=0.0),
        ParameterSchema(id="vehicle.mass.wet", canonical_unit="kg", role="derived", minimum=1.0),
    )
    variant_space = VariantSpace(
        policy="reject",
        modifiers=(
            VariantModifier(
                id=modifier_id,
                target=parameter_id,
                operation="scale",
                canonical_unit="dimensionless",
                minimum=0.5,
                maximum=1.5,
                qualified_minimum=0.8,
                qualified_maximum=1.2,
                requires_retrim=True,
                provenance=f"synthetic {archetype} interface fixture",
            ),
        ),
        derived=(
            DerivedParameter(
                id="vehicle.mass.wet",
                operation="sum",
                dependencies=("vehicle.mass.initial", "vehicle.mass.propellant"),
                canonical_unit="kg",
            ),
        ),
    )
    family = FamilyPackage.model_validate(
        base.model_dump(mode="python")
        | {
            "family_id": f"{archetype}_contract_fixture",
            "display_name": f"Synthetic {archetype} variant contract fixture",
            "parameters": parameters,
            "variant_space": variant_space,
            "fidelities": ("point_mass_3dof", "pseudo_6dof", "rigid_body_6dof"),
        }
    )
    return FamilyCatalog(families=(family,))


def test_resolved_case_is_canonical_and_provenance_complete() -> None:
    resolved = resolve_case(_load_case("case-light.yaml"), CATALOG)

    assert resolved.parameters["vehicle.mass.initial"].value == pytest.approx(850.0)
    assert resolved.parameters["mission.initial_speed"].value == pytest.approx(50.0)
    assert resolved.parameters["mission.target_range"].value == pytest.approx(80_000.0)
    assert resolved.parameters["vehicle.booster.thrust"].value == pytest.approx(120_000.0)
    assert resolved.recompute_identity() == resolved.identity_sha256
    assert resolved.variant_resolution is not None
    assert resolved.variant_resolution.status == "qualified"
    assert resolved.resolved_variant is not None
    assert resolved.resolved_variant.fingerprint == resolved.variant_resolution.fingerprint
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


def test_bounded_variant_resolution_records_derivation_and_qualification() -> None:
    base = _load_case("case-heavy.yaml")
    resolved = resolve_case(
        base.model_copy(update={"variant_parameters": {"propulsion.thrust_scale": CaseValue(value=1.1)}}),
        _variant_catalog(),
    )

    assert resolved.parameters["vehicle.booster.thrust"].value == pytest.approx(110_000.0)
    assert resolved.parameters["vehicle.mass.wet"].value == pytest.approx(1100.0)
    assert resolved.variant_resolution is not None
    assert resolved.variant_resolution.status == "qualified"
    assert resolved.variant_resolution.invalidations == ("retrim:vehicle.booster.thrust",)
    assert len(resolved.variant_resolution.fingerprint) == 64
    wet = next(row for row in resolved.explain() if row["parameter_id"] == "vehicle.mass.wet")
    assert wet["dependencies"] == ["vehicle.mass.initial", "vehicle.mass.propellant"]
    assert wet["derivation"] == "sum(vehicle.mass.initial, vehicle.mass.propellant)"
    assert resolved.recompute_identity() == resolved.identity_sha256


def test_bounded_variant_can_be_extended_or_projected_explicitly() -> None:
    base = _load_case("case-heavy.yaml")
    extended = resolve_case(
        base.model_copy(update={"variant_parameters": {"propulsion.thrust_scale": CaseValue(value=1.4)}}),
        _variant_catalog(),
    )
    assert extended.variant_resolution is not None
    assert extended.variant_resolution.status == "extended"
    assert extended.variant_resolution.diagnostics

    projected = resolve_case(
        base.model_copy(update={"variant_parameters": {"propulsion.thrust_scale": CaseValue(value=2.0)}}),
        _variant_catalog(policy="project"),
    )
    assert projected.parameters["vehicle.booster.thrust"].value == pytest.approx(150_000.0)
    assert projected.variant_resolution is not None
    assert projected.variant_resolution.status == "projected"
    assert projected.variant_resolution.projection_distance == pytest.approx(0.5)


def test_bounded_variant_rejects_invalid_candidates_and_derived_overrides() -> None:
    base = _load_case("case-heavy.yaml")
    with pytest.raises(ResolutionError, match="variant-out-of-range"):
        resolve_case(
            base.model_copy(update={"variant_parameters": {"propulsion.thrust_scale": CaseValue(value=2.0)}}),
            _variant_catalog(),
        )
    with pytest.raises(ResolutionError, match="derived-override"):
        resolve_case(
            base.model_copy(update={"overrides": {"vehicle.mass.wet": CaseValue(value=5.0)}}),
            _variant_catalog(),
        )


def test_checked_in_catalog_exposes_bounded_thrust_variant() -> None:
    base = _load_case("case-heavy.yaml")
    resolved = resolve_case(
        base.model_copy(update={"variant_parameters": {"propulsion.thrust_scale": CaseValue(value=0.9)}}),
        CATALOG,
    )

    assert resolved.parameters["vehicle.booster.thrust"].value == pytest.approx(90_000.0)
    assert resolved.variant_resolution is not None
    assert resolved.variant_resolution.modifiers_applied == ("propulsion.thrust_scale",)


@pytest.mark.parametrize("archetype", ["fixed_wing", "rotorcraft"])
def test_fixed_wing_and_rotorcraft_contract_fixtures_share_variant_compiler(archetype: str) -> None:
    """Different family vocabularies resolve through one bounded contract."""

    catalog = _archetype_variant_catalog(archetype)
    family_id = f"{archetype}_contract_fixture"
    modifier_id = "aero.lift_scale" if archetype == "fixed_wing" else "rotor.collective_authority_scale"
    resolved = resolve_case(
        _load_case("case-heavy.yaml").model_copy(
            update={
                "family": family_id,
                "variant_parameters": {modifier_id: CaseValue(value=1.1)},
            }
        ),
        catalog,
    )

    assert resolved.variant_resolution is not None
    assert resolved.variant_resolution.status == "qualified"
    expected_target = "vehicle.aero.lift_scale" if archetype == "fixed_wing" else "vehicle.rotor.collective_authority"
    assert resolved.variant_resolution.invalidations == (f"retrim:{expected_target}",)
    assert resolved.resolved_variant is not None
    assert len(resolved.resolved_variant.fingerprint) == 64
    assert resolved.parameters["vehicle.mass.wet"].value == pytest.approx(1100.0)


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
