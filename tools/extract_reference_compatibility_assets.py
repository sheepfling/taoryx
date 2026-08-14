"""Materialize the reference-model compatibility catalog without split families.

The repository-root verification files remain the canonical migration source.
A320, F-16, X-15, HL-20, Hummingbird, NESC, passive bodies, Simple Aero, the
Dual Launch workflow, the development-only analytical/debug providers, and the
shared source-table X8/B747 package own their rows in independently
installable plug-ins. This tool removes those rows from the
reference-model wheel's packaged mirror so the aggregate provider consumes
installed fragments.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "packages/taoryx-reference-models/src/taoryx_reference_models/data/verification"
FAMILY_IDS = frozenset(
    {
        "a320_openap_3dof",
        "b747",
        "f16_s119",
        "hummingbird",
        "hl20_mod_k",
        "reference_nesc_two_stage_rocket",
        "tumbling_body",
        "x15",
        "skywalker_x8",
    }
)
WORKFLOW_MATURITY_RECORD_IDS = frozenset({"dual_launch_glider"})


def _read(relative: str) -> dict[str, Any]:
    """Read a canonical YAML catalog as a mutable mapping."""

    payload = yaml.safe_load((ROOT / relative).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{relative} must contain a mapping")
    return payload
    ####


def _write(relative: str, payload: dict[str, Any]) -> None:
    """Write the deterministic compatibility fragment under package data."""

    path = TARGET / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
    ####


def _write_json(relative: str, payload: dict[str, Any]) -> None:
    """Write one deterministic JSON compatibility fragment."""

    path = TARGET.parent / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _without_family_rows(payload: dict[str, Any], key: str) -> dict[str, Any]:
    """Filter one list catalog by the standard composition-family identity."""

    rows = payload.get(key)
    if not isinstance(rows, list):
        raise ValueError(f"{key} must contain a list")
    result = dict(payload)
    result[key] = [row for row in rows if not isinstance(row, dict) or row.get("family_id") not in FAMILY_IDS]
    return result
    ####


def extract() -> None:
    """Write reference-owned catalog fragments after every extracted family split."""

    vehicle_models = _read("verification/vehicle_models.yaml")
    vehicles = vehicle_models.get("vehicles")
    if not isinstance(vehicles, dict) or "hummingbird" not in vehicles:
        raise ValueError("canonical vehicle registry is missing the Hummingbird split-family record")
    vehicle_models["vehicles"] = {identifier: value for identifier, value in vehicles.items() if identifier not in FAMILY_IDS}
    _write("vehicle_models.yaml", vehicle_models)

    for relative, key in (
        ("vehicle_composition_registry.yaml", "vehicles"),
        ("horizontal_fidelity_registry.yaml", "families"),
        ("vehicle_execution_bindings.yaml", "bindings"),
        ("vehicle_execution_parity.yaml", "bindings"),
        ("vehicle_endpoint_specs.yaml", "endpoints"),
    ):
        _write(relative, _without_family_rows(_read(f"verification/{relative}"), key))

    pseudo = _read("verification/pseudo6dof_profiles.yaml")
    for key in ("profiles", "direct_wrench_profiles", "surface_allocation_profiles", "bindings"):
        pseudo = _without_family_rows(pseudo, key)
    _write("pseudo6dof_profiles.yaml", pseudo)

    maturity = _read("verification/vehicle_maturity_registry.yaml")
    records = maturity.get("records")
    if not isinstance(records, list):
        raise ValueError("vehicle maturity registry must contain records")
    maturity["records"] = [
        row
        for row in records
        if not isinstance(row, dict) or (row.get("composition_family_id") not in FAMILY_IDS and row.get("id") not in WORKFLOW_MATURITY_RECORD_IDS)
    ]
    _write("vehicle_maturity_registry.yaml", maturity)

    lqr = _read("verification/lqr_scaling_profiles.yaml")
    profiles = lqr.get("profiles")
    if not isinstance(profiles, dict):
        raise ValueError("LQR scaling profiles must contain a profiles mapping")
    lqr["profiles"] = {identifier: row for identifier, row in profiles.items() if not isinstance(row, dict) or row.get("vehicle") not in FAMILY_IDS}
    _write("lqr_scaling_profiles.yaml", lqr)

    _write_powered_fixed_wing_mission_profiles()
    _remove_compatibility_workflow_endpoints()
    _remove_extracted_family_assets()
    ####


def _write_powered_fixed_wing_mission_profiles() -> None:
    """Remove focused-family planning profiles from the compatibility package."""

    profile = _read("verification/powered_fixed_wing_mission_profiles.yaml")
    profiles = profile.get("profiles")
    baselines = profile.get("baselines")
    if not isinstance(profiles, dict) or not isinstance(baselines, dict):
        raise ValueError("powered fixed-wing mission profiles must contain profiles and baselines mappings")
    focused_profiles = {"a320-cruise", "b747-cruise", "f16-subsonic", "x8-cruise"}
    profile["profiles"] = {identifier: value for identifier, value in profiles.items() if identifier not in focused_profiles}
    profile["baselines"] = {identifier: value for identifier, value in baselines.items() if identifier not in focused_profiles}
    _write("powered_fixed_wing_mission_profiles.yaml", profile)
    ####


def _remove_compatibility_workflow_endpoints() -> None:
    """Remove endpoint witnesses that now belong to focused workflow packages."""

    path = TARGET / "mission_workflow_endpoint_specs.yaml"
    if path.is_file():
        path.unlink()
    ####


def _remove_extracted_family_assets() -> None:
    """Remove exact generated source assets that now belong to family packages."""

    example = TARGET.parent / "examples/vehicle_composition/nesc_staged_source_replay_with_passive_child_pseudo6dof_compose.yaml"
    if example.is_file():
        example.unlink()
    family = TARGET.parent / "families/reference_nesc_two_stage_rocket"
    if family.is_dir():
        shutil.rmtree(family)
    reduction = TARGET / "daveml_nesc_reduction_qualification.json"
    if reduction.is_file():
        reduction.unlink()
    simple_aero_witness = TARGET / "workflow_endpoint_witnesses/simple_aero_fixed_ld_baseline.yaml"
    if simple_aero_witness.is_file():
        simple_aero_witness.unlink()
    dual_launch_witness = TARGET / "workflow_endpoint_witnesses/dual_launch_attached_booster.yaml"
    if dual_launch_witness.is_file():
        dual_launch_witness.unlink()
    for witness in (
        "reference_ballistic_3dof.yaml",
        "reference_waypoint_3dof.yaml",
        "debug_contract_probe.yaml",
    ):
        path = TARGET / "workflow_endpoint_witnesses" / witness
        if path.is_file():
            path.unlink()
    f16_family = TARGET.parent / "families/reference_f16_s119"
    if f16_family.is_dir():
        shutil.rmtree(f16_family)
    f16_linearization = TARGET / "f16_runtime_linearization_evidence.json"
    if f16_linearization.is_file():
        f16_linearization.unlink()
    for family_id in ("a320_openap_3dof", "a320_openap_jsbsim_pseudo6dof", "reference_a320"):
        family = TARGET.parent / "families" / family_id
        if family.is_dir():
            shutil.rmtree(family)
    for evidence in ("daveml_a320_openap_integration.json", "daveml_a320_matched_comparison.json"):
        path = TARGET / evidence
        if path.is_file():
            path.unlink()
    source_table_programs = (
        "b747_canonical_source_anchor_6dof.prb",
        "b747_condition3_surface_trim_6dof.prb",
        "skywalker_x8_canonical_6dof.prb",
        "skywalker_x8_controller_recovery_10_6dof.prb",
        "skywalker_x8_source_trim_reduction_3dof.prb",
        "skywalker_x8_table_coordinate_trim_6dof.prb",
    )
    for program in source_table_programs:
        path = TARGET.parent / "examples/generated/vehicles" / program
        if path.is_file():
            path.unlink()
    source_table_bundle = TARGET.parent / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1"
    if source_table_bundle.is_dir():
        shutil.rmtree(source_table_bundle)
    hl20_family = TARGET.parent / "families/reference_hl20_mod_k"
    if hl20_family.is_dir():
        shutil.rmtree(hl20_family)
    hl20_trim_evidence = TARGET / "daveml_hl20_trim_evidence.json"
    if hl20_trim_evidence.is_file():
        hl20_trim_evidence.unlink()
    x15_example = TARGET.parent / "examples/generated/vehicles/x15_canonical_research_6dof.prb"
    if x15_example.is_file():
        x15_example.unlink()
    for showcase in ("x15_6dof_research", "x15_rocket_to_hawaii"):
        directory = TARGET.parent / "examples/showcases" / showcase
        if directory.is_dir():
            shutil.rmtree(directory)
    x15_fixture = TARGET.parent / "tests/fixtures/x15_coherent_6dof_public_research_v1"
    if x15_fixture.is_dir():
        shutil.rmtree(x15_fixture)
    composition_directory = TARGET.parent / "examples/vehicle_composition"
    if composition_directory.is_dir():
        for composition in composition_directory.glob("x15_*.yaml"):
            composition.unlink()
    x15_assets = (
        TARGET / "alpha3_cross_fidelity/x15.json",
        TARGET / "alpha3_x15_direct_wrench",
        TARGET / "alpha3_x15_r1",
        TARGET / "generated/x15_physical_lqr_readiness.json",
        TARGET / "x15_fidelity_ladder",
        TARGET / "x15_maneuver_catalog.yaml",
        TARGET / "x15_trim_adjudication.yaml",
    )
    for asset in x15_assets:
        if asset.is_dir():
            shutil.rmtree(asset)
        elif asset.is_file():
            asset.unlink()
    ####


if __name__ == "__main__":
    extract()
