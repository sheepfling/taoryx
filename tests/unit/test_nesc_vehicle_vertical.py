"""Fast vertical Composition proof for the NESC source-replay vehicle."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, cast

import pytest
from taoryx.nesc_composition_execution import execute_nesc_source_replay_composition

from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import PluginCatalog, discover_plugins
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog

ROOT = Path(__file__).resolve().parents[2]
PROVIDER_ID = "taoryx.registry.mission-composition"
MODEL_ID = "reference_nesc_two_stage_rocket"
MISSION_ID = "staged_rocket_launch_target_state_v1"
FIDELITY = "pseudo_6dof"
COMPOSITION = "nesc_staged_source_replay_pseudo6dof_compose.yaml"


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover the installed distributions once for the focused slice."""

    return discover_plugins(include_external=False)
    ####


def _source_replay_composition(name: str = COMPOSITION):
    """Compile the documented NESC source-replay request."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / name)
    return compile_vehicle_composition(request)
    ####


def test_nesc_advertisement_builds_a_complete_source_replay_authoring_plan(
    plugins: PluginCatalog,
) -> None:
    """The plug-in advertises source data and intentionally no control campaign."""

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity=FIDELITY,
        realization_id=FIDELITY,
        mission_template_id=MISSION_ID,
    )

    assert plan["status"] == "ready_to_author"
    selection = cast(dict[str, object], plan["selection"])
    assert selection["physical_family"] == "mass_depleting_rocket"
    data_contract = cast(dict[str, object], plan["data_contract"])
    assert data_contract["properties"]
    assert data_contract["source_refs"]
    controller = cast(dict[str, object], plan["controller_automation"])
    assert controller["channels"] == []
    assert controller["campaigns"] == []
    assert cast(dict[str, object], plan["segment_automation"])["instances"]
    ####


@pytest.mark.parametrize(
    ("composition_name", "fidelity", "control_realization"),
    (
        ("nesc_staged_source_replay_3dof_compose.yaml", "point_mass_3dof", "uncontrolled_source_replay"),
        ("nesc_staged_source_replay_pseudo6dof_compose.yaml", "pseudo_6dof", "response_law"),
    ),
)
def test_nesc_composition_executes_the_advertised_source_replay(
    tmp_path: Path,
    composition_name: str,
    fidelity: str,
    control_realization: str,
) -> None:
    """Every runnable NESC fidelity writes stage truth, status, and provenance."""

    result = execute_nesc_source_replay_composition(
        _source_replay_composition(composition_name),
        tmp_path / f"nesc-source-replay-{fidelity}",
    )

    assert result.composition.family_id == MODEL_ID
    assert result.composition.fidelity == fidelity
    assert result.mission_pass is True
    assert result.preflight.status == "translation_ready"
    assert result.runtime["participating_nonlinear_plant"] is False
    assert result.runtime["control_realization"] == control_realization
    assert result.runtime["physical_gimbal_allocation"] is False
    assert result.truth_evaluation["required_passed"] == 4
    assert result.envelope["phase_order"] == ["stage1_burn", "stack_coast", "stage2_burn", "orbit_coast"]
    action_trace = cast(
        dict[str, object],
        json.loads((result.output_dir / "semantic_action_trace.json").read_text(encoding="utf-8")),
    )
    assert action_trace["requested_action_channels"] == []
    assert action_trace["achieved_effector_channels"] == []
    assert (result.output_dir / "status_trace.json").is_file()
    assert (result.output_dir / "resource_ledger.json").is_file()
    assert (result.output_dir / "source_provenance.json").is_file()
    ####


def test_nesc_public_adapters_exercise_the_exact_source_replay(
    plugins: PluginCatalog,
) -> None:
    """Both public reductions probe the pinned history through the common replay seam."""

    registry = plugins.build_family_adapter_registry()
    for tier in ("point_mass_3dof", "pseudo_6dof"):
        check = registry.check(MODEL_ID, tier)
        assert check.status == "pass"
        assert check.probe is not None
        operations = {item.operation: item.status for item in check.probe.operations}
        assert operations["replay"] == "pass"
        assert operations["state_derivative"] == "not_applicable"
        assert operations["trim"] == "not_applicable"
        assert operations["effectiveness"] == "not_applicable"
        assert operations["allocate"] == "not_applicable"
    ####


def test_nesc_catalog_advertises_the_same_batch_only_source_replay_endpoint() -> None:
    """The public catalog does not imply an undeclared interactive controller."""

    kit = load_resolved_vehicle_composition_catalog().vehicle(MODEL_ID).authoring_kit_dict(
        MISSION_ID,
        FIDELITY,
    )
    endpoints = cast(list[dict[str, Any]], kit["execution_endpoints"])

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "nesc_source_replay.v1"),
    }
    assert not any(item["operation"] == "episode" and item["status"] == "runnable" for item in endpoints)
    ####
