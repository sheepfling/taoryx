"""Fast vertical Composition proof for the passive tumbling-body vehicle."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, cast

import pytest
from taoryx.passive_tumbling_composition_execution import execute_passive_tumbling_composition

from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import PluginCatalog, discover_plugins
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request, resolve_vehicle_composition_interface_contract
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog

ROOT = Path(__file__).resolve().parents[2]
PROVIDER_ID = "taoryx.registry.mission-composition"
MODEL_ID = "tumbling_body"
MISSION_ID = "tumbling_body_release_damping_impact_v1"
FIDELITY = "pseudo_6dof"
COMPOSITION = "tumbling_body_direct_release_pseudo6dof_compose.yaml"


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover the installed distributions once for the focused slice."""

    return discover_plugins(include_external=False)
    ####


def _passive_composition(name: str = COMPOSITION):
    """Compile the documented passive rigid-body-reuse request."""

    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / name)
    return compile_vehicle_composition(request)
    ####


def test_tumbling_body_advertisement_builds_a_complete_passive_authoring_plan(
    plugins: PluginCatalog,
) -> None:
    """The plug-in advertises data and no nonexistent control or tuning surface."""

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
    assert selection["physical_family"] == "passive_ballistic_tumbling_body"
    data_contract = cast(dict[str, object], plan["data_contract"])
    assert data_contract["properties"]
    assert data_contract["source_refs"]
    controller = cast(dict[str, object], plan["controller_automation"])
    assert controller["channels"] == []
    assert controller["campaigns"] == []
    assert cast(dict[str, object], plan["segment_automation"])["instances"]
    ####


def test_tumbling_body_passive_adapters_are_public_for_both_runnable_reductions(
    plugins: PluginCatalog,
) -> None:
    """The common adapter registry names the exact passive state without control claims."""

    registry = plugins.build_family_adapter_registry()
    registration = registry.registration(MODEL_ID)

    assert registration.status == "available"
    assert registration.supported_tiers == ("point_mass_3dof", "pseudo_6dof")
    for tier in registration.supported_tiers:
        check = registry.check(MODEL_ID, tier)
        assert check.status == "pass"
        assert check.probe is None
        assert check.conformance is not None
        descriptor = registry.build(MODEL_ID, tier).describe()
        assert descriptor.control_realization == "uncontrolled"
        assert descriptor.control_channels == ()
        assert {item.name for item in descriptor.state_channels} >= {
            "position_m",
            "velocity_m_s",
            "projected_area_m2",
            "drag_force_n",
            "angular_rate_norm_rad_s",
        }
    ####


@pytest.mark.parametrize(
    ("composition_name", "fidelity", "realized_fidelity", "area_policy"),
    (
        (
            "tumbling_body_direct_release_3dof_compose.yaml",
            "point_mass_3dof",
            "point_mass_3dof",
            "orientation_averaged_projected_area",
        ),
        (
            "tumbling_body_direct_release_pseudo6dof_compose.yaml",
            "pseudo_6dof",
            "rigid_body_6dof",
            "native_rigid_body_reuse_instantaneous_projected_area",
        ),
    ),
)
def test_tumbling_body_composition_executes_without_a_hidden_controller(
    tmp_path: Path,
    composition_name: str,
    fidelity: str,
    realized_fidelity: str,
    area_policy: str,
) -> None:
    """Every runnable passive fidelity reports terminal truth without controls."""

    result = execute_passive_tumbling_composition(
        _passive_composition(composition_name),
        tmp_path / f"passive-tumbling-{fidelity}",
    )

    assert result.composition.family_id == MODEL_ID
    assert result.composition.fidelity == fidelity
    assert result.mission_pass is True
    assert result.preflight.status == "translation_ready"
    assert result.runtime["control_realization"] == "uncontrolled"
    assert result.runtime["direct_wrench_injection"] is False
    assert result.runtime["physical_effector_allocation"] is False
    assert result.envelope["realized_fidelity"] == realized_fidelity
    assert result.envelope["area_policy"] == area_policy
    assert result.truth_evaluation["required_passed"] == 3
    assert all(item["required"] is True and item["status"] == "pass" for item in result.truth_evaluation["results"])
    action_trace = cast(
        dict[str, object],
        json.loads((result.output_dir / "semantic_action_trace.json").read_text(encoding="utf-8")),
    )
    assert action_trace["requested_action_channels"] == []
    assert action_trace["achieved_effector_channels"] == []
    status_trace = cast(dict[str, object], json.loads((result.output_dir / "status_trace.json").read_text(encoding="utf-8")))
    first = cast(dict[str, object], cast(list[dict[str, object]], status_trace["samples"])[0]["values"])
    assert {"aerodynamics.drag_force", "aerodynamics.projected_area", "angular_rate.norm"} <= set(first)
    assert float(first["aerodynamics.drag_force"]) >= 0.0
    assert float(first["aerodynamics.projected_area"]) > 0.0
    assert float(first["angular_rate.norm"]) >= 0.0
    assert (result.output_dir / "resource_ledger.json").is_file()
    assert (result.output_dir / "truth_telemetry.csv").is_file()
    ####


def test_tumbling_body_catalog_advertises_the_same_batch_only_passive_endpoint() -> None:
    """The public catalog does not imply a controller or interactive fallback."""

    kit = load_resolved_vehicle_composition_catalog().vehicle(MODEL_ID).authoring_kit_dict(
        MISSION_ID,
        FIDELITY,
    )
    endpoints = cast(list[dict[str, Any]], kit["execution_endpoints"])

    assert {(item["operation"], item["factory_id"]) for item in endpoints if item["status"] == "runnable"} == {
        ("batch", "passive_tumbling_direct_release.v1"),
    }
    assert not any(item["operation"] == "episode" and item["status"] == "runnable" for item in endpoints)
    interface = resolve_vehicle_composition_interface_contract(_passive_composition())
    status = {item.id: item for item in interface.status_channels}
    assert {"aerodynamics.drag_force", "aerodynamics.projected_area", "angular_rate.norm"} <= set(status)
    assert all(status[item].availability == "available_in_batch" for item in {
        "aerodynamics.drag_force",
        "aerodynamics.projected_area",
        "angular_rate.norm",
    })
    ####
