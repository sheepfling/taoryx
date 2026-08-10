"""Fast coverage contract between the vehicle catalogue and vertical slices."""

from __future__ import annotations

from pathlib import Path
from typing import cast

from taoryx.trajectory.registry_mission_composition import RegistryMissionCompositionProvider

import tools.dev as dev
from taoryx.plugins import discover_plugins
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog

ROOT = Path(__file__).resolve().parents[2]


def test_every_registered_controller_campaign_has_one_runnable_vehicle_slice() -> None:
    """Every runnable vehicle, workflow, and controller campaign has a focused proof."""

    registry = discover_plugins(include_external=False).build_controller_tuning_campaign_registry()
    campaign_families = {registration.family_id for registration in registry.registrations}
    catalog = load_resolved_vehicle_composition_catalog()
    runnable_vehicle_families = {
        vehicle.family.family_id
        for vehicle in catalog.vehicles
        if any(
            cast(dict[str, object], tier["operational_maturity"])["endpoint_maturity"] in {"batch_and_episode_ready", "batch_ready", "episode_ready"}
            for mission in cast(list[dict[str, object]], vehicle.authoring_worklist_dict()["missions"])
            for tier in cast(list[dict[str, object]], mission["tiers"])
        )
    }
    runnable_workflow_models = {
        model.id
        for model in RegistryMissionCompositionProvider().list_models()
        if model.model_kind != "canonical_vehicle_family" and "batch" in model.common_runner_operations
    }

    assert campaign_families <= set(dev.VEHICLE_VERTICAL_TEST_PATHS)
    assert set(dev.VEHICLE_VERTICAL_TEST_PATHS) == runnable_vehicle_families | runnable_workflow_models
    for family_id, paths in dev.VEHICLE_VERTICAL_TEST_PATHS.items():
        assert paths
        assert all((ROOT / path).is_file() for path in paths), family_id
    ####


def test_every_advertised_vehicle_tier_reports_explicit_operational_maturity() -> None:
    """A declared tier must distinguish endpoint readiness from qualification status."""

    catalog = load_resolved_vehicle_composition_catalog()
    for vehicle in catalog.vehicles:
        worklist = vehicle.authoring_worklist_dict()
        missions = cast(list[dict[str, object]], worklist["missions"])
        for mission in missions:
            tiers = cast(list[dict[str, object]], mission["tiers"])
            for tier in tiers:
                maturity = cast(dict[str, object], tier["operational_maturity"])
                assert maturity["endpoint_maturity"] in {
                    "batch_and_episode_ready",
                    "batch_ready",
                    "episode_ready",
                    "declared_execution_gap",
                    "unbound",
                }
                assert maturity["scope"] in {"mission", "local_controller_screen"}
                assert maturity["interface_status"] in {"pass", None}
    ####
