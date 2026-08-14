"""Selected-data boundary for the shared X8/B747 source-table plug-in."""

from __future__ import annotations

import pytest
from taoryx_source_table_fixed_wing.resources import model_resource_root

from taoryx.plugins import VehicleCatalogFragment, discover_plugins, plugin_catalog_scope
from taoryx.vehicle_catalog_resources import vehicle_catalog_resources
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog
from taoryx.vehicle_execution_bindings import load_vehicle_execution_binding_catalog
from taoryx.vehicle_execution_parity_witnesses import load_vehicle_execution_parity_witness_catalog
from taoryx.vehicle_execution_witnesses import load_vehicle_execution_witness_catalog

PACKAGE_ID = "taoryx.source-table-fixed-wing"
FAMILY_IDS = frozenset({"skywalker_x8", "b747"})
_EXECUTION_WITNESS_IDS = frozenset(
    {
        "x8-3dof-batch",
        "x8-pseudo6dof-batch",
        "x8-3dof-episode",
        "x8-pseudo6dof-episode",
        "x8-direct-wrench-batch",
        "x8-local-physical-surface-lqr-screen-batch",
        "x8-local-physical-surface-lqi-screen-batch",
        "x8-local-physical-surface-lqi-long-recovery-screen-batch",
        "b747-condition3-local-physical-surface-lqr-screen-batch",
        "b747-condition3-local-physical-surface-lqi-screen-batch",
        "b747-3dof-batch",
        "b747-pseudo6dof-batch",
        "b747-3dof-episode",
        "b747-pseudo6dof-episode",
        "b747-direct-wrench-batch",
    }
)
_PARITY_WITNESS_IDS = frozenset(
    {
        "x8-3dof-native-bridge",
        "x8-pseudo6dof-native-bridge",
        "b747-3dof-native-bridge",
        "b747-pseudo6dof-native-bridge",
    }
)


def test_selected_shared_catalog_exposes_only_its_two_owned_families() -> None:
    """A selected shared package may expose its pair, never sibling data."""

    plugins = discover_plugins(include_external=False, selected=(PACKAGE_ID,))
    contribution = plugins.contribution(
        "vehicle_catalog_fragment",
        "taoryx.source-table-fixed-wing.vehicle-catalog",
    )
    fragment = contribution.value
    assert isinstance(fragment, VehicleCatalogFragment)
    assert fragment.resource_package == "taoryx_source_table_fixed_wing"
    assert fragment.resource_root == "data"
    assert fragment.family_ids == ("skywalker_x8", "b747")

    resources = vehicle_catalog_resources("verification/vehicle_composition_registry.yaml", plugins=plugins)
    assert resources == (model_resource_root() / "verification/vehicle_composition_registry.yaml",)
    assert all("taoryx-reference-models" not in source.parts for source in resources)
    assert {item.family.family_id for item in load_resolved_vehicle_composition_catalog(plugins=plugins).vehicles} == FAMILY_IDS

    parity_resources = vehicle_catalog_resources("verification/vehicle_execution_parity_witnesses.yaml", plugins=plugins)
    assert parity_resources == (model_resource_root() / "verification/vehicle_execution_parity_witnesses.yaml",)
    witnesses = load_vehicle_execution_witness_catalog(plugins=plugins)
    assert {item.id for item in witnesses.witnesses} == _EXECUTION_WITNESS_IDS
    assert witnesses.variant_witnesses == ()
    assert witnesses.graph_extension_witnesses == ()
    assert {item.id for item in load_vehicle_execution_parity_witness_catalog(plugins=plugins).witnesses} == _PARITY_WITNESS_IDS

    with plugin_catalog_scope(plugins):
        assert vehicle_catalog_resources("verification/vehicle_composition_registry.yaml") == resources
        assert {binding.family_id for binding in load_vehicle_execution_binding_catalog().bindings} == FAMILY_IDS
        assert {item.id for item in load_vehicle_execution_witness_catalog().witnesses} == _EXECUTION_WITNESS_IDS
        assert {item.id for item in load_vehicle_execution_parity_witness_catalog().witnesses} == _PARITY_WITNESS_IDS

    with pytest.raises(FileNotFoundError, match="do not own resource"):
        vehicle_catalog_resources("verification/f16_runtime_linearization_evidence.json", plugins=plugins)
    ####
