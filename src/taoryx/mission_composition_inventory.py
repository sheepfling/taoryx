"""Authoritative Mission Composition asset inventory and drift audit."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Literal, Protocol

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .vehicle_registry import ROOT

MISSION_COMPOSITION_INVENTORY = ROOT / "verification/mission_composition_inventory.yaml"

AssetClassification = Literal[
    "family",
    "realization",
    "stage_component",
    "deployable_child",
    "generated_child",
    "debug_contract_fixture",
    "explicit_exclusion",
]
AssetDisposition = Literal[
    "published",
    "published_blocked",
    "component",
    "generated",
    "blocked",
    "debug_only",
    "excluded",
]


class MissionCompositionInventoryAsset(BaseModel):
    """One deliberate disposition for an asset visible in repository catalogs."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    classification: AssetClassification
    status: AssetDisposition
    parent_id: str | None = None
    published_model_id: str | None = None
    realization_id: str | None = None
    catalog_ids: tuple[str, ...] = ()
    exclusion_reason: str | None = None

    @model_validator(mode="after")
    def validate_disposition(self) -> MissionCompositionInventoryAsset:
        if self.classification == "family" and self.status.startswith("published") and self.published_model_id is None:
            raise ValueError(f"published family asset {self.id!r} requires published_model_id")
        if self.classification == "realization" and (self.parent_id is None or self.realization_id is None):
            raise ValueError(f"realization asset {self.id!r} requires parent_id and realization_id")
        if self.classification in {"stage_component", "deployable_child", "generated_child", "realization"} and self.parent_id is None:
            raise ValueError(f"nested asset {self.id!r} requires parent_id")
        if self.status in {"published_blocked", "blocked", "excluded", "debug_only"} and not self.exclusion_reason:
            raise ValueError(f"asset {self.id!r} requires a substantive disposition reason")
        return self
        ####

    ####


class MissionCompositionInventory(BaseModel):
    """Versioned authority covering all known vehicle/model catalog identities."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-inventory/v1"] = Field(alias="schema", serialization_alias="schema")
    version: int = Field(ge=1)
    description: str = Field(min_length=1)
    assets: tuple[MissionCompositionInventoryAsset, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_graph(self) -> MissionCompositionInventory:
        ids = tuple(item.id for item in self.assets)
        duplicates = sorted({item for item in ids if ids.count(item) > 1})
        if duplicates:
            raise ValueError(f"Mission Composition inventory has duplicate asset IDs: {duplicates!r}")
        known = set(ids)
        dangling = sorted({item.parent_id for item in self.assets if item.parent_id is not None} - known)
        if dangling:
            raise ValueError(f"Mission Composition inventory has dangling parent IDs: {dangling!r}")
        catalog_ids = tuple(item for asset in self.assets for item in asset.catalog_ids)
        repeated = sorted({item for item in catalog_ids if catalog_ids.count(item) > 1})
        if repeated:
            raise ValueError(f"Mission Composition inventory assigns catalog identities more than once: {repeated!r}")
        return self
        ####

    def asset(self, asset_id: str) -> MissionCompositionInventoryAsset:
        try:
            return next(item for item in self.assets if item.id == asset_id)
        except StopIteration as error:
            raise KeyError(f"unknown Mission Composition inventory asset {asset_id!r}") from error
        ####

    ####


class _DiscoveryProvider(Protocol):
    def list_models(self) -> tuple[object, ...]: ...


class MissionCompositionInventoryAudit(BaseModel):
    """Serializable inventory/catalog/provider reconciliation report."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-inventory-audit/v1"] = Field(
        default="taoryx.mission-composition-inventory-audit/v1",
        alias="schema",
        serialization_alias="schema",
    )
    status: Literal["pass", "fail"]
    asset_count: int = Field(ge=0)
    source_identity_count: int = Field(ge=0)
    published_model_count: int = Field(ge=0)
    debug_model_count: int = Field(default=0, ge=0)
    errors: tuple[str, ...] = ()


def load_mission_composition_inventory(path: str | Path | None = None) -> MissionCompositionInventory:
    """Load the one checked-in asset disposition authority."""

    source = Path(path) if path is not None else MISSION_COMPOSITION_INVENTORY
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{source} must contain a mapping")
    return MissionCompositionInventory.model_validate(payload)
    ####


def audit_mission_composition_inventory(
    inventory: MissionCompositionInventory | None = None,
    *,
    provider: _DiscoveryProvider | None = None,
    debug_providers: tuple[_DiscoveryProvider, ...] = (),
) -> MissionCompositionInventoryAudit:
    """Reconcile every known source catalog identity and production model."""

    selected = inventory or load_mission_composition_inventory()
    errors: list[str] = []
    expected_catalog_ids = _source_catalog_identities()
    inventoried_catalog_ids = {item for asset in selected.assets for item in asset.catalog_ids}
    missing = sorted(expected_catalog_ids - inventoried_catalog_ids)
    stale = sorted(inventoried_catalog_ids - expected_catalog_ids)
    if missing:
        errors.append(f"source catalog identities are not inventoried: {missing!r}")
    if stale:
        errors.append(f"inventory references unknown source catalog identities: {stale!r}")

    published_ids = {
        item.published_model_id
        for item in selected.assets
        if item.published_model_id is not None and item.status in {"published", "published_blocked"}
    }
    inventory_ids = {item.id for item in selected.assets}
    if provider is not None:
        models = provider.list_models()
        provider_ids = {str(getattr(item, "id")) for item in models}
        missing_publications = sorted(published_ids - provider_ids)
        unexplained_publications = sorted(provider_ids - published_ids)
        if missing_publications:
            errors.append(f"inventoried published models are absent from discovery: {missing_publications!r}")
        if unexplained_publications:
            errors.append(f"discovery models have no production inventory disposition: {unexplained_publications!r}")
        debug_ids = {
            item.published_model_id or item.id
            for item in selected.assets
            if item.classification == "debug_contract_fixture"
        }
        leaked_debug = sorted(provider_ids & debug_ids)
        if leaked_debug:
            errors.append(f"debug/contract fixtures leaked into production discovery: {leaked_debug!r}")

        model_by_id = {str(getattr(item, "id")): item for item in models}
        inventoried_realizations = {
            (item.published_model_id, item.realization_id): item
            for item in selected.assets
            if item.classification == "realization"
            and item.published_model_id is not None
            and item.realization_id is not None
        }
        for model_id, model in model_by_id.items():
            realization_by_id = {
                str(getattr(realization, "id")): realization
                for realization in getattr(model, "realizations", ())
            }
            fidelity_ids = {str(getattr(fidelity, "id")) for fidelity in getattr(model, "fidelities", ())}
            for realization_id, realization in realization_by_id.items():
                if realization_id in fidelity_ids:
                    continue
                inventory_record = inventoried_realizations.get((model_id, realization_id))
                if inventory_record is None:
                    errors.append(
                        f"model {model_id!r} realization {realization_id!r} has no explicit inventory disposition"
                    )
                    continue
                realization_status = str(getattr(realization, "status"))
                if inventory_record.status == "published" and realization_status != "available":
                    errors.append(
                        f"inventory publishes {model_id!r}/{realization_id!r}, but discovery marks it {realization_status!r}"
                    )
                if inventory_record.status == "published_blocked" and realization_status == "available":
                    errors.append(
                        f"inventory blocks {model_id!r}/{realization_id!r}, but discovery marks it available"
                    )
            for (published_model_id, realization_id), inventory_record in inventoried_realizations.items():
                if published_model_id != model_id:
                    continue
                if realization_id not in realization_by_id:
                    errors.append(
                        f"inventory realization {inventory_record.id!r} is absent from model {model_id!r} discovery"
                    )
            for deployment in getattr(model, "deployments", ()):
                child_id = getattr(deployment, "child_model_id", None)
                child_scope = getattr(deployment, "child_model_scope", None)
                if child_id is not None and child_scope != "external" and child_id not in inventory_ids and child_id not in provider_ids:
                    errors.append(f"model {model_id!r} deployment has dangling child model ID {child_id!r}")

    expected_debug_ids = {
        item.published_model_id
        for item in selected.assets
        if item.classification == "debug_contract_fixture"
        and item.status == "debug_only"
        and item.published_model_id is not None
    }
    discovered_debug_ids = {
        str(getattr(model, "id"))
        for debug_provider in debug_providers
        for model in debug_provider.list_models()
    }
    if debug_providers:
        missing_debug_publications = sorted(expected_debug_ids - discovered_debug_ids)
        unexplained_debug_publications = sorted(discovered_debug_ids - expected_debug_ids)
        if missing_debug_publications:
            errors.append(f"inventoried debug models are absent from debug discovery: {missing_debug_publications!r}")
        if unexplained_debug_publications:
            errors.append(f"debug discovery models have no inventory disposition: {unexplained_debug_publications!r}")
        for debug_provider in debug_providers:
            debug_model_ids = {str(getattr(model, "id")) for model in debug_provider.list_models()}
            for model in debug_provider.list_models():
                model_id = str(getattr(model, "id"))
                for deployment in getattr(model, "deployments", ()):
                    child_id = getattr(deployment, "child_model_id", None)
                    child_scope = getattr(deployment, "child_model_scope", None)
                    if (
                        child_id is not None
                        and child_scope != "external"
                        and child_id not in inventory_ids
                        and child_id not in debug_model_ids
                    ):
                        errors.append(
                            f"debug model {model_id!r} deployment has dangling child model ID {child_id!r}"
                        )

    return MissionCompositionInventoryAudit(
        status="pass" if not errors else "fail",
        asset_count=len(selected.assets),
        source_identity_count=len(expected_catalog_ids),
        published_model_count=len(published_ids),
        debug_model_count=len(expected_debug_ids),
        errors=tuple(errors),
    )
    ####


def _source_catalog_identities() -> set[str]:
    """Read stable identities from catalogs that can imply a vehicle/model asset."""

    result: set[str] = set()
    _collect_mapping_keys(result, "verification/vehicle_models.yaml", "vehicles")
    _collect_list_ids(result, "verification/vehicle_catalog.yaml", "vehicles", "id")
    _collect_list_ids(result, "verification/horizontal_fidelity_registry.yaml", "families", "family_id")
    _collect_list_ids(result, "verification/vehicle_composition_registry.yaml", "vehicles", "family_id")
    _collect_list_ids(result, "verification/alpha2_family_catalog.yaml", "families", "family_id")
    _collect_list_ids(result, "verification/daveml_family_layer_dispositions.yaml", "families", "family_id")
    _collect_list_ids(result, "verification/reference_model_registry.yaml", "models", "id")

    catalog_path = ROOT / "verification/reference_family_catalog.yaml"
    payload = yaml.safe_load(catalog_path.read_text(encoding="utf-8"))
    manifests = payload.get("families") if isinstance(payload, Mapping) else None
    if not isinstance(manifests, list):
        raise ValueError(f"{catalog_path} has no manifests list")
    for relative in manifests:
        manifest_path = (catalog_path.parent / str(relative)).resolve()
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
        if not isinstance(manifest, Mapping) or not isinstance(manifest.get("family_id"), str):
            raise ValueError(f"{manifest_path} has no family_id")
        result.add(f"verification/reference_family_catalog.yaml#{manifest['family_id']}")
    return result
    ####


def _collect_mapping_keys(result: set[str], relative_path: str, collection: str) -> None:
    payload = _yaml_mapping(relative_path)
    records = payload.get(collection)
    if not isinstance(records, Mapping):
        raise ValueError(f"{relative_path} has no {collection!r} mapping")
    result.update(f"{relative_path}#{identifier}" for identifier in records)
    ####


def _collect_list_ids(result: set[str], relative_path: str, collection: str, id_field: str) -> None:
    payload = _yaml_mapping(relative_path)
    records = payload.get(collection)
    if not isinstance(records, list):
        raise ValueError(f"{relative_path} has no {collection!r} list")
    for record in records:
        if not isinstance(record, Mapping) or not isinstance(record.get(id_field), str):
            raise ValueError(f"{relative_path} contains an invalid {collection!r} record")
        result.add(f"{relative_path}#{record[id_field]}")
    ####


def _yaml_mapping(relative_path: str) -> Mapping[str, object]:
    source = ROOT / relative_path
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{source} must contain a mapping")
    return payload
    ####


__all__ = [
    "MISSION_COMPOSITION_INVENTORY",
    "MissionCompositionInventory",
    "MissionCompositionInventoryAsset",
    "MissionCompositionInventoryAudit",
    "audit_mission_composition_inventory",
    "load_mission_composition_inventory",
]
