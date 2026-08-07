"""Typed discovery catalog for the canonical Product 2 scenarios."""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from taoryx.product_two_contracts import ProductTwoStatus
from taoryx.product_two_quality import ProductTwoQualitySpec

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PRODUCT_TWO_CATALOG = ROOT / "verification/product_two_scenario_catalog.yaml"

ProductTwoEntrypoint = Literal["source", "composition", "showcase", "interactive"]


class ProductTwoCatalogInput(BaseModel):
    """One explicit source or model input named by a catalog entry."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    path: str = Field(min_length=1)
    role: str = Field(min_length=1)
    kind: Literal["problem", "table", "composition", "script", "showcase", "model"]
    ####


class ProductTwoScenario(BaseModel):
    """Discoverable Product 2 scenario and its reproducible command shape."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(pattern=r"^[a-z0-9][a-z0-9-]*$")
    aliases: tuple[str, ...] = ()
    title: str = Field(min_length=1)
    description: str = Field(min_length=1)
    entrypoint: ProductTwoEntrypoint
    family: str = Field(min_length=1)
    fidelity: str = Field(min_length=1)
    realization: str = Field(min_length=1)
    operation: str = Field(min_length=1)
    expected_disposition: ProductTwoStatus
    profile: str = "taos96"
    inputs: tuple[ProductTwoCatalogInput, ...] = Field(min_length=1)
    setup_commands: tuple[tuple[str, ...], ...] = ()
    run_command: tuple[str, ...] = Field(min_length=1)
    artifact_paths: tuple[str, ...] = ()
    integrator: str | None = None
    seed: int | None = None
    requested_duration_s: float | None = Field(default=None, gt=0.0)
    claim_boundary: str = Field(min_length=1)
    quality: ProductTwoQualitySpec
    tags: tuple[str, ...] = ()

    @model_validator(mode="after")
    def require_pseudo6dof_declarations(self) -> ProductTwoScenario:
        if "pseudo_6dof" in self.fidelity.casefold():
            contract = self.quality.fidelity
            missing = {
                name
                for name, value in (
                    ("response_law", contract.response_law),
                    ("omitted_physics", contract.omitted_physics),
                    ("controls", contract.controls),
                    ("envelope", contract.envelope),
                    ("operation_availability", contract.operation_availability),
                )
                if not value
            }
            if missing:
                raise ValueError(f"pseudo-6DOF scenario {self.id!r} is missing fidelity declarations: {', '.join(sorted(missing))}")
        return self
        ####

    def matches(self, query: str) -> bool:
        """Return whether a case-insensitive query matches discovery text."""

        needle = query.casefold().strip()
        if not needle:
            return True
        haystack = " ".join(
            (
                self.id,
                *self.aliases,
                self.title,
                self.description,
                self.family,
                self.fidelity,
                self.realization,
                self.operation,
                *self.tags,
            )
        ).casefold()
        return needle in haystack
        ####

    def input_path(self, item: ProductTwoCatalogInput, *, root: Path = ROOT) -> Path:
        """Resolve one repository-relative catalog input."""

        path = Path(item.path)
        return path if path.is_absolute() else root / path
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the stable JSON discovery projection."""

        return self.model_dump(mode="json")
        ####


class ProductTwoScenarioCatalog(BaseModel):
    """Versioned Product 2 scenario catalog."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(
        default="taoryx.product-two-scenario-catalog/v1alpha1",
        alias="schema",
        serialization_alias="schema",
    )
    schema_version: int = 1
    scenarios: tuple[ProductTwoScenario, ...] = Field(min_length=1)

    def model_post_init(self, __context: object) -> None:
        identifiers = [scenario.id for scenario in self.scenarios]
        aliases = [alias for scenario in self.scenarios for alias in scenario.aliases]
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("Product 2 catalog contains duplicate scenario IDs")
        if len(aliases) != len(set(aliases)):
            raise ValueError("Product 2 catalog contains duplicate scenario aliases")
        if set(identifiers) & set(aliases):
            raise ValueError("Product 2 catalog aliases must not shadow scenario IDs")
        ####

    def find(self, identifier: str) -> ProductTwoScenario:
        """Resolve a stable ID or documented alias."""

        needle = identifier.casefold()
        for scenario in self.scenarios:
            if scenario.id == needle or needle in {alias.casefold() for alias in scenario.aliases}:
                return scenario
        raise KeyError(f"unknown Product 2 scenario {identifier!r}")
        ####

    def search(self, query: str) -> tuple[ProductTwoScenario, ...]:
        """Return catalog entries in stable catalog order."""

        return tuple(scenario for scenario in self.scenarios if scenario.matches(query))
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the complete machine-readable catalog."""

        return self.model_dump(mode="json", by_alias=True)
        ####


def load_product_two_catalog(path: str | Path = DEFAULT_PRODUCT_TWO_CATALOG) -> ProductTwoScenarioCatalog:
    """Load and validate the checked-in Product 2 scenario catalog."""

    source = Path(path)
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"Product 2 catalog must contain a mapping: {source}")
    return ProductTwoScenarioCatalog.model_validate(payload)
    ####


__all__ = [
    "DEFAULT_PRODUCT_TWO_CATALOG",
    "ProductTwoCatalogInput",
    "ProductTwoScenario",
    "ProductTwoScenarioCatalog",
    "load_product_two_catalog",
]
####
