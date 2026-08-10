"""Plug-in-owned static advertisements for exact local controller screens.

Some executable local screens use a retained, source-owned controller rather
than a campaign which the common tuner can reconstruct.  They must still be
discoverable without constructing a plant or running that controller.  This
small registry lets a vehicle plug-in publish that exact metadata beside the
common authoring plan while keeping controller execution and qualification
ownership with the plug-in.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from copy import deepcopy
from typing import Any

from pydantic import BaseModel, ConfigDict, Field, model_validator


class LocalControllerScreenAdvertisement(BaseModel):
    """One immutable plug-in advertisement for an exact controller endpoint."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    provider_id: str = Field(min_length=1)
    model_id: str = Field(min_length=1)
    family_id: str | None = None
    fidelity: str = Field(min_length=1)
    realization_id: str = Field(min_length=1)
    mission_template_id: str = Field(min_length=1)
    advertisement: Mapping[str, Any]

    @model_validator(mode="after")
    def validate_advertisement(self) -> LocalControllerScreenAdvertisement:
        """Reject incomplete or mismatched static endpoint metadata."""

        required = {
            "schema",
            "id",
            "mission_template_id",
            "fidelity",
            "operations",
            "control_realization",
            "controller",
            "claim_boundary",
        }
        missing = sorted(required - set(self.advertisement))
        if missing:
            raise ValueError(f"local controller-screen advertisement is missing: {', '.join(missing)}")
        if self.advertisement["id"] != self.id:
            raise ValueError("local controller-screen advertisement ID must match its registration")
        if self.advertisement["mission_template_id"] != self.mission_template_id:
            raise ValueError("local controller-screen advertisement mission must match its registration")
        if self.advertisement["fidelity"] != self.fidelity:
            raise ValueError("local controller-screen advertisement fidelity must match its registration")
        operations = self.advertisement["operations"]
        if not isinstance(operations, Sequence) or isinstance(operations, str) or not operations:
            raise ValueError("local controller-screen advertisement operations must be a nonempty sequence")
        if not all(isinstance(item, str) and item.strip() for item in operations):
            raise ValueError("local controller-screen advertisement operations must be nonempty strings")
        controller = self.advertisement["controller"]
        if not isinstance(controller, Mapping) or not isinstance(controller.get("method"), str):
            raise ValueError("local controller-screen advertisement requires a controller method")
        if not isinstance(self.advertisement["claim_boundary"], str) or not self.advertisement["claim_boundary"].strip():
            raise ValueError("local controller-screen advertisement requires a claim boundary")
        return self
        ####

    def matches(
        self,
        *,
        provider_id: str,
        model_id: str,
        family_id: str | None,
        fidelity: str,
        realization_id: str | None,
        mission_template_id: str | None,
    ) -> bool:
        """Return whether one authoring selection exactly names this screen."""

        return (
            self.provider_id == provider_id
            and self.model_id == model_id
            and self.family_id == family_id
            and self.fidelity == fidelity
            and self.realization_id == realization_id
            and self.mission_template_id == mission_template_id
        )
        ####

    def public_advertisement(self) -> dict[str, object]:
        """Return static JSON-safe metadata without constructing a runtime."""

        return deepcopy(dict(self.advertisement))
        ####

    ####


class LocalControllerScreenAdvertisementRegistry(BaseModel):
    """Fail-closed lookup surface for plug-in-owned local screen metadata."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    registrations: tuple[LocalControllerScreenAdvertisement, ...] = ()

    @model_validator(mode="after")
    def validate_unique_registrations(self) -> LocalControllerScreenAdvertisementRegistry:
        """Require one static screen per exact authoring endpoint."""

        identifiers = tuple(item.id for item in self.registrations)
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("local controller-screen advertisement registry has duplicate IDs")
        keys = tuple(
            (
                item.provider_id,
                item.model_id,
                item.family_id,
                item.fidelity,
                item.realization_id,
                item.mission_template_id,
            )
            for item in self.registrations
        )
        if len(keys) != len(set(keys)):
            raise ValueError("local controller-screen advertisement registry has duplicate endpoint selections")
        return self
        ####

    def matching(
        self,
        *,
        provider_id: str,
        model_id: str,
        family_id: str | None,
        fidelity: str,
        realization_id: str | None,
        mission_template_id: str | None,
    ) -> tuple[LocalControllerScreenAdvertisement, ...]:
        """Resolve a screen only for the exact selected provider endpoint."""

        return tuple(
            item
            for item in self.registrations
            if item.matches(
                provider_id=provider_id,
                model_id=model_id,
                family_id=family_id,
                fidelity=fidelity,
                realization_id=realization_id,
                mission_template_id=mission_template_id,
            )
        )
        ####

    ####


__all__ = [
    "LocalControllerScreenAdvertisement",
    "LocalControllerScreenAdvertisementRegistry",
]
