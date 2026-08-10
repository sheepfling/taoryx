"""Typed host contract for optional reachability workbench providers."""

from __future__ import annotations

from typing import Protocol

from pydantic import BaseModel, ConfigDict, Field


class ReachabilityProviderMetadata(BaseModel):
    """Immutable discovery metadata for one reachability workbench."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(pattern=r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$")
    version: str = Field(min_length=1)
    description: str = Field(min_length=1)
    families: tuple[str, ...] = ()
    profiles: tuple[str, ...] = ()
    operations: tuple[str, ...] = ()
    claim_boundary: str = Field(min_length=1)

    def public_dict(self) -> dict[str, object]:
        """Return deterministic JSON-safe provider metadata."""

        return self.model_dump(mode="json")
        ####

    ####


class ReachabilityProvider(Protocol):
    """Optional implementation layered over core and installed model packages."""

    @property
    def metadata(self) -> ReachabilityProviderMetadata:
        """Return lightweight provider identity and supported operations."""

        ...

    def run_cli(self, arguments: object) -> int:
        """Handle a parsed reachability CLI command."""

        ...


class ReachabilityProviderRegistry:
    """Immutable exact-ID registry for installed reachability providers."""

    def __init__(self, providers: tuple[ReachabilityProvider, ...] = ()) -> None:
        identifiers = tuple(provider.metadata.id for provider in providers)
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("reachability provider registry contains duplicate provider IDs")
        self._providers = providers
        self._by_id = {provider.metadata.id: provider for provider in providers}
        ####

    @property
    def providers(self) -> tuple[ReachabilityProvider, ...]:
        """Return providers in plug-in registration order."""

        return self._providers
        ####

    @property
    def ids(self) -> tuple[str, ...]:
        """Return stable registered provider identities."""

        return tuple(self._by_id)
        ####

    def provider(self, identifier: str) -> ReachabilityProvider:
        """Resolve one exact provider without a fallback implementation."""

        try:
            return self._by_id[identifier]
        except KeyError as error:
            raise KeyError(f"unknown reachability provider {identifier!r}") from error
        ####

    def public_dict(self) -> dict[str, object]:
        """Describe installed providers without serializing executable code."""

        return {
            "schema": "taoryx.reachability-provider-registry/v1",
            "providers": [provider.metadata.public_dict() for provider in self._providers],
        }
        ####

    ####


__all__ = [
    "ReachabilityProvider",
    "ReachabilityProviderMetadata",
    "ReachabilityProviderRegistry",
]
####
