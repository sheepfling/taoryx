"""Source-bundle loading for a CADAC case and its referenced decks."""

from __future__ import annotations

import hashlib
from pathlib import Path

from pydantic import Field, computed_field, model_validator

from .deck import CadacDeck, parse_cadac_deck_file
from .input_ast import CadacDeckKind, CadacInputCase, CadacModel, source_name_for
from .input_parser import parse_cadac_input_file


class CadacSourceArtifact(CadacModel):
    """Cryptographic identity of one source artifact consumed by a CADAC run."""

    resolved_path: str = Field(min_length=1)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    byte_size: int = Field(ge=0)


####


class CadacDeckBinding(CadacModel):
    """One vehicle-to-deck binding resolved from the source case."""

    vehicle_model: str = Field(min_length=1)
    vehicle_role: str = Field(min_length=1)
    vehicle_source_line: int = Field(ge=1)
    kind: CadacDeckKind
    keyword: str = Field(min_length=1)
    reference: str = Field(min_length=1)
    resolved_path: str = Field(min_length=1)
    source_line: int = Field(ge=1)
    artifact: CadacSourceArtifact
    deck: CadacDeck


####


class CadacSourceBundle(CadacModel):
    """One parsed case plus all vehicle-local deck resources and source identities."""

    source_root: str = Field(min_length=1)
    input_artifact: CadacSourceArtifact
    case: CadacInputCase
    deck_bindings: tuple[CadacDeckBinding, ...]

    @computed_field
    @property
    def artifacts(self) -> tuple[CadacSourceArtifact, ...]:
        """Return input plus unique source decks in deterministic order."""

        seen = {self.input_artifact.resolved_path}
        result = [self.input_artifact]
        for binding in self.deck_bindings:
            if binding.artifact.resolved_path not in seen:
                result.append(binding.artifact)
                seen.add(binding.artifact.resolved_path)
            ####
        ####
        return tuple(result)

    ####

    def decks_for(
        self,
        vehicle_model: str,
        *,
        vehicle_role: str | None = None,
    ) -> tuple[CadacDeckBinding, ...]:
        """Return source-ordered deck bindings for one model and optional role."""

        key = vehicle_model.casefold()
        role_key = vehicle_role.casefold() if vehicle_role is not None else None
        return tuple(
            binding
            for binding in self.deck_bindings
            if binding.vehicle_model.casefold() == key and (role_key is None or binding.vehicle_role.casefold() == role_key)
        )

    ####

    def deck_for(
        self,
        vehicle_model: str,
        kind: CadacDeckKind,
        *,
        vehicle_role: str | None = None,
    ) -> CadacDeck:
        """Return one uniquely bound deck for a vehicle selector and deck kind."""

        matches = tuple(binding.deck for binding in self.decks_for(vehicle_model, vehicle_role=vehicle_role) if binding.kind is kind)
        if len(matches) != 1:
            raise KeyError(f"expected one {kind.value} binding for {vehicle_model!r}, found {len(matches)}")
        ####
        return matches[0]

    ####

    def deck_binding_for(
        self,
        vehicle_model: str,
        kind: CadacDeckKind,
        *,
        vehicle_role: str | None = None,
    ) -> CadacDeckBinding:
        """Return one uniquely bound deck binding including its fingerprint."""

        matches = tuple(binding for binding in self.decks_for(vehicle_model, vehicle_role=vehicle_role) if binding.kind is kind)
        if len(matches) != 1:
            raise KeyError(f"expected one {kind.value} binding for {vehicle_model!r}, found {len(matches)}")
        ####
        return matches[0]

    ####

    @model_validator(mode="after")
    def validate_unique_vehicle_bindings(self) -> "CadacSourceBundle":
        identities = [(binding.vehicle_source_line, binding.keyword.casefold(), binding.reference.casefold()) for binding in self.deck_bindings]
        if len(identities) != len(set(identities)):
            raise ValueError("source bundle contains duplicate vehicle deck bindings")
        ####
        return self

    ####


####


def load_cadac_source_bundle(input_path: str | Path) -> CadacSourceBundle:
    """Parse one case and resolve all deck references relative to that case."""

    source_path = Path(input_path).resolve()
    case = parse_cadac_input_file(source_path)
    source_root = source_path.parent
    deck_cache: dict[Path, tuple[CadacDeck, CadacSourceArtifact]] = {}
    bindings: list[CadacDeckBinding] = []
    for vehicle in case.vehicles:
        for reference in vehicle.deck_references:
            resolved = _resolve_deck_reference(source_path, reference.path)
            if resolved is None:
                raise FileNotFoundError(f"{case.source_name}:{reference.source_line}: referenced deck does not exist: {reference.path}")
            ####
            cached = deck_cache.get(resolved)
            if cached is None:
                cached = (parse_cadac_deck_file(resolved), _fingerprint(resolved))
                deck_cache[resolved] = cached
            ####
            deck, artifact = cached
            bindings.append(
                CadacDeckBinding(
                    vehicle_model=vehicle.model_name,
                    vehicle_role=vehicle.role,
                    vehicle_source_line=vehicle.source_line,
                    kind=reference.kind,
                    keyword=reference.keyword,
                    reference=reference.path,
                    resolved_path=source_name_for(resolved),
                    source_line=reference.source_line,
                    artifact=artifact,
                    deck=deck,
                )
            )
        ####
    ####
    return CadacSourceBundle(
        source_root=source_name_for(source_root),
        input_artifact=_fingerprint(source_path),
        case=case,
        deck_bindings=tuple(bindings),
    )


####


def _resolve_deck_reference(input_path: Path, reference: str) -> Path | None:
    """Resolve CADAC decks from the case directory, then its project directory.

    Several CADAC packages keep selectable input cases in an ``Inputs``
    subdirectory while deck names remain relative to the package/project
    working directory used by the original executables. The local case
    directory remains authoritative when both locations contain a match.
    """

    candidates = (
        (input_path.parent / reference).resolve(),
        (input_path.parent.parent / reference).resolve(),
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
        ####
    ####
    return None


####


def _fingerprint(path: Path) -> CadacSourceArtifact:
    payload = path.read_bytes()
    return CadacSourceArtifact(
        resolved_path=source_name_for(path),
        sha256=hashlib.sha256(payload).hexdigest(),
        byte_size=len(payload),
    )


####


__all__ = ["CadacDeckBinding", "CadacSourceArtifact", "CadacSourceBundle", "load_cadac_source_bundle"]
