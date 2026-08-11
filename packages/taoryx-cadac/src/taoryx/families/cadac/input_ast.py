"""Typed source-level AST for CADAC ``input.asc`` cases."""

from __future__ import annotations

from enum import StrEnum
from pathlib import Path
from typing import TypeAlias

from pydantic import BaseModel, ConfigDict, Field, computed_field, model_validator


class CadacModel(BaseModel):
    """Frozen, strict base model for source-intake records."""

    model_config = ConfigDict(extra="forbid", frozen=True)


####


class CadacModuleStage(StrEnum):
    """Execution stages accepted by a CADAC module declaration."""

    DEFINE = "def"
    INITIALIZE = "init"
    EXECUTE = "exec"
    TERMINATE = "term"


####


class CadacDeckKind(StrEnum):
    """Known source deck-reference categories."""

    AERODYNAMIC = "AERO_DECK"
    PROPULSION = "PROP_DECK"
    GENERIC = "DECK"


####


class CadacStochasticKind(StrEnum):
    """Source stochastic declarations supported by CADAC vehicle input blocks."""

    GAUSSIAN = "GAUSS"
    MARKOV = "MARKOV"
    RAYLEIGH = "RAYL"


####


class CadacRelationalOperator(StrEnum):
    """Single-character event operators used by the CADAC input reader."""

    EQUAL = "="
    GREATER = ">"
    LESS = "<"


####


class CadacMonteCarloSpec(CadacModel):
    """Optional top-level ``MONTE <runs> <seed>`` execution declaration."""

    runs: int = Field(gt=0)
    seed: int
    source_line: int = Field(ge=1)


####


class CadacModuleInvocation(CadacModel):
    """One ordered module invocation from the ``MODULES`` block."""

    name: str = Field(min_length=1)
    stages: tuple[CadacModuleStage, ...] = Field(min_length=1)
    source_line: int = Field(ge=1)

    @model_validator(mode="after")
    def validate_unique_stages(self) -> "CadacModuleInvocation":
        if len(set(self.stages)) != len(self.stages):
            raise ValueError(f"module {self.name!r} declares duplicate stages")
        ####
        return self

    ####


####


class CadacTimingSetting(CadacModel):
    """One ordered timing value from the ``TIMING`` block."""

    name: str = Field(min_length=1)
    value: float
    source_line: int = Field(ge=1)


####


class CadacParameterAssignment(CadacModel):
    """One vehicle parameter assignment with source provenance."""

    name: str = Field(min_length=1)
    value: int | float | str
    comment: str | None = None
    source_line: int = Field(ge=1)


####


class CadacDeckReference(CadacModel):
    """One vehicle-local aerodynamic, propulsion, or generic deck reference."""

    kind: CadacDeckKind
    keyword: str = Field(min_length=1)
    path: str = Field(min_length=1)
    source_line: int = Field(ge=1)


####


class CadacStochasticAssignment(CadacModel):
    """One source random-variable declaration retained without sampling it at parse time."""

    kind: CadacStochasticKind
    name: str = Field(min_length=1)
    parameters: tuple[float, ...] = Field(min_length=1)
    comment: str | None = None
    source_line: int = Field(ge=1)


####


class CadacEventCondition(CadacModel):
    """One sequential CADAC event watchpoint."""

    variable: str = Field(min_length=1)
    operator: CadacRelationalOperator
    value: int | float


####


class CadacEventBlock(CadacModel):
    """One source-ordered ``IF ... ENDIF`` event mutation block."""

    condition: CadacEventCondition
    assignments: tuple[CadacParameterAssignment, ...] = ()
    source_line: int = Field(ge=1)
    end_source_line: int = Field(ge=1)

    @model_validator(mode="after")
    def validate_unique_assignments(self) -> "CadacEventBlock":
        names = [assignment.name.casefold() for assignment in self.assignments]
        if len(names) != len(set(names)):
            raise ValueError("event contains duplicate parameter assignments")
        ####
        if self.end_source_line <= self.source_line:
            raise ValueError("event ENDIF must follow IF")
        ####
        return self

    ####


####


CadacVehicleEntry: TypeAlias = CadacParameterAssignment | CadacDeckReference | CadacStochasticAssignment | CadacEventBlock


class CadacVehicleBlock(CadacModel):
    """One source vehicle block, preserving assignment and event order."""

    model_name: str = Field(min_length=1)
    role: str = Field(min_length=1)
    entries: tuple[CadacVehicleEntry, ...]
    source_line: int = Field(ge=1)

    @computed_field
    @property
    def assignments(self) -> tuple[CadacParameterAssignment, ...]:
        """Return only initial, non-event assignments."""

        return tuple(entry for entry in self.entries if isinstance(entry, CadacParameterAssignment))

    ####

    @computed_field
    @property
    def deck_references(self) -> tuple[CadacDeckReference, ...]:
        return tuple(entry for entry in self.entries if isinstance(entry, CadacDeckReference))

    ####

    @computed_field
    @property
    def stochastic_assignments(self) -> tuple[CadacStochasticAssignment, ...]:
        """Return source stochastic declarations without materializing random draws."""

        return tuple(entry for entry in self.entries if isinstance(entry, CadacStochasticAssignment))

    ####

    @computed_field
    @property
    def events(self) -> tuple[CadacEventBlock, ...]:
        return tuple(entry for entry in self.entries if isinstance(entry, CadacEventBlock))

    ####

    def parameter(self, name: str) -> int | float | str:
        """Return the final case-insensitive initial assignment or raise ``KeyError``.

        CADAC processes vehicle data sequentially, so a later source assignment
        replaces an earlier value.  The full ordered assignment history remains
        available through :attr:`assignments` for provenance and diagnostics.
        """

        key = name.casefold()
        for assignment in reversed(self.assignments):
            if assignment.name.casefold() == key:
                return assignment.value
            ####
        ####
        raise KeyError(name)

    ####


####


class CadacInputCase(CadacModel):
    """Parsed CADAC case before lowering into Taoryx runtime contracts."""

    source_name: str = Field(min_length=1)
    title_source: str = Field(min_length=1)
    description: str = ""
    monte_carlo: CadacMonteCarloSpec | None = None
    options: tuple[str, ...]
    modules: tuple[CadacModuleInvocation, ...] = Field(min_length=1)
    timing: tuple[CadacTimingSetting, ...] = Field(min_length=1)
    vehicles: tuple[CadacVehicleBlock, ...] = Field(min_length=1)
    end_time_s: float = Field(gt=0.0)
    stop_declared: bool = True

    @computed_field
    @property
    def timing_values(self) -> dict[str, float]:
        return {setting.name: setting.value for setting in self.timing}

    ####

    def vehicles_named(self, model_name: str) -> tuple[CadacVehicleBlock, ...]:
        """Return all source-ordered vehicles whose model name matches."""

        key = model_name.casefold()
        return tuple(vehicle for vehicle in self.vehicles if vehicle.model_name.casefold() == key)

    ####

    def vehicle(self, model_name: str) -> CadacVehicleBlock:
        """Return one uniquely named vehicle model or raise ``KeyError``."""

        matches = self.vehicles_named(model_name)
        if len(matches) != 1:
            raise KeyError(f"expected one {model_name!r} vehicle, found {len(matches)}")
        ####
        return matches[0]

    ####

    @model_validator(mode="after")
    def validate_case_identity(self) -> "CadacInputCase":
        timing_names = [setting.name.casefold() for setting in self.timing]
        if len(timing_names) != len(set(timing_names)):
            raise ValueError("TIMING contains duplicate settings")
        ####
        module_names = [module.name.casefold() for module in self.modules]
        if len(module_names) != len(set(module_names)):
            raise ValueError("MODULES contains duplicate module names")
        ####
        return self

    ####


####


def source_name_for(path: str | Path) -> str:
    """Normalize a source path for stable case provenance."""

    return Path(path).as_posix()


####


__all__ = [
    "CadacDeckKind",
    "CadacDeckReference",
    "CadacEventBlock",
    "CadacEventCondition",
    "CadacInputCase",
    "CadacModel",
    "CadacMonteCarloSpec",
    "CadacModuleInvocation",
    "CadacModuleStage",
    "CadacParameterAssignment",
    "CadacStochasticAssignment",
    "CadacStochasticKind",
    "CadacRelationalOperator",
    "CadacTimingSetting",
    "CadacVehicleBlock",
    "CadacVehicleEntry",
    "source_name_for",
]
