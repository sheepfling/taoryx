"""Stable typed views over the generated TAOS algorithm catalog."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field


class ImplementationTarget(BaseModel):
    """Proposed namespace and symbol for an algorithm implementation."""

    model_config = ConfigDict(extra="ignore")

    package: str
    module: str
    symbol: str
    public_api: bool = True
####


class AlgorithmRecord(BaseModel):
    """The catalog fields needed to plan and locate an implementation."""

    model_config = ConfigDict(extra="ignore")

    id: str
    slug: str
    name: str
    domain: str
    implementation_kind: str
    implementation_target: ImplementationTarget
    dependencies: list[str] = Field(default_factory=list)
    phase: str
    priority: str
    complexity: str
    status: str = "cataloged"
    source_equations: list[str] = Field(default_factory=list, alias="equations")
####
