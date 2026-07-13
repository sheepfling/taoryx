from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field


class OracleSpec(BaseModel):
    type: str
    column: str | None = None
    path: str | None = None
    value: float | None = None
    values: list[float] = Field(default_factory=list)
    atol: float = 0.0
    rtol: float = 0.0
    low: float | None = None
    high: float | None = None
    direction: Literal["increasing", "decreasing"] | None = None
    strict: bool = False
    split_time: float | None = None
    before: float | None = None
    after: float | None = None
    minimum_fraction: float | None = None
    minimum_jump: float | None = None
####


class CaseSpec(BaseModel):
    id: str
    title: str
    kind: Literal["positive", "negative"]
    path: str
    problem_file: str
    table_files: list[str] = Field(default_factory=list)
    features: list[str] = Field(default_factory=list)
    grammar_productions: list[str] = Field(default_factory=list)
    runtime_tier: str
    static_expectation: Literal["pass", "error", "future-error"]
    expected_diagnostics: list[str] = Field(default_factory=list)
    output_file: str | None = None
    oracles: list[OracleSpec] = Field(default_factory=list)
    notes: list[str] = Field(default_factory=list)
    source_sections: list[str] = Field(default_factory=list)
####


class MetamorphicSpec(BaseModel):
    id: str
    title: str
    cases: list[str]
    comparison: str
    columns: list[str] = Field(default_factory=list)
    atol: float = 0.0
    rtol: float = 0.0
    notes: list[str] = Field(default_factory=list)
####
