"""Pluggable controller-design metadata and factories.

The plant, trim, and allocator contracts are shared.  This module records
which synthesis method is being used and provides the first concrete factory
for LQR.  Other methods can be added without changing vehicle problem files.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .runtime.lqr import LqrController, solve_continuous_lqr
from .trim import TrimResult

ControllerDesignMethod = Literal[
    "lqr",
    "pid",
    "mpc",
    "pole_placement",
    "dynamic_inversion",
    "rule_based",
]


class ControllerDesignSpec(BaseModel):
    """Vehicle-independent controller design declaration.

    ``trim`` identifies the operating-point artifact and ``allocator`` names
    the vehicle contract that applies actuator limits and mappings.  The
    method is metadata until a matching factory is requested.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    method: ControllerDesignMethod
    trim: str = Field(min_length=1)
    allocator: str = Field(min_length=1)
    states: tuple[str, ...] = ()
    controls: tuple[str, ...] = ()
    notes: str = ""

    @model_validator(mode="after")
    def validate_channels(self) -> ControllerDesignSpec:
        if len(set(self.states)) != len(self.states):
            raise ValueError(f"controller design {self.id!r} has duplicate states")
        if len(set(self.controls)) != len(self.controls):
            raise ValueError(f"controller design {self.id!r} has duplicate controls")
        if set(self.states) & set(self.controls):
            raise ValueError(f"controller design {self.id!r} overlaps state and control names")
        return self
        ####
    ####


class ControllerDesignCatalog(BaseModel):
    """Validated collection of controller design declarations."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = Field(ge=1)
    id: str = Field(min_length=1)
    designs: tuple[ControllerDesignSpec, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def unique_ids(self) -> ControllerDesignCatalog:
        ids = [design.id for design in self.designs]
        if len(set(ids)) != len(ids):
            raise ValueError("controller design IDs must be unique")
        return self
        ####
    ####

    def get(self, identifier: str) -> ControllerDesignSpec:
        """Return one design declaration by stable identifier."""

        for design in self.designs:
            if design.id == identifier:
                return design
        raise KeyError(f"unknown controller design {identifier!r}")
        ####


def load_controller_catalog(path: Path) -> ControllerDesignCatalog:
    """Load a YAML controller-design catalog."""

    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"controller design catalog must be a mapping: {path}")
    return ControllerDesignCatalog.model_validate(payload)
    ####


def build_lqr_controller(
    design: ControllerDesignSpec,
    trim: TrimResult,
    a: Sequence[Sequence[float]],
    b: Sequence[Sequence[float]],
    q: Sequence[Sequence[float]],
    r: Sequence[Sequence[float]],
    *,
    lower: Mapping[str, float] | None = None,
    upper: Mapping[str, float] | None = None,
) -> LqrController:
    """Build an LQR controller from a solved, named plant trim.

    This is intentionally the only LQR-specific entry point.  The trim's
    state/control names are checked against the design declaration before the
    gain is created, preventing accidental reuse of gains across vehicles.
    """

    if design.method != "lqr":
        raise ValueError(f"controller design {design.id!r} uses method {design.method!r}, not 'lqr'")
    if tuple(design.states) != tuple(trim.spec.state_names):
        raise ValueError(f"controller design {design.id!r} states do not match trim {design.trim!r}")
    if tuple(design.controls) != tuple(trim.spec.control_names):
        raise ValueError(f"controller design {design.id!r} controls do not match trim {design.trim!r}")
    result = solve_continuous_lqr(
        a,
        b,
        q,
        r,
        state_names=design.states,
        control_names=design.controls,
    )
    return LqrController(
        result,
        state_trim=trim.state,
        control_trim=trim.controls,
        lower=lower or {},
        upper=upper or {},
    )
    ####
