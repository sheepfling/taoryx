"""Numerical and scheduling semantics for CADAC source-parity runs."""

from __future__ import annotations

import math
from collections.abc import Sequence
from enum import StrEnum

from pydantic import Field

from .input_ast import CadacModel


class CadacModuleOrdering(StrEnum):
    """Module-order policy for a compatibility execution."""

    SOURCE_DECLARATION = "source_declaration"


####


class CadacEventEvaluation(StrEnum):
    """Event-evaluation location relative to each fixed step."""

    PRE_STEP = "pre_step"


####


class CadacIntegrationRule(StrEnum):
    """State update used by the reference utility integration function."""

    STORED_DERIVATIVE_TRAPEZOID = "stored_derivative_trapezoid"


####


class CadacCompatibilityProfile(CadacModel):
    """Explicit source-parity semantics, separate from Taoryx-native runs."""

    profile_id: str = Field(min_length=1)
    module_ordering: CadacModuleOrdering
    event_evaluation: CadacEventEvaluation
    integration_rule: CadacIntegrationRule
    fixed_step: bool
    lower_table_boundary: str
    upper_table_boundary: str
    notes: tuple[str, ...] = ()


####


CADAC_COMPATIBILITY_PROFILE = CadacCompatibilityProfile(
    profile_id="cadac_compat/v0",
    module_ordering=CadacModuleOrdering.SOURCE_DECLARATION,
    event_evaluation=CadacEventEvaluation.PRE_STEP,
    integration_rule=CadacIntegrationRule.STORED_DERIVATIVE_TRAPEZOID,
    fixed_step=True,
    lower_table_boundary="linear",
    upper_table_boundary="clamp",
    notes=(
        "Preserve source module ordering until one-step parity is established.",
        "The caller owns storage of the previous derivative and updates it after the accepted state step.",
    ),
)


def cadac_stored_derivative_step(
    state: Sequence[float],
    derivative_current: Sequence[float],
    derivative_previous: Sequence[float],
    step_size_s: float,
) -> tuple[float, ...]:
    """Advance one state using CADAC's stored-derivative trapezoid update.

    This function intentionally does not evaluate a derivative callback. It
    reproduces the source utility boundary where the current and previously
    stored rates are already known. The caller must store
    ``derivative_current`` for the next source-ordered module execution.
    """

    values = tuple(float(value) for value in state)
    current = tuple(float(value) for value in derivative_current)
    previous = tuple(float(value) for value in derivative_previous)
    if not math.isfinite(step_size_s) or step_size_s <= 0.0:
        raise ValueError("step_size_s must be positive and finite")
    ####
    if len(values) != len(current) or len(values) != len(previous):
        raise ValueError("state and derivative vectors must have equal length")
    ####
    if not values:
        raise ValueError("state vector must not be empty")
    ####
    if any(not math.isfinite(value) for value in values + current + previous):
        raise ValueError("state and derivative vectors must contain finite values")
    ####
    return tuple(
        value + 0.5 * step_size_s * (rate_current + rate_previous) for value, rate_current, rate_previous in zip(values, current, previous, strict=True)
    )


####


__all__ = [
    "CADAC_COMPATIBILITY_PROFILE",
    "CadacCompatibilityProfile",
    "CadacEventEvaluation",
    "CadacIntegrationRule",
    "CadacModuleOrdering",
    "cadac_stored_derivative_step",
]
