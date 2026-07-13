"""Output-evaluation planning contracts."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class OutputEvaluationPlan:
    """Ordered set of derived evaluators required by a problem."""

    required: tuple[str, ...]
    unavailable: tuple[str, ...]
####


def build_output_evaluation_plan(
    references: Iterable[str],
    final_conditions: Iterable[str] = (),
    expressions: Iterable[str] = (),
    searches: Iterable[str] = (),
    optimization_constraints: Iterable[str] = (),
    available_evaluators: Mapping[str, object] | Iterable[str] = (),
) -> OutputEvaluationPlan:
    """Evaluate TAOS-ALG-OUT-001's demand-driven output dependency plan."""

    required = tuple(
        dict.fromkeys(
            name.casefold()
            for source in (references, final_conditions, expressions, searches, optimization_constraints)
            for name in source
        )
    )
    available = {name.casefold() for name in available_evaluators} if not isinstance(available_evaluators, Mapping) else {name.casefold() for name in available_evaluators}
    return OutputEvaluationPlan(required, tuple(name for name in required if name not in available))
####
