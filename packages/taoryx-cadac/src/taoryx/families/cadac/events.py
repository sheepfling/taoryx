"""Sequential CADAC event evaluation with source-order mutation semantics."""

from __future__ import annotations

from collections.abc import MutableMapping, Sequence
from dataclasses import dataclass

from .input_ast import CadacEventBlock, CadacParameterAssignment, CadacRelationalOperator

CadacRuntimeScalar = int | float


@dataclass(frozen=True, slots=True)
class CadacEventApplication:
    """One event transition applied before a vehicle module pass."""

    event_index: int
    source_line: int
    watch_variable: str
    operator: CadacRelationalOperator
    criterion: CadacRuntimeScalar
    previous_values: tuple[tuple[str, CadacRuntimeScalar], ...]
    updated_values: tuple[tuple[str, CadacRuntimeScalar], ...]


####


@dataclass(slots=True)
class CadacEventCursor:
    """Mutable cursor matching CADAC's one-next-event-at-a-time evaluator."""

    events: tuple[CadacEventBlock, ...]
    next_index: int = 0

    @classmethod
    def from_events(cls, events: Sequence[CadacEventBlock]) -> "CadacEventCursor":
        return cls(tuple(events))

    ####

    @property
    def complete(self) -> bool:
        return self.next_index >= len(self.events)

    ####

    def evaluate_and_apply(
        self,
        values: MutableMapping[str, CadacRuntimeScalar],
    ) -> CadacEventApplication | None:
        """Evaluate only the next source event and mutate values if it trips."""

        if self.complete:
            return None
        ####
        event = self.events[self.next_index]
        watch_key = _resolve_key(values, event.condition.variable)
        current = values[watch_key]
        criterion = _numeric(event.condition.value, event.condition.variable)
        if not _condition_met(current, event.condition.operator, criterion):
            return None
        ####
        previous: list[tuple[str, CadacRuntimeScalar]] = []
        updated: list[tuple[str, CadacRuntimeScalar]] = []
        for assignment in event.assignments:
            key = _resolve_key(values, assignment.name)
            value = _assignment_value(assignment, values[key])
            previous.append((key, values[key]))
            values[key] = value
            updated.append((key, value))
        ####
        application = CadacEventApplication(
            event_index=self.next_index,
            source_line=event.source_line,
            watch_variable=watch_key,
            operator=event.condition.operator,
            criterion=criterion,
            previous_values=tuple(previous),
            updated_values=tuple(updated),
        )
        self.next_index += 1
        return application

    ####


####


def _resolve_key(values: MutableMapping[str, CadacRuntimeScalar], name: str) -> str:
    requested = name.casefold()
    for key in values:
        if key.casefold() == requested:
            return key
        ####
    ####
    raise KeyError(f"CADAC event references undefined runtime variable {name!r}")


####


def _numeric(value: int | float | str, name: str) -> CadacRuntimeScalar:
    if not isinstance(value, (int, float)):
        raise TypeError(f"CADAC event scalar {name!r} must be numeric")
    ####
    return value


####


def _assignment_value(assignment: CadacParameterAssignment, current: CadacRuntimeScalar) -> CadacRuntimeScalar:
    value = _numeric(assignment.value, assignment.name)
    if isinstance(current, int) and not isinstance(current, bool):
        return int(value)
    ####
    return float(value)


####


def _condition_met(current: CadacRuntimeScalar, operator: CadacRelationalOperator, criterion: CadacRuntimeScalar) -> bool:
    compare_current: CadacRuntimeScalar = current
    compare_criterion: CadacRuntimeScalar = int(criterion) if isinstance(current, int) and not isinstance(current, bool) else float(criterion)
    if operator is CadacRelationalOperator.LESS:
        return compare_current < compare_criterion
    ####
    if operator is CadacRelationalOperator.EQUAL:
        return compare_current == compare_criterion
    ####
    if operator is CadacRelationalOperator.GREATER:
        return compare_current > compare_criterion
    ####
    raise AssertionError(f"unhandled CADAC relational operator {operator!r}")


####


__all__ = ["CadacEventApplication", "CadacEventCursor", "CadacRuntimeScalar"]
