"""Stable identifiers for equations referenced by the manual and examples."""

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Equation:
    """Metadata for one numbered equation."""

    identifier: str
    name: str
    expression: str
    status: str = "scaffold"


class EquationRegistry:
    """An in-memory registry with duplicate and missing-ID protection."""

    def __init__(self) -> None:
        self._equations: dict[str, Equation] = {}

    def register(self, equation: Equation) -> None:
        if equation.identifier in self._equations:
            raise ValueError(f"equation already registered: {equation.identifier}")
        self._equations[equation.identifier] = equation

    def get(self, identifier: str) -> Equation:
        try:
            return self._equations[identifier]
        except KeyError as error:
            raise KeyError(f"unknown equation: {identifier}") from error

    def all(self) -> tuple[Equation, ...]:
        return tuple(self._equations.values())


registry = EquationRegistry()
registry.register(
    Equation(
        identifier="EQ-001",
        name="gravity_force",
        expression="F = m * g",
    )
)
