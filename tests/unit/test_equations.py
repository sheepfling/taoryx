import pytest

from taoryx.equations import Equation, EquationRegistry


def test_registry_returns_registered_equation() -> None:
    registry = EquationRegistry()
    equation = Equation("EQ-001", "gravity_force", "F = m * g")

    registry.register(equation)

    assert registry.get("EQ-001") == equation


def test_registry_rejects_duplicate_identifier() -> None:
    registry = EquationRegistry()
    equation = Equation("EQ-001", "gravity_force", "F = m * g")
    registry.register(equation)

    with pytest.raises(ValueError, match="already registered"):
        registry.register(equation)


def test_registry_reports_unknown_identifier() -> None:
    with pytest.raises(KeyError, match="unknown equation"):
        EquationRegistry().get("EQ-999")
