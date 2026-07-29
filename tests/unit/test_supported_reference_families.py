from __future__ import annotations

from tools.validate_supported_reference_families import validate


def test_supported_reference_family_registry_is_consistent() -> None:
    """F-16 and HL-20 remain explicit, source-grounded support entries."""

    validate()
    ####
