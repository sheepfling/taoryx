"""Regression coverage for the documentation ownership layout."""

from __future__ import annotations

from tools.validate_documentation_layout import validate_documentation_layout


def test_documentation_layout_has_canonical_pages_and_valid_navigation() -> None:
    assert validate_documentation_layout() == []
    ####
