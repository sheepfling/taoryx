"""Regression tests for package-based developer tool entrypoints."""

from __future__ import annotations

from tools.dev import tool_script


def test_tool_script_builds_module_invocation() -> None:
    command = tool_script("validate_hl20_qualification.py", "--validate-only")

    assert command[1:4] == ["-m", "tools.validate_hl20_qualification", "--validate-only"]
