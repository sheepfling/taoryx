"""Regression tests for package-based developer tool entrypoints."""

from __future__ import annotations

from tools.dev import QUICK_TEST_PATHS, _changed_test_paths, tool_script


def test_tool_script_builds_module_invocation() -> None:
    command = tool_script("validate_hl20_qualification.py", "--validate-only")

    assert command[1:4] == ["-m", "tools.validate_hl20_qualification", "--validate-only"]


def test_changed_test_selection_maps_changed_source_to_importing_tests() -> None:
    selected = _changed_test_paths(("src/taoryx/simulation_runtime_quality.py",))

    assert "tests/unit/test_simulation_runtime_quality.py" in selected


def test_changed_test_selection_falls_back_to_quick_suite_without_test_mapping() -> None:
    assert _changed_test_paths(("docs/BUILDING_TESTS.md",)) == QUICK_TEST_PATHS
