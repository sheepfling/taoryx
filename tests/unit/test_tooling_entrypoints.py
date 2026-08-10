"""Regression tests for package-based developer tool entrypoints."""

from __future__ import annotations

import tools.dev as dev
from scripts.bootstrap import editable_install_command
from tools.dev import QUICK_TEST_PATHS, _changed_test_paths, tool_script


def test_tool_script_builds_module_invocation() -> None:
    command = tool_script("validate_hl20_qualification.py", "--validate-only")

    assert command[1:4] == ["-m", "tools.validate_hl20_qualification", "--validate-only"]


def test_changed_test_selection_maps_changed_source_to_importing_tests() -> None:
    selected = _changed_test_paths(("src/taoryx/simulation_runtime_quality.py",))

    assert "tests/unit/test_simulation_runtime_quality.py" in selected


def test_changed_test_selection_falls_back_to_quick_suite_without_test_mapping() -> None:
    assert _changed_test_paths(("docs/BUILDING_TESTS.md",)) == QUICK_TEST_PATHS


def test_vehicle_slice_selects_only_the_declared_f16_vertical_contract(
    monkeypatch,
) -> None:
    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.test_vehicle_vertical("f16_s119")

    assert commands == [
        [
            dev.project_python(),
            "-m",
            "pytest",
            "tests/unit/test_f16_vehicle_vertical.py",
            "-q",
            "-x",
            "-o",
            "addopts=",
            "--strict-markers",
            "--basetemp",
            ".pytest-vehicle-f16_s119",
        ]
    ]
    ####


def test_vehicle_slice_selects_the_simple_aero_workflow_contract(
    monkeypatch,
) -> None:
    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.test_vehicle_vertical("simple_aero")

    assert commands[0][3] == "tests/unit/test_simple_aero_vehicle_vertical.py"
    assert commands[0][-1] == ".pytest-vehicle-simple_aero"
    ####


def test_vehicle_slice_selects_the_dual_launch_workflow_contract(
    monkeypatch,
) -> None:
    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.test_vehicle_vertical("dual_launch_glider")

    assert commands[0][3] == "tests/unit/test_dual_launch_vehicle_vertical.py"
    assert commands[0][-1] == ".pytest-vehicle-dual_launch_glider"
    ####


def test_vehicle_catalogue_gate_runs_only_catalogue_contract_checks(
    monkeypatch,
) -> None:
    calls: list[str] = []
    for name in (
        "test_vehicle_catalogue",
        "check_vehicle_models",
        "check_vehicle_interfaces",
        "onboard_vehicles",
    ):
        monkeypatch.setattr(dev, name, lambda name=name: calls.append(name))

    dev.vehicle_catalogue()

    assert calls == [
        "test_vehicle_catalogue",
        "check_vehicle_models",
        "check_vehicle_interfaces",
        "onboard_vehicles",
    ]
    ####


def test_bootstrap_full_profile_installs_every_editable_distribution() -> None:
    command = editable_install_command(
        "python",
        profile="full",
        with_dependencies=True,
        with_sensors=False,
    )

    assert command == [
        "python",
        "-m",
        "pip",
        "install",
        "-e",
        ".[dev]",
        "-e",
        "packages/taoryx-daveml",
        "-e",
        "packages/taoryx-simple-aero",
        "-e",
        "packages/taoryx-reference-models",
        "-e",
        "packages/taoryx-reachability",
    ]
    ####


def test_bootstrap_offline_sensor_fallback_keeps_full_local_distribution_set() -> None:
    command = editable_install_command(
        "python",
        profile="full",
        with_dependencies=False,
        with_sensors=True,
    )

    assert command[4:6] == ["--no-build-isolation", "--no-deps"]
    assert command.count("-e") == 5
    assert command[-1] == "packages/taoryx-reachability"
    ####
