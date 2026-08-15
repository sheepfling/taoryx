"""Shared pytest fixtures for opt-in test categories."""

from __future__ import annotations

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
VIEW_FILES: dict[str, frozenset[str]] = {
    "equations": frozenset(
        {
            "tests/parser/test_equation_provenance.py",
            "tests/unit/test_atmosphere_equations.py",
            "tests/unit/test_equations.py",
            "tests/unit/test_forces_equations.py",
            "tests/unit/test_frames_equations.py",
            "tests/unit/test_geodesy_equations.py",
            "tests/unit/test_gravity_equations.py",
            "tests/unit/test_guidance_equations.py",
            "tests/unit/test_motion_equations.py",
            "tests/unit/test_radar_equations.py",
            "tests/unit/test_radar_geometry_equations.py",
            "tests/unit/test_ranges_equations.py",
            "tests/unit/test_rates_equations.py",
            "tests/unit/test_remaining_equations.py",
            "tests/unit/test_search_optimization_equations.py",
            "tests/unit/test_state_rates.py",
            "tests/unit/test_verification_baseline.py",
        }
    ),
    "table": frozenset(
        {
            "tests/parser/test_table_parser.py",
            "tests/unit/test_table_examples_problem_harnesses.py",
            "tests/unit/test_table_examples_runtime_harnesses.py",
            "tests/unit/test_table_examples_workspace.py",
            "tests/unit/test_table_png_artifacts.py",
        }
    ),
    "algorithms": frozenset(
        {
            "tests/unit/test_algorithm_catalog.py",
            "tests/unit/test_guidance.py",
            "tests/unit/test_iip.py",
            "tests/unit/test_integration.py",
            "tests/unit/test_optimization.py",
            "tests/unit/test_runtime_algorithms.py",
            "tests/unit/test_runtime_cli.py",
            "tests/unit/test_runtime_lowering.py",
            "tests/unit/test_searches.py",
            "tests/unit/test_simulation_kernel.py",
            "tests/unit/test_verification_baseline.py",
        }
    ),
}


@pytest.fixture
def artifact_dir(request: pytest.FixtureRequest, pytestconfig: pytest.Config) -> Path:
    """Return a stable ignored directory for one human-readable test artifact set."""

    root = Path(pytestconfig.getoption("--artifact-dir"))
    test_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", request.node.nodeid)
    destination = root / test_name
    destination.mkdir(parents=True, exist_ok=True)
    return destination


def pytest_addoption(parser: pytest.Parser) -> None:
    """Register the artifact output location used by artifact-marked tests."""

    parser.addoption(
        "--artifact-dir",
        action="store",
        default="artifacts",
        help="directory for human-readable outputs produced by artifact-marked tests",
    )


def pytest_collection_modifyitems(items: list[pytest.Item]) -> None:
    """Apply overlapping grammar/equation/algorithm views to collected tests."""

    for item in items:
        relative_path = Path(str(item.fspath)).resolve().relative_to(ROOT).as_posix()
        if relative_path.startswith("tests/parser/"):
            item.add_marker("grammar")
        if relative_path.startswith("tests/unit/test_daveml"):
            item.add_marker("daveml")
        if relative_path.startswith(("tests/e2e/", "tests/families/")):
            item.add_marker("integration")
        if Path(relative_path).stem.endswith("_matrix"):
            item.add_marker("matrix")
        if relative_path.startswith("tests/parser/test_table_parser.py"):
            item.add_marker("table")
        for view, paths in VIEW_FILES.items():
            if relative_path in paths:
                item.add_marker(view)
