"""Tests for the air-breathing fixed R1 perturbation harness."""

from __future__ import annotations

from pathlib import Path

import pytest

from tools.validate_airbreathing_r1_matrix import _mutate_problem, run_matrix


def test_problem_mutation_changes_only_initial_state(tmp_path: Path) -> None:
    source = Path("examples/mission_families/slower_x8/SV03_racetrack_altitude_turns_3dof.prb")
    destination = tmp_path / "mutated.prb"
    _mutate_problem(source, destination, {"initial_altitude_offset_m": 50.0})

    source_lines = source.read_text(encoding="utf-8").splitlines()
    mutated_lines = destination.read_text(encoding="utf-8").splitlines()
    changed = [index for index, (left, right) in enumerate(zip(source_lines, mutated_lines, strict=True)) if left != right]
    assert len(changed) == 1
    assert "*initial geodetic" in mutated_lines[changed[0]] or "*initial ecic" in mutated_lines[changed[0]]
    ####


@pytest.mark.slow
def test_airbreathing_r1_matrix_records_boundary_cases(tmp_path: Path) -> None:
    report = run_matrix(tmp_path, families=("skywalker_x8",))

    assert report["status"] == "R1_fixed_matrix_complete"
    assert report["case_count"] == 8
    assert report["passed_case_count"] + report["failed_case_count"] == 8
    assert all(record["family_id"] == "skywalker_x8" for record in report["cases"])
    ####
