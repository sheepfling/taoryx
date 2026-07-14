from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.runtime.runner import run_files

from .support.loader import case_directory, load_manifest
from .support.models import CaseSpec
from .support.oracles import evaluate

LOCAL_ANALYTIC_CASE_IDS: frozenset[str] = frozenset(
    {
        "p001_linear_ecfc_zero_force",
        "p002_constant_thrust_table",
    }
)

LOCAL_ANALYTIC_CASES: tuple[CaseSpec, ...] = tuple(case for case in load_manifest() if case.id in LOCAL_ANALYTIC_CASE_IDS)


def _case_id(case: CaseSpec) -> str:
    return case.id
####


@pytest.mark.parametrize("case", LOCAL_ANALYTIC_CASES, ids=_case_id)
def test_local_runtime_satisfies_declared_analytic_oracles(case: CaseSpec, tmp_path: Path) -> None:
    """Promote the manifest's analytic cases into executable local contracts."""

    source = case_directory(case) / "input"
    problem = source / case.problem_file.removeprefix("input/")
    tables = tuple(source / path.removeprefix("input/") for path in case.table_files)

    report = run_files(problem, tables, output_dir=tmp_path, max_steps=20_000)

    assert report.exit_code == 0, [(diagnostic.code, diagnostic.message) for diagnostic in report.diagnostics]
    assert all(result.completed for result in report.results)
    evaluate(case.oracles, tmp_path, case.output_file, report.exit_code)
####

