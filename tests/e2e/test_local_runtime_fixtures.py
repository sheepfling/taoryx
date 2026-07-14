from __future__ import annotations

import pytest

from taoryx.runtime.runner import run_files

from .support.loader import case_directory, load_manifest

LOCAL_RUNTIME_CASES = tuple(
    case
    for case in load_manifest()
    if case.id.startswith("p") and case.id[1:4].isdigit() and int(case.id[1:4]) <= 32
)


@pytest.mark.parametrize("case", LOCAL_RUNTIME_CASES, ids=lambda item: item.id)
def test_supported_fixture_executes_through_local_runtime(case, tmp_path) -> None:
    """Exercise the checked-in parser, lowering, engine, and output boundary."""

    source = case_directory(case) / "input"
    problem = source / case.problem_file.removeprefix("input/")
    tables = tuple(source / path.removeprefix("input/") for path in case.table_files)

    report = run_files(problem, tables, output_dir=tmp_path, max_steps=20000)

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    assert report.cases >= 1
    assert all(result.completed for result in report.results)
    if case.output_file is not None:
        assert report.outputs
####
