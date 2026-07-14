from __future__ import annotations

import pytest

from .support.comparisons import compare_metamorphic_outputs
from .support.loader import load_manifest, load_metamorphic
from .support.output import parse_column_file
from .support.runtime import run_case, taos_executable

CASES = {item.id: item for item in load_manifest()}


@pytest.mark.runtime
@pytest.mark.metamorphic
@pytest.mark.slow
@pytest.mark.parametrize("group", load_metamorphic(), ids=lambda item: item.id)
def test_metamorphic_group(group) -> None:
    if taos_executable() is None:
        pytest.skip("TAOS_EXE is not set")
    ####
    results = [run_case(CASES[case_id]) for case_id in group.cases]
    assert all(item.returncode == 0 for item in results)
    outputs = [
        parse_column_file(result.workdir / CASES[case_id].output_file)
        for case_id, result in zip(group.cases, results, strict=True)
    ]
    compare_metamorphic_outputs(group, outputs)
    ####
