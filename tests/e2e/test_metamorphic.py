from __future__ import annotations

import numpy as np
import pytest

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
    if group.comparison == "allclose":
        baseline = outputs[0]
        for current in outputs[1:]:
            for column in group.columns:
                assert np.allclose(baseline[column], current[column], atol=group.atol, rtol=group.rtol)
            ####
        ####
    elif group.comparison == "convergence":
        column = group.columns[0]
        reference = float(outputs[-1][column][-1])
        errors = [abs(float(item[column][-1]) - reference) for item in outputs[:-1]]
        assert all(left > right for left, right in zip(errors, errors[1:], strict=True))
    elif group.comparison == "crosswind_increases_airspeed":
        assert float(outputs[1]["vair"][-1]) > float(outputs[0]["vair"][-1])
    elif group.comparison == "both_reduce_relative_range":
        for output in outputs:
            series = output[group.columns[0]]
            assert float(series[-1]) < float(series[0])
        ####
    else:
        raise AssertionError(f"Unsupported metamorphic comparison {group.comparison}")
    ####
