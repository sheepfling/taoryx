from __future__ import annotations

import numpy as np

from .models import MetamorphicSpec


def compare_metamorphic_outputs(group: MetamorphicSpec, outputs: list[dict[str, np.ndarray]]) -> None:
    """Apply a manifest metamorphic relation to already parsed output columns."""

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
        raise AssertionError(f"Unsupported metamorphic comparison: {group.comparison}")
    ####
####
