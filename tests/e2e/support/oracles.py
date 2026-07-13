from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from .models import OracleSpec
from .output import has_nan, parse_column_file


def _column(columns: dict[str, np.ndarray], name: str | None) -> np.ndarray:
    if name is None or name not in columns:
        raise AssertionError(f"Missing output column {name!r}; available: {sorted(columns)}")
    ####
    return columns[name]
####


def evaluate(oracles: list[OracleSpec], workdir: Path, output_file: str | None, returncode: int) -> None:
    columns: dict[str, np.ndarray] = {}
    if output_file is not None and (workdir / output_file).exists():
        columns = parse_column_file(workdir / output_file)
    ####
    for oracle in oracles:
        if oracle.type == "clean_exit":
            assert returncode == 0
        elif oracle.type == "output_exists":
            target = oracle.path or output_file
            assert target is not None and (workdir / target).exists()
        elif oracle.type == "no_nan":
            assert columns and not has_nan(columns)
        elif oracle.type == "final_approx":
            series = _column(columns, oracle.column)
            assert oracle.value is not None
            assert math.isclose(float(series[-1]), oracle.value, rel_tol=oracle.rtol, abs_tol=oracle.atol)
        elif oracle.type == "final_between":
            series = _column(columns, oracle.column)
            assert oracle.low is not None and oracle.high is not None
            assert oracle.low <= float(series[-1]) <= oracle.high
        elif oracle.type == "series_constant":
            series = _column(columns, oracle.column)
            assert np.max(np.abs(series - series[0])) <= oracle.atol
        elif oracle.type == "series_constant_value":
            series = _column(columns, oracle.column)
            assert oracle.value is not None
            assert np.max(np.abs(series - oracle.value)) <= oracle.atol
        elif oracle.type == "series_monotonic":
            series = _column(columns, oracle.column)
            delta = np.diff(series)
            if oracle.direction == "increasing":
                assert np.all(delta > 0) if oracle.strict else np.all(delta >= -oracle.atol)
            else:
                assert np.all(delta < 0) if oracle.strict else np.all(delta <= oracle.atol)
            ####
        elif oracle.type == "series_nonnegative":
            assert np.all(_column(columns, oracle.column) >= -oracle.atol)
        elif oracle.type == "series_positive":
            assert np.all(_column(columns, oracle.column) > 0)
        elif oracle.type == "series_between":
            series = _column(columns, oracle.column)
            assert oracle.low is not None and oracle.high is not None
            assert np.all((series >= oracle.low) & (series <= oracle.high))
        elif oracle.type == "max_interior":
            series = _column(columns, oracle.column)
            index = int(np.argmax(series))
            assert 0 < index < len(series) - 1
        elif oracle.type == "contains_times":
            times = _column(columns, "time")
            for value in oracle.values:
                assert np.min(np.abs(times - value)) <= oracle.atol
            ####
        elif oracle.type == "relative_range_reduces":
            series = _column(columns, oracle.column)
            assert oracle.minimum_fraction is not None
            assert float(series[-1]) <= float(series[0]) * (1.0 - oracle.minimum_fraction)
        elif oracle.type == "contains_step_change":
            series = _column(columns, oracle.column)
            assert oracle.minimum_jump is not None
            assert np.max(np.abs(np.diff(series))) >= oracle.minimum_jump
        elif oracle.type == "value_before_after":
            times = _column(columns, "time")
            series = _column(columns, oracle.column)
            assert oracle.split_time is not None and oracle.before is not None and oracle.after is not None
            before = series[times < oracle.split_time - 1e-12]
            after = series[times > oracle.split_time + 1e-12]
            assert before.size and after.size
            assert np.max(np.abs(before - oracle.before)) <= oracle.atol
            assert np.max(np.abs(after - oracle.after)) <= oracle.atol
        else:
            raise AssertionError(f"Unsupported oracle type: {oracle.type}")
        ####
    ####
####
