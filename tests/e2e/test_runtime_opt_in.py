from __future__ import annotations

import os

import pytest

from .support.loader import load_manifest
from .support.oracles import evaluate
from .support.runtime import run_case, taos_executable

POSITIVE = [
    case
    for case in load_manifest()
    if case.kind == "positive" and case.runtime_tier not in {"exploratory", "historical", "stress"}
]
SPECIAL = [
    case
    for case in load_manifest()
    if case.kind == "positive" and case.runtime_tier in {"exploratory", "historical"}
]
STRESS = [case for case in load_manifest() if case.kind == "positive" and case.runtime_tier == "stress"]
NEGATIVE = [case for case in load_manifest() if case.kind == "negative"]


def _enabled(name: str) -> bool:
    return os.environ.get(name, "").lower() in {"1", "true", "yes", "on"}
####


@pytest.mark.runtime
@pytest.mark.parametrize("case", POSITIVE, ids=lambda item: item.id)
def test_runtime_case(case) -> None:
    if taos_executable() is None:
        pytest.skip("TAOS_EXE is not set")
    result = run_case(case, timeout=float(os.environ.get("TAOS_TIMEOUT", "180")))
    assert result.returncode == 0, result.stdout + "\n" + result.stderr
    evaluate(case.oracles, result.workdir, case.output_file, result.returncode)
####


@pytest.mark.runtime
@pytest.mark.historical
@pytest.mark.slow
@pytest.mark.parametrize("case", SPECIAL, ids=lambda item: item.id)
def test_special_runtime_case(case) -> None:
    if taos_executable() is None:
        pytest.skip("TAOS_EXE is not set")
    ####
    if not _enabled("TAOS_RUN_SPECIAL"):
        pytest.skip("Set TAOS_RUN_SPECIAL=1 to run exploratory and historical cases")
    ####
    result = run_case(case, timeout=float(os.environ.get("TAOS_SPECIAL_TIMEOUT", "600")))
    assert result.returncode == 0, result.stdout + "\n" + result.stderr
    evaluate(case.oracles, result.workdir, case.output_file, result.returncode)
####


@pytest.mark.runtime
@pytest.mark.stress
@pytest.mark.slow
@pytest.mark.parametrize("case", STRESS, ids=lambda item: item.id)
def test_stress_runtime_case(case) -> None:
    if taos_executable() is None:
        pytest.skip("TAOS_EXE is not set")
    ####
    if not _enabled("TAOS_RUN_STRESS"):
        pytest.skip("Set TAOS_RUN_STRESS=1 to run stress cases")
    ####
    result = run_case(case, timeout=float(os.environ.get("TAOS_STRESS_TIMEOUT", "1200")))
    assert result.returncode == 0, result.stdout + "\n" + result.stderr
    evaluate(case.oracles, result.workdir, case.output_file, result.returncode)
####


@pytest.mark.runtime
@pytest.mark.negative_runtime
@pytest.mark.parametrize("case", NEGATIVE, ids=lambda item: item.id)
def test_negative_runtime_case(case) -> None:
    if taos_executable() is None:
        pytest.skip("TAOS_EXE is not set")
    ####
    if not _enabled("TAOS_RUN_NEGATIVE"):
        pytest.skip("Set TAOS_RUN_NEGATIVE=1 to execute malformed inputs")
    ####
    result = run_case(case, timeout=float(os.environ.get("TAOS_NEGATIVE_TIMEOUT", "60")))
    transcript = (result.stdout + "\n" + result.stderr).lower()
    assert result.returncode != 0 or "error" in transcript or "invalid" in transcript, transcript
####
