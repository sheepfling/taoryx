from __future__ import annotations

from taoryx.simulation_runtime_contracts import (
    SIMULATION_RUNTIME_RUN_MANIFEST_REQUIRED_FIELDS,
    SimulationRuntimeStatus,
    classify_runtime_outcome,
)


def test_simulation_runtime_status_vocabulary_is_closed_and_ordered() -> None:
    assert tuple(status.value for status in SimulationRuntimeStatus) == (
        "passed",
        "development",
        "blocked",
        "incomplete",
        "out_of_envelope",
        "failed",
    )
    assert "schema" in SIMULATION_RUNTIME_RUN_MANIFEST_REQUIRED_FIELDS
    assert "claim_boundary" in SIMULATION_RUNTIME_RUN_MANIFEST_REQUIRED_FIELDS
    ####


def test_runtime_outcome_classification_keeps_execution_and_maturity_separate() -> None:
    assert classify_runtime_outcome(has_errors=False, execution_started=True, completed=True) is SimulationRuntimeStatus.PASSED
    assert classify_runtime_outcome(has_errors=False, execution_started=True, completed=False) is SimulationRuntimeStatus.INCOMPLETE
    assert classify_runtime_outcome(has_errors=True, execution_started=False, completed=False) is SimulationRuntimeStatus.BLOCKED
    assert classify_runtime_outcome(has_errors=True, execution_started=True, completed=False) is SimulationRuntimeStatus.FAILED
    assert classify_runtime_outcome(has_errors=False, execution_started=True, completed=False, out_of_envelope=True) is SimulationRuntimeStatus.OUT_OF_ENVELOPE
    ####
