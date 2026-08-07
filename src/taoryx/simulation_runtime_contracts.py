"""Simulation Runtime outcome and run-manifest contracts.

Simulation Runtime has many lower-level status vocabularies for parsing, preflight,
lowering, evidence, and qualification.  This module defines the small
top-level vocabulary that a consumer may use to classify one run outcome.
Lower-level statuses remain available in their native reports.
"""

from __future__ import annotations

from enum import StrEnum


class SimulationRuntimeStatus(StrEnum):
    """Closed top-level outcome vocabulary for Simulation Runtime consumers."""

    PASSED = "passed"
    DEVELOPMENT = "development"
    BLOCKED = "blocked"
    INCOMPLETE = "incomplete"
    OUT_OF_ENVELOPE = "out_of_envelope"
    FAILED = "failed"
####


SIMULATION_RUNTIME_STATUS_SCHEMA = "taoryx.simulation-runtime-status/v1alpha1"
SIMULATION_RUNTIME_RUN_MANIFEST_SCHEMA = "taoryx.simulation-runtime-run-manifest/v1alpha1"
SIMULATION_RUNTIME_RUN_MANIFEST_SCHEMA_VERSION = 1

SIMULATION_RUNTIME_RUN_MANIFEST_REQUIRED_FIELDS = (
    "schema",
    "schema_version",
    "scenario_id",
    "status",
    "expected_disposition",
    "operation",
    "fidelity",
    "realization",
    "source_inputs",
    "run_identity",
    "runtime",
    "integration",
    "time",
    "termination",
    "artifacts",
    "claim_boundary",
)


def classify_runtime_outcome(
    *,
    has_errors: bool,
    execution_started: bool,
    completed: bool,
    out_of_envelope: bool = False,
) -> SimulationRuntimeStatus:
    """Map a source-run result to the Simulation Runtime top-level vocabulary.

    ``development`` is intentionally not inferred here.  It describes the
    maturity of evidence, not whether the runtime completed.  Scenario and
    composition evaluators may retain it alongside a successful execution.
    """

    if out_of_envelope:
        return SimulationRuntimeStatus.OUT_OF_ENVELOPE
    if has_errors:
        return SimulationRuntimeStatus.FAILED if execution_started else SimulationRuntimeStatus.BLOCKED
    return SimulationRuntimeStatus.PASSED if completed else SimulationRuntimeStatus.INCOMPLETE
    ####


__all__ = [
    "SIMULATION_RUNTIME_RUN_MANIFEST_SCHEMA",
    "SIMULATION_RUNTIME_RUN_MANIFEST_SCHEMA_VERSION",
    "SIMULATION_RUNTIME_RUN_MANIFEST_REQUIRED_FIELDS",
    "SIMULATION_RUNTIME_STATUS_SCHEMA",
    "SimulationRuntimeStatus",
    "classify_runtime_outcome",
]
####
