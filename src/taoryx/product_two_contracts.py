"""Product 2 outcome and run-manifest contracts.

Product 2 has many lower-level status vocabularies for parsing, preflight,
lowering, evidence, and qualification.  This module defines the small
top-level vocabulary that a consumer may use to classify one run outcome.
Lower-level statuses remain available in their native reports.
"""

from __future__ import annotations

from enum import StrEnum


class ProductTwoStatus(StrEnum):
    """Closed top-level outcome vocabulary for Product 2 consumers."""

    PASSED = "passed"
    DEVELOPMENT = "development"
    BLOCKED = "blocked"
    INCOMPLETE = "incomplete"
    OUT_OF_ENVELOPE = "out_of_envelope"
    FAILED = "failed"
####


PRODUCT_TWO_STATUS_SCHEMA = "taoryx.product-two-status/v1alpha1"
PRODUCT_TWO_RUN_MANIFEST_SCHEMA = "taoryx.product-two-run-manifest/v1alpha1"
PRODUCT_TWO_RUN_MANIFEST_SCHEMA_VERSION = 1

PRODUCT_TWO_RUN_MANIFEST_REQUIRED_FIELDS = (
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
) -> ProductTwoStatus:
    """Map a source-run result to the Product 2 top-level vocabulary.

    ``development`` is intentionally not inferred here.  It describes the
    maturity of evidence, not whether the runtime completed.  Scenario and
    composition evaluators may retain it alongside a successful execution.
    """

    if out_of_envelope:
        return ProductTwoStatus.OUT_OF_ENVELOPE
    if has_errors:
        return ProductTwoStatus.FAILED if execution_started else ProductTwoStatus.BLOCKED
    return ProductTwoStatus.PASSED if completed else ProductTwoStatus.INCOMPLETE
    ####


__all__ = [
    "PRODUCT_TWO_RUN_MANIFEST_SCHEMA",
    "PRODUCT_TWO_RUN_MANIFEST_SCHEMA_VERSION",
    "PRODUCT_TWO_RUN_MANIFEST_REQUIRED_FIELDS",
    "PRODUCT_TWO_STATUS_SCHEMA",
    "ProductTwoStatus",
    "classify_runtime_outcome",
]
####
