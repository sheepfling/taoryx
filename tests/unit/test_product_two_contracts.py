from __future__ import annotations

from taoryx.product_two_contracts import (
    PRODUCT_TWO_RUN_MANIFEST_REQUIRED_FIELDS,
    ProductTwoStatus,
    classify_runtime_outcome,
)


def test_product_two_status_vocabulary_is_closed_and_ordered() -> None:
    assert tuple(status.value for status in ProductTwoStatus) == (
        "passed",
        "development",
        "blocked",
        "incomplete",
        "out_of_envelope",
        "failed",
    )
    assert "schema" in PRODUCT_TWO_RUN_MANIFEST_REQUIRED_FIELDS
    assert "claim_boundary" in PRODUCT_TWO_RUN_MANIFEST_REQUIRED_FIELDS
    ####


def test_runtime_outcome_classification_keeps_execution_and_maturity_separate() -> None:
    assert classify_runtime_outcome(has_errors=False, execution_started=True, completed=True) is ProductTwoStatus.PASSED
    assert classify_runtime_outcome(has_errors=False, execution_started=True, completed=False) is ProductTwoStatus.INCOMPLETE
    assert classify_runtime_outcome(has_errors=True, execution_started=False, completed=False) is ProductTwoStatus.BLOCKED
    assert classify_runtime_outcome(has_errors=True, execution_started=True, completed=False) is ProductTwoStatus.FAILED
    assert classify_runtime_outcome(has_errors=False, execution_started=True, completed=False, out_of_envelope=True) is ProductTwoStatus.OUT_OF_ENVELOPE
    ####
