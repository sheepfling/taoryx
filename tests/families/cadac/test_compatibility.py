from __future__ import annotations

import pytest
from taoryx.families.cadac.compatibility import (
    CADAC_COMPATIBILITY_PROFILE,
    CadacEventEvaluation,
    CadacIntegrationRule,
    cadac_stored_derivative_step,
)


def test_compatibility_profile_preserves_source_boundaries() -> None:
    assert CADAC_COMPATIBILITY_PROFILE.fixed_step is True
    assert CADAC_COMPATIBILITY_PROFILE.event_evaluation is CadacEventEvaluation.PRE_STEP
    assert CADAC_COMPATIBILITY_PROFILE.integration_rule is CadacIntegrationRule.STORED_DERIVATIVE_TRAPEZOID
    assert CADAC_COMPATIBILITY_PROFILE.lower_table_boundary == "linear"
    assert CADAC_COMPATIBILITY_PROFILE.upper_table_boundary == "clamp"


####


def test_stored_derivative_step() -> None:
    updated = cadac_stored_derivative_step(
        state=(10.0, -2.0),
        derivative_current=(4.0, 2.0),
        derivative_previous=(2.0, -2.0),
        step_size_s=0.5,
    )

    assert updated == pytest.approx((11.5, -2.0))


####


def test_stored_derivative_step_rejects_mismatched_vectors() -> None:
    with pytest.raises(ValueError, match="equal length"):
        cadac_stored_derivative_step((0.0,), (1.0, 2.0), (1.0,), 0.1)
    ####


####
