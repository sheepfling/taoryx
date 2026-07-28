from __future__ import annotations

import numpy as np
import pytest

from taoryx.trim import (
    TrimConfigurationError,
    TrimEvaluationError,
    TrimGate,
    TrimProcedure,
    TrimSpec,
    finite_difference_dynamics_linearization,
    finite_difference_linearization,
    solve_trim,
    solve_trim_continuation,
    solve_trim_procedure,
)


def _plant(state: dict[str, float], controls: dict[str, float]) -> dict[str, float]:
    x = state["x"]
    u = controls["u"]
    return {"force": x + u - 1.0, "moment": 2.0 * x - u}


def _spec() -> TrimSpec:
    return TrimSpec(
        state_names=("x",),
        control_names=("u",),
        residual_names=("force", "moment"),
        state_initial={"x": 0.0},
        control_initial={"u": 0.0},
        state_lower={"x": -10.0},
        state_upper={"x": 10.0},
        control_lower={"u": -10.0},
        control_upper={"u": 10.0},
        residual_scales={"force": 1.0, "moment": 1.0},
    )


def test_solve_trim_returns_named_plant_bound_solution() -> None:
    result = solve_trim(_spec(), _plant)

    assert result.success
    assert result.state["x"] == 1.0 / 3.0
    assert result.controls["u"] == 2.0 / 3.0
    assert result.max_residual < 1e-10
    assert result.iterations > 0


def test_trim_finite_difference_linearization_is_named_and_local() -> None:
    spec = _spec()
    result = solve_trim(spec, _plant)

    a, b = finite_difference_linearization(spec, _plant, result)

    np.testing.assert_allclose(a, [[1.0], [2.0]], rtol=1e-6, atol=1e-8)
    np.testing.assert_allclose(b, [[1.0], [-1.0]], rtol=1e-6, atol=1e-8)


def test_dynamics_linearization_requires_true_named_state_derivatives() -> None:
    spec = TrimSpec(
        state_names=("x", "v"),
        control_names=("u",),
        residual_names=("force",),
        state_initial={"x": 0.0, "v": 0.0},
        control_initial={"u": 0.0},
    )
    trim = solve_trim(spec, lambda state, controls: {"force": state["x"] + controls["u"]})

    linearization = finite_difference_dynamics_linearization(
        spec,
        lambda state, controls: {"x": state["v"], "v": -state["x"] + controls["u"]},
        trim,
        metadata={"source_quality": "estimated", "mass_kg": 10.0},
    )

    np.testing.assert_allclose(linearization.a_matrix, [[0.0, 1.0], [-1.0, 0.0]], rtol=1e-6, atol=1e-8)
    np.testing.assert_allclose(linearization.b_matrix, [[0.0], [1.0]], rtol=1e-6, atol=1e-8)
    assert linearization.metadata_dict["source_quality"] == "estimated"


def test_trim_rejects_duplicate_state_and_control_names() -> None:
    try:
        TrimSpec(
            state_names=("x",),
            control_names=("x",),
            residual_names=("force",),
            state_initial={"x": 0.0},
            control_initial={"x": 0.0},
        )
    except ValueError as error:
        assert "unique" in str(error)
    else:
        raise AssertionError("duplicate trim names should be rejected")


def test_trim_configuration_error_identifies_field_and_repair() -> None:
    with pytest.raises(TrimConfigurationError) as raised:
        TrimSpec(
            state_names=("x",),
            control_names=("x",),
            residual_names=("force",),
            state_initial={"x": 0.0},
            control_initial={"x": 0.0},
        )

    assert raised.value.diagnostic.code == "duplicate-variable-name"
    assert raised.value.diagnostic.field == "state_names/control_names"
    assert "Fix:" in str(raised.value)


def test_trim_reports_bound_and_residual_diagnostics() -> None:
    spec = TrimSpec(
        state_names=("x",),
        control_names=(),
        residual_names=("force",),
        state_initial={"x": 0.0},
        control_initial={},
        state_lower={"x": 0.0},
        state_upper={"x": 1.0},
    )

    result = solve_trim(spec, lambda state, controls: {"force": state["x"] - 2.0})
    codes = {diagnostic.code for diagnostic in result.diagnostics}

    assert result.success is False
    assert "solution-at-bound" in codes
    assert "residual-above-tolerance" in codes
    assert result.as_dict()["diagnostics"]


def test_trim_evaluation_error_identifies_nonfinite_residual() -> None:
    with pytest.raises(TrimEvaluationError) as raised:
        solve_trim(_spec(), lambda state, controls: {"force": np.nan, "moment": 0.0})

    assert raised.value.diagnostic.code == "nonfinite-residual"
    assert raised.value.diagnostic.field == "force"


def test_trim_configuration_error_identifies_non_numeric_initial_value() -> None:
    with pytest.raises(TrimConfigurationError) as raised:
        TrimSpec(
            state_names=("x",),
            control_names=(),
            residual_names=("force",),
            state_initial={"x": "not-a-number"},
            control_initial={},
        )

    assert raised.value.diagnostic.code == "non-numeric-initial-value"
    assert raised.value.diagnostic.field == "x"
    assert "Fix:" in str(raised.value)


def test_dynamics_linearization_rejects_force_mapping_as_state_derivative() -> None:
    spec = TrimSpec(
        state_names=("x",),
        control_names=(),
        residual_names=("force",),
        state_initial={"x": 0.0},
        control_initial={},
    )
    trim = solve_trim(spec, lambda state, controls: {"force": state["x"]})

    with pytest.raises(TrimEvaluationError, match=r"\[trim:missing-state-derivative\].*Fix:"):
        finite_difference_dynamics_linearization(
            spec,
            lambda state, controls: {"force": state["x"]},
            trim,
        )


def _procedure() -> TrimProcedure:
    return TrimProcedure(
        id="demo-procedure-v1",
        vehicle="demo",
        fidelity="point_mass_3dof",
        spec=_spec(),
        operating_point={"speed_m_s": 10.0},
        provenance={"source": "unit-test"},
        multi_start=4,
        seed=17,
        gates=(TrimGate("force-limit", "force_abs", "maximum", 1.0e-10, "N"),),
    )


def test_trim_procedure_is_deterministic_and_preserves_provenance() -> None:
    procedure = _procedure()

    def evaluator(state, controls, operating_point):
        assert operating_point["speed_m_s"] == 10.0
        values = _plant(dict(state), dict(controls))
        values["force_abs"] = abs(values["force"])
        return values

    first = solve_trim_procedure(procedure, evaluator, metrics={"force_abs": 0.0})
    second = solve_trim_procedure(procedure, evaluator, metrics={"force_abs": 0.0})

    assert first.converged
    assert first.as_dict() == second.as_dict()
    assert first.procedure.as_dict()["provenance"] == {"source": "unit-test"}
    assert len(first.attempts) == 4
    assert first.gate_results[0].status == "pass"


def test_trim_procedure_classifies_unavailable_gate_and_bad_adapter() -> None:
    blocked = solve_trim_procedure(_procedure(), lambda state, controls, operating_point: _plant(dict(state), dict(controls)))
    assert blocked.status == "out_of_envelope"
    assert blocked.gate_results[0].status == "blocked"

    invalid = solve_trim_procedure(
        _procedure(),
        lambda state, controls, operating_point: {"missing": 0.0},
    )
    assert invalid.status == "adapter_invalid"
    assert invalid.failure_reason
    assert invalid.diagnostics[0].code == "missing-residual"


def test_trim_continuation_warm_starts_declared_operating_points() -> None:
    procedure = TrimProcedure(
        id="continuation-v1",
        vehicle="demo",
        fidelity="point_mass_3dof",
        spec=_spec(),
        operating_point={"speed_m_s": 0.0},
        continuation_axis="speed_m_s",
        continuation_values=(0.0, 1.0, 2.0),
    )

    def evaluator(state, controls, operating_point):
        speed = float(operating_point["speed_m_s"])
        return {"force": state["x"] + controls["u"] - 1.0 - speed, "moment": 2.0 * state["x"] - controls["u"]}

    result = solve_trim_continuation(procedure, evaluator)

    assert result.converged
    assert len(result.continuation) == 3
    assert result.continuation[-1].best is not None
    assert result.continuation[-1].best.state["x"] == pytest.approx(1.0 / 3.0 + 2.0 / 3.0)
