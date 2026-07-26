from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.controller_design import ControllerDesignSpec, build_lqr_controller
from taoryx.controller_realization import (
    AllocationResult,
    ControllerProvenance,
    ControllerRuntimeState,
    ControllerSchedule,
    GeneralizedControlRequest,
    GuidanceReference,
    preflight_controller_realization,
)
from taoryx.trim import TrimSpec, solve_trim


def _controller():
    trim_spec = TrimSpec(
        state_names=("position", "velocity"),
        control_names=("acceleration",),
        residual_names=("equilibrium",),
        state_initial={"position": 0.0, "velocity": 0.0},
        control_initial={"acceleration": 0.0},
    )
    trim = solve_trim(trim_spec, lambda state, controls: {"equilibrium": state["position"] + controls["acceleration"]})
    design = ControllerDesignSpec(
        id="double-integrator-lqr",
        method="lqr",
        trim="double-integrator-trim-v1",
        allocator="double-integrator-allocator",
        states=("position", "velocity"),
        controls=("acceleration",),
        role="local_regulator",
        implementation_version="scaled-lqr-v1",
        fidelity="rigid_body_6dof",
        state_units=("m", "m/s"),
        state_frames=("inertial", "inertial"),
        control_units=("m/s^2",),
        control_frames=("inertial",),
        plant_source="synthetic-double-integrator",
        linearization_source="analytic-double-integrator-v1",
        q_id="double-integrator-q-v1",
        r_id="double-integrator-r-v1",
        state_scale_id="double-integrator-state-v1",
        control_scale_id="double-integrator-control-v1",
    )
    return build_lqr_controller(
        design,
        trim,
        ((0.0, 1.0), (0.0, 0.0)),
        ((0.0,), (1.0,)),
        ((1.0, 0.0), (0.0, 1.0)),
        ((1.0,),),
    )


def test_lqr_factory_attaches_reproducible_realization_contract() -> None:
    controller = _controller()

    assert controller.realization is not None
    realization = controller.realization
    assert realization.a_sha256 is not None
    assert realization.k_sha256 is not None
    report = preflight_controller_realization(
        realization,
        expected_state_names=("position", "velocity"),
        expected_input_names=("acceleration",),
        controllability_rank=2,
        connected_outputs=("allocator", "actuator", "plant"),
        required_outputs=("allocator", "actuator", "plant"),
    )
    assert report.passed
    assert len(report.realization_sha256) == 64


def test_controller_preflight_rejects_order_and_controllability() -> None:
    realization = _controller().realization
    assert realization is not None

    report = preflight_controller_realization(
        realization,
        expected_state_names=("velocity", "position"),
        controllability_rank=1,
    )

    assert not report.passed
    codes = {issue.code for issue in report.issues}
    assert {"state-order-mismatch", "uncontrollable-plant"}.issubset(codes)


def test_controller_preflight_rejects_hidden_fallback_and_gain_override() -> None:
    realization = _controller().realization
    assert realization is not None

    report = preflight_controller_realization(
        realization,
        fallback_active=True,
        scenario_overrides=("q-angle",),
    )

    assert not report.passed
    codes = {issue.code for issue in report.issues}
    assert {"hidden-fallback", "scenario-gain-override"}.issubset(codes)


def test_controller_preflight_rejects_schedule_outside_domain() -> None:
    realization = _controller().realization
    assert realization is not None
    scheduled = realization.model_copy(
        update={
            "schedule": ControllerSchedule(
                id="altitude-schedule-v1",
                coordinates=("mass_kg",),
                interpolation="linear",
                domains={"mass_kg": (1.0, 2.0)},
            )
        }
    )

    report = preflight_controller_realization(scheduled, schedule_point={"mass_kg": 3.0})

    assert not report.passed
    assert any(issue.code == "schedule-out-of-domain" for issue in report.issues)


def test_repository_controller_catalog_declares_realization_metadata() -> None:
    from taoryx.controller_design import load_controller_catalog

    catalog = load_controller_catalog(Path("verification/controller_designs.yaml"))
    for design in catalog.designs:
        assert design.role != "unspecified"
        assert design.implementation_version != "unversioned"
        assert design.plant_source != "unspecified"
        assert design.linearization_source != "unspecified"


def test_preflight_require_pass_raises_with_stable_codes() -> None:
    realization = _controller().realization
    assert realization is not None
    report = preflight_controller_realization(
        realization,
        connected_outputs=(),
        required_outputs=("allocator", "actuator", "plant"),
    )

    with pytest.raises(ValueError, match="disconnected-output"):
        report.require_pass()


def test_runtime_controller_state_keeps_provenance_and_committed_time_aligned() -> None:
    realization = _controller().realization
    assert realization is not None
    provenance = ControllerProvenance(
        architecture="lqr",
        design_id=realization.design_id,
        design_version=realization.implementation_version,
        design_hash=realization.digest(),
        runtime_backend="taoryx.runtime.lqr",
        active_trim_id="double-integrator-trim-v1",
        trim_source="test-trim",
        linear_model_id="double-integrator-linear-v1",
        state_scaling_id=realization.state_scale_id,
        control_scaling_id=realization.control_scale_id,
    )
    reference = GuidanceReference(
        time_s=1.0,
        segment_id="hold",
        source="test-guidance",
        values={"acceleration": 0.0},
        units={"acceleration": "m/s^2"},
        frames={"acceleration": "inertial"},
    )
    runtime = ControllerRuntimeState(
        time_s=1.0,
        provenance=provenance,
        active_mode="nominal",
        active_trim_id="double-integrator-trim-v1",
        reference=reference,
        requested_control=GeneralizedControlRequest(
            time_s=1.0,
            source="double-integrator-lqr",
            values={"acceleration": 0.0},
            units={"acceleration": "m/s^2"},
            frames={"acceleration": "inertial"},
        ),
        allocation=AllocationResult(
            allocator_id="double-integrator-allocator",
            requested={"acceleration": 0.0},
            allocated={"acceleration": 0.0},
            achieved={"acceleration": 0.0},
            residual={"acceleration": 0.0},
        ),
    )

    assert runtime.provenance.design_hash == realization.digest()
    assert runtime.requested_control.time_s == runtime.time_s


def test_runtime_controller_state_rejects_undeclared_schedule_coordinates() -> None:
    realization = _controller().realization
    assert realization is not None
    provenance = ControllerProvenance(
        architecture="lqr",
        design_id=realization.design_id,
        design_version=realization.implementation_version,
        design_hash=realization.digest(),
        runtime_backend="taoryx.runtime.lqr",
        active_trim_id="double-integrator-trim-v1",
        trim_source="test-trim",
        linear_model_id="double-integrator-linear-v1",
        state_scaling_id=realization.state_scale_id,
        control_scaling_id=realization.control_scale_id,
    )
    reference = GuidanceReference(
        time_s=1.0,
        segment_id="hold",
        source="test-guidance",
        values={"acceleration": 0.0},
        units={"acceleration": "m/s^2"},
        frames={"acceleration": "inertial"},
    )

    with pytest.raises(ValueError, match="schedule coordinates"):
        ControllerRuntimeState(
            time_s=1.0,
            provenance=provenance,
            active_mode="nominal",
            active_trim_id="double-integrator-trim-v1",
            schedule_coordinates={"mass_kg": 2.0},
            reference=reference,
            requested_control=GeneralizedControlRequest(
                time_s=1.0,
                source="double-integrator-lqr",
                values={"acceleration": 0.0},
                units={"acceleration": "m/s^2"},
                frames={"acceleration": "inertial"},
            ),
        )


def test_declared_scenario_override_is_fail_closed() -> None:
    realization = _controller().realization
    assert realization is not None
    declared = realization.model_copy(update={"scenario_gain_overrides": ("route-specific-q",)})
    report = preflight_controller_realization(declared)

    assert not report.passed
    assert any(issue.code == "scenario-gain-override" for issue in report.issues)


def test_qualified_controller_provenance_rejects_scenario_overrides() -> None:
    with pytest.raises(ValueError, match="qualified controller provenance"):
        ControllerProvenance(
            architecture="gain_scheduled_rslqr",
            design_id="design",
            design_version="1.0.0",
            design_hash="0" * 64,
            runtime_backend="test",
            active_trim_id="trim",
            trim_source="test",
            linear_model_id="linear",
            state_scaling_id="states",
            control_scaling_id="controls",
            scenario_gain_overrides=True,
            qualification_status="qualified",
        )


def test_preflight_rejects_qualified_realization_with_allowed_overrides() -> None:
    realization = _controller().realization
    assert realization is not None
    qualified = realization.model_copy(update={"claim_status": "qualified", "scenario_overrides_allowed": True})

    report = preflight_controller_realization(qualified)

    assert not report.passed
    assert any(issue.code == "qualified-controller-override" for issue in report.issues)


@pytest.mark.parametrize("implementation", ("rslqr", "gain_scheduled_lqr", "gain_scheduled_rslqr"))
def test_lqr_family_realizations_require_the_same_closed_loop_path(implementation: str) -> None:
    realization = _controller().realization
    assert realization is not None
    scheduled = realization.model_copy(update={"implementation": implementation})

    report = preflight_controller_realization(
        scheduled,
        connected_outputs=("allocator", "actuator", "plant"),
        required_outputs=("allocator", "actuator", "plant"),
    )

    assert report.passed
