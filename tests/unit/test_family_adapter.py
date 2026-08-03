from __future__ import annotations

from collections.abc import Mapping

import pytest

from taoryx.control_allocation import (
    EffectorEffectiveness,
    EffectorLimits,
    PhysicalAllocationStep,
    ProvenancedLinearization,
    allocate_and_advance_wrench,
)
from taoryx.family_adapter import (
    AdapterCapability,
    AdapterCapabilityError,
    AdapterChannel,
    FamilyAdapterDescriptor,
    StandardFamilyAdapter,
    TrimFragmentResult,
    descriptor_from_control_plant,
    validate_family_adapter,
)
from taoryx.trim import TrimResult


class _Plant:
    state_names = ("x_m",)
    control_names = ("elevon",)

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        del state, effectors, environment
        return {"x_m": 1.0}

    def trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        del target, initial_guess
        raise NotImplementedError

    def linearize(self, trim: TrimResult, options: Mapping[str, float | str]) -> ProvenancedLinearization:
        del trim, options
        raise NotImplementedError

    def effectiveness(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> EffectorEffectiveness:
        del state, effectors
        raise NotImplementedError

    def allocate(
        self,
        state: Mapping[str, float],
        desired_wrench: Mapping[str, float],
        previous_effectors: Mapping[str, float],
        dt_s: float,
    ) -> PhysicalAllocationStep:
        del state, desired_wrench, previous_effectors, dt_s
        raise NotImplementedError


def _descriptor(*, tier: str = "rigid_body_6dof_surface_allocated", passive: bool = False) -> FamilyAdapterDescriptor:
    return FamilyAdapterDescriptor(
        family_id="test_x8",
        adapter_id="test.x8.adapter",
        physical_family="powered_fixed_wing",
        tier=tier,  # type: ignore[arg-type]
        state_channels=(AdapterChannel("x_m", "m", "state", frame="body"),),
        control_channels=() if passive else (AdapterChannel("elevon", "rad", "effector", frame="body"),),
        resource_channels=(AdapterChannel("fuel_kg", "kg", "resource"),),
        control_realization_override="uncontrolled" if passive else None,
        omitted_physics=("aerodynamic_tables",),
    )


def test_control_plant_is_wrapped_without_changing_its_channel_order() -> None:
    plant = _Plant()
    descriptor = descriptor_from_control_plant(
        plant,
        family_id="test_x8",
        adapter_id="test.x8.adapter",
        physical_family="powered_fixed_wing",
        tier="rigid_body_6dof_surface_allocated",  # type: ignore[arg-type]
        state_units={"x_m": "m"},
        control_units={"elevon": "rad"},
        resource_channels=(AdapterChannel("fuel_kg", "kg", "resource"),),
    )
    adapter = StandardFamilyAdapter.from_control_plant(descriptor, plant)

    assert adapter.describe().control_realization == "surface_allocated"
    assert adapter.state_schema()[0].name == "x_m"
    assert adapter.control_names == ("elevon",)
    assert adapter.resource_schema()[0].name == "fuel_kg"
    assert adapter.state_derivative({}, {}, {}) == {"x_m": 1.0}
    assert adapter.capability_report().capability("allocate").status == "available"
    assert validate_family_adapter(adapter, expected_family_id="test_x8").status == "pass"


def test_direct_wrench_wrapper_does_not_promote_physical_allocation() -> None:
    adapter = StandardFamilyAdapter.from_control_plant(
        _descriptor(tier="rigid_body_6dof_direct_wrench"),
        _Plant(),
    )

    assert adapter.describe().control_realization == "direct_wrench"
    assert adapter.capability_report().capability("allocate").status == "not_applicable"
    assert validate_family_adapter(adapter).status == "pass"


def test_passive_adapter_reports_uncontrolled_and_rejects_control_operations() -> None:
    adapter = StandardFamilyAdapter.passive(_descriptor(tier="rigid_body_6dof_direct_wrench", passive=True))
    report = adapter.capability_report()

    assert report.descriptor.control_realization == "uncontrolled"
    assert report.capability("allocate").status == "not_applicable"
    assert validate_family_adapter(adapter).status == "pass"
    with pytest.raises(AdapterCapabilityError, match="passive or open-loop"):
        adapter.allocate({}, {}, {}, 0.1)


def test_state_only_adapter_preserves_passive_truth_without_fabricating_controls() -> None:
    descriptor = _descriptor(tier="rigid_body_6dof_direct_wrench", passive=True)
    adapter = StandardFamilyAdapter.from_state_derivative(
        descriptor,
        lambda state, effectors, environment: {"x_m": state.get("x_m", 0.0) + len(effectors) + len(environment)},
    )

    report = validate_family_adapter(adapter)
    assert report.status == "pass"
    assert adapter.state_derivative({"x_m": 2.0}, {}, {}) == {"x_m": 2.0}
    assert report.errors == ()
    assert adapter.capability_report().capability("allocate").status == "not_applicable"


def test_provider_backed_partial_adapter_can_expose_effectivity_without_trim() -> None:
    descriptor = _descriptor()
    effectiveness = EffectorEffectiveness(
        wrench_names=("moment_z_nm",),
        effector_names=("elevon",),
        matrix=((2.0,),),
        reference_effectors={"elevon": 0.0},
        source="test-source-effectivity",
    )
    limits = {"elevon": EffectorLimits("elevon", -1.0, 1.0, "rad", rate_limit_per_s=10.0)}

    def provide_effectiveness(state: Mapping[str, float], effectors: Mapping[str, float]) -> EffectorEffectiveness:
        del state, effectors
        return effectiveness

    def provide_allocation(
        state: Mapping[str, float],
        desired: Mapping[str, float],
        previous: Mapping[str, float],
        dt_s: float,
    ) -> PhysicalAllocationStep:
        del state
        return allocate_and_advance_wrench(effectiveness, limits, desired, previous, dt_s)

    adapter = StandardFamilyAdapter.from_state_derivative(
        descriptor,
        lambda state, effectors, environment: {"x_m": 0.0},
        effectiveness_provider=provide_effectiveness,
        allocation_provider=provide_allocation,
    )

    assert adapter.capability_report().capability("trim").status == "not_applicable"
    assert adapter.capability_report().capability("effectiveness").status == "available"
    assert adapter.effectiveness({}, {}) is effectiveness
    result = adapter.allocate({}, {"moment_z_nm": 1.0}, {"elevon": 0.0}, 0.1)
    assert result.allocation.status in {"feasible", "feasible_near_limit"}
    assert validate_family_adapter(adapter).status == "pass"


def test_provider_backed_partial_adapter_can_expose_trim_fragment_without_full_trim() -> None:
    descriptor = _descriptor(tier="rigid_body_6dof_direct_wrench")
    fragment = TrimFragmentResult(
        fragment_id="test.pitch-channel",
        status="verified",
        state={"alpha_deg": 5.0},
        controls={},
        residuals={"pitch_cm": 1.0e-12},
        max_residual=1.0e-12,
        claim_boundary="scalar pitch-channel equilibrium only",
        operating_point={"mach": 1.0},
        provenance={"source": "test"},
    )
    adapter = StandardFamilyAdapter.from_state_derivative(
        descriptor,
        lambda state, effectors, environment: {"x_m": 0.0},
        trim_fragment_provider=lambda request: fragment,
    )

    report = adapter.capability_report()
    assert report.capability("trim").status == "not_applicable"
    assert report.capability("trim_fragment").status == "available"
    assert adapter.trim_fragment({"mach": 1.0}) is fragment
    assert validate_family_adapter(adapter).status == "pass"


def test_missing_capabilities_are_not_silently_inferred() -> None:
    descriptor = _descriptor()
    adapter = StandardFamilyAdapter(
        descriptor,
        _Plant(),
        {"state_derivative": AdapterCapability("state_derivative", "available", "test")},
    )

    assert adapter.capability_report().capability("resource_rates").status == "not_available"
    with pytest.raises(AdapterCapabilityError, match="not_available"):
        adapter.trim({}, {})


def test_optional_resource_and_observation_providers_use_the_same_facade() -> None:
    plant = _Plant()
    descriptor = descriptor_from_control_plant(
        plant,
        family_id="test_x8",
        adapter_id="test.x8.adapter",
        physical_family="powered_fixed_wing",
        tier="rigid_body_6dof_surface_allocated",  # type: ignore[arg-type]
    )
    adapter = StandardFamilyAdapter.from_control_plant(
        descriptor,
        plant,
        resource_rate_provider=lambda state, commands: {"fuel_kg_s": abs(commands.get("throttle", 0.0))},
        observation_provider=lambda state, controls, resources, events: {
            "state_count": len(state),
            "control_count": len(controls),
            "resource_count": len(resources),
            "event_count": len(events),
        },
    )

    assert adapter.resource_rates({}, {"throttle": 0.5}) == {"fuel_kg_s": 0.5}
    assert adapter.observe({}, {}, {}, ()) == {
        "state_count": 0,
        "control_count": 0,
        "resource_count": 0,
        "event_count": 0,
    }
    assert validate_family_adapter(adapter).status == "pass"


def test_conformance_fails_when_surface_claim_has_no_allocator() -> None:
    adapter = StandardFamilyAdapter(
        _descriptor(),
        _Plant(),
        {
            "state_derivative": AdapterCapability("state_derivative", "available", "test"),
            "trim": AdapterCapability("trim", "available", "test"),
            "linearize": AdapterCapability("linearize", "available", "test"),
        },
    )

    report = validate_family_adapter(adapter)
    assert report.status == "fail"
    assert {item.code for item in report.errors} == {"surface-capability-missing"}


def test_descriptor_rejects_duplicate_channels() -> None:
    with pytest.raises(ValueError, match="state channel names must be unique"):
        FamilyAdapterDescriptor(
            family_id="test",
            adapter_id="test",
            physical_family="test",
            tier="pseudo_6dof",  # type: ignore[arg-type]
            state_channels=(AdapterChannel("x", "m", "state"), AdapterChannel("x", "m", "state")),
        )
