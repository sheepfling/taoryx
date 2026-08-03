from __future__ import annotations

from collections.abc import Mapping

import pytest

from taoryx.control_allocation import (
    EffectorEffectiveness,
    PhysicalAllocationStep,
    ProvenancedLinearization,
)
from taoryx.family_adapter import AdapterChannel, FamilyAdapterDescriptor, StandardFamilyAdapter
from taoryx.family_adapter_probes import AdapterProbeOperation, AdapterProbeReport
from taoryx.family_adapter_registry import (
    AdapterRegistrationCheck,
    AdapterRegistrationError,
    AdapterRegistryReport,
    FamilyAdapterRegistration,
    FamilyAdapterRegistry,
)
from taoryx.trim import TrimResult


class _Plant:
    state_names = ("x_m",)
    control_names = ("elevon",)

    def state_derivative(self, state: Mapping[str, float], effectors: Mapping[str, float], environment: Mapping[str, float | str]) -> Mapping[str, float]:
        del state, effectors, environment
        return {"x_m": 0.0}

    def trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        del target, initial_guess
        raise NotImplementedError

    def linearize(self, trim: TrimResult, options: Mapping[str, float | str]) -> ProvenancedLinearization:
        del trim, options
        raise NotImplementedError

    def effectiveness(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> EffectorEffectiveness:
        del state, effectors
        raise NotImplementedError

    def allocate(self, state: Mapping[str, float], desired_wrench: Mapping[str, float], previous_effectors: Mapping[str, float], dt_s: float) -> PhysicalAllocationStep:
        del state, desired_wrench, previous_effectors, dt_s
        raise NotImplementedError


def _adapter(tier: str = "rigid_body_6dof_surface_allocated") -> StandardFamilyAdapter:
    plant = _Plant()
    descriptor = FamilyAdapterDescriptor(
        family_id="test_family",
        adapter_id="test.adapter.v1",
        physical_family="powered_fixed_wing",
        tier=tier,  # type: ignore[arg-type]
        state_channels=(AdapterChannel("x_m", "m", "state"),),
        control_channels=(AdapterChannel("elevon", "rad", "effector"),),
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)


def test_registry_builds_and_checks_a_family_tier() -> None:
    registry = FamilyAdapterRegistry(
        (
            FamilyAdapterRegistration(
                "test_family",
                "test.adapter.v1",
                "available",
                lambda tier: _adapter(tier),
            ),
        )
    )

    check = registry.check("test_family", "rigid_body_6dof_surface_allocated")
    assert check.status == "pass"
    assert check.conformance is not None
    assert registry.check_all().status == "pass"
    matrix = registry.check_matrix()
    assert len(matrix.checks) == 4
    assert {item.tier for item in matrix.checks} == {
        "point_mass_3dof",
        "pseudo_6dof",
        "rigid_body_6dof_direct_wrench",
        "rigid_body_6dof_surface_allocated",
    }


def test_registry_rejects_factory_identity_or_tier_drift() -> None:
    registry = FamilyAdapterRegistry(
        (
            FamilyAdapterRegistration(
                "test_family",
                "test.adapter.v1",
                "available",
                lambda tier: _adapter("rigid_body_6dof_direct_wrench"),
            ),
        )
    )

    check = registry.check("test_family", "rigid_body_6dof_surface_allocated")
    assert check.status == "blocked"
    assert "returned tier" in check.message
    with pytest.raises(AdapterRegistrationError, match="returned tier"):
        registry.build("test_family", "rigid_body_6dof_surface_allocated")


def test_planned_registration_is_visible_without_being_called() -> None:
    registry = FamilyAdapterRegistry(
        (
            FamilyAdapterRegistration(
                "future_family",
                "future.adapter.v1",
                "planned",
                note="plant adapter is not registered yet",
            ),
        )
    )

    check = registry.check("future_family", "pseudo_6dof")
    assert check.status == "development"
    assert "not registered yet" in check.message


def test_duplicate_family_registration_is_rejected() -> None:
    registry = FamilyAdapterRegistry()
    registration = FamilyAdapterRegistration("test_family", "test.adapter.v1", "planned")
    registry.register(registration)
    with pytest.raises(ValueError, match="already exists"):
        registry.register(registration)


def test_factory_failure_is_returned_as_structured_blocked_status() -> None:
    registry = FamilyAdapterRegistry(
        (
            FamilyAdapterRegistration(
                "broken_family",
                "broken.adapter.v1",
                "available",
                lambda tier: (_ for _ in ()).throw(RuntimeError("source asset missing")),
            ),
        )
    )

    check = registry.check("broken_family", "rigid_body_6dof_surface_allocated")
    assert check.status == "blocked"
    assert "source asset missing" in check.message


def test_registry_report_exposes_operation_status_for_lowering() -> None:
    probe = AdapterProbeReport(
        "test_family",
        "test.adapter.v1",
        "rigid_body_6dof_surface_allocated",
        "pass",
        (
            AdapterProbeOperation("trim", "pass", "trim passed"),
            AdapterProbeOperation("allocate", "blocked", "allocator blocked"),
        ),
    )
    report = AdapterRegistryReport(
        (
            AdapterRegistrationCheck(
                "test_family",
                "test.adapter.v1",
                "rigid_body_6dof_surface_allocated",
                "pass",
                None,
                probe,
                "probe passed",
            ),
        )
    )

    status = report.operation_status_by_family_tier()

    assert status["test_family"]["rigid_body_6dof_surface_allocated"] == {
        "trim": "pass",
        "allocate": "blocked",
    }
