from __future__ import annotations

from taoryx.hl20_adapter import build_hl20_source_surface_adapter
from taoryx.source_table_fixed_wing import build_x8_source_table_plant
from taoryx.source_table_multirotor import build_hummingbird_individual_rotor_source_table_plant
from taoryx.x15_adapter import build_x15_source_surface_local_adapter

from taoryx.family_adapter import (
    StandardFamilyAdapter,
    descriptor_from_control_plant,
    validate_family_adapter,
)


def test_x8_source_table_plant_uses_the_common_surface_adapter_contract() -> None:
    plant = build_x8_source_table_plant()
    descriptor = descriptor_from_control_plant(
        plant,
        family_id="skywalker_x8",
        adapter_id="skywalker-x8-table-coordinate-plant",
        physical_family="powered_fixed_wing",
        tier="rigid_body_6dof_surface_allocated",  # type: ignore[arg-type]
        control_units={name: plant.effector_limits[name].unit for name in plant.control_names},
        evidence_status="development",
        omitted_physics=("left-right-hardware-elevon-sign-resolution",),
    )

    adapter = StandardFamilyAdapter.from_control_plant(descriptor, plant)
    report = validate_family_adapter(adapter, expected_family_id="skywalker_x8")

    assert report.status == "pass"
    assert adapter.control_names == tuple(plant.control_names)
    assert adapter.capability_report().capability("allocate").usable


def test_hummingbird_source_rotor_plant_uses_the_same_surface_adapter_contract() -> None:
    plant = build_hummingbird_individual_rotor_source_table_plant()
    descriptor = descriptor_from_control_plant(
        plant,
        family_id="hummingbird",
        adapter_id="hummingbird-individual-rotor-source-plant",
        physical_family="multirotor",
        tier="rigid_body_6dof_surface_allocated",  # type: ignore[arg-type]
        control_units={name: plant.effector_limits[name].unit for name in plant.control_names},
        evidence_status="development",
    )

    adapter = StandardFamilyAdapter.from_control_plant(descriptor, plant)
    report = validate_family_adapter(adapter, expected_family_id="hummingbird")

    assert report.status == "pass"
    assert len(adapter.control_names) == 4
    assert adapter.capability_report().capability("effectiveness").usable


def test_hl20_source_surface_adapter_exposes_source_fragment_and_local_moment_trim() -> None:
    """The local surface adapter distinguishes a source fragment from its local trim."""

    adapter = build_hl20_source_surface_adapter()
    fragment = adapter.trim_fragment({})

    assert fragment.status == "verified"
    assert fragment.fragment_id == "hl20_source_pitch_channel_trim"
    assert fragment.residuals["pitch_cm"] < 1.0e-10
    assert "not full 6-DOF equilibrium" in fragment.claim_boundary
    assert adapter.capability_report().capability("trim").usable
    assert adapter.capability_report().capability("trim_fragment").usable


def test_x15_source_surface_adapter_exposes_local_effectors_and_allocation() -> None:
    """The X-15 local source-surface bridge is executable without full-flight claims."""

    adapter = build_x15_source_surface_local_adapter("rigid_body_6dof_surface_allocated")
    report = validate_family_adapter(adapter, expected_family_id="x15")

    assert report.status == "pass"
    assert adapter.control_names == ("symmetric_stabilator", "differential_stabilator", "rudder")
    assert adapter.capability_report().capability("effectiveness").usable
    assert adapter.capability_report().capability("allocate").usable
