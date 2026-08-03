from __future__ import annotations

from taoryx.family_adapter import (
    StandardFamilyAdapter,
    descriptor_from_control_plant,
    validate_family_adapter,
)
from taoryx.hl20_adapter import build_hl20_source_surface_adapter
from tools.validate_hummingbird_physical_lqr import build_plant as build_hummingbird_plant
from tools.validate_x8_physical_lqr import build_plant as build_x8_plant


def test_x8_source_table_plant_uses_the_common_surface_adapter_contract() -> None:
    plant = build_x8_plant()
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
    plant = build_hummingbird_plant()
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


def test_hl20_source_surface_adapter_exposes_partial_trim_evidence_without_full_trim() -> None:
    adapter = build_hl20_source_surface_adapter()
    fragment = adapter.trim_fragment({})

    assert fragment.status == "verified"
    assert fragment.fragment_id == "hl20_source_pitch_channel_trim"
    assert fragment.residuals["pitch_cm"] < 1.0e-10
    assert "not full 6-DOF equilibrium" in fragment.claim_boundary
    assert adapter.capability_report().capability("trim").status == "not_applicable"
