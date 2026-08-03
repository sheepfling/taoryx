from __future__ import annotations

import pytest

from taoryx.family_adapter import AdapterChannel, FamilyAdapterDescriptor, StandardFamilyAdapter
from taoryx.showcase.adapter_binding import build_showcase_realization_from_adapter


def _adapter(*, tier: str = "rigid_body_6dof_surface_allocated") -> StandardFamilyAdapter:
    descriptor = FamilyAdapterDescriptor(
        family_id="synthetic_x8",
        adapter_id="synthetic.x8.adapter",
        physical_family="powered_fixed_wing",
        tier=tier,  # type: ignore[arg-type]
        state_channels=(AdapterChannel("u_m_s", "m/s", "state", frame="body"),),
        control_channels=(AdapterChannel("elevon_left_deg", "deg", "effector", frame="body"),),
    )
    return StandardFamilyAdapter.from_state_derivative(
        descriptor,
        lambda state, effectors, environment: {"u_m_s": 0.0},
    )


def test_adapter_binding_copies_canonical_surface_contract() -> None:
    realization = build_showcase_realization_from_adapter(
        _adapter(),
        claim="Synthetic surface-allocated response inside the declared envelope.",
        evidence_grade="synthetic",
        required_operations=("state_derivative",),
    )

    assert realization.fidelity == "rigid_body_6dof_surface_allocated"
    assert realization.control_realization == "surface_allocated"
    assert realization.state_schema == ("u_m_s",)
    assert realization.physical_effectors == ("elevon_left_deg",)


def test_adapter_binding_never_turns_direct_wrench_channels_into_effectors() -> None:
    descriptor = FamilyAdapterDescriptor(
        family_id="synthetic_x15",
        adapter_id="synthetic.x15.direct",
        physical_family="powered_fixed_wing",
        tier="rigid_body_6dof_direct_wrench",
        state_channels=(AdapterChannel("q_rad_s", "rad/s", "state", frame="body"),),
        control_channels=(AdapterChannel("moment_y_nm", "N*m", "direct_wrench", frame="body"),),
    )
    adapter = StandardFamilyAdapter.from_state_derivative(
        descriptor,
        lambda state, effectors, environment: {"q_rad_s": 0.0},
    )

    realization = build_showcase_realization_from_adapter(
        adapter,
        claim="Synthetic direct-wrench bridge response.",
        evidence_grade="derived",
    )

    assert realization.fidelity == "rigid_body_6dof_direct_wrench"
    assert realization.control_realization == "direct_wrench"
    assert realization.physical_effectors == ()


def test_adapter_binding_preflights_required_operations() -> None:
    with pytest.raises(ValueError, match=r"trim \(not_applicable\)"):
        build_showcase_realization_from_adapter(
            _adapter(),
            claim="Should not build without trim.",
            evidence_grade="synthetic",
            required_operations=("trim",),
        )
