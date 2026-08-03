"""Real reduced-tier witnesses for the reusable tuning-campaign seam."""

from __future__ import annotations

from taoryx.control_allocation import ControlPlantAdapter
from taoryx.family_adapter import StandardFamilyAdapter, descriptor_from_control_plant
from taoryx.trajectory.a320_adapter import (
    A320Pseudo6DOFControlPlant,
    build_a320_pseudo_tuning_campaign,
)
from taoryx.trajectory.a320_pseudo6dof import A320Pseudo6DOFModel
from taoryx.trajectory.hummingbird_adapter import (
    HummingbirdPseudo6DOFControlPlant,
    build_hummingbird_pseudo_tuning_campaign,
)
from taoryx.tuning_campaign import run_tuning_campaign
from taoryx.vehicle_registry import ROOT


def _adapter(plant: ControlPlantAdapter, *, family_id: str, adapter_id: str, physical_family: str) -> StandardFamilyAdapter:
    """Wrap one reduced plant through the same family façade as runtime use."""

    descriptor = descriptor_from_control_plant(
        plant,
        family_id=family_id,
        adapter_id=adapter_id,
        physical_family=physical_family,
        tier="pseudo_6dof",
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)
    ####


def test_hummingbird_pseudo_campaign_is_a_closed_attitude_inner_loop() -> None:
    plant = HummingbirdPseudo6DOFControlPlant()
    report = run_tuning_campaign(
        _adapter(
            plant,
            family_id="hummingbird",
            adapter_id="taoryx.multirotor.native_quad_x.v1",
            physical_family="multirotor",
        ),
        build_hummingbird_pseudo_tuning_campaign(),
    )

    assert report.status == "candidate_ready"
    node = report.nodes[0]
    assert node.status == "candidate_ready"
    assert node.authority_preflight is not None and node.authority_preflight.status == "passed"
    assert node.derivative_metrics["maximum_omitted_state_coupling"] == 0.0
    assert node.lqr is not None and node.lqr.best is not None
    assert len(node.lqr.candidates) == 9
    ####


def test_a320_pseudo_campaign_reuses_the_fixed_wing_strategy() -> None:
    plant = A320Pseudo6DOFControlPlant(A320Pseudo6DOFModel.from_repository(ROOT))
    report = run_tuning_campaign(
        _adapter(
            plant,
            family_id="a320_openap_3dof",
            adapter_id="taoryx.fixed_wing.openap.v1",
            physical_family="powered_fixed_wing",
        ),
        build_a320_pseudo_tuning_campaign(),
    )

    assert report.status == "candidate_ready"
    node = report.nodes[0]
    assert node.status == "candidate_ready"
    assert node.authority_preflight is not None and node.authority_preflight.status == "passed"
    assert node.derivative_metrics["maximum_omitted_state_coupling"] < 1.0e-8
    assert node.lqr is not None and node.lqr.best is not None
    assert len(node.lqr.candidates) == 9
    ####
