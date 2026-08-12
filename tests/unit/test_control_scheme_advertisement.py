"""Cross-family control-scheme discovery without rigid-body qualification work."""

from __future__ import annotations

from typing import cast

from taoryx.trajectory.reference_mission_composition import (
    ReferenceMissionCompositionProvider,
)
from taoryx.trajectory.registry_mission_composition import (
    RegistryMissionCompositionProvider,
)

from taoryx.control_schemes import CONTROL_SCHEME_CATALOG
from taoryx.controller_tuning_registry import ControllerTuningCampaignRegistry
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.trajectory.configuration_contract import (
    ConfigurableTrajectoryProviderRegistry,
    TrajectoryControlSchemeSupportMetadata,
    TrajectoryModelMetadata,
)
from taoryx.trajectory.contract_probe_mission_composition import (
    ContractProbeMissionCompositionProvider,
)


def _support_by_authority(
    model: TrajectoryModelMetadata,
) -> dict[tuple[str, str], TrajectoryControlSchemeSupportMetadata]:
    return {
        (item.realization_id, item.authority_profile_id): item
        for item in model.control_scheme_support
    }
    ####


def test_every_published_authority_has_an_exact_ui_ready_support_record() -> None:
    providers = (
        RegistryMissionCompositionProvider(),
        ReferenceMissionCompositionProvider(),
        ContractProbeMissionCompositionProvider(),
    )
    for provider in providers:
        for model in provider.list_models():
            support = _support_by_authority(model)
            expected = {
                (realization.id, authority.id)
                for realization in model.realizations
                for authority in realization.controls.authorities
            }
            assert set(support) == expected
            for realization in model.realizations:
                for authority in realization.controls.authorities:
                    record = support[(realization.id, authority.id)]
                    assert record.action_ids == authority.channel_ids
                    assert record.operations == authority.operations
                    assert set(record.fidelity_ids) == set(realization.fidelity_aliases) & {
                        item.id for item in model.fidelities
                    }

            encoded = model.model_dump_json()
            restored = TrajectoryModelMetadata.model_validate_json(encoded)
            assert restored.control_scheme_support == model.control_scheme_support
    ####


def test_four_low_fidelity_pilots_declare_their_schemes_without_fallback() -> None:
    registry = RegistryMissionCompositionProvider()
    reference = ReferenceMissionCompositionProvider()
    probe = ContractProbeMissionCompositionProvider()
    models = {
        item.id: item
        for provider in (registry, reference, probe)
        for item in provider.list_models()
    }

    expected = {
        "reference_ballistic_3dof": {"open_loop.coast"},
        "reference_constant_velocity_waypoint_3dof": {
            "mission.waypoint",
            "kinematic.flight_path",
        },
        "contract_probe_vehicle": {
            "provider.program",
            "debug.mixed",
            "debug.discrete",
            "debug.event",
        },
        "simple_aero": {"provider.program", "kinematic.energy"},
    }
    for model_id, scheme_ids in expected.items():
        records = models[model_id].control_scheme_support
        assert {item.scheme_id for item in records} == scheme_ids
        assert {item.advertisement_source for item in records} == {"declared"}
        assert all(item.fidelity_ids for item in records)
    ####


def test_reduced_fixed_wing_keeps_common_schemes_across_fidelity_tiers() -> None:
    model = RegistryMissionCompositionProvider().model("f16_s119")
    reduced = tuple(
        item
        for item in model.control_scheme_support
        if item.realization_id in {"point_mass_3dof", "pseudo_6dof"}
    )
    by_realization = {
        realization_id: {
            item.scheme_id for item in reduced if item.realization_id == realization_id
        }
        for realization_id in ("point_mass_3dof", "pseudo_6dof")
    }
    common = {
        "mission.waypoint",
        "kinematic.flight_path",
        "pilot.normalized_axes",
    }
    assert by_realization["point_mass_3dof"] == common
    assert by_realization["pseudo_6dof"] == common | {"body_motion.body_rate"}
    ####


def test_model_plan_projects_only_the_selected_tier_and_realization() -> None:
    provider = RegistryMissionCompositionProvider()
    plan = build_model_authoring_plan(
        ConfigurableTrajectoryProviderRegistry((provider,)),
        ControllerTuningCampaignRegistry(),
        provider.metadata.id,
        "f16_s119",
        fidelity="point_mass_3dof",
        realization_id="point_mass_3dof",
    )
    controller = plan["controller_automation"]
    assert isinstance(controller, dict)
    support = cast(list[dict[str, object]], controller["control_scheme_support"])
    assert {item["scheme_id"] for item in support} == {
        "mission.waypoint",
        "kinematic.flight_path",
        "pilot.normalized_axes",
    }
    assert {
        tuple(cast(list[str], item["fidelity_ids"])) for item in support
    } == {
        ("point_mass_3dof",)
    }
    ####


def test_catalog_reserves_family_extensions_and_deprioritizes_physical_bridges() -> None:
    assert CONTROL_SCHEME_CATALOG["pilot.rotorcraft"].consumer_roles == (
        "human_pilot",
        "remote_operator",
        "autonomy",
    )
    assert CONTROL_SCHEME_CATALOG["mission.orbit_target"].layer == "mission"
    assert CONTROL_SCHEME_CATALOG["mission.relative_pose"].layer == "mission"
    assert CONTROL_SCHEME_CATALOG["kinematic.relative_motion"].layer == "kinematic"
    assert CONTROL_SCHEME_CATALOG["pilot.ground_vehicle"].layer == "pilot"
    assert CONTROL_SCHEME_CATALOG["pilot.marine"].layer == "pilot"
    assert CONTROL_SCHEME_CATALOG["wrench.direct"].streaming_preference == "diagnostic"
    assert CONTROL_SCHEME_CATALOG["effector.direct"].streaming_preference == "diagnostic"
    ####
