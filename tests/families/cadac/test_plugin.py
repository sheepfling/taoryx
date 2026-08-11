from __future__ import annotations

import pytest
from taoryx.families.cadac.plugin import CADAC_PLUGIN_CATALOG, CadacPluginRegistry


def test_catalog_projects_every_manifest_actor() -> None:
    assert len(CADAC_PLUGIN_CATALOG.plugins) == 19
    assert len({item.plugin_id for item in CADAC_PLUGIN_CATALOG.plugins}) == 19
    ####


####


def test_aim5_missile_is_first_runnable_plugin() -> None:
    missile = CADAC_PLUGIN_CATALOG.plugin("cadac.aim5.missile")
    target = CADAC_PLUGIN_CATALOG.plugin("cadac.aim5.target")
    assert missile.status == "runnable"
    assert missile.operations == ("discover", "validate", "batch")
    assert missile.batch_factory_id == "cadac.aim5.source_compatibility.batch"
    assert target.status == "embedded"
    assert target.embedded_in_model_ids == ("cadac.aim5.missile",)
    ####


####


def test_mixed_fidelity_actor_keeps_phase_claims() -> None:
    rocket = CADAC_PLUGIN_CATALOG.plugin("cadac.rocket6g.launch_vehicle")
    assert {phase.phase_id for phase in rocket.phases} == {
        "aggregate_rcs",
        "physical_tvc",
        "mixed_tvc_rcs",
        "ballistic_coast",
    }
    assert {phase.taoryx_tier for phase in rocket.phases} == {
        "rigid_body_6dof_direct_wrench",
        "rigid_body_6dof_surface_allocated",
    }
    ####


####


def test_registry_does_not_infer_runtime_from_discovery() -> None:
    registry = CadacPluginRegistry(CADAC_PLUGIN_CATALOG.plugins)
    with pytest.raises(KeyError, match="discoverable but has no installed runtime"):
        registry.runtime("cadac.falcon6.aircraft")
    ####


####


def test_falcon6_is_second_runnable_actor_plugin() -> None:
    falcon = CADAC_PLUGIN_CATALOG.plugin("cadac.falcon6.aircraft")
    assert falcon.status == "runnable"
    assert falcon.operations == ("discover", "validate", "batch")
    assert falcon.batch_factory_id == "cadac.falcon6.physical_surface.batch"
    assert "guidance" in falcon.claim_boundary


####


def test_cruise5_is_runnable_pseudo6dof_actor_plugin() -> None:
    cruise = CADAC_PLUGIN_CATALOG.plugin("cadac.cruise5.cruise_vehicle")
    assert cruise.status == "runnable"
    assert cruise.source_model == "CRUISE3"
    assert cruise.batch_factory_id == "cadac.cruise5.source_compatibility.batch"
    assert {phase.taoryx_tier for phase in cruise.phases} == {"point_mass_3dof", "pseudo_6dof"}


####


def test_ghame3_is_source_grounded_point_mass_actor_plugin() -> None:
    ghame = CADAC_PLUGIN_CATALOG.plugin("cadac.ghame3.hypersonic_vehicle")
    assert ghame.status == "runnable"
    assert ghame.source_model == "CRUISE3"
    assert ghame.operations == ("discover", "validate", "batch")
    assert ghame.batch_factory_id == "cadac.ghame3.source_compatibility.batch"
    assert len(ghame.phases) == 1
    assert ghame.phases[0].phase_id == "source_model"
    assert ghame.phases[0].taoryx_tier == "point_mass_3dof"


####


def test_known_source_actor_class_names_are_preserved() -> None:
    assert CADAC_PLUGIN_CATALOG.plugin("cadac.falcon6.aircraft").source_model == "PLANE6"
    assert CADAC_PLUGIN_CATALOG.plugin("cadac.rocket6g.launch_vehicle").source_model == "HYPER6"
    assert CADAC_PLUGIN_CATALOG.plugin("cadac.sraam6.missile").source_model == "MISSILE6"
    assert CADAC_PLUGIN_CATALOG.plugin("cadac.sraam6.target").source_model == "TARGET3"


####


def test_rocket6g_is_runnable_phase_aware_mixed_effector_plugin() -> None:
    rocket = CADAC_PLUGIN_CATALOG.plugin("cadac.rocket6g.launch_vehicle")
    assert rocket.status == "runnable"
    assert rocket.operations == ("discover", "validate", "batch")
    assert rocket.batch_factory_id == "cadac.rocket6g.phase_aware.batch"
    assert rocket.default_phase_id == "mixed_tvc_rcs"
    assert rocket.blockers == ()
    assert "phase fidelity is emitted per sample" in rocket.claim_boundary


####


def test_sraam6_is_runnable_physical_fin_plugin_with_embedded_target() -> None:
    missile = CADAC_PLUGIN_CATALOG.plugin("cadac.sraam6.missile")
    target = CADAC_PLUGIN_CATALOG.plugin("cadac.sraam6.target")

    assert missile.status == "runnable"
    assert missile.operations == ("discover", "validate", "batch")
    assert missile.batch_factory_id == "cadac.sraam6.standard_fin.batch"
    assert missile.default_phase_id == "fin_control"
    assert target.status == "embedded"
    assert target.embedded_in_model_ids == ("cadac.sraam6.missile",)


####


def test_agm6_is_runnable_physical_fin_plugin_with_embedded_track_aircraft_and_target() -> None:
    missile = CADAC_PLUGIN_CATALOG.plugin("cadac.agm6.missile")
    aircraft = CADAC_PLUGIN_CATALOG.plugin("cadac.agm6.aircraft")
    target = CADAC_PLUGIN_CATALOG.plugin("cadac.agm6.ground_target")

    assert missile.status == "runnable"
    assert missile.source_model == "MISSILE6"
    assert missile.batch_factory_id == "cadac.agm6.standard_fin.batch"
    assert missile.default_phase_id == "fin_control"
    assert aircraft.status == "embedded"
    assert aircraft.scope.value == "embedded_vehicle"
    assert target.status == "embedded"
    assert aircraft.embedded_in_model_ids == ("cadac.agm6.missile",)
    assert target.embedded_in_model_ids == ("cadac.agm6.missile",)


####


def test_ghame6_is_phase_aware_runnable_without_invented_tvc() -> None:
    vehicle = CADAC_PLUGIN_CATALOG.plugin("cadac.ghame6.hypersonic_vehicle")
    satellite = CADAC_PLUGIN_CATALOG.plugin("cadac.ghame6.satellite")
    radar = CADAC_PLUGIN_CATALOG.plugin("cadac.ghame6.ground_site")

    assert vehicle.status == "runnable"
    assert vehicle.batch_factory_id == "cadac.ghame6.phase_aware.batch"
    assert vehicle.default_phase_id == "atmospheric_surfaces"
    assert {phase.taoryx_tier for phase in vehicle.phases} == {
        "rigid_body_6dof_surface_allocated",
        "rigid_body_6dof_direct_wrench",
    }
    assert "no TVC" in vehicle.claim_boundary
    assert satellite.status == "embedded"
    assert satellite.embedded_in_model_ids == ("cadac.ghame6.hypersonic_vehicle",)
    assert radar.status == "static"
    assert not radar.trajectory_capable


####


def test_ads6_dynamic_actors_are_runnable_while_radar_remains_static() -> None:
    sam = CADAC_PLUGIN_CATALOG.plugin("cadac.ads6.sam")
    srbm = CADAC_PLUGIN_CATALOG.plugin("cadac.ads6.srbm")
    aircraft = CADAC_PLUGIN_CATALOG.plugin("cadac.ads6.aircraft")
    radar = CADAC_PLUGIN_CATALOG.plugin("cadac.ads6.radar")

    assert sam.status == "runnable"
    assert sam.batch_factory_id == "cadac.ads6.sam.multi_realization.batch"
    assert sam.default_phase_id == "fin_control"
    assert {phase.taoryx_tier for phase in sam.phases} == {
        "rigid_body_6dof_surface_allocated",
        "rigid_body_6dof_direct_wrench",
    }
    assert srbm.status == "runnable"
    assert srbm.source_model == "ROCKET5"
    assert srbm.batch_factory_id == "cadac.ads6.srbm.source_compatibility.batch"
    assert srbm.default_phase_id == "source_model"
    assert {phase.taoryx_tier for phase in srbm.phases} == {"pseudo_6dof"}
    assert aircraft.status == "runnable"
    assert aircraft.source_model == "AIRCRAFT3"
    assert aircraft.batch_factory_id == "cadac.ads6.aircraft.source_compatibility.batch"
    assert aircraft.default_phase_id == "source_model"
    assert {phase.taoryx_tier for phase in aircraft.phases} == {"point_mass_3dof"}
    assert radar.status == "static"


####
