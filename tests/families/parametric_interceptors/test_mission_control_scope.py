"""Fail-closed Composition witnesses for mutually exclusive mission controls."""

from __future__ import annotations

from pathlib import Path

import pytest
from taoryx_parametric_interceptors import (
    DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,
    MISSION_CONTROL_SCOPE_CONTRACT,
    MISSION_TEMPLATE_ID,
    TARGET_TRACK_MISSION_TEMPLATE_ID,
    ParametricInterceptorMissionCompositionProvider,
    interceptor,
)
from taoryx_parametric_interceptors.__main__ import main

from taoryx.trajectory.configuration_contract import ConfigurationContractError

_EXAMPLES = Path("examples/parametric_interceptors")


def _provider() -> ParametricInterceptorMissionCompositionProvider:
    return ParametricInterceptorMissionCompositionProvider.from_profiles(interceptor("mission-control-scope-sam"))
    ####


@pytest.mark.parametrize(
    ("mission_template_id", "override", "owner_mission_template_id"),
    (
        (
            MISSION_TEMPLATE_ID,
            {"navigation_target_velocity_east_command": 50.0},
            TARGET_TRACK_MISSION_TEMPLATE_ID,
        ),
        (
            TARGET_TRACK_MISSION_TEMPLATE_ID,
            {"navigation_waypoint_east_command": 500.0},
            MISSION_TEMPLATE_ID,
        ),
        (
            DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,
            {"navigation_waypoint_north_command": 12_000.0},
            MISSION_TEMPLATE_ID,
        ),
        (
            MISSION_TEMPLATE_ID,
            {"control_lateral_acceleration_local_east_command": 10.0},
            DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,
        ),
    ),
)
def test_nondefault_inactive_mission_controls_fail_with_exact_owner(
    mission_template_id: str,
    override: dict[str, float],
    owner_mission_template_id: str,
) -> None:
    provider = _provider()
    configuration = provider.configuration(
        "mission-control-scope-sam",
        mission_template_id=mission_template_id,
        **override,
    )

    with pytest.raises(ConfigurationContractError) as captured:
        provider.validate_configuration(configuration)

    error = captured.value
    supplied_identifier = next(iter(override)).replace("_", ".")
    assert error.code == "inactive-mission-control"
    assert error.path.startswith("configuration.root.")
    assert owner_mission_template_id in str(error)
    assert "does not consume" in str(error)
    assert supplied_identifier.rsplit(".", 1)[-1] in error.path
    ####


def test_default_inactive_values_remain_portable_and_contract_is_advertised() -> None:
    provider = _provider()
    for mission_template_id in (
        MISSION_TEMPLATE_ID,
        TARGET_TRACK_MISSION_TEMPLATE_ID,
        DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,
    ):
        prepared = provider.validate_configuration(
            provider.configuration(
                "mission-control-scope-sam",
                mission_template_id=mission_template_id,
            )
        )
        assert prepared.configuration.mission_template_id == mission_template_id

    model = provider.model("mission-control-scope-sam")
    properties = {item.id: item.value for item in model.presentation.properties}
    assert properties["mission_control_scope.contract"] == MISSION_CONTROL_SCOPE_CONTRACT
    assert "inactive mission grammar fail" in model.realizations[0].controls.claim_boundary
    ####


def test_cli_reports_wrong_mission_controls_instead_of_running_them(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    destination = tmp_path / "must-not-exist.json"

    assert (
        main(
            [
                "run",
                str(_EXAMPLES / "generic_medium_sam.yaml"),
                "--mission-template",
                DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,
                "--set",
                "navigation.waypoint.east.command=500",
                "--output",
                str(destination),
            ]
        )
        == 2
    )
    assert "inactive-mission-control" in capsys.readouterr().err
    assert not destination.exists()
    ####
