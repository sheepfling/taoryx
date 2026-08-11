from __future__ import annotations

import pytest

from taoryx.control_automation import ControlAutomationDeclaration


def test_auto_declaration_selects_lqi_for_offset_free_outputs() -> None:
    declaration = ControlAutomationDeclaration(
        id="synthetic-attitude",
        campaign_id="synthetic-attitude-v1",
        family_id="synthetic",
        tier="pseudo_6dof",
        strategy_id="synthetic.v1",
        node_id="hover",
        state_scales={"angle": 0.2, "rate": 1.0},
        control_scales={"command": 0.3},
        offset_free_outputs=("angle",),
    )

    campaign = declaration.build_campaign()
    node = campaign.nodes[0]
    assert declaration.controller_method == "lqi"
    assert node.controller_method == "lqi"
    assert node.integral_output_names == ("angle",)
    assert node.integral_q_diagonal == (8.0,)
    assert node.design_state_names == ("angle", "rate")
    ####


def test_auto_declaration_selects_lqr_without_offset_free_intent() -> None:
    declaration = ControlAutomationDeclaration(
        id="synthetic-rate",
        campaign_id="synthetic-rate-v1",
        family_id="synthetic",
        tier="pseudo_6dof",
        strategy_id="synthetic.v1",
        node_id="rate",
        state_scales={"rate": 1.0},
        control_scales={"command": 0.3},
    )

    node = declaration.build_campaign().nodes[0]
    assert declaration.controller_method == "lqr"
    assert node.controller_method == "lqr"
    assert node.integral_output_names == ()
    ####


def test_auto_declaration_rejects_unknown_offset_free_output() -> None:
    with pytest.raises(ValueError, match="offset-free outputs"):
        ControlAutomationDeclaration(
            id="bad",
            campaign_id="bad-v1",
            family_id="synthetic",
            tier="pseudo_6dof",
            strategy_id="synthetic.v1",
            node_id="bad",
            state_scales={"rate": 1.0},
            control_scales={"command": 0.3},
            offset_free_outputs=("position",),
        )
    ####


def test_auto_declaration_can_preserve_a_public_profile_grid_identity() -> None:
    declaration = ControlAutomationDeclaration(
        id="renamed-internal-declaration",
        campaign_id="stable-campaign-v1",
        family_id="synthetic",
        tier="pseudo_6dof",
        strategy_id="synthetic.v1",
        node_id="rate",
        state_scales={"rate": 1.0},
        control_scales={"command": 0.3},
        profile_grid_id_prefix="stable-public-profile",
    )

    node = declaration.build_campaign().nodes[0]
    assert node.profile_grid is not None
    assert node.profile_grid.id_prefix == "stable-public-profile"
    ####


def test_auto_declaration_can_pin_a_source_screen_weight_contract() -> None:
    """A plug-in can retain a source-screen Q/R baseline without owning a tuner loop."""

    declaration = ControlAutomationDeclaration(
        id="source-screen",
        campaign_id="source-screen-v1",
        family_id="synthetic",
        tier="pseudo_6dof",
        strategy_id="synthetic.v1",
        node_id="trim",
        state_scales={"angle": 0.2, "rate": 1.0},
        control_scales={"command": 0.3},
        state_weight_multipliers=(1.0,),
        control_effort_multipliers=(1.0,),
        state_base_weights=(2.0, 3.0),
        control_base_weights=(4.0,),
    )

    grid = declaration.build_campaign().nodes[0].profile_grid

    assert grid is not None
    assert grid.profiles(2, 1)[0].q_diagonal == (2.0, 3.0)
    assert grid.profiles(2, 1)[0].r_diagonal == (4.0,)
    ####


def test_auto_declaration_can_sweep_integral_priority_for_lqi() -> None:
    """Plug-ins can request offset-rejection profiles without owning a tuner loop."""

    declaration = ControlAutomationDeclaration(
        id="synthetic-offset-rejection",
        campaign_id="synthetic-offset-rejection-v1",
        family_id="synthetic",
        tier="pseudo_6dof",
        strategy_id="synthetic.v1",
        node_id="trim",
        state_scales={"position": 1.0, "velocity": 1.0},
        control_scales={"force": 1.0},
        offset_free_outputs=("position",),
        integral_weight_multiplier=2.0,
        integral_weight_multipliers=(1.0, 10.0),
    )

    node = declaration.build_campaign().nodes[0]

    assert node.integral_q_diagonal == (2.0,)
    assert node.profile_grid is not None
    assert node.profile_grid.integral_weight_multipliers == (1.0, 10.0)
    ####


def test_auto_declaration_can_preserve_nonuniform_source_lqi_integral_weights() -> None:
    """Exact physical screens retain their declared output-priority contract."""

    declaration = ControlAutomationDeclaration(
        id="synthetic-source-lqi",
        campaign_id="synthetic-source-lqi-v1",
        family_id="synthetic",
        tier="pseudo_6dof",
        strategy_id="synthetic.v1",
        node_id="trim",
        state_scales={"roll": 1.0, "vertical_speed": 1.0},
        control_scales={"force": 1.0},
        offset_free_outputs=("roll", "vertical_speed"),
        integral_base_weights=(40.0, 16.0),
    )

    node = declaration.build_campaign().nodes[0]

    assert node.integral_q_diagonal == (40.0, 16.0)
    assert declaration.as_dict()["integral_base_weights"] == [40.0, 16.0]
    ####


def test_lqr_declaration_rejects_integral_base_weights() -> None:
    with pytest.raises(ValueError, match="integral base weights"):
        ControlAutomationDeclaration(
            id="synthetic-lqr",
            campaign_id="synthetic-lqr-v1",
            family_id="synthetic",
            tier="pseudo_6dof",
            strategy_id="synthetic.v1",
            node_id="trim",
            state_scales={"rate": 1.0},
            control_scales={"moment": 1.0},
            integral_base_weights=(2.0,),
        )
    ####


def test_auto_declaration_can_select_a_closed_design_subsystem() -> None:
    declaration = ControlAutomationDeclaration(
        id="synthetic-sidecar",
        campaign_id="synthetic-sidecar-v1",
        family_id="synthetic",
        tier="pseudo_6dof",
        strategy_id="synthetic.v1",
        node_id="cruise",
        state_scales={"altitude": 10.0, "bank": 5.0, "observed_yaw": 10.0},
        control_scales={"guidance": 2.0, "bank_command": 3.0},
        design_state_names=("altitude", "bank"),
        design_control_names=("guidance", "bank_command"),
        authority_state_names=("altitude", "bank"),
        offset_free_outputs=("altitude", "bank"),
    )

    node = declaration.build_campaign().nodes[0]
    assert node.design_state_names == ("altitude", "bank")
    assert node.design_control_names == ("guidance", "bank_command")
    assert node.state_scales == (10.0, 5.0)
    assert node.control_scales == (2.0, 3.0)
    ####
