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
