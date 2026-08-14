"""Hummingbird-owned direct-wrench comparator declaration.

The source-hover comparator is an intentionally narrow plug-in endpoint. Its
static contract is available without loading RotorPy-derived tables; the
source-plant configuration is constructed only after the Hummingbird screen is
selected for preflight or execution.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from taoryx.local_direct_wrench_screen_registry import LocalDirectWrenchScreenDefinition

if TYPE_CHECKING:
    from taoryx.local_direct_wrench import LocalDirectWrenchScreenConfig


def _config() -> LocalDirectWrenchScreenConfig:
    """Load the Hummingbird source-hover comparator only when selected."""

    from .source_table_multirotor import build_hummingbird_local_direct_wrench_screen_config

    return build_hummingbird_local_direct_wrench_screen_config()
    ####


def hummingbird_local_direct_wrench_screen_definition() -> LocalDirectWrenchScreenDefinition:
    """Return the one Hummingbird-owned local direct-wrench screen."""

    identifier = "hummingbird-source-hover-local-direct-wrench-v1"
    force_limit_n = 0.25
    force_z_limit_n = 0.50
    moment_limit_nm = 0.01
    controls = (
        ("force_x_n", "N", force_limit_n),
        ("force_y_n", "N", force_limit_n),
        ("force_z_n", "N", force_z_limit_n),
        ("moment_x_nm", "N m", moment_limit_nm),
        ("moment_y_nm", "N m", moment_limit_nm),
        ("moment_z_nm", "N m", moment_limit_nm),
    )
    return LocalDirectWrenchScreenDefinition(
        id=identifier,
        family_id="hummingbird",
        mission_id="hummingbird_local_direct_wrench_screen_v1",
        initialization_id="source_individual_rotor_hover_local_point",
        segment_id="local_wrench_recovery_screen",
        capability_adapter_id="taoryx.hummingbird.local_direct_wrench_screen.capability.v1",
        config_factory=_config,
        advertisement={
            "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
            "id": identifier,
            "mission_template_id": "hummingbird_local_direct_wrench_screen_v1",
            "fidelity": "rigid_body_6dof_direct_wrench",
            "operations": ["batch"],
            "control_realization": "direct_wrench",
            "physical_effector_allocation": False,
            "batch_action_trace": "emits_committed_interval_trace",
            "controller": {
                "method": "lqr",
                "campaign_id": None,
                "integral_output_names": [],
                "fixed_cadence_s": 0.002,
                "screen_duration_s": 2.0,
            },
            "direct_wrench_controls": [{"id": name, "unit": unit, "lower": -limit, "upper": limit} for name, unit, limit in controls],
            "claim_boundary": (
                "The wrench is an internally generated comparator input after source-hover rotor trim. "
                "It bypasses allocation and is not an externally owned rotor or flight-control command."
            ),
        },
        claim_boundary=(
            "This is a local Hummingbird source-hover direct-wrench comparator. It proves only the pinned "
            "RotorPy-derived local derivative, bounded injected wrench, and local recovery result. It does "
            "not prove rotor allocation, motor lag, translation, landing, battery behavior, wind rejection, "
            "gain scheduling, or family qualification."
        ),
    )
    ####


__all__ = ["hummingbird_local_direct_wrench_screen_definition"]
