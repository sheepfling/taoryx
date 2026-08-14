"""HL-20-owned local direct-wrench controller-screen declarations."""

from __future__ import annotations

from typing import TYPE_CHECKING

from taoryx.local_direct_wrench_screen_registry import LocalDirectWrenchScreenDefinition

if TYPE_CHECKING:
    from taoryx.local_direct_wrench import LocalDirectWrenchScreenConfig


def _build_lqr_config() -> LocalDirectWrenchScreenConfig:
    """Create the selected HL-20 source-local LQR screen only at use time."""

    from .hl20_adapter import build_hl20_local_direct_wrench_screen_config

    return build_hl20_local_direct_wrench_screen_config()
    ####


def _build_lqi_config() -> LocalDirectWrenchScreenConfig:
    """Create the selected HL-20 source-local LQI screen only at use time."""

    from .hl20_adapter import build_hl20_local_direct_wrench_lqi_screen_config

    return build_hl20_local_direct_wrench_lqi_screen_config()
    ####


def _advertisement(
    *,
    identifier: str,
    method: str,
    operations: tuple[str, ...],
    cadence_s: float,
    duration_s: float,
    claim_boundary: str,
    campaign_id: str | None = None,
    integral_output_names: tuple[str, ...] = (),
) -> dict[str, object]:
    """Return a static six-axis local screen contract without loading HL-20."""

    controls = (
        ("force_x_n", "N", 2.0e5),
        ("force_y_n", "N", 2.0e5),
        ("force_z_n", "N", 2.0e5),
        ("moment_x_nm", "N m", 1.0e6),
        ("moment_y_nm", "N m", 1.0e6),
        ("moment_z_nm", "N m", 1.0e6),
    )
    return {
        "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
        "id": identifier,
        "fidelity": "rigid_body_6dof_direct_wrench",
        "operations": list(operations),
        "control_realization": "direct_wrench",
        "physical_effector_allocation": False,
        "batch_action_trace": "emits_committed_interval_trace",
        "controller": {
            "method": method,
            "campaign_id": campaign_id,
            "integral_output_names": list(integral_output_names),
            "fixed_cadence_s": cadence_s,
            "screen_duration_s": duration_s,
        },
        "direct_wrench_controls": [
            {"id": name, "unit": unit, "lower": -limit, "upper": limit}
            for name, unit, limit in controls
        ],
        "claim_boundary": claim_boundary,
    }
    ####


def hl20_local_direct_wrench_screen_definitions() -> tuple[LocalDirectWrenchScreenDefinition, ...]:
    """Return the two bounded local direct-wrench endpoints owned by HL-20."""

    return (
        LocalDirectWrenchScreenDefinition(
            id="hl20-source-mach0p5-local-direct-wrench-v1",
            family_id="hl20_mod_k",
            mission_id="hl20_local_direct_wrench_screen_v1",
            initialization_id="source_subsonic_local_point",
            segment_id="local_wrench_recovery_screen",
            capability_adapter_id="taoryx.hl20_local_direct_wrench_screen.capability.v1",
            config_factory=_build_lqr_config,
            advertisement=_advertisement(
                identifier="hl20-source-mach0p5-local-direct-wrench-v1",
                method="lqr",
                operations=("batch", "step"),
                cadence_s=0.002,
                duration_s=4.0,
                claim_boundary=(
                    "The batch screen internally generates bounded injected wrench. The paired step endpoint accepts "
                    "the same direct-wrench coordinates, but neither is a physical HL-20 surface allocator."
                ),
            ),
            claim_boundary=(
                "This is a local HL-20 source-load direct-wrench LQR recovery screen at one pinned subsonic "
                "source condition. It proves only the declared source-local derivative, explicit bounded wrench "
                "projection, and local error-recovery result. It does not prove flight trim, release, glide guidance, "
                "high-energy performance, physical control-surface allocation, or lifting-body-family qualification."
            ),
            paired_lqi_mission_id="hl20_local_direct_wrench_lqi_screen_v1",
            paired_lqi_capability_adapter_id="taoryx.hl20_local_direct_wrench_lqi_screen.capability.v1",
            paired_lqi_campaign_id="hl20-source-subsonic-direct-wrench-lqi-v1",
        ),
        LocalDirectWrenchScreenDefinition(
            id="hl20-source-subsonic-local-direct-wrench-lqi-v1",
            family_id="hl20_mod_k",
            mission_id="hl20_local_direct_wrench_lqi_screen_v1",
            initialization_id="source_subsonic_local_point",
            segment_id="local_wrench_lqi_recovery_screen",
            capability_adapter_id="taoryx.hl20_local_direct_wrench_lqi_screen.capability.v1",
            config_factory=_build_lqi_config,
            advertisement=_advertisement(
                identifier="hl20-source-subsonic-local-direct-wrench-lqi-v1",
                method="lqi",
                operations=("batch",),
                cadence_s=0.01,
                duration_s=4.0,
                campaign_id="hl20-source-subsonic-direct-wrench-lqi-v1",
                integral_output_names=("u_m_s",),
                claim_boundary=(
                    "The LQI screen internally generates bounded injected wrench and emits its committed batch trace. "
                    "It has no caller-driven step endpoint and does not establish physical HL-20 surface allocation."
                ),
            ),
            claim_boundary=(
                "This is a local HL-20 source-load direct-wrench body-speed LQI recovery screen at one pinned "
                "subsonic source condition. It proves only the declared source-local derivative, bounded wrench "
                "projection, and one integrated body-speed error recovery. It does not prove flight trim, release, "
                "glide guidance, high-energy performance, physical control-surface allocation, persistent-disturbance "
                "rejection, or lifting-body-family qualification."
            ),
        ),
    )
    ####


__all__ = ["hl20_local_direct_wrench_screen_definitions"]
