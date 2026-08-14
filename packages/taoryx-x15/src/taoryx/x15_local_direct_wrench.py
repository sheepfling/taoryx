"""X-15-owned direct-wrench controller-screen declarations.

The declarations are intentionally static and lazy: discovery exposes the
exact controls, limits, cadence, and claim boundary without parsing source
tables or constructing the X-15 plant.  The configuration factories import
the source-backed adapter only after a selected endpoint is preflighted or
executed.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from taoryx.local_direct_wrench_screen_registry import LocalDirectWrenchScreenDefinition

if TYPE_CHECKING:
    from taoryx.local_direct_wrench import LocalDirectWrenchScreenConfig


def _build_lqr_config() -> LocalDirectWrenchScreenConfig:
    """Build the exact source-release LQR screen only when selected."""

    from .x15_adapter import build_x15_local_direct_wrench_screen_config

    return build_x15_local_direct_wrench_screen_config()
    ####


def _build_lqi_config() -> LocalDirectWrenchScreenConfig:
    """Build the exact source-release LQI screen only when selected."""

    from .x15_adapter import build_x15_local_direct_wrench_lqi_screen_config

    return build_x15_local_direct_wrench_lqi_screen_config()
    ####


def x15_local_direct_wrench_screen_definitions() -> tuple[LocalDirectWrenchScreenDefinition, ...]:
    """Return the two declared X-15 local direct-wrench endpoints."""

    return (
        LocalDirectWrenchScreenDefinition(
            id="x15-source-release-glide-local-direct-wrench-v1",
            family_id="x15",
            mission_id="x15_local_direct_wrench_screen_v1",
            initialization_id="source_release_glide_local_point",
            segment_id="local_wrench_recovery_screen",
            capability_adapter_id="taoryx.x15_local_direct_wrench_screen.capability.v1",
            config_factory=_build_lqr_config,
            advertisement=_advertisement(
                identifier="x15-source-release-glide-local-direct-wrench-v1",
                mission_id="x15_local_direct_wrench_screen_v1",
                method="lqr",
                operations=("batch", "step"),
                fixed_cadence_s=0.002,
                screen_duration_s=2.0,
                claim_boundary=(
                    "The batch screen internally generates bounded injected wrench. The paired step endpoint accepts "
                    "the same direct-wrench coordinates, but neither is a physical X-15 effector allocator."
                ),
            ),
            claim_boundary=(
                "This is a local X-15 source-load direct-wrench LQR recovery screen. It proves only the declared "
                "source-local derivative, explicit bounded wrench projection, and local error-recovery result. It "
                "does not prove X-15 flight trim, release, propulsion scheduling, navigation, terminal handoff, "
                "physical stabilator/rudder/RCS allocation, or vehicle-family qualification."
            ),
            paired_lqi_mission_id="x15_local_direct_wrench_lqi_screen_v1",
            paired_lqi_capability_adapter_id="taoryx.x15_local_direct_wrench_lqi_screen.capability.v1",
            paired_lqi_campaign_id="x15-source-release-direct-wrench-lqi-v1",
        ),
        LocalDirectWrenchScreenDefinition(
            id="x15-source-release-glide-local-direct-wrench-lqi-v1",
            family_id="x15",
            mission_id="x15_local_direct_wrench_lqi_screen_v1",
            initialization_id="source_release_glide_local_point",
            segment_id="local_wrench_lqi_recovery_screen",
            capability_adapter_id="taoryx.x15_local_direct_wrench_lqi_screen.capability.v1",
            config_factory=_build_lqi_config,
            advertisement=_advertisement(
                identifier="x15-source-release-glide-local-direct-wrench-lqi-v1",
                mission_id="x15_local_direct_wrench_lqi_screen_v1",
                method="lqi",
                operations=("batch",),
                fixed_cadence_s=0.01,
                screen_duration_s=3.5,
                campaign_id="x15-source-release-direct-wrench-lqi-v1",
                integral_output_names=("u_m_s",),
                claim_boundary=(
                    "The LQI screen internally generates bounded injected wrench and emits its committed batch trace. "
                    "It has no caller-driven step endpoint and does not establish physical X-15 effector allocation."
                ),
            ),
            claim_boundary=(
                "This is a local X-15 source-load direct-wrench body-speed LQI recovery screen. It proves only the "
                "declared source-local derivative, bounded wrench projection, and one integrated body-speed error "
                "recovery. It does not prove X-15 flight trim, release, propulsion scheduling, navigation, terminal "
                "handoff, physical stabilator/rudder/RCS allocation, persistent-disturbance rejection, or "
                "vehicle-family qualification."
            ),
        ),
    )
    ####


def _advertisement(
    *,
    identifier: str,
    mission_id: str,
    method: str,
    operations: tuple[str, ...],
    fixed_cadence_s: float,
    screen_duration_s: float,
    claim_boundary: str,
    campaign_id: str | None = None,
    integral_output_names: tuple[str, ...] = (),
) -> dict[str, object]:
    """Return the portable six-axis direct-wrench contract for one endpoint."""

    force_limit_n = 4.0e5
    moment_limit_nm = 1.0e6
    controls = (
        ("force_x_n", "N", force_limit_n),
        ("force_y_n", "N", force_limit_n),
        ("force_z_n", "N", force_limit_n),
        ("moment_x_nm", "N m", moment_limit_nm),
        ("moment_y_nm", "N m", moment_limit_nm),
        ("moment_z_nm", "N m", moment_limit_nm),
    )
    return {
        "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
        "id": identifier,
        "mission_template_id": mission_id,
        "fidelity": "rigid_body_6dof_direct_wrench",
        "operations": list(operations),
        "control_realization": "direct_wrench",
        "physical_effector_allocation": False,
        "batch_action_trace": "emits_committed_interval_trace",
        "controller": {
            "method": method,
            "campaign_id": campaign_id,
            "integral_output_names": list(integral_output_names),
            "fixed_cadence_s": fixed_cadence_s,
            "screen_duration_s": screen_duration_s,
        },
        "direct_wrench_controls": [{"id": name, "unit": unit, "lower": -limit, "upper": limit} for name, unit, limit in controls],
        "claim_boundary": claim_boundary,
    }
    ####


__all__ = ["x15_local_direct_wrench_screen_definitions"]
