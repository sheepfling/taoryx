"""Declared source-local direct-wrench screen definitions.

These screens are deliberately narrow bridge evidence.  A definition binds a
single family-owned source operating point to the common local LQR evaluator;
it does not turn that point into a route mission, trim result, or physical
effector-allocation claim.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from copy import deepcopy
from dataclasses import dataclass

from .local_direct_wrench import LocalDirectWrenchScreenConfig
from .vehicle_composition import CompiledVehicleComposition


@dataclass(frozen=True, slots=True)
class LocalDirectWrenchScreenDefinition:
    """One exact family-owned source-local screen declaration."""

    family_id: str
    mission_id: str
    initialization_id: str
    segment_id: str
    capability_adapter_id: str
    config_factory: Callable[[], LocalDirectWrenchScreenConfig]
    advertisement: Mapping[str, object]
    claim_boundary: str
    paired_lqi_mission_id: str | None = None
    paired_lqi_capability_adapter_id: str | None = None
    paired_lqi_campaign_id: str | None = None

    def __post_init__(self) -> None:
        """Reject incomplete static metadata without constructing a controller."""

        required = {
            "schema",
            "id",
            "fidelity",
            "operations",
            "control_realization",
            "controller",
            "direct_wrench_controls",
            "claim_boundary",
        }
        missing = sorted(required - set(self.advertisement))
        if missing:
            raise ValueError(f"local direct-wrench advertisement is missing: {', '.join(missing)}")
        if self.advertisement["fidelity"] != "rigid_body_6dof_direct_wrench":
            raise ValueError("local direct-wrench advertisements require direct-wrench fidelity")
        if self.advertisement["control_realization"] != "direct_wrench":
            raise ValueError("local direct-wrench advertisement must retain direct-wrench realization")
        operations = self.advertisement["operations"]
        if not isinstance(operations, list) or "batch" not in operations:
            raise ValueError("local direct-wrench advertisements must include batch execution")
        if not self.claim_boundary.strip():
            raise ValueError("local direct-wrench definitions require a claim boundary")
        ####
    ####

    def public_advertisement(self) -> dict[str, object]:
        """Return portable static screen metadata without tuning or executing."""

        return deepcopy(dict(self.advertisement))
        ####

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Return whether the composition exactly selects this local screen."""

        return (
            composition.family_id == self.family_id
            and composition.mission == self.mission_id
            and composition.initialization.id == self.initialization_id
            and composition.fidelity == "rigid_body_6dof_direct_wrench"
        )
        ####

    ####


def local_direct_wrench_screen_definitions() -> tuple[LocalDirectWrenchScreenDefinition, ...]:
    """Return installed source-local direct-wrench screens in stable order."""

    from .hl20_adapter import (
        build_hl20_local_direct_wrench_lqi_screen_config,
        build_hl20_local_direct_wrench_screen_config,
    )
    from .source_table_multirotor import build_hummingbird_local_direct_wrench_screen_config
    from .x15_adapter import (
        build_x15_local_direct_wrench_lqi_screen_config,
        build_x15_local_direct_wrench_screen_config,
    )

    return (
        LocalDirectWrenchScreenDefinition(
            family_id="hummingbird",
            mission_id="hummingbird_local_direct_wrench_screen_v1",
            initialization_id="source_individual_rotor_hover_local_point",
            segment_id="local_wrench_recovery_screen",
            capability_adapter_id="taoryx.hummingbird.local_direct_wrench_screen.capability.v1",
            config_factory=build_hummingbird_local_direct_wrench_screen_config,
            advertisement=_direct_wrench_advertisement(
                id="hummingbird-source-hover-local-direct-wrench-v1",
                method="lqr",
                operations=("batch",),
                force_limit_n=0.25,
                force_z_limit_n=0.50,
                moment_limit_nm=0.01,
                fixed_cadence_s=0.002,
                screen_duration_s=2.0,
                claim_boundary=(
                    "The wrench is an internally generated comparator input after source-hover rotor trim. "
                    "It bypasses allocation and is not an externally owned rotor or flight-control command."
                ),
            ),
            claim_boundary=(
                "This is a local Hummingbird source-hover direct-wrench comparator. It proves only the pinned "
                "RotorPy-derived local derivative, bounded injected wrench, and local recovery result. It does "
                "not prove rotor allocation, motor lag, translation, landing, battery behavior, wind rejection, "
                "gain scheduling, or family qualification."
            ),
        ),
        LocalDirectWrenchScreenDefinition(
            family_id="x15",
            mission_id="x15_local_direct_wrench_screen_v1",
            initialization_id="source_release_glide_local_point",
            segment_id="local_wrench_recovery_screen",
            capability_adapter_id="taoryx.x15_local_direct_wrench_screen.capability.v1",
            config_factory=build_x15_local_direct_wrench_screen_config,
            advertisement=_direct_wrench_advertisement(
                id="x15-source-release-glide-local-direct-wrench-v1",
                method="lqr",
                operations=("batch", "step"),
                force_limit_n=4.0e5,
                moment_limit_nm=1.0e6,
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
            family_id="x15",
            mission_id="x15_local_direct_wrench_lqi_screen_v1",
            initialization_id="source_release_glide_local_point",
            segment_id="local_wrench_lqi_recovery_screen",
            capability_adapter_id="taoryx.x15_local_direct_wrench_lqi_screen.capability.v1",
            config_factory=build_x15_local_direct_wrench_lqi_screen_config,
            advertisement=_direct_wrench_advertisement(
                id="x15-source-release-glide-local-direct-wrench-lqi-v1",
                method="lqi",
                operations=("batch",),
                force_limit_n=4.0e5,
                moment_limit_nm=1.0e6,
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
        LocalDirectWrenchScreenDefinition(
            family_id="hl20_mod_k",
            mission_id="hl20_local_direct_wrench_screen_v1",
            initialization_id="source_subsonic_local_point",
            segment_id="local_wrench_recovery_screen",
            capability_adapter_id="taoryx.hl20_local_direct_wrench_screen.capability.v1",
            config_factory=build_hl20_local_direct_wrench_screen_config,
            advertisement=_direct_wrench_advertisement(
                id="hl20-source-mach0p5-local-direct-wrench-v1",
                method="lqr",
                operations=("batch", "step"),
                force_limit_n=2.0e5,
                moment_limit_nm=1.0e6,
                fixed_cadence_s=0.002,
                screen_duration_s=4.0,
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
            family_id="hl20_mod_k",
            mission_id="hl20_local_direct_wrench_lqi_screen_v1",
            initialization_id="source_subsonic_local_point",
            segment_id="local_wrench_lqi_recovery_screen",
            capability_adapter_id="taoryx.hl20_local_direct_wrench_lqi_screen.capability.v1",
            config_factory=build_hl20_local_direct_wrench_lqi_screen_config,
            advertisement=_direct_wrench_advertisement(
                id="hl20-source-subsonic-local-direct-wrench-lqi-v1",
                method="lqi",
                operations=("batch",),
                force_limit_n=2.0e5,
                moment_limit_nm=1.0e6,
                fixed_cadence_s=0.01,
                screen_duration_s=4.0,
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


def _direct_wrench_advertisement(
    *,
    id: str,
    method: str,
    operations: tuple[str, ...],
    force_limit_n: float,
    moment_limit_nm: float,
    fixed_cadence_s: float,
    screen_duration_s: float,
    claim_boundary: str,
    force_z_limit_n: float | None = None,
    campaign_id: str | None = None,
    integral_output_names: tuple[str, ...] = (),
) -> dict[str, object]:
    """Build one static direct-wrench screen advertisement from declared limits."""

    force_z_limit = force_limit_n if force_z_limit_n is None else force_z_limit_n
    controls = (
        ("force_x_n", "N", force_limit_n),
        ("force_y_n", "N", force_limit_n),
        ("force_z_n", "N", force_z_limit),
        ("moment_x_nm", "N m", moment_limit_nm),
        ("moment_y_nm", "N m", moment_limit_nm),
        ("moment_z_nm", "N m", moment_limit_nm),
    )
    return {
        "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
        "id": id,
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
        "direct_wrench_controls": [
            {"id": name, "unit": unit, "lower": -limit, "upper": limit}
            for name, unit, limit in controls
        ],
        "claim_boundary": claim_boundary,
    }
    ####


def resolve_local_direct_wrench_screen_definition(
    composition: CompiledVehicleComposition,
) -> LocalDirectWrenchScreenDefinition | None:
    """Resolve one exact screen without a nearest-family fallback."""

    return next((definition for definition in local_direct_wrench_screen_definitions() if definition.supports(composition)), None)
    ####


def resolve_local_direct_wrench_screen_advertisement(
    *,
    family_id: str | None,
    mission_id: str | None,
    fidelity: str,
) -> dict[str, object] | None:
    """Return selected static metadata without compiling a screen or tuning."""

    if family_id is None or mission_id is None or fidelity != "rigid_body_6dof_direct_wrench":
        return None
    definition = next(
        (
            item
            for item in local_direct_wrench_screen_definitions()
            if item.family_id == family_id and item.mission_id == mission_id
        ),
        None,
    )
    return definition.public_advertisement() if definition is not None else None
    ####


__all__ = [
    "LocalDirectWrenchScreenDefinition",
    "local_direct_wrench_screen_definitions",
    "resolve_local_direct_wrench_screen_advertisement",
    "resolve_local_direct_wrench_screen_definition",
]
