"""F-16-owned additions to the generic reduced fixed-wing interface."""

from __future__ import annotations

from dataclasses import dataclass

from taoryx.fidelity_contracts import FidelityTier
from taoryx.vehicle_interface import (
    AuthorityProfile,
    InterfaceAvailability,
    InterfaceChannel,
    InterfaceContractAugmentation,
)

_FAMILY_ID = "f16_s119"


@dataclass(frozen=True, slots=True)
class F16ReducedInterfaceExtension:
    """Advertise F-16 pseudo-6DOF rate control without a physical FCS claim."""

    id: str = "taoryx.f16.reduced-interface.v1"
    family_id: str = _FAMILY_ID

    def augment(
        self,
        fidelity: FidelityTier,
        *,
        episode_runnable: bool,
        batch_runnable: bool,
    ) -> InterfaceContractAugmentation | None:
        """Add rate-reference controls and exact axis readbacks for pseudo-6DOF."""

        if fidelity != "pseudo_6dof":
            return None
        action_availability: InterfaceAvailability = "available" if episode_runnable else "unavailable_at_runtime"
        status_availability: InterfaceAvailability = (
            "available"
            if episode_runnable
            else "available_in_batch"
            if batch_runnable
            else "unavailable_at_runtime"
        )
        controls = tuple(
            InterfaceChannel(
                identifier,
                "action",
                "scalar",
                "rad/s",
                description,
                frame="body",
                lower=lower,
                upper=upper,
                availability=action_availability,
                provenance="engineering_surrogate",
                sampling="held_action",
                binding={
                    "native_action": native_action,
                    "control_role": "body_rate_command",
                    "action_adapter": "f16_pseudo_6dof_body_rate_v1",
                    "state_authority": "pseudo_6dof_rate_reference_adapter",
                    "frame": "body",
                    "feedback_channel_id": feedback_channel_id,
                },
                claim_boundary=(
                    "The pseudo-6DOF adapter converts this held rate reference into bounded attitude/kinematic "
                    "references. It does not expose source FCS, moment, actuator, or surface authority."
                ),
            )
            for identifier, lower, upper, native_action, description, feedback_channel_id in (
                (
                    "body_rate.roll.command",
                    -0.5,
                    0.5,
                    "body-roll-rate-command-rad-s",
                    "Body-frame roll-rate reference p.",
                    "body_rate.roll",
                ),
                (
                    "body_rate.pitch.command",
                    -0.35,
                    0.35,
                    "body-pitch-rate-command-rad-s",
                    "Body-frame pitch-rate reference q.",
                    "body_rate.pitch",
                ),
                (
                    "body_rate.yaw.command",
                    -0.35,
                    0.35,
                    "body-yaw-rate-command-rad-s",
                    "Body-frame yaw-rate reference r.",
                    "body_rate.yaw",
                ),
            )
        )
        readbacks = tuple(
            InterfaceChannel(
                identifier,
                "status",
                "scalar",
                "rad/s",
                description,
                frame="body",
                availability=status_availability,
                provenance="engineering_surrogate",
                sampling="truth_boundary",
                binding={
                    "episode_value": identifier,
                    "batch_telemetry": batch_telemetry,
                    "frame": "body",
                },
                claim_boundary=(
                    "This is a committed pseudo-6DOF engineering-surrogate rate response for streaming control "
                    "readback, not F-16 flight-control-system or effector-allocation evidence."
                ),
            )
            for identifier, batch_telemetry, description in (
                (
                    "body_rate.roll",
                    "body_rate_rad_s[0]",
                    "Committed pseudo-6DOF body roll-rate response p for rate-command readback.",
                ),
                (
                    "body_rate.pitch",
                    "body_rate_rad_s[1]",
                    "Committed pseudo-6DOF body pitch-rate response q for rate-command readback.",
                ),
                (
                    "body_rate.yaw",
                    "body_rate_rad_s[2]",
                    "Committed pseudo-6DOF body yaw-rate response r for rate-command readback.",
                ),
            )
        )
        return InterfaceContractAugmentation(
            action_channels=controls,
            authority_profiles=(
                AuthorityProfile(
                    "body_rate_command",
                    "body_motion",
                    action_availability,
                    ("propulsion.command.fraction", *(item.id for item in controls)),
                    "Body-frame roll-, pitch-, and yaw-rate references plus normalized propulsion for the F-16 pseudo-6DOF response.",
                    "This is a bounded engineering-surrogate rate-reference adapter. Roll/yaw response remains coupled by "
                    "the named pseudo-6DOF law; it is not a source flight-control computer, moment balance, or surface authority.",
                    switching_policy="explicit_bumpless",
                    lowering_chain=(
                        "external_body_rate_reference",
                        "pseudo_6dof_rate_reference_adapter",
                        "reduced_fixed_wing_response_law",
                    ),
                    scheme_id="body_motion.body_rate",
                ),
            ),
            status_channels=readbacks,
        )
        ####

    ####


def f16_reduced_interface_extension() -> F16ReducedInterfaceExtension:
    """Construct the package-owned reduced interface augmentation on demand."""

    return F16ReducedInterfaceExtension()
    ####


__all__ = ["F16ReducedInterfaceExtension", "f16_reduced_interface_extension"]
