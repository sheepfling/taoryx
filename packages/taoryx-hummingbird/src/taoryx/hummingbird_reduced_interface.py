"""Hummingbird-owned public controls and readbacks for the reduced runtime."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from taoryx.fidelity_contracts import FidelityTier
from taoryx.vehicle_interface import (
    AuthorityProfile,
    InterfaceAvailability,
    InterfaceChannel,
    InterfaceContractAugmentation,
    InterfaceValueType,
    ProvenanceKind,
)

_FAMILY_ID = "hummingbird"


def _action(
    identifier: str,
    unit: str,
    lower: float,
    upper: float,
    description: str,
    native: str,
    availability: InterfaceAvailability,
    *,
    frame: str | None = None,
    binding: Mapping[str, object] | None = None,
    claim_boundary: str | None = None,
) -> InterfaceChannel:
    """Build one source-backed reduced-response action declaration."""

    return InterfaceChannel(
        identifier,
        "action",
        "scalar",
        unit,
        description,
        frame=frame,
        lower=lower,
        upper=upper,
        availability=availability,
        provenance="source_backed",
        sampling="held_action",
        binding={"native_action": native, **(dict(binding) if binding is not None else {})},
        claim_boundary=(
            claim_boundary
            if claim_boundary is not None
            else "This action binds to the named source/runtime coordinate. The selected lower fidelity does not turn that coordinate into physical actuator evidence."
        ),
    )
    ####


def _status(
    identifier: str,
    unit: str | None,
    description: str,
    native: str,
    availability: InterfaceAvailability,
    *,
    value_type: InterfaceValueType = "scalar",
    provenance: ProvenanceKind = "source_backed",
) -> InterfaceChannel:
    """Build one committed pseudo-6DOF truth readback declaration."""

    return InterfaceChannel(
        identifier,
        "status",
        value_type,
        unit,
        description,
        availability=availability,
        provenance=provenance,
        sampling="truth_boundary",
        binding={"episode_value": native},
        claim_boundary="Published only at committed truth boundaries; it is not a policy observation unless an observation profile includes it.",
    )
    ####


def _diagnostic(
    identifier: str,
    native: str,
    availability: InterfaceAvailability,
) -> InterfaceChannel:
    """Build one reduced-runtime diagnostic retained with the truth readback."""

    return InterfaceChannel(
        identifier,
        "diagnostic",
        "enum" if identifier == "control.realization" else "boolean",
        None,
        "Raw claim-boundary diagnostic retained with the canonical status view.",
        availability=availability,
        provenance="derived",
        sampling="truth_boundary",
        binding={"episode_value": native},
        claim_boundary="Diagnostic provenance; not an independent physical-control claim.",
    )
    ####


@dataclass(frozen=True, slots=True)
class HummingbirdReducedInterfaceExtension:
    """Advertise pseudo-6DOF controls without making a rotor-allocation claim."""

    id: str = "taoryx.hummingbird.reduced-interface.v1"
    family_id: str = _FAMILY_ID

    def augment(
        self,
        fidelity: FidelityTier,
        *,
        episode_runnable: bool,
        batch_runnable: bool,
    ) -> InterfaceContractAugmentation | None:
        """Return Hummingbird's reduced controls, truth, resources, and diagnostics."""

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
        attitude_channels: tuple[InterfaceChannel, ...] = (
            _action(
                "attitude.roll.command",
                "rad",
                -1.5707963267948966,
                1.5707963267948966,
                "Held roll-angle reference accepted by the named aggregate-thrust response law.",
                "roll_rad",
                action_availability,
                binding={"feedback_channel_id": "attitude.roll"},
            ),
            _action(
                "attitude.pitch.command",
                "rad",
                -1.5707963267948966,
                1.5707963267948966,
                "Held pitch-angle reference accepted by the named aggregate-thrust response law.",
                "pitch_rad",
                action_availability,
                binding={"feedback_channel_id": "attitude.pitch"},
            ),
            _action(
                "attitude.yaw.command",
                "rad",
                -3.141592653589793,
                3.141592653589793,
                "Held yaw-angle reference in the local north-east-up navigation frame.",
                "yaw_rad",
                action_availability,
                binding={"feedback_channel_id": "attitude.yaw"},
            ),
            _action(
                "propulsion.command.fraction",
                "dimensionless",
                0.0,
                1.0,
                "Requested aggregate thrust fraction before the declared battery-availability limit.",
                "thrust_ratio",
                action_availability,
                binding={"feedback_channel_id": "propulsion.output.thrust.fraction"},
                claim_boundary="Aggregate thrust only; this is not an individual rotor-speed, motor-current, or physical allocation command.",
            ),
            InterfaceChannel(
                "propulsion.enable",
                "action",
                "boolean",
                None,
                "Aggregate motor-enable state for the response-law plant.",
                availability=action_availability,
                provenance="engineering_surrogate",
                sampling="held_action",
                binding={"native_action": "motors_enabled", "feedback_channel_id": "propulsion.enabled"},
                claim_boundary="This is aggregate enable state only; it does not establish individual motor allocation.",
            ),
        )
        velocity_channels: tuple[InterfaceChannel, ...] = (
            _action(
                "velocity.north.command",
                "m/s",
                -5.0,
                5.0,
                "Held north-velocity target lowered by the quadcopter translational response adapter.",
                "velocity-north-mps",
                action_availability,
                frame="NED",
                binding={"feedback_channel_id": "velocity.north"},
                claim_boundary="Reduced translational response only; this is not a force, moment, or rotor command.",
            ),
            _action(
                "velocity.east.command",
                "m/s",
                -5.0,
                5.0,
                "Held east-velocity target lowered by the quadcopter translational response adapter.",
                "velocity-east-mps",
                action_availability,
                frame="NED",
                binding={"feedback_channel_id": "velocity.east"},
                claim_boundary="Reduced translational response only; this is not a force, moment, or rotor command.",
            ),
            _action(
                "velocity.vertical.command",
                "m/s",
                -3.0,
                3.0,
                "Held vertical-velocity target; positive values command climb in the local navigation frame.",
                "velocity-vertical-mps",
                action_availability,
                binding={"feedback_channel_id": "velocity.vertical"},
                claim_boundary="Reduced translational response only; this is not a collective, force, or individual rotor command.",
            ),
            attitude_channels[2],
            attitude_channels[4],
        )
        waypoint_channels: tuple[InterfaceChannel, ...] = (
            _action(
                "navigation.waypoint.north.command",
                "m",
                -1_000_000.0,
                1_000_000.0,
                "Live absolute waypoint north coordinate in the composition local-navigation frame.",
                "waypoint-north-m",
                action_availability,
                frame="NED",
                binding={"feedback_channel_id": "position.north"},
                claim_boundary="The held target is lowered by the quadcopter waypoint adapter; it is not a prequalified route or physical-control claim.",
            ),
            _action(
                "navigation.waypoint.east.command",
                "m",
                -1_000_000.0,
                1_000_000.0,
                "Live absolute waypoint east coordinate in the composition local-navigation frame.",
                "waypoint-east-m",
                action_availability,
                frame="NED",
                binding={"feedback_channel_id": "position.east"},
                claim_boundary="The held target is lowered by the quadcopter waypoint adapter; it is not a prequalified route or physical-control claim.",
            ),
            _action(
                "navigation.waypoint.altitude.command",
                "m",
                0.0,
                10_000.0,
                "Live absolute altitude target above the composition navigation datum.",
                "waypoint-altitude-m",
                action_availability,
                binding={"feedback_channel_id": "position.altitude"},
                claim_boundary="The held target is lowered by the quadcopter waypoint adapter; it does not assert terrain clearance or route feasibility.",
            ),
            _action(
                "navigation.waypoint.capture_radius.command",
                "m",
                0.05,
                1_000.0,
                "Three-dimensional radius used to report live-waypoint capture.",
                "waypoint-capture-radius-m",
                action_availability,
                claim_boundary="This is an adapter capture tolerance, not a navigation-accuracy or mission-success guarantee.",
            ),
            _action(
                "navigation.waypoint.speed.command",
                "m/s",
                0.0,
                5.0,
                "Maximum horizontal speed used while tracking the held live waypoint.",
                "waypoint-speed-mps",
                action_availability,
                binding={"feedback_channel_id": "velocity.horizontal.speed"},
                claim_boundary="This bounds the reduced waypoint response and does not establish an aerodynamic or rotor envelope.",
            ),
            _action(
                "navigation.waypoint.vertical_speed.command",
                "m/s",
                0.0,
                3.0,
                "Maximum climb or descent speed used while tracking the held live waypoint.",
                "waypoint-vertical-speed-mps",
                action_availability,
                binding={"feedback_channel_id": "velocity.vertical"},
                claim_boundary="This bounds the reduced waypoint response and does not establish climb performance or rotor margin.",
            ),
            attitude_channels[2],
            attitude_channels[4],
        )
        return InterfaceContractAugmentation(
            action_channels=(*attitude_channels, *velocity_channels[:3], *waypoint_channels[:6]),
            authority_profiles=(
                AuthorityProfile(
                    "body_motion_response",
                    "body_motion",
                    action_availability,
                    tuple(item.id for item in attitude_channels),
                    "Bounded roll, pitch, yaw, and aggregate-thrust response command.",
                    "Pseudo-6DOF body-motion response only; no individual rotor, motor, or moment-balance claim.",
                    switching_policy="explicit_bumpless",
                    lowering_chain=("held_attitude_and_aggregate_thrust", "hummingbird_pseudo_6dof_response_law"),
                    scheme_id="body_motion.attitude",
                ),
                AuthorityProfile(
                    "velocity_yaw_command",
                    "kinematic",
                    action_availability,
                    tuple(item.id for item in velocity_channels),
                    "North, east, and positive-up velocity targets with yaw and aggregate propulsion enable.",
                    "The adapter closes a reduced velocity loop into aggregate attitude and thrust; it does not expose force, moment, or rotor authority.",
                    switching_policy="explicit_bumpless",
                    lowering_chain=("held_local_velocity_and_yaw", "bounded_translational_response", "aggregate_thrust_vector_response"),
                    scheme_id="kinematic.velocity",
                ),
                AuthorityProfile(
                    "live_waypoint_guidance",
                    "mission",
                    action_availability,
                    tuple(item.id for item in waypoint_channels),
                    "Live local waypoint, speed limits, yaw target, and aggregate propulsion enable.",
                    "The adapter provides boundary-updatable waypoint guidance over the pseudo plant; it does not qualify route feasibility, sensing, or physical allocation.",
                    switching_policy="explicit_bumpless",
                    lowering_chain=(
                        "held_live_waypoint",
                        "position_to_velocity_guidance",
                        "bounded_translational_response",
                        "aggregate_thrust_vector_response",
                    ),
                    scheme_id="mission.waypoint",
                ),
            ),
            status_channels=(
                _status("position.north", "m", "Committed NED north position.", "position_ned_m[0]", status_availability),
                _status("position.east", "m", "Committed NED east position.", "position_ned_m[1]", status_availability),
                _status("position.altitude", "m", "Committed altitude above the NED origin.", "position_ned_m[2] (sign-inverted)", status_availability),
                _status("velocity.north", "m/s", "Committed NED north velocity.", "velocity_ned_m_s[0]", status_availability),
                _status("velocity.east", "m/s", "Committed NED east velocity.", "velocity_ned_m_s[1]", status_availability),
                _status("velocity.down", "m/s", "Committed NED down velocity.", "velocity_ned_m_s[2]", status_availability),
                _status("velocity.vertical", "m/s", "Committed local-navigation vertical velocity; positive values are climb.", "velocity_ned_m_s[2] (sign-inverted)", status_availability),
                _status("velocity.horizontal.speed", "m/s", "Committed horizontal speed in the local navigation plane.", "velocity_horizontal_speed_m_s", status_availability, provenance="derived"),
                _status("attitude.roll", "rad", "Committed roll component of the declared pseudo-6DOF response.", "attitude_rad[0]", status_availability),
                _status("attitude.pitch", "rad", "Committed pitch component of the declared pseudo-6DOF response.", "attitude_rad[1]", status_availability),
                _status("attitude.yaw", "rad", "Committed yaw component of the declared pseudo-6DOF response.", "attitude_rad[2]", status_availability),
                _status("attitude.euler", "rad", "Declared pseudo-6DOF Euler attitude response.", "attitude_rad", status_availability, value_type="vector3"),
                _status("body_rate", "rad/s", "Declared pseudo-6DOF body-rate response.", "body_rate_rad_s", status_availability, value_type="vector3"),
                _status("contact.state", None, "Declared ground-contact state.", "contact", status_availability, value_type="boolean"),
                _status("propulsion.enabled", None, "Committed aggregate motor-enable state.", "motors_enabled", status_availability, value_type="boolean", provenance="derived"),
                _status("guidance.waypoint.range", "m", "Current three-dimensional range to the held live waypoint, or zero when waypoint authority is inactive.", "guidance_waypoint_range_m", status_availability, provenance="derived"),
                _status("guidance.waypoint.captured", None, "Whether the held live waypoint is inside its declared capture radius.", "guidance_waypoint_captured", status_availability, value_type="boolean", provenance="derived"),
                _status("guidance.waypoint.status", None, "Live-waypoint adapter state: inactive, tracking, or captured.", "guidance_waypoint_status", status_availability, value_type="enum", provenance="derived"),
                _status("propulsion.output.thrust.aggregate", "N", "Achieved aggregate thrust in the pseudo response law.", "aggregate_thrust_n", status_availability),
                _status("propulsion.output.thrust.fraction", "dimensionless", "Achieved aggregate thrust divided by the declared maximum, including battery and enable limits.", "aggregate_thrust_fraction", status_availability, provenance="derived"),
            ),
            resource_channels=(
                InterfaceChannel(
                    "resources.mass.total",
                    "resource",
                    "scalar",
                    "kg",
                    "Modeled total mass retained by the aggregate-thrust pseudo-6DOF plant.",
                    availability=status_availability,
                    provenance="engineering_surrogate",
                    sampling="truth_boundary",
                    binding={"episode_value": "mass_kg"},
                    claim_boundary=(
                        "This is the configured aggregate model mass. It does not establish a payload distribution, "
                        "inertia update, fuel mass flow, or a complete mass-property ledger."
                    ),
                ),
                InterfaceChannel(
                    "resources.battery.fraction_remaining",
                    "resource",
                    "scalar",
                    "dimensionless",
                    "Declared bounded engineering battery reserve.",
                    lower=0.0,
                    upper=1.0,
                    availability=status_availability,
                    provenance="engineering_surrogate",
                    sampling="truth_boundary",
                    binding={"episode_value": "battery_fraction"},
                    claim_boundary="This is the pseudo-plant reserve model; it is not a cell-voltage or motor-current claim.",
                ),
            ),
            diagnostic_channels=(
                _diagnostic("control.realization", "control_realization", status_availability),
                _diagnostic("control.physical_motor_allocation", "physical_motor_allocation", status_availability),
            ),
        )
        ####

    ####


def hummingbird_reduced_interface_extension() -> HummingbirdReducedInterfaceExtension:
    """Construct the package-owned pseudo-6DOF interface augmentation on demand."""

    return HummingbirdReducedInterfaceExtension()
    ####


__all__ = ["HummingbirdReducedInterfaceExtension", "hummingbird_reduced_interface_extension"]
