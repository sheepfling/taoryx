"""Passive-body-owned public truth contract for direct-release models."""

from __future__ import annotations

from dataclasses import dataclass

from taoryx.fidelity_contracts import FidelityTier
from taoryx.vehicle_interface import (
    InterfaceAvailability,
    InterfaceChannel,
    InterfaceContractAugmentation,
    InterfaceValueType,
    ProvenanceKind,
)

_FAMILY_ID = "tumbling_body"


def _status(
    identifier: str,
    unit: str | None,
    description: str,
    availability: InterfaceAvailability,
    *,
    value_type: InterfaceValueType = "scalar",
    binding: dict[str, object],
    provenance: ProvenanceKind = "engineering_surrogate",
) -> InterfaceChannel:
    """Build one committed passive-body truth readback declaration."""

    return InterfaceChannel(
        identifier,
        "status",
        value_type,
        unit,
        description,
        availability=availability,
        provenance=provenance,
        sampling="truth_boundary",
        binding=binding,
        claim_boundary="Published only at committed truth boundaries; it is not a policy observation unless an observation profile includes it.",
    )
    ####


@dataclass(frozen=True, slots=True)
class PassiveBodyInterfaceExtension:
    """Advertise passive direct-release readback without inventing control."""

    id: str = "taoryx.passive-bodies.interface.v1"
    family_id: str = _FAMILY_ID

    def augment(
        self,
        fidelity: FidelityTier,
        *,
        episode_runnable: bool,
        batch_runnable: bool,
    ) -> InterfaceContractAugmentation | None:
        """Return the exact batch-truth channels for passive direct release."""

        if fidelity not in {"point_mass_3dof", "pseudo_6dof"}:
            return None
        availability: InterfaceAvailability = (
            "available"
            if episode_runnable
            else "available_in_batch"
            if batch_runnable
            else "unavailable_at_runtime"
        )
        status: list[InterfaceChannel] = [
            _status(
                "position.local",
                "m",
                "Committed local-frame passive-body position.",
                availability,
                value_type="vector3",
                binding={"batch_telemetry": "position_m", "frame": "local_reduced"},
            ),
            _status(
                "position.altitude",
                "m",
                "Committed passive-body altitude above the impact plane.",
                availability,
                binding={"batch_telemetry": "position_m[2]", "frame": "local_reduced"},
            ),
            _status(
                "velocity.local",
                "m/s",
                "Committed local-frame passive-body velocity.",
                availability,
                value_type="vector3",
                binding={"batch_telemetry": "velocity_m_s", "frame": "local_reduced"},
            ),
            _status(
                "velocity.speed",
                "m/s",
                "Magnitude of the committed passive-body velocity.",
                availability,
                binding={"derived_from": "batch_telemetry.velocity_m_s", "transform": "norm"},
            ),
            _status(
                "aerodynamics.drag_force",
                "N",
                "Committed aerodynamic drag-force magnitude from the passive-body truth model.",
                availability,
                binding={"batch_telemetry": "drag_force_n", "frame": "local_reduced"},
            ),
            _status(
                "aerodynamics.projected_area",
                "m^2",
                "Committed projected area used by the selected passive-body aerodynamic representation.",
                availability,
                binding={"batch_telemetry": "projected_area_m2", "frame": "body"},
            ),
            _status(
                "angular_rate.norm",
                "rad/s",
                "Committed angular-rate magnitude; zero in the orientation-averaged 3DOF reduction.",
                availability,
                binding={"batch_telemetry": "angular_rate_norm_rad_s", "frame": "body"},
            ),
            _status(
                "phase.mode",
                None,
                "Passive direct-release phase, always ballistic after the declared release.",
                availability,
                value_type="enum",
                binding={"constant": "ballistic"},
                provenance="derived",
            ),
        ]
        if fidelity == "pseudo_6dof":
            status.extend(
                (
                    _status(
                        "attitude.quaternion",
                        "dimensionless",
                        "Native rigid-body attitude reused by the passive pseudo-6DOF profile.",
                        availability,
                        value_type="vector4",
                        binding={"batch_telemetry": "attitude_quaternion"},
                    ),
                    _status(
                        "body_rate",
                        "rad/s",
                        "Native rigid-body angular rate reused by the passive pseudo-6DOF profile.",
                        availability,
                        value_type="vector3",
                        binding={"batch_telemetry": "attitude_rate_rad_s"},
                    ),
                )
            )
        return InterfaceContractAugmentation(
            status_channels=tuple(status),
            resource_channels=(
                InterfaceChannel(
                    "resources.mass.total",
                    "resource",
                    "scalar",
                    "kg",
                    "Passive-body mass at the committed truth state.",
                    availability=availability,
                    provenance="engineering_surrogate",
                    sampling="truth_boundary",
                    binding={"batch_telemetry": "mass_kg"},
                    claim_boundary="The witness has no propulsion resource; this is fixed detached-body mass, not fuel or propellant.",
                ),
            ),
            diagnostic_channels=(
                InterfaceChannel(
                    "control.realization",
                    "diagnostic",
                    "enum",
                    None,
                    "Raw claim-boundary diagnostic retained with the canonical status view.",
                    availability=availability,
                    provenance="derived",
                    sampling="truth_boundary",
                    binding={"batch_report": "runtime.control_realization"},
                    claim_boundary="Diagnostic provenance; not an independent physical-control claim.",
                ),
            ),
        )
        ####

    ####


def passive_body_interface_extension() -> PassiveBodyInterfaceExtension:
    """Construct the package-owned passive truth interface on demand."""

    return PassiveBodyInterfaceExtension()
    ####


__all__ = ["PassiveBodyInterfaceExtension", "passive_body_interface_extension"]
