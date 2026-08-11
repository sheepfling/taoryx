"""Shared semantic quantities for CADAC Mission Composition output channels."""

_UNIT_QUANTITIES = {
    "m": "length",
    "in": "length",
    "m/s": "velocity",
    "m/s^2": "acceleration",
    "g": "acceleration",
    "deg": "angle",
    "rad": "angle",
    "deg/s": "angular_velocity",
    "rad/s": "angular_velocity",
    "rpm": "angular_velocity",
    "s": "time",
    "kg": "mass",
    "kg/s": "mass_flow_rate",
    "kg*m^2": "moment_of_inertia",
    "N": "force",
    "N*m": "moment",
    "Pa": "pressure",
    "K": "temperature",
    "kg/m^3": "density",
}


def cadac_output_quantity(channel_id: str, unit: str | None, data_type: str = "float64") -> str:
    """Return a portable physical or semantic quantity for one CADAC channel.

    CADAC names carry useful source provenance, but consumers must not need to
    parse those names to distinguish a physical scalar from a mode, flag, or
    orientation. This function is the single family-level normalization point
    used by all CADAC output schemas.
    """

    if unit is not None:
        try:
            return _UNIT_QUANTITIES[unit]
        except KeyError as error:
            raise ValueError(f"CADAC output channel {channel_id!r} uses an unmapped unit {unit!r}") from error
        ####
    key = channel_id.casefold()
    if "quaternion" in key:
        return "orientation"
    ####
    if data_type == "boolean" or key.endswith(("_active", "_alive", "_held", "_limited", "_flag")):
        return "boolean"
    ####
    if data_type == "int64" or key.endswith(("_count", "_sequence")):
        return "count"
    ####
    if data_type == "json" or key == "telemetry":
        return "provider_telemetry"
    ####
    if data_type == "string" or any(token in key for token in ("mode", "phase", "kind", "fidelity", "realization", "status", "model_effective")):
        return "categorical"
    ####
    if key == "mach":
        return "mach_number"
    ####
    return "dimensionless"


####


def validate_cadac_output_schema(schema: object) -> None:
    """Reject incomplete CADAC channel semantics or ungrouped telemetry."""

    from taoryx.trajectory.configuration_contract import TrajectoryOutputSchema

    if not isinstance(schema, TrajectoryOutputSchema):
        raise TypeError("CADAC output validation requires TrajectoryOutputSchema")
    ####
    channels = (*schema.core_channels, *schema.telemetry_channels)
    missing_quantities = tuple(channel.id for channel in channels if not channel.quantity)
    if missing_quantities:
        raise ValueError(f"CADAC output channels are missing semantic quantities: {missing_quantities!r}")
    ####
    telemetry_ids = {channel.id for channel in schema.telemetry_channels}
    grouped_ids = {channel_id for group in schema.telemetry_groups for channel_id in group.channel_ids}
    missing_groups = tuple(sorted(telemetry_ids - grouped_ids))
    unknown_group_channels = tuple(sorted(grouped_ids - telemetry_ids))
    if missing_groups:
        raise ValueError(f"CADAC telemetry channels are not assigned to an output group: {missing_groups!r}")
    ####
    if unknown_group_channels:
        raise ValueError(f"CADAC output groups reference unknown telemetry channels: {unknown_group_channels!r}")
    ####


####


def validate_cadac_model_io_contract(model: object) -> None:
    """Validate unitized controls and their standard-output evidence links."""

    from collections.abc import Mapping, Sequence

    from taoryx.trajectory.configuration_contract import TrajectoryModelMetadata

    if not isinstance(model, TrajectoryModelMetadata):
        raise TypeError("CADAC model I/O validation requires TrajectoryModelMetadata")
    ####
    output_by_id = {channel.id: channel for channel in model.output_schema.channels}
    for realization in model.realizations:
        for control in realization.controls.channels:
            if not control.operations:
                continue
            ####
            if not control.quantity:
                raise ValueError(f"CADAC control {model.id!r}/{realization.id!r}/{control.id!r} is missing a semantic quantity")
            ####
            evidence = control.provider_binding.get("output_evidence")
            if not isinstance(evidence, Mapping):
                raise ValueError(f"CADAC external control {model.id!r}/{realization.id!r}/{control.id!r} has no requested/realized standard-output evidence")
            ####
            requested = evidence.get("requested")
            realized = evidence.get("realized")
            if not isinstance(requested, Mapping) or not isinstance(realized, Sequence) or not realized:
                raise ValueError(f"CADAC control {control.id!r} has malformed output evidence")
            ####
            _validate_evidence_endpoint(
                model.id,
                control.id,
                "requested",
                requested,
                output_by_id,
            )
            for index, endpoint in enumerate(realized):
                if not isinstance(endpoint, Mapping):
                    raise ValueError(f"CADAC control {control.id!r} realized evidence {index} is malformed")
                ####
                _validate_evidence_endpoint(
                    model.id,
                    control.id,
                    f"realized[{index}]",
                    endpoint,
                    output_by_id,
                )
            ####
        ####
    ####


####


def _validate_evidence_endpoint(
    model_id: str,
    control_id: str,
    role: str,
    endpoint: object,
    output_by_id: object,
) -> None:
    """Validate one control-evidence reference and optional vector component."""

    from collections.abc import Mapping

    if not isinstance(endpoint, Mapping) or not isinstance(output_by_id, Mapping):
        raise TypeError("CADAC control evidence validation requires mappings")
    ####
    channel_id = endpoint.get("channel_id")
    if not isinstance(channel_id, str) or channel_id not in output_by_id:
        raise ValueError(f"CADAC control {model_id!r}/{control_id!r} {role} evidence references unknown output {channel_id!r}")
    ####
    component = endpoint.get("component")
    if component is None:
        return
    ####
    if not isinstance(component, int) or component < 0:
        raise ValueError(f"CADAC control {control_id!r} {role} component must be a nonnegative integer")
    ####
    channel = output_by_id[channel_id]
    shape = channel.shape
    if not shape or shape[0] == "variable" or component >= shape[0]:
        raise ValueError(f"CADAC control {control_id!r} {role} component {component} is invalid for output {channel_id!r}")
    ####


####


__all__ = [
    "cadac_output_quantity",
    "validate_cadac_model_io_contract",
    "validate_cadac_output_schema",
]
