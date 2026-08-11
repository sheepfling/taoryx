"""Explicit source-default materialization for CADAC configuration builders."""

from __future__ import annotations

from taoryx.trajectory.configuration_contract import (
    ConfigurationChoiceSchema,
    ConfigurationGroupSchema,
    ConfigurationGroupValue,
    ConfigurationNode,
    ConfigurationNodeValue,
    ConfigurationOptionalSchema,
    ConfigurationParameterSchema,
    ConfigurationParameterValue,
    ConfigurationSequenceSchema,
    ConfigurationSequenceValue,
    TrajectoryConfigurationInstance,
    TrajectoryConfigurationSchema,
)


def materialize_configuration_defaults(
    schema: TrajectoryConfigurationSchema,
    configuration: TrajectoryConfigurationInstance,
) -> TrajectoryConfigurationInstance:
    """Fill only recursively declared defaults before validating a CADAC case.

    CADAC input decks establish a source-default tree. The common Taoryx
    contract requires that tree to be explicit rather than treating a missing
    required group as an implicit object. This helper creates those groups only
    when all descendants declare a default or are themselves optional.
    """

    if not isinstance(schema.root, ConfigurationGroupSchema) or not isinstance(configuration.root, ConfigurationGroupValue):
        return configuration
    ####
    return configuration.model_copy(update={"root": _materialize_group(schema.root, configuration.root)})
    ####


def _materialize_group(schema: ConfigurationGroupSchema, value: ConfigurationGroupValue) -> ConfigurationGroupValue:
    resolved: dict[str, ConfigurationNodeValue] = dict(value.values)
    for child in schema.children:
        existing = resolved.get(child.id)
        if isinstance(child, ConfigurationParameterSchema):
            if existing is None and child.default_declared:
                resolved[child.id] = ConfigurationParameterValue(value=child.default, unit=child.canonical_unit)
            elif (
                isinstance(existing, ConfigurationParameterValue)
                and existing.unit is None
                and child.canonical_unit is not None
                and existing.value == child.default
            ):
                resolved[child.id] = existing.model_copy(update={"unit": child.canonical_unit})
            ####
        elif isinstance(child, ConfigurationGroupSchema):
            if isinstance(existing, ConfigurationGroupValue):
                resolved[child.id] = _materialize_group(child, existing)
            elif existing is None and _can_materialize(child):
                resolved[child.id] = _materialize_group(child, ConfigurationGroupValue())
            ####
        elif isinstance(child, ConfigurationSequenceSchema):
            if existing is None and child.minimum_items == 0:
                resolved[child.id] = ConfigurationSequenceValue()
            ####
    ####
    return ConfigurationGroupValue(values=resolved)
    ####


def _can_materialize(node: ConfigurationNode) -> bool:
    if isinstance(node, ConfigurationParameterSchema):
        return node.default_declared or not node.required
    ####
    if isinstance(node, ConfigurationOptionalSchema):
        return True
    ####
    if isinstance(node, ConfigurationSequenceSchema):
        return node.minimum_items == 0
    ####
    if isinstance(node, ConfigurationGroupSchema):
        return all(_can_materialize(child) for child in node.children)
    ####
    if isinstance(node, ConfigurationChoiceSchema):
        return False
    ####
    raise TypeError(f"unsupported CADAC configuration node {node!r}")
    ####


__all__ = ["materialize_configuration_defaults"]
####
