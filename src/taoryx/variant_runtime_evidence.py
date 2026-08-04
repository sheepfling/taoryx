"""Provenance-preserving evidence that a selected variant reached runtime.

A compiled variant proves semantic resolution.  This module adds the separate
runtime claim: the selected value reached the exact native adapter input and
any declared status relation is visible at committed truth boundaries.  It
does not derive unmodeled resources or promote a hard-valid variant to a
qualified operating envelope.
"""

from __future__ import annotations

import math
from collections.abc import Mapping

from .vehicle_composition import CompiledVehicleComposition


def build_variant_runtime_evidence(
    composition: CompiledVehicleComposition,
    status_trace: Mapping[str, object],
    *,
    consumed_native_inputs: Mapping[str, object],
) -> dict[str, object]:
    """Build one fail-closed runtime-consumption report for selected variants."""

    if not composition.variant.inputs:
        return {
            "schema": "taoryx.variant-runtime-evidence/v1alpha1",
            "composition_id": composition.id,
            "composition_identity_sha256": composition.identity_sha256,
            "status": "not_applicable",
            "bindings": [],
            "claim_boundary": "No variant was selected for this composition.",
        }
    initial_values, final_values = _status_boundary_values(status_trace)
    bindings: list[dict[str, object]] = []
    for identifier, resolved in composition.variant.inputs.items():
        binding = composition.variant.runtime_bindings.get(identifier)
        if binding is None:
            raise ValueError(f"selected variant {identifier!r} has no compiled runtime binding")
        consumed = consumed_native_inputs.get(binding.runtime_input_path)
        input_match = _equivalent(resolved.value, consumed)
        channel_records: list[dict[str, object]] = []
        channel_pass = True
        for channel_id in binding.derived_status_channels:
            initial = initial_values.get(channel_id)
            final = final_values.get(channel_id)
            available = channel_id in initial_values and channel_id in final_values
            relation_match: bool | None = None
            if binding.status_derivation_relation == "equal_to_target" and available:
                relation_match = _equivalent(resolved.value, initial)
            channel_pass = channel_pass and available and relation_match is not False
            channel_records.append(
                {
                    "id": channel_id,
                    "initial_value": initial,
                    "final_value": final,
                    "available_at_committed_boundaries": available,
                    "relation": binding.status_derivation_relation,
                    "relation_match": relation_match,
                }
            )
        status = "pass" if input_match and channel_pass else "fail"
        bindings.append(
            {
                "id": identifier,
                "status": status,
                "resolved_value": resolved.value,
                "canonical_unit": resolved.canonical_unit,
                "runtime_adapter_id": binding.runtime_adapter_id,
                "runtime_input_path": binding.runtime_input_path,
                "consumed_native_value": consumed,
                "consumed_native_input_match": input_match,
                "coupling_group": binding.coupling_group,
                "coupling_policy": binding.coupling_policy,
                "resource_derivation": binding.resource_derivation,
                "status_channels": channel_records,
                "derivation_claim_boundary": binding.derivation_claim_boundary,
            }
        )
    return {
        "schema": "taoryx.variant-runtime-evidence/v1alpha1",
        "composition_id": composition.id,
        "composition_identity_sha256": composition.identity_sha256,
        "status": "pass" if all(item["status"] == "pass" for item in bindings) else "fail",
        "bindings": bindings,
        "claim_boundary": (
            "This verifies declared native-input consumption and declared committed-status relations only. It does not "
            "derive missing resources, retrim the plant, or establish qualification beyond the resolved variant report."
        ),
    }
    ####


def _status_boundary_values(status_trace: Mapping[str, object]) -> tuple[Mapping[str, object], Mapping[str, object]]:
    """Return the first and final committed status mappings without interpolation."""

    samples = status_trace.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError("variant runtime evidence requires a nonempty committed status trace")
    first = samples[0]
    final = samples[-1]
    if not isinstance(first, Mapping) or not isinstance(final, Mapping):
        raise ValueError("variant runtime evidence status trace samples must be mappings")
    first_values = first.get("values")
    final_values = final.get("values")
    if not isinstance(first_values, Mapping) or not isinstance(final_values, Mapping):
        raise ValueError("variant runtime evidence status trace samples require values mappings")
    return first_values, final_values
    ####


def _equivalent(expected: object, actual: object) -> bool:
    """Compare finite scalar provenance values without coercing arbitrary types."""

    if isinstance(expected, bool) or isinstance(actual, bool):
        return expected is actual
    if isinstance(expected, int | float) and isinstance(actual, int | float):
        if not math.isfinite(float(expected)) or not math.isfinite(float(actual)):
            return False
        return math.isclose(float(expected), float(actual), rel_tol=1.0e-12, abs_tol=1.0e-12)
    return expected == actual
    ####


__all__ = ["build_variant_runtime_evidence"]
