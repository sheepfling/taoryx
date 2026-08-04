"""Discoverable vehicle-to-trajectory composition registry.

This module is deliberately a *projection*, not a second vehicle model
database.  Physics, source provenance, and fidelity readiness continue to
belong to their existing authorities.  The composition registry adds the
missing user-facing answer: given a vehicle-family selection, which fidelity
tiers, initialization contracts, reusable segment kinds, and mission
templates may be requested?

Entries may describe planned work.  ``promotion_status`` on the resolved
fidelity record remains the authoritative distinction between declared,
development, and qualified availability.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .family_manifest import UnifiedFamilyManifest, UnifiedFamilyManifestCatalog, load_unified_family_manifest_catalog
from .fidelity_contracts import CANONICAL_FIDELITY_TIERS, ControlRealization, FidelityTier, control_realization_for, runtime_fidelity_for
from .parameter_value_spaces import (
    parameter_value_space_contract,
    validate_parameter_value_space_coverage,
    value_space_for_parameter_profile,
)
from .value_space import (
    ValueSpaceSpec,
    bounded_interval,
    default_value_space_for_value_type,
    finite_set,
    periodic_circle,
    positive_half_line,
    quaternion_so3,
    unit_interval,
    validate_value_space_value,
)
from .vehicle_registry import ROOT

VEHICLE_COMPOSITION_REGISTRY = ROOT / "verification/vehicle_composition_registry.yaml"

CompositionStatus = Literal["runnable", "development", "planned"]
VariantBindingStatus = Literal["runnable", "planned"]
VariantDerivationMode = Literal["runtime_adapter_owned", "not_represented", "planned"]
VariantCouplingPolicy = Literal["exclusive", "composable"]
VariantStatusDerivationRelation = Literal["equal_to_target", "not_asserted"]


class ParameterSpec(BaseModel):
    """One bounded semantic parameter shared by composition surfaces.

    The registry originally exposed only a name, unit, and required flag.  A
    parameter is now a complete public contract even when an imported family
    has not established a numeric default, qualified interval, transform, or
    runtime-bound modifier.  ``None`` is therefore evidence of an absent
    declaration, never an invitation for a caller to infer one.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    canonical_unit: str | None = None
    value_type_declared: Literal["scalar", "vector3", "vector4", "enum"] | None = None
    required: bool = False
    description: str = Field(min_length=1)
    default_value: Any | None = None
    default_declared: bool = False
    hard_lower: float | None = None
    hard_upper: float | None = None
    qualified_lower: float | None = None
    qualified_upper: float | None = None
    safe_extended_lower: float | None = None
    safe_extended_upper: float | None = None
    transform: Literal["identity", "log", "logit", "simplex", "categorical"] = "identity"
    conditional_visibility: str | None = None
    coupling_group: str | None = None
    projection_policy: Literal["reject_invalid", "project_to_valid"] = "reject_invalid"
    derivation: str | None = None
    invalidations: tuple[str, ...] = ()
    provenance: str | None = None

    @model_validator(mode="after")
    def validate_parameter_contract(self) -> ParameterSpec:
        for label, lower, upper in (
            ("hard", self.hard_lower, self.hard_upper),
            ("qualified", self.qualified_lower, self.qualified_upper),
            ("safe extended", self.safe_extended_lower, self.safe_extended_upper),
        ):
            if lower is not None and upper is not None and lower > upper:
                raise ValueError(f"parameter {self.id!r} has inverted {label} bounds")
        _validate_nested_parameter_bounds(
            self.id,
            self.hard_lower,
            self.hard_upper,
            self.qualified_lower,
            self.qualified_upper,
            "qualified",
        )
        _validate_nested_parameter_bounds(
            self.id,
            self.hard_lower,
            self.hard_upper,
            self.safe_extended_lower,
            self.safe_extended_upper,
            "safe extended",
        )
        if self.default_declared and self.default_value is None:
            raise ValueError(f"parameter {self.id!r} declares a null default")
        if self.transform == "categorical" and self.value_type != "enum":
            raise ValueError(f"parameter {self.id!r} uses a categorical transform without enum representation")
        if self.transform in {"log", "logit", "simplex"} and self.value_type != "scalar":
            raise ValueError(f"parameter {self.id!r} transform {self.transform!r} requires scalar representation")
        if self.transform == "simplex":
            raise ValueError(
                f"parameter {self.id!r} uses a simplex transform, but this scalar schema has no simplex membership contract"
            )
        if self.transform == "log" and (self.hard_lower is None or self.hard_lower <= 0.0):
            raise ValueError(f"parameter {self.id!r} log transform requires a strictly positive hard_lower")
        if self.transform == "logit" and (self.hard_lower != 0.0 or self.hard_upper != 1.0):
            raise ValueError(f"parameter {self.id!r} logit transform requires hard bounds [0, 1]")
        if self.default_declared:
            validate_value_space_value(self.value_space, self.default_value, context=f"parameter {self.id!r} default")
            if isinstance(self.default_value, int | float) and not isinstance(self.default_value, bool):
                numeric = float(self.default_value)
                hard_lower, hard_upper = self.hard_bounds
                if hard_lower is not None and numeric < hard_lower:
                    raise ValueError(f"parameter {self.id!r} default is below hard validity")
                if hard_upper is not None and numeric > hard_upper:
                    raise ValueError(f"parameter {self.id!r} default is above hard validity")
        return self
        ####

    @property
    def value_type(self) -> Literal["scalar", "vector3", "vector4", "enum"]:
        """Return the reviewed portable representation for this input.

        The registry predates rich parameter records.  This migration keeps a
        single explicit mapping at the registry boundary, so callers do not
        reconstruct shapes from units or source-file conventions.
        """

        if self.value_type_declared is not None:
            return self.value_type_declared
        catalog_contract = parameter_value_space_contract(self.id)
        if catalog_contract is not None:
            return catalog_contract.value_type
        if self.id in {"turn_direction", "cutoff_condition", "terminal_kind", "area_policy"}:
            return "enum"
        if self.id == "quaternion_wxyz":
            return "vector4"
        if self.id.endswith("_ned_m") or self.id == "body_rates_rad_s":
            return "vector3"
        return "scalar"
        ####

    @property
    def value_space(self) -> ValueSpaceSpec:
        """Return the reviewed value-space contract for this input.

        Every registry-owned initialization and segment input is resolved from
        the versioned parameter value-space catalog.  The conservative fallback
        keeps standalone, ad-hoc ``ParameterSpec`` fixtures usable, but catalog
        coverage is mandatory when a composition registry is loaded.
        """

        if self.transform == "logit":
            return unit_interval()
        catalog_contract = parameter_value_space_contract(self.id)
        if catalog_contract is not None:
            return catalog_contract.value_space
        if self.value_type == "enum":
            return finite_set()
        if self.id == "quaternion_wxyz":
            return quaternion_so3()
        if self.id in {"heading_deg", "target_heading_deg", "initial_heading_deg", "launch_azimuth_deg", "longitude_deg"}:
            return periodic_circle(360.0)
        if self.id in {
            "latitude_deg",
            "bank_limit_deg",
            "launch_elevation_deg",
            "glide_bank_deg",
            "pitch_program",
            "requested_bank_deg",
            "maximum_bank_deg",
        }:
            return bounded_interval()
        if self.id.endswith("_m") or self.id.endswith("_m_s") or self.id.endswith("_s") or self.id.endswith("_kg"):
            if self.value_type == "scalar":
                return positive_half_line()
        if self.id in {"mach", "target_mach", "load_factor_limit", "max_dynamic_pressure_pa"}:
            return positive_half_line()
        return default_value_space_for_value_type(self.value_type)
        ####

    @property
    def hard_bounds(self) -> tuple[float | None, float | None]:
        """Return known hard numeric bounds without inventing qualified ones."""

        if self.hard_lower is not None or self.hard_upper is not None:
            return (self.hard_lower, self.hard_upper)
        if self.id == "latitude_deg":
            return (-90.0, 90.0)
        if self.id == "bank_limit_deg":
            return (0.0, 89.0)
        if self.id == "bank_command_deg":
            return (-89.0, 89.0)
        if self.value_space.topology == "positive_half_line":
            return (0.0, None)
        return (None, None)
        ####

    @property
    def options(self) -> tuple[str, ...]:
        """Return finite options when the existing registry declares them."""

        if self.id == "turn_direction":
            return ("left", "right")
        if self.id == "area_policy":
            return (
                "orientation_averaged_projected_area",
                "native_rigid_body_reuse_instantaneous_projected_area",
            )
        return ()
        ####

    def public_dict(self) -> dict[str, object]:
        """Return the composition-facing parameter descriptor."""

        payload = self.model_dump(mode="json")
        payload.update(
            {
                "value_type": self.value_type,
                "value_type_source": (
                    "declared"
                    if self.value_type_declared is not None
                    else "parameter_value_space_catalog"
                    if parameter_value_space_contract(self.id) is not None
                    else "ad_hoc_legacy_mapping"
                ),
                "value_space": self.value_space.as_dict(),
                "value_space_source": (
                    "declared_value_type"
                    if self.value_type_declared is not None
                    else "parameter_value_space_catalog"
                    if parameter_value_space_contract(self.id) is not None
                    else "ad_hoc_legacy_mapping"
                ),
                "hard_lower": self.hard_bounds[0],
                "hard_upper": self.hard_bounds[1],
                "qualified_lower": self.qualified_lower,
                "qualified_upper": self.qualified_upper,
                "safe_extended_lower": self.safe_extended_lower,
                "safe_extended_upper": self.safe_extended_upper,
                "options": list(self.options),
                "default": self.default_value if self.default_declared else None,
                "default_declared": self.default_declared,
                "transform": self.transform,
                "conditional_visibility": self.conditional_visibility,
                "coupling_group": self.coupling_group,
                "projection_policy": self.projection_policy,
                "derivation": self.derivation,
                "invalidations": list(self.invalidations),
                "provenance": self.provenance,
            }
        )
        catalog_contract = parameter_value_space_contract(self.id)
        if catalog_contract is not None and catalog_contract.frame is not None:
            payload["frame"] = catalog_contract.frame
        return payload
        ####


####


# Compatibility name for existing registry and compiler annotations.  New
# callers should use ``ParameterSpec`` to make the shared contract explicit.
CompositionParameter = ParameterSpec

_PARAMETER_CONTRACT_MATURITY_KEYS = (
    "declared_default_count",
    "explicit_hard_bound_count",
    "effective_hard_bound_count",
    "qualified_range_count",
    "safe_extended_range_count",
    "non_identity_transform_count",
    "provenance_count",
    "derivation_count",
    "runtime_backed_variant_count",
)


def _parameter_contract_maturity(
    parameters: Iterable[CompositionParameter | VariantParameterBinding],
) -> dict[str, int]:
    """Count declared parameter-contract evidence without judging its quality.

    The result is deliberately inventory-like: absence stays visible to a
    vehicle author but does not itself mean the family is invalid or
    unqualified.  Segment values may be derived by a capability adapter rather
    than possessing one fixed source range.
    """

    counts = {key: 0 for key in _PARAMETER_CONTRACT_MATURITY_KEYS}
    for parameter in parameters:
        if isinstance(parameter, ParameterSpec):
            raw_hard_bounds = (parameter.hard_lower, parameter.hard_upper)
            effective_hard_bounds = parameter.hard_bounds
            declared_default = parameter.default_declared
            provenance = parameter.provenance
            runtime_backed_variant = False
        else:
            raw_hard_bounds = (parameter.hard_lower, parameter.hard_upper)
            effective_hard_bounds = raw_hard_bounds
            declared_default = False
            provenance = parameter.provenance
            runtime_backed_variant = parameter.status == "runnable"
        counts["declared_default_count"] += int(declared_default)
        counts["explicit_hard_bound_count"] += int(any(value is not None for value in raw_hard_bounds))
        counts["effective_hard_bound_count"] += int(any(value is not None for value in effective_hard_bounds))
        counts["qualified_range_count"] += int(parameter.qualified_lower is not None or parameter.qualified_upper is not None)
        counts["safe_extended_range_count"] += int(parameter.safe_extended_lower is not None or parameter.safe_extended_upper is not None)
        counts["non_identity_transform_count"] += int(parameter.transform != "identity")
        counts["provenance_count"] += int(provenance is not None and bool(provenance.strip()))
        counts["derivation_count"] += int(parameter.derivation is not None and bool(parameter.derivation.strip()))
        counts["runtime_backed_variant_count"] += int(runtime_backed_variant)
    return counts
    ####


def _validate_nested_parameter_bounds(
    parameter_id: str,
    hard_lower: float | None,
    hard_upper: float | None,
    nested_lower: float | None,
    nested_upper: float | None,
    label: str,
) -> None:
    """Reject an evidence range that extends beyond hard mathematical validity."""

    if hard_lower is not None and nested_lower is not None and nested_lower < hard_lower:
        raise ValueError(f"parameter {parameter_id!r} has {label} lower bound below hard validity")
    if hard_upper is not None and nested_upper is not None and nested_upper > hard_upper:
        raise ValueError(f"parameter {parameter_id!r} has {label} upper bound above hard validity")
    ####


class InitializationContract(BaseModel):
    """Family-appropriate initial-state contract and its public inputs."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    status: CompositionStatus = "development"
    description: str = Field(min_length=1)
    compatible_fidelities: tuple[FidelityTier, ...] = Field(min_length=1)
    parameters: tuple[CompositionParameter, ...] = ()

    @model_validator(mode="after")
    def validate_parameters(self) -> InitializationContract:
        ids = [item.id for item in self.parameters]
        if len(ids) != len(set(ids)):
            raise ValueError(f"initialization {self.id!r} has duplicate parameters")
        return self
        ####


####


class SegmentContract(BaseModel):
    """One composable semantic segment, independent of a native controller."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    status: CompositionStatus = "development"
    description: str = Field(min_length=1)
    compatible_fidelities: tuple[FidelityTier, ...] = Field(min_length=1)
    parameters: tuple[CompositionParameter, ...] = ()
    required_control_intents: tuple[str, ...] = ()
    preserves_state: bool = True
    permitted_transition_events: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_parameters(self) -> SegmentContract:
        ids = [item.id for item in self.parameters]
        if len(ids) != len(set(ids)):
            raise ValueError(f"segment {self.id!r} has duplicate parameters")
        return self
        ####


####


class VariantParameterBinding(BaseModel):
    """One bounded variant input with an exact adapter-consumption trace."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    canonical_unit: str | None = None
    target_initialization_id: str = Field(min_length=1)
    target_parameter_id: str = Field(min_length=1)
    status: VariantBindingStatus
    compatible_fidelities: tuple[FidelityTier, ...] = ()
    runtime_adapter_id: str | None = None
    runtime_input_path: str | None = None
    hard_lower: float | None = None
    hard_upper: float | None = None
    qualified_lower: float | None = None
    qualified_upper: float | None = None
    safe_extended_lower: float | None = None
    safe_extended_upper: float | None = None
    value_space_profile: Literal["bounded_interval", "euclidean_scalar", "positive_half_line"] | None = None
    transform: Literal["identity", "log", "logit", "simplex", "categorical"] = "identity"
    conditional_visibility: str | None = None
    requires_retrim: bool = False
    requires_requalification: bool = False
    coupling_group: str | None = None
    coupling_policy: VariantCouplingPolicy = "exclusive"
    derived_status_channels: tuple[str, ...] = ()
    status_derivation_relation: VariantStatusDerivationRelation = "not_asserted"
    resource_derivation: VariantDerivationMode = "planned"
    derivation_claim_boundary: str = ""
    derivation: str | None = None
    invalidations: tuple[str, ...] = ()
    resolution_policy: Literal["reject_invalid", "project_to_valid"] = "reject_invalid"
    provenance: str = Field(min_length=1)
    description: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_runtime_trace(self) -> VariantParameterBinding:
        if self.hard_lower is not None and self.hard_upper is not None and self.hard_lower > self.hard_upper:
            raise ValueError(f"variant {self.id!r} has inverted hard bounds")
        for label, lower, upper in (
            ("qualified", self.qualified_lower, self.qualified_upper),
            ("safe extended", self.safe_extended_lower, self.safe_extended_upper),
        ):
            if lower is not None and upper is not None and lower > upper:
                raise ValueError(f"variant {self.id!r} has inverted {label} bounds")
        _validate_nested_parameter_bounds(
            self.id,
            self.hard_lower,
            self.hard_upper,
            self.qualified_lower,
            self.qualified_upper,
            "qualified",
        )
        _validate_nested_parameter_bounds(
            self.id,
            self.hard_lower,
            self.hard_upper,
            self.safe_extended_lower,
            self.safe_extended_upper,
            "safe extended",
        )
        # Runtime variant values are deliberately scalar today. A simplex
        # needs a jointly resolved vector and categorical values need a
        # declared finite option set; neither can be represented honestly by
        # the scalar runtime input path.
        if self.transform in {"simplex", "categorical"}:
            raise ValueError(
                f"variant {self.id!r} transform {self.transform!r} is unsupported by the scalar runtime variant contract"
            )
        if self.transform == "log" and (self.hard_lower is None or self.hard_lower <= 0.0):
            raise ValueError(f"variant {self.id!r} log transform requires a strictly positive hard_lower")
        if self.transform == "logit" and (self.hard_lower != 0.0 or self.hard_upper != 1.0):
            raise ValueError(f"variant {self.id!r} logit transform requires hard bounds [0, 1]")
        if self.value_space_profile is not None:
            value_space_for_parameter_profile(self.value_space_profile)
        if self.status == "runnable" and self.value_space_profile is None:
            raise ValueError(f"runnable variant {self.id!r} requires an explicit value_space_profile")
        if self.status == "runnable" and (self.runtime_adapter_id is None or self.runtime_input_path is None):
            raise ValueError(f"runnable variant {self.id!r} requires runtime adapter and input path")
        if self.status == "runnable" and not self.compatible_fidelities:
            raise ValueError(f"runnable variant {self.id!r} requires compatible_fidelities")
        if self.status == "runnable" and (self.coupling_group is None or not self.coupling_group.strip()):
            raise ValueError(f"runnable variant {self.id!r} requires a coupling_group")
        if self.status == "runnable" and self.resource_derivation == "planned":
            raise ValueError(f"runnable variant {self.id!r} must declare resource_derivation")
        if self.status == "runnable" and not self.derivation_claim_boundary.strip():
            raise ValueError(f"runnable variant {self.id!r} requires a derivation_claim_boundary")
        if len(self.derived_status_channels) != len(set(self.derived_status_channels)):
            raise ValueError(f"variant {self.id!r} has duplicate derived_status_channels")
        if self.status_derivation_relation == "equal_to_target" and not self.derived_status_channels:
            raise ValueError(f"variant {self.id!r} equal_to_target relation requires a derived_status_channel")
        return self
        ####

    @property
    def value_space(self) -> ValueSpaceSpec:
        """Return the public mathematical contract for this numeric binding."""

        if self.transform == "logit":
            return unit_interval()
        if self.value_space_profile is not None:
            return value_space_for_parameter_profile(self.value_space_profile)
        return positive_half_line() if self.hard_lower is not None and self.hard_lower >= 0.0 else default_value_space_for_value_type("scalar")
        ####

    def public_dict(self) -> dict[str, object]:
        """Return a queryable modifier contract without hiding its binding."""

        return {
            "id": self.id,
            "canonical_unit": self.canonical_unit,
            "value_type": "scalar",
            "value_space": self.value_space.as_dict(),
            "value_space_source": "declared_profile" if self.value_space_profile is not None else "legacy_bound_inference",
            "value_space_profile": self.value_space_profile,
            "hard_lower": self.hard_lower,
            "hard_upper": self.hard_upper,
            "qualified_lower": self.qualified_lower,
            "qualified_upper": self.qualified_upper,
            "safe_extended_lower": self.safe_extended_lower,
            "safe_extended_upper": self.safe_extended_upper,
            "transform": self.transform,
            "conditional_visibility": self.conditional_visibility,
            "compatible_fidelities": list(self.compatible_fidelities),
            "status": self.status,
            "target": {
                "scope": "episode_reset",
                "initialization_id": self.target_initialization_id,
                "parameter_id": self.target_parameter_id,
            },
            "runtime_adapter_id": self.runtime_adapter_id,
            "runtime_input_path": self.runtime_input_path,
            "requires_retrim": self.requires_retrim,
            "requires_requalification": self.requires_requalification,
            "coupling_group": self.coupling_group,
            "coupling_policy": self.coupling_policy,
            "derived_status_channels": list(self.derived_status_channels),
            "status_derivation_relation": self.status_derivation_relation,
            "resource_derivation": self.resource_derivation,
            "derivation_claim_boundary": self.derivation_claim_boundary,
            "derivation": self.derivation,
            "invalidations": list(self.invalidations),
            "resolution_policy": self.resolution_policy,
            "provenance": self.provenance,
            "description": self.description,
            "claim_boundary": (
                "The binding proves this value reaches the named runtime input. It does not by itself qualify the resulting operating condition or mission."
            ),
        }
        ####


####


class MissionGraphExecutionExtension(BaseModel):
    """One non-template transition subset a native mission translator executes.

    Mission templates always provide their ordered success sequence.  A family
    may declare a narrowly bounded extra transition here when its native
    translator implements and validates that behavior.  This is deliberately
    declaration metadata rather than a generic permission for callers to add
    graph branches.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    fidelity: FidelityTier
    translator_id: str = Field(min_length=1)
    supported_transition_kinds: tuple[Literal["success", "timeout", "abort", "resource_limit", "envelope_limit"], ...] = Field(min_length=2)
    supported_state_transfers: tuple[Literal["previous_terminal_truth_state"], ...] = Field(min_length=1)
    timeout_constraint: dict[str, str] | None = None
    unsupported_transition_kinds: tuple[Literal["timeout", "abort", "resource_limit", "envelope_limit"], ...] = ()
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_extension_contract(self) -> MissionGraphExecutionExtension:
        if "success" not in self.supported_transition_kinds:
            raise ValueError("a graph execution extension must retain the template success transition")
        if len(set(self.supported_transition_kinds)) != len(self.supported_transition_kinds):
            raise ValueError("a graph execution extension has duplicate supported transition kinds")
        if set(self.supported_transition_kinds) & set(self.unsupported_transition_kinds):
            raise ValueError("a graph execution extension cannot both support and reject a transition kind")
        if "timeout" in self.supported_transition_kinds and self.timeout_constraint is None:
            raise ValueError("a timeout graph execution extension must declare its bounded timeout constraint")
        return self
        ####


####


class MissionTemplateContract(BaseModel):
    """One ordered composition recipe advertised by a vehicle family."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    status: CompositionStatus = "development"
    description: str = Field(min_length=1)
    initialization_contracts: tuple[str, ...] = Field(min_length=1)
    compatible_fidelities: tuple[FidelityTier, ...] = Field(min_length=1)
    segment_sequence: tuple[str, ...] = Field(min_length=1)
    backing_templates: tuple[str, ...] = ()
    qualification_missions: tuple[str, ...] = ()
    mission_capability_adapter_id: str | None = None
    mission_capability_fidelities: tuple[FidelityTier, ...] = ()
    semantic_translator_id: str | None = None
    semantic_translator_fidelities: tuple[FidelityTier, ...] = ()
    graph_execution_extension: MissionGraphExecutionExtension | None = None


####


class VehicleCompositionDeclaration(BaseModel):
    """Human-facing trajectory-composition overlay for one registered family."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    family_id: str = Field(min_length=1)
    initialization_contracts: tuple[InitializationContract, ...] = Field(min_length=1)
    segment_contracts: tuple[SegmentContract, ...] = Field(min_length=1)
    mission_templates: tuple[MissionTemplateContract, ...] = Field(min_length=1)
    variant_parameters: tuple[VariantParameterBinding, ...] = ()

    @model_validator(mode="after")
    def validate_references(self) -> VehicleCompositionDeclaration:
        initialization_ids = {item.id for item in self.initialization_contracts}
        segment_ids = {item.id for item in self.segment_contracts}
        if len(initialization_ids) != len(self.initialization_contracts):
            raise ValueError(f"family {self.family_id!r} has duplicate initialization contracts")
        if len(segment_ids) != len(self.segment_contracts):
            raise ValueError(f"family {self.family_id!r} has duplicate segment contracts")
        mission_ids = [item.id for item in self.mission_templates]
        if len(mission_ids) != len(set(mission_ids)):
            raise ValueError(f"family {self.family_id!r} has duplicate mission templates")
        variant_ids = [item.id for item in self.variant_parameters]
        if len(variant_ids) != len(set(variant_ids)):
            raise ValueError(f"family {self.family_id!r} has duplicate variant parameter IDs")
        initialization_by_id = {item.id: item for item in self.initialization_contracts}
        for variant in self.variant_parameters:
            initialization = initialization_by_id.get(variant.target_initialization_id)
            if initialization is None:
                raise ValueError(f"variant {variant.id!r} references unknown initialization {variant.target_initialization_id!r}")
            target_ids = {parameter.id for parameter in initialization.parameters}
            if variant.target_parameter_id not in target_ids:
                raise ValueError(f"variant {variant.id!r} references unknown initialization parameter {variant.target_parameter_id!r}")
            unsupported_variant_tiers = sorted(set(variant.compatible_fidelities) - set(initialization.compatible_fidelities))
            if unsupported_variant_tiers:
                raise ValueError(f"variant {variant.id!r} declares fidelities incompatible with {initialization.id!r}: {unsupported_variant_tiers}")
        for mission in self.mission_templates:
            unknown_initialization = sorted(set(mission.initialization_contracts) - initialization_ids)
            unknown_segments = sorted(set(mission.segment_sequence) - segment_ids)
            if unknown_initialization:
                raise ValueError(f"mission {mission.id!r} references unknown initialization: {unknown_initialization}")
            if unknown_segments:
                raise ValueError(f"mission {mission.id!r} references unknown segments: {unknown_segments}")
            if mission.graph_execution_extension is not None and mission.graph_execution_extension.fidelity not in mission.compatible_fidelities:
                raise ValueError(f"mission {mission.id!r} declares a graph extension for incompatible fidelity {mission.graph_execution_extension.fidelity!r}")
            for label, adapter_id, supported_fidelities in (
                ("mission capability", mission.mission_capability_adapter_id, mission.mission_capability_fidelities),
                ("semantic translator", mission.semantic_translator_id, mission.semantic_translator_fidelities),
            ):
                if adapter_id is None and supported_fidelities:
                    raise ValueError(f"mission {mission.id!r} declares {label} fidelities without an ID")
                if adapter_id is not None and not supported_fidelities:
                    raise ValueError(f"mission {mission.id!r} declares {label} ID without supported fidelities")
                if len(set(supported_fidelities)) != len(supported_fidelities):
                    raise ValueError(f"mission {mission.id!r} has duplicate {label} fidelities")
                unsupported = sorted(set(supported_fidelities) - set(mission.compatible_fidelities))
                if unsupported:
                    raise ValueError(f"mission {mission.id!r} declares {label} for incompatible fidelities: {unsupported}")
        return self
        ####


####


class VehicleCompositionRegistry(BaseModel):
    """Source registry for discoverable family trajectory composition."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(alias="schema", min_length=1)
    version: int = Field(ge=1)
    description: str = Field(min_length=1)
    vehicles: tuple[VehicleCompositionDeclaration, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_unique_families(self) -> VehicleCompositionRegistry:
        ids = [item.family_id for item in self.vehicles]
        if len(ids) != len(set(ids)):
            raise ValueError("composition registry contains duplicate family IDs")
        return self
        ####


####


def _initialization_public_dict(contract: InitializationContract) -> dict[str, object]:
    """Serialize one initialization contract with rich parameter records."""

    payload = contract.model_dump(mode="json")
    payload["parameters"] = [item.public_dict() for item in contract.parameters]
    return payload
    ####


def _segment_public_dict(contract: SegmentContract) -> dict[str, object]:
    """Serialize one segment contract with rich parameter records."""

    payload = contract.model_dump(mode="json")
    payload["parameters"] = [item.public_dict() for item in contract.parameters]
    return payload
    ####


def _authoring_parameter_dict(parameter: CompositionParameter | VariantParameterBinding) -> dict[str, object]:
    """Return one input schema without turning recommendations into compiler defaults."""

    payload = parameter.public_dict()
    required = parameter.required if isinstance(parameter, CompositionParameter) else False
    recommended_value = parameter.default_value if isinstance(parameter, CompositionParameter) and parameter.default_declared else None
    has_declared_default = recommended_value is not None
    payload["authoring_value"] = recommended_value
    payload["authoring_value_source"] = "declared_recommendation" if has_declared_default else "not_declared"
    if has_declared_default:
        payload["authoring_value_rule"] = "confirm_declared_default_explicitly" if required else "optional_declared_recommendation"
    else:
        payload["authoring_value_rule"] = "required" if required else "optional_explicit_override_only"
    return payload
    ####


def _authoring_input_completion(
    initialization_options: list[dict[str, object]],
    segment_sequence: list[dict[str, object]],
    variant_inputs: list[dict[str, object]],
) -> dict[str, object]:
    """Summarize the values an author must deliberately provide or confirm.

    A declared recommendation is useful for an authoring UI, but it is not an
    implicit compiler input.  Keeping this worklist beside the schema makes
    the distinction actionable without manufacturing an executable request.
    """

    def mandatory_inputs(inputs: object) -> list[str]:
        if not isinstance(inputs, list):
            raise ValueError("authoring input completion requires an input list")
        result: list[str] = []
        for item in inputs:
            if not isinstance(item, Mapping):
                raise ValueError("authoring input completion requires mapping inputs")
            input_id = item.get("id")
            rule = item.get("authoring_value_rule")
            if not isinstance(input_id, str) or not isinstance(rule, str):
                raise ValueError("authoring input completion requires input IDs and value rules")
            if rule in {"required", "confirm_declared_default_explicitly"}:
                result.append(input_id)
        return result

    initialization_worklist = [
        {
            "id": item["id"],
            "must_select_exactly_one": True,
            "required_or_confirmation_input_ids": mandatory_inputs(item.get("inputs")),
        }
        for item in initialization_options
    ]
    segment_worklist = [
        {
            "suggested_instance_id": f"{item['occurrence_index']:02d}-{item['segment_id']}",
            "segment_id": item["segment_id"],
            "required_or_confirmation_input_ids": mandatory_inputs(item.get("inputs")),
        }
        for item in segment_sequence
    ]
    return {
        "schema": "taoryx.vehicle-composition-authoring-completion/v1alpha1",
        "initialization_options": initialization_worklist,
        "segments": segment_worklist,
        "optional_variant_input_ids": [item["id"] for item in variant_inputs if item.get("authoring_value_rule") == "optional_explicit_override_only"],
        "declared_variant_recommendation_ids": [
            item["id"] for item in variant_inputs if item.get("authoring_value_rule") == "optional_declared_recommendation"
        ],
        "acceptance_rule": (
            "A request must supply every required input and explicitly repeat any required declared recommendation; "
            "recommendations are never inserted by the compiler."
        ),
    }
    ####


def _variant_authoring_worklist(
    bindings: tuple[VariantParameterBinding, ...],
) -> dict[str, object]:
    """Return the source-owned admission state for optional vehicle variants.

    A family declaration may have no safe mutable quantity at all.  This
    routine makes that an explicit and useful Product 3 result instead of
    inviting a caller to mutate an arbitrary nested model field.  A declared
    planned binding remains visible, but it cannot become runnable until the
    native runtime consumes its named input and emits the declared committed
    status evidence.
    """

    records: list[dict[str, object]] = []
    for binding in bindings:
        public = binding.public_dict()
        binding_status = binding.status
        if binding_status == "runnable":
            next_steps = [
                "retain the exact runtime input path and committed-status relation in every execution witness",
                "retrim and requalify whenever the declared invalidations apply",
            ]
        else:
            next_steps = [
                "bind one exact source-owned runtime input that consumes this value",
                "declare every coupled derived quantity and resource policy; do not leave a stale mass, fuel, or inertia field",
                "emit declared committed-status evidence and add one exact composed batch witness",
                "record retrim and requalification invalidations before promotion to runnable",
            ]
        records.append(
            {
                "id": binding.id,
                "status": binding_status,
                "target": public["target"],
                "compatible_fidelities": public["compatible_fidelities"],
                "runtime_binding": {
                    "adapter_id": binding.runtime_adapter_id,
                    "input_path": binding.runtime_input_path,
                },
                "coupling_group": binding.coupling_group,
                "resource_derivation": binding.resource_derivation,
                "derived_status_channels": public["derived_status_channels"],
                "invalidations": public["invalidations"],
                "next_steps": next_steps,
                "claim_boundary": public["claim_boundary"],
            }
        )
    status: Literal["runnable", "planned", "not_declared"]
    if any(item["status"] == "runnable" for item in records):
        status = "runnable"
    elif records:
        status = "planned"
    else:
        status = "not_declared"
    next_steps = (
        [
            "No bounded runtime variant is declared. Keep model fields immutable to callers until a source-owned runtime path can consume a coupled semantic modifier.",
            "Use episode-reset or segment inputs only for their separately declared meanings; they are not substitutes for a vehicle variant.",
        ]
        if not records
        else [
            "Promote only bindings that satisfy their per-variant source-owned admission checklist.",
        ]
    )
    return {
        "status": status,
        "variants": records,
        "next_steps": next_steps,
        "claim_boundary": (
            "A runnable binding verifies only declared native-input consumption and committed-status relations. "
            "It does not establish a qualified loadout, resource model, trim solution, or mission envelope."
        ),
    }
    ####


def _mission_public_dict(contract: MissionTemplateContract) -> dict[str, object]:
    """Serialize one mission and its explicit current sequence graph.

    Composition v1 accepts an ordered sequence, not arbitrary branching. A
    graph projection still gives a client stable instance identities and the
    exact truth-state handoff order without implying that a timeout, abort, or
    fallback branch exists. The status makes this deliberately narrow behavior
    visible until the general mission-graph compiler is available.
    """

    payload = contract.model_dump(mode="json")
    instance_ids = tuple(f"{index:02d}-{segment_id}" for index, segment_id in enumerate(contract.segment_sequence, start=1))
    payload["graph"] = {
        "schema": "taoryx.mission-graph/v1alpha1",
        "status": "linear_sequence_only",
        "entry_instance_id": instance_ids[0],
        "nodes": [
            {
                "instance_id": instance_id,
                "segment_id": segment_id,
                "success_transition": None
                if index + 1 == len(instance_ids)
                else {"target_instance_id": instance_ids[index + 1], "state_transfer": "previous_terminal_truth_state"},
                "abort_transition": None,
                "resource_limit_transition": None,
                "envelope_limit_transition": None,
                "timeout_transition": None,
            }
            for index, (instance_id, segment_id) in enumerate(zip(instance_ids, contract.segment_sequence, strict=True))
        ],
        "claim_boundary": (
            "This graph is the exact ordered sequence accepted by the v1 composition compiler. "
            "It does not yet expose caller-configurable branching, fallback, or abort semantics."
        ),
    }
    return payload
    ####


def mission_graph_execution_contract(
    family_id: str,
    mission_id: str,
    fidelity: FidelityTier,
) -> dict[str, object]:
    """Return the exact transition subset declared executable by one family.

    The generic compiler can validate a wider typed graph than most current
    native translators execute. This record keeps that difference queryable at
    authoring time, without treating a graph declaration as an endpoint or
    qualification claim.
    """

    mission = _declared_mission_template(family_id, mission_id)
    if mission is not None:
        extension = mission.graph_execution_extension
        if extension is not None and extension.fidelity == fidelity:
            payload = extension.model_dump(mode="json")
            payload["status"] = "family_extension_declared"
            return payload
    return {
        "status": "template_success_sequence_only",
        "translator_id": None if mission is None else mission.semantic_translator_id,
        "supported_transition_kinds": ["success"],
        "supported_state_transfers": ["previous_terminal_truth_state"],
        "unsupported_transition_kinds": ["abort", "resource_limit", "envelope_limit", "timeout"],
        "claim_boundary": (
            "This declares only the template-owned ordered success sequence. A caller-authored branch or alternate "
            "state transfer remains blocked until the selected family translator explicitly declares it."
        ),
    }
    ####


def mission_semantic_translator_id(
    family_id: str,
    mission_id: str,
    fidelity: FidelityTier,
) -> str | None:
    """Return the exact native semantic translator declared for a mission tier.

    A missing value is deliberate: the composition may still be inspectable,
    planned, or have only a capability estimate, but it must not be presented
    as having an executable semantic lowering.  The mission field is shared by
    all of its compatible tiers unless a future template splits the mission.
    """

    mission = _declared_mission_template(family_id, mission_id)
    if mission is None or fidelity not in mission.semantic_translator_fidelities:
        return None
    return mission.semantic_translator_id
    ####


def _declared_mission_template(
    family_id: str,
    mission_id: str,
) -> MissionTemplateContract | None:
    """Find one mission declaration without guessing a nearby family or template."""

    registry = load_vehicle_composition_registry()
    declaration = next((item for item in registry.vehicles if item.family_id == family_id), None)
    if declaration is None:
        return None
    return next((item for item in declaration.mission_templates if item.id == mission_id), None)
    ####


@dataclass(frozen=True, slots=True)
class ResolvedVehicleComposition:
    """Composition entry joined to the canonical family/fidelity authority."""

    family: UnifiedFamilyManifest
    declaration: VehicleCompositionDeclaration

    def as_dict(self) -> dict[str, object]:
        """Return the user-facing inspect payload without copying plant data."""

        # Import here so the execution catalog can consume compiled
        # compositions without turning this discovery projection into a
        # registry-to-runtime import cycle.
        from .vehicle_execution_bindings import batch_episode_parity_record, execution_binding_records
        from .vehicle_interface import interface_contract_for_composition, validate_vehicle_interface_contract

        tier_records: dict[str, object] = {}
        interface_records: dict[str, object] = {}
        for tier in CANONICAL_FIDELITY_TIERS:
            binding = self.family.family.tiers[tier]
            tier_records[tier] = {
                "profile_id": binding.profile_id,
                "declared": binding.profile_id is not None,
                "promotion_status": binding.promotion_status,
                "blockers": list(binding.blockers),
                "required_operations": list(binding.required_operations),
                "runtime_fidelity": runtime_fidelity_for(tier),
                "control_realization": resolved_control_realization_for(self.family.family_id, tier),
            }
            interface = interface_contract_for_composition(self, tier)
            findings = validate_vehicle_interface_contract(interface)
            interface_records[tier] = {
                "interface_id": interface.id,
                "fingerprint_sha256": interface.fingerprint,
                "validation_status": "pass" if not findings else "fail",
                "available_authority_profiles": [item.id for item in interface.authority_profiles if item.availability == "available"],
                "available_observation_profiles": [item.id for item in interface.observation_profiles if item.availability == "available"],
            }
        parameter_records: tuple[CompositionParameter | VariantParameterBinding, ...] = (
            *(parameter for contract in self.declaration.initialization_contracts for parameter in contract.parameters),
            *(parameter for contract in self.declaration.segment_contracts for parameter in contract.parameters),
            *self.declaration.variant_parameters,
        )
        return {
            "schema": "taoryx.vehicle-composition/v1alpha1",
            "vehicle_id": self.family.family.vehicle_registry_id or self.family.family_id,
            "family_id": self.family.family_id,
            "display_name": (
                str(self.family.vehicle_definition.get("display_name"))
                if self.family.vehicle_definition is not None
                else self.family.source_manifest.display_name
                if self.family.source_manifest is not None
                else self.family.family_id
            ),
            "physical_family": self.family.family.physical_family,
            "mission_overlay": self.family.family.mission_overlay,
            "adapter_id": self.family.family.adapter_id,
            "automatic_lowering": self.family.family.automatic_lowering,
            "source_manifest": self.family.source_manifest_path,
            "fidelities": tier_records,
            "interfaces": interface_records,
            "initialization_contracts": [_initialization_public_dict(item) for item in self.declaration.initialization_contracts],
            "segment_contracts": [_segment_public_dict(item) for item in self.declaration.segment_contracts],
            "mission_templates": [_mission_public_dict(item) for item in self.declaration.mission_templates],
            "variant_parameters": [item.public_dict() for item in self.declaration.variant_parameters],
            "parameter_contract_maturity": _parameter_contract_maturity(parameter_records),
            "execution_bindings": execution_binding_records(self.family.family_id),
            "batch_episode_parity": [
                batch_episode_parity_record(self.family.family_id, template.id, tier)
                for template in self.declaration.mission_templates
                for tier in template.compatible_fidelities
            ],
        }
        ####

    def parameter_dict(self, scope: Literal["episode_reset", "segment", "variant_configuration"] | None = None) -> dict[str, object]:
        """Return queryable public inputs without requiring descriptor parsing."""

        records: list[dict[str, object]] = []
        if scope in {None, "episode_reset"}:
            for initialization_contract in self.declaration.initialization_contracts:
                records.extend(
                    {
                        **parameter.public_dict(),
                        "scope": "episode_reset",
                        "contract_id": initialization_contract.id,
                        "contract_status": initialization_contract.status,
                    }
                    for parameter in initialization_contract.parameters
                )
        if scope in {None, "segment"}:
            for segment_contract in self.declaration.segment_contracts:
                records.extend(
                    {
                        **parameter.public_dict(),
                        "scope": "segment",
                        "contract_id": segment_contract.id,
                        "contract_status": segment_contract.status,
                    }
                    for parameter in segment_contract.parameters
                )
        variant_records = [item.public_dict() for item in self.declaration.variant_parameters]
        if scope in {None, "variant_configuration"}:
            records.extend({**item, "scope": "variant_configuration", "contract_id": item["id"], "contract_status": item["status"]} for item in variant_records)
        variant_status = "runnable" if any(item.status == "runnable" for item in self.declaration.variant_parameters) else "planned"
        return {
            "schema": "taoryx.vehicle-parameter-query/v1alpha1",
            "vehicle_id": self.family.family.vehicle_registry_id or self.family.family_id,
            "family_id": self.family.family_id,
            "scope": scope,
            "parameters": records,
            "variant_space_status": variant_status,
            "claim_boundary": (
                "Initialization and segment inputs are currently queryable and compile-time validated. "
                "Variant entries are executable only when they name a source-owned runtime input binding. "
                "All other family variation remains planned."
            ),
        }
        ####

    def authoring_worklist_dict(self) -> dict[str, object]:
        """Return actionable Product 3 work items for one catalog member.

        This joins the existing declarations instead of guessing a new plant:
        profile/interface evidence, semantic mission availability, graph
        execution, and concrete batch/episode bindings stay separately
        visible. It is a composition-authoring checklist, not source intake or
        qualification evidence.
        """

        from .mission_capability import declared_mission_capability_adapter

        descriptor = self.as_dict()
        fidelities = descriptor["fidelities"]
        interfaces = descriptor["interfaces"]
        bindings = descriptor["execution_bindings"]
        parity_records = descriptor["batch_episode_parity"]
        parameter_maturity = descriptor["parameter_contract_maturity"]
        if (
            not isinstance(fidelities, Mapping)
            or not isinstance(interfaces, Mapping)
            or not isinstance(bindings, list)
            or not isinstance(parity_records, list)
            or not isinstance(parameter_maturity, Mapping)
        ):
            raise ValueError("resolved vehicle descriptor has invalid authoring fields")
        variant_worklist = _variant_authoring_worklist(self.declaration.variant_parameters)
        mission_worklists: list[dict[str, object]] = []
        for template in self.declaration.mission_templates:
            graph = _mission_public_dict(template)["graph"]
            graph_status = str(graph["status"]) if isinstance(graph, Mapping) else "invalid"
            tier_worklists: list[dict[str, object]] = []
            for tier in template.compatible_fidelities:
                fidelity = fidelities.get(tier)
                interface = interfaces.get(tier)
                selected = [item for item in bindings if isinstance(item, Mapping) and item.get("mission") == template.id and item.get("fidelity") == tier]
                runnable_operations = sorted(
                    str(item["operation"]) for item in selected if item.get("status") == "runnable" and isinstance(item.get("operation"), str)
                )
                batch_action_trace_dispositions = sorted(
                    str(item["batch_action_trace"]) for item in selected if item.get("operation") == "batch" and isinstance(item.get("batch_action_trace"), str)
                )
                execution_modes = sorted(
                    {
                        str(item["execution_mode"])
                        for item in selected
                        if isinstance(item.get("execution_mode"), str)
                    }
                )
                planned_execution_blockers = {
                    str(item["operation"]): [str(blocker) for blocker in item.get("blockers", []) if isinstance(blocker, str)]
                    for item in selected
                    if item.get("status") == "planned" and isinstance(item.get("operation"), str)
                }
                next_steps: list[str] = []
                capability_adapter_id = declared_mission_capability_adapter(self.family.family_id, template.id, tier)
                semantic_translator_id = mission_semantic_translator_id(self.family.family_id, template.id, tier)
                parity = next(
                    (item for item in parity_records if isinstance(item, Mapping) and item.get("mission") == template.id and item.get("fidelity") == tier),
                    None,
                )
                fidelity_promotion_blockers: list[str] = []
                if not isinstance(fidelity, Mapping) or fidelity.get("declared") is not True:
                    next_steps.append("declare canonical fidelity profile or mark it not applicable")
                else:
                    raw_promotion_blockers = fidelity.get("blockers", [])
                    if not isinstance(raw_promotion_blockers, list) or not all(isinstance(item, str) for item in raw_promotion_blockers):
                        raise ValueError(f"resolved fidelity {tier!r} has invalid promotion blockers")
                    fidelity_promotion_blockers = sorted(raw_promotion_blockers)
                    if fidelity.get("promotion_status") != "qualified":
                        if fidelity_promotion_blockers:
                            next_steps.append(
                                "retire declared fidelity-promotion blockers: "
                                + ", ".join(fidelity_promotion_blockers)
                            )
                        else:
                            next_steps.append("retain declared fidelity/evidence boundary and complete its promotion gates")
                if not isinstance(interface, Mapping) or interface.get("validation_status") != "pass":
                    next_steps.append("supply a fail-closed parameter/action/status interface contract")
                if "batch" not in runnable_operations:
                    next_steps.append("bind one source-owned batch factory and checked-in composition witness")
                for operation, blockers in sorted(planned_execution_blockers.items()):
                    if blockers:
                        next_steps.append(
                            f"resolve declared {operation} execution blockers: {', '.join(blockers)}"
                        )
                if "committed_interval_history_missing" in batch_action_trace_dispositions:
                    next_steps.append("retain committed interval command history in the native batch runner before claiming semantic action evidence")
                elif "not_emitted" in batch_action_trace_dispositions:
                    next_steps.append("emit the standardized committed semantic action trace or retain action/effect evidence explicitly unavailable")
                if graph_status != "linear_sequence_only":
                    next_steps.append("implement declared graph-transition semantics in the selected native translator")
                if capability_adapter_id is None:
                    next_steps.append("supply a family-owned first-pass mission capability adapter or retain explicit preflight gap")
                elif semantic_translator_id is None:
                    next_steps.append("declare the exact semantic translator before preflight can report translation_ready")
                status = "runnable" if "batch" in runnable_operations else "development" if selected else "planned"
                tier_worklists.append(
                    {
                        "tier": tier,
                        "status": status,
                        "interface_validation": None if not isinstance(interface, Mapping) else interface.get("validation_status"),
                        "fidelity_promotion_blockers": fidelity_promotion_blockers,
                        "runnable_operations": runnable_operations,
                        "batch_action_trace_dispositions": batch_action_trace_dispositions,
                        "execution_modes": execution_modes,
                        "planned_execution_blockers": planned_execution_blockers,
                        "batch_episode_parity": parity,
                        "declared_binding_count": len(selected),
                        "mission_capability_adapter": capability_adapter_id,
                        "semantic_translator_id": semantic_translator_id,
                        "graph_execution_contract": mission_graph_execution_contract(self.family.family_id, template.id, tier),
                        "next_steps": next_steps,
                    }
                )
            mission_worklists.append(
                {
                    "mission_id": template.id,
                    "mission_status": template.status,
                    "graph_status": graph_status,
                    "semantic_translator_id": template.semantic_translator_id,
                    "initialization_contracts": list(template.initialization_contracts),
                    "tiers": tier_worklists,
                }
            )
        return {
            "schema": "taoryx.vehicle-composition-authoring-worklist/v1alpha1",
            "vehicle_id": descriptor["vehicle_id"],
            "family_id": descriptor["family_id"],
            "physical_family": descriptor["physical_family"],
            "variant_parameters": descriptor["variant_parameters"],
            "variant_worklist": variant_worklist,
            "parameter_contract_maturity": dict(parameter_maturity),
            "missions": mission_worklists,
            "claim_boundary": (
                "This worklist joins declared Product 3 contracts and bindings. It identifies the next missing "
                "composition capability but does not inspect source equations, synthesize a runtime adapter, or "
                "promote qualification. The variant worklist is an admission checklist: it never authorizes a "
                "generic model-field override."
            ),
        }
        ####

    def authoring_kit_dict(self, mission_id: str, fidelity: str) -> dict[str, object]:
        """Return an explicit, non-executable composition-authoring kit.

        The kit is intentionally a schema-plus-checklist artifact rather than
        a YAML file containing invented numeric defaults.  A new vehicle or
        mission author can see every required value, its units and topology,
        its repeated segment-instance rule, the selected interface, and the
        next runtime/evidence gaps before creating a composition request.
        """

        from .mission_objectives import truth_objective_topology_schema

        mission_matches = tuple(item for item in self.declaration.mission_templates if item.id == mission_id)
        if not mission_matches:
            raise KeyError(f"unknown mission {mission_id!r} for family {self.family.family_id!r}")
        mission = mission_matches[0]
        if fidelity not in mission.compatible_fidelities:
            raise ValueError(f"mission {mission_id!r} is not declared for fidelity {fidelity!r} in family {self.family.family_id!r}")
        descriptor = self.as_dict()
        fidelities = descriptor["fidelities"]
        interfaces = descriptor["interfaces"]
        binding_records = descriptor["execution_bindings"]
        if not isinstance(fidelities, Mapping) or not isinstance(interfaces, Mapping) or not isinstance(binding_records, list):
            raise ValueError("resolved vehicle descriptor has invalid fidelity/interface records")
        selected_tier = fidelities.get(fidelity)
        selected_interface = interfaces.get(fidelity)
        if not isinstance(selected_tier, Mapping) or not isinstance(selected_interface, Mapping):
            raise ValueError(f"resolved vehicle descriptor lacks fidelity {fidelity!r}")
        initialization_options: list[dict[str, object]] = [
            {
                "id": item.id,
                "status": item.status,
                "inputs": [_authoring_parameter_dict(parameter) for parameter in item.parameters],
            }
            for item in self.declaration.initialization_contracts
            if item.id in mission.initialization_contracts and fidelity in item.compatible_fidelities
        ]
        segments_by_id = {item.id: item for item in self.declaration.segment_contracts}
        counts = {segment_id: mission.segment_sequence.count(segment_id) for segment_id in mission.segment_sequence}
        segment_sequence: list[dict[str, object]] = []
        for index, segment_id in enumerate(mission.segment_sequence, start=1):
            segment = segments_by_id.get(segment_id)
            if segment is None:
                raise ValueError(f"mission {mission_id!r} references unknown segment {segment_id!r}")
            segment_sequence.append(
                {
                    "occurrence_index": index,
                    "segment_id": segment.id,
                    "suggested_instance_id": f"{index:02d}-{segment.id}",
                    "segment_status": segment.status,
                    "instance_id_required": counts[segment_id] > 1,
                    "required_control_intents": list(segment.required_control_intents),
                    "permitted_transition_events": list(segment.permitted_transition_events),
                    "inputs": [_authoring_parameter_dict(parameter) for parameter in segment.parameters],
                }
            )
        worklist = self.authoring_worklist_dict()
        worklist_missions = worklist.get("missions")
        if not isinstance(worklist_missions, list):
            raise ValueError("vehicle authoring worklist has invalid mission records")
        mission_worklist = next(
            (item for item in worklist_missions if isinstance(item, Mapping) and item.get("mission_id") == mission_id),
            None,
        )
        if not isinstance(mission_worklist, Mapping):
            raise ValueError(f"vehicle authoring worklist lacks mission {mission_id!r}")
        mission_tiers = mission_worklist.get("tiers")
        if not isinstance(mission_tiers, list):
            raise ValueError(f"vehicle authoring worklist lacks tier records for mission {mission_id!r}")
        tier_worklist = next(
            (item for item in mission_tiers if isinstance(item, Mapping) and item.get("tier") == fidelity),
            None,
        )
        if not isinstance(tier_worklist, Mapping):
            raise ValueError(f"vehicle authoring worklist lacks fidelity {fidelity!r} for mission {mission_id!r}")
        variant_admission = worklist.get("variant_worklist")
        if not isinstance(variant_admission, Mapping):
            raise ValueError("vehicle authoring worklist has invalid variant admission")
        execution_endpoints = [
            {
                "operation": item.get("operation"),
                "status": item.get("status"),
                "execution_mode": item.get("execution_mode"),
                "factory_id": item.get("factory_id"),
                "batch_action_trace": item.get("batch_action_trace"),
                "description": item.get("description"),
                "claim_boundary": item.get("claim_boundary"),
                "blockers": item.get("blockers", []),
            }
            for item in binding_records
            if isinstance(item, Mapping) and item.get("mission") == mission_id and item.get("fidelity") == fidelity
        ]
        runnable_operations = {
            str(item["operation"]) for item in execution_endpoints if item.get("status") == "runnable" and isinstance(item.get("operation"), str)
        }
        authoring_commands = [
            "taoryx vehicle compose <request.yaml> --output <composition.json>",
            "taoryx vehicle preflight <composition.json>",
            "taoryx vehicle lower <composition.json>",
        ]
        if "batch" in runnable_operations:
            authoring_commands.extend(
                [
                    "taoryx vehicle run <composition.json> --output-dir <run-directory>",
                    "taoryx vehicle result <run-directory> --composition <composition.json>",
                ]
            )
        if "episode" in runnable_operations:
            authoring_commands.append("taoryx vehicle episode-info <composition.json>")
        parity = tier_worklist.get("batch_episode_parity")
        if isinstance(parity, Mapping) and parity.get("availability") == "registered":
            authoring_commands.append("taoryx vehicle batch-episode-parity <composition.json> <policy-trace.json>")
        variant_inputs = [_authoring_parameter_dict(parameter) for parameter in self.declaration.variant_parameters]
        request_shape = {
            "schema": "taoryx.vehicle-compose-request/v1alpha1",
            "id_rule": "caller_assigned_stable_semantic_identifier",
            "vehicle": descriptor["vehicle_id"],
            "fidelity": fidelity,
            "mission": {"id": mission_id},
            "initialization": {
                "select_exactly_one": initialization_options,
                "input_value_rule": "supply every required value in the stated canonical unit; do not infer numeric defaults",
            },
            "segments": segment_sequence,
            "mission_graph": _mission_public_dict(mission)["graph"],
            "graph_execution_contract": mission_graph_execution_contract(self.family.family_id, mission.id, fidelity),
            "variant_inputs": variant_inputs,
            "truth_objective_topology_schema": truth_objective_topology_schema(),
        }
        return {
            "schema": "taoryx.vehicle-composition-authoring-kit/v1alpha1",
            "vehicle_id": descriptor["vehicle_id"],
            "family_id": descriptor["family_id"],
            "physical_family": descriptor["physical_family"],
            "selection": {
                "mission_id": mission_id,
                "fidelity": fidelity,
                "control_realization": selected_tier.get("control_realization"),
                "runtime_fidelity": selected_tier.get("runtime_fidelity"),
                "tier_declared": selected_tier.get("declared"),
                "tier_promotion_status": selected_tier.get("promotion_status"),
                "interface_validation": selected_interface.get("validation_status"),
                "semantic_translator_id": tier_worklist.get("semantic_translator_id"),
            },
            "composition_request_shape": request_shape,
            "input_completion": _authoring_input_completion(initialization_options, segment_sequence, variant_inputs),
            "variant_admission": dict(variant_admission),
            "execution_endpoints": execution_endpoints,
            "runtime_and_evidence_worklist": tier_worklist,
            "authoring_commands": authoring_commands,
            "claim_boundary": (
                "This kit exposes declared composition inputs, value spaces, graph shape, variant admission, "
                "interface, and next runtime/evidence work. It is not an executable request, does not provide "
                "invented defaults, does not synthesize a plant or controller, and does not promote qualification."
            ),
        }
        ####


####


@dataclass(frozen=True, slots=True)
class ResolvedVehicleCompositionCatalog:
    """Validated complete user-facing vehicle registry."""

    vehicles: tuple[ResolvedVehicleComposition, ...]

    def vehicle(self, identifier: str) -> ResolvedVehicleComposition:
        """Resolve a stable family or native vehicle identifier."""

        matches = tuple(item for item in self.vehicles if identifier in {item.family.family_id, item.family.family.vehicle_registry_id})
        if not matches:
            raise KeyError(f"unknown vehicle or family identifier {identifier!r}")
        if len(matches) != 1:
            raise ValueError(f"ambiguous vehicle identifier {identifier!r}")
        return matches[0]
        ####

    def list_dict(self) -> list[dict[str, object]]:
        """Return compact records suitable for a command-line listing."""

        result: list[dict[str, object]] = []
        for item in self.vehicles:
            payload = item.as_dict()
            fidelities = payload["fidelities"]
            assert isinstance(fidelities, Mapping)
            execution_bindings = payload["execution_bindings"]
            assert isinstance(execution_bindings, list)
            result.append(
                {
                    "vehicle_id": payload["vehicle_id"],
                    "family_id": payload["family_id"],
                    "display_name": payload["display_name"],
                    "physical_family": payload["physical_family"],
                    "fidelities": {
                        name: record["promotion_status"] for name, record in fidelities.items() if isinstance(record, Mapping) and record["declared"]
                    },
                    "mission_templates": [template.id for template in item.declaration.mission_templates],
                    "runnable_operations": sorted(
                        {str(binding["operation"]) for binding in execution_bindings if isinstance(binding, Mapping) and binding.get("status") == "runnable"}
                    ),
                }
            )
        return result
        ####

    def authoring_worklist_dict(self) -> dict[str, object]:
        """Return every family-owned authoring worklist in one discovery document.

        The aggregate intentionally retains per-family details: a single
        scalar readiness score would erase whether a gap is a missing runtime,
        interface, graph transition, capability estimator, or evidence tier.
        """

        worklists = [item.authoring_worklist_dict() for item in self.vehicles]
        mission_tiers: list[Mapping[str, object]] = []
        for worklist in worklists:
            missions = worklist.get("missions")
            if not isinstance(missions, list):
                raise ValueError("family authoring worklist has no mission list")
            for mission in missions:
                if not isinstance(mission, Mapping):
                    raise ValueError("family authoring worklist contains an invalid mission record")
                tiers = mission.get("tiers")
                if not isinstance(tiers, list):
                    raise ValueError("family authoring worklist contains a mission without tiers")
                mission_tiers.extend(tier for tier in tiers if isinstance(tier, Mapping))
        batch_runnable_count = 0
        batch_action_trace_counts: Counter[str] = Counter()
        execution_mode_counts: Counter[str] = Counter()
        variant_space_status_counts: Counter[str] = Counter()
        runnable_variant_count = 0
        for worklist in worklists:
            variants = worklist.get("variant_worklist")
            if not isinstance(variants, Mapping):
                raise ValueError("family authoring worklist has invalid variant worklist")
            status = variants.get("status")
            if not isinstance(status, str):
                raise ValueError("family variant worklist has invalid status")
            variant_space_status_counts.update((status,))
            records = variants.get("variants")
            if not isinstance(records, list):
                raise ValueError("family variant worklist has invalid variant records")
            runnable_variant_count += sum(
                1 for item in records if isinstance(item, Mapping) and item.get("status") == "runnable"
            )
        for tier in mission_tiers:
            operations = tier.get("runnable_operations")
            if isinstance(operations, list) and "batch" in operations:
                batch_runnable_count += 1
            dispositions = tier.get("batch_action_trace_dispositions")
            if isinstance(dispositions, list):
                batch_action_trace_counts.update(item for item in dispositions if isinstance(item, str))
            execution_modes = tier.get("execution_modes")
            if isinstance(execution_modes, list):
                execution_mode_counts.update(item for item in execution_modes if isinstance(item, str))
        return {
            "schema": "taoryx.vehicle-composition-authoring-catalog/v1alpha1",
            "vehicle_count": len(worklists),
            "mission_tier_count": len(mission_tiers),
            "batch_runnable_mission_tier_count": batch_runnable_count,
            "batch_action_trace_disposition_counts": dict(sorted(batch_action_trace_counts.items())),
            "execution_mode_counts": dict(sorted(execution_mode_counts.items())),
            "variant_space_status_counts": dict(sorted(variant_space_status_counts.items())),
            "runnable_variant_count": runnable_variant_count,
            "vehicles": worklists,
            "claim_boundary": (
                "This catalog aggregates family authoring worklists. It does not merge their evidence, "
                "synthesize a missing plant adapter, or make an unqualified family runnable."
            ),
        }
        ####

    def as_dict(self, *, detail: Literal["summary", "full"] = "summary") -> dict[str, object]:
        """Return the versioned discovery document for composition clients.

        ``summary`` is suitable for menu or search discovery. ``full`` embeds
        each immutable descriptor so a remote client can inspect the complete
        advertised surface without a repository-local iteration over IDs.
        """

        if detail not in {"summary", "full"}:
            raise ValueError(f"unsupported catalog detail: {detail!r}")
        return {
            "schema": "taoryx.vehicle-composition-catalog/v1alpha1",
            "detail": detail,
            "vehicle_count": len(self.vehicles),
            "vehicles": self.list_dict() if detail == "summary" else [item.as_dict() for item in self.vehicles],
            "claim_boundary": (
                "This discovery catalog advertises declared composition and execution metadata. "
                "It does not resolve a variant, establish mission feasibility, or promote evidence."
            ),
        }
        ####


####


def load_vehicle_composition_registry(path: str | Path | None = None) -> VehicleCompositionRegistry:
    """Load the declarative user-facing composition overlay."""

    source = Path(path) if path is not None else VEHICLE_COMPOSITION_REGISTRY
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{source} must contain a mapping")
    registry = VehicleCompositionRegistry.model_validate(payload)
    parameter_identifiers: list[str] = []
    for vehicle in registry.vehicles:
        for initialization_contract in vehicle.initialization_contracts:
            parameter_identifiers.extend(parameter.id for parameter in initialization_contract.parameters)
        for segment_contract in vehicle.segment_contracts:
            parameter_identifiers.extend(parameter.id for parameter in segment_contract.parameters)
    validate_parameter_value_space_coverage(parameter_identifiers)
    return registry
    ####


def resolved_control_realization_for(family_id: str, tier: FidelityTier) -> ControlRealization:
    """Return the profile-specific realization when a tier has an exception.

    Most pseudo-6DOF profiles use a named response law.  The passive tumbling
    profile is deliberately different: it reuses native rigid-body dynamics
    and declares ``uncontrolled``.  Looking through the profile catalog here
    keeps a composition or inspection record from relabeling that exception as
    a generic controller merely because it shares the pseudo tier.
    """

    if tier != "pseudo_6dof":
        return control_realization_for(tier)
    from .trajectory.pseudo6dof_profiles import load_pseudo6dof_catalog

    _, profile = load_pseudo6dof_catalog().for_family(family_id)
    return profile.control_realization
    ####


def load_resolved_vehicle_composition_catalog(
    *,
    registry: VehicleCompositionRegistry | None = None,
    manifests: UnifiedFamilyManifestCatalog | None = None,
) -> ResolvedVehicleCompositionCatalog:
    """Join declared composition to authoritative family and fidelity records."""

    resolved_registry = registry or load_vehicle_composition_registry()
    resolved_manifests = manifests or load_unified_family_manifest_catalog()
    if resolved_manifests.errors:
        manifest_details = "; ".join(f"{item.family_id}: {item.code}" for item in resolved_manifests.errors)
        raise ValueError(f"unified family manifest cannot support composition registry: {manifest_details}")
    manifest_by_id = {item.family_id: item for item in resolved_manifests.families}
    declaration_by_id = {item.family_id: item for item in resolved_registry.vehicles}
    missing_declarations = sorted(set(manifest_by_id) - set(declaration_by_id))
    unknown_declarations = sorted(set(declaration_by_id) - set(manifest_by_id))
    if missing_declarations or unknown_declarations:
        join_details: list[str] = []
        if missing_declarations:
            join_details.append(f"missing composition declarations: {', '.join(missing_declarations)}")
        if unknown_declarations:
            join_details.append(f"unknown composition declarations: {', '.join(unknown_declarations)}")
        raise ValueError("; ".join(join_details))

    for declaration in resolved_registry.vehicles:
        family = manifest_by_id[declaration.family_id]
        declared_tiers = {tier for tier, binding in family.family.tiers.items() if binding.profile_id is not None}
        for initialization in declaration.initialization_contracts:
            unsupported = sorted(set(initialization.compatible_fidelities) - declared_tiers)
            if unsupported:
                raise ValueError(f"{declaration.family_id}/{initialization.id} declares unavailable fidelities: {unsupported}")
        for segment in declaration.segment_contracts:
            unsupported = sorted(set(segment.compatible_fidelities) - declared_tiers)
            if unsupported:
                raise ValueError(f"{declaration.family_id}/{segment.id} declares unavailable fidelities: {unsupported}")
        for mission in declaration.mission_templates:
            unsupported = sorted(set(mission.compatible_fidelities) - declared_tiers)
            if unsupported:
                raise ValueError(f"{declaration.family_id}/{mission.id} declares unavailable fidelities: {unsupported}")

    return ResolvedVehicleCompositionCatalog(tuple(ResolvedVehicleComposition(manifest_by_id[item.family_id], item) for item in resolved_registry.vehicles))
    ####


def build_vehicle_composition_topology_report(
    catalog: ResolvedVehicleCompositionCatalog | None = None,
) -> dict[str, object]:
    """Audit topology declarations across the public Product 3 surface.

    Interface validation alone does not cover registry-owned initialization,
    segment, and variant parameters. This report joins those public inputs
    with every resolved interface and the generic truth-objective schema so a
    caller can fail closed on absent or pending mathematical topology.
    """

    from .interface_channel_value_spaces import interface_channel_value_space_profile
    from .mission_objectives import truth_objective_topology_schema
    from .vehicle_interface import InterfaceChannel, interface_contract_for_composition

    if catalog is None:
        catalog = load_resolved_vehicle_composition_catalog()
    records: list[dict[str, object]] = []
    canonical_channel_coverage: dict[str, list[dict[str, object]]] = {}
    finding_count = 0
    parameter_count = 0
    channel_count = 0
    parameter_contract_totals = {key: 0 for key in _PARAMETER_CONTRACT_MATURITY_KEYS}
    parameter_value_space_catalog_findings: list[str] = []
    interface_channel_value_space_catalog_findings: list[str] = []
    for vehicle in catalog.vehicles:
        initialization_parameters: tuple[CompositionParameter, ...] = tuple(
            parameter for contract in vehicle.declaration.initialization_contracts for parameter in contract.parameters
        )
        segment_parameters: tuple[CompositionParameter, ...] = tuple(
            parameter for contract in vehicle.declaration.segment_contracts for parameter in contract.parameters
        )
        parameter_records: tuple[CompositionParameter | VariantParameterBinding, ...] = (
            *initialization_parameters,
            *segment_parameters,
            *vehicle.declaration.variant_parameters,
        )
        parameter_findings: list[str] = []
        parameter_contract_summary = _parameter_contract_maturity(parameter_records)
        for parameter in parameter_records:
            parameter_count += 1
            if parameter.value_space.topology == "topology_pending":
                parameter_findings.append(f"parameter {parameter.id}: topology_pending")
            if isinstance(parameter, ParameterSpec):
                source = parameter.public_dict()["value_space_source"]
                if source != "parameter_value_space_catalog":
                    parameter_value_space_catalog_findings.append(
                        f"parameter {parameter.id}: expected parameter_value_space_catalog, found {source}"
                    )
            elif parameter.public_dict()["value_space_source"] != "declared_profile":
                parameter_value_space_catalog_findings.append(
                    f"variant {parameter.id}: expected declared value_space_profile"
                )
        for key, count in parameter_contract_summary.items():
            parameter_contract_totals[key] += count
        interface_records: list[dict[str, object]] = []
        all_interface_findings: list[str] = []
        for fidelity in vehicle.family.family.tiers:
            contract = interface_contract_for_composition(vehicle, fidelity)
            interface_findings: list[str] = []
            channels = (
                *contract.parameter_channels,
                *contract.action_channels,
                *contract.effector_channels,
                *contract.status_channels,
                *contract.resource_channels,
                *contract.diagnostic_channels,
            )
            observed_channels: dict[str, InterfaceChannel] = {
                channel.id: channel for channel in (*contract.status_channels, *contract.resource_channels, *contract.diagnostic_channels)
            }
            for channel in channels:
                channel_count += 1
                canonical_channel_coverage.setdefault(channel.id, []).append(
                    {
                        "vehicle_id": vehicle.family.family.vehicle_registry_id or vehicle.family.family_id,
                        "family_id": vehicle.family.family_id,
                        "fidelity": fidelity,
                        "kind": channel.kind,
                        "availability": channel.availability,
                        "provenance": channel.provenance,
                        "sampling": channel.sampling,
                    }
                )
                if channel.value_space is None:
                    interface_findings.append(f"channel {channel.id}: missing value space")
                elif channel.value_space.topology == "topology_pending":
                    interface_findings.append(f"channel {channel.id}: topology_pending")
                if interface_channel_value_space_profile(channel.id) is None:
                    interface_channel_value_space_catalog_findings.append(
                        f"channel {channel.id}: missing interface_channel_value_space_catalog entry"
                    )
            for profile in contract.observation_profiles:
                for channel_id in profile.channel_ids:
                    observed_channel = observed_channels.get(channel_id)
                    if observed_channel is None:
                        interface_findings.append(f"observation {profile.id}: unknown channel {channel_id}")
                    elif observed_channel.value_space is None or observed_channel.value_space.topology == "topology_pending":
                        interface_findings.append(f"observation {profile.id}: channel {channel_id} has no resolved topology")
            for binding in vehicle.declaration.variant_parameters:
                if binding.status != "runnable" or fidelity not in binding.compatible_fidelities:
                    continue
                for channel_id in binding.derived_status_channels:
                    derived_channel = observed_channels.get(channel_id)
                    if derived_channel is None:
                        interface_findings.append(f"variant {binding.id}: declared derived status {channel_id} is absent at {fidelity}")
                    elif derived_channel.availability not in {"available", "available_in_batch"}:
                        interface_findings.append(f"variant {binding.id}: declared derived status {channel_id} is not available at {fidelity}")
            interface_records.append(
                {
                    "fidelity": fidelity,
                    "interface_id": contract.id,
                    "channel_count": len(channels),
                    "observation_profile_count": len(contract.observation_profiles),
                    "status": "pass" if not interface_findings else "fail",
                    "findings": interface_findings,
                }
            )
            all_interface_findings.extend(interface_findings)
        findings = [*parameter_findings, *all_interface_findings]
        finding_count += len(findings)
        records.append(
            {
                "vehicle_id": vehicle.family.family.vehicle_registry_id or vehicle.family.family_id,
                "family_id": vehicle.family.family_id,
                "parameter_count": len(parameter_records),
                "parameter_contract_maturity": parameter_contract_summary,
                "interface_count": len(interface_records),
                "status": "pass" if not findings else "fail",
                "findings": findings,
                "interfaces": interface_records,
            }
        )
    objective_schema = truth_objective_topology_schema()
    objective_findings = _topology_schema_findings(objective_schema)
    objective_catalog = objective_schema.get("channel_value_space_catalog")
    if not isinstance(objective_catalog, Mapping) or objective_catalog.get("status") != "pass":
        objective_findings.append("truth objective channel value-space catalog is not passing")
    finding_count += (
        len(objective_findings)
        + len(parameter_value_space_catalog_findings)
        + len(interface_channel_value_space_catalog_findings)
    )
    canonical_channel_coverage_records = {
        identifier: {
            "channel_kind": entries[0]["kind"],
            "binding_count": len(entries),
            "availability_counts": {
                availability: sum(1 for entry in entries if entry["availability"] == availability)
                for availability in sorted({str(entry["availability"]) for entry in entries})
            },
            "bindings": entries,
        }
        for identifier, entries in sorted(canonical_channel_coverage.items())
    }
    resource_channel_coverage = {
        identifier: record for identifier, record in canonical_channel_coverage_records.items() if record["channel_kind"] == "resource"
    }
    return {
        "schema": "taoryx.vehicle-composition-topology-report/v1alpha1",
        "status": "pass" if finding_count == 0 else "fail",
        "family_count": len(records),
        "parameter_count": parameter_count,
        "parameter_contract_maturity": parameter_contract_totals,
        "interface_channel_count": channel_count,
        "finding_count": finding_count,
        "parameter_value_space_catalog_status": "pass" if not parameter_value_space_catalog_findings else "fail",
        "parameter_value_space_catalog_findings": parameter_value_space_catalog_findings,
        "interface_channel_value_space_catalog_status": "pass" if not interface_channel_value_space_catalog_findings else "fail",
        "interface_channel_value_space_catalog_findings": interface_channel_value_space_catalog_findings,
        "truth_objective_schema": objective_schema,
        "truth_objective_findings": objective_findings,
        "truth_objective_channel_value_space_catalog_status": (
            objective_catalog.get("status") if isinstance(objective_catalog, Mapping) else "fail"
        ),
        "canonical_channel_coverage": canonical_channel_coverage_records,
        "resource_channel_coverage": resource_channel_coverage,
        "vehicles": records,
        "claim_boundary": (
            "This checks mathematical topology declarations on public composition inputs, interfaces, observations, "
            "and truth-objective schema. The parameter-contract inventory records declared data coverage, but does not "
            "establish source provenance quality, executable support, or qualification."
        ),
    }
    ####


def _topology_schema_findings(value: object, path: str = "truth_objective_schema") -> list[str]:
    """Find omitted or pending serialized value-space records in one schema."""

    if isinstance(value, Mapping):
        findings: list[str] = []
        if "value_space" in value:
            specification = value["value_space"]
            if not isinstance(specification, Mapping):
                findings.append(f"{path}: value_space is not a mapping")
            elif specification.get("topology") == "topology_pending":
                findings.append(f"{path}: value_space remains topology_pending")
            elif not isinstance(specification.get("topology"), str):
                findings.append(f"{path}: value_space has no topology")
        for key, item in value.items():
            findings.extend(_topology_schema_findings(item, f"{path}.{key}"))
        return findings
    if isinstance(value, list | tuple):
        return [finding for index, item in enumerate(value) for finding in _topology_schema_findings(item, f"{path}[{index}]")]
    return []
    ####


__all__ = [
    "CompositionParameter",
    "InitializationContract",
    "MissionTemplateContract",
    "ParameterSpec",
    "ResolvedVehicleComposition",
    "ResolvedVehicleCompositionCatalog",
    "build_vehicle_composition_topology_report",
    "SegmentContract",
    "VariantCouplingPolicy",
    "VariantStatusDerivationRelation",
    "VariantParameterBinding",
    "VEHICLE_COMPOSITION_REGISTRY",
    "VehicleCompositionDeclaration",
    "VehicleCompositionRegistry",
    "load_resolved_vehicle_composition_catalog",
    "load_vehicle_composition_registry",
    "mission_graph_execution_contract",
    "mission_semantic_translator_id",
    "resolved_control_realization_for",
]
