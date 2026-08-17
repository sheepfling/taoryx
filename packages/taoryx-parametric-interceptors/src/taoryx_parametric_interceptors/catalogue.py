"""Ergonomic adapter from ontology-shaped evidence records to profiles."""

from __future__ import annotations

import re
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any, Literal, Self

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .profile import (
    AssumptionCase,
    EvidenceConfidence,
    EvidenceGap,
    EvidenceInterval,
    EvidenceSourceValue,
    EvidenceValue,
    InterceptorEvidenceProfile,
    ResolutionStatus,
    ValueOrigin,
    interceptor,
)
from .thrust_curve import AbsoluteThrustCurve, DualPulseThrustProgram
from .thrust_schedule import ThrustProfileSchedule
from .units import canonical_unit_for_interceptor_parameter, canonicalize_interceptor_value

_FIELD_ALIASES = {
    "launch_mass": "launch_mass_kg",
    "burnout_mass": "burnout_mass_kg",
    "specific_impulse": "effective_specific_impulse_s",
    "effective_specific_impulse": "effective_specific_impulse_s",
    "length": "length_m",
    "body_diameter": "body_diameter_m",
    "wingspan": "wingspan_m",
    "reference_area": "reference_area_m2",
    "reference_length": "reference_length_m",
    "burn_time": "burn_time_s",
    "motor_burn_time": "burn_time_s",
    "nominal_thrust": "nominal_thrust_n",
    "mean_thrust": "nominal_thrust_n",
    "average_thrust": "nominal_thrust_n",
    "mean_active_burn_thrust": "nominal_thrust_n",
    "thrust_time_profile": "thrust_profile_schedule",
    "thrust_time_schedule": "thrust_profile_schedule",
    "first_pulse_burn_time": "first_pulse_burn_time_s",
    "inter_pulse_coast_time": "inter_pulse_coast_time_s",
    "second_pulse_burn_time": "second_pulse_burn_time_s",
    "max_thrust_vector_angle": "max_thrust_vector_angle_rad",
    "induced_drag_factor": "maneuver_drag_factor",
    "guidance_architecture": "guidance_family",
    "operating_altitude_min": "applicability_altitude_min_m",
    "operating_altitude_max": "applicability_altitude_max_m",
    "operating_mach_min": "applicability_mach_min",
    "operating_mach_max": "applicability_mach_max",
    "reported_speed": "reported_max_speed_mps",
    "reported_range": "reported_max_range_m",
    "reported_altitude": "reported_max_altitude_m",
}
_RESOLUTION_STATUS_ALIASES = {
    "unassessed": ResolutionStatus.UNASSESSED,
    "resolved_primarily_from_archetype": ResolutionStatus.ARCHETYPE_DOMINANT,
    "exact_variant_required": ResolutionStatus.EXACT_VARIANT_REQUIRED,
    "source_dominant": ResolutionStatus.SOURCE_DOMINANT,
    "mixed": ResolutionStatus.MIXED,
    "archetype_dominant": ResolutionStatus.ARCHETYPE_DOMINANT,
}

CatalogueEvidenceScalar = float | int | str | bool | tuple[str, ...]


class CatalogueEvidenceField(BaseModel):
    """Strict authoring shape for one sourced value or explicit gap."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    value: CatalogueEvidenceScalar | None
    unit: str | None = None
    origin: str | None = None
    status: str | None = None
    source_record_ids: tuple[str, ...] = ()
    catalogue_claim_id: str | None = None
    confidence: EvidenceConfidence = EvidenceConfidence.UNKNOWN
    method: str | None = None
    source_value: EvidenceSourceValue | None = None
    note: str = ""

    @model_validator(mode="after")
    def validate_value_or_gap(self) -> Self:
        if self.value is None:
            if not self.status:
                raise ValueError("null evidence requires a non-empty status")
            contradictory = {
                "origin": self.origin,
                "method": self.method,
                "source_value": self.source_value,
            }
            supplied = tuple(name for name, value in contradictory.items() if value is not None)
            if supplied:
                raise ValueError(f"null evidence cannot also supply {', '.join(supplied)}")
        elif self.status is not None:
            raise ValueError("populated evidence cannot also supply a gap status")
        return self
        ####

    ####


class CatalogueDerivedField(BaseModel):
    """Strict authoring shape for a derived catalogue value."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    value: CatalogueEvidenceScalar
    unit: str | None = None
    method: str = Field(min_length=1)

    ####


class CatalogueEvidenceIntervalInput(BaseModel):
    """Source-unit interval retained without choosing a false point value."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    parameter_id: str | None = None
    parameter: str | None = None
    minimum_value: float
    maximum_value: float
    unit: str = Field(min_length=1)
    origin: str = "observed_variant_interval"
    source_record_ids: tuple[str, ...] = ()
    note: str = ""

    @model_validator(mode="after")
    def validate_parameter_selector(self) -> Self:
        if bool(self.parameter_id) == bool(self.parameter):
            raise ValueError("supply exactly one of parameter_id or parameter")
        return self
        ####

    ####


class CatalogueResolverAssumptions(BaseModel):
    """Strict resolver-only values that must never be mistaken for evidence."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    surrogate_archetype_id: str = "generic_slender_sam_v1"
    assumption_case: AssumptionCase = AssumptionCase.NOMINAL
    required_diagnostics: tuple[str, ...] = ()
    unresolved_parameters: tuple[str, ...] = ()

    ####


class CatalogueInterceptorRecord(BaseModel):
    """Public, machine-readable ontology-to-simulation intake contract."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["interceptor_evidence_record"] | None = None
    interceptor_id: str = Field(min_length=1)
    model_id: str | None = None
    parameter_set_version: str = "0.1.0"
    calibration_reference_model_id: str | None = None
    variant_basis: str | None = None
    application_context: str | None = None
    source_record_ids: tuple[str, ...] = ()
    resolution_status: str = ResolutionStatus.UNASSESSED.value
    evidence: dict[str, CatalogueEvidenceField] = Field(default_factory=dict)
    derived: dict[str, CatalogueDerivedField] = Field(default_factory=dict)
    evidence_intervals: tuple[CatalogueEvidenceIntervalInput, ...] = ()
    thrust_profile_schedule: ThrustProfileSchedule | None = None
    thrust_time_curve: AbsoluteThrustCurve | None = None
    dual_pulse_thrust_program: DualPulseThrustProgram | None = None
    resolver_assumptions: CatalogueResolverAssumptions = Field(default_factory=CatalogueResolverAssumptions)
    model_notes: str = ""

    ####


def interceptor_from_catalogue_record(payload: Mapping[str, Any]) -> InterceptorEvidenceProfile:
    """Normalize one nested catalogue evidence seed into the compact profile API.

    Null evidence becomes an :class:`EvidenceGap`; it never enters the numeric
    parameter map. ``observed_converted`` remains observed evidence while the
    original source value and conversion note remain attached to the field.
    """

    record = CatalogueInterceptorRecord.model_validate(payload)
    normalized = record.model_dump(mode="python")
    catalogue_id = record.interceptor_id
    model_id = record.model_id
    profile_id = model_id or _profile_id(catalogue_id)
    default_sources = record.source_record_ids or (catalogue_id,)
    evidence = normalized["evidence"]
    derived = normalized["derived"]
    assumptions = normalized["resolver_assumptions"]
    thrust_schedule = record.thrust_profile_schedule
    thrust_curve = record.thrust_time_curve
    dual_program = record.dual_pulse_thrust_program

    values: dict[str, object] = {}
    gaps: list[EvidenceGap] = []
    parameter_locations: dict[str, str] = {}
    for source_name, raw in evidence.items():
        name = _field_name(source_name)
        _claim_parameter_location(parameter_locations, name, f"evidence.{source_name}")
        raw_value = raw.get("value")
        sources = _field_sources(raw, default_sources)
        if raw_value is None:
            status = _required_text(raw, "status")
            gaps.append(
                EvidenceGap(
                    parameter_id=name,
                    status=status,
                    unit=_optional_text(raw.get("unit")),
                    source_record_ids=sources,
                    note=_optional_text(raw.get("note")) or "",
                )
            )
            continue
        origin_text = _optional_text(raw.get("origin")) or "observed"
        origin = ValueOrigin.OBSERVED if origin_text == "observed_converted" else ValueOrigin(origin_text)
        value = _evidence_scalar(raw_value, source_name)
        source_value = raw.get("source_value")
        method = _optional_text(raw.get("method"))
        if source_value is not None:
            if not isinstance(source_value, Mapping):
                raise ValueError(f"source_value for {source_name!r} must be a mapping")
            source_value = EvidenceSourceValue.model_validate(source_value)
            method = method or "canonical value converted from retained source_value"
        value, unit, source_value, method = _canonicalized_evidence(
            name,
            value,
            unit=_optional_text(raw.get("unit")),
            source_value=source_value,
            method=method,
        )
        values[name] = EvidenceValue(
            value=value,
            origin=origin,
            unit=unit,
            source_record_ids=sources if origin in {ValueOrigin.OBSERVED, ValueOrigin.REPORTED} else (),
            confidence=EvidenceConfidence(_optional_text(raw.get("confidence")) or "unknown"),
            method=method,
            source_value=source_value,
        )

    for source_name, raw in derived.items():
        name = _field_name(source_name)
        _claim_parameter_location(parameter_locations, name, f"derived.{source_name}")
        method = _required_text(raw, "method")
        value, unit, source_value, conversion_method = _canonicalized_evidence(
            name,
            _evidence_scalar(raw.get("value"), source_name),
            unit=_optional_text(raw.get("unit")),
            source_value=None,
            method=None,
        )
        values[name] = EvidenceValue(
            value=value,
            origin=ValueOrigin.DERIVED,
            unit=unit,
            method=f"{method}; {conversion_method}" if conversion_method else method,
            source_value=source_value,
        )

    unresolved = assumptions.get("unresolved_parameters", ())
    for raw_name in _text_tuple(unresolved):
        name = _field_name(raw_name, require_profile_field=False)
        if any(item.parameter_id == name for item in gaps):
            continue
        gaps.append(EvidenceGap(parameter_id=name, status="resolver_input_missing", source_record_ids=default_sources))

    raw_intervals = normalized["evidence_intervals"]
    intervals = tuple(_interval(item, default_sources) for item in raw_intervals)
    raw_status = record.resolution_status
    try:
        status = _RESOLUTION_STATUS_ALIASES[raw_status]
    except KeyError as error:
        raise ValueError(f"unsupported catalogue resolution_status {raw_status!r}; choose one of {tuple(_RESOLUTION_STATUS_ALIASES)}") from error
    archetype = _optional_text(assumptions.get("surrogate_archetype_id")) or "generic_slender_sam_v1"
    assumption_case = AssumptionCase(_optional_text(assumptions.get("assumption_case")) or "nominal")
    diagnostics = _text_tuple(assumptions.get("required_diagnostics"))
    notes = record.model_notes or ("Prototype catalogue-linked evidence seed supplied for resolver development; not a verbatim catalogue export.")
    return interceptor(
        profile_id,
        parameter_set_version=record.parameter_set_version,
        calibration_reference_model_id=record.calibration_reference_model_id,
        catalogue_interceptor_id=catalogue_id,
        variant_basis=record.variant_basis,
        application_context=record.application_context,
        surrogate_archetype_id=archetype,
        default_assumption_case=assumption_case,
        resolution_status=status,
        required_diagnostics=diagnostics,
        evidence_gaps=tuple(gaps),
        evidence_intervals=intervals,
        source_record_ids=default_sources,
        model_notes=notes,
        thrust_profile_schedule=thrust_schedule,
        thrust_time_curve=thrust_curve,
        dual_pulse_thrust_program=dual_program,
        **values,
    )
    ####


def catalogue_interceptor_record_schema() -> dict[str, Any]:
    """Return the JSON Schema used by catalogue YAML ingestion."""

    schema = CatalogueInterceptorRecord.model_json_schema()
    schema["$id"] = "https://taoryx.dev/schemas/parametric-interceptor-catalogue/v1"
    return schema
    ####


def load_catalogue_interceptor_record(path: str | Path) -> InterceptorEvidenceProfile:
    """Load one nested catalogue evidence record from YAML."""

    source = Path(path)
    try:
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"could not load catalogue interceptor record {source}: {error}") from error
    if not isinstance(payload, Mapping):
        raise ValueError(f"catalogue interceptor record {source} must contain one mapping")
    return interceptor_from_catalogue_record(payload)
    ####


def _field_name(value: object, *, require_profile_field: bool = True) -> str:
    if not isinstance(value, str):
        raise ValueError("evidence field names must be text")
    name = _FIELD_ALIASES.get(value, value)
    if require_profile_field and name not in InterceptorEvidenceProfile.model_fields:
        raise ValueError(f"unsupported interceptor evidence field {value!r}")
    normalized = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")
    if not normalized:
        raise ValueError("evidence field name cannot normalize to empty")
    return normalized
    ####


def _claim_parameter_location(
    locations: dict[str, str],
    parameter_id: str,
    location: str,
) -> None:
    previous = locations.get(parameter_id)
    if previous is not None:
        raise ValueError(f"catalogue parameter {parameter_id!r} is supplied more than once at {previous} and {location}")
    locations[parameter_id] = location
    ####


def _interval(value: object, default_sources: tuple[str, ...]) -> EvidenceInterval:
    if not isinstance(value, Mapping):
        raise ValueError("each evidence interval must be a mapping")
    name = _field_name(value.get("parameter_id") or value.get("parameter"))
    minimum = float(value["minimum_value"])
    maximum = float(value["maximum_value"])
    source_unit = _required_text(value, "unit")
    minimum_conversion = canonicalize_interceptor_value(name, minimum, source_unit)
    maximum_conversion = canonicalize_interceptor_value(name, maximum, source_unit)
    converted = minimum_conversion.converted or maximum_conversion.converted
    note = _optional_text(value.get("note")) or ""
    if converted:
        note = f"{note} " if note else ""
        note += f"Canonical interval converted from [{minimum:g}, {maximum:g}] {source_unit}."
    return EvidenceInterval(
        parameter_id=name,
        minimum_value=minimum_conversion.canonical_value,
        maximum_value=maximum_conversion.canonical_value,
        unit=minimum_conversion.canonical_unit,
        origin=_optional_text(value.get("origin")) or "observed_variant_interval",
        source_record_ids=_text_tuple(value.get("source_record_ids")) or default_sources,
        source_minimum_value=minimum if converted else None,
        source_maximum_value=maximum if converted else None,
        source_unit=source_unit if converted else None,
        note=note,
    )
    ####


def _canonicalized_evidence(
    parameter_id: str,
    value: float | int | str | bool | tuple[str, ...],
    *,
    unit: str | None,
    source_value: EvidenceSourceValue | None,
    method: str | None,
) -> tuple[
    float | int | str | bool | tuple[str, ...],
    str | None,
    EvidenceSourceValue | None,
    str | None,
]:
    canonical_unit = canonical_unit_for_interceptor_parameter(parameter_id)
    if canonical_unit is None or isinstance(value, bool) or not isinstance(value, int | float):
        return value, unit, source_value, method
    conversion = canonicalize_interceptor_value(parameter_id, value, unit)
    if conversion.converted:
        if source_value is not None:
            raise ValueError(
                f"evidence field {parameter_id!r} cannot combine a noncanonical value/unit with source_value; "
                "supply the source scalar once and let the catalogue adapter convert it"
            )
        source_value = EvidenceSourceValue(value=value, unit=unit)
        method = f"{method}; {conversion.method}" if method else conversion.method
    return conversion.canonical_value, conversion.canonical_unit, source_value, method
    ####


def _field_sources(value: Mapping[object, object], default_sources: tuple[str, ...]) -> tuple[str, ...]:
    sources = list(_text_tuple(value.get("source_record_ids")))
    claim_id = value.get("catalogue_claim_id")
    if isinstance(claim_id, str) and claim_id:
        sources.append(claim_id)
    return tuple(dict.fromkeys(sources)) or default_sources
    ####


def _profile_id(catalogue_id: str) -> str:
    value = catalogue_id.split(":", 1)[-1]
    normalized = re.sub(r"[^a-z0-9]+", "-", value.lower()).strip("-")
    if not normalized:
        raise ValueError("catalogue interceptor_id cannot normalize to an empty model ID")
    return normalized
    ####


def _evidence_scalar(value: object, field_name: object) -> float | int | str | bool | tuple[str, ...]:
    if isinstance(value, Sequence) and not isinstance(value, str | bytes):
        if not all(isinstance(item, str) for item in value):
            raise ValueError(f"sequence value for {field_name!r} must contain strings")
        return tuple(value)
    if isinstance(value, float | int | str | bool):
        return value
    raise ValueError(f"unsupported evidence value for {field_name!r}")
    ####


def _text_tuple(value: object) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return (value,)
    if isinstance(value, Sequence) and not isinstance(value, bytes) and all(isinstance(item, str) for item in value):
        return tuple(value)
    raise ValueError("expected text or a sequence of text values")
    ####


def _required_text(value: Mapping[str, Any], name: str) -> str:
    result = value.get(name)
    if not isinstance(result, str) or not result:
        raise ValueError(f"catalogue record requires non-empty {name}")
    return result
    ####


def _optional_text(value: object) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str):
        raise ValueError("expected text")
    return value
    ####


__all__ = [
    "CatalogueInterceptorRecord",
    "catalogue_interceptor_record_schema",
    "interceptor_from_catalogue_record",
    "load_catalogue_interceptor_record",
]
####
