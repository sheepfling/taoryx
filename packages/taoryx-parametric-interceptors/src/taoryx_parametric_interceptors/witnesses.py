"""Prototype evidence seeds for three distinct interceptor resolver behaviors."""

from __future__ import annotations

from typing import Any, Final

from .catalogue import interceptor_from_catalogue_record
from .profile import InterceptorEvidenceProfile

_SEED_NOTE = (
    "Catalogue-linked, official-source-backed prototype evidence seed supplied for Taoryx resolver development; "
    "not a verbatim SQL export and not independently measured."
)

AIM9X_BLOCK2_RECORD: Final[dict[str, Any]] = {
    "kind": "interceptor_evidence_record",
    "interceptor_id": "int:us-aim9x-block2",
    "model_id": "aim9x-block2",
    "variant_basis": "AIM-9X Block II",
    "application_context": "NASAMS_surface_launch",
    "source_record_ids": ("int:us-aim9x-block2", "source:navair-aim9x-public-description"),
    "resolution_status": "source_dominant",
    "evidence": {
        "launch_mass": {"value": 84.37, "unit": "kg", "origin": "observed"},
        "length": {"value": 3.02, "unit": "m", "origin": "observed"},
        "body_diameter": {"value": 0.13, "unit": "m", "origin": "observed"},
        "wingspan": {"value": 0.45, "unit": "m", "origin": "observed"},
        "propulsion_architecture": {"value": "single_stage_solid", "origin": "observed"},
        "motor_designation": {"value": "ATK_MK_139", "origin": "observed"},
        "guidance_architecture": {
            "value": "infrared_homing_with_datalink",
            "origin": "observed",
        },
        "control_features": {"value": ["thrust_vectoring"], "origin": "observed"},
        "reported_speed": {"value": None, "status": "classified", "unit": "m/s"},
        "reported_range": {"value": None, "status": "classified", "unit": "m"},
    },
    "derived": {
        "reference_area": {
            "value": 0.01327,
            "unit": "m^2",
            "method": "circular_area_from_body_diameter",
        },
        "slenderness_ratio": {
            "value": 23.23,
            "unit": "1",
            "method": "length_divided_by_body_diameter",
        },
    },
    "resolver_assumptions": {
        "surrogate_archetype_id": "compact_high_agility_sam_v1",
        "unresolved_parameters": (
            "burnout_mass",
            "propellant_mass",
            "thrust_time_profile",
            "drag_coefficient_model",
            "lateral_acceleration_limit",
            "control_bandwidth",
        ),
    },
    "model_notes": _SEED_NOTE,
}

AIM120_C5_C7_RECORD: Final[dict[str, Any]] = {
    "kind": "interceptor_evidence_record",
    "interceptor_id": "int:us-aim120-amraam",
    "model_id": "aim120-amraam-c5-c7",
    "variant_basis": "AIM-120C5_or_C7",
    "application_context": "NASAMS_surface_launch",
    "source_record_ids": ("int:us-aim120-amraam", "source:navair-amraam-public-description"),
    "resolution_status": "source_dominant",
    "evidence": {
        "launch_mass": {
            "value": 161.48,
            "unit": "kg",
            "origin": "observed_converted",
            "source_value": {"value": 356, "unit": "lb"},
        },
        "length": {
            "value": 3.66,
            "unit": "m",
            "origin": "observed_converted",
            "source_value": {"value": 12, "unit": "ft"},
        },
        "body_diameter": {
            "value": 0.1778,
            "unit": "m",
            "origin": "observed_converted",
            "source_value": {"value": 7, "unit": "in"},
        },
        "wingspan": {
            "value": 0.4826,
            "unit": "m",
            "origin": "observed_converted",
            "source_value": {"value": 19, "unit": "in"},
        },
        "propulsion_architecture": {"value": "single_stage_solid", "origin": "observed"},
        "guidance_architecture": {"value": "active_radar", "origin": "observed"},
        "reported_speed": {"value": None, "status": "classified", "unit": "m/s"},
        "reported_range": {"value": None, "status": "classified", "unit": "m"},
    },
    "derived": {
        "reference_area": {
            "value": 0.02483,
            "unit": "m^2",
            "method": "circular_area_from_body_diameter",
        },
        "slenderness_ratio": {
            "value": 20.57,
            "unit": "1",
            "method": "length_divided_by_body_diameter",
        },
    },
    "evidence_intervals": (
        {
            "parameter_id": "launch_mass",
            "minimum_value": 157.85,
            "maximum_value": 162.39,
            "unit": "kg",
            "origin": "observed_variant_interval",
            "note": "Unresolved AMRAAM-family public mass interval; selected C5/C7 witness uses 161.48 kg.",
        },
    ),
    "resolver_assumptions": {
        "surrogate_archetype_id": "medium_range_active_rf_sam_v1",
        "required_diagnostics": ("family_identity_narrowed_to_c5_or_c7",),
        "unresolved_parameters": (
            "burnout_mass",
            "motor_burn_time",
            "thrust_time_profile",
            "drag_model",
            "lift_or_maneuver_authority",
            "control_bandwidth",
            "terminal_guidance_response",
        ),
    },
    "model_notes": _SEED_NOTE,
}

PAC3_MSE_RECORD: Final[dict[str, Any]] = {
    "kind": "interceptor_evidence_record",
    "interceptor_id": "int:us-pac3-mse",
    "model_id": "pac3-mse",
    "variant_basis": "PAC-3_MSE",
    "source_record_ids": ("int:us-pac3-mse", "source:lockheed-martin-pac3-mse-public-description"),
    "resolution_status": "resolved_primarily_from_archetype",
    "evidence": {
        "propulsion_architecture": {
            "value": "dual_pulse_solid",
            "origin": "observed",
            "catalogue_claim_id": "cl:us-mse-propulsion",
        },
        "intercept_mechanism": {"value": "hit_to_kill", "origin": "observed"},
        "control_features": {"value": ["enlarged_control_fins"], "origin": "observed"},
        "launch_mass": {"value": None, "status": "not_resolved_in_current_extract", "unit": "kg"},
        "length": {"value": None, "status": "not_resolved_in_current_extract", "unit": "m"},
        "body_diameter": {"value": None, "status": "not_resolved_in_current_extract", "unit": "m"},
        "reported_range": {
            "value": None,
            "status": "no_scenario_qualified_numeric_observation",
            "unit": "m",
        },
        "reported_altitude": {
            "value": None,
            "status": "no_scenario_qualified_numeric_observation",
            "unit": "m",
        },
    },
    "resolver_assumptions": {
        "surrogate_archetype_id": "high_energy_dual_pulse_interceptor_v1",
        "assumption_case": "nominal",
        "required_diagnostics": (
            "geometry_missing",
            "mass_properties_missing",
            "motor_pulse_timing_missing",
            "aerodynamic_model_missing",
            "control_authority_missing",
        ),
    },
    "model_notes": (
        f"{_SEED_NOTE} Resolver/provenance witness only in the built-in catalogue; "
        "the archetype-dominant case is not registered as a performance-credible runnable exemplar."
    ),
}


def aim9x_block2_profile() -> InterceptorEvidenceProfile:
    """Return the populated compact/high-agility prototype witness."""

    return interceptor_from_catalogue_record(AIM9X_BLOCK2_RECORD)
    ####


def aim120_c5_c7_profile() -> InterceptorEvidenceProfile:
    """Return the variant-qualified medium-range prototype witness."""

    return interceptor_from_catalogue_record(AIM120_C5_C7_RECORD)
    ####


def pac3_mse_profile() -> InterceptorEvidenceProfile:
    """Return the deliberately sparse archetype-dominant prototype witness."""

    return interceptor_from_catalogue_record(PAC3_MSE_RECORD)
    ####


def runnable_prototype_profiles() -> tuple[InterceptorEvidenceProfile, InterceptorEvidenceProfile]:
    """Return only the two prototype witnesses intended for immediate execution."""

    return (aim9x_block2_profile(), aim120_c5_c7_profile())
    ####


__all__ = [
    "AIM120_C5_C7_RECORD",
    "AIM9X_BLOCK2_RECORD",
    "PAC3_MSE_RECORD",
    "aim120_c5_c7_profile",
    "aim9x_block2_profile",
    "pac3_mse_profile",
    "runnable_prototype_profiles",
]
####
