"""Canonical vehicle model registry accessors."""

from __future__ import annotations

import math
from functools import lru_cache
from pathlib import Path
from typing import Any

import yaml

from taoryx.language.table_parser import parse_table_file

from .vehicle_catalog_resources import vehicle_catalog_resource, vehicle_catalog_resources

ROOT = Path(__file__).resolve().parents[2]
STANDARD_GRAVITY_M_S2 = 9.80665


def load_vehicle_registry() -> dict[str, dict[str, Any]]:
    """Load vehicle definitions from the repository source of truth."""

    vehicles: dict[str, dict[str, Any]] = {}
    for source in vehicle_catalog_resources("verification/vehicle_models.yaml"):
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
        records = payload.get("vehicles") if isinstance(payload, dict) else None
        if not isinstance(records, dict):
            raise ValueError(f"{source} must contain a vehicles mapping")
        for key, value in records.items():
            identifier = str(key)
            if identifier in vehicles:
                raise ValueError(f"vehicle registry has duplicate installed definition {identifier!r}")
            if not isinstance(value, dict):
                raise ValueError(f"vehicle registry definition {identifier!r} must be a mapping")
            vehicles[identifier] = value
    return vehicles
####


def vehicle_definition(vehicle_id: str) -> dict[str, Any]:
    """Return one canonical vehicle definition."""

    vehicles = load_vehicle_registry()
    try:
        return vehicles[vehicle_id]
    except KeyError as error:
        raise KeyError(f"unknown vehicle registry id: {vehicle_id}") from error
####


def vehicle_status_line(vehicle_id: str) -> str:
    """Render the physical vehicle status contract for a generated candidate."""

    vehicle = vehicle_definition(vehicle_id)
    values = [
        f"reference-area={vehicle['reference_area_m2']}",
        f"reference-length={vehicle['reference_length_m']}",
        f"dry-mass-kg={vehicle['dry_mass_kg']}",
        f"nominal-mass-kg={vehicle['nominal_mass_kg']}",
    ]
    values.extend(f"inertia-{axis}={vehicle['inertia_kg_m2'][axis]}" for axis in ("x", "y", "z"))
    for key, name in (
        ("min_forward_speed_m_s", "envelope-min-forward-speed"),
        ("max_speed_m_s", "envelope-max-speed"),
        ("max_mach", "envelope-max-mach"),
        ("max_alpha_deg", "envelope-max-alpha-deg"),
        ("max_beta_deg", "envelope-max-beta-deg"),
    ):
        if key in vehicle.get("envelope", {}):
            values.append(f"{name}={vehicle['envelope'][key]}")
    if "aero_alpha_reference_deg" in vehicle:
        values.append(f"aero-alpha-reference-deg={vehicle['aero_alpha_reference_deg']}")
    if "aero_load_mode" in vehicle:
        values.append(f"aero-load-mode={vehicle['aero_load_mode']}")
    if "aero_wrench_frame" in vehicle:
        values.append(f"aero-wrench-frame={vehicle['aero_wrench_frame']}")
    return "*runtime status vehicle " + " ".join(values)
####


def lqr_profile_attributes(profile_id: str) -> dict[str, str]:
    """Resolve one named controller profile into runtime LQR attributes."""

    payload = _load_lqr_scaling_profiles()
    try:
        profile = payload["profiles"][profile_id]
    except KeyError as error:
        raise KeyError(f"unknown LQR profile id: {profile_id}") from error
    weights = dict(profile.get("normalized_weights", {}))
    vehicle_id = str(profile["vehicle"])
    scale_contract = derive_lqr_scale_contract(vehicle_id)
    scale_source = str(profile.get("scale_source", "profile-file"))
    if scale_source == "derived-vehicle-contract":
        state_angle_scale = scale_contract["state_angle_scale_rad"]
        state_rate_scale = scale_contract["state_rate_scale_rad_s"]
        control_moment_scale = scale_contract["control_moment_scale_nm"]
    else:
        state_angle_scale = float(profile["state_angle_scale_rad"])
        state_rate_scale = float(profile["state_rate_scale_rad_s"])
        control_moment_scale = float(profile["control_moment_scale_nm"])
    result = {
        "state-angle-scale-rad": str(state_angle_scale),
        "state-rate-scale-rad-s": str(state_rate_scale),
        "control-moment-scale-nm": str(control_moment_scale),
        "mass-scale-kg": str(scale_contract["mass_scale_kg"]),
        "inertia-scale-kg-m2": str(scale_contract["inertia_scale_kg_m2"]),
        "force-scale-n": str(scale_contract["force_scale_n"]),
        "weight-moment-scale-nm": str(scale_contract["weight_moment_scale_nm"]),
        "inertia-moment-scale-nm": str(scale_contract["inertia_moment_scale_nm"]),
        "mass-scaling": "nominal-ratio",
        "update": "mass",
        "q-angle": str(weights["q_angle"]),
        "q-rate": str(weights["q_rate"]),
        "r-moment": str(weights["r_moment"]),
    }
    return result
####


@lru_cache(maxsize=1)
def _load_lqr_scaling_profiles() -> dict[str, Any]:
    """Merge model-owned controller-scale fragments without hidden fallback."""

    merged: dict[str, Any] | None = None
    profiles: dict[str, Any] = {}
    for source in vehicle_catalog_resources("verification/lqr_scaling_profiles.yaml"):
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
        if not isinstance(payload, dict) or not isinstance(payload.get("profiles"), dict):
            raise ValueError(f"{source} must contain LQR scaling profiles")
        if merged is None:
            merged = dict(payload)
        for identifier, profile in payload["profiles"].items():
            if identifier in profiles:
                raise ValueError(f"LQR scaling profile {identifier!r} is owned by more than one plug-in fragment")
            profiles[str(identifier)] = profile
    if merged is None:
        raise ValueError("no LQR scaling-profile fragment is installed")
    merged["profiles"] = profiles
    return merged
    ####


def __getattr__(name: str) -> object:
    """Resolve historical aggregate path constants only on explicit access."""

    relative_paths = {
        "REGISTRY": "verification/vehicle_models.yaml",
        "LQR_PROFILES": "verification/lqr_scaling_profiles.yaml",
    }
    try:
        relative_path = relative_paths[name]
    except KeyError as error:
        raise AttributeError(name) from error
    from .compatibility.vehicle_catalog_resources import legacy_vehicle_catalog_resource

    value = legacy_vehicle_catalog_resource(relative_path)
    globals()[name] = value
    return value
    ####


def _table_axis_limits(vehicle_id: str) -> dict[str, tuple[float, float]]:
    """Collect source table axis limits for a vehicle, preserving axis names."""

    limits: dict[str, tuple[float, float]] = {}
    for relative_path in vehicle_definition(vehicle_id).get("table_bindings", ()):
        path = vehicle_catalog_resource(str(relative_path))
        if not path.is_file():
            raise FileNotFoundError(
                f"vehicle {vehicle_id!r} table binding is missing: {relative_path}; "
                "fix vehicle_models.yaml before deriving controller scales"
            )
        try:
            document = parse_table_file(path)
        except (OSError, RuntimeError, TypeError, ValueError) as error:
            raise ValueError(f"vehicle {vehicle_id!r} table binding could not be parsed: {relative_path}: {error}") from error
        if not document.tables or not document.executable_complete:
            raise ValueError(
                f"vehicle {vehicle_id!r} table binding is not executable: {relative_path}; "
                "fix table syntax/diagnostics before deriving controller scales"
            )
        for table in document.tables:
            assignments = {assignment.name.casefold(): assignment.values for assignment in table.assignments}
            for axis_name in table.independent_variables:
                values = assignments.get(axis_name.casefold(), [])
                if not values:
                    continue
                lower = min(float(value) for value in values)
                upper = max(float(value) for value in values)
                current = limits.get(axis_name.casefold())
                limits[axis_name.casefold()] = (
                    min(lower, current[0]) if current is not None else lower,
                    max(upper, current[1]) if current is not None else upper,
                )
    return limits
    ####


def derive_lqr_scale_contract(vehicle_id: str) -> dict[str, Any]:
    """Derive dimensionful LQR scales from the vehicle contract and source tables.

    The result is a conditioning contract, not a gain or handling-quality
    claim. Table axes constrain typical attitude-error scales when present;
    actuator authority and inertia constrain the control-moment scale. The
    returned provenance makes fallback choices explicit for sparse source
    decks such as the direct-wrench Hummingbird model.
    """

    vehicle = vehicle_definition(vehicle_id)
    envelope = dict(vehicle.get("envelope", {}))
    table_limits = _table_axis_limits(vehicle_id)
    angle_candidates: list[float] = []
    for axis_name in ("alpha", "beta"):
        if axis_name in table_limits:
            lower, upper = table_limits[axis_name]
            angle_candidates.append(max(abs(lower), abs(upper)))
    for key in ("max_alpha_deg", "max_beta_deg"):
        if key in envelope:
            angle_candidates.append(float(envelope[key]) * 3.141592653589793 / 180.0)
    angle_source = "source-table-axis-or-registry-envelope"
    if angle_candidates:
        angle_scale = min(3.141592653589793 / 18.0, max(3.141592653589793 / 180.0, 0.75 * min(angle_candidates)))
    else:
        angle_scale = 3.141592653589793 / 18.0
        angle_source = "generic-attitude-fallback-no-angle-axis"

    actuator = dict(vehicle.get("actuator", {}))
    maximum_body_rate_deg_s = actuator.get("maximum_body_rate_deg_s")
    rate_axis_candidates = [
        max(abs(lower), abs(upper))
        for axis_name, (lower, upper) in table_limits.items()
        if axis_name.casefold() in {"p", "q", "r", "wx", "wy", "wz", "p_hat", "q_hat", "r_hat"}
    ]
    if maximum_body_rate_deg_s is not None:
        rate_candidates = [float(maximum_body_rate_deg_s) * 3.141592653589793 / 180.0 * 0.10]
        rate_source = "vehicle-actuator-limit"
    elif rate_axis_candidates:
        rate_candidates = [0.5 * min(rate_axis_candidates)]
        rate_source = "source-table-rate-axis"
    else:
        rate_candidates = [0.5]
        rate_source = "generic-rate-fallback-no-rate-axis"
    rate_scale = max(0.05, min(1.0, rate_candidates[0]))

    nominal_mass = float(vehicle["nominal_mass_kg"])
    if not math.isfinite(nominal_mass) or nominal_mass <= 0.0:
        raise ValueError(f"vehicle {vehicle_id!r} nominal_mass_kg must be finite and positive")
    inertia = [float(value) for value in vehicle["inertia_kg_m2"].values()]
    if not all(math.isfinite(value) and value > 0.0 for value in inertia):
        raise ValueError(f"vehicle {vehicle_id!r} inertia_kg_m2 must contain positive finite values")
    inertia_scale = max(inertia)
    force_scale = nominal_mass * STANDARD_GRAVITY_M_S2
    weight_moment_scale = force_scale * float(vehicle["reference_length_m"])
    response_time_s = float(vehicle.get("controller_response_time_s", 0.25))
    inertia_moment_scale = inertia_scale * rate_scale / max(response_time_s, 0.01)
    moment_scale = max(weight_moment_scale, inertia_moment_scale)
    moment_source = "max-mass-weight-and-inertia-rate-response"
    maximum_moment = actuator.get("maximum_moment")
    actuator_limit = float(maximum_moment) if maximum_moment is not None else None

    return {
        "vehicle": vehicle_id,
        "state_angle_scale_rad": angle_scale,
        "state_rate_scale_rad_s": rate_scale,
        "control_moment_scale_nm": moment_scale,
        "mass_scale_kg": nominal_mass,
        "inertia_scale_kg_m2": inertia_scale,
        "force_scale_n": force_scale,
        "weight_moment_scale_nm": weight_moment_scale,
        "inertia_moment_scale_nm": inertia_moment_scale,
        "actuator_moment_limit_nm": actuator_limit,
        "table_axis_limits": table_limits,
        "provenance": {
            "angle": angle_source,
            "rate": rate_source,
            "mass": "vehicle-registry-nominal-mass",
            "inertia": "vehicle-registry-principal-inertia-max",
            "force": "nominal-mass-times-standard-gravity",
            "weight_moment": "force-scale-times-reference-length",
            "inertia_moment": "inertia-scale-times-rate-over-response-time",
            "moment": moment_source,
            "actuator_limit": "vehicle-actuator-limit" if actuator_limit is not None else "not-declared",
        },
    }
    ####
