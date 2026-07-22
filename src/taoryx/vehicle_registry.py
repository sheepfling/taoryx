"""Canonical vehicle model registry accessors."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[2]
REGISTRY = ROOT / "verification/vehicle_models.yaml"


def load_vehicle_registry() -> dict[str, dict[str, Any]]:
    """Load vehicle definitions from the repository source of truth."""

    payload = yaml.safe_load(REGISTRY.read_text(encoding="utf-8"))
    return {str(key): value for key, value in payload["vehicles"].items()}
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
