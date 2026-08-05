"""Runtime-owned source plant construction for the F-16 S-119 family.

This module owns the exact first operating-point construction used by the
F-16 adapter registry and validation tools.  It is intentionally local: the
catalog's other operating points, gain scheduling, mission translation, and
release qualification remain separate integration gates.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import yaml

from .control_allocation import EffectorLimits
from .trajectory.f16_operating_points import runtime_trim_result, solve_f16_source_trim
from .trajectory.f16_reference import F16ReferencePhysicalPlant, load_f16_reference_plant

ROOT = Path(__file__).resolve().parents[2]
F16_OPERATING_POINT_CATALOG = ROOT / "families/reference_f16_s119/qualification/operating-points.yaml"
F16_SOURCE_SIDECAR = ROOT / "families/reference_f16_s119/plant/daveml-import.json"
F16_ATMOSPHERE = ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml"
F16_ACTUATOR_CONTRACT = ROOT / "families/reference_f16_s119/actuators/reference-first-order-v1.yaml"


def _f16_effectors() -> dict[str, EffectorLimits]:
    """Load the declared local F-16 actuator overlay without inventing limits."""

    payload = yaml.safe_load(F16_ACTUATOR_CONTRACT.read_text(encoding="utf-8"))
    names = {
        "elevator": "elevator_deg",
        "aileron": "aileron_deg",
        "rudder": "rudder_deg",
        "throttle": "throttle_fraction",
    }
    dynamics = payload.get("dynamics", {})
    default_time_constant = float(dynamics.get("time_constant_s", 0.0))
    limits: dict[str, EffectorLimits] = {}
    for source_name, values in payload["limits"].items():
        position = values.get("position")
        if position is None:
            position = [values["lower"], values["upper"]]
        rate = values.get("rate_per_s", values.get("rate_limit_per_s"))
        name = names[source_name]
        limits[name] = EffectorLimits(
            name,
            float(position[0]),
            float(position[1]),
            str(values.get("unit", "fraction" if source_name == "throttle" else "deg")),
            float(rate) if rate is not None else None,
            float(values.get("time_constant_s", default_time_constant)),
        )
    return limits
    ####


@lru_cache(maxsize=1)
def build_f16_source_physical_plant() -> F16ReferencePhysicalPlant:
    """Build the first source-backed F-16 physical operating-point plant."""

    catalog = yaml.safe_load(F16_OPERATING_POINT_CATALOG.read_text(encoding="utf-8"))
    point = catalog["points"][0]
    source = load_f16_reference_plant(F16_SOURCE_SIDECAR, F16_ATMOSPHERE)
    resolved = solve_f16_source_trim(
        source,
        point_id=str(point["id"]),
        altitude_m=float(point["environment"]["geometric_altitude_m"]),
        true_airspeed_m_s=float(point["environment"]["true_airspeed_m_s"]),
        initial_alpha_deg=float(point["state"]["alpha_deg"]),
        initial_elevator_deg=float(point["controls"]["elevator_deg"]),
        initial_throttle_fraction=float(point["controls"]["throttle_fraction"]),
    )
    return F16ReferencePhysicalPlant(
        source,
        runtime_trim_result(resolved),
        resolved.trim_pitch_rad,
        resolved.altitude_m,
        _f16_effectors(),
    )
    ####


__all__ = [
    "F16_ACTUATOR_CONTRACT",
    "F16_ATMOSPHERE",
    "F16_OPERATING_POINT_CATALOG",
    "F16_SOURCE_SIDECAR",
    "build_f16_source_physical_plant",
]
