"""Run the standardized generic tuning screen for the four reference aircraft."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import yaml

from taoryx.generic_tuning import (
    GenericLqrProfile,
    GenericLqrReport,
    LinearAuthorityRequirement,
    linear_authority_preflight,
    tune_lqr_profiles,
)
from taoryx.rotorcraft import QuadRotorAllocation
from taoryx.vehicle_registry import derive_lqr_scale_contract, lqr_profile_attributes, vehicle_definition

ROOT = Path(__file__).resolve().parents[1]
BINDINGS = ROOT / "verification/reference_tuning_bindings.yaml"
OUTPUT = ROOT / "verification/generated/reference_aircraft_tuning.json"
FAMILIES = ("b747", "skywalker_x8", "hummingbird", "x15")
####


def _attitude_bridge_clean(vehicle_id: str, controls: tuple[str, ...]) -> tuple[tuple[float, ...], tuple[float, ...], tuple[tuple[float, ...], ...], tuple[tuple[float, ...], ...]]:
    """Return the four typed parts of the direct-wrench bridge."""

    vehicle = vehicle_definition(vehicle_id)
    inertia = vehicle["inertia_kg_m2"]
    a = [[0.0] * 6 for _ in range(6)]
    for index in range(3):
        a[index][index + 3] = 1.0
    b = [[0.0] * len(controls) for _ in range(6)]
    for index in range(min(3, len(controls))):
        b[index + 3][index] = 1.0 / float(inertia["xyz"[index]])
    scales = derive_lqr_scale_contract(vehicle_id)
    return (
        (float(scales["state_angle_scale_rad"]),) * 3 + (float(scales["state_rate_scale_rad_s"]),) * 3,
        (float(scales["control_moment_scale_nm"]),) * len(controls),
        tuple(tuple(row) for row in a),
        tuple(tuple(row) for row in b),
    )
    ####


def _hummingbird_bridge() -> tuple[tuple[float, ...], tuple[float, ...], tuple[tuple[float, ...], ...], tuple[tuple[float, ...], ...]]:
    """Build a local rate bridge from the declared quad-X rotor allocator."""

    vehicle = vehicle_definition("hummingbird")
    inertia = vehicle["inertia_kg_m2"]
    allocation = QuadRotorAllocation(0.17, 5.57e-6, 1.36e-7)
    hover_speed = 469.124102661955
    a = [[0.0] * 6 for _ in range(6)]
    for index in range(3):
        a[index][index + 3] = 1.0
    b = [[0.0] * 4 for _ in range(6)]
    for rotor, ((x, y), direction) in enumerate(zip(allocation.rotor_positions_m, allocation.directions, strict=True)):
        moment_per_speed = (2.0 * hover_speed * y * allocation.thrust_coefficient_n_per_rad_s2, -2.0 * hover_speed * x * allocation.thrust_coefficient_n_per_rad_s2, 2.0 * hover_speed * direction * allocation.reaction_torque_coefficient_nm_per_rad_s2)
        for axis in range(3):
            b[axis + 3][rotor] = moment_per_speed[axis] / float(inertia["xyz"[axis]])
    scales = derive_lqr_scale_contract("hummingbird")
    return (
        (float(scales["state_angle_scale_rad"]),) * 3 + (float(scales["state_rate_scale_rad_s"]),) * 3,
        (hover_speed,) * 4,
        tuple(tuple(row) for row in a),
        tuple(tuple(row) for row in b),
    )
    ####


def _profiles(vehicle_id: str, control_count: int) -> tuple[GenericLqrProfile, ...]:
    """Resolve the common gentle/standard/aggressive profile family."""

    profiles: list[GenericLqrProfile] = []
    for suffix, moment_weight in (("gentle", 2.0), ("standard", 1.0), ("aggressive", 0.25)):
        attributes = lqr_profile_attributes(f"{vehicle_id.replace('_', '-')}-{suffix}")
        profiles.append(GenericLqrProfile(f"{vehicle_id}-{suffix}", (float(attributes["q-angle"]),) * 3 + (float(attributes["q-rate"]),) * 3, (float(attributes["r-moment"]) * moment_weight,) * control_count))
    return tuple(profiles)
    ####


def build(binding_path: Path = BINDINGS) -> dict[str, Any]:
    """Build one standardized generic tuning report for all four families."""

    payload = yaml.safe_load(binding_path.read_text(encoding="utf-8"))
    families: dict[str, Any] = {}
    for vehicle_id in FAMILIES:
        binding = payload["families"][vehicle_id]
        controls: tuple[str, ...]
        if vehicle_id == "hummingbird":
            state_scales, control_scales, a_matrix, b_matrix = _hummingbird_bridge()
            controls = ("rotor-1-speed", "rotor-2-speed", "rotor-3-speed", "rotor-4-speed")
        else:
            controls = ("moment-x", "moment-y", "moment-z")
            state_scales, control_scales, a_matrix, b_matrix = _attitude_bridge_clean(vehicle_id, controls)
        state_names = ("roll-error", "pitch-error", "yaw-error", "p", "q", "r")
        authority_binding = binding["authority_requirement"]
        authority_preflight = linear_authority_preflight(
            LinearAuthorityRequirement(
                str(authority_binding["id"]),
                tuple(str(name) for name in authority_binding["required_state_names"]),
            ),
            state_names=state_names,
            a_matrix=a_matrix,
            b_matrix=b_matrix,
        )
        if authority_preflight.status != "passed":
            raise RuntimeError(
                f"{vehicle_id} direct-wrench screen is structurally uncontrollable: "
                f"{authority_preflight.as_dict()}"
            )
        report: GenericLqrReport = tune_lqr_profiles(
            vehicle_id,
            a_matrix,
            b_matrix,
            state_names=state_names,
            control_names=controls,
            state_scales=state_scales,
            control_scales=control_scales,
            profiles=_profiles(vehicle_id, len(controls)),
            design_source=str(binding["linearization_mode"]),
        )
        families[vehicle_id] = {
            "trim_spec": str(binding["trim_spec"]),
            "linearization_mode": str(binding["linearization_mode"]),
            "control_realization": str(binding["control_realization"]),
            "evidence_tier": str(binding["evidence_tier"]),
            "source_linearization_status": str(binding["source_linearization_status"]),
            "tuning_stage": "generic_lqr_screen",
            "qualification_status": "screen_only",
            "plant_backed_linearization": str(binding["source_linearization_status"]) == "available",
            "gap": str(binding["gap"]),
            "authority_preflight": authority_preflight.as_dict(),
            "report": report.as_dict(),
        }
    return {
        "schema_version": 1,
        "id": str(payload["id"]),
        "claim_boundary": str(payload["claim_boundary"]),
        "families": families,
    }
    ####


def main() -> int:
    """Generate or check the standardized report."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--output", type=Path, default=OUTPUT)
    arguments = parser.parse_args()
    expected = json.dumps(build(), indent=2, sort_keys=True) + "\n"
    if arguments.check:
        if not arguments.output.is_file() or arguments.output.read_text(encoding="utf-8") != expected:
            raise SystemExit(f"stale reference tuning report: {arguments.output}")
    else:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(expected, encoding="utf-8")
        print(arguments.output)
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
