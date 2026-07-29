"""Generate bounded HL-20 source robustness and timeout-rerun evidence."""

from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path

from taoryx.contracts import Vector3
from taoryx.hl20_reachability import (
    hl20_source_release_commands,
    hl20_source_release_vehicle,
    run_hl20_source_release,
)
from taoryx.reachability_aerodynamics import HL20DavemlAerodynamics
from taoryx.reachability_envelope import ReachabilityEnvelope, ReachabilityFidelity, rerun_timed_out_envelope, run_reachability_envelope
from taoryx.vehicle import DetachedBodyDefinition

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "artifacts/verification/hl20-source-robustness.json"


def _summary(envelope: ReachabilityEnvelope) -> dict[str, object]:
    return {
        "fidelity": envelope.fidelity.value,
        "step_size_s": envelope.step_size_s,
        "horizon_s": envelope.horizon_s,
        "classification_counts": envelope.classification_counts,
        "failure_reason_counts": envelope.failure_reason_counts,
        "timed_out_query_ids": list(envelope.timed_out_query_ids),
        "invalid_query_ids": [sample.query_id for sample in envelope.samples if sample.classification == "invalid"],
        "source_query_count": sum(
            int("source_aerodynamics" in row)
            for sample in envelope.samples
            for row in sample.trajectory.telemetry
            if "source_aerodynamics" in row
        ),
    }


def _boundary_probe() -> dict[str, object]:
    """Exercise source-domain edges without extrapolating any query."""

    import math

    provider = HL20DavemlAerodynamics()
    cases = {
        "mach_min_valid": (0.3, 5.0, 0.0),
        "mach_below_min_invalid": (0.299, 5.0, 0.0),
        "alpha_max_valid": (1.0, 15.0, 0.0),
        "alpha_above_max_invalid": (1.0, 15.1, 0.0),
        "beta_max_valid": (1.0, 5.0, 10.0),
        "beta_above_max_invalid": (1.0, 5.0, 10.1),
    }
    results: dict[str, object] = {}
    for identifier, (mach, alpha_deg, beta_deg) in cases.items():
        speed = mach * provider.speed_of_sound_m_s
        alpha = math.radians(alpha_deg)
        beta = math.radians(beta_deg)
        velocity = (
            speed * math.cos(alpha) * math.cos(beta),
            speed * math.sin(beta),
            -speed * math.sin(alpha) * math.cos(beta),
        )
        try:
            loads = provider.evaluate(velocity, 0.0)
        except ValueError as error:
            results[identifier] = {"status": "rejected", "diagnostic": str(error)}
        else:
            results[identifier] = {
                "status": "accepted",
                "mach": dict(loads.operating_point)["mach"],
                "alpha_deg": math.degrees(math.atan2(-velocity[2], velocity[0])),
            }
    return results


def _spent_booster(mass_kg: float) -> DetachedBodyDefinition:
    scale = mass_kg / 1_500.0
    return DetachedBodyDefinition.cylinder(
        "hl20-synthetic-spent-booster",
        mass_kg=mass_kg,
        radius_m=0.8,
        length_m=6.0,
        inertia_kg_m2=Vector3(5_000.0 * scale, 5_000.0 * scale, 500.0 * scale),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--horizon-s", type=float, default=70.0)
    args = parser.parse_args()
    if args.horizon_s <= 25.0:
        parser.error("--horizon-s must exceed the 25 s release")

    convergence: list[dict[str, object]] = []
    for step_size_s in (1.0, 0.5):
        envelope = run_hl20_source_release(
            fidelity=ReachabilityFidelity.PSEUDO_6DOF,
            step_size_s=step_size_s,
            horizon_s=args.horizon_s,
            spawn_children=True,
        )
        convergence.append(_summary(envelope))

    base_vehicle = hl20_source_release_vehicle()
    inertia = base_vehicle.inertia_body_kg_m2
    assert inertia is not None
    variants = (
        ("nominal", base_vehicle),
        ("glider_mass_minus_5pct", replace(base_vehicle, configuration_variant_id="glider_mass_minus_5pct", dry_mass_kg=base_vehicle.dry_mass_kg * 0.95, inertia_body_kg_m2=inertia.scaled(0.95))),
        ("glider_mass_plus_5pct", replace(base_vehicle, configuration_variant_id="glider_mass_plus_5pct", dry_mass_kg=base_vehicle.dry_mass_kg * 1.05, inertia_body_kg_m2=inertia.scaled(1.05))),
        ("booster_thrust_minus_10pct", replace(base_vehicle, configuration_variant_id="booster_thrust_minus_10pct", booster_thrust_n=base_vehicle.booster_thrust_n * 0.9)),
        ("booster_thrust_plus_10pct", replace(base_vehicle, configuration_variant_id="booster_thrust_plus_10pct", booster_thrust_n=base_vehicle.booster_thrust_n * 1.1)),
        ("booster_burn_minus_10pct", replace(base_vehicle, configuration_variant_id="booster_burn_minus_10pct", booster_burn_time_s=base_vehicle.booster_burn_time_s * 0.9)),
        ("booster_burn_plus_10pct", replace(base_vehicle, configuration_variant_id="booster_burn_plus_10pct", booster_burn_time_s=base_vehicle.booster_burn_time_s * 1.1)),
        ("booster_dry_mass_minus_10pct", replace(base_vehicle, configuration_variant_id="booster_dry_mass_minus_10pct", booster_dry_mass_kg=base_vehicle.booster_dry_mass_kg * 0.9, booster_detached_body=_spent_booster(base_vehicle.booster_dry_mass_kg * 0.9))),
        ("booster_dry_mass_plus_10pct", replace(base_vehicle, configuration_variant_id="booster_dry_mass_plus_10pct", booster_dry_mass_kg=base_vehicle.booster_dry_mass_kg * 1.1, booster_detached_body=_spent_booster(base_vehicle.booster_dry_mass_kg * 1.1))),
        ("separation_impulse_plus_50n_s", replace(base_vehicle, configuration_variant_id="separation_impulse_plus_50n_s", booster_separation_impulse_n_s=Vector3(0.0, 0.0, 50.0))),
        ("crosswind_plus_50m_s", replace(base_vehicle, configuration_variant_id="crosswind_plus_50m_s", wind_velocity_m_s=Vector3(0.0, 50.0, 0.0))),
        ("crosswind_minus_50m_s", replace(base_vehicle, configuration_variant_id="crosswind_minus_50m_s", wind_velocity_m_s=Vector3(0.0, -50.0, 0.0))),
        ("inertia_pitch_plus_10pct", replace(base_vehicle, configuration_variant_id="inertia_pitch_plus_10pct", inertia_body_kg_m2=Vector3(inertia.x, inertia.y * 1.1, inertia.z))),
    )
    variant_reports: list[dict[str, object]] = []
    for identifier, vehicle in variants:
        envelope = run_reachability_envelope(
            vehicle,
            hl20_source_release_commands(ReachabilityFidelity.RIGID_BODY_6DOF),
            fidelity=ReachabilityFidelity.RIGID_BODY_6DOF,
            step_size_s=0.5,
            horizon_s=args.horizon_s,
            spawn_children=True,
            study_id=f"hl20_source_robustness_{identifier}_v1",
            provenance={"robustness_variant": identifier},
        )
        variant_reports.append(
            {
                "variant_id": identifier,
                "vehicle": {
                    "dry_mass_kg": vehicle.dry_mass_kg,
                    "booster_thrust_n": vehicle.booster_thrust_n,
                    "booster_burn_time_s": vehicle.booster_burn_time_s,
                    "booster_dry_mass_kg": vehicle.booster_dry_mass_kg,
                    "booster_propellant_mass_kg": vehicle.booster_propellant_mass_kg,
                    "booster_separation_impulse_n_s": [vehicle.booster_separation_impulse_n_s.x, vehicle.booster_separation_impulse_n_s.y, vehicle.booster_separation_impulse_n_s.z],
                    "wind_velocity_m_s": [vehicle.wind_velocity_m_s.x, vehicle.wind_velocity_m_s.y, vehicle.wind_velocity_m_s.z],
                    "inertia_body_kg_m2": [vehicle.inertia_body_kg_m2.x, vehicle.inertia_body_kg_m2.y, vehicle.inertia_body_kg_m2.z] if vehicle.inertia_body_kg_m2 else None,
                },
                "summary": _summary(envelope),
            }
        )

    short = run_hl20_source_release(
        fidelity=ReachabilityFidelity.POINT_MASS_3DOF,
        step_size_s=0.5,
        horizon_s=30.0,
        spawn_children=True,
    )
    timeout_vehicle = hl20_source_release_vehicle()
    timeout_rerun = rerun_timed_out_envelope(timeout_vehicle, short, horizon_s=args.horizon_s, step_size_s=0.5, workers=1)
    report = {
        "schema": "taoryx.hl20-source-robustness/v1alpha1",
        "status": "bounded_robustness_and_timeout_rerun_complete",
        "claim_boundary": "numerical and classification robustness of the source-bound witness; not a controller, route, or performance claim",
        "convergence": convergence,
        "source_boundary_probes": _boundary_probe(),
        "configuration_variants": variant_reports,
        "timeout_rerun": {
            "parent": _summary(short),
            "rerun": _summary(timeout_rerun),
            "parent_query_ids": list(short.timed_out_query_ids),
            "rerun_study_id": timeout_rerun.study_id,
        },
        "variants": {
            "source_envelope": "Mach 0.3-4.0, alpha 0-15 deg, beta +/-10 deg, altitude -1000-20000 m",
            "booster": "synthetic 1500 kg dry + 1500 kg propellant, 120 kN, 15 s burn, 25 s release",
            "actuator_profile": "hl20.reference_first_order.v1",
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
