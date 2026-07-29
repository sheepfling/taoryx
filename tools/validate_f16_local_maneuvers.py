"""Run local bank- and pitch-rate reversal maneuvers through F-16 effectors."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from taoryx.trajectory import F16ReferencePhysicalPlant
from tools.validate_f16_physical_wrench_perturbations import _build_case

ROOT = Path(__file__).resolve().parents[1]
STATE_NAMES = ("u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s")


def _rk4_step(
    plant: F16ReferencePhysicalPlant,
    state: dict[str, float],
    effectors: dict[str, float],
    dt_s: float,
) -> dict[str, float]:
    """Integrate the source local body-state derivative with held effectors."""

    def derivative(values: dict[str, float]) -> np.ndarray:
        rates = plant.state_derivative(values, effectors, {})
        return np.asarray([float(rates[name]) for name in STATE_NAMES], dtype=float)

    base = np.asarray([state[name] for name in STATE_NAMES], dtype=float)
    first = derivative(state)
    second_values = base + 0.5 * dt_s * first
    second = derivative(dict(zip(STATE_NAMES, second_values, strict=True)))
    third_values = base + 0.5 * dt_s * second
    third = derivative(dict(zip(STATE_NAMES, third_values, strict=True)))
    fourth_values = base + dt_s * third
    fourth = derivative(dict(zip(STATE_NAMES, fourth_values, strict=True)))
    result = base + dt_s * (first + 2.0 * second + 2.0 * third + fourth) / 6.0
    return {name: float(value) for name, value in zip(STATE_NAMES, result, strict=True)}
    ####


def _reference_for_phase(
    trim: dict[str, float],
    phase: str,
) -> dict[str, float]:
    """Return the declared small local rate target for one maneuver phase."""

    reference = dict(trim)
    if phase == "bank_left":
        reference["p_rad_s"] = 0.015
    elif phase == "bank_reversal":
        reference["p_rad_s"] = -0.015
    elif phase == "pitch_up":
        reference["q_rad_s"] = 0.008
    elif phase == "pitch_reversal":
        reference["q_rad_s"] = -0.008
    return reference
    ####


def _maneuver_pass(phase: str, phase_samples: list[dict[str, Any]], initial_error: float, final_error: float) -> bool:
    """Apply phase-specific transient or settling acceptance semantics."""

    if phase == "bank_left":
        return max(float(sample["state"]["p_rad_s"]) for sample in phase_samples) > 5.0e-5
    if phase == "bank_reversal":
        return min(float(sample["state"]["p_rad_s"]) for sample in phase_samples) < -5.0e-5
    if phase == "pitch_up":
        return max(float(sample["state"]["q_rad_s"]) for sample in phase_samples) > 2.0e-3
    if phase == "pitch_reversal":
        return min(float(sample["state"]["q_rad_s"]) for sample in phase_samples) < -2.0e-3
    return final_error < max(1.0e-3, initial_error * 0.25)
    ####


def main() -> int:
    """Run a deterministic local maneuver sequence and emit evidence."""

    plant, trim, design = _build_case()
    phases = (
        ("trim_hold", 2.0),
        ("bank_left", 2.0),
        ("bank_reversal", 2.0),
        ("pitch_up", 2.0),
        ("pitch_reversal", 2.0),
        ("settle", 2.0),
    )
    dt_s = 0.02
    state = dict(trim.state)
    actual_effectors = dict(trim.controls)
    samples: list[dict[str, Any]] = []
    phase_results: dict[str, dict[str, Any]] = {}
    elapsed = 0.0
    all_passed = True
    for phase, duration_s in phases:
        target = _reference_for_phase(dict(trim.state), phase)
        phase_samples: list[dict[str, Any]] = []
        for _ in range(int(round(duration_s / dt_s))):
            requested_wrench, increment = design.requested_wrench_for_reference(state, target)
            allocation = plant.allocate(state, requested_wrench, actual_effectors, dt_s)
            actual_effectors = dict(allocation.actuator.actual_positions)
            state_error = {name: state[name] - target[name] for name in STATE_NAMES}
            normalized_error = float(
                np.linalg.norm(
                    np.asarray(
                        [state_error[name] / scale for name, scale in zip(STATE_NAMES, design.state_scales, strict=True)],
                        dtype=float,
                    )
                )
            )
            sample = {
                "time_s": elapsed,
                "phase": phase,
                "state": dict(state),
                "reference": dict(target),
                "state_error": state_error,
                "normalized_error": normalized_error,
                "wrench_increment": increment,
                "allocation_status": allocation.allocation.status,
                "achieved_residual_norm": allocation.achieved_controlled_residual_norm,
                "actual_effectors": actual_effectors,
                "effector_rates": dict(allocation.actuator.rates_per_s),
                "saturated_channels": sorted(
                    set(allocation.allocation.position_saturated)
                    | set(allocation.allocation.rate_limited)
                    | set(allocation.actuator.position_saturated)
                    | set(allocation.actuator.rate_limited)
                ),
            }
            samples.append(sample)
            phase_samples.append(sample)
            state = _rk4_step(plant, state, actual_effectors, dt_s)
            elapsed += dt_s
        final = phase_samples[-1]
        initial_error = float(phase_samples[0]["normalized_error"])
        final_error = float(final["normalized_error"])
        phase_passed = (
            _maneuver_pass(phase, phase_samples, initial_error, final_error)
            and all(sample["allocation_status"] == "feasible" for sample in phase_samples)
            and all(not sample["saturated_channels"] for sample in phase_samples)
        )
        all_passed = all_passed and phase_passed
        phase_results[phase] = {
            "start_s": phase_samples[0]["time_s"],
            "end_s": final["time_s"],
            "reference": target,
            "initial_normalized_error": initial_error,
            "final_normalized_error": final_error,
            "maximum_normalized_error": max(float(sample["normalized_error"]) for sample in phase_samples),
            "response_metrics": {
                "maximum_p_rad_s": max(float(sample["state"]["p_rad_s"]) for sample in phase_samples),
                "minimum_p_rad_s": min(float(sample["state"]["p_rad_s"]) for sample in phase_samples),
                "maximum_q_rad_s": max(float(sample["state"]["q_rad_s"]) for sample in phase_samples),
                "minimum_q_rad_s": min(float(sample["state"]["q_rad_s"]) for sample in phase_samples),
            },
            "maximum_achieved_residual_norm": max(float(sample["achieved_residual_norm"]) for sample in phase_samples),
            "allocation_statuses": sorted({sample["allocation_status"] for sample in phase_samples}),
            "saturated_channels": sorted({channel for sample in phase_samples for channel in sample["saturated_channels"]}),
            "passed": phase_passed,
        }
    report = {
        "schema_version": "taoryx.f16-local-maneuver-evidence/v1",
        "status": "development_screen_passed" if all_passed else "development_screen_failed",
        "family_id": "reference_f16_s119",
        "plant_id": "reference-f16-s119-source-runtime-plant",
        "controller_id": design.id,
        "control_path": "state_reference_to_wrench_lqr_to_bounded_effectors_to_source_nonlinear_plant",
        "dt_s": dt_s,
        "phases": phase_results,
        "samples": samples,
        "claim_boundary": (
            "local body-rate command and reversal screen at one fixed operating point; "
            "not a translation/attitude trajectory, scheduled controller, broad envelope, "
            "source-validated actuator, or flight qualification"
        ),
    }
    output = ROOT / "verification/f16_local_maneuver_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if all_passed else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
