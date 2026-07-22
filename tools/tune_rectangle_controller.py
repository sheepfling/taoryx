"""Search native rectangle-course controller gains and rank safe candidates."""

from __future__ import annotations

import argparse
import json
import random
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True, slots=True)
class TuningCase:
    """One problem-file tuning search definition."""

    identifier: str
    problem: Path
    tables: tuple[Path, ...]
    max_steps: int
    search_max_steps: int | None
    limits: dict[str, float]
    bounds: dict[str, tuple[float, float]]
    baseline: dict[str, float]
    coordinated_turn: bool
    ####


def load_cases(path: Path) -> tuple[TuningCase, ...]:
    """Load validated tuning cases from YAML."""

    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    return tuple(
        TuningCase(
            identifier=str(item["id"]),
            problem=ROOT / str(item["problem"]),
            tables=tuple(ROOT / str(table) for table in item.get("tables", [])),
            max_steps=int(item["max_steps"]),
            search_max_steps=int(item["search_max_steps"]) if item.get("search_max_steps") is not None else None,
            limits={str(key): float(value) for key, value in dict(item["limits"]).items()},
            bounds={str(key): (float(value[0]), float(value[1])) for key, value in dict(item["bounds"]).items()},
            baseline={str(key): float(value) for key, value in dict(item.get("baseline", {})).items()},
            coordinated_turn=bool(item.get("coordinated_turn", True)),
        )
        for item in payload["cases"]
    )
    ####


def inject_controls(source: str, values: dict[str, float], *, coordinated_turn: bool = True) -> str:
    """Replace ordinary runtime guidance attributes in a temporary problem."""

    result = source
    missing: dict[str, float] = {}
    for name, value in values.items():
        pattern = re.compile(rf"(?P<prefix>\b{re.escape(name)}=)[^\s]+", re.IGNORECASE)
        replacement = rf"\g<prefix>{value:.16g}"
        if pattern.search(result):
            result = pattern.sub(replacement, result)
        else:
            missing[name] = value
    if missing:
        guidance = re.compile(r"(?P<prefix>^\s*\*runtime\s+status\s+guidance\b[^\n]*)(?P<newline>\n|$)", re.IGNORECASE | re.MULTILINE)
        match = guidance.search(result)
        if match is None:
            # Route geometry is a controller input too.  Keep the tuner
            # generic by allowing route attributes to be searched without
            # teaching it vehicle-specific problem-file layouts.
            guidance = re.compile(r"(?P<prefix>^\s*\*runtime\s+status\s+route\b[^\n]*)(?P<newline>\n|$)", re.IGNORECASE | re.MULTILINE)
            match = guidance.search(result)
        if match is None:
            raise ValueError("baseline problem has no *runtime status guidance or route line for tuning attributes")
        additions = "".join(f" {name}={value:.16g}" for name, value in missing.items())
        result = result[: match.end("prefix")] + additions + result[match.start("newline") :]
    result = re.sub(
        r"rectangle-coordinated-turn=(?:false|true|0|1|no|yes)",
        f"rectangle-coordinated-turn={'true' if coordinated_turn else 'false'}",
        result,
        flags=re.IGNORECASE,
    )
    return result
    ####


def evaluate_candidate(case: TuningCase, values: dict[str, float], work: Path) -> dict[str, Any]:
    """Run and score one native candidate, failing closed on runtime errors."""

    work.mkdir(parents=True, exist_ok=True)
    candidate_problem = work / f"{case.identifier}-candidate.prb"
    candidate_problem.write_text(
        inject_controls(case.problem.read_text(encoding="utf-8"), values, coordinated_turn=case.coordinated_turn),
        encoding="utf-8",
    )
    report = run_files(
        candidate_problem,
        case.tables,
        output_dir=work / "run",
        max_steps=case.max_steps,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
    result: dict[str, Any] = {
        "controls": values,
        "exit_code": report.exit_code,
        "diagnostics": [item.message for item in report.diagnostics],
        "stop_reason": report.results[0].stop_reason if report.results else "no-result",
    }
    if not report.results or not report.results[0].states:
        result["score"] = 1.0e12
        result["status"] = "failed"
        return result
    history = tuple(next(iter(report.results[0].states.values())))
    initial = history[0].named
    final_state = history[-1]
    final = final_state.named
    max_alpha = max((abs(sample.named.get("aero_alpha_deg", 0.0)) for sample in history if sample.named.get("aero_air_data_valid", 1.0) >= 0.5), default=0.0)
    max_beta = max((abs(sample.named.get("aero_sideslip_deg", 0.0)) for sample in history if sample.named.get("aero_air_data_valid", 1.0) >= 0.5), default=0.0)
    final_range = abs(final.get("range_to_target_m", 1.0e6))
    altitude_error = abs(final.get("altitude_m", 0.0) - initial.get("altitude_m", 0.0))
    speed_delta = abs(final.get("speed_m_s", 0.0) - initial.get("speed_m_s", 0.0))
    corner_capture_errors = tuple(
        min(
            float(sample.named[f"route_corner_{index}_error_m"])
            for sample in history
            if f"route_corner_{index}_error_m" in sample.named
        )
        for index in range(4)
        if any(f"route_corner_{index}_error_m" in sample.named for sample in history)
    )
    duration = final_state.time - history[0].time
    corner_limit = case.limits.get("max_corner_capture_error_m")
    corner_violations = tuple(
        max(0.0, error - corner_limit)
        for error in corner_capture_errors
    ) if corner_limit is not None else ()
    violations = (
        max(0.0, max_alpha - case.limits["max_alpha_deg"]),
        max(0.0, max_beta - case.limits["max_beta_deg"]),
        max(0.0, altitude_error - case.limits["max_altitude_error_m"]),
        max(0.0, speed_delta - case.limits["max_speed_delta_m_s"]),
        *corner_violations,
    )
    complete = report.exit_code == 0 and report.results[0].completed
    result.update(
        {
            "duration_s": duration,
            "final_range_m": final_range,
            "final_altitude_error_m": altitude_error,
            "speed_delta_m_s": speed_delta,
            "max_alpha_deg": max_alpha,
            "max_beta_deg": max_beta,
            "corner_capture_errors_m": corner_capture_errors,
            "violations": violations,
            "score": final_range
            + 2.0 * altitude_error
            + speed_delta
            + sum(corner_capture_errors)
            + 1.0e8 * sum(violations)
            + (1.0e6 if not complete else 0.0),
        }
    )
    safe = complete and not any(violation > 0.0 for violation in violations)
    result["safe"] = safe
    result["status"] = "safe" if safe else "completed_unsafe" if complete else "incomplete"
    return result
    ####


def _selected_case(case: TuningCase, max_steps: int | None) -> TuningCase:
    """Apply an optional development step-limit override."""

    if max_steps is None:
        return case
    return TuningCase(
        identifier=case.identifier,
        problem=case.problem,
        tables=case.tables,
        max_steps=min(case.max_steps, max(1, max_steps)),
        search_max_steps=case.search_max_steps,
        limits=case.limits,
        bounds=case.bounds,
        baseline=case.baseline,
        coordinated_turn=case.coordinated_turn,
    )
    ####


def _random_search(case: TuningCase, evaluations: int, seed: int, output: Path) -> list[dict[str, Any]]:
    """Explore a bounded parameter space with a reproducible random design."""

    rng = random.Random(seed)
    results: list[dict[str, Any]] = []
    candidates: list[dict[str, float]] = []
    if case.baseline:
        candidates.append(case.baseline)
    candidates.extend(
        {name: rng.uniform(lower, upper) for name, (lower, upper) in case.bounds.items()}
        for _ in range(max(0, evaluations - len(candidates)))
    )
    for index, values in enumerate(candidates):
        results.append(evaluate_candidate(case, values, output / f"candidate-{index:03d}"))
    return results
    ####


def _differential_evolution_search(case: TuningCase, evaluations: int, seed: int, output: Path) -> list[dict[str, Any]]:
    """Search bounded gains with SciPy's global differential-evolution solver."""

    from scipy.optimize import differential_evolution

    names = tuple(case.bounds)
    bounds = tuple(case.bounds[name] for name in names)
    # A small population keeps expensive native trajectory evaluations bounded.
    population = max(4, min(12, evaluations // max(1, len(names))))
    maxiter = max(0, evaluations // max(1, population * len(names)) - 1)
    results: list[dict[str, Any]] = []
    candidate_index = 0

    def objective(vector: list[float]) -> float:
        nonlocal candidate_index
        values = {name: float(value) for name, value in zip(names, vector, strict=True)}
        result = evaluate_candidate(case, values, output / f"candidate-{candidate_index:03d}")
        candidate_index += 1
        results.append(result)
        return float(result["score"])

    differential_evolution(
        objective,
        bounds,
        seed=seed,
        maxiter=maxiter,
        popsize=population,
        polish=False,
        updating="immediate",
        workers=1,
    )
    return results
    ####


def main() -> int:
    """Run a deterministic bounded random search and write rankings."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", required=True, help="tuning case id from verification/rectangle_tuning.yaml")
    parser.add_argument("--evaluations", type=int, default=8)
    parser.add_argument("--seed", type=int, default=1995)
    parser.add_argument("--max-steps", type=int, default=None, help="optional development override for each candidate")
    parser.add_argument("--phase", choices=("full", "turn-prefix"), default="full")
    parser.add_argument("--method", choices=("random", "differential-evolution"), default="random")
    parser.add_argument("--output", type=Path, default=Path("artifacts/rectangle-tuning"))
    args = parser.parse_args()
    cases = {case.identifier: case for case in load_cases(ROOT / "verification/rectangle_tuning.yaml")}
    case = cases[args.case]
    args.output.mkdir(parents=True, exist_ok=True)
    requested_steps = args.max_steps
    if requested_steps is None and args.phase == "turn-prefix":
        requested_steps = case.search_max_steps
    selected_case = _selected_case(case, requested_steps)
    search_output = args.output / case.identifier
    if args.method == "differential-evolution":
        results = _differential_evolution_search(selected_case, args.evaluations, args.seed, search_output)
    else:
        results = _random_search(selected_case, args.evaluations, args.seed, search_output)
    results.sort(key=lambda item: float(item["score"]))
    payload = {
        "case": case.identifier,
        "method": args.method,
        "phase": args.phase,
        "seed": args.seed,
        "evaluations": len(results),
        "results": results,
        "best": results[0],
    }
    (args.output / f"{case.identifier}-ranking.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(payload["best"], indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
