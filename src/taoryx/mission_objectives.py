"""Independent truth-telemetry mission objective evaluation.

This module deliberately sits outside controller guidance.  A controller may
request a route transition, but only truth telemetry can satisfy a required
mission objective.  The returned records preserve both facts so a packet can
show where the controller and the physical result disagree.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence, Set
from dataclasses import dataclass, field
from typing import Literal

ObjectiveType = Literal[
    "fly_over",
    "dwell",
    "fly_by_gate",
    "path_corridor",
    "loiter",
    "event",
    "energy_corridor",
    "terminal_state_gate",
    "touchdown",
]
TruthStatus = Literal["pass", "fail", "blocked"]
TransitionReason = Literal[
    "CAPTURED",
    "GATE_CROSSED",
    "DWELL_COMPLETE",
    "EVENT_COMPLETE",
    "TIMEOUT_SKIP",
    "FORCED_ADVANCE",
    "ABORT",
    "NUMERICAL_TERMINATION",
    "UNCLASSIFIED",
]

_CHANNEL_UNITS = {
    "north_m": "m",
    "east_m": "m",
    "altitude_m": "m",
    "speed_m_s": "m/s",
    "horizontal_speed_m_s": "m/s",
    "vertical_speed_m_s": "m/s",
    "local_roll_deg": "deg",
    "local_pitch_deg": "deg",
    "local_heading_deg": "deg",
    "heading_deg": "deg",
    "flight_path_angle_deg": "deg",
}

_OBJECTIVE_TYPES = frozenset(
    {
        "fly_over",
        "dwell",
        "fly_by_gate",
        "path_corridor",
        "loiter",
        "event",
        "energy_corridor",
        "terminal_state_gate",
        "touchdown",
    }
)


@dataclass(frozen=True, slots=True)
class TruthObjectiveSpec:
    """One independently evaluated physical mission objective."""

    id: str
    objective_type: ObjectiveType
    target: Mapping[str, float] = field(default_factory=dict)
    tolerance: Mapping[str, float] = field(default_factory=dict)
    dwell_s: float = 0.0
    required: bool = True
    event_id: str | None = None
    gate_normal: tuple[float, float, float] | None = None
    crossing_direction: int = 1
    window_start_s: float | None = None
    window_end_s: float | None = None

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise ValueError("truth objective id must not be empty")
        if self.objective_type not in _OBJECTIVE_TYPES:
            raise ValueError(f"unsupported truth objective type: {self.objective_type!r}")
        if self.dwell_s < 0.0 or not math.isfinite(self.dwell_s):
            raise ValueError("truth objective dwell_s must be finite and non-negative")
        if set(self.target) - set(self.tolerance) and self.objective_type not in {"event", "fly_by_gate"}:
            raise ValueError(f"truth objective {self.id!r} has target channels without tolerances")
        if any(float(value) <= 0.0 or not math.isfinite(float(value)) for value in self.tolerance.values()):
            raise ValueError(f"truth objective {self.id!r} tolerances must be finite and positive")
        if self.gate_normal is not None:
            norm = math.sqrt(sum(float(value) ** 2 for value in self.gate_normal))
            if not math.isfinite(norm) or norm <= 0.0:
                raise ValueError(f"truth objective {self.id!r} gate_normal must be nonzero")
        if self.crossing_direction not in {-1, 1}:
            raise ValueError(f"truth objective {self.id!r} crossing_direction must be -1 or 1")
        ####


@dataclass(frozen=True, slots=True)
class ControllerTransition:
    """A transition emitted by the controller, kept diagnostic-only."""

    objective_id: str
    time_s: float
    reason: TransitionReason = "UNCLASSIFIED"
    source: str = "controller"


@dataclass(frozen=True, slots=True)
class TruthObjectiveResult:
    """Independent result for one objective, including controller disagreement."""

    id: str
    objective_type: ObjectiveType
    required: bool
    status: TruthStatus
    truth_time_s: float | None
    controller_time_s: float | None
    controller_reason: TransitionReason | None
    controller_transition_valid: bool | None
    closest_error: float | None
    tolerance: float | None
    margin: float | None
    dwell_actual_s: float | None
    critical_metric: Mapping[str, object] | None = None
    message: str | None = None

    def as_dict(self) -> dict[str, object]:
        return {
            "id": self.id,
            "objective_type": self.objective_type,
            "required": self.required,
            "status": self.status,
            "truth_time_s": self.truth_time_s,
            "controller_time_s": self.controller_time_s,
            "controller_reason": self.controller_reason,
            "controller_transition_valid": self.controller_transition_valid,
            "closest_error": self.closest_error,
            "tolerance": self.tolerance,
            "margin": self.margin,
            "dwell_actual_s": self.dwell_actual_s,
            "critical_metric": None if self.critical_metric is None else dict(self.critical_metric),
            "message": self.message,
        }


def _finite(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _channel_units(channel: str) -> str:
    if channel in _CHANNEL_UNITS:
        return _CHANNEL_UNITS[channel]
    if channel.endswith("_deg") or channel.endswith("_degrees"):
        return "deg"
    if channel.endswith("_m_s"):
        return "m/s"
    if channel.endswith("_m"):
        return "m"
    return "model units"


def _metric_number(metric: object, key: str) -> float:
    if not isinstance(metric, Mapping):
        raise ValueError("metric must be a mapping")
    value = _finite(metric.get(key))
    if value is None:
        raise ValueError(f"metric {key!r} must contain a finite number")
    return value


def _metric_snapshot(row: Mapping[str, object], spec: TruthObjectiveSpec) -> tuple[bool, float | None, tuple[dict[str, object], ...]]:
    metrics: list[dict[str, object]] = []
    for channel, target in spec.target.items():
        actual = _finite(row.get(channel))
        limit = float(spec.tolerance[channel]) if channel in spec.tolerance else None
        if actual is None or limit is None:
            return False, None, ()
        error = abs(actual - float(target))
        metrics.append(
            {
                "channel": channel,
                "actual": actual,
                "target": float(target),
                "limit": limit,
                "error": error,
                "margin": limit - error,
                "units": _channel_units(channel),
            }
        )
    normalized = [_metric_number(metric, "error") / _metric_number(metric, "limit") for metric in metrics]
    return max(normalized, default=0.0) <= 1.0, max((_metric_number(metric, "error") for metric in metrics), default=0.0), tuple(metrics)


def _critical_metric(snapshots: Sequence[tuple[dict[str, object], ...]]) -> dict[str, object] | None:
    candidates: list[dict[str, object]] = [metric for snapshot in snapshots for metric in snapshot]
    if not candidates:
        return None
    return dict(min(candidates, key=lambda metric: _metric_number(metric, "margin")))


def _best_attempt_metric(snapshots: Sequence[tuple[dict[str, object], ...]]) -> dict[str, object] | None:
    """Return the limiting channel at the most nearly satisfied sample."""

    attempts: list[dict[str, object]] = [min(snapshot, key=lambda metric: _metric_number(metric, "margin")) for snapshot in snapshots if snapshot]
    return None if not attempts else dict(max(attempts, key=lambda metric: _metric_number(metric, "margin")))


def _window(rows: Sequence[Mapping[str, object]], spec: TruthObjectiveSpec, start_index: int) -> list[Mapping[str, object]]:
    selected: list[Mapping[str, object]] = []
    for row in rows[start_index:]:
        time_s = _finite(row.get("time_s", row.get("time")))
        if time_s is None:
            continue
        if spec.window_start_s is not None and time_s < spec.window_start_s:
            continue
        if spec.window_end_s is not None and time_s > spec.window_end_s:
            continue
        selected.append(row)
    return selected


def _residual(row: Mapping[str, object], spec: TruthObjectiveSpec) -> tuple[bool, float | None, float | None, float | None]:
    if not spec.target:
        return True, 0.0, None, None
    normalized: list[float] = []
    physical: list[float] = []
    margins: list[float] = []
    for channel, target in spec.target.items():
        actual = _finite(row.get(channel))
        tolerance = float(spec.tolerance[channel]) if channel in spec.tolerance else None
        if actual is None or tolerance is None:
            return False, None, None, None
        error = abs(actual - float(target))
        physical.append(error)
        normalized.append(error / tolerance)
        margins.append(tolerance - error)
    return (
        max(normalized, default=0.0) <= 1.0,
        max(physical, default=0.0),
        max(float(value) for value in spec.tolerance.values()) if spec.tolerance else None,
        min(margins, default=None),
    )


def _dwell(rows: Sequence[Mapping[str, object]], flags: Sequence[bool], required_s: float) -> tuple[float, int | None]:
    if not rows:
        return 0.0, None
    if required_s == 0.0:
        first_in_band = next((index for index, in_band in enumerate(flags) if in_band), None)
        return (0.0, first_in_band)
    best = 0.0
    best_index: int | None = None
    start_index: int | None = None
    for index, in_band in enumerate(flags):
        if in_band and start_index is None:
            start_index = index
        if (not in_band or index == len(flags) - 1) and start_index is not None:
            end_index = index if in_band and index == len(flags) - 1 else index - 1
            start_time = _finite(rows[start_index].get("time_s", rows[start_index].get("time")))
            end_time = _finite(rows[end_index].get("time_s", rows[end_index].get("time")))
            duration = max(0.0, (end_time or 0.0) - (start_time or 0.0))
            if duration > best:
                best = duration
                best_index = start_index
            start_index = None
    return best, best_index if best >= required_s else None


def _controller_record(objective_id: str, transitions: Sequence[ControllerTransition]) -> ControllerTransition | None:
    matches = [item for item in transitions if item.objective_id == objective_id]
    return min(matches, key=lambda item: item.time_s) if matches else None


def _result(
    spec: TruthObjectiveSpec,
    *,
    status: TruthStatus,
    truth_time_s: float | None,
    closest_error: float | None,
    tolerance: float | None,
    dwell_actual_s: float | None,
    transition: ControllerTransition | None,
    margin: float | None = None,
    critical_metric: Mapping[str, object] | None = None,
    message: str | None = None,
) -> TruthObjectiveResult:
    transition_valid = None
    if transition is not None and truth_time_s is not None:
        transition_valid = transition.reason not in {"TIMEOUT_SKIP", "FORCED_ADVANCE", "ABORT", "NUMERICAL_TERMINATION"} and transition.time_s + 1.0e-9 >= truth_time_s
    return TruthObjectiveResult(
        id=spec.id,
        objective_type=spec.objective_type,
        required=spec.required,
        status=status,
        truth_time_s=truth_time_s,
        controller_time_s=None if transition is None else transition.time_s,
        controller_reason=None if transition is None else transition.reason,
        controller_transition_valid=transition_valid,
        closest_error=closest_error,
        tolerance=tolerance,
        margin=margin if margin is not None else (None if closest_error is None or tolerance is None else tolerance - closest_error),
        dwell_actual_s=dwell_actual_s,
        critical_metric=critical_metric,
        message=message,
    )


def evaluate_truth_objectives(
    objectives: Sequence[TruthObjectiveSpec],
    telemetry: Sequence[Mapping[str, object]],
    *,
    controller_transitions: Sequence[ControllerTransition] = (),
    truth_events: Set[str] = frozenset(),
    truth_event_times: Mapping[str, float] | None = None,
    hard_gates_passed: bool = True,
) -> dict[str, object]:
    """Evaluate objectives from truth telemetry without trusting the controller."""

    results: list[TruthObjectiveResult] = []
    cursor = 0
    for spec in objectives:
        rows = _window(telemetry, spec, cursor)
        transition = _controller_record(spec.id, controller_transitions)
        if spec.objective_type == "event":
            passed = (spec.event_id or spec.id) in truth_events
            event_time = None if truth_event_times is None else truth_event_times.get(spec.event_id or spec.id)
            result = _result(spec, status="pass" if passed else "fail", truth_time_s=event_time, closest_error=0.0 if passed else None, tolerance=None, dwell_actual_s=None, transition=transition, message=None if passed else "truth event did not occur")
        elif not rows:
            result = _result(spec, status="blocked", truth_time_s=None, closest_error=None, tolerance=max(spec.tolerance.values(), default=None), dwell_actual_s=None, transition=transition, message="no truth telemetry in objective window")
        elif spec.objective_type == "fly_by_gate":
            if spec.gate_normal is None:
                result = _result(spec, status="blocked", truth_time_s=None, closest_error=None, tolerance=None, dwell_actual_s=None, transition=transition, message="fly_by_gate requires gate_normal")
            else:
                result = _evaluate_gate(spec, rows, transition)
        else:
            flags: list[bool] = []
            errors: list[float] = []
            margins: list[float] = []
            snapshots: list[tuple[dict[str, object], ...]] = []
            tolerance = max(spec.tolerance.values(), default=None)
            for row in rows:
                in_band, error, metric_snapshot = _metric_snapshot(row, spec)
                margin = min((_metric_number(metric, "margin") for metric in metric_snapshot), default=None)
                flags.append(in_band)
                snapshots.append(metric_snapshot)
                if error is not None:
                    errors.append(error)
                if margin is not None:
                    margins.append(margin)
            closest_error = min(errors, default=None)
            limiting_margin = min(margins, default=None)
            truth_index: int | None
            critical_metric: dict[str, object] | None = None
            if spec.objective_type in {"path_corridor", "loiter"}:
                passed = bool(flags) and all(flags)
                truth_index = next((index for index, flag in enumerate(flags) if not flag), len(flags) - 1)
                dwell_actual = None
            elif spec.objective_type in {"dwell", "terminal_state_gate", "touchdown"}:
                dwell_actual, dwell_index = _dwell(rows, flags, spec.dwell_s)
                passed = dwell_index is not None
                truth_index = dwell_index if dwell_index is not None else len(flags) - 1
                if dwell_index is not None:
                    dwell_end = dwell_index
                    while dwell_end + 1 < len(flags) and flags[dwell_end + 1]:
                        dwell_end += 1
                    critical_metric = _critical_metric(snapshots[dwell_index : dwell_end + 1])
                    limiting_margin = None if critical_metric is None else _metric_number(critical_metric, "margin")
            else:
                truth_index = next((index for index, flag in enumerate(flags) if flag), None)
                passed = truth_index is not None
                dwell_actual = None
                critical_metric = _critical_metric(snapshots[truth_index : truth_index + 1] if truth_index is not None else snapshots)
                limiting_margin = None if critical_metric is None else _metric_number(critical_metric, "margin")
            if critical_metric is None:
                critical_metric = _best_attempt_metric(snapshots)
                limiting_margin = None if critical_metric is None else _metric_number(critical_metric, "margin")
            truth_time = None if truth_index is None else _finite(rows[truth_index].get("time_s", rows[truth_index].get("time")))
            result = _result(spec, status="pass" if passed else "fail", truth_time_s=truth_time, closest_error=closest_error, tolerance=tolerance, margin=limiting_margin, dwell_actual_s=dwell_actual, transition=transition, critical_metric=critical_metric, message=None if passed else "truth telemetry did not satisfy objective")
        results.append(result)
        if result.status == "pass" and result.truth_time_s is not None:
            cursor = next((index + 1 for index, row in enumerate(telemetry) if _finite(row.get("time_s", row.get("time"))) == result.truth_time_s), cursor)
    required = [item for item in results if item.required]
    objective_pass = bool(required) and all(item.status == "pass" and item.controller_transition_valid is not False for item in required)
    mission_pass = objective_pass and hard_gates_passed
    return {
        "schema_version": 1,
        "evaluator": "independent_truth_telemetry",
        "mission_pass": mission_pass,
        "objective_pass": objective_pass,
        "hard_gates_passed": hard_gates_passed,
        "required_objectives": len(required),
        "required_passed": sum(item.status == "pass" for item in required),
        "results": [item.as_dict() for item in results],
        "claim_boundary": "truth telemetry verifies objective geometry; controller transitions are diagnostic and cannot create a pass",
    }
    ####


def _evaluate_gate(spec: TruthObjectiveSpec, rows: Sequence[Mapping[str, object]], transition: ControllerTransition | None) -> TruthObjectiveResult:
    """Evaluate a plane-crossing gate with lateral, altitude, and speed limits."""

    normal = spec.gate_normal
    if normal is None:
        raise AssertionError("gate normal was validated before gate evaluation")
    norm = math.sqrt(sum(float(value) ** 2 for value in normal))
    origin = tuple(float(spec.target.get(name, 0.0)) for name in ("north_m", "east_m", "altitude_m"))
    previous_signed: float | None = None
    corridor_limit = float(spec.tolerance.get("corridor_m", max((value for name, value in spec.tolerance.items() if name != "altitude_m"), default=0.0)))
    altitude_limit = float(spec.tolerance["altitude_m"]) if "altitude_m" in spec.tolerance and "altitude_m" in spec.target else None
    speed_limit = float(spec.tolerance["speed_m_s"]) if "speed_m_s" in spec.tolerance and "speed_m_s" in spec.target else None
    best_score: float | None = None
    best_time: float | None = None
    best_metric: dict[str, object] | None = None
    crossing_metric: dict[str, object] | None = None
    crossing_time: float | None = None

    def metrics_for(row: Mapping[str, object], delta: tuple[float, float, float], lateral: float) -> tuple[dict[str, object], ...]:
        metrics: list[dict[str, object]] = []
        if corridor_limit > 0.0:
            metrics.append(
                {
                    "channel": "gate_lateral_error",
                    "actual": lateral,
                    "target": 0.0,
                    "limit": corridor_limit,
                    "error": lateral,
                    "margin": corridor_limit - lateral,
                    "units": "m",
                }
            )
        if altitude_limit is not None:
            altitude_error = abs(delta[2])
            metrics.append(
                {
                    "channel": "gate_altitude_error",
                    "actual": altitude_error,
                    "target": 0.0,
                    "limit": altitude_limit,
                    "error": altitude_error,
                    "margin": altitude_limit - altitude_error,
                    "units": "m",
                }
            )
        if speed_limit is not None:
            speed = _finite(row.get("speed_m_s"))
            if speed is None:
                return ()
            speed_error = abs(speed - float(spec.target["speed_m_s"]))
            metrics.append(
                {
                    "channel": "gate_speed_error",
                    "actual": speed_error,
                    "target": 0.0,
                    "limit": speed_limit,
                    "error": speed_error,
                    "margin": speed_limit - speed_error,
                    "units": "m/s",
                }
            )
        return tuple(metrics)

    for row in rows:
        point = tuple(_finite(row.get(name)) for name in ("north_m", "east_m", "altitude_m"))
        time_s = _finite(row.get("time_s", row.get("time")))
        if any(value is None for value in point) or time_s is None:
            continue
        point_values = tuple(float(value) for value in point if value is not None)
        if len(point_values) != 3:
            continue
        delta: tuple[float, float, float] = (
            point_values[0] - origin[0],
            point_values[1] - origin[1],
            point_values[2] - origin[2],
        )
        signed = sum(delta[index] * float(normal[index]) for index in range(3)) / norm
        altitude_component = delta[2] if altitude_limit is not None else 0.0
        lateral = math.sqrt(max(0.0, sum(value * value for value in delta) - signed * signed - altitude_component * altitude_component))
        metrics = metrics_for(row, delta, lateral)
        if not metrics:
            previous_signed = signed
            continue
        normalized = max(
            (_metric_number(metric, "error") / max(_metric_number(metric, "limit"), 1.0e-12) for metric in metrics),
            default=math.inf,
        )
        if best_score is None or normalized < best_score:
            best_score = normalized
            best_time = time_s
            best_metric = _critical_metric((metrics,))
        crossing = previous_signed is not None and previous_signed * signed <= 0.0 and (signed - previous_signed) * spec.crossing_direction >= 0.0
        within_limits = all(_metric_number(metric, "error") <= _metric_number(metric, "limit") for metric in metrics)
        if crossing:
            crossing_metric = _critical_metric((metrics,))
            crossing_time = time_s
        if crossing and within_limits:
            critical = _critical_metric((metrics,))
            if critical is None:
                return _result(spec, status="blocked", truth_time_s=time_s, closest_error=None, tolerance=None, dwell_actual_s=None, transition=transition, message="fly_by_gate has no positive gate metric")
            return _result(
                spec,
                status="pass",
                truth_time_s=time_s,
                closest_error=_metric_number(critical, "actual"),
                tolerance=_metric_number(critical, "limit"),
                dwell_actual_s=None,
                transition=transition,
                margin=_metric_number(critical, "margin"),
                critical_metric=critical,
            )
        previous_signed = signed
    if crossing_metric is not None:
        best_metric = crossing_metric
        best_time = crossing_time
    if best_metric is None:
        return _result(spec, status="blocked", truth_time_s=None, closest_error=None, tolerance=None, dwell_actual_s=None, transition=transition, message="truth telemetry did not contain a finite gate sample")
    return _result(
        spec,
        status="fail",
        truth_time_s=best_time,
        closest_error=_metric_number(best_metric, "actual"),
        tolerance=_metric_number(best_metric, "limit"),
        dwell_actual_s=None,
        transition=transition,
        margin=_metric_number(best_metric, "margin"),
        critical_metric=best_metric,
        message="truth telemetry did not cross the gate within lateral and altitude limits",
    )
