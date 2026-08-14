"""Executable Taoryx binding for the pinned OpenAP A320 product.

This adapter is intentionally a point-mass 3-DOF product.  It loads the
compiled OpenAP runtime artifact from the immutable aerospace corpus, keeps
the source and package hashes attached to every result, and exposes the
generic Taoryx trim, optimization, objective, and linearization contracts.
It does not provide manufacturer-authoritative data or rigid-body moments.
"""

from __future__ import annotations

import bisect
import csv
import hashlib
import io
import itertools
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Literal
from zipfile import ZipFile

import numpy as np
from taoryx_a320.resources import model_resource_root

from taoryx.numeric import DifferenceMode, finite_difference_jacobian
from taoryx.objectives import ObjectiveSpec, score_objectives
from taoryx.runtime.optimization_runtime import OptimizeRuntime
from taoryx.trim import DynamicsLinearization, TrimResult, TrimSpec, solve_trim

CORPUS_RELATIVE_PATH = Path("resources/aerospace/daveml/taoryx-corpus-v1.1/corpus.zip")
PACKAGE_MEMBER = "taoryx-aerospace-data-corpus-v1.1/qualified-models/a320-openap/taoryx-a320-openap-2.6-v0.8.txair"
CORPUS_SHA256 = "9dbb38f23924f2cd2775cb37b1b87476d9ddda672ecab9b84828b253931ebd31"
PACKAGE_SHA256 = "f72662740bd4e7f57c57244fa535e08daeef1cb25f3302c95dc5f8acfcfb4c87"
G0_MPS2 = 9.80665


class A320OpenAPEnvelopeError(ValueError):
    """Raised when a point lies outside the pinned runtime envelope."""


@dataclass(frozen=True, slots=True)
class A320OpenAPOperatingPoint:
    """Inputs to the compiled OpenAP performance binding."""

    altitude_m: float
    mach: float
    mass_kg: float
    flap_angle_deg: float = 0.0
    landing_gear_extended: bool = False
    vertical_speed_mps: float = 0.0
    thrust_mode: Literal["cruise", "climb", "takeoff"] = "cruise"
    throttle_ratio: float | None = None
    temperature_delta_k: float = 0.0

    def __post_init__(self) -> None:
        values = (self.altitude_m, self.mach, self.mass_kg, self.flap_angle_deg, self.vertical_speed_mps, self.temperature_delta_k)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("A320 operating-point values must be finite")
        if self.altitude_m < 0.0 or self.mach < 0.0 or self.mass_kg <= 0.0:
            raise ValueError("A320 altitude and Mach must be nonnegative and mass must be positive")
        if self.flap_angle_deg < 0.0:
            raise ValueError("A320 flap angle must be nonnegative")
        if self.temperature_delta_k != 0.0:
            raise ValueError("the pinned OpenAP A320 binding currently supports only ISA temperature_delta_k=0")
        if self.throttle_ratio is not None and (not math.isfinite(self.throttle_ratio) or not 0.0 <= self.throttle_ratio <= 1.0):
            raise ValueError("A320 throttle ratio must be in [0, 1]")
        if self.thrust_mode not in {"cruise", "climb", "takeoff"}:
            raise ValueError(f"unsupported A320 thrust mode: {self.thrust_mode!r}")
        ####
    ####


@dataclass(frozen=True, slots=True)
class A320OpenAPResult:
    """Named performance outputs with source provenance."""

    input: A320OpenAPOperatingPoint
    true_airspeed_mps: float
    density_kg_m3: float
    dynamic_pressure_pa: float
    lift_coefficient: float
    lift_n: float
    drag_coefficient: float
    drag_n: float
    required_thrust_n: float
    idle_thrust_n: float
    maximum_thrust_n: float
    thrust_n: float
    required_throttle_ratio: float
    fuel_flow_kg_s: float
    fuel_flow_at_throttle_kg_s: float
    thrust_margin_n: float
    feasible: bool
    package_sha256: str
    qualification_class: str = "derived_exact"

    def as_dict(self) -> dict[str, object]:
        """Return a JSON-safe result including the qualification boundary."""

        return {
            "input": {
                "altitude_m": self.input.altitude_m,
                "mach": self.input.mach,
                "mass_kg": self.input.mass_kg,
                "flap_angle_deg": self.input.flap_angle_deg,
                "landing_gear_extended": self.input.landing_gear_extended,
                "vertical_speed_mps": self.input.vertical_speed_mps,
                "thrust_mode": self.input.thrust_mode,
                "throttle_ratio": self.input.throttle_ratio,
            },
            "true_airspeed_mps": self.true_airspeed_mps,
            "density_kg_m3": self.density_kg_m3,
            "dynamic_pressure_pa": self.dynamic_pressure_pa,
            "lift_coefficient": self.lift_coefficient,
            "lift_n": self.lift_n,
            "drag_coefficient": self.drag_coefficient,
            "drag_n": self.drag_n,
            "required_thrust_n": self.required_thrust_n,
            "idle_thrust_n": self.idle_thrust_n,
            "maximum_thrust_n": self.maximum_thrust_n,
            "thrust_n": self.thrust_n,
            "required_throttle_ratio": self.required_throttle_ratio,
            "fuel_flow_kg_s": self.fuel_flow_kg_s,
            "fuel_flow_at_throttle_kg_s": self.fuel_flow_at_throttle_kg_s,
            "thrust_margin_n": self.thrust_margin_n,
            "feasible": self.feasible,
            "package_sha256": self.package_sha256,
            "qualification_class": self.qualification_class,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class _CompiledTable:
    """A deterministic numeric table compiled from a corpus CSV."""

    axes: tuple[str, ...]
    coordinates: Mapping[str, tuple[float | bool, ...]]
    rows: Mapping[tuple[float | bool, ...], Mapping[str, float]]

    def interpolate(self, query: Mapping[str, float | bool]) -> dict[str, float]:
        brackets: list[tuple[tuple[float | bool, ...], tuple[float, ...]]] = []
        for axis in self.axes:
            coordinate = query[axis]
            values = self.coordinates[axis]
            if isinstance(values[0], bool):
                if coordinate not in values:
                    raise A320OpenAPEnvelopeError(f"{axis}={coordinate!r} is outside the categorical envelope")
                brackets.append(((coordinate,), (1.0,)))
                continue
            numeric = float(coordinate)
            numeric_values = tuple(float(value) for value in values)
            if numeric < numeric_values[0] or numeric > numeric_values[-1]:
                raise A320OpenAPEnvelopeError(f"{axis}={numeric} is outside [{numeric_values[0]}, {numeric_values[-1]}]")
            right = bisect.bisect_right(numeric_values, numeric)
            if right == 0:
                right = 1
            if right == len(numeric_values):
                right -= 1
            left = right - 1
            if numeric_values[left] == numeric_values[right]:
                brackets.append(((numeric_values[left],), (1.0,)))
            else:
                fraction = (numeric - numeric_values[left]) / (numeric_values[right] - numeric_values[left])
                brackets.append(((numeric_values[left], numeric_values[right]), (1.0 - fraction, fraction)))
        result: dict[str, float] = {}
        for indices in itertools.product(*(range(len(item[0])) for item in brackets)):
            key = tuple(brackets[index][0][choice] for index, choice in enumerate(indices))
            row = self.rows[key]
            weight = math.prod(brackets[index][1][choice] for index, choice in enumerate(indices))
            for name, value in row.items():
                result[name] = result.get(name, 0.0) + weight * value
        return result
        ####
    ####


class A320OpenAPModel:
    """Pinned OpenAP A320 performance family binding."""

    state_names = ("altitude_m", "mach", "mass_kg", "range_m")
    control_names = ("throttle_ratio", "flight_path_angle_rad")

    def __init__(self, *, runtime: Mapping[str, Any], tables: Mapping[str, _CompiledTable], package_sha256: str) -> None:
        self.runtime = dict(runtime)
        self.tables = dict(tables)
        self.package_sha256 = package_sha256
        self.engine_count = int(runtime["aircraft"]["engine_count"])
        self.maximum_static_thrust_n = float(runtime["engine"]["maximum_static_thrust_n"]) * self.engine_count
        ####

    @classmethod
    def from_repository(cls, root: str | Path | None = None) -> "A320OpenAPModel":
        """Load and hash-verify the committed corpus package."""

        repository = Path(root) if root is not None else model_resource_root()
        corpus_path = repository / CORPUS_RELATIVE_PATH
        corpus_bytes = corpus_path.read_bytes()
        corpus_sha256 = hashlib.sha256(corpus_bytes).hexdigest()
        if corpus_sha256 != CORPUS_SHA256:
            raise ValueError(f"A320 corpus hash mismatch: {corpus_sha256} != {CORPUS_SHA256}")
        with ZipFile(io.BytesIO(corpus_bytes)) as outer:
            package_bytes = outer.read(PACKAGE_MEMBER)
        package_sha256 = hashlib.sha256(package_bytes).hexdigest()
        if package_sha256 != PACKAGE_SHA256:
            raise ValueError(f"A320 package hash mismatch: {package_sha256} != {PACKAGE_SHA256}")
        with ZipFile(io.BytesIO(package_bytes)) as package:
            checksums = package.read("checksums.sha256").decode("utf-8")
            for line in checksums.splitlines():
                if not line.strip():
                    continue
                expected, member = line.split(maxsplit=1)
                member = member.lstrip("*")
                actual = hashlib.sha256(package.read(member)).hexdigest()
                if actual != expected:
                    raise ValueError(f"A320 member hash mismatch for {member}: {actual} != {expected}")
            runtime = json.loads(package.read("runtime/openap-a320.json"))
            index = json.loads(package.read("tables/index.json"))
            tables = {
                name.removeprefix("tables/"): _compile_table(package.read(name), details)
                for name, details in index["tables"].items()
                if details["axes"]
            }
        return cls(runtime=runtime, tables=tables, package_sha256=package_sha256)
        ####

    @property
    def provenance(self) -> dict[str, str]:
        """Return immutable source identity for evidence and exports."""

        return {
            "model_id": "a320-openap-3dof",
            "qualification_class": "derived_exact",
            "upstream": "OpenAP 2.6.0",
            "package_sha256": self.package_sha256,
            "corpus_sha256": CORPUS_SHA256,
            "source_exact": "true",
            "manufacturer_validated": "false",
        }
        ####

    def evaluate(self, point: A320OpenAPOperatingPoint) -> A320OpenAPResult:
        """Evaluate drag, lift, thrust, fuel flow, and level-flight margin."""

        drag_table = self.tables["drag-clean.csv" if point.flap_angle_deg == 0.0 and not point.landing_gear_extended else "drag-configured.csv"]
        drag_query: dict[str, float | bool] = {
            "altitude_m": point.altitude_m,
            "mach": point.mach,
            "mass_kg": point.mass_kg,
        }
        if "flap_angle_deg" in drag_table.axes:
            drag_query.update({"flap_angle_deg": point.flap_angle_deg, "landing_gear_extended": point.landing_gear_extended})
        drag = drag_table.interpolate(drag_query)
        thrust_table = self.tables["thrust-takeoff.csv" if point.thrust_mode == "takeoff" else "thrust-climb.csv"]
        thrust_query: dict[str, float | bool] = {"altitude_m": point.altitude_m, "mach": point.mach}
        if "vertical_speed_mps" in thrust_table.axes:
            thrust_query["vertical_speed_mps"] = point.vertical_speed_mps
        thrust = thrust_table.interpolate(thrust_query)
        if point.thrust_mode == "takeoff":
            maximum_thrust = thrust["maximum_takeoff_thrust_n"]
            idle_thrust = float(self.runtime["engine"]["fuel_flow_idle_kg_s"]) * 0.0
        elif point.thrust_mode == "climb":
            maximum_thrust = thrust["maximum_climb_thrust_n"]
            idle_thrust = thrust["descent_idle_thrust_n"]
        else:
            maximum_thrust = thrust["maximum_cruise_thrust_n"]
            idle_thrust = thrust["descent_idle_thrust_n"]
        required_thrust = drag["drag_n"]
        required_ratio = _clamp((required_thrust - idle_thrust) / max(maximum_thrust - idle_thrust, 1.0e-12), 0.0, 1.0)
        throttle = required_ratio if point.throttle_ratio is None else point.throttle_ratio
        actual_thrust = idle_thrust + throttle * (maximum_thrust - idle_thrust)
        required_fuel_flow = drag.get("required_fuel_flow_kg_s", self._fuel_flow(required_thrust))
        actual_fuel_flow = self._fuel_flow(actual_thrust)
        lift = drag.get("lift_n", drag["lift_coefficient"] * float(self.runtime["aircraft"]["wing_area_m2"]) * drag.get("dynamic_pressure_pa", 0.0))
        dynamic_pressure = drag.get("dynamic_pressure_pa")
        if dynamic_pressure is None or dynamic_pressure == 0.0:
            dynamic_pressure = lift / max(drag["lift_coefficient"] * float(self.runtime["aircraft"]["wing_area_m2"]), 1.0e-12)
        density = drag.get("density_kg_m3")
        if density is None:
            density = 2.0 * dynamic_pressure / max(drag["true_airspeed_mps"] ** 2, 1.0e-12)
        return A320OpenAPResult(
            point,
            drag["true_airspeed_mps"],
            density,
            dynamic_pressure,
            drag["lift_coefficient"],
            lift,
            drag["drag_coefficient"],
            required_thrust,
            required_thrust,
            idle_thrust,
            maximum_thrust,
            actual_thrust,
            required_ratio,
            required_fuel_flow,
            actual_fuel_flow,
            maximum_thrust - required_thrust,
            maximum_thrust >= required_thrust,
            self.package_sha256,
        )
        ####

    def _fuel_flow(self, thrust_n: float) -> float:
        table = self.tables["fuel-flow.csv"]
        ratio = thrust_n / self.maximum_static_thrust_n
        return table.interpolate({"per_engine_static_thrust_ratio": _clamp(ratio, 0.0, 1.2)})["fuel_flow_kg_s"]
        ####

    def trim_level_flight(self, point: A320OpenAPOperatingPoint) -> TrimResult:
        """Solve the bounded level-flight throttle and flight-path trim."""

        baseline = self.evaluate(point)
        spec = TrimSpec(
            state_names=(),
            control_names=self.control_names,
            residual_names=("altitude_rate_mps", "axial_acceleration_mps2"),
            state_initial={},
            control_initial={"throttle_ratio": baseline.required_throttle_ratio, "flight_path_angle_rad": 0.0},
            control_lower={"throttle_ratio": 0.0, "flight_path_angle_rad": -0.2},
            control_upper={"throttle_ratio": 1.0, "flight_path_angle_rad": 0.2},
            residual_scales={"altitude_rate_mps": 1.0, "axial_acceleration_mps2": 1.0},
        )

        def residuals(_state: Mapping[str, float], controls: Mapping[str, float]) -> Mapping[str, float]:
            evaluated = self.evaluate(
                A320OpenAPOperatingPoint(
                    **{**asdict(point), "throttle_ratio": float(controls["throttle_ratio"])}
                )
            )
            gamma = float(controls["flight_path_angle_rad"])
            speed = evaluated.true_airspeed_mps
            acceleration = (evaluated.thrust_n - evaluated.drag_n) / point.mass_kg - G0_MPS2 * math.sin(gamma)
            return {"altitude_rate_mps": speed * math.sin(gamma), "axial_acceleration_mps2": acceleration}

        return solve_trim(spec, residuals, residual_tolerance=1.0e-10, acceptance_tolerance=1.0e-7)
        ####

    def point_mass_derivatives(self, state: Mapping[str, float], controls: Mapping[str, float], *, thrust_mode: str = "cruise") -> dict[str, float]:
        """Return named 3-DOF derivatives for scenario stepping and tuning."""

        point = A320OpenAPOperatingPoint(
            altitude_m=float(state["altitude_m"]),
            mach=float(state["mach"]),
            mass_kg=float(state["mass_kg"]),
            thrust_mode=thrust_mode,  # type: ignore[arg-type]
            throttle_ratio=float(controls["throttle_ratio"]),
        )
        result = self.evaluate(point)
        gamma = float(controls["flight_path_angle_rad"])
        axial_acceleration = (result.thrust_n - result.drag_n) / point.mass_kg - G0_MPS2 * math.sin(gamma)
        speed_of_sound = result.true_airspeed_mps / max(point.mach, 1.0e-8)
        return {
            "altitude_m": result.true_airspeed_mps * math.sin(gamma),
            "mach": axial_acceleration / speed_of_sound,
            "mass_kg": -result.fuel_flow_at_throttle_kg_s,
            "range_m": result.true_airspeed_mps * math.cos(gamma),
        }
        ####

    def linearize_point_mass(self, point: A320OpenAPOperatingPoint, trim: TrimResult | None = None) -> DynamicsLinearization:
        """Return the local named point-mass ``A/B`` matrices."""

        trim_result = trim or self.trim_level_flight(point)
        state = {"altitude_m": point.altitude_m, "mach": point.mach, "mass_kg": point.mass_kg, "range_m": 0.0}
        controls = {name: float(trim_result.controls[name]) for name in self.control_names}
        base = tuple(state[name] for name in self.state_names) + tuple(controls[name] for name in self.control_names)

        def evaluate(vector: tuple[float, ...]) -> Sequence[float]:
            split = len(self.state_names)
            return tuple(
                self.point_mass_derivatives(
                    dict(zip(self.state_names, vector[:split], strict=True)),
                    dict(zip(self.control_names, vector[split:], strict=True)),
                    thrust_mode=point.thrust_mode,
                )[name]
                for name in self.state_names
            )

        jacobian = finite_difference_jacobian(
            evaluate,
            base,
            (1.0, 1.0e-4, 1.0, 1.0, 1.0e-3, 1.0e-3),
            mode=DifferenceMode.CENTRAL,
        )
        state_count = len(self.state_names)
        return DynamicsLinearization(
            self.state_names,
            self.control_names,
            np.asarray([row[:state_count] for row in jacobian.rows], dtype=float),
            np.asarray([row[state_count:] for row in jacobian.rows], dtype=float),
            state,
            controls,
            metadata={**self.provenance, "claim_boundary": "point_mass_3dof; no rigid_body_moments"},
        )
        ####

    def tune_cruise_throttle(self, point: A320OpenAPOperatingPoint) -> Any:
        """Minimize fuel flow subject to the OpenAP thrust-margin constraint."""

        baseline = self.evaluate(point)

        def objective(values: tuple[float, ...]) -> float:
            throttle = float(values[0])
            return self.evaluate(A320OpenAPOperatingPoint(**{**asdict(point), "throttle_ratio": throttle})).fuel_flow_at_throttle_kg_s

        def margin(values: tuple[float, ...]) -> float:
            throttle = float(values[0])
            evaluated = self.evaluate(A320OpenAPOperatingPoint(**{**asdict(point), "throttle_ratio": throttle}))
            return evaluated.thrust_n - evaluated.required_thrust_n

        return OptimizeRuntime(objective, ((0.0, 1.0),), inequality_constraints=(margin,)).run((baseline.required_throttle_ratio,))
        ####

    def score_level_flight_objectives(self, result: A320OpenAPResult | None = None) -> dict[str, object]:
        """Score standard family objectives against one evaluated point."""

        if result is None:
            raise ValueError("an evaluated A320 result is required")
        specs = (ObjectiveSpec("thrust-margin", "performance", "thrust_margin_n", 0.0, 1.0, "N", comparison="minimum"),)
        observed = {"thrust_margin_n": result.thrust_margin_n}
        return score_objectives(specs, observed, scenario_contract_sha256=self.package_sha256)
        ####


def _compile_table(payload: bytes, details: Mapping[str, Any]) -> _CompiledTable:
    axes_payload = details["axes"]
    axes = tuple(str(axis) for axis in axes_payload)
    coordinates: dict[str, tuple[float | bool, ...]] = {}
    for axis in axes:
        raw_values = tuple(axes_payload[axis])
        coordinates[axis] = tuple(bool(value) if isinstance(value, bool) else float(value) for value in raw_values)
    rows: dict[tuple[float | bool, ...], Mapping[str, float]] = {}
    for raw in csv.DictReader(io.StringIO(payload.decode("utf-8"))):
        key = tuple(_parse_axis(raw[axis], coordinates[axis]) for axis in axes)
        values = {name: float(value) for name, value in raw.items() if name not in axes and _is_float(value)}
        rows[key] = values
    if len(rows) != int(details["row_count"]):
        raise ValueError(f"table row count mismatch: expected {details['row_count']}, got {len(rows)}")
    return _CompiledTable(axes, coordinates, rows)
    ####


def _parse_axis(value: str, coordinates: Sequence[float | bool]) -> float | bool:
    if isinstance(coordinates[0], bool):
        return value.strip().lower() == "true"
    return float(value)
    ####


def _is_float(value: str | None) -> bool:
    if value is None:
        return False
    try:
        float(value)
    except ValueError:
        return False
    return True
    ####


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))
    ####


__all__ = [
    "A320OpenAPEnvelopeError",
    "A320OpenAPModel",
    "A320OpenAPOperatingPoint",
    "A320OpenAPResult",
]
####
