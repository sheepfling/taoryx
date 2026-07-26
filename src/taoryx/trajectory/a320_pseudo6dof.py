"""Explicit OpenAP plus JSBSim A320 surrogate-composite binding."""

from __future__ import annotations

import bisect
import csv
import io
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from zipfile import ZipFile

from .a320_openap import CORPUS_RELATIVE_PATH, CORPUS_SHA256, A320OpenAPModel, A320OpenAPOperatingPoint, A320OpenAPResult

JSBSIM_PREFIX = "taoryx-aerospace-data-corpus-v1.1/source-corpora/jsbsim-1.3.1/normalized-models/A320/A320/"


@dataclass(frozen=True, slots=True)
class A320Pseudo6DOFOperatingPoint:
    """A point-mass operating point plus JSBSim rotational channels."""

    openap: A320OpenAPOperatingPoint
    alpha_rad: float = 0.0
    beta_rad: float = 0.0
    roll_rate_rad_s: float = 0.0
    pitch_rate_rad_s: float = 0.0
    yaw_rate_rad_s: float = 0.0
    aileron_rad: float = 0.0
    elevator_rad: float = 0.0
    rudder_rad: float = 0.0

    def __post_init__(self) -> None:
        values = (
            self.alpha_rad,
            self.beta_rad,
            self.roll_rate_rad_s,
            self.pitch_rate_rad_s,
            self.yaw_rate_rad_s,
            self.aileron_rad,
            self.elevator_rad,
            self.rudder_rad,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("A320 pseudo-6DOF channels must be finite")
        ####
    ####


@dataclass(frozen=True, slots=True)
class A320Pseudo6DOFResult:
    """Combined performance and rotational result with authority evidence."""

    input: A320Pseudo6DOFOperatingPoint
    performance: A320OpenAPResult
    side_force_coefficient: float
    side_force_n: float
    roll_moment_coefficient: float
    pitch_moment_coefficient: float
    yaw_moment_coefficient: float
    roll_moment_nm: float
    pitch_moment_nm: float
    yaw_moment_nm: float
    package_sha256: str
    qualification_class: str = "surrogate_composite"

    def as_dict(self) -> dict[str, object]:
        """Return the combined output without implying source equivalence."""

        return {
            "performance": self.performance.as_dict(),
            "side_force_coefficient": self.side_force_coefficient,
            "side_force_n": self.side_force_n,
            "roll_moment_coefficient": self.roll_moment_coefficient,
            "pitch_moment_coefficient": self.pitch_moment_coefficient,
            "yaw_moment_coefficient": self.yaw_moment_coefficient,
            "roll_moment_nm": self.roll_moment_nm,
            "pitch_moment_nm": self.pitch_moment_nm,
            "yaw_moment_nm": self.yaw_moment_nm,
            "package_sha256": self.package_sha256,
            "qualification_class": self.qualification_class,
            "authority_map": {
                "aerodynamics.drag": "openap-2.6.0",
                "aerodynamics.normal_force_target": "openap-2.6.0",
                "aerodynamics.side_force": "jsbsim-1.3.1-a320",
                "moments.roll": "jsbsim-1.3.1-a320",
                "moments.pitch": "jsbsim-1.3.1-a320",
                "moments.yaw": "jsbsim-1.3.1-a320",
                "propulsion.thrust": "openap-2.6.0",
                "propulsion.fuel_flow": "openap-2.6.0",
                "mass.scalar_schedule": "openap-2.6.0",
            "mass.cg_and_inertia": "jsbsim-rescaled-estimate",
            "inertia_policy": "linear_mass_rescale_from_jsbsim_empty_mass",
            },
            "disabled_contributions": [
                "jsbsim.aerodynamic_drag",
                "jsbsim.propulsion_thrust",
                "jsbsim.fuel_flow",
            ],
        }
        ####
    ####


class A320Pseudo6DOFModel:
    """Surrogate composite with no duplicated translational forces."""

    def __init__(self, openap: A320OpenAPModel, tables: Mapping[str, "_OneDimensionalTable"], coefficients: Mapping[str, float], quantities: Mapping[str, float]) -> None:
        self.openap = openap
        self.tables = dict(tables)
        self.coefficients = dict(coefficients)
        self.quantities = dict(quantities)
        self.package_sha256 = openap.package_sha256
        self.source_inertia = tuple(quantities[name] for name in ("ixx", "iyy", "izz"))
        self.source_empty_mass_kg = quantities["emptywt"]
        self.reference_area_m2 = quantities["wingarea"]
        self.reference_span_m = quantities["wingspan"]
        self.reference_chord_m = quantities["chord"]
        ####

    @classmethod
    def from_repository(cls, root: str | Path | None = None) -> "A320Pseudo6DOFModel":
        """Load OpenAP and the normalized JSBSim source corpus together."""

        repository = Path(root) if root is not None else Path(__file__).resolve().parents[3]
        openap = A320OpenAPModel.from_repository(repository)
        with ZipFile(repository / CORPUS_RELATIVE_PATH) as archive:
            functions = json.loads(archive.read(JSBSIM_PREFIX + "aerodynamic-expressions.json"))
            tables_payload = archive.read(JSBSIM_PREFIX + "aerodynamic-tables.csv")
            quantities_payload = archive.read(JSBSIM_PREFIX + "quantities.csv")
        tables = _load_moment_tables(tables_payload)
        coefficients = {item["name"]: _expression_value(item["expression"]) for item in functions}
        quantities = {
            row["name"]: float(row["si_value"])
            for row in csv.DictReader(io.StringIO(quantities_payload.decode("utf-8")))
            if row["name"] in {"wingarea", "wingspan", "chord", "ixx", "iyy", "izz", "emptywt"}
        }
        missing = {"wingarea", "wingspan", "chord", "ixx", "iyy", "izz", "emptywt"} - set(quantities)
        if missing:
            raise ValueError("JSBSim A320 quantities are missing: " + ", ".join(sorted(missing)))
        return cls(openap, tables, coefficients, quantities)
        ####

    @property
    def provenance(self) -> dict[str, object]:
        """Return the explicit source authority map for the surrogate."""

        return {
            "model_id": "a320-openap-jsbsim-pseudo6dof",
            "qualification_class": "surrogate_composite",
            "manufacturer_validated": False,
            "source_exact": False,
            "openap_package_sha256": self.package_sha256,
            "corpus_sha256": CORPUS_SHA256,
            "inertia_policy": "linear_mass_rescale_from_jsbsim_empty_mass",
            "authorities": {
                "aerodynamics.drag": "openap-2.6.0",
                "aerodynamics.normal_force_target": "openap-2.6.0",
                "aerodynamics.alpha_mapping": "jsbsim-1.3.1-a320",
                "aerodynamics.side_force": "jsbsim-1.3.1-a320",
                "moments.roll": "jsbsim-1.3.1-a320",
                "moments.pitch": "jsbsim-1.3.1-a320",
                "moments.yaw": "jsbsim-1.3.1-a320",
                "propulsion.thrust": "openap-2.6.0",
                "propulsion.fuel_flow": "openap-2.6.0",
                "mass.scalar_schedule": "openap-2.6.0",
                "mass.cg_and_inertia": "jsbsim-rescaled-estimate",
                "actuators": "taoryx-policy",
                "stability_augmentation": "taoryx-controller",
            },
            "disabled_contributions": ["jsbsim.aerodynamic_drag", "jsbsim.propulsion_thrust", "jsbsim.fuel_flow"],
        }
        ####

    def evaluate(self, point: A320Pseudo6DOFOperatingPoint) -> A320Pseudo6DOFResult:
        """Evaluate OpenAP translational performance plus JSBSim moments."""

        performance = self.openap.evaluate(point.openap)
        beta = point.beta_rad
        flap = point.openap.flap_angle_deg
        side_coefficient = self.tables["aero/coefficient/CYb"].evaluate(beta)
        roll_coefficient = (
            self.tables["aero/coefficient/Clb"].evaluate(beta)
            + self.coefficients["aero/coefficient/Clp"] * point.roll_rate_rad_s * self.reference_span_m / (2.0 * performance.true_airspeed_mps)
            + self.coefficients["aero/coefficient/Clr"] * point.yaw_rate_rad_s * self.reference_span_m / (2.0 * performance.true_airspeed_mps)
            + self.coefficients["aero/coefficient/Clda"] * point.aileron_rad
            + self.coefficients["aero/coefficient/Cldr"] * point.rudder_rad
        )
        pitch_coefficient = (
            self.tables["aero/coefficient/Cmo"].evaluate(flap)
            + self.coefficients["aero/coefficient/Cmalpha"] * point.alpha_rad
            + self.coefficients["aero/coefficient/Cmq"] * point.pitch_rate_rad_s * self.reference_chord_m / (2.0 * performance.true_airspeed_mps)
            + self.coefficients["aero/coefficient/CmDe"] * point.elevator_rad
        )
        yaw_coefficient = (
            self.coefficients["aero/coefficient/Cnb"] * beta
            + self.coefficients["aero/coefficient/Cnr"] * point.yaw_rate_rad_s * self.reference_span_m / (2.0 * performance.true_airspeed_mps)
            + self.coefficients["aero/coefficient/Cndr"] * point.rudder_rad
        )
        qbar = performance.dynamic_pressure_pa
        return A320Pseudo6DOFResult(
            point,
            performance,
            side_coefficient,
            qbar * self.reference_area_m2 * side_coefficient,
            roll_coefficient,
            pitch_coefficient,
            yaw_coefficient,
            qbar * self.reference_area_m2 * self.reference_span_m * roll_coefficient,
            qbar * self.reference_area_m2 * self.reference_chord_m * pitch_coefficient,
            qbar * self.reference_area_m2 * self.reference_span_m * yaw_coefficient,
            self.package_sha256,
        )
        ####

    def rotational_derivatives(self, point: A320Pseudo6DOFOperatingPoint) -> dict[str, float]:
        """Return the source-scaled rotational rates for bounded pulse cases."""

        result = self.evaluate(point)
        inertia = self.inertia_for_mass(point.openap.mass_kg)
        return {
            "roll_rate_rad_s": result.roll_moment_nm / inertia[0],
            "pitch_rate_rad_s": result.pitch_moment_nm / inertia[1],
            "yaw_rate_rad_s": result.yaw_moment_nm / inertia[2],
        }
        ####

    def inertia_for_mass(self, mass_kg: float) -> tuple[float, float, float]:
        """Return the explicitly estimated inertia schedule used by the surrogate."""

        if not math.isfinite(mass_kg) or mass_kg <= 0.0:
            raise ValueError("A320 pseudo-6DOF mass must be positive and finite")
        scale = mass_kg / self.source_empty_mass_kg
        return tuple(value * scale for value in self.source_inertia)
        ####


@dataclass(frozen=True, slots=True)
class _OneDimensionalTable:
    x: tuple[float, ...]
    y: tuple[float, ...]

    def evaluate(self, value: float) -> float:
        if value < self.x[0] or value > self.x[-1]:
            raise ValueError(f"JSBSim A320 table query {value} outside [{self.x[0]}, {self.x[-1]}]")
        right = min(len(self.x) - 1, max(1, bisect.bisect_right(self.x, value)))
        left = right - 1
        if self.x[left] == self.x[right]:
            return self.y[left]
        fraction = (value - self.x[left]) / (self.x[right] - self.x[left])
        return self.y[left] + fraction * (self.y[right] - self.y[left])
        ####
    ####


def _load_moment_tables(payload: bytes) -> dict[str, _OneDimensionalTable]:
    grouped: dict[str, list[tuple[float, float]]] = {}
    selected = {"aero/coefficient/CYb", "aero/coefficient/Clb", "aero/coefficient/Cmo"}
    for row in csv.DictReader(io.StringIO(payload.decode("utf-8"))):
        name = row["function_name"]
        if name not in selected or row["column_value"]:
            continue
        grouped.setdefault(name, []).append((float(row["row_value"]), float(row["value"])))
    result = {name: _OneDimensionalTable(tuple(x for x, _ in sorted(values)), tuple(y for _, y in sorted(values))) for name, values in grouped.items()}
    missing = selected - set(result)
    if missing:
        raise ValueError("JSBSim A320 moment tables are missing: " + ", ".join(sorted(missing)))
    return result
    ####


def _expression_value(expression: Mapping[str, object]) -> float:
    values: list[float] = []

    def visit(node: Mapping[str, object]) -> None:
        if node.get("tag") == "value" and node.get("text") is not None:
            values.append(float(str(node["text"])))
        for child in node.get("children", ()) if isinstance(node.get("children", ()), list) else ():
            if isinstance(child, Mapping):
                visit(child)

    visit(expression)
    return values[-1] if values else 0.0
    ####


__all__ = ["A320Pseudo6DOFModel", "A320Pseudo6DOFOperatingPoint", "A320Pseudo6DOFResult"]
####
