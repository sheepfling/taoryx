"""Aerodynamic providers used by the reachability workbench.

The generic reachability solver intentionally keeps a small surrogate model.
Source-backed vehicles opt into one of the providers here so their source
identity and validity envelope remain visible at the simulation boundary.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from functools import lru_cache

from taoryx_reference_models.resources import model_resource_root

from .control_allocation import EffectorEffectiveness
from .hl20_controls import HL20_SOURCE_SURFACE_BOUNDS_DEG
from .trajectory import DAVEMLTrimBinding, load_daveml_trim_binding

Vector = tuple[float, float, float]

HL20_SOURCE_MODEL_ID = "hl20_mod_k_daveml_source_v1"
HL20_SOURCE_SIDECAR = str(model_resource_root() / "families/reference_hl20_mod_k/plant/daveml-import.json")
HL20_SOURCE_PACKAGE_SHA256 = "443e2ed905e310cc57014dc3bad8d3b24b6b5954b8423de9110d405d66154e63"
HL20_SOURCE_DOCUMENT_SHA256 = "b2ec6260ed60d241de250599b269ad35d0b96865b50da5e7f9ef7e04de3844ec"
HL20_SOURCE_SPEED_OF_SOUND_M_S = 340.294
HL20_SOURCE_NUMERICAL_BOUNDARY_TOLERANCE_RAD = 1.0e-2
HL20_SOURCE_ENVELOPE = {
    "mach_min": 0.3,
    "mach_max": 4.0,
    "alpha_min_rad": 0.0,
    "alpha_max_rad": math.radians(15.0),
    "beta_min_rad": math.radians(-10.0),
    "beta_max_rad": math.radians(10.0),
    "altitude_min_m": -1_000.0,
    "altitude_max_m": 20_000.0,
    "dynamic_pressure_min_pa": 0.0,
}
HL20_SOURCE_CONTROL_ENVELOPE = HL20_SOURCE_SURFACE_BOUNDS_DEG
HL20_SOURCE_WRENCH_NAMES = (
    "force_x_n",
    "force_y_n",
    "force_z_n",
    "moment_x_nm",
    "moment_y_nm",
    "moment_z_nm",
)


@dataclass(frozen=True, slots=True)
class ReachabilityAeroLoads:
    """One body-frame aerodynamic evaluation and its source query."""

    force_body_n: Vector
    moment_body_nm: Vector
    coefficients: tuple[tuple[str, float], ...]
    operating_point: tuple[tuple[str, float], ...]
    actuator_trace: dict[str, object] | None = None

    def as_dict(self) -> dict[str, object]:
        """Return JSON-safe load and source-query telemetry."""

        payload: dict[str, object] = {
            "force_body_n": list(self.force_body_n),
            "moment_body_nm": list(self.moment_body_nm),
            "coefficients": dict(self.coefficients),
            "operating_point": dict(self.operating_point),
        }
        if self.actuator_trace is not None:
            payload["actuator"] = self.actuator_trace
        return payload


def build_hl20_source_effectiveness(
    provider: "HL20DavemlAerodynamics",
    body_velocity_m_s: Vector,
    altitude_m: float,
    *,
    body_rates_rad_s: Vector = (0.0, 0.0, 0.0),
    controls: Mapping[str, float] | None = None,
    perturbation_deg: Mapping[str, float] | None = None,
) -> EffectorEffectiveness:
    """Build a local source-load matrix for the generic bounded allocator.

    The matrix is a centered finite-difference derivative of the pinned
    DAVE-ML source loads with respect to the seven declared physical surface
    channels. It is valid only at the supplied operating point; it does not
    replace nonlinear source replay or establish closed-loop qualification.
    """

    baseline_controls = {name: float((controls or {}).get(name, 0.0)) for name in HL20_SOURCE_SURFACE_BOUNDS_DEG}
    baseline = provider.evaluate(body_velocity_m_s, altitude_m, body_rates_rad_s, controls=baseline_controls)
    baseline_wrench = (*baseline.force_body_n, *baseline.moment_body_nm)
    columns: list[tuple[float, ...]] = []
    for name, (lower, upper) in HL20_SOURCE_SURFACE_BOUNDS_DEG.items():
        requested_step = float((perturbation_deg or {}).get(name, 1.0))
        if not math.isfinite(requested_step) or requested_step == 0.0:
            raise ValueError(f"HL-20 source-effectiveness perturbation must be finite and nonzero: {name}")
        magnitude = abs(requested_step)
        plus = min(upper, baseline_controls[name] + magnitude)
        minus = max(lower, baseline_controls[name] - magnitude)
        if plus == minus:
            raise ValueError(f"HL-20 source-effectiveness perturbation is clipped at both bounds: {name}")
        plus_controls = dict(baseline_controls)
        minus_controls = dict(baseline_controls)
        plus_controls[name] = plus
        minus_controls[name] = minus
        plus_loads = provider.evaluate(body_velocity_m_s, altitude_m, body_rates_rad_s, controls=plus_controls)
        minus_loads = provider.evaluate(body_velocity_m_s, altitude_m, body_rates_rad_s, controls=minus_controls)
        plus_wrench = (*plus_loads.force_body_n, *plus_loads.moment_body_nm)
        minus_wrench = (*minus_loads.force_body_n, *minus_loads.moment_body_nm)
        denominator = plus - minus
        columns.append(tuple((high - low) / denominator for high, low in zip(plus_wrench, minus_wrench, strict=True)))
    matrix = tuple(tuple(column[row] for column in columns) for row in range(len(HL20_SOURCE_WRENCH_NAMES)))
    return EffectorEffectiveness(
        wrench_names=HL20_SOURCE_WRENCH_NAMES,
        effector_names=tuple(HL20_SOURCE_SURFACE_BOUNDS_DEG),
        matrix=matrix,
        reference_wrench=dict(zip(HL20_SOURCE_WRENCH_NAMES, baseline_wrench, strict=True)),
        reference_effectors=baseline_controls,
        source=f"{provider.model_id}:centered-source-load-derivative",
    )


def probe_hl20_control_directions(
    *,
    alpha_deg: float = 5.0,
    mach: float = 1.0,
    altitude_m: float = 0.0,
    perturbation_deg: float = 10.0,
) -> dict[str, object]:
    """Measure each source surface's coefficient delta from a common anchor.

    This is deliberately an open-loop source probe.  It does not claim that a
    controller can track the resulting moment, only that the source graph
    responds to the declared actuator channels with a reproducible sign and
    magnitude.
    """

    if not math.isfinite(alpha_deg) or not math.isfinite(mach) or not math.isfinite(altitude_m):
        raise ValueError("HL-20 control probe operating point must be finite")
    alpha_rad = math.radians(alpha_deg)
    speed = mach * HL20_SOURCE_SPEED_OF_SOUND_M_S
    velocity = (speed * math.cos(alpha_rad), 0.0, -speed * math.sin(alpha_rad))
    provider = HL20DavemlAerodynamics()
    baseline = provider.evaluate(velocity, altitude_m)
    baseline_coefficients = dict(baseline.coefficients)
    responses: dict[str, object] = {}
    for name, (lower, upper) in HL20_SOURCE_CONTROL_ENVELOPE.items():
        value = min(upper, max(lower, perturbation_deg))
        if abs(value) <= 1.0e-12:
            value = lower if abs(lower) > 1.0e-12 else upper
        loads = provider.evaluate(velocity, altitude_m, controls={name: value})
        coefficients = dict(loads.coefficients)
        delta = {channel: coefficients[channel] - baseline_coefficients[channel] for channel in baseline_coefficients}
        responses[name] = {
            "command_deg": value,
            "delta_coefficients": delta,
            "nonzero_channels": [channel for channel, change in delta.items() if abs(change) > 1.0e-12],
        }
    return {
        "operating_point": {
            "alpha_deg": alpha_deg,
            "mach": mach,
            "altitude_m": altitude_m,
            "perturbation_policy": "positive_requested_then_clamped_to_declared_source_bound",
        },
        "baseline_coefficients": baseline_coefficients,
        "responses": responses,
        "provenance": provider.provenance,
    }


def _finite_vector(values: Vector, label: str) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} must contain only finite values")


def _norm(values: Vector) -> float:
    return math.sqrt(sum(value * value for value in values))


def _sub(left: Vector, right: Vector) -> Vector:
    return tuple(a - b for a, b in zip(left, right, strict=True))  # type: ignore[return-value]


def _scale(values: Vector, factor: float) -> Vector:
    return tuple(factor * value for value in values)  # type: ignore[return-value]


def _dot(left: Vector, right: Vector) -> float:
    return sum(a * b for a, b in zip(left, right, strict=True))


def _unit(values: Vector) -> Vector:
    length = _norm(values)
    if length <= 1.0e-12:
        raise ValueError("source aerodynamic evaluation requires positive airspeed")
    return _scale(values, 1.0 / length)


def _projected_up(velocity_unit: Vector) -> Vector:
    up = (0.0, 0.0, -1.0)
    projected = _sub(up, _scale(velocity_unit, _dot(up, velocity_unit)))
    return _unit(projected)


def _projected_right(velocity_unit: Vector) -> Vector:
    right = (0.0, 1.0, 0.0)
    projected = _sub(right, _scale(velocity_unit, _dot(right, velocity_unit)))
    return _unit(projected)


def _source_controls(controls: Mapping[str, float]) -> dict[str, float]:
    names = {
        "upper_left_body_flap": "DBFUL",
        "upper_right_body_flap": "DBFUR",
        "lower_left_body_flap": "DBFLL",
        "lower_right_body_flap": "DBFLR",
        "left_wing_flap": "DWFL",
        "right_wing_flap": "DWFR",
        "rudder": "DRUD",
    }
    return {public: float(controls.get(public, controls.get(source, 0.0))) for public, source in names.items()}


@lru_cache(maxsize=4)
def _load_hl20_binding(sidecar: str) -> DAVEMLTrimBinding:
    return load_daveml_trim_binding(
        sidecar,
        role="aerodynamics",
        state_inputs={
            "alpha_deg": "ALP_UNLIM",
            "beta_deg": "BETA",
            "mach": "XMACH",
            "roll_rate_rad_s": "PB",
            "pitch_rate_rad_s": "QB",
            "yaw_rate_rad_s": "RB",
            "true_airspeed_f_s": "VRW",
            "height_ft": "H_rwy",
        },
        control_inputs={
            "upper_left_body_flap": "DBFUL",
            "upper_right_body_flap": "DBFUR",
            "lower_left_body_flap": "DBFLL",
            "lower_right_body_flap": "DBFLR",
            "left_wing_flap": "DWFL",
            "right_wing_flap": "DWFR",
            "rudder": "DRUD",
        },
        residual_outputs={"cl": "CL", "cd": "CD", "cy": "CY", "cr": "CR", "cm": "CM", "cn": "CN"},
        fixed_inputs={"DLG": 0.0},
    )


@dataclass(frozen=True, slots=True)
class HL20DavemlAerodynamics:
    """Evaluate the pinned HL-20 source graph as body loads."""

    sidecar: str = HL20_SOURCE_SIDECAR
    reference_area_m2: float = 26.612075808
    mean_aerodynamic_chord_m: float = 8.607552
    span_m: float = 4.233672
    speed_of_sound_m_s: float = HL20_SOURCE_SPEED_OF_SOUND_M_S
    sea_level_density_kg_m3: float = 1.225
    density_scale_height_m: float = 8_500.0

    @property
    def model_id(self) -> str:
        return HL20_SOURCE_MODEL_ID

    @property
    def provenance(self) -> dict[str, object]:
        return {
            "aerodynamic_model": self.model_id,
            "source_package_sha256": HL20_SOURCE_PACKAGE_SHA256,
            "source_document_sha256": HL20_SOURCE_DOCUMENT_SHA256,
            "source_sidecar": self.sidecar,
            "source_force_frame": "body_axes_frd",
            "source_coefficient_channels": ["CL", "CD", "CY", "CR", "CM", "CN"],
            "source_validity_envelope": dict(HL20_SOURCE_ENVELOPE),
            "source_control_envelope_deg": dict(HL20_SOURCE_CONTROL_ENVELOPE),
            "source_numerical_boundary_tolerance_rad": HL20_SOURCE_NUMERICAL_BOUNDARY_TOLERANCE_RAD,
            "speed_of_sound_model": "constant_340.294_m_s_assumption",
            "atmosphere_model": "exponential_density",
        }

    def evaluate(
        self,
        body_velocity_m_s: Vector,
        altitude_m: float,
        body_rates_rad_s: Vector = (0.0, 0.0, 0.0),
        controls: Mapping[str, float] | None = None,
        actuator_profile_id: str | None = None,
        time_s: float = 0.0,
    ) -> ReachabilityAeroLoads:
        """Evaluate source loads, rejecting every out-of-envelope query."""

        _finite_vector(body_velocity_m_s, "body velocity")
        _finite_vector(body_rates_rad_s, "body rates")
        if not math.isfinite(altitude_m):
            raise ValueError("source aerodynamic altitude must be finite")
        speed = _norm(body_velocity_m_s)
        velocity_unit = _unit(body_velocity_m_s)
        u, v, w = body_velocity_m_s
        # The pinned source's ANU convention uses a velocity component above
        # the body X axis (negative FRD W) as positive angle of attack.
        alpha_rad = math.atan2(-w, max(abs(u), 1.0e-12))
        beta_rad = math.asin(max(-1.0, min(1.0, v / speed)))
        if -HL20_SOURCE_NUMERICAL_BOUNDARY_TOLERANCE_RAD <= alpha_rad < 0.0:
            alpha_rad = 0.0
        if abs(alpha_rad) < 1.0e-12:
            alpha_rad = 0.0
        if abs(beta_rad) < 1.0e-12:
            beta_rad = 0.0
        mach = speed / self.speed_of_sound_m_s
        bounds = HL20_SOURCE_ENVELOPE
        checks = (
            ("mach", mach, bounds["mach_min"], bounds["mach_max"]),
            ("alpha_rad", alpha_rad, bounds["alpha_min_rad"], bounds["alpha_max_rad"]),
            ("beta_rad", beta_rad, bounds["beta_min_rad"], bounds["beta_max_rad"]),
            ("altitude_m", altitude_m, bounds["altitude_min_m"], bounds["altitude_max_m"]),
        )
        outside = [f"{name}={value:.6g} not in [{lower:.6g}, {upper:.6g}]" for name, value, lower, upper in checks if not lower <= value <= upper]
        if outside:
            raise ValueError("HL-20 DAVE-ML source query is outside its validity envelope: " + "; ".join(outside))
        requested_controls = _source_controls(controls or {})
        actuator_trace: dict[str, object] | None = None
        source_controls = requested_controls
        if actuator_profile_id is not None:
            from .hl20_controls import HL20ActuatorProfile

            mode = "direct" if actuator_profile_id.endswith("ideal_direct.v1") else "first_order"
            trace = HL20ActuatorProfile(profile_id=actuator_profile_id, mode=mode).realize(
                requested_controls,
                duration_s=max(time_s, 1.0e-3),
            )
            source_controls = dict(trace.achieved_deg)
            actuator_trace = trace.as_dict()
        invalid_controls = []
        for name, value in source_controls.items():
            if not math.isfinite(value):
                invalid_controls.append(f"{name}={value!r} is not finite")
                continue
            lower, upper = HL20_SOURCE_CONTROL_ENVELOPE[name]
            if not lower <= value <= upper:
                invalid_controls.append(f"{name}={value:.6g} not in [{lower:.6g}, {upper:.6g}]")
        if invalid_controls:
            raise ValueError("HL-20 DAVE-ML surface query is outside its validity envelope: " + "; ".join(invalid_controls))
        binding = _load_hl20_binding(self.sidecar)
        state = {
            "alpha_deg": math.degrees(alpha_rad),
            "beta_deg": math.degrees(beta_rad),
            "mach": mach,
            "roll_rate_rad_s": body_rates_rad_s[0],
            "pitch_rate_rad_s": body_rates_rad_s[1],
            "yaw_rate_rad_s": body_rates_rad_s[2],
            "true_airspeed_f_s": speed * 3.280839895013123,
            "height_ft": altitude_m * 3.280839895013123,
        }
        coefficients = binding.evaluate(state, source_controls)
        density = self.sea_level_density_kg_m3 * math.exp(-max(0.0, altitude_m) / self.density_scale_height_m)
        dynamic_pressure_pa = 0.5 * density * speed**2
        if not math.isfinite(dynamic_pressure_pa) or dynamic_pressure_pa <= bounds["dynamic_pressure_min_pa"]:
            raise ValueError(f"HL-20 DAVE-ML dynamic pressure must be positive and finite: {dynamic_pressure_pa!r}")
        scale = dynamic_pressure_pa * self.reference_area_m2
        drag_direction = _scale(velocity_unit, -1.0)
        lift_direction = _projected_up(velocity_unit)
        side_direction = _projected_right(velocity_unit)
        force = _add_vectors(
            _add_vectors(_scale(drag_direction, scale * coefficients["cd"]), _scale(lift_direction, scale * coefficients["cl"])),
            _scale(side_direction, scale * coefficients["cy"]),
        )
        moment = (
            scale * self.span_m * coefficients["cr"],
            scale * self.mean_aerodynamic_chord_m * coefficients["cm"],
            scale * self.span_m * coefficients["cn"],
        )
        return ReachabilityAeroLoads(
            force,
            moment,
            tuple(sorted((name, float(value)) for name, value in coefficients.items())),
            tuple(sorted({**state, "dynamic_pressure_pa": dynamic_pressure_pa}.items())),
            actuator_trace,
        )


def _add_vectors(left: Vector, right: Vector) -> Vector:
    return tuple(a + b for a, b in zip(left, right, strict=True))  # type: ignore[return-value]


__all__ = [
    "HL20_SOURCE_WRENCH_NAMES",
    "HL20DavemlAerodynamics",
    "HL20_SOURCE_DOCUMENT_SHA256",
    "HL20_SOURCE_ENVELOPE",
    "HL20_SOURCE_MODEL_ID",
    "HL20_SOURCE_PACKAGE_SHA256",
    "HL20_SOURCE_SPEED_OF_SOUND_M_S",
    "ReachabilityAeroLoads",
    "build_hl20_source_effectiveness",
    "probe_hl20_control_directions",
]
