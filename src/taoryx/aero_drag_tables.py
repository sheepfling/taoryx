"""Full-angle aerodynamic drag-area models for tumbling-body studies."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class DragModel:
    """Illustrative coefficient model and reference area for one body."""

    name: str
    shape: str
    reference_area_m2: float
    parameters: dict[str, float]
####


@dataclass(frozen=True, slots=True)
class AeroSweep:
    """Full-angle drag-area sweep for one body."""

    angle_deg: FloatArray
    projected_area_m2: FloatArray
    drag_area_m2: FloatArray
    cd: FloatArray
####


def full_angle_axis(step_deg: float = 5.0) -> FloatArray:
    if step_deg <= 0.0 or 360.0 % step_deg != 0.0:
        raise ValueError("step_deg must be a positive divisor of 360")
    ####
    return np.arange(0.0, 360.0 + 0.5 * step_deg, step_deg, dtype=np.float64)
####


def periodic_linear_interpolate(angle_deg: float, axis_deg: FloatArray, values: FloatArray) -> float:
    """Interpolate periodic tabular data, including the 360-to-0 seam."""

    if len(axis_deg) != len(values) or len(axis_deg) < 2:
        raise ValueError("periodic interpolation requires two or more paired samples")
    ####
    wrapped = float(angle_deg % 360.0)
    axis = np.asarray(axis_deg, dtype=np.float64)
    data = np.asarray(values, dtype=np.float64)
    if axis[0] != 0.0:
        raise ValueError("periodic interpolation axis must start at zero degrees")
    ####
    if axis[-1] == 360.0:
        return float(np.interp(wrapped, axis, data))
    ####
    extended_axis = np.concatenate((axis, np.array([360.0], dtype=np.float64)))
    extended_data = np.concatenate((data, np.array([data[0]], dtype=np.float64)))
    return float(np.interp(wrapped, extended_axis, extended_data))
####


def spherical_direction(alpha_t_deg: FloatArray, phi_deg: FloatArray) -> tuple[FloatArray, FloatArray, FloatArray]:
    """Return body-direction components using TAOS total-angle conventions."""

    alpha = np.deg2rad(alpha_t_deg)
    phi = np.deg2rad(phi_deg)
    return np.cos(alpha), -np.sin(alpha) * np.sin(phi), np.sin(alpha) * np.cos(phi)
####


def projected_area(shape: str, angle_deg: FloatArray, parameters: dict[str, float]) -> FloatArray:
    """Evaluate planar projected area for an axisymmetric or spherical body."""

    angle = np.deg2rad(angle_deg)
    radius = parameters.get("radius_m", 0.5)
    if radius <= 0.0:
        raise ValueError("radius_m must be positive")
    ####
    if shape == "sphere":
        return np.full_like(angle, math.pi * radius**2)
    if shape == "spheroid":
        axial = parameters["axial_semi_axis_m"]
        transverse = parameters["transverse_semi_axis_m"]
        c = np.cos(angle)
        s = np.sin(angle)
        return math.pi * transverse * np.sqrt(transverse**2 * c**2 + axial**2 * s**2)
    if shape == "cylinder":
        length = parameters["length_m"]
        return math.pi * radius**2 * np.abs(np.cos(angle)) + 2.0 * radius * length * np.abs(np.sin(angle))
    if shape == "cone":
        height = parameters["height_m"]
        c = np.abs(np.cos(angle))
        s = np.abs(np.sin(angle))
        result = np.empty_like(angle)
        broadside = c <= 1e-12
        result[broadside] = radius * height
        active = np.flatnonzero(~broadside)
        delta = height * s[active] / (radius * c[active])
        subcritical = delta <= 1.0
        result[active[subcritical]] = math.pi * radius**2 * c[active[subcritical]]
        supercritical = ~subcritical
        result[active[supercritical]] = radius**2 * c[active[supercritical]] * (
            math.pi
            + np.sqrt(delta[supercritical] ** 2 - 1.0)
            - np.arccos(1.0 / delta[supercritical])
        )
        return result
    raise ValueError(f"unsupported planar shape: {shape}")
####


def triaxial_projected_area(
    alpha_t_deg: FloatArray,
    phi_deg: FloatArray,
    semi_axes_m: tuple[float, float, float],
) -> FloatArray:
    a, b, c = semi_axes_m
    if min(a, b, c) <= 0.0:
        raise ValueError("triaxial semi-axes must be positive")
    ####
    nx, ny, nz = spherical_direction(alpha_t_deg, phi_deg)
    return math.pi * a * b * c * np.sqrt((nx / a) ** 2 + (ny / b) ** 2 + (nz / c) ** 2)
####


def drag_coefficient(model: DragModel, angle_deg: FloatArray, projected_area_m2: FloatArray) -> FloatArray:
    """Return illustrative CD values, with drag area kept as the primary quantity."""

    shape = model.shape
    angle = np.deg2rad(angle_deg)
    if shape == "sphere":
        cd = np.full_like(angle, model.parameters.get("cd", 0.47))
    elif shape in {"cylinder", "cone"}:
        exponent = model.parameters.get("blend_exponent", 3.0)
        c = np.cos(angle)
        s = np.abs(np.sin(angle))
        cd = model.parameters.get("cd_broadside", 1.0) * s**exponent
        cd += model.parameters.get("cd_front", model.parameters.get("cd_end", 1.0)) * np.maximum(c, 0.0) ** exponent
        cd += model.parameters.get("cd_rear", model.parameters.get("cd_end", 1.0)) * np.maximum(-c, 0.0) ** exponent
    else:
        cd = projected_area_m2 / model.reference_area_m2
    return cd
####


def generate_sweep(model: DragModel, angle_deg: FloatArray | None = None) -> AeroSweep:
    angles = full_angle_axis() if angle_deg is None else np.asarray(angle_deg, dtype=np.float64)
    if model.shape == "triaxial":
        raise ValueError("use generate_triaxial_sweep for a two-angle body")
    ####
    area = projected_area(model.shape, angles, model.parameters)
    cd = drag_coefficient(model, angles, area)
    return AeroSweep(angles, area, model.reference_area_m2 * cd, cd)
####


def generate_triaxial_sweep(model: DragModel, *, step_deg: float = 10.0) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray]:
    if model.shape != "triaxial":
        raise ValueError("generate_triaxial_sweep requires a triaxial model")
    ####
    alpha = np.arange(0.0, 180.0 + 0.5 * step_deg, step_deg, dtype=np.float64)
    phi = np.arange(0.0, 360.0 + 0.5 * step_deg, step_deg, dtype=np.float64)
    alpha_grid, phi_grid = np.meshgrid(alpha, phi, indexing="ij")
    semi_axes: tuple[float, float, float] = (
        model.parameters["a_m"],
        model.parameters["b_m"],
        model.parameters["c_m"],
    )
    area = triaxial_projected_area(alpha_grid, phi_grid, semi_axes)
    cd = area / model.reference_area_m2
    return alpha, phi, area, cd
####


def write_sweep_csv(path: Path, sweep: AeroSweep) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("angle_deg", "projected_area_m2", "drag_area_m2", "cd"))
        writer.writerows(zip(sweep.angle_deg, sweep.projected_area_m2, sweep.drag_area_m2, sweep.cd, strict=True))
    ####
####
