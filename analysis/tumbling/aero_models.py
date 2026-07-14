from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]


def sphere_projected_area(radius: float) -> float:
    if radius <= 0.0:
        raise ValueError("radius must be positive")
    ####
    return float(np.pi * radius**2)
####


def spheroid_projected_area(
    axial_semi_axis: float,
    transverse_semi_axis: float,
    alpha_rad: FloatArray,
) -> FloatArray:
    if axial_semi_axis <= 0.0 or transverse_semi_axis <= 0.0:
        raise ValueError("semi-axes must be positive")
    ####
    c = np.cos(alpha_rad)
    s = np.sin(alpha_rad)
    return np.pi * transverse_semi_axis * np.sqrt(
        transverse_semi_axis**2 * c**2 + axial_semi_axis**2 * s**2,
    )
####


def triaxial_ellipsoid_projected_area(
    semi_axes: tuple[float, float, float],
    flow_direction_body: FloatArray,
) -> float:
    a, b, c = semi_axes
    if min(a, b, c) <= 0.0:
        raise ValueError("all semi-axes must be positive")
    ####
    n = np.asarray(flow_direction_body, dtype=np.float64)
    norm = float(np.linalg.norm(n))
    if norm <= 0.0:
        raise ValueError("flow direction must be non-zero")
    ####
    n = n / norm
    return float(np.pi * a * b * c * np.sqrt((n[0] / a) ** 2 + (n[1] / b) ** 2 + (n[2] / c) ** 2))
####


def cylinder_projected_area(radius: float, length: float, alpha_rad: FloatArray) -> FloatArray:
    if radius <= 0.0 or length <= 0.0:
        raise ValueError("radius and length must be positive")
    ####
    c = np.abs(np.cos(alpha_rad))
    s = np.abs(np.sin(alpha_rad))
    return np.pi * radius**2 * c + 2.0 * radius * length * s
####


def cone_projected_area(radius: float, height: float, alpha_rad: FloatArray) -> FloatArray:
    if radius <= 0.0 or height <= 0.0:
        raise ValueError("radius and height must be positive")
    ####
    c = np.abs(np.cos(alpha_rad))
    s = np.abs(np.sin(alpha_rad))
    out = np.empty_like(alpha_rad)
    broadside = c <= 1e-12
    out[broadside] = radius * height
    active_idx = np.flatnonzero(~broadside)
    delta = height * s[active_idx] / (radius * c[active_idx])
    subcritical = delta <= 1.0
    out[active_idx[subcritical]] = np.pi * radius**2 * c[active_idx[subcritical]]
    supercritical = ~subcritical
    out[active_idx[supercritical]] = (
        radius**2
        * c[active_idx[supercritical]]
        * (
            np.pi
            + np.sqrt(delta[supercritical] ** 2 - 1.0)
            - np.arccos(1.0 / delta[supercritical])
        )
    )
    return out
####


def sphere_isotropic_mean_projected_area(radius: float) -> float:
    return sphere_projected_area(radius)
####


def cylinder_isotropic_mean_projected_area(radius: float, length: float) -> float:
    if radius <= 0.0 or length <= 0.0:
        raise ValueError("radius and length must be positive")
    ####
    return float(np.pi * radius * (length + radius) / 2.0)
####


def cone_isotropic_mean_projected_area(radius: float, height: float) -> float:
    if radius <= 0.0 or height <= 0.0:
        raise ValueError("radius and height must be positive")
    ####
    return float(np.pi * radius * (radius + np.sqrt(radius**2 + height**2)) / 4.0)
####


def ellipsoid_isotropic_mean_projected_area(surface_area: float) -> float:
    if surface_area <= 0.0:
        raise ValueError("surface area must be positive")
    ####
    return float(surface_area / 4.0)
####
