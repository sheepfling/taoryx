"""Isolated CADAC source-environment compatibility equations.

This module exists so source parity code does not masquerade as a second host
environment service. Native Taoryx execution must use the shared environment
and gravity provider boundaries instead of calling these helpers.
"""

from __future__ import annotations

import math

CADAC_SOURCE_EARTH_RADIUS_M = 6_370_987.308
CADAC_SOURCE_GRAVITATIONAL_CONSTANT = 6.673e-11
CADAC_SOURCE_EARTH_MASS_KG = 5.973e24


def cadac_source_inverse_square_gravity_mps2(altitude_m: float) -> float:
    """Return the legacy flat-Earth CADAC inverse-square gravity magnitude."""

    if not math.isfinite(altitude_m):
        raise ValueError("CADAC source gravity altitude must be finite")
    radius = CADAC_SOURCE_EARTH_RADIUS_M + altitude_m
    if radius <= 0.0:
        raise ValueError("CADAC source gravity radius must be positive")
    return CADAC_SOURCE_GRAVITATIONAL_CONSTANT * CADAC_SOURCE_EARTH_MASS_KG / (radius * radius)
    ####


def atmosphere76(altitude_m: float) -> tuple[float, float, float]:
    """Return CADAC's legacy US-1976 density, pressure, and temperature."""

    if not math.isfinite(altitude_m):
        raise ValueError("CADAC source atmosphere altitude must be finite")
    rearth_km = 6369.0
    gmr = 34.163195
    rho_sl = 1.22500
    pressure_sl = 101_325.0
    temperature_sl = 288.15
    htab = (0.0, 11.0, 20.0, 32.0, 47.0, 51.0, 71.0, 84.852)
    ttab = (288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.946)
    ptab = (
        1.0,
        2.233611e-1,
        5.403295e-2,
        8.5666784e-3,
        1.0945601e-3,
        6.6063531e-4,
        3.9046834e-5,
        3.68501e-6,
    )
    gtab = (-6.5, 0.0, 1.0, 2.8, 0.0, -2.8, -2.0, 0.0)
    altitude_km = altitude_m / 1000.0
    geopotential_km = altitude_km * rearth_km / (altitude_km + rearth_km)
    lower = 0
    upper = 7
    while True:
        middle = (lower + upper) // 2
        if geopotential_km < htab[middle]:
            upper = middle
        else:
            lower = middle
        ####
        if upper <= lower + 1:
            break
        ####
    ####
    gradient = gtab[lower]
    base_temperature = ttab[lower]
    delta_height = geopotential_km - htab[lower]
    local_temperature = base_temperature + gradient * delta_height
    theta = local_temperature / ttab[0]
    if gradient == 0.0:
        delta = ptab[lower] * math.exp(-gmr * delta_height / base_temperature)
    else:
        delta = ptab[lower] * (base_temperature / local_temperature) ** (gmr / gradient)
    ####
    sigma = delta / theta
    return rho_sl * sigma, pressure_sl * delta, temperature_sl * theta
    ####


__all__ = [
    "CADAC_SOURCE_EARTH_MASS_KG",
    "CADAC_SOURCE_EARTH_RADIUS_M",
    "CADAC_SOURCE_GRAVITATIONAL_CONSTANT",
    "atmosphere76",
    "cadac_source_inverse_square_gravity_mps2",
]
