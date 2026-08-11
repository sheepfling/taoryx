"""Reusable source-compatible round-Earth 3-DoF CADAC substrate."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TypeAlias

import numpy as np
from numpy.typing import NDArray
from pydantic import Field

from .compatibility import cadac_stored_derivative_step
from .input_ast import CadacModel

FloatVector: TypeAlias = NDArray[np.float64]
FloatMatrix: TypeAlias = NDArray[np.float64]

REARTH_M = 6_370_987.308
EARTH_ROTATION_RAD_S = 7.292115e-5
GRAVITATIONAL_CONSTANT = 6.673e-11
EARTH_MASS_KG = 5.973e24
AIR_GAS_CONSTANT = 287.053
RAD_PER_DEG = 0.0174532925199432
DEG_PER_RAD = 57.2957795130823


class CadacRound3InitialState(CadacModel):
    """Source initial state used by CADAC ``Round3`` actors."""

    longitude_deg: float
    latitude_deg: float
    altitude_m: float
    speed_mps: float = Field(gt=0.0)
    heading_deg: float
    flight_path_deg: float


####


@dataclass(frozen=True, slots=True)
class CadacRound3Environment:
    """Atmosphere and flight-condition outputs of the source environment module."""

    gravity_mps2: float
    density_kg_m3: float
    dynamic_pressure_pa: float
    mach: float
    speed_of_sound_mps: float


####


@dataclass(slots=True)
class CadacRound3RuntimeState:
    """Mutable high-rate state mirroring the participating ``Round3`` arrays."""

    time_s: float
    longitude_rad: float
    latitude_rad: float
    altitude_m: float
    speed_mps: float
    heading_rad: float
    flight_path_rad: float
    tgv: FloatMatrix
    tig: FloatMatrix
    weii: FloatMatrix
    sbeg_m: FloatVector
    vbeg_mps: FloatVector
    sbii_m: FloatVector
    vbii_mps: FloatVector
    abii_mps2: FloatVector


####


def cadac_mat2tr(heading_rad: float, flight_path_rad: float) -> FloatMatrix:
    """Return CADAC's velocity-wrt-geographic transformation matrix."""

    cpsi = math.cos(heading_rad)
    spsi = math.sin(heading_rad)
    ctht = math.cos(flight_path_rad)
    stht = math.sin(flight_path_rad)
    return np.asarray(
        (
            (ctht * cpsi, ctht * spsi, -stht),
            (-spsi, cpsi, 0.0),
            (stht * cpsi, stht * spsi, ctht),
        ),
        dtype=np.float64,
    )


####


def cadac_cadtge(longitude_rad: float, latitude_rad: float) -> FloatMatrix:
    """Return CADAC's geographic-wrt-Earth transformation matrix."""

    clon = math.cos(longitude_rad)
    slon = math.sin(longitude_rad)
    clat = math.cos(latitude_rad)
    slat = math.sin(latitude_rad)
    return np.asarray(
        (
            (-slat * clon, -slat * slon, clat),
            (-slon, clon, 0.0),
            (-clat * clon, -clat * slon, -slat),
        ),
        dtype=np.float64,
    )


####


def cadac_cadtei(time_s: float) -> FloatMatrix:
    """Return CADAC's Earth-wrt-inertial rotation at one simulation time."""

    xi = EARTH_ROTATION_RAD_S * time_s
    cxi = math.cos(xi)
    sxi = math.sin(xi)
    return np.asarray(
        (
            (cxi, sxi, 0.0),
            (-sxi, cxi, 0.0),
            (0.0, 0.0, 1.0),
        ),
        dtype=np.float64,
    )


####


def cadac_cadsph(earth_position_m: FloatVector) -> tuple[float, float, float]:
    """Return source-compatible longitude, latitude, and altitude from Earth coordinates."""

    x, y, z = (float(value) for value in earth_position_m)
    radius = math.sqrt(x * x + y * y + z * z)
    if radius <= 0.0:
        raise ValueError("CADAC spherical conversion requires nonzero radius")
    ####
    latitude = math.asin(z / radius)
    altitude = radius - REARTH_M
    horizontal = math.sqrt(x * x + y * y)
    if horizontal <= 0.0:
        longitude = 0.0
    else:
        dum4 = math.asin(max(-1.0, min(1.0, y / horizontal)))
        if x >= 0.0 and y >= 0.0:
            longitude = dum4
        elif x < 0.0:
            longitude = math.pi - dum4
        else:
            longitude = 2.0 * math.pi + dum4
        ####
        if longitude > math.pi:
            longitude = -(2.0 * math.pi - longitude)
        ####
    ####
    return longitude, latitude, altitude


####


def cadac_cadtbv(bank_rad: float, alpha_rad: float) -> FloatMatrix:
    """Return the source body-wrt-velocity transform for bank-to-turn 3-DoF."""

    salpha = math.sin(alpha_rad)
    calpha = math.cos(alpha_rad)
    sphi = math.sin(bank_rad)
    cphi = math.cos(bank_rad)
    return np.asarray(
        (
            (calpha, sphi * salpha, -cphi * salpha),
            (0.0, cphi, sphi),
            (salpha, -sphi * calpha, cphi * calpha),
        ),
        dtype=np.float64,
    )


####


def cadac_cadine(longitude_rad: float, latitude_rad: float, altitude_m: float, time_s: float) -> FloatVector:
    """Return source inertial coordinates from longitude, latitude, altitude and time."""

    radius = altitude_m + REARTH_M
    celestial_longitude = longitude_rad + EARTH_ROTATION_RAD_S * time_s
    clat = math.cos(latitude_rad)
    return np.asarray(
        (
            radius * clat * math.cos(celestial_longitude),
            radius * clat * math.sin(celestial_longitude),
            radius * math.sin(latitude_rad),
        ),
        dtype=np.float64,
    )


####


def cadac_round3_initialize(initial: CadacRound3InitialState) -> CadacRound3RuntimeState:
    """Initialize source inertial/geographic state exactly at the CADAC epoch."""

    longitude = initial.longitude_deg * RAD_PER_DEG
    latitude = initial.latitude_deg * RAD_PER_DEG
    heading = initial.heading_deg * RAD_PER_DEG
    flight_path = initial.flight_path_deg * RAD_PER_DEG
    sbig = np.asarray((0.0, 0.0, -(initial.altitude_m + REARTH_M)), dtype=np.float64)
    tge = cadac_cadtge(longitude, latitude)
    teg = tge.T
    sbii = teg @ sbig
    vbeg = _cart_from_polar(initial.speed_mps, heading, flight_path)
    weii = np.asarray(
        (
            (0.0, -EARTH_ROTATION_RAD_S, 0.0),
            (EARTH_ROTATION_RAD_S, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        ),
        dtype=np.float64,
    )
    tig = teg.copy()
    vbii = tig @ vbeg + weii @ sbii
    tvg = cadac_mat2tr(heading, flight_path)
    return CadacRound3RuntimeState(
        time_s=0.0,
        longitude_rad=longitude,
        latitude_rad=latitude,
        altitude_m=initial.altitude_m,
        speed_mps=initial.speed_mps,
        heading_rad=heading,
        flight_path_rad=flight_path,
        tgv=tvg.T,
        tig=tig,
        weii=weii,
        sbeg_m=np.zeros(3, dtype=np.float64),
        vbeg_mps=vbeg,
        sbii_m=sbii,
        vbii_mps=vbii,
        abii_mps2=np.zeros(3, dtype=np.float64),
    )


####


def cadac_round3_environment(state: CadacRound3RuntimeState) -> CadacRound3Environment:
    """Evaluate the CRUISE/``Round3`` ISO-62 environment at the current truth state."""

    altitude = state.altitude_m
    gravity = GRAVITATIONAL_CONSTANT * EARTH_MASS_KG / (REARTH_M + altitude) ** 2
    if altitude < 11_000.0:
        temperature_k = 288.15 - 0.0065 * altitude
        pressure_pa = 101_325.0 * (temperature_k / 288.15) ** 5.2559
    else:
        temperature_k = 216.0
        pressure_pa = 22_630.0 * math.exp(-0.00015769 * (altitude - 11_000.0))
    ####
    density = pressure_pa / (AIR_GAS_CONSTANT * temperature_k)
    speed_of_sound = math.sqrt(1.4 * AIR_GAS_CONSTANT * temperature_k)
    mach = abs(state.speed_mps / speed_of_sound)
    dynamic_pressure = 0.5 * density * state.speed_mps * state.speed_mps
    return CadacRound3Environment(gravity, density, dynamic_pressure, mach, speed_of_sound)


####


def cadac_round3_newton_step(
    state: CadacRound3RuntimeState,
    specific_force_velocity_mps2: FloatVector,
    gravity_mps2: float,
    dt_s: float,
) -> None:
    """Advance the source round-Earth Newton equations by one stored-derivative step."""

    force = np.asarray(specific_force_velocity_mps2, dtype=np.float64)
    if force.shape != (3,) or not np.all(np.isfinite(force)):
        raise ValueError("Round3 specific force must be a finite 3-vector")
    ####
    gravity = np.asarray((0.0, 0.0, gravity_mps2), dtype=np.float64)
    abii_new = state.tig @ (state.tgv @ force + gravity)
    vbii_new = _integrate_vector(abii_new, state.abii_mps2, state.vbii_mps, dt_s)
    sbii_new = _integrate_vector(vbii_new, state.vbii_mps, state.sbii_m, dt_s)
    state.abii_mps2 = abii_new
    state.vbii_mps = vbii_new
    state.sbii_m = sbii_new

    tei = cadac_cadtei(state.time_s)
    sbie = tei @ state.sbii_m
    longitude, latitude, altitude = cadac_cadsph(sbie)
    tge = cadac_cadtge(longitude, latitude)
    tgi = tge @ tei
    vbeg_new = tgi @ (state.vbii_mps - state.weii @ state.sbii_m)
    state.sbeg_m = _integrate_vector(vbeg_new, state.vbeg_mps, state.sbeg_m, dt_s)
    state.vbeg_mps = vbeg_new

    speed, heading, flight_path = _polar_from_cartesian(state.vbeg_mps)
    state.longitude_rad = longitude
    state.latitude_rad = latitude
    state.altitude_m = altitude
    state.speed_mps = speed
    state.heading_rad = heading
    state.flight_path_rad = flight_path
    state.tig = tgi.T
    state.tgv = cadac_mat2tr(heading, flight_path).T


####


def _cart_from_polar(magnitude: float, azimuth: float, elevation: float) -> FloatVector:
    return np.asarray(
        (
            magnitude * math.cos(elevation) * math.cos(azimuth),
            magnitude * math.cos(elevation) * math.sin(azimuth),
            -magnitude * math.sin(elevation),
        ),
        dtype=np.float64,
    )


####


def _polar_from_cartesian(vector: FloatVector) -> tuple[float, float, float]:
    x, y, z = (float(value) for value in vector)
    magnitude = math.sqrt(x * x + y * y + z * z)
    azimuth = math.atan2(y, x)
    horizontal = math.sqrt(x * x + y * y)
    if horizontal > 0.0:
        elevation = math.atan2(-z, horizontal)
    elif z > 0.0:
        elevation = -math.pi / 2.0
    elif z < 0.0:
        elevation = math.pi / 2.0
    else:
        elevation = 0.0
    ####
    return magnitude, azimuth, elevation


####


def _integrate_vector(new_rate: FloatVector, old_rate: FloatVector, value: FloatVector, dt_s: float) -> FloatVector:
    return np.asarray(
        [cadac_stored_derivative_step((float(v),), (float(n),), (float(o),), dt_s)[0] for n, o, v in zip(new_rate, old_rate, value, strict=True)],
        dtype=np.float64,
    )


####


__all__ = [
    "AIR_GAS_CONSTANT",
    "CadacRound3Environment",
    "CadacRound3InitialState",
    "CadacRound3RuntimeState",
    "DEG_PER_RAD",
    "EARTH_MASS_KG",
    "EARTH_ROTATION_RAD_S",
    "GRAVITATIONAL_CONSTANT",
    "RAD_PER_DEG",
    "REARTH_M",
    "cadac_cadine",
    "cadac_cadsph",
    "cadac_cadtbv",
    "cadac_cadtei",
    "cadac_cadtge",
    "cadac_mat2tr",
    "cadac_round3_environment",
    "cadac_round3_initialize",
    "cadac_round3_newton_step",
]
