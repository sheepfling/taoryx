"""Problem-level wind resolution."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Callable, Protocol

from taoryx.contracts import Frame, FrameVector3, Vector3


@dataclass(frozen=True, slots=True)
class EnvironmentSample:
    """One deterministic environment sample consumed by a runtime step.

    The core atmosphere fields are sufficient for the current 3-DOF force
    model. Weather fields are optional data carried through the boundary so a
    provider can add them without changing the translational state contract.
    Wind is an earth-fixed ECFC vector; callers must subtract it from the
    earth-relative vehicle velocity before deriving air-relative quantities.
    """

    density: float
    pressure: float
    temperature: float
    speed_of_sound: float
    wind: FrameVector3
    humidity: float | None = None
    cloud_fraction: float | None = None
    rain_rate: float | None = None
####


class EnvironmentProvider(Protocol):
    """Provider boundary for deterministic or live environment data."""

    def sample(self, *, time: float, position: FrameVector3) -> EnvironmentSample:
        """Return the environment at one simulation time and position."""
        ...
    ####
####


@dataclass(frozen=True, slots=True)
class StaticEnvironmentProvider:
    """Provider useful for tests and replayable deterministic scenarios."""

    value: EnvironmentSample

    def sample(self, *, time: float, position: FrameVector3) -> EnvironmentSample:
        del time, position
        return self.value
    ####


@dataclass(frozen=True, slots=True)
class ExponentialAtmosphereProvider:
    """Deterministic altitude atmosphere with a fixed ECFC wind vector."""

    reference_radius_m: float
    sea_level_density: float = 1.225
    scale_height_m: float = 8_500.0
    sea_level_temperature_k: float = 288.15
    vacuum_altitude_m: float = 120_000.0
    wind: FrameVector3 = FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC)

    def __post_init__(self) -> None:
        if self.reference_radius_m <= 0.0 or self.sea_level_density < 0.0 or self.scale_height_m <= 0.0:
            raise ValueError("atmosphere geometry and density parameters must be valid")
        if self.vacuum_altitude_m < 0.0:
            raise ValueError("vacuum altitude must be nonnegative")
        if self.wind.frame is not Frame.ECFC:
            raise ValueError("atmosphere wind must be expressed in ECFC")
        ####

    def sample(self, *, time: float, position: FrameVector3) -> EnvironmentSample:
        """Return density, thermodynamic values, and wind at ECFC position."""

        del time
        altitude = position.vector.norm() - self.reference_radius_m
        if altitude >= self.vacuum_altitude_m:
            density = 0.0
        else:
            density = self.sea_level_density * math.exp(-max(0.0, altitude) / self.scale_height_m)
        temperature = max(180.0, self.sea_level_temperature_k - 0.0065 * max(0.0, altitude))
        pressure = density * 287.05 * temperature
        speed_of_sound = math.sqrt(1.4 * 287.05 * temperature)
        return EnvironmentSample(density, pressure, temperature, speed_of_sound, self.wind)
        ####
    ####
####


@dataclass(frozen=True, slots=True)
class WindFieldEnvironmentProvider:
    """Overlay a position/time-dependent wind field on an atmosphere.

    The atmosphere remains responsible for thermodynamic quantities.  The
    resolver only supplies the earth-fixed ECFC wind required by the
    air-relative aerodynamic calculation.
    """

    atmosphere: EnvironmentProvider
    wind_resolver: Callable[[float, FrameVector3], FrameVector3]

    def sample(self, *, time: float, position: FrameVector3) -> EnvironmentSample:
        sample = self.atmosphere.sample(time=time, position=position)
        wind = self.wind_resolver(time, position)
        if wind.frame is not Frame.ECFC:
            raise ValueError("resolved atmosphere wind must be expressed in ECFC")
        return replace(sample, wind=wind)
    ####
####


@dataclass(frozen=True, slots=True)
class EnvironmentKeyframe:
    """A time-stamped environment sample for deterministic replay."""

    time: float
    value: EnvironmentSample
####


@dataclass(frozen=True, slots=True)
class ScheduledEnvironmentProvider:
    """Linearly interpolate environment samples between source keyframes."""

    keyframes: tuple[EnvironmentKeyframe, ...]

    def __post_init__(self) -> None:
        if not self.keyframes:
            raise ValueError("scheduled environment requires at least one keyframe")
        if any(left.time >= right.time for left, right in zip(self.keyframes, self.keyframes[1:], strict=False)):
            raise ValueError("environment keyframe times must be strictly increasing")
        ####
    ####

    def sample(self, *, time: float, position: FrameVector3) -> EnvironmentSample:
        del position
        if time <= self.keyframes[0].time:
            return self.keyframes[0].value
        if time >= self.keyframes[-1].time:
            return self.keyframes[-1].value
        for left, right in zip(self.keyframes, self.keyframes[1:], strict=True):
            if left.time <= time <= right.time:
                fraction = (time - left.time) / (right.time - left.time)
                return _interpolate_environment(left.value, right.value, fraction)
        raise RuntimeError("environment keyframe lookup failed")
    ####
####


def _interpolate_environment(left: EnvironmentSample, right: EnvironmentSample, fraction: float) -> EnvironmentSample:
    def blend(a: float, b: float) -> float:
        return a + fraction * (b - a)
    ####

    return EnvironmentSample(
        density=blend(left.density, right.density),
        pressure=blend(left.pressure, right.pressure),
        temperature=blend(left.temperature, right.temperature),
        speed_of_sound=blend(left.speed_of_sound, right.speed_of_sound),
        wind=FrameVector3(
            Vector3(
                blend(left.wind.vector.x, right.wind.vector.x),
                blend(left.wind.vector.y, right.wind.vector.y),
                blend(left.wind.vector.z, right.wind.vector.z),
            ),
            Frame.ECFC,
        ),
        humidity=_blend_optional(left.humidity, right.humidity, fraction),
        cloud_fraction=_blend_optional(left.cloud_fraction, right.cloud_fraction, fraction),
        rain_rate=_blend_optional(left.rain_rate, right.rain_rate, fraction),
    )
####


def _blend_optional(left: float | None, right: float | None, fraction: float) -> float | None:
    if left is None or right is None:
        return right if fraction >= 1.0 else left
    return left + fraction * (right - left)
####


@dataclass(frozen=True, slots=True)
class WindResult:
    """Wind in local east/north/down and ECFC components."""

    local: Vector3
    ecfc: FrameVector3


def evaluate_wind(
    *,
    magnitude: float | None = None,
    heading: float | None = None,
    east: float | None = None,
    north: float | None = None,
    down: float = 0.0,
    longitude: float = 0.0,
    latitude: float = 0.0,
) -> WindResult:
    """Resolve speed/heading or E/N/D input and rotate local wind to ECFC.

    Heading is measured clockwise from north. This implements
    ``TAOS-ALG-PRB-012`` and the local-frame convention of equations 2-19--2-21.
    """

    if magnitude is not None or heading is not None:
        if magnitude is None or heading is None:
            raise ValueError("magnitude and heading must be supplied together")
        east_value = magnitude * math.sin(heading)
        north_value = magnitude * math.cos(heading)
    elif east is not None and north is not None:
        east_value, north_value = east, north
    else:
        raise ValueError("wind requires magnitude/heading or east/north components")
    local = Vector3(float(north_value), float(east_value), float(down))
    north_axis = Vector3(-math.sin(latitude) * math.cos(longitude), -math.sin(latitude) * math.sin(longitude), math.cos(latitude))
    east_axis = Vector3(-math.sin(longitude), math.cos(longitude), 0.0)
    down_axis = Vector3(-math.cos(latitude) * math.cos(longitude), -math.cos(latitude) * math.sin(longitude), -math.sin(latitude))
    ecfc = north_axis.scaled(local.x) + east_axis.scaled(local.y) + down_axis.scaled(local.z)
    return WindResult(local, FrameVector3(ecfc, Frame.ECFC))
####
