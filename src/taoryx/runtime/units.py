"""Units/format resolution at the parser/runtime boundary."""

from __future__ import annotations

import re
from collections.abc import Mapping
from dataclasses import dataclass

from taoryx.contracts import Quantity, Unit


@dataclass(frozen=True, slots=True)
class ResolvedFormat:
    variable: str
    unit: Unit | None
    format: str | None


_UNIT_TO_SI: dict[str, tuple[str, float]] = {
    "ft": ("length", 0.3048),
    "in": ("length", 0.0254),
    "mi": ("length", 1609.344),
    "nm": ("length", 1852.0),
    "m": ("length", 1.0),
    "km": ("length", 1000.0),
    "ft/sec": ("speed", 0.3048),
    "ft/min": ("speed", 0.3048 / 60.0),
    "ft/hr": ("speed", 0.3048 / 3600.0),
    "in/sec": ("speed", 0.0254),
    "m/sec": ("speed", 1.0),
    "m/s": ("speed", 1.0),
    "km/sec": ("speed", 1000.0),
    "km/s": ("speed", 1000.0),
    "knots": ("speed", 1852.0 / 3600.0),
    "sec": ("time", 1.0),
    "s": ("time", 1.0),
    "min": ("time", 60.0),
    "hr": ("time", 3600.0),
    "deg": ("angle", 3.141592653589793 / 180.0),
    "rad": ("angle", 1.0),
    "lb": ("mass", 0.45359237),
    "kg": ("mass", 1.0),
    "gm": ("mass", 0.001),
    "g": ("mass", 0.001),
    "lbf": ("force", 4.4482216152605),
    "n": ("force", 1.0),
    "kn": ("force", 1000.0),
}

_UNIT_TO_SI.update(
    {
        "in/min": ("speed", 0.0254 / 60.0), "in/hr": ("speed", 0.0254 / 3600.0),
        "mi/sec": ("speed", 1609.344), "mi/min": ("speed", 1609.344 / 60.0), "mi/hr": ("speed", 1609.344 / 3600.0),
        "m/min": ("speed", 1.0 / 60.0), "m/hr": ("speed", 1.0 / 3600.0),
        "km/min": ("speed", 1000.0 / 60.0), "km/hr": ("speed", 1000.0 / 3600.0),
        "deg/sec": ("angular_rate", 3.141592653589793 / 180.0),
        "deg/min": ("angular_rate", 3.141592653589793 / 180.0 / 60.0),
        "deg/hr": ("angular_rate", 3.141592653589793 / 180.0 / 3600.0),
        "rad/sec": ("angular_rate", 1.0), "rad/min": ("angular_rate", 1.0 / 60.0), "rad/hr": ("angular_rate", 1.0 / 3600.0),
        "rev/sec": ("angular_rate", 2.0 * 3.141592653589793), "rpm": ("angular_rate", 2.0 * 3.141592653589793 / 60.0),
        "ft/sec2": ("acceleration", 0.3048), "ft/min2": ("acceleration", 0.3048 / 3600.0), "ft/hr2": ("acceleration", 0.3048 / 12960000.0),
        "in/sec2": ("acceleration", 0.0254), "in/min2": ("acceleration", 0.0254 / 3600.0), "in/hr2": ("acceleration", 0.0254 / 12960000.0),
        "mi/sec2": ("acceleration", 1609.344), "mi/min2": ("acceleration", 1609.344 / 3600.0), "mi/hr2": ("acceleration", 1609.344 / 12960000.0),
        "nm/hr2": ("acceleration", 1852.0 / 12960000.0), "m/sec2": ("acceleration", 1.0), "m/min2": ("acceleration", 1.0 / 3600.0),
        "m/hr2": ("acceleration", 1.0 / 12960000.0), "km/sec2": ("acceleration", 1000.0), "km/min2": ("acceleration", 1000.0 / 3600.0),
        "km/hr2": ("acceleration", 1000.0 / 12960000.0), "deg/sec2": ("acceleration", 3.141592653589793 / 180.0),
        "deg/min2": ("acceleration", 3.141592653589793 / 180.0 / 3600.0), "deg/hr2": ("acceleration", 3.141592653589793 / 180.0 / 12960000.0),
        "rad/sec2": ("acceleration", 1.0), "rad/min2": ("acceleration", 1.0 / 3600.0), "rad/hr2": ("acceleration", 1.0 / 12960000.0),
        "rev/sec2": ("acceleration", 2.0 * 3.141592653589793), "rev/min2": ("acceleration", 2.0 * 3.141592653589793 / 3600.0), "rev/hr2": ("acceleration", 2.0 * 3.141592653589793 / 12960000.0),
        "g": ("acceleration", 9.80665),
        "lb/sec": ("mass_rate", 0.45359237), "lb/min": ("mass_rate", 0.45359237 / 60.0), "lb/hr": ("mass_rate", 0.45359237 / 3600.0),
        "slugs/sec": ("mass_rate", 14.59390294), "slugs/min": ("mass_rate", 14.59390294 / 60.0), "slugs/hr": ("mass_rate", 14.59390294 / 3600.0),
        "g/sec": ("mass_rate", 0.001), "g/min": ("mass_rate", 0.001 / 60.0), "g/hr": ("mass_rate", 0.001 / 3600.0),
        "kg/sec": ("mass_rate", 1.0), "kg/min": ("mass_rate", 1.0 / 60.0), "kg/hr": ("mass_rate", 1.0 / 3600.0),
        "lbf/ft2": ("pressure", 4.4482216152605 / 0.3048**2), "psi": ("pressure", 6894.757293168), "pascal": ("pressure", 1.0), "kpascal": ("pressure", 1000.0),
        "1/in": ("inverse_length", 1.0 / 0.0254), "1/ft": ("inverse_length", 1.0 / 0.3048), "1/m": ("inverse_length", 1.0),
        "ft2/sec": ("kinematic_viscosity", 0.3048**2), "m2/sec": ("kinematic_viscosity", 1.0),
        "lb/ft3": ("density", 0.45359237 / 0.3048**3), "lb/m3": ("density", 0.45359237), "g/cm3": ("density", 1000.0), "kg/m3": ("density", 1.0),
        "ft2": ("area", 0.3048**2), "m2": ("area", 1.0),
    }
)

_ALIASES = {
    "xecfc": "x", "yecfc": "y", "zecfc": "z",
    "xecfcdt": "xdt", "yecfcdt": "ydt", "zecfcdt": "zdt",
    "latgd": "lat", "gamgd": "gama", "psigd": "psi",
}

_CANONICAL_UNITS = {
    "length": "ft",
    "speed": "ft/sec",
    "time": "sec",
    "angle": "deg",
    "mass": "lb",
    "mass_rate": "lb/sec",
    "acceleration": "ft/sec2",
    "angular_rate": "deg/sec",
    "force": "lbf",
    "pressure": "lbf/ft2",
    "inverse_length": "1/ft",
    "kinematic_viscosity": "ft2/sec",
    "density": "lb/ft3",
    "area": "ft2",
}


def selected_setting(variable: str, settings: Mapping[str, str | None]) -> str | None:
    """Resolve a setting through canonical and historical TAOS aliases."""

    name = variable.casefold().split("[", 1)[0]
    if name in settings:
        return settings[name]
    canonical = _ALIASES.get(name, name)
    if canonical in settings:
        return settings[canonical]
    for alias, canonical in _ALIASES.items():
        if canonical == _ALIASES.get(name, name) and alias in settings:
            return settings[alias]
    return None
####


def variable_dimension(variable: str) -> str | None:
    """Return the standard TAOS dimension for a variable, if known."""

    name = variable.casefold().split("[", 1)[0]
    name = _ALIASES.get(name, name)
    if name in {"x", "y", "z", "alt", "range", "east", "north", "down", "dwnrng", "crsrng", "rcm", "iip_rng", "xtp", "ytp", "ztp"}:
        return "length"
    if name in {"vel", "vair", "xdt", "ydt", "zdt", "ground_speed", "iip_rng_rate"}:
        return "speed"
    if name in {"nx", "ny", "nz", "accel", "gaccel"}:
        return "acceleration"
    if name in {"time", "tseg", "tmark", "iip_time"}:
        return "time"
    if name in {"lat", "long", "lon", "gama", "psi", "alpha", "alphat", "beta", "betae", "pitch", "pitchi", "pitchgd", "yaw", "yawi", "yawgd", "roll", "rolli", "rollgd", "azm", "bankgc", "bankgd"}:
        return "angle"
    if name in {"wt", "mass", "fuel"}:
        return "mass"
    if name in {"thrust", "force"}:
        return "force"
    if name in {"pres", "dynprs", "pressure"}:
        return "pressure"
    if name in {"rho", "density"}:
        return "density"
    if name in {"nu", "kinematic_viscosity"}:
        return "kinematic_viscosity"
    if name in {"sref", "area"}:
        return "area"
    return None
####


def unit_scale(unit: str | None, dimension: str | None) -> float:
    """Return a selected unit's SI scale, validating its dimension."""

    if unit is None or dimension is None:
        return 1.0
    normalized = unit.casefold().replace(" ", "")
    try:
        selected_dimension, scale = _UNIT_TO_SI[normalized]
    except KeyError as error:
        raise ValueError(f"unsupported runtime unit: {unit}") from error
    if dimension == "area" and selected_dimension == "length":
        return scale**2
    if selected_dimension != dimension:
        raise ValueError(f"unit {unit!r} is incompatible with {dimension}")
    return scale
####


def to_internal(value: float, variable: str, settings: Mapping[str, str | None]) -> float:
    """Convert a user value from its selected unit to TAOS canonical units."""

    dimension = variable_dimension(variable)
    unit = selected_setting(variable, settings)
    if unit is None or dimension is None:
        return float(value)
    canonical = _UNIT_TO_SI[_CANONICAL_UNITS[dimension]][1]
    return float(value) * unit_scale(unit, dimension) / canonical
####


def from_internal(value: float, variable: str, settings: Mapping[str, str | None]) -> float:
    """Convert a canonical runtime value to its selected output unit."""

    dimension = variable_dimension(variable)
    unit = selected_setting(variable, settings)
    if unit is None or dimension is None:
        return float(value)
    canonical = _UNIT_TO_SI[_CANONICAL_UNITS[dimension]][1]
    return float(value) * canonical / unit_scale(unit, dimension)
####


def format_number(value: float, output_format: str | None) -> str:
    """Render TAOS ``f.N`` and ``e.N`` output formats."""

    if output_format is None:
        rounded = round(float(value), 12)
        return str(rounded) if abs(float(value) - rounded) <= 1e-12 * max(1.0, abs(float(value))) else str(float(value))
    match = re.fullmatch(r"([feFE])\.(\d+)", output_format.strip())
    if match is None:
        raise ValueError(f"invalid output format: {output_format}")
    kind, digits = match.groups()
    return format(float(value), f".{int(digits)}{kind.lower()}")
####


def resolve_units_and_formats(settings: Mapping[str, tuple[str | Unit | None, str | None]]) -> dict[str, ResolvedFormat]:
    """Resolve unit names and retain output format strings."""

    resolved: dict[str, ResolvedFormat] = {}
    for variable, (unit, output_format) in settings.items():
        resolved[variable] = ResolvedFormat(variable, _resolve_unit(unit), output_format)
    return resolved
####


def convert_value(value: Quantity, target: Unit) -> Quantity:
    """Apply the canonical quantity conversion used by formatted outputs."""

    return value.to(target)
####


def _resolve_unit(unit: str | Unit | None) -> Unit | None:
    if unit is None or isinstance(unit, Unit):
        return unit
    normalized = unit.casefold().replace("^", "")
    aliases = {member.value.casefold(): member for member in Unit}
    aliases.update({"m": Unit.METER, "km": Unit.KILOMETER, "s": Unit.SECOND, "deg": Unit.DEGREE, "rad": Unit.RADIAN})
    try:
        return aliases[normalized]
    except KeyError as error:
        raise ValueError(f"unsupported unit: {unit}") from error
    ####
####
