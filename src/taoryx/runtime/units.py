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
    if name in {"time", "tseg", "tmark", "iip_time"}:
        return "time"
    if name in {"lat", "long", "lon", "gama", "psi", "alpha", "alphat", "beta", "betae", "pitch", "pitchi", "pitchgd", "yaw", "yawi", "yawgd", "roll", "rolli", "rollgd", "azm", "bankgc", "bankgd"}:
        return "angle"
    if name in {"wt", "mass", "fuel"}:
        return "mass"
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
