"""Units/format resolution at the parser/runtime boundary."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from taoryx.contracts import Quantity, Unit


@dataclass(frozen=True, slots=True)
class ResolvedFormat:
    variable: str
    unit: Unit | None
    format: str | None


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
