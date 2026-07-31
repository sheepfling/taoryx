"""Explicit bilateral-surface mapping contracts for air-breathing vehicles.

Source aerodynamic decks often expose collective/differential virtual
coordinates while the hardware interface is left/right surface commands. A
sign convention is part of the physical model, not a cosmetic rename. This
module keeps both possible bilateral mappings explicit and makes an unresolved
source convention fail closed before hardware-effector claims are promoted.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class BilateralSurfaceMapping:
    """Map collective/differential virtual commands to left/right surfaces."""

    mapping_id: str
    differential_sign: int
    source: str
    status: str = "hypothesis"

    def __post_init__(self) -> None:
        if not self.mapping_id.strip() or not self.source.strip():
            raise ValueError("bilateral mapping requires an identifier and source")
        if self.differential_sign not in (-1, 1):
            raise ValueError("differential sign must be either -1 or +1")
        if self.status not in {"hypothesis", "selected", "rejected", "unresolved"}:
            raise ValueError(f"unsupported bilateral mapping status {self.status!r}")
        ####
    ####

    def virtual_to_physical(self, collective_deg: float, differential_deg: float) -> tuple[float, float]:
        """Return left/right surface commands for the declared sign hypothesis."""

        differential = self.differential_sign * float(differential_deg)
        return float(collective_deg) + differential, float(collective_deg) - differential
        ####

    def physical_to_virtual(self, left_deg: float, right_deg: float) -> tuple[float, float]:
        """Recover collective/differential virtual coordinates from left/right."""

        collective = 0.5 * (float(left_deg) + float(right_deg))
        differential = self.differential_sign * 0.5 * (float(left_deg) - float(right_deg))
        return collective, differential
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a machine-readable mapping contract."""

        return {
            "mapping_id": self.mapping_id,
            "differential_sign": self.differential_sign,
            "source": self.source,
            "status": self.status,
            "left": "collective + differential_sign * differential",
            "right": "collective - differential_sign * differential",
        }
        ####
    ####


def x8_mapping_hypotheses(source: str = "public_package_control_mapping_status") -> tuple[BilateralSurfaceMapping, ...]:
    """Return both X8 sign conventions for audit and negative-control tests.

    The source-table differential coordinate is resolved separately by
    :func:`x8_source_mapping`.  Keeping both algebraic conventions available
    is intentional: TensorAeroSpace documents the opposite aileron sign, and
    a conversion must remain visible whenever that convention is imported.
    """

    return (
        BilateralSurfaceMapping("x8-left-plus-differential", 1, source),
        BilateralSurfaceMapping("x8-left-minus-differential", -1, source),
    )
    ####


def x8_source_mapping() -> BilateralSurfaceMapping:
    """Return the resolved physical mapping for the checked-in X8 source deck.

    Løw-Hansen et al., CEAS Aeronautical Journal 16 (2025), Eq. (14), defines
    ``delta_a = (delta_el - delta_er) / 2``.  Inverting that equation gives
    ``delta_el = delta_e + delta_a`` and ``delta_er = delta_e - delta_a``.
    The public package calls these virtual coordinates ``collective_elevon``
    and ``differential_elevon``; its differential therefore uses the
    ``left-plus/right-minus`` mapping below.  TensorAeroSpace's alternate
    ``(right-left)/2`` convention is retained as an explicit audit hypothesis,
    not silently mixed into the source deck.
    """

    return BilateralSurfaceMapping(
        mapping_id="x8-low-hansen-2025-eq14-left-plus-differential",
        differential_sign=1,
        source=(
            "Løw-Hansen et al., Modeling and identification of a small fixed-wing UAV "
            "using estimated aerodynamic angles, CEAS Aeronautical Journal 16 (2025), "
            "Eq. (14), doi:10.1007/s13272-025-00816-3"
        ),
        status="selected",
    )
    ####


__all__ = ["BilateralSurfaceMapping", "x8_mapping_hypotheses", "x8_source_mapping"]
