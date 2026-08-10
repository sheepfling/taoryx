"""Hash-verified DAVE-ML mass-property bindings."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from .daveml_import import DAVEMLFamilyGraphBinding


@dataclass(frozen=True, slots=True)
class DAVEMLInertiaBinding:
    """Evaluate a static DAVE-ML mass-property graph in SI units."""

    graph: DAVEMLFamilyGraphBinding
    mass_output: str = "XMASS"
    cg_output: str = "CG_PCT_MAC"
    inertia_outputs: tuple[str, str, str] = ("XIXX", "XIYY", "XIZZ")
    product_outputs: tuple[str, str, str] = ("XIXY", "XIYZ", "XIZX")
    slug_to_kg: float = 14.59390294
    slug_ft2_to_kg_m2: float = 1.3558179483314004

    def evaluate(self, inputs: Mapping[str, float] | None = None) -> dict[str, float]:
        """Return mass, CG, inertia, and product channels in SI units."""

        identifiers = (self.mass_output, self.cg_output, *self.inertia_outputs, *self.product_outputs)
        values = self.graph.evaluate(inputs or {}, identifiers)
        return {
            "mass_kg": values[self.mass_output] * self.slug_to_kg,
            "cg_percent_mac": values[self.cg_output],
            "inertia_xx_kg_m2": values[self.inertia_outputs[0]] * self.slug_ft2_to_kg_m2,
            "inertia_yy_kg_m2": values[self.inertia_outputs[1]] * self.slug_ft2_to_kg_m2,
            "inertia_zz_kg_m2": values[self.inertia_outputs[2]] * self.slug_ft2_to_kg_m2,
            "product_xy_kg_m2": values[self.product_outputs[0]] * self.slug_ft2_to_kg_m2,
            "product_yz_kg_m2": values[self.product_outputs[1]] * self.slug_ft2_to_kg_m2,
            "product_zx_kg_m2": values[self.product_outputs[2]] * self.slug_ft2_to_kg_m2,
        }

    def as_inertia_matrix(self, inputs: Mapping[str, float] | None = None) -> tuple[tuple[float, ...], ...]:
        """Return the symmetric body inertia matrix in kg m^2."""

        values = self.evaluate(inputs)
        return (
            (values["inertia_xx_kg_m2"], -values["product_xy_kg_m2"], -values["product_zx_kg_m2"]),
            (-values["product_xy_kg_m2"], values["inertia_yy_kg_m2"], -values["product_yz_kg_m2"]),
            (-values["product_zx_kg_m2"], -values["product_yz_kg_m2"], values["inertia_zz_kg_m2"]),
        )


__all__ = ["DAVEMLInertiaBinding"]

