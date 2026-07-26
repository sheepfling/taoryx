"""Hash-verified DAVE-ML atmosphere bindings."""

from __future__ import annotations

import hashlib
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from .daveml_evaluator import DAVEMLGraph, load_daveml_graph


@dataclass(frozen=True, slots=True)
class DAVEMLAtmosphereBinding:
    """Evaluate a scalar DAVE-ML atmosphere with declared SI conversions."""

    graph: DAVEMLGraph
    source_path: Path
    source_sha256: str
    altitude_input: str = "alt_ft"
    temperature_output: str = "t_amb_C"
    pressure_ratio_output: str = "p_p0"
    density_ratio_output: str = "sigma"
    speed_of_sound_output: str = "v_sound_fps"
    pressure_output: str = "p_amb_psf"
    sea_level_pressure_pa: float = 101325.0
    sea_level_density_kg_m3: float = 1.225
    feet_to_m: float = 0.3048
    feet_per_second_to_m_per_second: float = 0.3048
    psf_to_pa: float = 47.88025898033584

    def __post_init__(self) -> None:
        if len(self.source_sha256) != 64:
            raise ValueError("DAVE-ML atmosphere source hash must be SHA-256")
        if self.sea_level_pressure_pa <= 0.0 or self.sea_level_density_kg_m3 <= 0.0:
            raise ValueError("DAVE-ML atmosphere reference values must be positive")
        outputs = (
            self.temperature_output,
            self.pressure_ratio_output,
            self.density_ratio_output,
            self.speed_of_sound_output,
            self.pressure_output,
        )
        missing = [name for name in outputs if name not in self.graph.variables]
        if self.altitude_input not in self.graph.variables:
            missing.append(self.altitude_input)
        if missing:
            raise ValueError("DAVE-ML atmosphere channels are unresolved: " + ", ".join(missing))
        ####

    def evaluate(self, geometric_altitude_m: float) -> dict[str, float]:
        """Return source channels and normalized SI atmosphere properties."""

        altitude_ft = float(geometric_altitude_m) / self.feet_to_m
        values = self.graph.evaluate(
            {self.altitude_input: altitude_ft},
            (
                self.temperature_output,
                self.pressure_ratio_output,
                self.density_ratio_output,
                self.speed_of_sound_output,
                self.pressure_output,
            ),
        )
        return {
            "geometric_altitude_m": float(geometric_altitude_m),
            "temperature_k": values[self.temperature_output] + 273.15,
            "pressure_pa": values[self.pressure_output] * self.psf_to_pa,
            "density_kg_m3": values[self.density_ratio_output] * self.sea_level_density_kg_m3,
            "speed_of_sound_m_s": values[self.speed_of_sound_output] * self.feet_per_second_to_m_per_second,
            "temperature_c": values[self.temperature_output],
            "pressure_ratio": values[self.pressure_ratio_output],
            "density_ratio": values[self.density_ratio_output],
            "speed_of_sound_fps": values[self.speed_of_sound_output],
            "pressure_psf": values[self.pressure_output],
        }
        ####

    def as_environment(self, geometric_altitude_m: float) -> Mapping[str, float]:
        """Return normalized channels suitable for a family environment map."""

        return self.evaluate(geometric_altitude_m)
        ####


def load_daveml_atmosphere(path: str | Path, **kwargs: object) -> DAVEMLAtmosphereBinding:
    """Load an atmosphere document and bind it to the standard output contract."""

    source_path = Path(path)
    payload = source_path.read_bytes()
    return DAVEMLAtmosphereBinding(
        graph=load_daveml_graph(payload, document_id=source_path.name),
        source_path=source_path,
        source_sha256=hashlib.sha256(payload).hexdigest(),
        **kwargs,
    )
    ####


__all__ = ["DAVEMLAtmosphereBinding", "load_daveml_atmosphere"]

