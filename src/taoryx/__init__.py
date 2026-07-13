"""Python successor to the TAOS 1995 simulation system."""

__version__ = "0.1.0.dev0"

from . import (
    aerodynamics,
    atmosphere,
    attitude,
    contracts,
    coordinates,
    earth,
    equations,
    forces,
    geodesy,
    gravity,
    guidance,
    language,
    linalg,
    numeric,
    radar,
    searches,
    simulation,
    state_rates,
    tables,
)

__all__ = ["__version__", "aerodynamics", "atmosphere", "attitude", "contracts", "coordinates", "earth", "equations", "forces", "geodesy", "gravity", "guidance", "language", "linalg", "numeric", "radar", "searches", "simulation", "state_rates", "tables"]
