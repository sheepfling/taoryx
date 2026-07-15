"""Source-dataset models and deterministic TAORYX table compilation."""

from .compiler import compile_dataset, flatten_rectangular_grid
from .models import DatasetConventions, DatasetManifest, DatasetProvenance, DatasetTableSpec

__all__ = [
    "DatasetConventions",
    "DatasetManifest",
    "DatasetProvenance",
    "DatasetTableSpec",
    "compile_dataset",
    "flatten_rectangular_grid",
]
####
