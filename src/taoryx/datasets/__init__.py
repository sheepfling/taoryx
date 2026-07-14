"""Source-dataset models and deterministic TAORYX table compilation."""

from .compiler import compile_dataset, flatten_rectangular_grid
from .models import DatasetManifest, DatasetTableSpec

__all__ = ["DatasetManifest", "DatasetTableSpec", "compile_dataset", "flatten_rectangular_grid"]
####
