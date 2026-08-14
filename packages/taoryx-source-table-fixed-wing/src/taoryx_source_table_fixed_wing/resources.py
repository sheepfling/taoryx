"""Package-resource root for source-table X8 and B747 assets."""

from __future__ import annotations

from importlib.resources import files
from pathlib import Path


def model_resource_root() -> Path:
    """Return packaged source-table assets or the source-checkout fallback."""

    packaged = Path(str(files("taoryx_source_table_fixed_wing").joinpath("data")))
    if packaged.is_dir():
        return packaged
    checkout = Path(__file__).resolve().parents[4]
    candidate = checkout / "packages/taoryx-source-table-fixed-wing/src/taoryx_source_table_fixed_wing/data"
    if candidate.is_dir():
        return candidate
    return packaged
    ####


__all__ = ["model_resource_root"]
