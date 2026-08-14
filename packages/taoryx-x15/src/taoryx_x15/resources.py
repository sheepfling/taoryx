"""Package-resource root for X-15 source data, catalogs, and evidence."""

from __future__ import annotations

from importlib.resources import files
from pathlib import Path


def model_resource_root() -> Path:
    """Return packaged X-15 assets or the source-checkout fallback."""

    packaged = Path(str(files("taoryx_x15").joinpath("data")))
    if packaged.is_dir():
        return packaged
    checkout = Path(__file__).resolve().parents[4]
    candidate = checkout / "packages/taoryx-x15/src/taoryx_x15/data"
    if candidate.is_dir():
        return candidate
    return packaged
    ####


__all__ = ["model_resource_root"]
