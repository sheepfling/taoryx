"""Package-resource root for HL-20 source data, catalogs, and evidence."""

from __future__ import annotations

from importlib.resources import files
from pathlib import Path


def model_resource_root() -> Path:
    """Return packaged HL-20 assets or the source-checkout fallback."""

    packaged = Path(str(files("taoryx_hl20").joinpath("data")))
    if packaged.is_dir():
        return packaged
    checkout = Path(__file__).resolve().parents[4]
    candidate = checkout / "packages/taoryx-hl20/src/taoryx_hl20/data"
    if candidate.is_dir():
        return candidate
    return packaged
    ####


__all__ = ["model_resource_root"]
