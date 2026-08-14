"""Package-resource root for F-16 S.119 source data and evidence."""

from __future__ import annotations

from importlib.resources import files
from pathlib import Path


def model_resource_root() -> Path:
    """Return packaged F-16 assets or the source-checkout fallback."""

    packaged = Path(str(files("taoryx_f16").joinpath("data")))
    if packaged.is_dir():
        return packaged
    checkout = Path(__file__).resolve().parents[4]
    candidate = checkout / "packages/taoryx-f16/src/taoryx_f16/data"
    if candidate.is_dir():
        return candidate
    return packaged
    ####


__all__ = ["model_resource_root"]
