"""Package-resource root for Hummingbird source and evidence assets."""

from __future__ import annotations

from importlib.resources import files
from pathlib import Path


def model_resource_root() -> Path:
    """Return packaged Hummingbird assets or the source checkout fallback."""

    packaged = Path(str(files("taoryx_hummingbird").joinpath("data")))
    if packaged.is_dir():
        return packaged
    checkout = Path(__file__).resolve().parents[4]
    candidate = checkout / "packages/taoryx-hummingbird/src/taoryx_hummingbird/data"
    if candidate.is_dir():
        return candidate
    return packaged
    ####


__all__ = ["model_resource_root"]
