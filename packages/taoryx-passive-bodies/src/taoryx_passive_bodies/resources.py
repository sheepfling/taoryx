"""Package-resource root for passive-body catalog and evidence assets."""

from __future__ import annotations

from importlib.resources import files
from pathlib import Path


def model_resource_root() -> Path:
    """Return packaged passive-body assets or the source-checkout fallback."""

    packaged = Path(str(files("taoryx_passive_bodies").joinpath("data")))
    if packaged.is_dir():
        return packaged
    checkout = Path(__file__).resolve().parents[4]
    candidate = checkout / "packages/taoryx-passive-bodies/src/taoryx_passive_bodies/data"
    if candidate.is_dir():
        return candidate
    return packaged
    ####


__all__ = ["model_resource_root"]
