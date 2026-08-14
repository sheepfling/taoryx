"""Package-resource lookup for the dual-launch workflow."""

from __future__ import annotations

from importlib.resources import files
from pathlib import Path


def model_resource_root() -> Path:
    """Return package-owned workflow metadata in a wheel or source checkout."""

    packaged = Path(str(files("taoryx_dual_launch").joinpath("data")))
    if packaged.is_dir():
        return packaged
    checkout = Path(__file__).resolve().parents[4]
    candidate = checkout / "packages/taoryx-dual-launch/src/taoryx_dual_launch/data"
    if candidate.is_dir():
        return candidate
    return packaged
    ####


__all__ = ["model_resource_root"]
