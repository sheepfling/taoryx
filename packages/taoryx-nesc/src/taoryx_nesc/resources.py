"""Package-resource root for NESC source-replay data and evidence."""

from __future__ import annotations

from importlib.resources import files
from pathlib import Path


def model_resource_root() -> Path:
    """Return packaged NESC assets or the source-checkout fallback."""

    packaged = Path(str(files("taoryx_nesc").joinpath("data")))
    if packaged.is_dir():
        return packaged
    checkout = Path(__file__).resolve().parents[4]
    candidate = checkout / "packages/taoryx-nesc/src/taoryx_nesc/data"
    if candidate.is_dir():
        return candidate
    return packaged
    ####


__all__ = ["model_resource_root"]
