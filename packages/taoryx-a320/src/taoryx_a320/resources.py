"""Package-resource root for the A320 OpenAP family assets and evidence."""

from __future__ import annotations

from importlib.resources import files
from pathlib import Path


def model_resource_root() -> Path:
    """Return packaged A320 assets or the source-checkout fallback."""

    packaged = Path(str(files("taoryx_a320").joinpath("data")))
    if packaged.is_dir():
        return packaged
    checkout = Path(__file__).resolve().parents[4]
    candidate = checkout / "packages/taoryx-a320/src/taoryx_a320/data"
    if candidate.is_dir():
        return candidate
    return packaged
    ####


__all__ = ["model_resource_root"]
