"""Package-resource root for Simple Aero model and fixture assets."""

from __future__ import annotations

from importlib.resources import files
from pathlib import Path


def simple_aero_resource_root() -> Path:
    """Return packaged assets, or the canonical root in a source checkout."""

    packaged = Path(str(files("taoryx_simple_aero").joinpath("data")))
    if packaged.is_dir():
        return packaged
    checkout = Path(__file__).resolve().parents[4]
    if (checkout / "pyproject.toml").is_file() and (checkout / "src/taoryx").is_dir():
        return checkout
    return packaged
    ####


__all__ = ["simple_aero_resource_root"]
####
