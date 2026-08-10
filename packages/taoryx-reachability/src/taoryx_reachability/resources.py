"""Package-resource root for reachability catalog assets."""

from __future__ import annotations

from importlib.resources import files
from pathlib import Path


def reachability_resource_root() -> Path:
    """Return packaged assets, or the canonical root in a source checkout."""

    packaged = Path(str(files("taoryx_reachability").joinpath("data")))
    if packaged.is_dir() and any(packaged.iterdir()):
        return packaged
    checkout = Path(__file__).resolve().parents[4]
    if (checkout / "pyproject.toml").is_file() and (checkout / "src/taoryx").is_dir():
        return checkout
    return packaged
    ####


__all__ = ["reachability_resource_root"]
####
