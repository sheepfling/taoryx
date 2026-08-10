"""Package-resource root for reference-model source and evidence assets."""

from __future__ import annotations

from importlib.resources import files
from pathlib import Path


def model_resource_root() -> Path:
    """Return packaged assets, or the canonical root in a source checkout."""

    packaged = Path(str(files("taoryx_reference_models").joinpath("data")))
    if packaged.is_dir():
        return packaged
    checkout = Path(__file__).resolve().parents[4]
    if (checkout / "pyproject.toml").is_file() and (checkout / "src/taoryx").is_dir():
        return checkout
    return packaged
    ####


__all__ = ["model_resource_root"]
####
