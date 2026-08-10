"""Read-only fallback paths for data owned by optional distributions."""

from __future__ import annotations

from importlib.resources import files
from pathlib import Path, PurePosixPath


def packaged_resource_fallback(
    canonical: Path,
    *,
    package: str,
    resource: str,
) -> Path:
    """Prefer a canonical checkout file, then an installed package resource."""

    if canonical.is_file():
        return canonical
    try:
        candidate = files(package)
    except (ModuleNotFoundError, TypeError):
        return canonical
    for part in PurePosixPath(resource).parts:
        candidate = candidate.joinpath(part)
    packaged = Path(str(candidate))
    return packaged if packaged.is_file() else canonical
    ####


__all__ = ["packaged_resource_fallback"]
####
