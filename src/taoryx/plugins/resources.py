"""Read-only fallback paths for data owned by optional distributions."""

from __future__ import annotations

from collections.abc import Sequence
from importlib.resources import files
from pathlib import Path, PurePosixPath


def packaged_resource(*, package: str, resource: str) -> Path | None:
    """Resolve an existing resource owned by one optional distribution.

    Unlike :func:`packaged_resource_fallback`, this helper never treats a
    repository checkout path as an answer.  Family-catalog resolution uses it
    when a partial source checkout intentionally omits one of the sibling
    vehicle packages and must behave like an installed wheel set.
    """

    try:
        candidate = files(package)
    except (ModuleNotFoundError, TypeError):
        return None
    for part in PurePosixPath(resource).parts:
        candidate = candidate.joinpath(part)
    packaged = Path(str(candidate))
    return packaged if packaged.is_file() else None
    ####


def packaged_resource_fallback(
    canonical: Path,
    *,
    package: str,
    resource: str,
) -> Path:
    """Prefer a canonical checkout file, then an installed package resource."""

    if canonical.is_file():
        return canonical
    return packaged_resource(package=package, resource=resource) or canonical
    ####


def packaged_resource_fallbacks(
    canonical: Path,
    *,
    candidates: Sequence[tuple[str, str]],
) -> Path:
    """Resolve one optional resource from the first installed owner.

    Vehicle catalog fragments remain package-owned during the migration away
    from the reference aggregate.  The canonical checkout file remains the
    source of truth when present; an installed host instead selects the first
    distribution that supplies the exact resource.  Callers must request one
    full, self-consistent fragment rather than attempting to merge YAML at a
    generic resource boundary.
    """

    if canonical.is_file():
        return canonical
    for package, resource in candidates:
        candidate = packaged_resource_fallback(canonical, package=package, resource=resource)
        if candidate.is_file():
            return candidate
    return canonical
    ####


__all__ = ["packaged_resource", "packaged_resource_fallback", "packaged_resource_fallbacks"]
####
