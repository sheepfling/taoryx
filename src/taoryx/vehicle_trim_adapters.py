"""Aggregate plug-in-owned trim evidence through a generic core seam.

The core owns report aggregation and declarative trim orchestration.  Each
vehicle plug-in owns the source evaluator and the package-relative trim recipe
needed to solve its work items.  This keeps the host from carrying a hidden
reference-family source binding while retaining one stable evidence API.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Literal

from .trim import TrimResult
from .vehicle_trim_orchestration import TrimWorkItem, orchestrate_trim_recipe

TrimSolveStatus = Literal["verified", "blocked", "failed"]
TrimWorkItemSolver = Callable[[Sequence[TrimWorkItem]], tuple[tuple[str, TrimResult], ...]]

if TYPE_CHECKING:
    from .plugins import PluginCatalog


@dataclass(frozen=True, slots=True)
class TrimSolvePoint:
    """One family-adapter trim result."""

    point_id: str
    success: bool
    state: Mapping[str, float]
    controls: Mapping[str, float]
    residuals: Mapping[str, float]
    max_residual: float
    iterations: int
    solver_status: int
    solver_message: str

    @classmethod
    def from_result(cls, point_id: str, result: TrimResult) -> TrimSolvePoint:
        """Convert the generic solver result into stable scalar JSON values."""

        return cls(
            point_id,
            result.success,
            {str(key): float(value) for key, value in result.state.items()},
            {str(key): float(value) for key, value in result.controls.items()},
            {str(key): float(value) for key, value in result.residuals.items()},
            float(result.max_residual),
            int(result.iterations),
            int(result.status),
            str(result.message),
        )

    def as_dict(self) -> dict[str, object]:
        """Return a stable evidence record."""

        return {
            "point_id": self.point_id,
            "success": self.success,
            "state": dict(self.state),
            "controls": dict(self.controls),
            "residuals": dict(self.residuals),
            "max_residual": self.max_residual,
            "iterations": self.iterations,
            "solver_status": self.solver_status,
            "solver_message": self.solver_message,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class VehicleTrimSolveReport:
    """Aggregate adapter-bound trim evidence."""

    family_id: str
    adapter: str | None
    status: TrimSolveStatus
    claim_boundary: str
    points: tuple[TrimSolvePoint, ...]
    findings: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        """Return a stable machine-readable report."""

        return {
            "schema_version": "taoryx.vehicle-trim-solve/v1",
            "family_id": self.family_id,
            "adapter": self.adapter,
            "status": self.status,
            "claim_boundary": self.claim_boundary,
            "points": [point.as_dict() for point in self.points],
            "findings": list(self.findings),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class VehicleTrimEvidenceBinding:
    """One family-owned source binding available through the shared API."""

    family_id: str
    adapter: str
    resource_root: Path
    trim_recipe_path: Path
    solve_work_items: TrimWorkItemSolver


def _effective_plugin_catalog(plugins: PluginCatalog | None) -> PluginCatalog:
    """Use an explicit focused catalog or discover the available plug-in set."""

    if plugins is not None:
        return plugins
    from .plugins import current_plugin_catalog, discover_plugins

    return current_plugin_catalog() or discover_plugins(strict=False)
    ####


def _load_trim_evidence_binding(
    family_id: str,
    *,
    plugins: PluginCatalog | None,
) -> tuple[VehicleTrimEvidenceBinding | None, str | None]:
    """Resolve one selected family binding through its typed plug-in record."""

    catalog = _effective_plugin_catalog(plugins)
    try:
        binding = catalog.trim_evidence_binding(family_id)
    except KeyError:
        return None, "no selected plug-in registers trim evidence for this source family"
    except Exception as error:  # noqa: BLE001 - the binding is plug-in-owned code
        return None, f"trim-evidence binding could not load: {error}"
    if not isinstance(binding, VehicleTrimEvidenceBinding):
        return None, "the selected plug-in returned an invalid trim-evidence binding contract"
    if binding.family_id != family_id:
        return None, f"selected trim-evidence binding advertised {binding.family_id!r}"
    return binding, None
    ####


def solve_vehicle_trim_evidence(
    family_id: str,
    *,
    plugins: PluginCatalog | None = None,
) -> VehicleTrimSolveReport:
    """Solve one source family through its selected plug-in-owned binding.

    Pass a focused :class:`~taoryx.plugins.PluginCatalog` whenever the caller
    already selected a family package. Without one, the helper discovers
    available plug-ins descriptively and resolves only this family's binding.
    """

    binding, binding_error = _load_trim_evidence_binding(family_id, plugins=plugins)
    if binding is None:
        return VehicleTrimSolveReport(
            family_id,
            None,
            "blocked",
            "the family-owned source trim binding is unavailable",
            (),
            (binding_error or "no family trim adapter is registered",),
        )
    orchestration = orchestrate_trim_recipe(
        family_id,
        binding.trim_recipe_path,
        resource_root=binding.resource_root,
    )
    if orchestration.status == "blocked":
        return VehicleTrimSolveReport(
            family_id,
            binding.adapter,
            "blocked",
            orchestration.claim_boundary,
            (),
            tuple(item.message for item in orchestration.errors),
        )
    items = orchestration.work_items
    points = tuple(
        TrimSolvePoint.from_result(point_id, result)
        for point_id, result in binding.solve_work_items(items)
    )
    failures = tuple(point.point_id for point in points if not point.success)
    status: TrimSolveStatus = "verified" if not failures else "failed"
    findings = (f"solve failed at: {', '.join(failures)}",) if failures else ()
    return VehicleTrimSolveReport(family_id, binding.adapter, status, orchestration.claim_boundary, points, findings)
    ####


def solve_all_vehicle_trim_evidence(
    *,
    plugins: PluginCatalog | None = None,
) -> tuple[VehicleTrimSolveReport, ...]:
    """Solve every trim worklist contributed by the selected plug-in catalog."""

    catalog = _effective_plugin_catalog(plugins)
    return tuple(
        solve_vehicle_trim_evidence(contribution.id, plugins=catalog)
        for contribution in catalog.records("trim_evidence_binding")
    )
    ####


__all__ = [
    "TrimSolvePoint",
    "VehicleTrimEvidenceBinding",
    "VehicleTrimSolveReport",
    "solve_all_vehicle_trim_evidence",
    "solve_vehicle_trim_evidence",
]
####
