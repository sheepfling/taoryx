"""Declarative trim recipes and operating-point worklists.

This module owns the provider-neutral part of trim orchestration: validating
unknowns, residuals, bounds, continuation policy, and operating-point records,
then producing a typed :class:`TrimSpec` work item.  A family adapter still
supplies the evaluator that maps the spec into source equations.  The generic
layer never invents force, moment, propulsion, or actuator behavior.
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, cast

import yaml

from .trim import TrimSpec
from .vehicle_registry import ROOT

TrimVariableRole = Literal["state", "control"]
RecipeStatus = Literal["ready_for_adapter", "blocked"]


@dataclass(frozen=True, slots=True)
class TrimRecipeFinding:
    """One actionable trim-recipe or operating-point finding."""

    severity: Literal["error", "warning"]
    code: str
    path: str
    message: str
    hint: str

    def as_dict(self) -> dict[str, str]:
        """Return a stable JSON representation."""

        return {
            "severity": self.severity,
            "code": self.code,
            "path": self.path,
            "message": self.message,
            "hint": self.hint,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class TrimRecipeVariable:
    """One bounded state or control unknown in a trim recipe."""

    name: str
    role: TrimVariableRole
    unit: str
    initial: float
    lower: float
    upper: float
    point_path: str | None = None
    point_scale: float = 1.0


@dataclass(frozen=True, slots=True)
class TrimRecipeResidual:
    """One physical residual contract."""

    name: str
    unit: str
    scale: float
    tolerance: float


@dataclass(frozen=True, slots=True)
class TrimRecipe:
    """Validated provider-neutral trim recipe."""

    family_id: str
    recipe_id: str
    adapter: str
    claim_boundary: str
    variables: tuple[TrimRecipeVariable, ...]
    residuals: tuple[TrimRecipeResidual, ...]
    fixed_inputs: Mapping[str, float | str]
    solver: Mapping[str, object]
    continuation: Mapping[str, object]
    operating_point_catalog: str
    source_evidence: str

    @property
    def state_variables(self) -> tuple[TrimRecipeVariable, ...]:
        """Return state unknowns in declared order."""

        return tuple(variable for variable in self.variables if variable.role == "state")
        ####

    @property
    def control_variables(self) -> tuple[TrimRecipeVariable, ...]:
        """Return control unknowns in declared order."""

        return tuple(variable for variable in self.variables if variable.role == "control")
        ####


@dataclass(frozen=True, slots=True)
class TrimWorkItem:
    """One operating point ready to be handed to a family evaluator."""

    point_id: str
    recipe_id: str
    adapter: str
    source_evidence: str
    fixed_inputs: Mapping[str, float | str]
    trim_spec: TrimSpec
    execution_status: str

    def as_dict(self) -> dict[str, object]:
        """Return the serializable work-item contract."""

        return {
            "point_id": self.point_id,
            "recipe_id": self.recipe_id,
            "adapter": self.adapter,
            "source_evidence": self.source_evidence,
            "fixed_inputs": dict(self.fixed_inputs),
            "execution_status": self.execution_status,
            "trim_spec": {
                "state_names": list(self.trim_spec.state_names),
                "control_names": list(self.trim_spec.control_names),
                "residual_names": list(self.trim_spec.residual_names),
                "state_initial": dict(self.trim_spec.state_initial),
                "control_initial": dict(self.trim_spec.control_initial),
                "state_lower": dict(self.trim_spec.state_lower or {}),
                "state_upper": dict(self.trim_spec.state_upper or {}),
                "control_lower": dict(self.trim_spec.control_lower or {}),
                "control_upper": dict(self.trim_spec.control_upper or {}),
                "residual_scales": dict(self.trim_spec.residual_scales or {}),
                "operating_point": dict(self.trim_spec.operating_point),
            },
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class TrimOrchestrationReport:
    """Recipe validation and generated worklist report."""

    family_id: str
    recipe_id: str | None
    status: RecipeStatus
    claim_boundary: str
    findings: tuple[TrimRecipeFinding, ...]
    work_items: tuple[TrimWorkItem, ...]
    catalog_path: str | None

    @property
    def errors(self) -> tuple[TrimRecipeFinding, ...]:
        """Return blocking recipe findings."""

        return tuple(item for item in self.findings if item.severity == "error")
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a stable machine-readable report."""

        return {
            "schema_version": "taoryx.vehicle-trim-orchestration/v1",
            "family_id": self.family_id,
            "recipe_id": self.recipe_id,
            "status": self.status,
            "claim_boundary": self.claim_boundary,
            "catalog_path": self.catalog_path,
            "error_count": len(self.errors),
            "findings": [item.as_dict() for item in self.findings],
            "work_items": [item.as_dict() for item in self.work_items],
        }
        ####
    ####


def _finding(
    findings: list[TrimRecipeFinding],
    severity: Literal["error", "warning"],
    code: str,
    path: str,
    message: str,
    hint: str,
) -> None:
    findings.append(TrimRecipeFinding(severity, code, path, message, hint))
    ####


def _read_yaml(path: Path) -> Mapping[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{path} must contain a mapping")
    return payload
    ####


def _read_json(path: Path) -> Mapping[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{path} must contain a mapping")
    return payload
    ####


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)
    ####


def _number(value: object, path: str, findings: list[TrimRecipeFinding]) -> float | None:
    try:
        numeric = float(cast(Any, value))
    except (TypeError, ValueError):
        _finding(findings, "error", "non-numeric-value", path, f"value {value!r} is not numeric", "Use a finite scalar in the declared unit.")
        return None
    if not math.isfinite(numeric):
        _finding(findings, "error", "nonfinite-value", path, "value is not finite", "Replace NaN or infinity with a source-backed finite value.")
        return None
    return numeric
    ####


def _path_value(value: Mapping[str, Any], path: str) -> object | None:
    current: object = value
    for part in path.split("."):
        if not isinstance(current, Mapping) or part not in current:
            return None
        current = current[part]
    return current
    ####


def _variable(raw: object, path: str, findings: list[TrimRecipeFinding]) -> TrimRecipeVariable | None:
    if not isinstance(raw, Mapping):
        _finding(findings, "error", "variable-not-a-mapping", path, "trim variable must be a mapping", "Declare name, role, unit, initial, and bounds.")
        return None
    name = str(raw.get("name", ""))
    role = str(raw.get("role", ""))
    unit = str(raw.get("unit", ""))
    if not name or role not in {"state", "control"} or not unit:
        _finding(findings, "error", "variable-contract-incomplete", path, "trim variable requires name, state/control role, and unit", "Complete the variable declaration before solving.")
        return None
    initial = _number(raw.get("initial"), f"{path}.initial", findings)
    bounds = raw.get("bounds")
    if not isinstance(bounds, Sequence) or isinstance(bounds, (str, bytes)) or len(bounds) != 2:
        _finding(findings, "error", "variable-bounds-incomplete", f"{path}.bounds", "trim variable bounds must contain exactly [lower, upper]", "Declare finite lower and upper bounds in the variable's unit.")
        return None
    lower = _number(bounds[0], f"{path}.bounds[0]", findings)
    upper = _number(bounds[1], f"{path}.bounds[1]", findings)
    if initial is None or lower is None or upper is None:
        return None
    if lower > upper or not lower <= initial <= upper:
        _finding(findings, "error", "variable-bounds-invalid", path, f"bounds do not contain the initial value: [{lower}, {upper}], initial={initial}", "Correct the source-backed bounds or initial guess.")
    scale = _number(raw.get("point_scale", 1.0), f"{path}.point_scale", findings)
    if scale is None:
        scale = 1.0
    return TrimRecipeVariable(name, cast(TrimVariableRole, role), unit, initial, lower, upper, str(raw["point_path"]) if raw.get("point_path") else None, scale)
    ####


def _residual(raw: object, path: str, findings: list[TrimRecipeFinding]) -> TrimRecipeResidual | None:
    if not isinstance(raw, Mapping):
        _finding(findings, "error", "residual-not-a-mapping", path, "trim residual must be a mapping", "Declare name, unit, scale, and tolerance.")
        return None
    name = str(raw.get("name", ""))
    unit = str(raw.get("unit", ""))
    scale = _number(raw.get("scale"), f"{path}.scale", findings)
    tolerance = _number(raw.get("tolerance"), f"{path}.tolerance", findings)
    if not name or not unit or scale is None or tolerance is None:
        _finding(findings, "error", "residual-contract-incomplete", path, "trim residual requires name, unit, positive scale, and tolerance", "Complete the physical residual contract.")
        return None
    if scale <= 0.0 or tolerance < 0.0:
        _finding(findings, "error", "residual-contract-invalid", path, "residual scale must be positive and tolerance nonnegative", "Use a characteristic physical residual scale and a nonnegative gate.")
    return TrimRecipeResidual(name, unit, scale, tolerance)
    ####


def load_trim_recipe(path: str | Path) -> tuple[TrimRecipe | None, tuple[TrimRecipeFinding, ...]]:
    """Load and structurally validate one declarative trim recipe."""

    source = Path(path)
    findings: list[TrimRecipeFinding] = []
    try:
        raw = _read_yaml(source)
    except (OSError, ValueError, yaml.YAMLError) as error:
        _finding(findings, "error", "recipe-load-failed", _relative(source), str(error), "Restore the recipe and rerun the integration pipeline.")
        return None, tuple(findings)
    if raw.get("kind") != "taoryx.trim-recipe/v1alpha1":
        _finding(findings, "error", "recipe-kind-invalid", _relative(source), "recipe kind is not taoryx.trim-recipe/v1alpha1", "Use the provider-neutral trim recipe schema.")
    family_id = str(raw.get("family", ""))
    recipe_id = str(raw.get("recipe_id", ""))
    adapter = str(raw.get("adapter", ""))
    claim_boundary = str(raw.get("claim_boundary", ""))
    if not family_id or not recipe_id or not adapter or not claim_boundary:
        _finding(findings, "error", "recipe-identity-incomplete", _relative(source), "recipe requires family, recipe_id, adapter, and claim_boundary", "Complete the recipe identity and claim boundary.")
    raw_variables = raw.get("variables")
    variables = tuple(variable for index, item in enumerate(raw_variables if isinstance(raw_variables, list) else ()) if (variable := _variable(item, f"{_relative(source)}:variables[{index}]", findings)) is not None)
    raw_residuals = raw.get("residuals")
    residuals = tuple(residual for index, item in enumerate(raw_residuals if isinstance(raw_residuals, list) else ()) if (residual := _residual(item, f"{_relative(source)}:residuals[{index}]", findings)) is not None)
    if not variables:
        _finding(findings, "error", "recipe-variables-missing", _relative(source), "trim recipe has no valid variables", "Declare at least one state or control unknown unless the adapter explicitly supports a fixed-point evaluator.")
    if not residuals:
        _finding(findings, "error", "recipe-residuals-missing", _relative(source), "trim recipe has no valid residuals", "Declare the physical force, acceleration, or moment residuals defining equilibrium.")
    names = [variable.name for variable in variables]
    if len(names) != len(set(names)):
        _finding(findings, "error", "duplicate-variable-name", _relative(source), "trim variable names must be unique", "Use one stable name per state/control coordinate.")
    residual_names = [residual.name for residual in residuals]
    if len(residual_names) != len(set(residual_names)):
        _finding(findings, "error", "duplicate-residual-name", _relative(source), "trim residual names must be unique", "Use one stable name per physical residual equation.")
    catalog = str(raw.get("operating_point_catalog", ""))
    source_evidence = str(raw.get("source_evidence", ""))
    if not catalog:
        _finding(findings, "error", "operating-point-catalog-missing", _relative(source), "trim recipe does not name an operating-point catalog", "Declare the relative qualification/operating-points.yaml path.")
    if not source_evidence:
        _finding(findings, "warning", "source-evidence-missing", _relative(source), "trim recipe does not name source evidence", "Add the evidence artifact used to seed or validate the recipe.")
    elif not (ROOT / source_evidence).is_file():
        _finding(findings, "error", "source-evidence-not-found", source_evidence, "trim recipe source evidence does not exist", "Generate or correct the pinned evidence artifact before building a worklist.")
    solver_raw = raw.get("solver")
    continuation_raw = raw.get("continuation")
    solver: dict[str, object] = {str(key): value for key, value in solver_raw.items()} if isinstance(solver_raw, Mapping) else {}
    continuation: dict[str, object] = {str(key): value for key, value in continuation_raw.items()} if isinstance(continuation_raw, Mapping) else {}
    if not solver.get("method"):
        _finding(findings, "error", "solver-method-missing", _relative(source), "trim recipe does not declare a solver method", "Declare the generic solver policy; the family adapter still owns evaluation.")
    if not continuation.get("strategy"):
        _finding(findings, "warning", "continuation-strategy-missing", _relative(source), "trim recipe has no continuation strategy", "Declare how neighboring operating points are ordered and seeded.")
    if any(item.severity == "error" for item in findings):
        return None, tuple(findings)
    fixed_inputs_raw = raw.get("fixed_inputs")
    fixed_inputs = (
        {str(key): value for key, value in fixed_inputs_raw.items() if isinstance(value, (str, int, float))}
        if isinstance(fixed_inputs_raw, Mapping)
        else {}
    )
    return TrimRecipe(family_id, recipe_id, adapter, claim_boundary, variables, residuals, fixed_inputs, solver, continuation, catalog, source_evidence), tuple(findings)
    ####


def _load_catalog(recipe_path: Path, recipe: TrimRecipe, findings: list[TrimRecipeFinding]) -> tuple[Path, Mapping[str, Any] | None]:
    catalog_path = recipe_path.parent / recipe.operating_point_catalog
    if not catalog_path.is_file():
        _finding(findings, "error", "operating-point-catalog-missing", _relative(catalog_path), "operating-point catalog does not exist", "Create the source-backed catalog before generating trim work items.")
        return catalog_path, None
    try:
        catalog = _read_yaml(catalog_path)
    except (OSError, ValueError, yaml.YAMLError) as error:
        _finding(findings, "error", "operating-point-catalog-load-failed", _relative(catalog_path), str(error), "Repair the catalog YAML and rerun validation.")
        return catalog_path, None
    if catalog.get("kind") != "taoryx.operating-point-catalog/v1alpha1" or str(catalog.get("family")) != recipe.family_id:
        _finding(findings, "error", "operating-point-catalog-identity-mismatch", _relative(catalog_path), "catalog kind or family does not match the trim recipe", "Regenerate the catalog from the same family recipe.")
    points = catalog.get("points")
    if not isinstance(points, list) or not points:
        _finding(findings, "error", "operating-point-catalog-empty", _relative(catalog_path), "operating-point catalog has no points", "Declare at least one source-backed operating point.")
    return catalog_path, catalog
    ####


def _build_spec(recipe: TrimRecipe, point: Mapping[str, Any], point_id: str) -> TrimSpec:
    state_initial: dict[str, float] = {}
    control_initial: dict[str, float] = {}
    state_lower: dict[str, float] = {}
    state_upper: dict[str, float] = {}
    control_lower: dict[str, float] = {}
    control_upper: dict[str, float] = {}
    for variable in recipe.variables:
        initial = variable.initial
        if variable.point_path:
            point_value = _path_value(point, variable.point_path)
            if point_value is not None:
                initial = float(cast(Any, point_value)) * variable.point_scale
        if variable.role == "state":
            state_initial[variable.name] = initial
            state_lower[variable.name] = variable.lower
            state_upper[variable.name] = variable.upper
        else:
            control_initial[variable.name] = initial
            control_lower[variable.name] = variable.lower
            control_upper[variable.name] = variable.upper
    environment_raw = point.get("environment")
    environment: Mapping[str, Any] = environment_raw if isinstance(environment_raw, Mapping) else {}
    operating_point = {str(key): value for key, value in environment.items() if isinstance(value, (str, int, float))}
    operating_point["point_id"] = point_id
    return TrimSpec(
        state_names=tuple(variable.name for variable in recipe.state_variables),
        control_names=tuple(variable.name for variable in recipe.control_variables),
        residual_names=tuple(residual.name for residual in recipe.residuals),
        state_initial=state_initial,
        control_initial=control_initial,
        state_lower=state_lower,
        state_upper=state_upper,
        control_lower=control_lower,
        control_upper=control_upper,
        residual_scales={residual.name: residual.scale for residual in recipe.residuals},
        operating_point=operating_point,
    )
    ####


def orchestrate_trim_recipe(family_id: str, recipe_path: str | Path | None = None) -> TrimOrchestrationReport:
    """Validate a family recipe and produce adapter-ready trim work items."""

    path = Path(recipe_path) if recipe_path is not None else ROOT / "families" / family_id / "qualification/trim-recipe.yaml"
    recipe, recipe_findings = load_trim_recipe(path)
    findings = list(recipe_findings)
    if recipe is None:
        return TrimOrchestrationReport(family_id, None, "blocked", "trim recipe is invalid; no solver is invoked", tuple(findings), (), None)
    if recipe.family_id != family_id:
        _finding(findings, "error", "recipe-family-mismatch", _relative(path), f"recipe family {recipe.family_id!r} does not match requested family {family_id!r}", "Regenerate the recipe for the selected family.")
    catalog_path, catalog = _load_catalog(path, recipe, findings)
    work_items: list[TrimWorkItem] = []
    if catalog is not None:
        points = catalog.get("points")
        if isinstance(points, list):
            seen: set[str] = set()
            for index, raw_point in enumerate(points):
                point_path = f"{_relative(catalog_path)}:points[{index}]"
                if not isinstance(raw_point, Mapping):
                    _finding(findings, "error", "operating-point-not-a-mapping", point_path, "operating point must be a mapping", "Declare environment, state, controls, residuals, and validity.")
                    continue
                point_id = str(raw_point.get("id", ""))
                if not point_id or point_id in seen:
                    _finding(findings, "error", "operating-point-id-invalid", point_path, f"operating-point ID is missing or duplicated: {point_id!r}", "Give every point a unique stable ID.")
                    continue
                seen.add(point_id)
                for required in ("environment", "target", "state", "controls", "validity", "next_use"):
                    if required not in raw_point:
                        _finding(findings, "error", "operating-point-field-missing", f"{point_path}.{required}", f"operating point lacks {required!r}", "Keep the full source-backed point contract; do not infer missing state.")
                if "residuals" not in raw_point:
                    _finding(findings, "warning", "operating-point-residuals-pending", f"{point_path}.residuals", "operating point has no independently recorded residual values", "Run the family adapter and write residual evidence before promoting the point beyond a worklist.")
                try:
                    spec = _build_spec(recipe, raw_point, point_id)
                except (TypeError, ValueError) as error:
                    _finding(findings, "error", "trim-spec-build-failed", point_path, str(error), "Make point values compatible with the recipe variable mappings.")
                    continue
                work_items.append(TrimWorkItem(point_id, recipe.recipe_id, recipe.adapter, recipe.source_evidence, recipe.fixed_inputs, spec, "ready_for_family_evaluator"))
    if any(item.severity == "error" for item in findings):
        status: RecipeStatus = "blocked"
        work_items = []
    else:
        status = "ready_for_adapter"
    return TrimOrchestrationReport(family_id, recipe.recipe_id, status, recipe.claim_boundary, tuple(findings), tuple(work_items), _relative(catalog_path))
    ####


__all__ = [
    "TrimOrchestrationReport",
    "TrimRecipe",
    "TrimRecipeFinding",
    "TrimRecipeResidual",
    "TrimRecipeVariable",
    "TrimWorkItem",
    "load_trim_recipe",
    "orchestrate_trim_recipe",
]
####
