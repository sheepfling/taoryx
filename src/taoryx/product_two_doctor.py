"""Ordered setup diagnostics for Product 2 catalog scenarios."""

from __future__ import annotations

import hashlib
from collections.abc import Mapping
from pathlib import Path

import yaml

from taoryx.product_two_catalog import ROOT, ProductTwoScenario
from taoryx.product_two_contracts import ProductTwoStatus
from taoryx.scenario import ScenarioCompileError, ScenarioCompiler
from taoryx.table_explorer import inspect_table_file
from taoryx.vehicle_composition import (
    VehicleCompositionError,
    compile_vehicle_composition,
    load_vehicle_composition_request,
    resolve_vehicle_composition_interface_contract,
)
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog
from taoryx.vehicle_execution_preflight import preflight_vehicle_composition
from taoryx.vehicle_interface import validate_vehicle_interface_contract
from taoryx.vehicle_runtime_lowering import lower_vehicle_composition


def doctor_scenario(scenario: ProductTwoScenario, *, root: Path = ROOT) -> dict[str, object]:
    """Run the M1 validation, inspection, compilation, lowering, and capability ladder."""

    checks: list[dict[str, object]] = []
    diagnostics: list[dict[str, object]] = []
    source_inputs: list[dict[str, object]] = []
    context: dict[str, object] = {}

    input_check = _check_inputs(scenario, root, source_inputs, diagnostics)
    checks.append(input_check)
    if input_check["status"] == "blocked":
        return _doctor_result(scenario, checks, diagnostics, source_inputs, context)

    if scenario.entrypoint in {"source", "interactive"}:
        _doctor_source(scenario, root, checks, diagnostics, context)
    elif scenario.entrypoint == "composition":
        _doctor_composition(scenario, root, checks, diagnostics, context)
    else:
        _doctor_showcase(scenario, root, checks, diagnostics, context)
    return _doctor_result(scenario, checks, diagnostics, source_inputs, context)
    ####


def _doctor_source(
    scenario: ProductTwoScenario,
    root: Path,
    checks: list[dict[str, object]],
    diagnostics: list[dict[str, object]],
    context: dict[str, object],
) -> None:
    problem = _input_path(scenario, "problem", root)
    tables = tuple(scenario.input_path(item, root=root) for item in scenario.inputs if item.kind == "table")
    for table in tables:
        try:
            inspection = inspect_table_file(table)
            checks.append(
                _check(
                    "table-inspection",
                    "passed",
                    f"{table.relative_to(root)}: {len(inspection.tables)} table(s), bounds and names available",
                )
            )
        except (OSError, KeyError, TypeError, ValueError) as error:
            _failure(
                checks,
                diagnostics,
                "table-inspection",
                "failed",
                f"could not inspect {table.relative_to(root)}: {error}",
                code="table-inspection-failed",
                location=str(table.relative_to(root)),
                remediation="Run `taoryx table inspect <path> --json` and correct the table syntax or bindings.",
                blocking=True,
            )
    try:
        resolved = ScenarioCompiler().compile(
            problem,
            table_paths=tables,
            profile="taos96" if scenario.id == "two-stage-ballistic" else "taoryx",
            integrator=scenario.integrator,
        )
        context["scenario_identity"] = resolved.identity
        context["table_names"] = list(resolved.table_names)
        checks.append(_check("compilation", "passed", f"resolved source identity {resolved.identity}"))
    except ScenarioCompileError as error:
        detail = "; ".join(f"{item.code}: {item.message}" for item in error.diagnostics)
        _failure(
            checks,
            diagnostics,
            "compilation",
            "blocked",
            detail or "source compilation failed",
            code="source-compilation-failed",
            location=str(problem.relative_to(root)),
            remediation="Run `taoryx scenario compile` with the catalog inputs and resolve the reported source diagnostics.",
            blocking=True,
        )
        return
    except (OSError, TypeError, ValueError) as error:
        _failure(
            checks,
            diagnostics,
            "compilation",
            "failed",
            str(error),
            code="source-compilation-error",
            location=str(problem.relative_to(root)),
            remediation="Check the source path, grammar profile, table paths, and integrator configuration.",
            blocking=True,
        )
        return
    try:
        lowered = resolved.lower()
        checks.append(_check("resolution", "passed", f"resolved {len(lowered.cases)} executable case(s)"))
        checks.append(_check("lowering", "passed", "source lowered to the runtime execution boundary"))
    except (OSError, TypeError, ValueError) as error:
        _failure(
            checks,
            diagnostics,
            "lowering",
            "failed",
            str(error),
            code="source-lowering-failed",
            location=str(problem.relative_to(root)),
            remediation="Inspect the parser and table-binding diagnostics before attempting a runtime execution.",
            blocking=True,
        )
    checks.append(_check("capability", "not_applicable", "source runtime owns the execution path; no composition capability adapter is required"))
    ####


def _doctor_composition(
    scenario: ProductTwoScenario,
    root: Path,
    checks: list[dict[str, object]],
    diagnostics: list[dict[str, object]],
    context: dict[str, object],
) -> None:
    request_path = _input_path(scenario, "composition", root)
    try:
        catalog = load_resolved_vehicle_composition_catalog()
        request = load_vehicle_composition_request(request_path)
        composition = compile_vehicle_composition(request, catalog=catalog)
        context["composition_identity_sha256"] = composition.identity_sha256
        context["composition_id"] = composition.id
        checks.append(_check("compilation", "passed", f"compiled composition {composition.id}"))
    except (OSError, KeyError, TypeError, ValueError, VehicleCompositionError) as error:
        _failure(
            checks,
            diagnostics,
            "compilation",
            "blocked",
            str(error),
            code="composition-compilation-failed",
            location=str(request_path.relative_to(root)),
            remediation="Run `taoryx vehicle compose` and correct the request, vehicle, fidelity, initialization, or mission values.",
            blocking=True,
        )
        return
    try:
        interface = resolve_vehicle_composition_interface_contract(composition, catalog=catalog)
        findings = validate_vehicle_interface_contract(interface)
        if findings:
            _failure(
                checks,
                diagnostics,
                "resolution",
                "blocked",
                "; ".join(findings),
                code="composition-interface-invalid",
                location=str(request_path.relative_to(root)),
                remediation="Run `taoryx vehicle interface-composition` and repair the declared interface contract.",
                blocking=True,
            )
        else:
            checks.append(_check("resolution", "passed", f"resolved interface {interface.id} ({interface.fingerprint})"))
    except (OSError, KeyError, TypeError, ValueError, VehicleCompositionError) as error:
        _failure(
            checks,
            diagnostics,
            "resolution",
            "failed",
            str(error),
            code="composition-resolution-failed",
            location=str(request_path.relative_to(root)),
            remediation="Run `taoryx vehicle interface-composition` for the compiled request and inspect its contract diagnostics.",
            blocking=True,
        )
    try:
        lowering = lower_vehicle_composition(composition)
        if lowering.status == "blocked":
            _failure(
                checks,
                diagnostics,
                "lowering",
                "blocked",
                str(lowering.as_dict()["claim_boundary"]),
                code="composition-lowering-blocked",
                location=str(request_path.relative_to(root)),
                remediation="Run `taoryx vehicle lower` and follow the declared adapter or factory blocker.",
                blocking=True,
            )
        else:
            checks.append(_check("lowering", "passed", f"lowering status {lowering.status}"))
    except (OSError, KeyError, TypeError, ValueError, VehicleCompositionError) as error:
        _failure(
            checks,
            diagnostics,
            "lowering",
            "failed",
            str(error),
            code="composition-lowering-failed",
            location=str(request_path.relative_to(root)),
            remediation="Run `taoryx vehicle lower` and inspect the selected execution binding.",
            blocking=True,
        )
    try:
        preflight = preflight_vehicle_composition(composition)
        if preflight.status == "translation_ready":
            checks.append(_check("capability", "passed", f"translator ready: {preflight.translator_id}"))
        elif preflight.status == "not_applicable":
            checks.append(_check("capability", "not_applicable", "catalog declares no semantic execution translator"))
        else:
            _failure(
                checks,
                diagnostics,
                "capability",
                "blocked",
                "; ".join(preflight.diagnostics) or "semantic composition is not translation-ready",
                code="composition-capability-blocked",
                location=str(request_path.relative_to(root)),
                remediation="Run `taoryx vehicle preflight` and satisfy the listed mission capability checks.",
                blocking=True,
            )
    except (OSError, KeyError, TypeError, ValueError, VehicleCompositionError) as error:
        _failure(
            checks,
            diagnostics,
            "capability",
            "failed",
            str(error),
            code="composition-capability-failed",
            location=str(request_path.relative_to(root)),
            remediation="Run `taoryx vehicle preflight` and inspect the family-owned translator diagnostics.",
            blocking=True,
        )
    ####


def _doctor_showcase(
    scenario: ProductTwoScenario,
    root: Path,
    checks: list[dict[str, object]],
    diagnostics: list[dict[str, object]],
    context: dict[str, object],
) -> None:
    yaml_inputs = tuple(scenario.input_path(item, root=root) for item in scenario.inputs if item.kind == "showcase")
    for path in yaml_inputs:
        try:
            payload = yaml.safe_load(path.read_text(encoding="utf-8"))
            if not isinstance(payload, Mapping):
                raise ValueError("showcase contract must contain a mapping")
            checks.append(_check("showcase-contract", "passed", f"validated {path.relative_to(root)}"))
        except (OSError, UnicodeError, ValueError, yaml.YAMLError) as error:
            _failure(
                checks,
                diagnostics,
                "showcase-contract",
                "blocked",
                str(error),
                code="showcase-contract-invalid",
                location=str(path.relative_to(root)),
                remediation="Open the showcase contract and correct its YAML mapping before running the showcase.",
                blocking=True,
            )
    problem = next((item for item in scenario.inputs if item.kind == "problem"), None)
    if problem is not None:
        _doctor_source(scenario, root, checks, diagnostics, context)
    else:
        checks.append(_check("compilation", "not_applicable", "showcase runner owns its multi-artifact setup"))
        checks.append(_check("resolution", "not_applicable", "showcase contract is resolved by its runner"))
        checks.append(_check("lowering", "not_applicable", "showcase runner owns its fidelity ladder"))
        checks.append(_check("capability", "not_applicable", "showcase capability gates remain in the showcase packet"))
    ####


def _check_inputs(
    scenario: ProductTwoScenario,
    root: Path,
    source_inputs: list[dict[str, object]],
    diagnostics: list[dict[str, object]],
) -> dict[str, object]:
    missing = False
    for item in scenario.inputs:
        path = scenario.input_path(item, root=root)
        if not path.is_file():
            missing = True
            diagnostics.append(_diagnostic(
                code="input-missing",
                location=item.path,
                message=f"catalog input does not exist: {item.path}",
                remediation="Restore the catalog input or update the catalog entry to the authoritative source path.",
                blocking=True,
            ))
            continue
        data = path.read_bytes()
        source_inputs.append({"path": item.path, "role": item.role, "kind": item.kind, "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()})
    if missing:
        return _check("inputs", "blocked", "one or more catalog inputs are missing")
    return _check("inputs", "passed", f"verified {len(source_inputs)} catalog input(s)")
    ####


def _doctor_result(
    scenario: ProductTwoScenario,
    checks: list[dict[str, object]],
    diagnostics: list[dict[str, object]],
    source_inputs: list[dict[str, object]],
    context: dict[str, object],
) -> dict[str, object]:
    if any(item.get("blocking") for item in diagnostics):
        status = ProductTwoStatus.BLOCKED
    elif any(item.get("status") == "failed" for item in checks):
        status = ProductTwoStatus.FAILED
    else:
        status = ProductTwoStatus.PASSED
    return {
        "schema": "taoryx.product-two-doctor/v1alpha1",
        "scenario_id": scenario.id,
        "expected_disposition": scenario.expected_disposition.value,
        "status": status.value,
        "entrypoint": scenario.entrypoint,
        "checks": checks,
        "diagnostics": diagnostics,
        "source_inputs": source_inputs,
        "resolved": context,
        "claim_boundary": (
            "Doctor validates setup, source/table structure, semantic resolution, lowering, and declared capability "
            "readiness. It does not execute a trajectory, prove numerical accuracy, or qualify a vehicle."
        ),
    }
    ####


def _check(identifier: str, status: str, message: str) -> dict[str, object]:
    return {"id": identifier, "status": status, "message": message}
    ####


def _failure(
    checks: list[dict[str, object]],
    diagnostics: list[dict[str, object]],
    identifier: str,
    status: str,
    message: str,
    *,
    code: str,
    location: str,
    remediation: str,
    blocking: bool,
) -> None:
    checks.append(_check(identifier, status, message))
    diagnostics.append(_diagnostic(code=code, location=location, message=message, remediation=remediation, blocking=blocking))
    ####


def _diagnostic(*, code: str, location: str, message: str, remediation: str, blocking: bool) -> dict[str, object]:
    return {"code": code, "location": location, "message": message, "remediation": remediation, "blocking": blocking, "status": "blocked" if blocking else "failed"}
    ####


def _input_path(scenario: ProductTwoScenario, role: str, root: Path) -> Path:
    for item in scenario.inputs:
        if item.role == role or (role == "composition" and item.kind == "composition"):
            return scenario.input_path(item, root=root)
    raise ValueError(f"scenario {scenario.id!r} has no {role!r} input")
    ####


__all__ = ["doctor_scenario"]
####
