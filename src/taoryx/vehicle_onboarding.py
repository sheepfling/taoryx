"""Diagnostics for registering a new vehicle in the standard TAORYX workflow."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import yaml

from .language.table_parser import parse_table_file
from .vehicle_registry import ROOT, derive_lqr_scale_contract

Severity = Literal["error", "warning"]

_REGISTRY = ROOT / "verification/vehicle_models.yaml"
_FAMILIES = ROOT / "verification/vehicle_families.yaml"
_PUBLIC_CATALOG = ROOT / "verification/vehicle_catalog.yaml"
_PROBLEM_CATALOG = ROOT / "verification/problem_generation.yaml"
_TRIM_CATALOG = ROOT / "verification/trim_specs.yaml"
_CONTROLLER_CATALOG = ROOT / "verification/controller_designs.yaml"
_DIRECTION_CATALOG = ROOT / "verification/control_direction_contracts.yaml"
_LQR_PROFILES = ROOT / "verification/lqr_scaling_profiles.yaml"


@dataclass(frozen=True, slots=True)
class OnboardingFinding:
    """One actionable registration diagnostic."""

    severity: Severity
    code: str
    path: str
    message: str
    hint: str

    def as_dict(self) -> dict[str, str]:
        """Return a JSON-compatible diagnostic."""

        return {
            "severity": self.severity,
            "code": self.code,
            "path": self.path,
            "message": self.message,
            "hint": self.hint,
        }
    ####


@dataclass(frozen=True, slots=True)
class VehicleOnboardingReport:
    """Complete metadata and source-contract report for one vehicle."""

    vehicle_id: str
    findings: tuple[OnboardingFinding, ...]

    @property
    def errors(self) -> tuple[OnboardingFinding, ...]:
        """Return blocking diagnostics."""

        return tuple(item for item in self.findings if item.severity == "error")
    ####

    @property
    def warnings(self) -> tuple[OnboardingFinding, ...]:
        """Return non-blocking but explicit caveats."""

        return tuple(item for item in self.findings if item.severity == "warning")
    ####

    @property
    def status(self) -> str:
        """Return the onboarding state used by the CLI and evidence reports."""

        if self.errors:
            return "blocked"
        return "ready-with-warnings" if self.warnings else "ready"
    ####

    def ready(self, *, strict: bool = False) -> bool:
        """Return whether the vehicle can advance under the selected policy."""

        return not self.errors and (not strict or not self.warnings)
    ####

    def as_dict(self) -> dict[str, Any]:
        """Return a stable machine-readable onboarding report."""

        return {
            "vehicle_id": self.vehicle_id,
            "status": self.status,
            "error_count": len(self.errors),
            "warning_count": len(self.warnings),
            "findings": [item.as_dict() for item in self.findings],
        }
    ####


def _relative(path: Path) -> str:
    """Render repository paths consistently in diagnostics."""

    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)
    ####


def _read_yaml(path: Path, findings: list[OnboardingFinding]) -> Mapping[str, Any]:
    """Read one catalog while turning syntax and I/O failures into guidance."""

    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        findings.append(
            OnboardingFinding(
                "error",
                "metadata-load-failed",
                _relative(path),
                f"cannot read vehicle onboarding metadata: {error}",
                "Restore the catalog file or fix its YAML syntax, then rerun this validator.",
            )
        )
        return {}
    if not isinstance(payload, Mapping):
        findings.append(
            OnboardingFinding(
                "error",
                "metadata-not-a-mapping",
                _relative(path),
                "vehicle onboarding metadata must contain a YAML mapping at the document root",
                "Use the existing catalog files as the schema reference; do not wrap the document in a list.",
            )
        )
        return {}
    return payload
    ####


def _add(
    findings: list[OnboardingFinding],
    severity: Severity,
    code: str,
    path: str,
    message: str,
    hint: str,
) -> None:
    """Append one consistently shaped finding."""

    findings.append(OnboardingFinding(severity, code, path, message, hint))
    ####


def _vehicle_catalog_entry(catalog: Mapping[str, Any], vehicle_id: str) -> Mapping[str, Any] | None:
    """Find a public golden-plant catalog entry by model ID."""

    entries = catalog.get("vehicles", ())
    if not isinstance(entries, list):
        return None
    return next((entry for entry in entries if isinstance(entry, Mapping) and str(entry.get("model_id")) == vehicle_id), None)
    ####


def validate_vehicle_onboarding(vehicle_id: str) -> VehicleOnboardingReport:
    """Validate one vehicle across every canonical registration catalog.

    The validator checks metadata completeness, source/table reachability,
    generated problem coverage, trim/controller hooks, signed-direction
    coverage, and LQR profile availability. It intentionally does not claim
    that a trim or mission passes; those remain the golden-plant and maneuver
    harness stages.
    """

    findings: list[OnboardingFinding] = []
    registry = _read_yaml(_REGISTRY, findings)
    families = _read_yaml(_FAMILIES, findings)
    public_catalog = _read_yaml(_PUBLIC_CATALOG, findings)
    problem_catalog = _read_yaml(_PROBLEM_CATALOG, findings)
    trim_catalog = _read_yaml(_TRIM_CATALOG, findings)
    controller_catalog = _read_yaml(_CONTROLLER_CATALOG, findings)
    direction_catalog = _read_yaml(_DIRECTION_CATALOG, findings)
    profiles_catalog = _read_yaml(_LQR_PROFILES, findings)

    vehicles = registry.get("vehicles", {})
    if not isinstance(vehicles, Mapping) or vehicle_id not in vehicles:
        known = sorted(str(name) for name in vehicles) if isinstance(vehicles, Mapping) else []
        _add(
            findings,
            "error",
            "unknown-vehicle-id",
            f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}",
            f"vehicle ID {vehicle_id!r} is not registered",
            f"Add it under vehicles or choose one of: {', '.join(known) or '(none loaded)'}.",
        )
        return VehicleOnboardingReport(vehicle_id, tuple(findings))

    vehicle = vehicles[vehicle_id]
    if not isinstance(vehicle, Mapping):
        _add(
            findings,
            "error",
            "vehicle-entry-not-a-mapping",
            f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}",
            "vehicle entry must be a YAML mapping",
            "Copy the shape of an existing vehicle entry and keep physical data in SI units.",
        )
        return VehicleOnboardingReport(vehicle_id, tuple(findings))

    family_id = str(vehicle.get("family_id", ""))
    family_map = families.get("families", {})
    family = family_map.get(family_id) if isinstance(family_map, Mapping) else None
    if not isinstance(family, Mapping):
        _add(
            findings,
            "error",
            "unknown-family",
            f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.family_id",
            f"vehicle selects unknown family {family_id!r}",
            f"Add the family to {_relative(_FAMILIES)} first, or select one of: {', '.join(sorted(str(key) for key in family_map))}.",
        )
    else:
        required = family.get("required_vehicle_fields", ())
        for field in required if isinstance(required, list) else ():
            if field not in vehicle:
                _add(
                    findings,
                    "error",
                    "missing-family-field",
                    f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.{field}",
                    f"family {family_id!r} requires field {field!r}",
                    f"Add the source-backed field to the {vehicle_id!r} registry entry; the family contract is not inferred.",
                )
        if vehicle.get("mode") != family.get("dynamics_mode"):
            _add(
                findings,
                "error",
                "dynamics-mode-family-mismatch",
                f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.mode",
                f"mode {vehicle.get('mode')!r} does not match family {family_id!r} mode {family.get('dynamics_mode')!r}",
                "Use the family dynamics mode unless you are defining a new family contract.",
            )
        if vehicle.get("body_frame") != family.get("body_frame"):
            _add(
                findings,
                "error",
                "body-frame-family-mismatch",
                f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.body_frame",
                f"body frame {vehicle.get('body_frame')!r} does not match family {family_id!r}",
                "Add a tested frame adapter and a new family if the source model uses a different convention.",
            )

    for field in ("reference_area_m2", "reference_length_m", "dry_mass_kg", "nominal_mass_kg"):
        try:
            value = float(vehicle[field])
        except (KeyError, TypeError, ValueError):
            _add(
                findings,
                "error",
                "invalid-physical-field",
                f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.{field}",
                f"{field} must be a finite positive number in SI units",
                "Copy the value from the source model and preserve the unit in the field name.",
            )
        else:
            if not math.isfinite(value) or value <= 0.0:
                _add(
                    findings,
                    "error",
                    "nonpositive-physical-field",
                    f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.{field}",
                    f"{field} must be positive, got {value}",
                    "Use the source reference geometry or mass in canonical SI units.",
                )
    ####

    inertia = vehicle.get("inertia_kg_m2")
    if not isinstance(inertia, Mapping) or set(inertia) != {"x", "y", "z"}:
        _add(
            findings,
            "error",
            "incomplete-inertia",
            f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.inertia_kg_m2",
            "inertia_kg_m2 must provide positive x, y, and z components",
            "For products of inertia or a full tensor, extend the family contract and frame adapter explicitly.",
        )
    else:
        for axis, value in inertia.items():
            try:
                numeric = float(value)
            except (TypeError, ValueError):
                _add(
                    findings,
                    "error",
                    "invalid-inertia",
                    f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.inertia_kg_m2.{axis}",
                    f"inertia component must be a finite positive number, got {value!r}",
                    "Check the source unit conversion and axis ordering.",
                )
                continue
            if not math.isfinite(numeric) or numeric <= 0.0:
                _add(
                    findings,
                    "error",
                    "nonpositive-inertia",
                    f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.inertia_kg_m2.{axis}",
                    f"inertia component must be positive, got {value}",
                    "Check the source unit conversion and axis ordering.",
                )

    controls = vehicle.get("controls", ())
    if not isinstance(controls, list) or not controls:
        _add(
            findings,
            "error",
            "missing-control-contract",
            f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.controls",
            "vehicle has no bounded control declarations",
            "Declare each propulsion, attitude, and allocation input with default, lower, and upper values.",
        )
    else:
        control_names: set[str] = set()
        for index, control in enumerate(controls):
            field_path = f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.controls[{index}]"
            if not isinstance(control, Mapping):
                _add(
                    findings,
                    "error",
                    "control-entry-not-a-mapping",
                    field_path,
                    "control declaration must be a mapping",
                    "Use name/default/lower/upper fields like the existing vehicle entries.",
                )
                continue
            name = str(control.get("name", ""))
            if not name or name in control_names:
                _add(
                    findings,
                    "error",
                    "invalid-control-name",
                    f"{field_path}.name",
                    f"control name is missing or duplicated: {name!r}",
                    "Give every actuator/control coordinate a unique stable name.",
                )
            control_names.add(name)
            try:
                lower = float(control["lower"])
                upper = float(control["upper"])
                default = float(control["default"])
            except (KeyError, TypeError, ValueError):
                _add(
                    findings,
                    "error",
                    "incomplete-control-bounds",
                    field_path,
                    "control must provide finite default, lower, and upper values",
                    "Keep command limits in the control's declared unit and name the unit in the field or control contract.",
                )
            else:
                if not all(math.isfinite(value) for value in (lower, upper, default)) or lower > upper or not lower <= default <= upper:
                    _add(
                        findings,
                        "error",
                        "invalid-control-bounds",
                        field_path,
                        f"control bounds are invalid: lower={lower}, default={default}, upper={upper}",
                        "Ensure lower <= default <= upper and verify the source unit conversion.",
                    )

    bindings = tuple(str(path) for path in vehicle.get("table_bindings", ()))
    if not bindings:
        _add(
            findings,
            "error",
            "missing-table-bindings",
            f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.table_bindings",
            "vehicle has no source table bindings",
            "Add every required .tbl file, then include the same paths in the public vehicle catalog.",
        )
    for index, relative_path in enumerate(bindings):
        path = ROOT / relative_path
        field_path = f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.table_bindings[{index}]"
        if not path.is_file():
            _add(
                findings,
                "error",
                "missing-table-file",
                field_path,
                f"table binding does not exist: {relative_path}",
                "Place the verified .tbl fixture at that path or correct the registry path; do not silently fall back.",
            )
            continue
        try:
            document = parse_table_file(path)
            if not document.tables:
                raise ValueError("contains no table definitions")
            if not document.executable_complete:
                diagnostic = next((item for item in document.diagnostics if item.severity.value == "error"), None)
                detail = diagnostic.message if diagnostic is not None else "parser reported an incomplete table"
                raise ValueError(detail)
        except (OSError, RuntimeError, TypeError, ValueError) as error:
            _add(
                findings,
                "error",
                "table-ingestion-failed",
                field_path,
                f"could not ingest {relative_path}: {error}",
                "Run the table parser fixture tests and fix the .tbl syntax or axis declarations before controller work.",
            )

    source = vehicle.get("source", {})
    if not isinstance(source, Mapping):
        _add(
            findings,
            "error",
            "missing-source-provenance",
            f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.source",
            "source provenance must identify a bundle and model path",
            "Record whether the data is public research, measured, generated, or synthetic.",
        )
    else:
        bundle = ROOT / str(source.get("bundle", ""))
        if not bundle.is_dir():
            _add(
                findings,
                "error",
                "missing-source-bundle",
                f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.source.bundle",
                f"source bundle does not exist: {source.get('bundle', '')}",
                "Preserve the source bundle under tests/fixtures or update the provenance path.",
            )
        model_path = source.get("model_path")
        if not model_path:
            _add(
                findings,
                "error",
                "missing-source-model-path",
                f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.source.model_path",
                "source.model_path is missing",
                "Point to the source-native model directory or file inside the declared bundle.",
            )
        elif bundle.is_dir() and not (bundle / str(model_path)).exists():
            _add(
                findings,
                "error",
                "missing-source-model",
                f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.source.model_path",
                f"source model path does not exist inside the bundle: {model_path}",
                "Correct the model path or retain the source-native directory in the fixture bundle.",
            )

    public_entry = _vehicle_catalog_entry(public_catalog, vehicle_id)
    if public_entry is None:
        _add(
            findings,
            "error",
            "missing-public-catalog-entry",
            f"{_relative(_PUBLIC_CATALOG)}:vehicles",
            "vehicle is not present in the standard golden-plant catalog",
            "Add one .prb entry and its table list so the common harness can discover the vehicle.",
        )
    else:
        problem = ROOT / str(public_entry.get("problem", ""))
        if not problem.is_file():
            _add(
                findings,
                "error",
                "missing-golden-problem",
                f"{_relative(_PUBLIC_CATALOG)}:vehicles[{vehicle_id}].problem",
                f"golden problem file does not exist: {public_entry.get('problem', '')}",
                "Generate it with `python tools/dev.py generate-problems` after adding its metadata profile.",
            )
        catalog_tables = {str(path) for path in public_entry.get("tables", ())}
        missing_from_registry = catalog_tables - set(bindings)
        if missing_from_registry:
            _add(
                findings,
                "error",
                "catalog-table-not-registered",
                f"{_relative(_PUBLIC_CATALOG)}:vehicles[{vehicle_id}].tables",
                f"public catalog references tables absent from the vehicle registry: {', '.join(sorted(missing_from_registry))}",
                "Add the table to vehicle_models.yaml or remove it from the public catalog after review.",
            )

    profile_map = profiles_catalog.get("profiles", {})
    profile_prefix = vehicle_id.replace("_", "-")
    active_profile = vehicle.get("attitude_lqr_profile")
    if not active_profile:
        _add(
            findings,
            "error",
            "missing-active-lqr-profile",
            f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.attitude_lqr_profile",
            "vehicle does not select an active LQR profile",
            "Select the standard profile after the three style profiles are registered.",
        )
    elif not isinstance(profile_map, Mapping) or active_profile not in profile_map:
        _add(
            findings,
            "error",
            "active-lqr-profile-not-found",
            f"{_relative(_REGISTRY)}:vehicles.{vehicle_id}.attitude_lqr_profile",
            f"active LQR profile {active_profile!r} is not defined",
            "Add the profile to lqr_scaling_profiles.yaml or correct the registry reference.",
        )
    for style in ("gentle", "standard", "aggressive"):
        profile_id = f"{profile_prefix}-{style}"
        profile = profile_map.get(profile_id) if isinstance(profile_map, Mapping) else None
        if not isinstance(profile, Mapping):
            _add(
                findings,
                "error",
                "missing-lqr-profile",
                f"{_relative(_LQR_PROFILES)}:profiles.{profile_id}",
                f"missing {style} LQR profile for {vehicle_id}",
                "Copy the three profile variants for an existing vehicle and adjust only source-backed scales/weights.",
            )
        elif str(profile.get("vehicle")) != vehicle_id:
            _add(
                findings,
                "error",
                "lqr-profile-vehicle-mismatch",
                f"{_relative(_LQR_PROFILES)}:profiles.{profile_id}.vehicle",
                f"profile points to {profile.get('vehicle')!r}, not {vehicle_id!r}",
                "Keep profile IDs and vehicle IDs aligned; gains must never cross vehicle contracts silently.",
            )
    try:
        derive_lqr_scale_contract(vehicle_id)
    except (OSError, RuntimeError, TypeError, ValueError, KeyError) as error:
        _add(
            findings,
            "error",
            "lqr-scale-contract-failed",
            f"{_relative(_LQR_PROFILES)}:profiles.{profile_prefix}-standard",
            f"cannot derive controller scales: {error}",
            "Fix table bindings, inertia, actuator authority, or add an explicit family-specific scale contract.",
        )

    trim_entries = trim_catalog.get("entries", ())
    vehicle_trims = tuple(entry for entry in trim_entries if isinstance(entry, Mapping) and str(entry.get("vehicle")) == vehicle_id)
    if not vehicle_trims:
        _add(
            findings,
            "error",
            "missing-trim-spec",
            f"{_relative(_TRIM_CATALOG)}:entries",
            "vehicle has no trim specification",
            "Add a bounded source-trim entry before asking the controller autotuner for a plant-backed design.",
        )
    elif not any(str(entry.get("status", "ready")) == "ready" for entry in vehicle_trims):
        _add(
            findings,
            "warning",
            "trim-source-only",
            f"{_relative(_TRIM_CATALOG)}:entries[{vehicle_id}]",
            "vehicle has a trim entry, but none is marked ready for an independent residual adapter",
            "Implement the source-backed residual adapter and change status to ready only after the trim evidence passes.",
        )

    controller_entries = controller_catalog.get("designs", ())
    vehicle_controllers = tuple(
        entry
        for entry in controller_entries
        if isinstance(entry, Mapping)
        and any(str(trim.get("id")) == str(entry.get("trim")) for trim in vehicle_trims)
    )
    if not vehicle_controllers:
        _add(
            findings,
            "error",
            "missing-controller-design",
            f"{_relative(_CONTROLLER_CATALOG)}:designs",
            "vehicle has no controller design linked to one of its trim specs",
            "Add a controller-design entry with method, trim, allocator, states, and controls.",
        )
    else:
        for index, entry in enumerate(vehicle_controllers):
            if not entry.get("allocator") or not entry.get("states") or not entry.get("controls"):
                _add(
                    findings,
                    "error",
                    "incomplete-controller-design",
                    f"{_relative(_CONTROLLER_CATALOG)}:designs[{index}]",
                    f"controller design {entry.get('id', '(unnamed)')!r} lacks allocator, states, or controls",
                    "Complete the design contract before selecting a profile for a maneuver.",
                )

    direction_entries = direction_catalog.get("vehicles", ())
    direction = next(
        (entry for entry in direction_entries if isinstance(entry, Mapping) and str(entry.get("vehicle")) == vehicle_id),
        None,
    )
    if direction is None:
        _add(
            findings,
            "error",
            "missing-control-direction-contract",
            f"{_relative(_DIRECTION_CATALOG)}:vehicles",
            "vehicle has no signed control-direction probes",
            "Add baseline commands and at least one expected signed response for every controller axis.",
        )
    else:
        direction_problem = ROOT / str(direction.get("problem", ""))
        if not direction_problem.is_file():
            _add(
                findings,
                "error",
                "missing-direction-probe-problem",
                f"{_relative(_DIRECTION_CATALOG)}:vehicles[{vehicle_id}].problem",
                f"control-direction problem does not exist: {direction.get('problem', '')}",
                "Point the probe at a standard generated or source-preserved .prb file.",
            )
        probes = direction.get("probes", ())
        if not isinstance(probes, list) or not probes:
            _add(
                findings,
                "error",
                "empty-control-direction-probes",
                f"{_relative(_DIRECTION_CATALOG)}:vehicles[{vehicle_id}].probes",
                "control-direction contract has no probes",
                "Declare signed finite-difference probes for the vehicle's meaningful controls.",
            )

    problem_profiles = problem_catalog.get("profiles", {})
    problem_scenarios = problem_catalog.get("scenarios", ())
    profile_ids = {
        str(name)
        for name, profile in problem_profiles.items()
        if isinstance(profile, Mapping) and str(profile.get("vehicle")) == vehicle_id
    }
    generated_scenarios = tuple(
        scenario
        for scenario in problem_scenarios
        if isinstance(scenario, Mapping) and str(scenario.get("profile")) in profile_ids
    )
    if not profile_ids:
        _add(
            findings,
            "error",
            "missing-problem-generation-profile",
            f"{_relative(_PROBLEM_CATALOG)}:profiles",
            "vehicle has no metadata-driven problem-generation profile",
            "Add a profile selecting the vehicle and a standard family template before hand-writing .prb files.",
        )
    if not generated_scenarios:
        _add(
            findings,
            "error",
            "missing-generated-problem-scenario",
            f"{_relative(_PROBLEM_CATALOG)}:scenarios",
            "vehicle has no generated problem scenario",
            "Add a source-anchor scenario and run `python tools/dev.py generate-problems`.",
        )

    return VehicleOnboardingReport(vehicle_id, tuple(findings))
    ####


def validate_all_vehicle_onboarding() -> tuple[VehicleOnboardingReport, ...]:
    """Validate every vehicle currently present in the canonical registry."""

    findings: list[OnboardingFinding] = []
    registry = _read_yaml(_REGISTRY, findings)
    vehicles = registry.get("vehicles", {})
    if not isinstance(vehicles, Mapping):
        return (VehicleOnboardingReport("<registry>", tuple(findings)),)
    return tuple(validate_vehicle_onboarding(str(vehicle_id)) for vehicle_id in vehicles)
    ####


__all__ = [
    "OnboardingFinding",
    "VehicleOnboardingReport",
    "validate_all_vehicle_onboarding",
    "validate_vehicle_onboarding",
]
