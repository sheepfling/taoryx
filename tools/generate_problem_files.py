"""Generate native TAOS/TAORYX problem files from scenario metadata."""

from __future__ import annotations

import argparse
import re
from pathlib import Path
from typing import Any

import yaml

from taoryx.vehicle_registry import vehicle_status_line

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "verification/problem_generation.yaml"
VEHICLES = ROOT / "verification/vehicle_models.yaml"
FAMILIES = ROOT / "verification/vehicle_families.yaml"
TOKEN = re.compile(r"{{([A-Za-z0-9_]+)}}")


def _values(catalog: dict[str, Any], scenario: dict[str, Any]) -> dict[str, str]:
    profile = catalog["profiles"][scenario["profile"]]
    vehicle_registry = yaml.safe_load((ROOT / catalog["vehicle_registry"]).read_text(encoding="utf-8"))
    family_registry = yaml.safe_load((ROOT / catalog["family_registry"]).read_text(encoding="utf-8"))
    vehicle_id = profile["vehicle"]
    vehicle = vehicle_registry["vehicles"][vehicle_id]
    family_id = vehicle["family_id"]
    family = family_registry["families"][family_id]
    vehicle_runtime = vehicle_status_line(vehicle_id)
    controls = "\n".join(
        f"*runtime control {control['name']} vehicle=1 default={control['default']} lower={control['lower']} upper={control['upper']}"
        for control in vehicle.get("controls", [])
    )
    actuator = " ".join(f"{key.replace('_', '-')}={value}" for key, value in vehicle.get("actuator", {}).items())
    earth_gm_suffix = f" gm={vehicle['earth_gm_m3_s2']}" if "earth_gm_m3_s2" in vehicle else ""
    values = {
        "mode": str(vehicle["mode"]),
        "trajectory_vehicle": str(vehicle["trajectory_vehicle"]),
        "family_id": family_id,
        "family_title": str(family["title"]),
        "earth_model": str(vehicle["earth_model"]),
        "earth_omega_rad_s": str(vehicle["earth_omega_rad_s"]),
        "earth_gm_suffix": earth_gm_suffix,
        "vehicle_runtime": vehicle_runtime,
        "control_lines": controls,
        "actuator_runtime": actuator,
        "aero_comment": "# Aerodynamic channels use the vehicle metadata convention; inspect the bound table headers before interpreting force or drag terms.",
    }
    values.update({key: str(value) for key, value in profile.items() if key != "vehicle"})
    values.update({key: str(value) for key, value in scenario.items() if key not in {"output", "profile"}})
    values["vehicle_runtime"] = _complete_vehicle_runtime(vehicle_id, values["vehicle_runtime"])
    for key in ("target_line", "route_line", "guidance_line", "fly_line", "file_line", "definition_line"):
        values.setdefault(key, "")
    if "propulsion_line" not in values:
        values["propulsion_line"] = ""
    if "aero_line" not in values:
        values["aero_line"] = "    *aero cx=(cx) cy=(cy) cz=(cz) cmx=(cmx) cmy=(cmy) cmz=(cmz)"
    elif "\n" in values["aero_line"]:
        # Multiline composed coefficient definitions belong to the active
        # segment.  The template placeholder is at column zero, so preserve
        # the native four-space segment indentation on every emitted line.
        values["aero_line"] = "\n".join(
            f"    {line}" if line.strip() else line
            for line in values["aero_line"].splitlines()
        )
    if "file_line" not in values:
        values["file_line"] = ""
    values["problem_id"] = str(scenario["id"])
    convention = vehicle.get("aero_convention", {})
    problem_comment = str(convention.get("problem_comment", "")).strip()
    if problem_comment:
        values["aero_comment"] = "\n".join(f"    # {line}" for line in problem_comment.splitlines())
    return values
####


def _complete_vehicle_runtime(vehicle_id: str, runtime_line: str) -> str:
    """Add canonical contract fields to profile-specific runtime overrides.

    A parity profile may intentionally override source-native fields such as
    the release mass or frame adapter.  It must still carry the shared
    controller contract, including the source-backed nominal mass, so the
    generated problem cannot silently fall back to an incomplete scale basis.
    """

    canonical = vehicle_status_line(vehicle_id)
    if not runtime_line.strip():
        return canonical
    canonical_attributes = canonical.split(" vehicle ", 1)[1].split()
    existing_keys = {
        token.split("=", 1)[0]
        for token in runtime_line.split()
        if "=" in token
    }
    missing = [
        token
        for token in canonical_attributes
        if token.split("=", 1)[0] not in existing_keys
    ]
    return runtime_line.rstrip() + (" " + " ".join(missing) if missing else "")
####


def validate_vehicle_registry(catalog_path: Path = CATALOG) -> tuple[str, ...]:
    """Validate family membership and required fields for every vehicle."""

    catalog = yaml.safe_load(catalog_path.read_text(encoding="utf-8"))
    vehicles = yaml.safe_load((ROOT / catalog["vehicle_registry"]).read_text(encoding="utf-8"))["vehicles"]
    families = yaml.safe_load((ROOT / catalog["family_registry"]).read_text(encoding="utf-8"))["families"]
    template_ids = catalog["templates"]
    generated_outputs = {str(scenario["output"]) for scenario in catalog["scenarios"]}
    public_catalog = yaml.safe_load((ROOT / "verification/vehicle_catalog.yaml").read_text(encoding="utf-8"))
    for entry in public_catalog["vehicles"]:
        if str(entry["problem"]) not in generated_outputs:
            raise ValueError(
                f"public vehicle catalog problem is not generated by the scenario catalog: {entry['problem']}"
            )
    for vehicle_id, vehicle in vehicles.items():
        if vehicle_id not in catalog_vehicle_ids(catalog):
            raise ValueError(f"vehicle {vehicle_id!r} is missing from the public vehicle catalog")
        family_id = vehicle.get("family_id")
        if family_id not in families:
            raise ValueError(f"vehicle {vehicle_id!r} selects unknown family {family_id!r}")
        family = families[family_id]
        missing = [field for field in family["required_vehicle_fields"] if field not in vehicle]
        if missing:
            raise ValueError(f"vehicle {vehicle_id!r} is missing family fields: {', '.join(missing)}")
        if vehicle["mode"] != family["dynamics_mode"]:
            raise ValueError(f"vehicle {vehicle_id!r} mode does not match family {family_id!r}")
        if vehicle["body_frame"] != family["body_frame"]:
            raise ValueError(f"vehicle {vehicle_id!r} frame does not match family {family_id!r}")
        if family["problem_template"] not in template_ids:
            raise ValueError(f"family {family_id!r} selects unknown problem template")
        for table in vehicle["table_bindings"]:
            if not (ROOT / table).is_file():
                raise ValueError(f"vehicle {vehicle_id!r} table binding does not exist: {table}")
    return tuple(vehicles)
####


def catalog_vehicle_ids(catalog: dict[str, Any]) -> set[str]:
    """Return vehicle IDs represented by the public vehicle catalog."""

    vehicle_catalog_path = ROOT / "verification/vehicle_catalog.yaml"
    vehicle_catalog = yaml.safe_load(vehicle_catalog_path.read_text(encoding="utf-8"))
    return {str(entry["model_id"]) for entry in vehicle_catalog["vehicles"]}
####


def render_scenario(catalog: dict[str, Any], scenario: dict[str, Any]) -> str:
    """Render one scenario and reject missing or unused template values."""

    profile = catalog["profiles"][scenario["profile"]]
    vehicle_registry = yaml.safe_load((ROOT / catalog["vehicle_registry"]).read_text(encoding="utf-8"))
    family_registry = yaml.safe_load((ROOT / catalog["family_registry"]).read_text(encoding="utf-8"))
    family_id = vehicle_registry["vehicles"][profile["vehicle"]]["family_id"]
    template_id = str(scenario.get("template", family_registry["families"][family_id]["problem_template"]))
    template_path = ROOT / catalog["templates"][template_id]
    template = template_path.read_text(encoding="utf-8")
    values = _values(catalog, scenario)

    def replace(match: re.Match[str]) -> str:
        name = match.group(1)
        if name not in values:
            raise ValueError(f"scenario {scenario['id']!r} is missing template value {name!r}")
        return values[name]
    ####

    rendered = template
    for _ in range(3):
        updated = TOKEN.sub(replace, rendered)
        if updated == rendered:
            break
        rendered = updated
    if "{{" in rendered or "}}" in rendered:
        raise ValueError(f"scenario {scenario['id']!r} contains unresolved template text")
    return rendered.rstrip() + "\n"
####


def render_catalog(catalog_path: Path = CATALOG) -> tuple[Path, ...]:
    """Write every catalog scenario and return the generated paths."""

    catalog = yaml.safe_load(catalog_path.read_text(encoding="utf-8"))
    generated: list[Path] = []
    for scenario in catalog["scenarios"]:
        output = ROOT / scenario["output"]
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(render_scenario(catalog, scenario), encoding="utf-8")
        generated.append(output)
    return tuple(generated)
####


def check_catalog(catalog_path: Path = CATALOG) -> tuple[Path, ...]:
    """Verify generated files are current without modifying the workspace."""

    validate_vehicle_registry(catalog_path)
    catalog = yaml.safe_load(catalog_path.read_text(encoding="utf-8"))
    stale: list[Path] = []
    for scenario in catalog["scenarios"]:
        output = ROOT / scenario["output"]
        expected = render_scenario(catalog, scenario)
        if not output.is_file() or output.read_text(encoding="utf-8") != expected:
            stale.append(output)
    if stale:
        names = ", ".join(str(path.relative_to(ROOT)) for path in stale)
        raise ValueError(f"generated problem files are stale: {names}")
    return tuple(ROOT / scenario["output"] for scenario in catalog["scenarios"])
####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, default=CATALOG)
    parser.add_argument("--check", action="store_true", help="check generated files without writing")
    parser.add_argument("--check-models", action="store_true", help="check vehicle-family registry and bindings")
    args = parser.parse_args()
    if args.check_models:
        validate_vehicle_registry(args.catalog)
    elif args.check:
        check_catalog(args.catalog)
    else:
        render_catalog(args.catalog)
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
####
