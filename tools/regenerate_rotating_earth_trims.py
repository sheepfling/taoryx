"""Regenerate the representative rotating-Earth trim evidence packet."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT))

try:
    from tools.rotating_earth_trim import NOMINAL_EARTH_RATE_RAD_S
except ModuleNotFoundError:
    from rotating_earth_trim import NOMINAL_EARTH_RATE_RAD_S

from taoryx.contracts import EarthModel, Latitude, Longitude, Quantity, Unit
from taoryx.rigid_body_frames import EarthOperatingPoint

OUTPUT_ROOT = ROOT / "artifacts/golden_plants/rotating_earth"
PROFILE_PATH = ROOT / "verification/rotating_earth_trim_profiles.yaml"
SOLVERS = {
    "b747": ROOT / "tools/solve_b747_trim.py",
    "skywalker_x8": ROOT / "tools/solve_x8_trim.py",
    "hummingbird": ROOT / "tools/solve_hummingbird_trim.py",
    "x15": ROOT / "tools/solve_x15_trim.py",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--earth-omega", type=float, default=NOMINAL_EARTH_RATE_RAD_S, help="Earth rotation rate in rad/s")
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT, help="directory for the generated trim packet")
    parser.add_argument("--profile", type=Path, default=PROFILE_PATH, help="tracked rotating-Earth profile catalog")
    parser.add_argument("--vehicle", choices=tuple(SOLVERS), action="append", help="regenerate only the selected vehicle; repeatable")
    parser.add_argument(
        "--latitude-deg",
        type=float,
        action="append",
        help="latitude sample for the reuse screen; repeatable (default: -60, -30, 0, 30, 60)",
    )
    parser.add_argument(
        "--gravity-tolerance-fraction",
        type=float,
        default=0.01,
        help="maximum effective-gravity delta for reusing a local trim",
    )
    return parser


def _wgs84(earth_omega_rad_s: float) -> EarthModel:
    return EarthModel(
        equatorial_radius=Quantity(6_378_137.0, Unit.METER),
        flattening=1.0 / 298.257223563,
        gravitational_parameter=Quantity(3.986004418e14, Unit.METER_CUBED_PER_SECOND_SQUARED),
        rotation_rate=Quantity(earth_omega_rad_s, Unit.RADIAN_PER_SECOND),
    )


def _latitude_sensitivity(
    profile: dict[str, object],
    earth_omega_rad_s: float,
    latitude_degrees: tuple[float, ...],
    gravity_tolerance_fraction: float,
) -> dict[str, object]:
    latitude = float(profile["latitude_deg"])
    longitude = float(profile["longitude_deg"])
    altitude = float(profile["altitude_m"])
    anchor = EarthOperatingPoint(
        _wgs84(earth_omega_rad_s),
        longitude=Longitude(math.radians(longitude)),
        latitude=Latitude(math.radians(latitude)),
        altitude_m=altitude,
    )
    samples = anchor.latitude_sensitivity(
        tuple(Latitude(math.radians(value)) for value in latitude_degrees),
        gravity_tolerance_fraction=gravity_tolerance_fraction,
    )
    return {
        "anchor": anchor.to_metadata(),
        "gravity_tolerance_fraction": gravity_tolerance_fraction,
        "screening_only": True,
        "samples": list(samples),
    }


def main() -> None:
    args = _parser().parse_args()
    if not (args.earth_omega == args.earth_omega and abs(args.earth_omega) < float("inf")):
        raise ValueError("--earth-omega must be finite")
    if not (math.isfinite(args.gravity_tolerance_fraction) and args.gravity_tolerance_fraction >= 0.0):
        raise ValueError("--gravity-tolerance-fraction must be finite and nonnegative")
    latitude_degrees = tuple(args.latitude_deg) if args.latitude_deg else (-60.0, -30.0, 0.0, 30.0, 60.0)
    if not all(math.isfinite(value) and -90.0 <= value <= 90.0 for value in latitude_degrees):
        raise ValueError("--latitude-deg values must be finite and within [-90, 90]")
    selected = tuple(args.vehicle) if args.vehicle else tuple(SOLVERS)
    profile_path = args.profile.resolve()
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    if not isinstance(profile, dict) or profile.get("schema_version") != 1 or not isinstance(profile.get("profiles"), list):
        raise ValueError(f"invalid rotating-Earth trim profile catalog: {profile_path}")
    profile_by_vehicle = {str(item["vehicle"]): item for item in profile["profiles"] if isinstance(item, dict) and "vehicle" in item}
    if any(vehicle not in profile_by_vehicle for vehicle in selected):
        raise ValueError("rotating-Earth profile catalog does not cover every selected vehicle")
    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, object]] = []
    for vehicle in selected:
        output = output_root / f"{vehicle}_trim_report.json"
        environment = os.environ.copy()
        source_path = str(ROOT / "src")
        environment["PYTHONPATH"] = source_path if not environment.get("PYTHONPATH") else source_path + os.pathsep + environment["PYTHONPATH"]
        environment["TAORYX_TRIM_EARTH_OMEGA"] = f"{args.earth_omega:.16g}"
        environment["TAORYX_TRIM_OUTPUT"] = str(output)
        subprocess.run([sys.executable, str(SOLVERS[vehicle])], cwd=ROOT, env=environment, check=True)
        payload = json.loads(output.read_text(encoding="utf-8"))
        records.append(
            {
                "vehicle": vehicle,
                "solver": str(SOLVERS[vehicle].relative_to(ROOT)),
                "report": str(output.relative_to(ROOT)) if output.is_relative_to(ROOT) else str(output),
                "report_sha256": _sha256(output),
                "success": bool(payload.get("solver", {}).get("success", False)),
                "earth_omega_rad_s": float(payload.get("earth_omega_rad_s", args.earth_omega)),
                "operating_point_mode": str(payload.get("operating_point_mode", "unspecified")),
                "representative_location": {
                    key: profile_by_vehicle[vehicle][key]
                    for key in ("latitude_deg", "longitude_deg", "altitude_m")
                    if key in profile_by_vehicle[vehicle]
                },
            }
        )
    manifest = {
        "schema_version": 1,
        "id": "taoryx-rotating-earth-trim-packet-v1",
        "earth": {
            "omega_rad_s": args.earth_omega,
            "frame": "ECIC integration with ECFC/local trim variables",
            "representative_location_policy": "vehicle source location or equatorial canonical anchor",
            "profile": str(profile_path.relative_to(ROOT)) if profile_path.is_relative_to(ROOT) else str(profile_path),
            "profile_sha256": _sha256(profile_path),
            "zero_rate_baseline": {
                "mode": "zero_rate_source_parity",
                "direct_solver_default_earth_omega_rad_s": 0.0,
                "batch_command_is_explicitly_rotating": True,
            },
            "latitude_sensitivity": {
                "latitudes_deg": list(latitude_degrees),
                "gravity_tolerance_fraction": args.gravity_tolerance_fraction,
                "profiles": {
                    vehicle: _latitude_sensitivity(
                        profile_by_vehicle[vehicle],
                        args.earth_omega,
                        latitude_degrees,
                        args.gravity_tolerance_fraction,
                    )
                    for vehicle in selected
                },
            },
        },
        "vehicles": records,
        "command": " ".join(sys.argv),
    }
    manifest_path = output_root / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(manifest_path)


if __name__ == "__main__":
    main()
