"""Build the airbreathing powered-fixed-wing racetrack fidelity ladder."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Final

try:
    from build_family_qualification_packet import build
except ModuleNotFoundError:  # pragma: no cover - direct script execution puts tools on sys.path
    from tools.build_family_qualification_packet import build

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "artifacts/showcases/airbreathing-racetrack-fidelity-ladder"

FIDELITIES: Final[tuple[str, ...]] = ("3dof", "pseudo-6dof", "6dof-direct-wrench", "6dof-surfaces")

FIDELITY_DEFINITIONS: Final[dict[str, dict[str, str]]] = {
    "3dof": {
        "runtime_mode": "point-mass",
        "canonical_name": "point_mass_3dof",
        "control_path": "achievable translational force/lift-vector/throttle",
    },
    "pseudo-6dof": {
        "runtime_mode": "kinematic-6dof",
        "canonical_name": "pseudo_6dof_kinematic_bridge",
        "control_path": "force-integrated translation plus prescribed attitude/rate response",
    },
    "6dof-direct-wrench": {
        "runtime_mode": "rigid-body-6dof",
        "canonical_name": "rigid_body_6dof_direct_wrench",
        "control_path": "Newton-Euler body state plus declared direct/induced force or moment",
    },
    "6dof-surfaces": {
        "runtime_mode": "rigid-body-6dof",
        "canonical_name": "rigid_body_6dof_surface_allocated",
        "control_path": "Newton-Euler body state plus bounded physical effector allocation",
    },
}

F16_PACKET_DIR: Final[str] = "f16-s119-racetrack-fidelity-ladder"
F16_PACKET_FIDELITY_DIR: Final[dict[str, str]] = {
    "3dof": "3dof",
    "pseudo-6dof": "pseudo-6dof",
    "6dof-direct-wrench": "6dof-direct-wrench",
    "6dof-surfaces": "6dof-surfaces",
}

MISSION_IDS: Final[dict[str, dict[str, str | None]]] = {
    "x8": {
        "3dof": "x8-racetrack-altitude-turns-3dof-v1",
        "pseudo-6dof": "x8-racetrack-altitude-turns-pseudo-6dof-v1",
        "6dof-direct-wrench": "x8-racetrack-altitude-turns-direct-wrench-v1",
        "6dof-surfaces": "x8-racetrack-altitude-turns-v1",
    },
    "b747": {
        "3dof": "b747-racetrack-altitude-turns-3dof-v1",
        "pseudo-6dof": "b747-racetrack-altitude-turns-pseudo-6dof-v1",
        "6dof-direct-wrench": "b747-racetrack-altitude-turns-6dof-v1",
        "6dof-surfaces": None,
    },
    "f16": {
        "3dof": None,
        "pseudo-6dof": None,
        "6dof-direct-wrench": None,
        "6dof-surfaces": None,
    },
}


def _selected_missions(vehicle: str, fidelity: str) -> tuple[tuple[str, str, str | None], ...]:
    vehicles = tuple(MISSION_IDS) if vehicle == "all" else (vehicle,)
    fidelities = FIDELITIES if fidelity == "all" else (fidelity,)
    return tuple(
        (vehicle_id, fidelity_id, MISSION_IDS[vehicle_id][fidelity_id])
        for vehicle_id in vehicles
        for fidelity_id in fidelities
    )


def _write_ladder_manifest(output: Path, selected: tuple[tuple[str, str, str | None], ...], records: list[dict[str, object]]) -> None:
    output.mkdir(parents=True, exist_ok=True)
    manifest = {
        "schema_version": 1,
        "catalog": "airbreathing-racetrack-fidelity-ladder-v1",
        "vehicle_families": [vehicle for vehicle in MISSION_IDS if any(item[0] == vehicle for item in selected)],
        "fidelities": [fidelity for fidelity in FIDELITIES if any(item[1] == fidelity for item in selected)],
        "fidelity_definitions": FIDELITY_DEFINITIONS,
        "records": records,
        "commands": [record["reproduction_command"] for record in records],
    }
    (output / "summary.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(
        "\n".join(
            [
                "# Airbreathing racetrack fidelity ladder",
                "",
                "Build all runnable packets and record unavailable tiers:",
                "PYTHONPATH=src python3 tools/build_airbreathing_racetrack_ladder.py",
                "",
                "Build one vehicle at every advertised fidelity:",
                "PYTHONPATH=src python3 tools/build_airbreathing_racetrack_ladder.py --vehicle x8",
                "PYTHONPATH=src python3 tools/build_airbreathing_racetrack_ladder.py --vehicle b747",
                "PYTHONPATH=src python3 tools/build_airbreathing_racetrack_ladder.py --vehicle f16",
                "",
                "Build one exact packet:",
                "PYTHONPATH=src python3 tools/build_airbreathing_racetrack_ladder.py --vehicle x8 --fidelity pseudo-6dof",
                "",
            ]
            + [str(record["reproduction_command"]) for record in records]
        )
        + "\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--vehicle", choices=("all", "x8", "b747", "f16"), default="all")
    parser.add_argument("--fidelity", choices=("all", *FIDELITIES), default="all")
    args = parser.parse_args()

    selected = _selected_missions(args.vehicle, args.fidelity)
    records: list[dict[str, object]] = []
    f16_archive: Path | None = None
    f16_packet: Path | None = None
    for vehicle, fidelity, mission_id in selected:
        if mission_id is None:
            if vehicle == "f16":
                if f16_archive is None:
                    try:
                        from build_f16_racetrack_fidelity_packet import build as build_f16_packet
                    except ModuleNotFoundError:  # pragma: no cover - package execution path
                        from tools.build_f16_racetrack_fidelity_packet import build as build_f16_packet

                    f16_packet = args.output / F16_PACKET_DIR
                    f16_archive = build_f16_packet(f16_packet)
                assert f16_packet is not None
                packet = f16_packet / F16_PACKET_FIDELITY_DIR[fidelity]
                evidence = json.loads((packet / "evidence.json").read_text(encoding="utf-8"))
                runtime = evidence["runtime"]
                evaluation = evidence["evaluation"]
                record = {
                    "vehicle": vehicle,
                    "fidelity": fidelity,
                    "fidelity_definition": FIDELITY_DEFINITIONS[fidelity],
                    "mission_id": "f16-s119-shared-racetrack-fidelity-v1",
                    "archive": str(f16_archive),
                    "status": evidence["status"],
                    "mission_pass": evaluation["mission_pass"],
                    "numerical_valid": runtime["numerical_valid"],
                    "duration_s": runtime["duration_s"],
                    "reproduction_command": (
                        "PYTHONPATH=src python3 tools/build_f16_racetrack_fidelity_packet.py "
                        f"--output {f16_packet} --dt-s 0.5"
                    ),
                }
                records.append(record)
                print(f"{vehicle:5s} {fidelity:18s} {evidence['status']}: {f16_archive}")
                continue
            records.append(
                {
                    "vehicle": vehicle,
                    "fidelity": fidelity,
                    "fidelity_definition": FIDELITY_DEFINITIONS[fidelity],
                    "mission_id": None,
                    "archive": None,
                    "status": "not_available",
                    "mission_pass": False,
                    "numerical_valid": None,
                    "duration_s": None,
                    "reproduction_command": "NOT AVAILABLE: no declared physical surface-allocation model for this vehicle",
                }
            )
            print(f"{vehicle:5s} {fidelity:18s} not_available: physical surface model pending")
            continue
        archive = build(args.output, mission_id)
        summary_path = args.output / mission_id / "summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        command = f"PYTHONPATH=src python3 tools/build_family_qualification_packet.py --output {args.output} --mission {mission_id}"
        records.append(
            {
                "vehicle": vehicle,
                "fidelity": fidelity,
                "fidelity_definition": FIDELITY_DEFINITIONS[fidelity],
                "mission_id": mission_id,
                "archive": str(archive),
                "status": summary["status"],
                "mission_pass": summary["mission_pass"],
                "numerical_valid": summary["numerical_valid"],
                "duration_s": summary["run"]["duration_s"],
                "reproduction_command": command,
            }
        )
        print(f"{vehicle:5s} {fidelity:18s} {summary['status']}: {archive}")
    _write_ladder_manifest(args.output, selected, records)
    print(f"ladder summary: {args.output / 'summary.json'}")


if __name__ == "__main__":
    main()
