from __future__ import annotations

from pathlib import Path


SPEEDS_MPS = (450.0, 600.0, 750.0)
FLIGHT_PATH_ANGLES_DEG = (10.0, 20.0, 30.0)


def write_case(path: Path, *, name: str, speed_mps: float, flight_path_angle_deg: float) -> None:
    content = f"""({name})

*title Ballistic cone launch sweep case

*atmos standard
*earth wgs-84 omega=0

*trajectory 1 cone projectile start on 1

  *initial geodetic
    alt=10000.0 long=0.0 lat=0.0 vel={speed_mps:.1f} gama={flight_path_angle_deg:.1f} psi=90.0 time=0.0 mass=100.0

  *file {name}.dat time alt vel gama dynprs cd nx

  *segment 1 coast
    *integ dtprnt=0.5 dt=0.1
    *aero cd=(cone-cd)
    *fly alpha=0.0
    *when alt<0 stop

*end
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
####


def main() -> None:
    root = Path(__file__).resolve().parent / "generated" / "sweeps"
    entries: list[str] = ["# Ballistic Cone Launch Sweeps", ""]
    for speed in SPEEDS_MPS:
        for gamma in FLIGHT_PATH_ANGLES_DEG:
            case_name = f"cone_launch_v{int(speed)}_g{int(gamma)}"
            path = root / f"{case_name}.prb"
            write_case(path, name=case_name, speed_mps=speed, flight_path_angle_deg=gamma)
            entries.append(f"- `{path.name}`: initial speed {speed:.0f} m/s, flight-path angle {gamma:.0f} deg")
    ####
    root.mkdir(parents=True, exist_ok=True)
    (root / "README.md").write_text("\n".join(entries) + "\n", encoding="utf-8")
    print(root)
####


if __name__ == "__main__":
    main()
####
