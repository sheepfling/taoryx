"""Create a project virtual environment and install a selected Taoryx suite."""

from __future__ import annotations

import argparse
import subprocess
import sys
import venv
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VENV = ROOT / ".venv"

PROFILE_PROJECTS: dict[str, tuple[str, ...]] = {
    "core": (),
    "cadac": (
        "packages/taoryx-cadac",
    ),
    "models": (
        "packages/taoryx-daveml",
        "packages/taoryx-simple-aero",
        "packages/taoryx-reference-models",
    ),
    "full": (
        "packages/taoryx-daveml",
        "packages/taoryx-simple-aero",
        "packages/taoryx-reference-models",
        "packages/taoryx-reachability",
    ),
}


def venv_python() -> Path:
    relative = Path("Scripts/python.exe") if sys.platform == "win32" else Path("bin/python")
    return VENV / relative


def run(command: list[str]) -> None:
    print("+", " ".join(command))
    subprocess.run(command, cwd=ROOT, check=True)
    ####
####


def editable_install_command(
    python: str,
    *,
    profile: str,
    with_dependencies: bool,
    with_sensors: bool,
) -> list[str]:
    """Build one pip command for the requested local distribution set."""

    command = [python, "-m", "pip", "install"]
    if not with_dependencies:
        command.extend(("--no-build-isolation", "--no-deps"))
    extras = ["dev"]
    if with_sensors:
        extras.append("sensors")
    root_spec = f".[{','.join(extras)}]" if with_dependencies else "."
    for project in (root_spec, *PROFILE_PROJECTS[profile]):
        command.extend(("-e", project))
    return command
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--upgrade-pip",
        action="store_true",
        help="upgrade pip inside the virtual environment before installing the project",
    )
    parser.add_argument(
        "--profile",
        choices=tuple(PROFILE_PROJECTS),
        default="full",
        help="distribution set to install (default: all official model and reachability plug-ins)",
    )
    parser.add_argument(
        "--with-sensors",
        action="store_true",
        help="also install the optional third-party sensor-model dependencies",
    )
    args = parser.parse_args()

    if not VENV.exists():
        print(f"Creating {VENV}")
        venv.EnvBuilder(with_pip=True, system_site_packages=True).create(VENV)
    else:
        print(f"Using existing {VENV}")

    python = str(venv_python())
    if args.upgrade_pip:
        try:
            run([python, "-m", "pip", "install", "--upgrade", "pip"])
        except subprocess.CalledProcessError:
            print("pip upgrade failed; continuing with the bundled version.")
        ####
    ####
    try:
        run(
            editable_install_command(
                python,
                profile=args.profile,
                with_dependencies=True,
                with_sensors=args.with_sensors,
            )
        )
    except subprocess.CalledProcessError:
        print("Falling back to an offline editable install without dependency resolution.")
        run(
            editable_install_command(
                python,
                profile=args.profile,
                with_dependencies=False,
                with_sensors=args.with_sensors,
            )
        )
    ####
    run([python, "-m", "taoryx.runtime.cli", "plugins", "check", "--profile", args.profile])
    activation = VENV / ("Scripts/activate" if sys.platform == "win32" else "bin/activate")
    print(f"\nBootstrap complete for the {args.profile!r} profile.")
    print(f"Activate with: source {activation}")
    print("Then run: python -m tools.dev doctor && python -m tools.dev test")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
