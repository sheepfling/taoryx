"""Create the project virtual environment and install development extras."""

from __future__ import annotations

import argparse
import subprocess
import sys
import venv
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VENV = ROOT / ".venv"


def venv_python() -> Path:
    relative = Path("Scripts/python.exe") if sys.platform == "win32" else Path("bin/python")
    return VENV / relative


def run(command: list[str]) -> None:
    print("+", " ".join(command))
    subprocess.run(command, cwd=ROOT, check=True)
    ####
####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--upgrade-pip",
        action="store_true",
        help="upgrade pip inside the virtual environment before installing the project",
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
        run([python, "-m", "pip", "install", "-e", ".[dev]"])
    except subprocess.CalledProcessError:
        print("Falling back to an offline editable install without dependency resolution.")
        run([python, "-m", "pip", "install", "--no-build-isolation", "--no-deps", "-e", "."])
    ####
    print(f"\nBootstrap complete. Activate with: source {VENV}/bin/activate")
    print("Then run: python tools/dev.py doctor && python tools/dev.py test")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
