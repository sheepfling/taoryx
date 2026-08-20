"""Build and inspect the standalone trajectory-contracts wheel in isolation."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "packages" / "taoryx-trajectory-contracts"


def _metadata_text(wheel: Path) -> str:
    """Read the one wheel metadata file or fail with an actionable package error."""

    with zipfile.ZipFile(wheel) as archive:
        names = tuple(name for name in archive.namelist() if name.endswith(".dist-info/METADATA"))
        if len(names) != 1:
            raise ValueError(f"wheel {wheel.name!r} must contain one METADATA file, found {names!r}")
        return archive.read(names[0]).decode("utf-8")
    ####


def _wheel_paths(wheel: Path) -> tuple[str, ...]:
    """Return normalized archive paths for package-content assertions."""

    with zipfile.ZipFile(wheel) as archive:
        return tuple(sorted(archive.namelist()))
    ####


def build_report(*, python: str = sys.executable) -> dict[str, Any]:
    """Build a fresh wheel and verify its independent package boundary."""

    errors: list[str] = []
    with tempfile.TemporaryDirectory(prefix="taoryx-trajectory-contracts-") as temporary:
        output = Path(temporary)
        build = subprocess.run(
            [
                python,
                "-m",
                "build",
                "--wheel",
                "--no-isolation",
                "--outdir",
                str(output),
                str(PROJECT),
            ],
            cwd=ROOT,
            text=True,
            capture_output=True,
            check=False,
        )
        if build.returncode != 0:
            errors.append(f"wheel build failed: {build.stderr.strip() or build.stdout.strip()}")
            return {"schema": "taoryx.trajectory-contracts-wheel-check/v1", "status": "fail", "errors": errors}
        wheels = tuple(output.glob("taoryx_trajectory_contracts-*.whl"))
        if len(wheels) != 1:
            errors.append(f"expected one trajectory-contracts wheel, found {[item.name for item in wheels]!r}")
            return {"schema": "taoryx.trajectory-contracts-wheel-check/v1", "status": "fail", "errors": errors}
        wheel = wheels[0]
        metadata = _metadata_text(wheel)
        paths = _wheel_paths(wheel)
        requirements = tuple(line.removeprefix("Requires-Dist: ") for line in metadata.splitlines() if line.startswith("Requires-Dist: "))
        if not requirements or not all(item.lower().startswith("pydantic") for item in requirements):
            errors.append(f"contract wheel must depend only on Pydantic, found {requirements!r}")
        if any(item.startswith("taoryx/") for item in paths):
            errors.append("contract wheel must not ship the TAORYX runtime namespace")
        if "taoryx_trajectory_contracts/py.typed" not in paths:
            errors.append("contract wheel must include taoryx_trajectory_contracts/py.typed")
        import_check = subprocess.run(
            [
                python,
                "-I",
                "-c",
                (
                    "import sys; "
                    f"sys.path.insert(0, {str(wheel)!r}); "
                    "import taoryx_trajectory_contracts as contracts; "
                    "assert contracts.StandardEcefState.model_fields['frame_id'].default == 'ecfc'; "
                    "assert 'taoryx' not in sys.modules"
                ),
            ],
            cwd=ROOT,
            text=True,
            capture_output=True,
            check=False,
        )
        if import_check.returncode != 0:
            errors.append(f"wheel standalone import failed: {import_check.stderr.strip() or import_check.stdout.strip()}")
        return {
            "schema": "taoryx.trajectory-contracts-wheel-check/v1",
            "status": "pass" if not errors else "fail",
            "wheel": wheel.name,
            "requirements": list(requirements),
            "standalone_import": import_check.returncode == 0,
            "errors": errors,
        }
    ####


def main(argv: list[str] | None = None) -> int:
    """Build one fresh standalone wheel and return its compact verification report."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python", default=sys.executable, help="Python interpreter used to build and inspect the wheel")
    args = parser.parse_args(argv)
    report = build_report(python=args.python)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "pass" else 2
    ####


if __name__ == "__main__":
    raise SystemExit(main())
