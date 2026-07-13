from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

from .loader import case_directory
from .models import CaseSpec


@dataclass(slots=True)
class RuntimeResult:
    returncode: int
    workdir: Path
    stdout: str
    stderr: str
####


def taos_executable() -> Path | None:
    value = os.environ.get("TAOS_EXE")
    return Path(value).expanduser().resolve() if value else None
####


def run_case(case: CaseSpec, root: Path | None = None, timeout: float = 120.0) -> RuntimeResult:
    executable = taos_executable()
    if executable is None:
        raise RuntimeError("TAOS_EXE is not set")
    ####
    source = case_directory(case, root) / "input"
    temporary = Path(tempfile.mkdtemp(prefix=f"taos-{case.id}-"))
    for item in source.iterdir():
        shutil.copy2(item, temporary / item.name)
    ####
    tables = [Path(relative).name for relative in case.table_files]
    problem = Path(case.problem_file).name
    completed = subprocess.run(
        [str(executable), *tables, problem],
        cwd=temporary,
        text=True,
        capture_output=True,
        timeout=timeout,
        check=False,
    )
    return RuntimeResult(completed.returncode, temporary, completed.stdout, completed.stderr)
####
