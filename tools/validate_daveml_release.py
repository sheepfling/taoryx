"""Run the bounded fresh-process DAVE-ML promotion gate."""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def file_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(command: list[str]) -> dict[str, object]:
    environment = dict(os.environ)
    environment["PYTHONPATH"] = str(ROOT / "src")
    process = subprocess.run(command, cwd=ROOT, env=environment, capture_output=True, text=True)
    return {"command": command, "returncode": process.returncode, "stdout_tail": process.stdout[-1000:], "stderr_tail": process.stderr[-1000:]}


def main() -> int:
    python = sys.executable
    commands = [
        [python, "tools/validate_daveml_family_readiness.py", "--readiness", "verification/daveml_family_readiness.yaml", "--output", "verification/daveml_family_readiness.json"],
        [python, "tools/validate_daveml_atmosphere.py"],
        [python, "tools/validate_daveml_trim.py"],
        [python, "tools/validate_daveml_linearization.py"],
        [python, "tools/validate_daveml_tuning.py"],
        [python, "tools/validate_daveml_scenario.py"],
        [python, "-m", "taoryx.runtime.cli", "daveml", "smoke", "--family", "reference_f16_s119"],
        [python, "-m", "taoryx.runtime.cli", "daveml", "smoke", "--family", "reference_hl20_mod_k"],
    ]
    runs = tuple(run(command) for command in commands)
    artifacts = (
        "verification/daveml_family_readiness.json",
        "verification/daveml_atmosphere_binding.json",
        "verification/daveml_f16_trim_evidence.json",
        "verification/daveml_hl20_trim_evidence.json",
        "verification/daveml_f16_linearization_evidence.json",
        "verification/daveml_f16_tuning_evidence.json",
        "verification/daveml_f16_scenario_evidence.json",
    )
    report = {
        "schema_version": "taoryx.daveml-release-gate/v1",
        "status": "verified" if all(run_result["returncode"] == 0 for run_result in runs) else "failed",
        "claim_boundary": "bounded source integration release gate; not flight qualification",
        "runs": list(runs),
        "artifacts": {path: file_hash(ROOT / path) for path in artifacts if (ROOT / path).is_file()},
    }
    output = ROOT / "verification/daveml_release_gate.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "verified" else 1


if __name__ == "__main__":
    raise SystemExit(main())

