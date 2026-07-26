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
        [python, "tools/validate_daveml_equilibrium_trim.py"],
        [python, "tools/validate_daveml_hl20_load.py"],
        [python, "tools/validate_daveml_hl20_linearization.py"],
        [python, "tools/validate_daveml_hl20_scenario.py"],
        [python, "tools/validate_daveml_collection_roundtrip.py"],
        [python, "tools/validate_daveml_nesc_replay.py"],
        [python, "tools/validate_daveml_nesc_objectives.py"],
        [python, "tools/canonical_daveml_roundtrip.py", "--catalog-root", "INBOX/taoryx-daveml-nesc-model-catalog-v1.0", "--output-dir", "build/daveml-release-catalog", "--summary-output", "verification/daveml_catalog_roundtrip.json"],
        [python, "tools/evaluate_daveml_checkdata.py", "resources/aerospace/daveml/official-conformance-v1", "--output", "verification/daveml_official_checkdata.json"],
        [python, "tools/official_daveml_conformance.py", "resources/aerospace/daveml/official-conformance-v1", "--output", "verification/daveml_official_conformance.json"],
        [python, "tools/validate_daveml_linearization.py"],
        [python, "tools/validate_daveml_tuning.py"],
        [python, "tools/validate_daveml_scenario.py"],
        [python, "-m", "taoryx.runtime.cli", "daveml", "smoke", "--family", "reference_f16_s119"],
        [python, "-m", "taoryx.runtime.cli", "daveml", "smoke", "--family", "reference_hl20_mod_k"],
        [python, "-m", "taoryx.runtime.cli", "daveml", "smoke", "--family", "reference_nesc_two_stage_rocket"],
    ]
    runs = tuple(run(command) for command in commands)
    artifacts = (
        "verification/daveml_family_readiness.json",
        "verification/daveml_atmosphere_binding.json",
        "verification/daveml_f16_trim_evidence.json",
        "verification/daveml_f16_equilibrium_trim_evidence.json",
        "verification/daveml_hl20_trim_evidence.json",
        "verification/daveml_hl20_load_evidence.json",
        "verification/daveml_hl20_linearization_evidence.json",
        "verification/daveml_hl20_scenario_evidence.json",
        "verification/daveml_collection_roundtrip_evidence.json",
        "verification/daveml_nesc_replay_evidence.json",
        "verification/daveml_nesc_objectives_evidence.json",
        "verification/daveml_catalog_roundtrip.json",
        "verification/daveml_official_checkdata.json",
        "verification/daveml_official_conformance.json",
        "build/daveml-release-catalog/canonical/export-manifest.json",
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
