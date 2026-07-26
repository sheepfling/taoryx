"""Replay the authoritative NESC two-stage DAVE-ML package in a fresh process."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    package = ROOT / "INBOX/taoryx-daveml-nesc-model-catalog-v1.0/qualified/nesc-two-stage-rocket/nesc-two-stage-rocket-v0.9.txair"
    environment = dict(os.environ)
    environment["PYTHONPATH"] = str(ROOT / "src")
    process = subprocess.run(
        [sys.executable, "tools/replay_daveml_reference.py", str(package)],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
    )
    payload = json.loads(process.stdout) if process.stdout.strip() else {}
    model = payload.get("models", [{}])[0]
    report = {
        "schema_version": "taoryx.daveml-nesc-replay-evidence/v1",
        "status": "verified" if process.returncode == 0 and model.get("status") == "runtime_replay_qualification_passed" else "failed",
        "claim_boundary": "fresh-process NESC package replay and benchmark acceptance; not independent full trajectory equivalence",
        "family_id": "reference_nesc_two_stage_rocket",
        "package": str(package.relative_to(ROOT)).replace("\\", "/"),
        "subprocess_returncode": process.returncode,
        "replay": model,
        "stderr_tail": process.stderr[-1000:],
    }
    output = ROOT / "verification/daveml_nesc_replay_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "verified" else 1


if __name__ == "__main__":
    raise SystemExit(main())
