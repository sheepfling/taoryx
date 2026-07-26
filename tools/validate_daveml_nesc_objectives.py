"""Score the retained NESC Scenario 17 benchmark as family objectives."""

from __future__ import annotations

import hashlib
import json
import sys
import zipfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from taoryx.objectives import ObjectiveSpec, score_objectives

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "resources/aerospace/daveml/nesc-model-catalog-v1.0/qualified/nesc-two-stage-rocket/nesc-two-stage-rocket-v0.9.txair"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    """Evaluate source-retained checkpoint and schedule objectives."""

    with zipfile.ZipFile(PACKAGE) as archive:
        acceptance = json.loads(archive.read("validation/acceptance.json"))
    benchmark = acceptance["benchmark"]
    objectives = (
        ObjectiveSpec("scenario17-altitude-error", "trajectory", "altitude_maximum_absolute_error_m", 10000.0, 1.0, "m", comparison="maximum", source="summary"),
        ObjectiveSpec("scenario17-pitch-error", "trajectory", "pitch_maximum_absolute_error_deg", 4.0, 1.0, "deg", comparison="maximum", source="summary"),
        ObjectiveSpec("scenario17-speed-error", "trajectory", "speed_maximum_absolute_error_mps", 25.0, 1.0, "m/s", comparison="maximum", source="summary"),
        ObjectiveSpec("scenario17-schedule", "staging", "schedule_passed", True, None, "event", comparison="event"),
    )
    observed = {
        "altitude_maximum_absolute_error_m": benchmark["altitude_maximum_absolute_error_m"],
        "pitch_maximum_absolute_error_deg": benchmark["pitch_maximum_absolute_error_deg"],
        "speed_maximum_absolute_error_mps": benchmark["speed_maximum_absolute_error_mps"],
    }
    scored = score_objectives(
        objectives,
        observed,
        time_s=200.0,
        completed_events={"scenario17-schedule"} if acceptance["schedule_passed"] else set(),
        termination={"status": "completed", "reason": "retained package acceptance benchmark"},
        scenario_contract_sha256=_sha256(PACKAGE),
    )
    report = {
        "schema_version": "taoryx.daveml-nesc-objectives/v1",
        "status": scored["status"],
        "claim_boundary": "source-retained NESC Scenario 17 checkpoint objective qualification; not independent trajectory equivalence",
        "family_id": "reference_nesc_two_stage_rocket",
        "package_sha256": _sha256(PACKAGE),
        "acceptance": {"benchmark_passed": acceptance["benchmark_passed"], "schedule_passed": acceptance["schedule_passed"]},
        "objective_report": scored,
    }
    output = ROOT / "verification/daveml_nesc_objectives_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if scored["status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
