"""Generate reproducible evidence for the official DAVE-ML atmosphere fixture."""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.trajectory import load_daveml_atmosphere


def main() -> int:
    source = ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml"
    binding = load_daveml_atmosphere(source)
    altitudes_m = (0.0, 3048.0, 15240.0)
    report = {
        "schema_version": "taoryx.daveml-atmosphere-evidence/v1",
        "status": "verified",
        "claim_boundary": "source-bounded atmosphere channel adapter; no extrapolation",
        "model_id": "daveml-atmosphere-1976",
        "source": {"path": str(source.relative_to(ROOT)), "sha256": binding.source_sha256},
        "samples": [binding.evaluate(altitude) for altitude in altitudes_m],
    }
    output = ROOT / "verification/daveml_atmosphere_binding.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
