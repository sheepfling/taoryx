"""Compare a deterministic IMU fixture with an explicit upstream profile."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.imu_profile_comparison import compare_imu_profiles

CATALOG = ROOT / "resources/sensors/imu_profiles/catalog.json"
DEFAULT_BASELINE = ROOT / "tests/fixtures/imu_profiles/taoryx_demo.yaml"
DEFAULT_UPSTREAM_ROOT = ROOT.parent.parent / "imu-error-model"
DEFAULT_CANDIDATE = DEFAULT_UPSTREAM_ROOT / "examples/imu_profiles/hardware-estimates/hg1700ag58.yaml"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_commit(path: Path) -> str:
    result = subprocess.run(
        ["git", "-C", str(path), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    parser.add_argument("--candidate", type=Path, default=DEFAULT_CANDIDATE)
    parser.add_argument("--upstream-root", type=Path, default=DEFAULT_UPSTREAM_ROOT)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=41)
    parser.add_argument("--horizon-s", type=float, default=1.0)
    arguments = parser.parse_args()

    catalog = json.loads(CATALOG.read_text(encoding="utf-8"))
    expected_commit = str(catalog["source_commit"])
    actual_commit = _git_commit(arguments.upstream_root.resolve())
    if actual_commit != expected_commit:
        raise SystemExit(f"upstream imu-error-model checkout is not pinned: expected {expected_commit}, got {actual_commit}")
    baseline = arguments.baseline.resolve()
    candidate = arguments.candidate.resolve()
    if not baseline.is_file() or not candidate.is_file():
        raise SystemExit("both --baseline and --candidate must name existing profile files")
    comparison = compare_imu_profiles(baseline, candidate, seed=arguments.seed, horizon_s=arguments.horizon_s).as_dict()
    comparison["source_contract"] = {
        "catalog_path": str(CATALOG.resolve()),
        "catalog_sha256": _sha256(CATALOG),
        "source_repository": catalog["source_repository"],
        "expected_commit": expected_commit,
        "actual_commit": actual_commit,
        "checkout_path": str(arguments.upstream_root.resolve()),
    }
    comparison["inputs"] = {
        "baseline_path": str(baseline),
        "baseline_sha256": _sha256(baseline),
        "candidate_path": str(candidate),
        "candidate_sha256": _sha256(candidate),
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(comparison, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(comparison, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
