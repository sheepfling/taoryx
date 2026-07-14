from __future__ import annotations

import argparse
from pathlib import Path

from taoryx.fixtures.spectre_trajectories import load_spec, write_workspace

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SPEC = ROOT / "tests" / "fixtures" / "problem_file_dumps" / "spectre_trajectories" / "spec.yaml"
DEFAULT_OUTPUT = ROOT / "tests" / "fixtures" / "problem_file_dumps" / "spectre_trajectories"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    workspace = load_spec(args.spec)
    write_workspace(workspace, args.output)
    print(f"Rendered Spectre trajectory dump workspace: {args.output.relative_to(ROOT)}")
    ####
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
