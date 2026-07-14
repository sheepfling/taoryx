from __future__ import annotations

import argparse
from pathlib import Path

from taoryx.fixtures.spectre_dumps import (
    load_catalog,
    load_spec,
    validate_catalog_against_workspace,
    write_workspace,
)

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SPEC = ROOT / "tests" / "fixtures" / "problem_file_dumps" / "spectre_segments" / "spec.yaml"
DEFAULT_CATALOG = ROOT / "tests" / "fixtures" / "problem_file_dumps" / "spectre_segments" / "catalog.yaml"
DEFAULT_OUTPUT = ROOT / "tests" / "fixtures" / "problem_file_dumps" / "spectre_segments"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    workspace = load_spec(args.spec)
    catalog = load_catalog(args.catalog)
    validate_catalog_against_workspace(catalog, workspace)
    write_workspace(workspace, args.output)
    print(f"Rendered Spectre problem-file dump workspace: {args.output.relative_to(ROOT)}")
    ####
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
