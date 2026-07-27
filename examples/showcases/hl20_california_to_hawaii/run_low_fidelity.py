"""Run the low-fidelity HL-20 rocket-release showcase."""

from __future__ import annotations

import argparse
from pathlib import Path

from taoryx.hl20_reachability import write_hl20_fidelity_bundle


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts/showcases/hl20_california_to_hawaii/low_fidelity"),
    )
    args = parser.parse_args()
    bundle = write_hl20_fidelity_bundle(args.output_dir)
    print(bundle["manifest_path"])


if __name__ == "__main__":
    main()
