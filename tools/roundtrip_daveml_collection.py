"""Run the lossless/source-preserving portion of a Taoryx collection round trip."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from taoryx.trajectory import export_source_preserving_collection


def build_parser() -> argparse.ArgumentParser:
    """Build the source-preserving round-trip CLI."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collection", type=Path, required=True, help="Input .txcollection archive")
    parser.add_argument("--output-dir", type=Path, required=True, help="Directory for exported source and report")
    return parser
    ####


def main(argv: list[str] | None = None) -> int:
    """Export source bytes and print the machine-readable report."""

    args = build_parser().parse_args(argv)
    report = export_source_preserving_collection(args.collection.resolve(), args.output_dir.resolve())
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "verified_source_preserving" else 1
    ####


if __name__ == "__main__":
    sys.exit(main())
