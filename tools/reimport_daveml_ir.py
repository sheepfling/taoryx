"""Re-import one exported DAVE-ML document in a fresh Python process."""

from __future__ import annotations

import argparse
from pathlib import Path

from taoryx.trajectory import build_daveml_ir, export_daveml_ir


def main() -> int:
    """Build and write the fresh-process semantic IR."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("payload", type=Path)
    parser.add_argument("--document-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    ir = build_daveml_ir(arguments.payload.read_bytes(), document_id=arguments.document_id)
    arguments.output.write_bytes(export_daveml_ir(ir))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
