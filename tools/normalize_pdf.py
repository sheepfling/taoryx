from __future__ import annotations

import argparse
import os
from pathlib import Path

from pypdf import PdfReader, PdfWriter


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Rewrite a PDF through pypdf to produce a conventional cross-reference table."
    )
    parser.add_argument("pdf", type=Path)
    return parser.parse_args()
####


def normalize_pdf(path: Path) -> None:
    if not path.exists() or path.stat().st_size == 0:
        raise SystemExit(f"PDF does not exist or is empty: {path}")
    ####

    reader = PdfReader(path, strict=False)
    if len(reader.pages) == 0:
        raise SystemExit(f"PDF has no readable pages: {path}")
    ####

    expected_pages = len(reader.pages)
    expected_labels = list(reader.page_labels)
    expected_title = str(reader.metadata.title or "") if reader.metadata else ""

    writer = PdfWriter(clone_from=reader)
    temporary = path.with_suffix(path.suffix + ".normalized.tmp")
    with temporary.open("wb") as handle:
        writer.write(handle)
    ####

    normalized = PdfReader(temporary, strict=True)
    if len(normalized.pages) != expected_pages:
        temporary.unlink(missing_ok=True)
        raise SystemExit(
            f"Normalization changed the page count: {expected_pages} -> {len(normalized.pages)}"
        )
    ####
    if list(normalized.page_labels) != expected_labels:
        temporary.unlink(missing_ok=True)
        raise SystemExit("Normalization changed the PDF page-label sequence.")
    ####
    normalized_title = str(normalized.metadata.title or "") if normalized.metadata else ""
    if normalized_title != expected_title:
        temporary.unlink(missing_ok=True)
        raise SystemExit("Normalization changed the PDF title metadata.")
    ####

    os.replace(temporary, path)
    print(f"Normalized {path} ({expected_pages} pages).")
####


def main() -> None:
    args = parse_args()
    normalize_pdf(args.pdf)
####


if __name__ == "__main__":
    main()
####
