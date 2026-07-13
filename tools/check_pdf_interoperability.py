from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path

import yaml
from pypdf import PdfReader

ROOT = Path(__file__).resolve().parents[1]
PDF_PATH = ROOT / "build" / "manual.pdf"
PROGRESS_PATH = ROOT / "metadata" / "progress.yaml"


def _expected_page_count() -> int:
    payload = yaml.safe_load(PROGRESS_PATH.read_text(encoding="utf-8"))
    return int(payload["quality_assurance"]["final_manual_pages"])
####


def _run(command: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
    )
####


def main() -> None:
    if not PDF_PATH.exists() or PDF_PATH.stat().st_size == 0:
        raise SystemExit("build/manual.pdf is missing or empty.")
    ####

    expected_page_count = _expected_page_count()
    reader = PdfReader(PDF_PATH, strict=True)
    if len(reader.pages) != expected_page_count:
        raise SystemExit(
            f"Expected {expected_page_count} reconstructed pages, found {len(reader.pages)}."
        )
    ####
    labels = list(reader.page_labels)
    for required in ("1-1", "2-1", "3-1", "4-1", "A-1", "Ref-1", "Index-1", "Dist-1"):
        if required not in labels:
            raise SystemExit(f"Missing historical page label: {required}")
        ####
    ####

    pdfinfo = _run(["pdfinfo", str(PDF_PATH)])
    if pdfinfo.returncode != 0 or pdfinfo.stderr.strip():
        raise SystemExit(f"Poppler pdfinfo failed: {pdfinfo.stderr.strip()}")
    ####

    with tempfile.TemporaryDirectory(prefix="taos-pdf-check-") as directory:
        directory_path = Path(directory)
        text_output = directory_path / "manual.txt"
        pdftotext = _run(["pdftotext", str(PDF_PATH), str(text_output)])
        if pdftotext.returncode != 0 or pdftotext.stderr.strip():
            raise SystemExit(f"Poppler pdftotext failed: {pdftotext.stderr.strip()}")
        ####
        if not text_output.exists() or text_output.stat().st_size < 100_000:
            raise SystemExit("Poppler extracted unexpectedly little text from the manual.")
        ####

        for page_number in (1, 150, expected_page_count):
            prefix = directory_path / f"page-{page_number}"
            render = _run(
                [
                    "pdftoppm",
                    "-f",
                    str(page_number),
                    "-singlefile",
                    "-png",
                    "-r",
                    "72",
                    str(PDF_PATH),
                    str(prefix),
                ]
            )
            if render.returncode != 0 or render.stderr.strip():
                raise SystemExit(
                    f"Poppler render failed for page {page_number}: {render.stderr.strip()}"
                )
            ####
            output = prefix.with_suffix(".png")
            if not output.exists() or output.stat().st_size < 1_000:
                raise SystemExit(f"Poppler did not produce page {page_number} correctly.")
            ####
        ####
    ####

    print(
        "PDF interoperability checks passed: strict pypdf, pdfinfo, pdftotext, "
        "and selected Poppler renders."
    )
####


if __name__ == "__main__":
    main()
####
