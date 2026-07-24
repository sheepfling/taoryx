"""Build the composite TAORYX extension reference PDF."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import tempfile
from pathlib import Path

from pypdf import PdfReader, PdfWriter

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "output" / "pdf" / "taoryx_extensions_composite.pdf"
REFERENCE_SOURCES = (
    ROOT / "docs" / "extensions" / "README.md",
    ROOT / "docs" / "extensions" / "taoryx-language-reference.md",
    ROOT / "docs" / "extensions" / "problem-file-guide.md",
    ROOT / "docs" / "extensions" / "lqr.md",
    ROOT / "docs" / "extensions" / "terminal-guidance.md",
    ROOT / "docs" / "extensions" / "thermal-entry.md",
    ROOT / "docs" / "extensions" / "showcase-catalog.md",
)
BASE_GUIDE = ROOT / "output" / "pdf" / "taoryx_extensions_and_verification.pdf"
LANGUAGE_GUIDE = ROOT / "output" / "pdf" / "taoryx_language_reference.pdf"


def run(command: list[str], *, cwd: Path = ROOT) -> None:
    """Run one build command with the repository as its default directory."""

    print("+", " ".join(command))
    subprocess.run(command, cwd=cwd, check=True)
    ####
####


def reference_markdown(start_page: int) -> str:
    """Create the deterministic Markdown input for the reference appendix."""

    chunks = [
        "---",
        "title: TAORYX Extension Reference",
        "subtitle: Canonical Markdown extension contracts",
        "date: July 2026",
        "toc: true",
        "geometry: margin=0.8in",
        "---",
        "",
        "This appendix is generated from the canonical Markdown extension sources.",
        "The preceding composite-guide section contains the broader verification",
        "and controller-design narrative.",
        "",
        f"\\setcounter{{page}}{{{start_page}}}",
        "",
    ]
    for source in REFERENCE_SOURCES:
        if not source.is_file():
            raise FileNotFoundError(f"missing extension reference source: {source}")
        chunks.append(source.read_text(encoding="utf-8").rstrip())
        chunks.extend(("", "\\newpage", ""))
    return "\n".join(chunks).rstrip() + "\n"
    ####


def merge_pdfs(sources: tuple[Path, ...], output: Path) -> None:
    """Merge the built LaTeX guides and generated Markdown appendix in order."""

    writer = PdfWriter()
    for source in sources:
        reader = PdfReader(str(source))
        for page in reader.pages:
            writer.add_page(page)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as stream:
        writer.write(stream)
    ####


def build(output: Path = DEFAULT_OUTPUT) -> None:
    """Build the composite PDF from the current canonical documentation."""

    if not BASE_GUIDE.is_file() or not LANGUAGE_GUIDE.is_file():
        raise FileNotFoundError(
            "a TAORYX LaTeX guide is missing; run "
            "'python tools/dev.py taoryx-extension-pdf' first"
        )
    pandoc = shutil.which("pandoc")
    if pandoc is None:
        raise RuntimeError("pandoc is required to build the Markdown extension appendix")

    temporary_root = ROOT / "tmp" / "pdfs"
    temporary_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="taoryx-extension-pdf-", dir=temporary_root) as temporary:
        workspace = Path(temporary)
        source = workspace / "extension-reference.md"
        appendix = workspace / "extension-reference.pdf"
        guide_pages = len(PdfReader(str(BASE_GUIDE)).pages) + len(PdfReader(str(LANGUAGE_GUIDE)).pages)
        source.write_text(reference_markdown(guide_pages + 1), encoding="utf-8")
        run(
            [
                pandoc,
                "--from=markdown+raw_tex+pipe_tables+task_lists+autolink_bare_uris",
                "--standalone",
                "--toc",
                "--number-sections",
                "--pdf-engine=xelatex",
                "--output",
                str(appendix),
                str(source),
            ],
            cwd=workspace,
        )
        merge_pdfs((BASE_GUIDE, LANGUAGE_GUIDE, appendix), output)
    print(f"Wrote composite TAORYX extension PDF: {output}")
    ####


def main() -> int:
    """Parse the optional output path and build the composite."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    build(args.output if args.output.is_absolute() else ROOT / args.output)
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
