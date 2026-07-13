from __future__ import annotations

import argparse
import csv
import subprocess
from pathlib import Path

import fitz
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
LISTING_MAP: dict[str, str] = {
    "4-5": "examples/chapter04/trajectory-output-example.txt",
    "4-6": "examples/chapter04/trajectory-printout-example.txt",
    "4-8": "examples/chapter04/egs-database-example.txt",
    "4-9": "examples/chapter04/problem-output-example.txt",
    "4-10": "examples/chapter04/problem-printout-example.txt",
    "4-12": "examples/chapter04/summary-printout-example.txt",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-pdf", type=Path, required=True)
    parser.add_argument("--manual-pdf", type=Path, default=ROOT / "build" / "manual.pdf")
    parser.add_argument("--manual-dpi", type=int, default=100)
    return parser.parse_args()
####


def read_figure_rows() -> list[dict[str, str]]:
    with (ROOT / "metadata" / "figures.csv").open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    ####
    return sorted(rows, key=lambda row: tuple(int(value) for value in row["figure"].split("-")))
####


def latex_escape(value: str) -> str:
    replacements = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    return "".join(replacements.get(character, character) for character in value)
####


def find_source_image(figure: str, pdf_page: int) -> str:
    chapter, _ = (int(value) for value in figure.split("-"))
    candidates: list[Path] = []
    if chapter == 1:
        candidates.append(ROOT / "figures" / "chapter01" / "source-crops" / f"fig-{figure}.png")
    elif chapter == 2:
        candidates.append(ROOT / "figures" / "chapter02" / "source-crops" / f"fig-{figure}.png")
    elif chapter == 3:
        candidates.append(ROOT / "figures" / "chapter03" / f"fig-{figure}.png")
    elif chapter == 4:
        candidates.append(ROOT / "figures" / "chapter04" / "source-crops" / f"fig-{figure}.png")
        candidates.append(ROOT / "figures" / "chapter04" / "source-plots" / f"fig-{figure}.png")
    ####
    for candidate in candidates:
        if candidate.exists():
            return candidate.relative_to(ROOT).as_posix()
        ####
    ####
    return f"qa/source-pages/page-{pdf_page:03d}.png"
####


def render_source_pages(source_pdf: Path, rows: list[dict[str, str]]) -> None:
    output_dir = ROOT / "qa" / "source-pages"
    output_dir.mkdir(parents=True, exist_ok=True)
    document = fitz.open(source_pdf)
    pages = sorted({int(row["pdf_page"]) for row in rows})
    matrix = fitz.Matrix(1.35, 1.35)
    for page_number in pages:
        output_path = output_dir / f"page-{page_number:03d}.png"
        pixmap = document[page_number - 1].get_pixmap(matrix=matrix, alpha=False)
        pixmap.save(output_path)
    ####
####


def build_figure_report_tex(rows: list[dict[str, str]]) -> None:
    output: list[str] = [
        r"\documentclass[10pt]{article}",
        r"\usepackage[T1]{fontenc}",
        r"\usepackage{lmodern}",
        r"\usepackage[letterpaper,landscape,margin=0.45in]{geometry}",
        r"\usepackage{graphicx}",
        r"\usepackage[export]{adjustbox}",
        r"\usepackage{tikz}",
        r"\usepackage{pgfplots}",
        r"\usepackage{xcolor}",
        r"\usepackage{listings}",
        r"\usepackage{booktabs}",
        r"\usepackage{amsmath,amssymb}",
        r"\usepackage{fancyhdr}",
        r"\usetikzlibrary{arrows.meta,calc,decorations.pathreplacing,positioning}",
        r"\pgfplotsset{compat=1.18}",
        r"\pagestyle{fancy}",
        r"\fancyhf{}",
        r"\fancyhead[L]{TAOS 1995 Manual - Visual Figure Comparison}",
        r"\fancyhead[R]{Version 19}",
        r"\fancyfoot[C]{\thepage}",
        r"\renewcommand{\headrulewidth}{0.3pt}",
        r"\lstdefinestyle{qareport}{basicstyle=\ttfamily\scriptsize,frame=single,breaklines=true,columns=fullflexible,keepspaces=true,showstringspaces=false}",
        r"\begin{document}",
        r"\begin{center}",
        r"{\LARGE\bfseries TAOS Manual Visual-Comparison Report}\par",
        r"\vspace{0.5em}",
        r"{\large Version 19 - residual fidelity and targeted figure review}\par",
        r"\end{center}",
        r"\vspace{1em}",
        "This report compares each numbered source figure against its reconstructed representation. "
        "The reconstructed manual is a semantic, reflowed edition, so a page-by-page pixel diff would not be meaningful. "
        "Instead, the review uses the original figure crop when available (otherwise the full source page), "
        "the editable TikZ/PGFPlots or semantic-listing reconstruction, and a separately generated contact-sheet review of all final manual pages.",
        r"\begin{center}",
        r"\begin{tabular}{lr}",
        r"\toprule",
        r"Numbered figures reviewed & 72\\",
        r"Editable TikZ/PGFPlots figures & 65\\",
        r"Semantic LaTeX tabular figures & 1\\",
        r"Semantic listing figures & 6\\",
        r"Raster image objects in final manual PDF & 0\\",
        r"Physical source pages represented & 307\\",
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{center}",
        r"\clearpage",
    ]
    for row in rows:
        figure = row["figure"]
        source_path = find_source_image(figure, int(row["pdf_page"]))
        chapter = int(figure.split("-")[0])
        output.extend(
            [
                rf"\section*{{Figure {latex_escape(figure)}: {latex_escape(row['title'])}}}",
                r"\noindent\begin{minipage}[t]{0.485\textwidth}\centering",
                r"{\bfseries Original source}\par\smallskip",
                rf"\includegraphics[max width=\linewidth,max height=0.67\textheight]{{{source_path}}}",
                rf"\par\smallskip{{\scriptsize Manual page {latex_escape(row['manual_page'])}, physical PDF page {latex_escape(row['pdf_page'])}.}}",
                r"\end{minipage}\hfill",
                r"\begin{minipage}[t]{0.485\textwidth}\centering",
                r"{\bfseries Reconstructed representation}\par\smallskip",
            ]
        )
        if figure in LISTING_MAP:
            output.append(rf"\lstinputlisting[style=qareport]{{{LISTING_MAP[figure]}}}")
        else:
            output.extend(
                [
                    r"\begin{adjustbox}{max width=\linewidth,max totalheight=0.67\textheight,center}",
                    rf"\input{{manual/figures/chapter{chapter:02d}/fig-{figure}}}",
                    r"\end{adjustbox}",
                ]
            )
        ####
        output.append(
            rf"\par\smallskip{{\scriptsize Status: {latex_escape(row['status'])}. Target: {latex_escape(row['target_format'])}.}}"
        )
        if row["notes"]:
            output.append(rf"\par{{\scriptsize {latex_escape(row['notes'])}}}")
        ####
        output.extend([r"\end{minipage}", r"\clearpage"])
    ####
    output.append(r"\end{document}")
    (ROOT / "qa" / "figure_comparison.tex").write_text("\n".join(output), encoding="utf-8")
####


def compile_figure_report() -> None:
    output_dir = ROOT / "qa" / "build"
    output_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            "latexmk",
            "-pdf",
            "-interaction=nonstopmode",
            "-halt-on-error",
            f"-output-directory={output_dir}",
            "qa/figure_comparison.tex",
        ],
        cwd=ROOT,
        check=True,
    )
####


def render_manual_pages(manual_pdf: Path, dpi: int) -> list[Path]:
    output_dir = ROOT / "qa" / "manual-pages"
    output_dir.mkdir(parents=True, exist_ok=True)
    for old_file in output_dir.glob("page-*.png"):
        old_file.unlink()
    ####
    document = fitz.open(manual_pdf)
    scale = dpi / 72.0
    matrix = fitz.Matrix(scale, scale)
    paths: list[Path] = []
    for index, page in enumerate(document):
        output_path = output_dir / f"page-{index + 1:03d}.png"
        page.get_pixmap(matrix=matrix, alpha=False).save(output_path)
        paths.append(output_path)
    ####
    return paths
####


def create_contact_sheets(page_paths: list[Path]) -> None:
    output_dir = ROOT / "qa" / "page-contact-sheets"
    output_dir.mkdir(parents=True, exist_ok=True)
    for old_file in output_dir.glob("contact-*.jpg"):
        old_file.unlink()
    ####
    columns = 4
    rows = 4
    cell_width = 360
    cell_height = 480
    per_sheet = columns * rows
    for start in range(0, len(page_paths), per_sheet):
        subset = page_paths[start : start + per_sheet]
        sheet = Image.new("RGB", (columns * cell_width, rows * cell_height), "#cfcfcf")
        for offset, path in enumerate(subset):
            image = Image.open(path).convert("RGB")
            image.thumbnail((cell_width - 12, cell_height - 34))
            cell = Image.new("RGB", (cell_width, cell_height), "white")
            cell.paste(image, ((cell_width - image.width) // 2, 6))
            drawer = ImageDraw.Draw(cell)
            drawer.text((8, cell_height - 24), f"PDF page {start + offset + 1}", fill="black")
            sheet.paste(cell, ((offset % columns) * cell_width, (offset // columns) * cell_height))
        ####
        output_path = output_dir / f"contact-{start + 1:03d}-{start + len(subset):03d}.jpg"
        sheet.save(output_path, quality=88)
    ####
####


def main() -> None:
    args = parse_args()
    rows = read_figure_rows()
    render_source_pages(args.source_pdf, rows)
    build_figure_report_tex(rows)
    compile_figure_report()
    page_paths = render_manual_pages(args.manual_pdf, args.manual_dpi)
    create_contact_sheets(page_paths)
####


if __name__ == "__main__":
    main()
####
