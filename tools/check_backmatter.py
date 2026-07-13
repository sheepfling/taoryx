from __future__ import annotations

import re
from collections import Counter
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
REFERENCES_METADATA = ROOT / "metadata" / "references.yaml"
INDEX_METADATA = ROOT / "metadata" / "index_entries.yaml"
DISTRIBUTION_METADATA = ROOT / "metadata" / "distribution.yaml"
EDITORIAL_METADATA = ROOT / "metadata" / "editorial_notes_backmatter.yaml"
PROGRESS_METADATA = ROOT / "metadata" / "progress.yaml"
REFERENCES_GENERATED = ROOT / "backmatter" / "references_generated.tex"
INDEX_GENERATED = ROOT / "backmatter" / "index_generated.tex"
DISTRIBUTION_GENERATED = ROOT / "backmatter" / "distribution_generated.tex"
WRAPPERS = [
    ROOT / "backmatter" / "references.tex",
    ROOT / "backmatter" / "index.tex",
    ROOT / "backmatter" / "distribution.tex",
]

EXPECTED_INDEX_TYPE_COUNTS = Counter(
    {
        "subentry": 318,
        "variable": 147,
        "subvariable": 145,
        "entry": 105,
        "subblock": 32,
        "letter": 23,
    }
)
EXPECTED_INDEX_LETTERS = [
    "A",
    "B",
    "C",
    "D",
    "E",
    "F",
    "G",
    "H",
    "I",
    "L",
    "M",
    "N",
    "O",
    "P",
    "R",
    "S",
    "T",
    "U",
    "V",
    "W",
    "X",
    "Y",
    "Z",
]
EXPECTED_EDITORIAL_IDS = {
    "appendix_physical_order_toc_discrepancy",
    "references_powell_page_range",
    "references_kwajalein_title",
    "index_density_name_preserved",
}


def _load_yaml(path: Path) -> Any:
    with path.open() as handle:
        return yaml.safe_load(handle)
    ####
####


def _check_references() -> None:
    payload = _load_yaml(REFERENCES_METADATA)["references"]
    entries = list(payload["entries"])
    if payload["entry_count"] != 23 or len(entries) != 23:
        raise SystemExit("References metadata must contain exactly 23 entries.")
    ####
    if [int(entry["number"]) for entry in entries] != list(range(1, 24)):
        raise SystemExit("Reference numbers are not sequential from 1 through 23.")
    ####
    if Counter(int(entry["source_pdf_page"]) for entry in entries) != Counter({294: 12, 295: 11}):
        raise SystemExit("Reference entries do not preserve the source Ref-1/Ref-2 split.")
    ####
    if any(entry["status"] != "visually_verified" for entry in entries):
        raise SystemExit("One or more reference entries are not visually verified.")
    ####

    latex = REFERENCES_GENERATED.read_text()
    targets = [int(value) for value in re.findall(r"\\hypertarget\{taos-ref-(\d+)\}", latex)]
    if targets != list(range(1, 24)):
        raise SystemExit("Generated reference targets are out of sync with metadata.")
    ####
    if latex.count(r"\taosbackmattertitle{References}") != 1:
        raise SystemExit("References must contain exactly one semantic backmatter title.")
    ####
    if "start=13" not in latex or "Source PDF page 295" not in latex:
        raise SystemExit("References do not preserve the two-page source pagination.")
    ####
####


def _check_index() -> None:
    payload = _load_yaml(INDEX_METADATA)["index"]
    pages = list(payload["pages"])
    if payload["content_page_count"] != 9 or len(pages) != 9:
        raise SystemExit("Index metadata must contain exactly nine content pages.")
    ####
    if [int(page["source_pdf_page"]) for page in pages] != list(range(296, 305)):
        raise SystemExit("Index content pages do not map to source pages 296 through 304.")
    ####
    if [str(page["source_manual_page"]) for page in pages] != [f"Index-{number}" for number in range(1, 10)]:
        raise SystemExit("Index manual-page labels are not sequential from Index-1 through Index-9.")
    ####
    if [bool(page["show_title"]) for page in pages] != [True] + [False] * 8:
        raise SystemExit("Only Index-1 should display the centered Index title.")
    ####

    items = [item for page in pages for side in ("left", "right") for item in page[side]]
    if len(items) != 770:
        raise SystemExit(f"Historical index must contain exactly 770 items, found {len(items)}.")
    ####
    type_counts = Counter(str(item["type"]) for item in items)
    if type_counts != EXPECTED_INDEX_TYPE_COUNTS:
        raise SystemExit(f"Unexpected index item-type counts: {type_counts!r}.")
    ####
    letters = [str(item["text"]) for item in items if item["type"] == "letter"]
    if letters != EXPECTED_INDEX_LETTERS:
        raise SystemExit("Index letter sequence does not match the source.")
    ####
    if not any(item["text"] == "density" for item in items):
        raise SystemExit("Source-faithful index term 'density' is missing.")
    ####

    blank = payload["blank_page"]
    if (
        payload["blank_page_count"] != 1
        or int(blank["source_pdf_page"]) != 305
        or str(blank["source_manual_page"]) != "Index-10"
        or str(blank["text"]) != "Intentionally Left Blank"
    ):
        raise SystemExit("Index-10 blank-page metadata is incorrect.")
    ####

    latex = INDEX_GENERATED.read_text()
    macros = re.findall(
        r"\\taosindex(?:letter|entry|subentry|variable|subvariable|subblock)\{",
        latex,
    )
    if len(macros) != 770:
        raise SystemExit("Generated historical index is out of sync with its 770 metadata items.")
    ####
    if latex.count(r"\taosbackmattertitle{Index}") != 1:
        raise SystemExit("Index must contain exactly one semantic backmatter title.")
    ####
    if latex.count(r"\begin{minipage}[t]{0.467\textwidth}") != 18:
        raise SystemExit("Index must contain two semantic columns on each of nine content pages.")
    ####
    if "Source PDF page 305; source manual page Index-10" not in latex:
        raise SystemExit("Generated Index-10 blank page is missing its source mapping.")
    ####
####


def _check_distribution() -> None:
    payload = _load_yaml(DISTRIBUTION_METADATA)["distribution"]
    entries = list(payload["entries"])
    if payload["entry_count"] != 35 or len(entries) != 35:
        raise SystemExit("Distribution metadata must contain exactly 35 recipient rows.")
    ####
    total_copies = sum(int(entry["copies"]) for entry in entries)
    if payload["total_copies"] != 60 or total_copies != 60:
        raise SystemExit("Distribution copy count must total 60.")
    ####
    if any(entry["status"] != "visually_verified" for entry in entries):
        raise SystemExit("One or more distribution entries are not visually verified.")
    ####
    if Counter(int(entry["group"]) for entry in entries) != Counter({1: 5, 2: 8, 3: 17, 4: 1, 5: 4}):
        raise SystemExit("Distribution group structure does not match the source page.")
    ####
    final_entry = entries[-1]
    if final_entry.get("recipient_lines") != ["Document Processing, 7613-2", "For DOE/OSTI"]:
        raise SystemExit("Final multiline distribution recipient is incorrect.")
    ####

    blank = payload["blank_page"]
    if (
        int(blank["source_pdf_page"]) != 307
        or str(blank["source_manual_page"]) != "Dist-2"
        or str(blank["text"]) != "Intentionally Left Blank"
    ):
        raise SystemExit("Dist-2 blank-page metadata is incorrect.")
    ####

    latex = DISTRIBUTION_GENERATED.read_text()
    rows = re.findall(r"^\d+ & MS \d{4} & .+ \\\\$", latex, flags=re.MULTILINE)
    if len(rows) != 35:
        raise SystemExit(f"Generated distribution must contain 35 rows, found {len(rows)}.")
    ####
    if latex.count(r"\taosbackmattertitle{Distribution}") != 1:
        raise SystemExit("Distribution must contain exactly one semantic backmatter title.")
    ####
    if r"\begin{tabular}[t]{@{}l@{}}Document Processing, 7613-2 \\ For DOE/OSTI\end{tabular}" not in latex:
        raise SystemExit("Final distribution recipient is not top-aligned as a two-line cell.")
    ####
####


def _check_scaffold_and_progress() -> None:
    for path in WRAPPERS:
        text = path.read_text()
        if r"\placeholder" in text:
            raise SystemExit(f"Backmatter wrapper still contains a placeholder: {path.name}.")
        ####
    ####
    generated_text = "\n".join(
        path.read_text()
        for path in (REFERENCES_GENERATED, INDEX_GENERATED, DISTRIBUTION_GENERATED)
    )
    if r"\placeholder" in generated_text:
        raise SystemExit("Generated backmatter still contains a placeholder.")
    ####

    editorial = _load_yaml(EDITORIAL_METADATA)["editorial_notes"]
    editorial_ids = {str(item["id"]) for item in editorial}
    if not EXPECTED_EDITORIAL_IDS.issubset(editorial_ids):
        missing = sorted(EXPECTED_EDITORIAL_IDS - editorial_ids)
        raise SystemExit("Missing backmatter editorial notes: " + ", ".join(missing))
    ####

    progress = _load_yaml(PROGRESS_METADATA)
    if progress["phases"]["backmatter"] != "complete":
        raise SystemExit("Backmatter phase is not marked complete.")
    ####
    parts = progress["backmatter_parts"]
    expected = {
        "references": (23, "entry_count"),
        "index": (770, "item_count"),
        "distribution": (35, "entry_count"),
    }
    for name, (count, count_key) in expected.items():
        part = parts[name]
        if part["status"] != "complete" or int(part[count_key]) != count:
            raise SystemExit(f"Backmatter progress metadata is incomplete for {name}.")
        ####
    ####
    if int(parts["index"]["content_page_count"]) != 9 or int(parts["index"]["blank_page_count"]) != 1:
        raise SystemExit("Index progress page counts are incorrect.")
    ####
    if int(parts["distribution"]["total_copies"]) != 60:
        raise SystemExit("Distribution progress copy count is incorrect.")
    ####
####


def main() -> None:
    _check_references()
    _check_index()
    _check_distribution()
    _check_scaffold_and_progress()
    print("References, historical Index, and Distribution validation passed.")
####


if __name__ == "__main__":
    main()
####
