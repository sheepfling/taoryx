from __future__ import annotations

import re
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
METADATA = ROOT / "metadata" / "output_variables_appendix.yaml"
GENERATED = ROOT / "backmatter" / "appendix_output_variables.tex"
APPENDIX = ROOT / "backmatter" / "appendix.tex"
EDITORIAL = ROOT / "metadata" / "editorial_notes_backmatter.yaml"
PROGRESS = ROOT / "metadata" / "progress.yaml"

EXPECTED_EDITORIAL_IDS = {
    "appendix_physical_order_toc_discrepancy",
    "appendix_ep_angle_units",
    "appendix_iip_long_description",
    "appendix_rcmdt_unit_and_reference",
    "appendix_tangent_plane_derivatives",
    "appendix_ecfc_direction_descriptions",
}


def _load_yaml(path: Path) -> Any:
    with path.open() as handle:
        return yaml.safe_load(handle)
    ####
####


def _all_latex_labels() -> set[str]:
    labels: set[str] = set()
    for path in ROOT.rglob("*.tex"):
        if path == GENERATED:
            continue
        ####
        labels.update(re.findall(r"\\label\{([^}]+)\}", path.read_text()))
    ####
    return labels
####


def main() -> None:
    payload = _load_yaml(METADATA)["appendix_output_variable_reference"]
    entries = list(payload["entries"])
    if payload["entry_count"] != 171 or len(entries) != 171:
        raise SystemExit("Appendix metadata must contain exactly 171 output-variable entries.")
    ####

    names = [str(item["name"]) for item in entries]
    if len(set(names)) != len(names):
        raise SystemExit("Appendix output-variable names are not unique.")
    ####
    if names[:3] != ["accebx", "acceby", "accebz"] or names[-3:] != ["ztp", "ztpdt", "ztpdt2"]:
        raise SystemExit("Appendix output-variable ordering does not match the source endpoints.")
    ####

    expected_pages = set(range(282, 293))
    source_pages = {int(item["source_pdf_page"]) for item in entries}
    if source_pages != expected_pages:
        raise SystemExit("Appendix entries do not cover every source page from 282 through 292.")
    ####
    if any(item["status"] != "visually_verified" for item in entries):
        raise SystemExit("One or more Appendix entries are not visually verified.")
    ####

    latex = GENERATED.read_text()
    generated_names = re.findall(r"\\statevar\{([^}]+)\}", latex)
    if generated_names != names:
        raise SystemExit("Generated Appendix rows are out of sync with the YAML metadata.")
    ####
    if latex.count(r"\begin{longtable}") != 11 or latex.count(r"\end{longtable}") != 11:
        raise SystemExit("Appendix output reference must contain 11 source-page-mapped longtables.")
    ####

    appendix_text = APPENDIX.read_text()
    if r"\placeholder" in appendix_text:
        raise SystemExit("Appendix still contains a scaffold placeholder.")
    ####
    for required in (
        "sec:table-file-format",
        "sec:trajectory-block-print",
        "sec:problem-block-print",
        "sec:block-units-fmt",
    ):
        if required not in appendix_text:
            raise SystemExit(f"Appendix does not connect to required documentation label {required}.")
        ####
    ####

    labels = _all_latex_labels()
    referenced = {
        label
        for item in entries
        for label in item.get("references", [])
    }
    missing_labels = sorted(referenced - labels)
    if missing_labels:
        raise SystemExit("Appendix metadata references missing LaTeX labels: " + ", ".join(missing_labels))
    ####

    by_name = {str(item["name"]): item for item in entries}
    expected_values: dict[str, tuple[str, int]] = {
        "alt": (r"\mathrm{ft}", 1),
        "latgddt": (r"\mathrm{deg/sec}", 7),
        "rho": (r"\mathrm{lb_m/ft^3}", 7),
        "reypft": (r"\mathrm{ft^{-1}}", 0),
        "rcmdt": (r"\mathrm{ft/sec}", 3),
        "ztpdt2": (r"\mathrm{ft/sec^2}", 4),
    }
    for name, expected in expected_values.items():
        actual = (str(by_name[name]["default_units_latex"]), int(by_name[name]["decimal_places"]))
        if actual != expected:
            raise SystemExit(f"Unexpected units or decimal places for {name}: {actual!r}.")
        ####
    ####

    editorial = _load_yaml(EDITORIAL)["editorial_notes"]
    editorial_ids = {str(item["id"]) for item in editorial}
    if not EXPECTED_EDITORIAL_IDS.issubset(editorial_ids):
        missing = sorted(EXPECTED_EDITORIAL_IDS - editorial_ids)
        raise SystemExit("Missing Appendix editorial notes: " + ", ".join(missing))
    ####

    progress = _load_yaml(PROGRESS)
    appendix_progress = progress["backmatter_parts"]["appendix_output_variable_reference"]
    if appendix_progress["status"] != "complete" or appendix_progress["entry_count"] != 171:
        raise SystemExit("Appendix progress metadata is incomplete.")
    ####

    print("Appendix output-variable reference validation passed.")
####


if __name__ == "__main__":
    main()
####
