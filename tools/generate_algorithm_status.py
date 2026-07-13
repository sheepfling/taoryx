"""Generate the tracked algorithm implementation-status ledger."""

from __future__ import annotations

import csv
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "metadata/algorithm_catalog/algorithms.json"
BINDINGS = ROOT / "docs/equations/implementation-bindings.md"
OUTPUT = ROOT / "metadata/algorithm_catalog/algorithm_status.csv"
ID_RE = re.compile(r"`(TAOS-ALG-[A-Z0-9-]+)`")


def generate() -> None:
    payload = json.loads(CATALOG.read_text(encoding="utf-8"))
    bound = set(ID_RE.findall(BINDINGS.read_text(encoding="utf-8")))
    rows = []
    for algorithm in payload["algorithms"]:
        algorithm_id = algorithm["id"]
        rows.append(
            {
                "id": algorithm_id,
                "catalog_status": algorithm["status"],
                "binding_documented": "true" if algorithm_id in bound else "false",
                "implementation_stage": "typed_binding" if algorithm_id in bound else "unbound",
                "target": f"{algorithm['implementation_target']['package']}.{algorithm['implementation_target']['module']}.{algorithm['implementation_target']['symbol']}",
                "source_pages": ";".join(algorithm["source"]["manual_pages"]),
                "test_plan_units": str(len(algorithm["test_plan"]["unit"])),
            }
        )
    with OUTPUT.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    generate()
####
