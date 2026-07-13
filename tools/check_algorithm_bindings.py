"""Validate that every restored algorithm-catalog ID has a documented binding."""

from __future__ import annotations

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "metadata" / "algorithm_catalog" / "algorithms.json"
BINDINGS = ROOT / "docs" / "equations" / "implementation-bindings.md"
ID_RE = re.compile(r"`(TAOS-ALG-[A-Z0-9-]+)`")


def audit(catalog_path: Path = CATALOG, bindings_path: Path = BINDINGS) -> tuple[int, tuple[str, ...]]:
    """Return catalog count and IDs missing from the implementation ledger."""

    if not catalog_path.exists():
        inbox_catalog = ROOT / "INBOX" / "taos-algorithm-catalog-v1" / "catalog" / "algorithms.json"
        if inbox_catalog.exists():
            catalog_path = inbox_catalog
    if not catalog_path.exists():
        raise FileNotFoundError(f"authoritative catalog is missing: {catalog_path}")
    payload = json.loads(catalog_path.read_text(encoding="utf-8"))
    catalog_ids = {item["id"] for item in payload["algorithms"]}
    binding_ids = set(ID_RE.findall(bindings_path.read_text(encoding="utf-8")))
    return len(catalog_ids), tuple(sorted(catalog_ids - binding_ids))


def main() -> int:
    count, missing = audit()
    if missing:
        print(f"algorithm bindings: {count - len(missing)}/{count}; missing: {', '.join(missing)}")
        return 1
    print(f"algorithm bindings: {count}/{count}; all catalog IDs have documented bindings")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
####
