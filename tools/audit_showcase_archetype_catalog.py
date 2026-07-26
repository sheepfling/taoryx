"""Validate the shared showcase-archetype catalog and emit a compact report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from taoryx.showcase import ShowcaseArchetypeCatalog

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CATALOG = ROOT / "verification/showcase_archetype_catalog.yaml"
####


def audit(path: Path = DEFAULT_CATALOG) -> dict[str, object]:
    """Parse and validate one catalog without running a vehicle simulation."""

    catalog = ShowcaseArchetypeCatalog(**yaml.safe_load(path.read_text(encoding="utf-8")))
    recipes = [
        {
            "id": recipe.id,
            "family": recipe.family,
            "priority": recipe.priority,
            "archetypes": list(recipe.archetypes),
            "lineage_required": recipe.lineage_required,
            "required_events": list(recipe.required_events),
        }
        for recipe in sorted(catalog.recipes, key=lambda item: item.priority)
    ]
    return {
        "schema_version": catalog.schema_version,
        "catalog_id": catalog.id,
        "archetype_count": len(catalog.archetypes),
        "recipe_count": len(catalog.recipes),
        "recipes": recipes,
        "status": "pass",
    }
    ####


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("catalog", type=Path, nargs="?", default=DEFAULT_CATALOG)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = audit(args.catalog)
    serialized = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output is None:
        print(serialized, end="")
    else:
        args.output.write_text(serialized, encoding="utf-8")
        print(args.output)
    ####


if __name__ == "__main__":
    main()
####
