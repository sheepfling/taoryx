"""Validate the complete resolved vehicle-interface catalog."""

from __future__ import annotations

import argparse
import json
from collections.abc import Sequence

from taoryx.plugins import PluginCatalog, discover_plugins
from taoryx.vehicle_interface import build_vehicle_interface_catalog_report


def build_report(
    *,
    family_ids: Sequence[str] | None = None,
    fidelity_ids: Sequence[str] | None = None,
    plugins: PluginCatalog | None = None,
) -> dict[str, object]:
    """Return the scoped interface report used by development and CI."""

    return build_vehicle_interface_catalog_report(
        family_ids=family_ids,
        fidelity_ids=fidelity_ids,
        plugins=plugins,
    )
    ####


def main(argv: Sequence[str] | None = None) -> int:
    """Print a compact result and fail when interface declarations drift."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if any resolved interface has a conformance finding")
    parser.add_argument(
        "--family",
        action="append",
        metavar="FAMILY_ID",
        help="limit validation to one vehicle family; repeat to select several families",
    )
    parser.add_argument(
        "--fidelity",
        action="append",
        metavar="FIDELITY_ID",
        help="limit validation to one fidelity tier; repeat to select several tiers",
    )
    parser.add_argument(
        "--plugin",
        action="append",
        metavar="PLUGIN_ID",
        help="use only named source-tree plug-in(s) for family-specific interface additions",
    )
    args = parser.parse_args(argv)
    plugins = None if not args.plugin else discover_plugins(include_external=False, selected=tuple(args.plugin))
    report = build_report(family_ids=args.family, fidelity_ids=args.fidelity, plugins=plugins)
    summary = {
        "schema": report["schema"],
        "status": report["status"],
        "family_count": report["family_count"],
        "interface_count": report["interface_count"],
        "error_count": report["error_count"],
        "family_filter": report["family_filter"],
        "fidelity_filter": report["fidelity_filter"],
        "plugin_ids": None if plugins is None else [item.id for item in plugins.plugins],
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if report["status"] == "pass" or not args.check else 2
    ####


if __name__ == "__main__":
    raise SystemExit(main())
