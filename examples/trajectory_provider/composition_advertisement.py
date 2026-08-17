"""Inspect or export the provider-neutral vehicle-composition advertisement."""

from __future__ import annotations

import argparse
import json
from collections import Counter

from taoryx.plugins import discover_plugins
from taoryx.trajectory.mission_composition import TrajectoryCompositionAdvertisement


def main() -> int:
    """Print one installed model's feature matrix or the standalone JSON Schema."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--provider",
        default="taoryx.registry.mission-composition",
        help="installed Mission Composition provider ID",
    )
    parser.add_argument("--model", default="simple_aero", help="model ID owned by the selected provider")
    output = parser.add_mutually_exclusive_group()
    output.add_argument("--json", action="store_true", help="emit the complete advertisement JSON")
    output.add_argument("--json-schema", action="store_true", help="emit the standalone structural JSON Schema")
    parser.add_argument(
        "--include-unavailable",
        action="store_true",
        help="include explicit not_available rows in the human-readable table",
    )
    args = parser.parse_args()

    if args.json_schema:
        print(json.dumps(TrajectoryCompositionAdvertisement.model_json_schema(by_alias=True), indent=2, sort_keys=True))
        return 0

    providers = discover_plugins().build_mission_composition_provider_registry()
    provider = providers.provider(args.provider)
    model = next((item for item in provider.list_models() if item.id == args.model), None)
    if model is None:
        available = ", ".join(item.id for item in provider.list_models())
        parser.error(f"provider {args.provider!r} does not advertise model {args.model!r}; available: {available}")

    advertisement = model.composition_advertisement
    # A backend or gateway can perform the same round trip at its JSON boundary.
    advertisement = TrajectoryCompositionAdvertisement.model_validate_json(
        advertisement.model_dump_json(by_alias=True)
    )

    if args.json:
        print(advertisement.model_dump_json(indent=2, by_alias=True))
        return 0

    counts = Counter(item.status for item in advertisement.features)
    status_order = ("available", "conditional", "declared", "blocked", "not_available")
    print(f"provider: {provider.metadata.id}@{provider.metadata.version}")
    print(f"model: {model.id}@{model.version}")
    print(f"schema: {advertisement.schema_id}")
    print("status counts: " + ", ".join(f"{status}={counts[status]}" for status in status_order))
    print()
    print("category | feature | status | operations | timing | scope")
    print("--- | --- | --- | --- | --- | ---")
    for feature in advertisement.features:
        if feature.status == "not_available" and not args.include_unavailable:
            continue
        selectors = (
            *(f"mission:{item}" for item in feature.mission_template_ids),
            *(f"fidelity:{item}" for item in feature.fidelity_ids),
            *(f"realization:{item}" for item in feature.realization_ids),
        )
        print(
            " | ".join(
                (
                    feature.category,
                    feature.id,
                    feature.status,
                    ",".join(feature.operations) or "-",
                    ",".join(feature.mutation_timing) or "-",
                    ",".join(selectors) or "all exact matching rows",
                )
            )
        )
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
