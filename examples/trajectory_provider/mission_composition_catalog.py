"""Inspect the registry-backed Mission Composition configuration contract."""

from __future__ import annotations

import argparse
import json

from taoryx.trajectory.mission_composition import (
    RegistryMissionCompositionProvider,
    audit_provider_advertisement,
    render_configuration_schema,
)


def main() -> int:
    """List all models or render one provider-independent configuration tree."""

    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--model", help="render the schema for one model ID")
    selection.add_argument("--audit", action="store_true", help="load and verify every advertised model schema")
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    args = parser.parse_args()

    provider = RegistryMissionCompositionProvider()
    if args.audit:
        report = audit_provider_advertisement(provider)
        if args.json:
            print(report.model_dump_json(indent=2, by_alias=True))
        else:
            print(f"{report.provider_id}: {report.status} ({report.validated_model_count}/{report.advertised_model_count} models)")
            for model in report.models:
                details = []
                if model.common_runner_operations:
                    details.append(f"common runner: {', '.join(model.common_runner_operations)}")
                if model.adapter_required_operations:
                    details.append(f"adapter required: {', '.join(model.adapter_required_operations)}")
                if model.deployment_ids:
                    details.append(f"deployments: {', '.join(model.deployment_ids)}")
                if model.deployment_adapter_required_ids:
                    details.append(f"deployment adapters required: {', '.join(model.deployment_adapter_required_ids)}")
                details.append(
                    f"metadata: {model.model_property_count} properties, {model.reference_frame_count} frames, "
                    f"{model.core_output_channel_count} core + {model.telemetry_channel_count} telemetry channels"
                )
                if model.supports_dynamic_spawning:
                    details.append("dynamic entity output")
                details.append(f"configuration: {', '.join(model.configuration_node_kinds)}")
                suffix = f"; {'; '.join(details)}" if details else ""
                print(f"  {model.model_id}: {model.status}{suffix}")
                for diagnostic in model.diagnostics:
                    print(f"    {diagnostic.code}: {diagnostic.message}")
        return 0 if report.status == "pass" else 1
    if args.model is None:
        if args.json:
            payload = {
                "provider": provider.metadata.model_dump(mode="json", by_alias=True),
                "models": [item.model_dump(mode="json") for item in provider.list_models()],
            }
            print(json.dumps(payload, indent=2, sort_keys=True))
        else:
            print(f"{provider.metadata.name} ({provider.metadata.model_count} models)")
            for model in provider.list_models():
                print(
                    f"{model.id}: {model.presentation.display_name} "
                    f"[{', '.join(model.operations)}; {len(model.presentation.properties)} properties; "
                    f"{len(model.output_schema.core_channels)} core + "
                    f"{len(model.output_schema.telemetry_channels)} telemetry channels]"
                )
        return 0

    model = provider.model(args.model)
    schema = provider.get_model_schema(args.model)
    output_schema = provider.get_model_output_schema(args.model)
    if args.json:
        print(
            json.dumps(
                {
                    "provider": provider.metadata.model_dump(mode="json", by_alias=True),
                    "model": model.model_dump(mode="json"),
                    "configuration_schema": schema.model_dump(mode="json", by_alias=True),
                    "configuration_schema_fingerprint": schema.fingerprint,
                    "output_schema": output_schema.model_dump(mode="json", by_alias=True),
                    "output_schema_fingerprint": output_schema.fingerprint,
                },
                indent=2,
                sort_keys=True,
            )
        )
    else:
        print(render_configuration_schema(schema))
        print("\nfidelity transitions:")
        for transition in model.fidelity_transitions:
            print(f"  {transition.from_fidelity} -> {transition.to_fidelity}: {transition.status} ({transition.selection_policy})")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
