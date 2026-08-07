"""Exercise every Mission Composition advertisement surface and feedback path."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.trajectory.mission_composition import (
    ContractProbeMissionCompositionProvider,
    MissionCompositionOutputSelection,
    MissionCompositionRunRequest,
    audit_provider_advertisement,
    build_contract_probe_configuration,
)


def main() -> int:
    """Audit, validate, run, and deliberately reject one request."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="write the complete proof bundle as JSON")
    parser.add_argument("--json", action="store_true", help="print the complete proof bundle")
    args = parser.parse_args()

    provider = ContractProbeMissionCompositionProvider()
    runner = provider.build_runner()
    audit = audit_provider_advertisement(provider, runner)
    configuration = build_contract_probe_configuration(provider)
    prepared = provider.validate_configuration(configuration)
    success = runner.run(
        MissionCompositionRunRequest(
            request_id="contract-probe-success",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )
    failure = runner.run(
        MissionCompositionRunRequest(
            request_id="contract-probe-failure",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(
                mode="selected",
                channels=("debug.channel.does_not_exist",),
            ),
        )
    )
    payload = {
        "provider": provider.metadata.model_dump(mode="json", by_alias=True),
        "model": provider.list_models()[0].model_dump(mode="json"),
        "configuration_schema": provider.get_model_schema(configuration.model_id).model_dump(mode="json", by_alias=True),
        "output_schema": provider.get_model_output_schema(configuration.model_id).model_dump(mode="json", by_alias=True),
        "advertisement_audit": audit.model_dump(mode="json", by_alias=True),
        "prepared_configuration": prepared.model_dump(mode="json", by_alias=True),
        "success_response": success.model_dump(mode="json", by_alias=True),
        "failure_response": failure.model_dump(mode="json", by_alias=True),
    }
    summary = {
        "advertisement_audit": audit.status,
        "configuration_nodes": ("parameter", "group", "choice", "sequence", "optional"),
        "model_properties": len(provider.list_models()[0].presentation.properties),
        "core_output_channels": len(provider.list_models()[0].output_schema.core_channels),
        "telemetry_channels": len(provider.list_models()[0].output_schema.telemetry_channels),
        "telemetry_groups": len(provider.list_models()[0].output_schema.telemetry_groups),
        "entity_relationships": len(success.result.relationships) if success.kind == "trajectory" else 0,
        "deployment_states": tuple(item.status for item in provider.list_models()[0].deployments),
        "success_kind": success.kind,
        "failure_kind": failure.kind,
    }
    print(json.dumps(payload if args.json else summary, indent=2, sort_keys=True))
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if not args.json:
            print(f"wrote {args.output}")
    return 0 if audit.status == "pass" and success.kind == "trajectory" and failure.kind == "failure" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
