"""Evidence-backed maturity projection for the Product 3 composition surface.

This is a catalog audit, not a score that erases family-specific evidence.  It
joins already authoritative Product 3 records so an author or release process
can distinguish a complete declared contract from a runnable endpoint, a
registered batch/episode parity witness, or a qualified vehicle model.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Mapping
from pathlib import Path

from .composition_result_catalog import index_composition_results
from .family_strategy import build_family_strategy_worklist
from .vehicle_composition_registry import (
    ResolvedVehicleCompositionCatalog,
    build_vehicle_composition_topology_report,
    load_resolved_vehicle_composition_catalog,
)
from .vehicle_execution_bindings import load_vehicle_execution_binding_catalog
from .vehicle_execution_parity_witnesses import validate_vehicle_execution_parity_witnesses
from .vehicle_execution_preflight import build_semantic_preflight_handler_report


def build_product_three_maturity_report(
    catalog: ResolvedVehicleCompositionCatalog | None = None,
    *,
    check_execution_witnesses: bool = False,
    execute_batch_witnesses: bool = False,
    execute_parity_witnesses: bool = False,
    results_directory: str | Path | None = None,
) -> dict[str, object]:
    """Join Product 3 coverage evidence without inferring evidence promotion.

    ``check_execution_witnesses`` is opt-in because it compiles every checked-in
    public endpoint witness and opens each declared episode.
    ``execute_batch_witnesses`` additionally runs each declared batch witness,
    including its exact action-trace disposition check, and therefore implies
    the endpoint check. ``results_directory`` is likewise opt-in: it only
    indexes existing artifacts and never treats an omitted result directory as
    completion evidence. The default is a fast catalog audit; it reports both
    evidence dimensions as absent rather than treating endpoint declarations as
    execution proof. ``execute_parity_witnesses`` independently replays one
    declared action trace through every exact pair whose parity evidence is
    registered; it never manufactures evidence for unregistered pairs.
    """

    selected = catalog or load_resolved_vehicle_composition_catalog()
    execution_catalog = load_vehicle_execution_binding_catalog()
    topology = build_vehicle_composition_topology_report(selected)
    semantic_preflight_handlers = build_semantic_preflight_handler_report(selected)
    authoring = selected.authoring_worklist_dict()
    vehicles = authoring.get("vehicles")
    if not isinstance(vehicles, list):
        raise ValueError("Product 3 authoring catalog has no vehicle worklists")
    variant_space_status_counts = authoring.get("variant_space_status_counts")
    runnable_variant_count = authoring.get("runnable_variant_count")
    if not isinstance(variant_space_status_counts, Mapping) or not all(
        isinstance(key, str) and isinstance(value, int) for key, value in variant_space_status_counts.items()
    ):
        raise ValueError("Product 3 authoring catalog has invalid variant-space status counts")
    if not isinstance(runnable_variant_count, int) or runnable_variant_count < 0:
        raise ValueError("Product 3 authoring catalog has invalid runnable-variant count")

    tiers: list[Mapping[str, object]] = []
    graph_statuses: Counter[str] = Counter()
    for vehicle in vehicles:
        if not isinstance(vehicle, Mapping):
            raise ValueError("Product 3 authoring catalog has an invalid vehicle worklist")
        missions = vehicle.get("missions")
        if not isinstance(missions, list):
            raise ValueError("Product 3 authoring vehicle worklist has no missions")
        for mission in missions:
            if not isinstance(mission, Mapping):
                raise ValueError("Product 3 authoring catalog has an invalid mission worklist")
            graph_status = mission.get("graph_status")
            if isinstance(graph_status, str):
                graph_statuses[graph_status] += 1
            mission_tiers = mission.get("tiers")
            if not isinstance(mission_tiers, list):
                raise ValueError("Product 3 authoring mission worklist has no fidelity tiers")
            tiers.extend(item for item in mission_tiers if isinstance(item, Mapping))

    operation_counts: Counter[str] = Counter()
    execution_endpoint_mode_counts: Counter[str] = Counter(item.execution_mode for item in execution_catalog.bindings)
    parity_counts: Counter[str] = Counter()
    status_counts: Counter[str] = Counter()
    batch_action_trace_disposition_counts: Counter[str] = Counter()
    execution_mode_counts: Counter[str] = Counter()
    fidelity_promotion_blocker_counts: Counter[str] = Counter()
    planned_execution_blocker_counts: Counter[str] = Counter()
    planned_execution_binding_count = 0
    capability_adapter_count = 0
    for tier in tiers:
        status = tier.get("status")
        if isinstance(status, str):
            status_counts[status] += 1
        operations = tier.get("runnable_operations")
        if isinstance(operations, list):
            operation_counts.update(item for item in operations if isinstance(item, str))
        parity = tier.get("batch_episode_parity")
        if isinstance(parity, Mapping) and isinstance(parity.get("availability"), str):
            parity_counts[str(parity["availability"])] += 1
        trace_dispositions = tier.get("batch_action_trace_dispositions")
        if isinstance(trace_dispositions, list):
            batch_action_trace_disposition_counts.update(item for item in trace_dispositions if isinstance(item, str))
        execution_modes = tier.get("execution_modes")
        if isinstance(execution_modes, list):
            execution_mode_counts.update(item for item in execution_modes if isinstance(item, str))
        promotion_blockers = tier.get("fidelity_promotion_blockers")
        if isinstance(promotion_blockers, list):
            fidelity_promotion_blocker_counts.update(item for item in promotion_blockers if isinstance(item, str))
        planned_blockers = tier.get("planned_execution_blockers")
        if isinstance(planned_blockers, Mapping):
            for blockers in planned_blockers.values():
                if not isinstance(blockers, list):
                    continue
                planned_execution_binding_count += 1
                planned_execution_blocker_counts.update(item for item in blockers if isinstance(item, str))
        if isinstance(tier.get("mission_capability_adapter"), str):
            capability_adapter_count += 1

    parameter_maturity = topology.get("parameter_contract_maturity")
    if not isinstance(parameter_maturity, Mapping):
        raise ValueError("Product 3 topology report has no parameter-contract maturity record")
    canonical_channel_coverage = topology.get("canonical_channel_coverage")
    resource_channel_coverage = topology.get("resource_channel_coverage")
    if not isinstance(canonical_channel_coverage, Mapping) or not isinstance(resource_channel_coverage, Mapping):
        raise ValueError("Product 3 topology report has no canonical-channel coverage record")
    topology_runtime_backed_variants = parameter_maturity.get("runtime_backed_variant_count")
    if not isinstance(topology_runtime_backed_variants, int) or topology_runtime_backed_variants < 0:
        raise ValueError("Product 3 topology report has invalid runtime-backed-variant count")
    expected_variant_space_status_counts: Counter[str] = Counter()
    expected_runnable_variant_count = 0
    for vehicle in vehicles:
        if not isinstance(vehicle, Mapping):
            raise ValueError("Product 3 authoring catalog has an invalid vehicle worklist")
        variant_worklist = vehicle.get("variant_worklist")
        if not isinstance(variant_worklist, Mapping):
            raise ValueError("Product 3 authoring vehicle worklist has no variant admission")
        state = variant_worklist.get("status")
        records = variant_worklist.get("variants")
        if not isinstance(state, str) or not isinstance(records, list):
            raise ValueError("Product 3 authoring vehicle variant admission is invalid")
        expected_variant_space_status_counts[state] += 1
        expected_runnable_variant_count += sum(
            1 for item in records if isinstance(item, Mapping) and item.get("status") == "runnable"
        )
    variant_admission_status = (
        "pass"
        if dict(sorted(expected_variant_space_status_counts.items())) == dict(sorted(variant_space_status_counts.items()))
        and expected_runnable_variant_count == runnable_variant_count == topology_runtime_backed_variants
        else "fail"
    )
    integration_worklist = build_family_strategy_worklist().as_dict()
    integration_items = integration_worklist.get("items")
    if not isinstance(integration_items, list):
        raise ValueError("family integration worklist has no item list")
    integration_status_counts: Counter[str] = Counter()
    integration_next_action_counts: Counter[str] = Counter()
    for item in integration_items:
        if not isinstance(item, Mapping):
            raise ValueError("family integration worklist contains an invalid item")
        item_status = item.get("status")
        if isinstance(item_status, str):
            integration_status_counts[item_status] += 1
        next_action = item.get("next_action")
        if isinstance(next_action, str):
            integration_next_action_counts[next_action] += 1

    witness: dict[str, object]
    if check_execution_witnesses or execute_batch_witnesses:
        from .vehicle_execution_witnesses import validate_vehicle_execution_witnesses

        witness = validate_vehicle_execution_witnesses(execute_batch=execute_batch_witnesses)
    else:
        witness = {
            "status": "not_checked",
            "claim_boundary": (
                "The fast Product 3 maturity report does not compile endpoint witnesses. "
                "Run with check_execution_witnesses=true to test declared endpoint construction."
            ),
        }

    concrete_capability_preflight: dict[str, object]
    episode_contract_conformance: dict[str, object]
    release_packet_conformance: dict[str, object]
    witness_records = witness.get("records")
    if not isinstance(witness_records, list):
        concrete_capability_preflight = {
            "status": "not_checked",
            "claim_boundary": (
                "Concrete capability estimates are checked only while compiling the endpoint-witness matrix. "
                "Run with check_execution_witnesses=true to validate them."
            ),
        }
        episode_contract_conformance = {
            "status": "not_checked",
            "claim_boundary": (
                "Episode semantic-contract conformance is checked only while opening the endpoint-witness matrix. "
                "Run with check_execution_witnesses=true to validate it."
            ),
        }
        release_packet_conformance = {
            "status": "not_checked",
            "claim_boundary": (
                "Release-packet conformance is checked only while executing public batch witnesses. "
                "Run with execute_batch_witnesses=true to validate generated packets."
            ),
        }
    else:
        feasibility_counts: Counter[str] = Counter()
        adapter_ids: set[str] = set()
        evidence_count = 0
        episode_projection_counts: Counter[str] = Counter()
        episode_contract_count = 0
        episode_contract_failures = 0
        for record in witness_records:
            if not isinstance(record, Mapping):
                continue
            evidence = record.get("capability_preflight")
            if not isinstance(evidence, Mapping):
                continue
            feasibility = evidence.get("feasibility")
            adapter_id = evidence.get("adapter_id")
            if isinstance(feasibility, str):
                feasibility_counts[feasibility] += 1
            if isinstance(adapter_id, str):
                adapter_ids.add(adapter_id)
            evidence_count += 1
            if record.get("operation") != "episode":
                continue
            episode_contract = record.get("episode_contract")
            if not isinstance(episode_contract, Mapping):
                episode_contract_failures += 1
                continue
            episode_contract_count += 1
            if episode_contract.get("status") != "pass":
                episode_contract_failures += 1
            projection = episode_contract.get("observation_schema_projection")
            if isinstance(projection, str):
                episode_projection_counts[projection] += 1
        concrete_capability_preflight = {
            "status": "pass" if witness.get("status") == "pass" else "fail",
            "witnessed_endpoint_count": len(witness_records),
            "evidence_count": evidence_count,
            "adapter_count": len(adapter_ids),
            "feasibility_counts": dict(sorted(feasibility_counts.items())),
            "claim_boundary": (
                "This summarizes fingerprinted family-owned capability estimates obtained while compiling "
                "the exact endpoint witnesses. It does not execute a vehicle, prove truth-objective success, "
                "or promote a feasibility estimate to qualification."
            ),
        }
        episode_contract_conformance = {
            "status": "pass" if witness.get("status") == "pass" and episode_contract_failures == 0 else "fail",
            "episode_endpoint_count": sum(
                1 for record in witness_records if isinstance(record, Mapping) and record.get("operation") == "episode"
            ),
            "contract_count": episode_contract_count,
            "failure_count": episode_contract_failures,
            "observation_schema_projection_counts": dict(sorted(episode_projection_counts.items())),
            "claim_boundary": (
                "This verifies that an opened episode has no native-control bypass and that its canonical status "
                "and observation frames retain the composed interface identity. It does not establish physical "
                "effector realization, mission completion, or batch/episode trajectory parity."
            ),
        }
        if not execute_batch_witnesses:
            release_packet_conformance = {
                "status": "not_checked",
                "claim_boundary": (
                    "Endpoint construction does not generate a batch packet. Run with execute_batch_witnesses=true "
                    "to validate release-catalog conformance."
                ),
            }
        else:
            batch_record_count = 0
            release_packet_count = 0
            release_packet_failures = 0
            for record in witness_records:
                if not isinstance(record, Mapping) or record.get("operation") != "batch":
                    continue
                batch_record_count += 1
                batch_execution = record.get("batch_execution")
                if not isinstance(batch_execution, Mapping):
                    release_packet_failures += 1
                    continue
                release_packet = batch_execution.get("release_packet")
                if not isinstance(release_packet, Mapping) or release_packet.get("status") != "pass":
                    release_packet_failures += 1
                    continue
                release_packet_count += 1
            release_packet_conformance = {
                "status": "pass" if witness.get("status") == "pass" and release_packet_failures == 0 else "fail",
                "batch_endpoint_count": batch_record_count,
                "verified_packet_count": release_packet_count,
                "failure_count": release_packet_failures,
                "claim_boundary": (
                    "This verifies that each generated public batch packet can be indexed and packaged as one "
                    "hash-bound release packet with a verified reproduction identity. It does not establish "
                    "robustness, physical fidelity, or mission qualification."
                ),
            }

    graph_extension_conformance: dict[str, object]
    graph_extension_records = witness.get("graph_extension_records")
    if not isinstance(graph_extension_records, list):
        graph_extension_conformance = {
            "status": "not_checked",
            "claim_boundary": (
                "Family graph-extension witnesses are checked only while compiling the endpoint-witness matrix. "
                "Run with check_execution_witnesses=true to validate declared alternate transitions."
            ),
        }
    else:
        graph_extension_failures = 0
        observed_graph_execution_count = 0
        for record in graph_extension_records:
            if not isinstance(record, Mapping):
                graph_extension_failures += 1
                continue
            if record.get("graph_status") in {"linear_sequence_only", "authored_linear_sequence_lowered"}:
                graph_extension_failures += 1
            transitions = record.get("supported_transition_kinds")
            required_transition = record.get("required_transition_kind")
            if not isinstance(transitions, list) or required_transition not in transitions:
                graph_extension_failures += 1
            if execute_batch_witnesses:
                batch_execution = record.get("batch_execution")
                graph_execution = batch_execution.get("graph_execution") if isinstance(batch_execution, Mapping) else None
                if not isinstance(graph_execution, Mapping) or graph_execution.get("observation_status") != "observed":
                    graph_extension_failures += 1
                else:
                    observed_graph_execution_count += 1
        graph_extension_conformance = {
            "status": "pass" if witness.get("status") == "pass" and graph_extension_failures == 0 else "fail",
            "witness_count": len(graph_extension_records),
            "observed_execution_count": observed_graph_execution_count if execute_batch_witnesses else None,
            "failure_count": graph_extension_failures,
            "claim_boundary": (
                "This verifies that a family-owned composed graph declares its non-success transition and, when "
                "public batch smoke runs, emits observed dispatch evidence. It does not make a recovery outcome "
                "a successful objective result or a qualification claim."
            ),
        }

    parity_witness: dict[str, object]
    if execute_parity_witnesses:
        parity_witness = validate_vehicle_execution_parity_witnesses()
    else:
        parity_witness = {
            "status": "not_checked",
            "claim_boundary": (
                "The fast Product 3 maturity report does not replay registered batch/episode parity witnesses. "
                "Run with execute_parity_witnesses=true to verify their exact declared action traces."
            ),
        }

    result_catalog: dict[str, object]
    if results_directory is None:
        result_catalog = {
            "status": "not_supplied",
            "claim_boundary": (
                "No result directory was supplied. This maturity audit therefore makes no claim about completed mission artifacts or qualification evidence."
            ),
        }
    else:
        indexed = index_composition_results(results_directory)
        errors = indexed["errors"]
        if not isinstance(errors, list):
            raise ValueError("Product 3 result catalog has an invalid error list")
        records = indexed["records"]
        if not isinstance(records, list):
            raise ValueError("Product 3 result catalog has an invalid record list")
        action_trace_status_counts: Counter[str] = Counter()
        resource_ledger_status_counts: Counter[str] = Counter()
        reproduction_status_counts: Counter[str] = Counter()
        capability_preflight_status_counts: Counter[str] = Counter()
        capability_feasibility_counts: Counter[str] = Counter()
        graph_execution_status_counts: Counter[str] = Counter()
        graph_observation_status_counts: Counter[str] = Counter()
        for record in records:
            if not isinstance(record, Mapping):
                continue
            evidence = record.get("semantic_action_trace_evidence")
            if isinstance(evidence, Mapping) and isinstance(evidence.get("status"), str):
                action_trace_status_counts[str(evidence["status"])] += 1
            resource_ledger = record.get("resource_ledger_evidence")
            if isinstance(resource_ledger, Mapping) and isinstance(resource_ledger.get("status"), str):
                resource_ledger_status_counts[str(resource_ledger["status"])] += 1
            reproduction = record.get("reproduction_evidence")
            if isinstance(reproduction, Mapping) and isinstance(reproduction.get("status"), str):
                reproduction_status_counts[str(reproduction["status"])] += 1
            capability_preflight = record.get("capability_preflight_evidence")
            if isinstance(capability_preflight, Mapping):
                capability_status = capability_preflight.get("status")
                if isinstance(capability_status, str):
                    capability_preflight_status_counts[capability_status] += 1
                feasibility = capability_preflight.get("feasibility")
                if isinstance(feasibility, str):
                    capability_feasibility_counts[feasibility] += 1
            graph_execution = record.get("graph_execution_evidence")
            if isinstance(graph_execution, Mapping):
                graph_status = graph_execution.get("status")
                if isinstance(graph_status, str):
                    graph_execution_status_counts[graph_status] += 1
                observation_status = graph_execution.get("observation_status")
                if isinstance(observation_status, str):
                    graph_observation_status_counts[observation_status] += 1
        result_catalog = {
            "status": indexed["status"],
            "root_directory": indexed["root_directory"],
            "result_count": indexed["result_count"],
            "mission_result_count": indexed["mission_result_count"],
            "local_controller_screen_count": indexed["local_controller_screen_count"],
            "valid_result_count": indexed["valid_result_count"],
            "error_count": len(errors),
            "semantic_action_trace_status_counts": dict(sorted(action_trace_status_counts.items())),
            "resource_ledger_status_counts": dict(sorted(resource_ledger_status_counts.items())),
            "reproduction_status_counts": dict(sorted(reproduction_status_counts.items())),
            "capability_preflight_status_counts": dict(sorted(capability_preflight_status_counts.items())),
            "capability_feasibility_counts": dict(sorted(capability_feasibility_counts.items())),
            "graph_execution_status_counts": dict(sorted(graph_execution_status_counts.items())),
            "graph_observation_status_counts": dict(sorted(graph_observation_status_counts.items())),
            "claim_boundary": (
                "This is a discovery-only projection of the supplied result directory. It validates artifact "
                "identity and envelopes, but does not rerun a vehicle or promote any result to qualification."
            ),
        }

    topology_status = topology.get("status")
    semantic_preflight_handler_status = semantic_preflight_handlers.get("status")
    witness_status = witness.get("status")
    result_catalog_status = result_catalog.get("status")
    status = (
        "pass"
        if topology_status == "pass"
        and variant_admission_status == "pass"
        and semantic_preflight_handler_status == "pass"
        and integration_worklist.get("status") == "pass"
        and witness_status in {"pass", "not_checked"}
        and parity_witness.get("status") in {"pass", "not_checked"}
        and result_catalog_status in {"pass", "empty", "not_supplied"}
        else "fail"
    )
    return {
        "schema": "taoryx.product-three-maturity-report/v1alpha1",
        "status": status,
        "topology": {
            "status": topology_status,
            "family_count": topology.get("family_count"),
            "interface_channel_count": topology.get("interface_channel_count"),
            "parameter_count": topology.get("parameter_count"),
            "finding_count": topology.get("finding_count"),
            "parameter_contract_maturity": dict(parameter_maturity),
            "canonical_channel_count": len(canonical_channel_coverage),
            "resource_channel_count": len(resource_channel_coverage),
        },
        "discovery_and_authoring": {
            "vehicle_count": len(vehicles),
            "mission_tier_count": len(tiers),
            "tier_status_counts": dict(sorted(status_counts.items())),
            "graph_status_counts": dict(sorted(graph_statuses.items())),
            "capability_adapter_count": capability_adapter_count,
            "batch_action_trace_disposition_counts": dict(sorted(batch_action_trace_disposition_counts.items())),
            "execution_mode_counts": dict(sorted(execution_mode_counts.items())),
            "fidelity_promotion_blocker_counts": dict(sorted(fidelity_promotion_blocker_counts.items())),
            "planned_execution_binding_count": planned_execution_binding_count,
            "planned_execution_blocker_counts": dict(sorted(planned_execution_blocker_counts.items())),
            "variant_admission": {
                "status": variant_admission_status,
                "variant_space_status_counts": dict(sorted(variant_space_status_counts.items())),
                "runnable_variant_count": runnable_variant_count,
                "topology_runtime_backed_variant_count": topology_runtime_backed_variants,
                "claim_boundary": (
                    "This verifies that Product 3 discovery, authoring, and topology agree about declared runtime-bound "
                    "variants. It does not prove the modifier's physical coupling, trim, or qualification beyond its "
                    "separate runtime evidence."
                ),
            },
        },
        "family_integration": {
            "strategy_conformance_status": integration_worklist.get("status"),
            "readiness_status": integration_worklist.get("readiness_status"),
            "tier_count": len(integration_items),
            "work_status_counts": dict(sorted(integration_status_counts.items())),
            "next_action_counts": dict(sorted(integration_next_action_counts.items())),
            "claim_boundary": (
                "This is the reusable topology strategy worklist for current catalog families. A development, "
                "planned, or blocked tier identifies the next integration prerequisite; it is not a Product 3 "
                "catalog failure or a vehicle qualification outcome."
            ),
        },
        "execution": {
            "runnable_operation_counts": dict(sorted(operation_counts.items())),
            "endpoint_execution_mode_counts": dict(sorted(execution_endpoint_mode_counts.items())),
            "parity_disposition_counts": dict(sorted(parity_counts.items())),
            "semantic_preflight_handlers": semantic_preflight_handlers,
            "concrete_capability_preflight": concrete_capability_preflight,
            "episode_contract_conformance": episode_contract_conformance,
            "release_packet_conformance": release_packet_conformance,
            "graph_extension_conformance": graph_extension_conformance,
            "execution_witnesses": witness,
            "batch_episode_parity_witnesses": parity_witness,
        },
        "result_catalog": result_catalog,
        "claim_boundary": (
            "This joins topology, discovery, composition, capability, semantic-translator dispatch, endpoint, "
            "parity, and optionally retained result-artifact evidence. It does not average maturity across "
            "families, create an executable adapter, establish physical control realization, or promote a "
            "trajectory to qualification."
        ),
    }
    ####


__all__ = ["build_product_three_maturity_report"]
