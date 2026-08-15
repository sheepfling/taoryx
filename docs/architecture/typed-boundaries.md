# Typed boundary survey and configuration normalization

Taoryx uses maps deliberately for named numeric vectors, table rows, and
provider-defined extensions. Those are not automatically defects. The risk is
an unbounded `dict[str, Any]`, `Mapping[str, object]`, or unannotated payload
that crosses a public boundary and is consumed by another module without a
schema-owned owner.

Run the survey from the repository root:

```bash
python -m tools.audit_loose_contracts --format markdown --output qa/loose-contract-audit.md
python -m tools.audit_loose_contracts --format json --output qa/loose-contract-audit.json
```

The report ranks public parameters, returns, and model fields by whether they
are open bags, configuration-shaped, and statically observed to have
cross-module consumers. It ignores generated `build/` copies, tests, data
fixtures, and numeric maps with specific value types such as
`Mapping[str, float]`.

## Migration rule

Normalize at the earliest owner boundary, validate there, and pass a named
typed contract downstream. Do not spread ad-hoc shape checks through every
consumer.

For model authoring, the public helpers (`select_variant`,
`sequence_template`, `segment_occurrence`, and `custom_sequence`) now produce
canonical JSON-shaped projections. `ModelAuthoringDraft` applies
`normalize_authoring_values` at ingress before provider-specific compilation.
The selected provider schema remains authoritative for field names, variants,
units, ranges, and defaults; after validation the downstream handoff is a
`PreparedTrajectoryConfiguration`, not the original authoring bag.

The first high-risk migration wave also gives several shared plug-in seams a
named owner: `RacetrackBinding` validates catalog/compiler geometry and
fidelity inputs; `MeasurementPacketRecord` normalizes checkpoint and sensor
serialization; `CommittedNativeStatusValues` and `NativeActionValues` mark
immutable adapter-owned status and action maps; and
`VehicleBatchEpisodeParityAdvertisement` carries typed parity discovery while
retaining an explicit JSON projection at persistence boundaries. The public
model-authoring plan now has a `ModelAuthoringPlanProjection` envelope whose
named sections are normalized to portable JSON values before CLI/UI consumers
receive them.

Persisted policy traces are handled similarly: `CompositionPolicyTraceRecord`
is the canonical trace envelope, and `parse_composition_policy_trace_record`
copies untrusted JSON into that form before generic replay or a family-owned
batch/episode parity verifier consumes it. It validates transport shape and
identity fields without treating the persisted expected status/observation
values as newly authoritative runtime state.

## Triage order

1. Normalize configuration/authoring projections that have consumers outside
   their owner module.
2. Replace public request, action, plan, and artifact bags with Pydantic
   models, discriminated unions, or `TypedDict` envelopes when the shape is
   stable.
3. Retain genuine extension bags only behind an explicitly named field with an
   owner, validator, compatibility policy, and claim boundary.
4. Leave typed numeric maps alone unless their key vocabulary itself needs a
   named coordinate contract.

The audit is a survey, not a blanket ban on mappings: source data, table rows,
and generic JSON persistence can remain map-shaped when their producer and
schema are explicit.
