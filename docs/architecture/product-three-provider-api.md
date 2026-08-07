# Product 3 provider API

The Product 3 provider is the plug-in boundary between a host application and
a vehicle/trajectory implementation. The host should be able to discover a
provider, display its usable models and inputs, compose an ordered mission,
and receive a standard trajectory without importing provider-specific model
classes.

This contract is intentionally above the lower-level Alpha 2
`TrajectoryProvider` session API. A Product 3 provider may wrap that API, the
existing vehicle-composition runtime, or another source-owned executor. It
must not use the contract to hide a missing plant, controller, resource model,
or qualification record.

The checked-in reference implementation is
[`taoryx.trajectory.product_three`](../../src/taoryx/trajectory/product_three.py),
with a runnable example in
[`examples/trajectory_provider/product_three_reference.py`](../../examples/trajectory_provider/product_three_reference.py).

## Lifecycle

```text
provider metadata
      │
      ▼
host selects provider / vehicle / fidelity
      │
      ▼
prepare(request)
  validate IDs, units, values, defaults,
  capability/fidelity compatibility, and segment graph
      │
      ▼
prepared request + SHA-256 identity
      │
      ▼
run(prepared)
      │
      ▼
taoryx.product-three-trajectory/v1
```

`prepare` has no execution side effects. It is the UI, optimizer, or batch
planner's safe validation boundary. `run` accepts only a prepared request and
must return the same result envelope for every provider.

## Provider publication

The Python surface is:

```python
class ProductThreeProvider(Protocol):
    @property
    def metadata(self) -> ProductThreeProviderMetadata: ...

    def prepare(
        self, request: ProductThreeTrajectoryRequest
    ) -> ProductThreePreparedRequest: ...

    def run(
        self, prepared: ProductThreePreparedRequest
    ) -> ProductThreeTrajectory: ...
```

The metadata publication has schema ID
`taoryx.product-three-provider/v1` and contains:

| Layer | Required information |
| --- | --- |
| Provider | stable ID, display name, version, API version, status, provenance, supported output schema, claim boundary, batch/prepare/step support |
| Vehicle | stable vehicle/model ID, version, model kind, status, description, supported fidelity IDs, provenance |
| Capability | stable ID, kind, native/emulated/approximated status, compatible fidelities, supported operations, segment IDs, provenance |
| Initialization | stable ID, compatible fidelities, parameter descriptors, provenance |
| Segment | stable ID and kind, compatible fidelities, parameter descriptors, entry/repetition/terminal rules, allowed next segment IDs, provenance |
| Channel | stable ID, canonical unit, frame, description, provenance |

A compact publication looks like this:

```json
{
  "schema": "taoryx.product-three-provider/v1",
  "provider_id": "taoryx.example.product3",
  "version": "0.1.0",
  "supports_batch": true,
  "vehicles": [
    {
      "vehicle_id": "reference_guided_point_mass",
      "model_kind": "guided_point_mass_3dof",
      "fidelities": ["point_mass_3dof"],
      "capabilities": [
        {
          "id": "waypoint-guidance",
          "kind": "guidance",
          "status": "native",
          "fidelities": ["point_mass_3dof"],
          "operations": ["prepare", "batch"],
          "segment_ids": ["waypoint_leg"]
        }
      ],
      "segments": [
        {
          "id": "waypoint_leg",
          "kind": "waypoint",
          "repeatable": true,
          "allowed_next": ["waypoint_leg", "bank_maneuver", "coast"],
          "parameters": [
            {"id": "waypoint_north_m", "value_type": "number", "canonical_unit": "m", "required": true, "role": "segment"},
            {"id": "max_load_factor_g", "value_type": "number", "canonical_unit": "g0", "minimum": 1.0, "maximum": 6.0, "default": 2.5, "default_declared": true, "role": "constraint"}
          ]
        }
      ]
    }
  ]
}
```

The publication is a capability declaration, not an execution or
qualification result. `status: runnable` means that this plug-in has a
registered executor for the advertised contract; it does not make an
unverified vehicle family runnable elsewhere in the repository.

## Parameter contract

Every public input is a `ProductThreeParameter` with:

- a stable ID and human-readable description;
- an explicit representation (`number`, `integer`, `boolean`, `string`, or
  `enum`);
- a canonical unit, or explicit unitlessness;
- required/default disposition;
- hard mathematical bounds;
- optional narrower qualified bounds;
- enum choices where applicable;
- semantic role (`initialization`, `segment`, or `constraint`);
- optional frame; and
- provenance.

The request carries `{ "value": ..., "unit": ... }` for each value. The
reference contract accepts only the advertised canonical unit. It does not
silently convert metres to kilometres, reinterpret signed coordinates as
magnitudes, or infer a topology from an identifier. A future shared unit
service can be inserted before `prepare`; until then, exact-unit rejection is
the safer boundary.

`minimum`/`maximum` are validity limits. `qualified_minimum`/
`qualified_maximum` describe evidence coverage and are not used to silently
project a caller's request. If a host wants projection for search, that must
be an explicitly advertised policy rather than a side effect of execution.

For example, `max_load_factor_g` is a segment `constraint` parameter. It
means “the executor may not command a maneuver above this limit.” The result's
`maneuver.load_factor_g` channel is the observed/applied value. The provider
must not report the requested limit as achieved load-factor evidence.

## Variable-length segment sequences

The request contains an ordered tuple of segment occurrences:

```json
{
  "schema": "taoryx.product-three-request/v1",
  "request_id": "waypoint-course-01",
  "vehicle_id": "reference_guided_point_mass",
  "fidelity": "point_mass_3dof",
  "initialization_id": "airborne_state",
  "initialization": {
    "altitude_m": {"value": 1000.0, "unit": "m"},
    "speed_m_s": {"value": 100.0, "unit": "m/s"},
    "heading_deg": {"value": 90.0, "unit": "deg"}
  },
  "segments": [
    {
      "id": "waypoint_leg",
      "instance_id": "wp-1",
      "parameters": {
        "duration_s": {"value": 20.0, "unit": "s"},
        "waypoint_north_m": {"value": 1000.0, "unit": "m"},
        "waypoint_east_m": {"value": 0.0, "unit": "m"},
        "waypoint_altitude_m": {"value": 1100.0, "unit": "m"},
        "max_load_factor_g": {"value": 2.5, "unit": "g0"}
      }
    },
    {
      "id": "waypoint_leg",
      "instance_id": "wp-2",
      "parameters": {
        "duration_s": {"value": 20.0, "unit": "s"},
        "waypoint_north_m": {"value": 1000.0, "unit": "m"},
        "waypoint_east_m": {"value": 1000.0, "unit": "m"},
        "waypoint_altitude_m": {"value": 1100.0, "unit": "m"}
      }
    }
  ],
  "output": {"cadence_s": 0.1}
}
```

Preparation checks:

1. the vehicle and fidelity exist and are runnable;
2. the initialization exists and is compatible with the fidelity;
3. required inputs are present, optional defaults are declared, and no
   unknown inputs are accepted;
4. units, types, choices, and bounds are valid;
5. each segment exists and is fidelity-compatible;
6. the first segment has `entry_allowed: true`;
7. every following segment is in the previous descriptor's `allowed_next`
   set when that set is declared;
8. repeatability, occurrence limits, and instance-ID uniqueness hold; and
9. requested output channels exist.

This gives a UI enough information to render a sequence editor and a useful
error before any model is loaded. A segment's `max_load_factor_g`, waypoint
coordinates, target heading, duration, and other per-occurrence values stay
with that occurrence; they are not global mutable provider state.

## Standard trajectory result

`run` returns `taoryx.product-three-trajectory/v1`:

```json
{
  "schema": "taoryx.product-three-trajectory/v1",
  "provider_id": "taoryx.example.product3",
  "provider_version": "0.1.0",
  "request_id": "waypoint-course-01",
  "request_fingerprint": "<sha256>",
  "vehicle_id": "reference_guided_point_mass",
  "fidelity": "point_mass_3dof",
  "status": "completed",
  "channel_units": {
    "position.north_m": "m",
    "position.east_m": "m",
    "maneuver.load_factor_g": "g0"
  },
  "samples": [
    {"time_s": 0.0, "segment_instance_id": null, "values": {"position.north_m": 0.0, "position.east_m": 0.0, "maneuver.load_factor_g": 1.0}},
    {"time_s": 0.1, "segment_instance_id": "wp-1", "values": {"position.north_m": 9.9, "position.east_m": 0.2, "maneuver.load_factor_g": 1.4}}
  ],
  "segments": [
    {"id": "waypoint_leg", "instance_id": "wp-1", "start_time_s": 0.0, "end_time_s": 20.0, "status": "completed"}
  ],
  "events": [
    {"time_s": 20.0, "kind": "waypoint_captured", "segment_instance_id": "wp-1"}
  ],
  "diagnostics": [],
  "claim_boundary": "..."
}
```

The standard envelope carries accepted times, numeric channels with units,
segment spans, discrete events, diagnostics, the exact request fingerprint,
and the provider's claim boundary. It intentionally does not prescribe the
provider's internal state vector, controller class, file layout, or physics
implementation. A provider may add richer sidecars in its own artifact
directory, but the common envelope remains sufficient for a host to plot,
index, compare, and reproduce a run.

The `events` list is not a substitute for truth. A waypoint-capture event
should only be emitted when the provider's evaluator has actually checked the
declared tolerance against a committed sample. Likewise, requested limits
and achieved channels remain separate.

## Errors and installation

The reference implementation raises `ProductThreeError` with a stable code
and optional field. Current codes include `unknown-provider`,
`unknown-vehicle`, `unsupported-fidelity`, `missing-parameter`,
`unknown-parameter`, `unit-mismatch`, `invalid-value`, `invalid-choice`,
`out-of-bounds`, `invalid-sequence`, `segment-not-repeatable`,
`segment-occurrence-limit`, `unknown-channel`, and `provider-mismatch`.

The host-side `ProductThreeProviderRegistry` is explicit:

```python
from taoryx.trajectory import ExampleProductThreeProvider, ProductThreeProviderRegistry

registry = ProductThreeProviderRegistry((ExampleProductThreeProvider(),))
publication = registry.catalog()
provider = registry.provider("taoryx.example.product3")
prepared = provider.prepare(request)
trajectory = provider.run(prepared)
```

This first implementation uses explicit registration so installation and
provenance are visible in tests and examples. A production host can discover
the same protocol through its package/plugin loader, but it should still
validate the returned metadata and reject duplicate provider IDs before
publishing a catalog.

## Relationship to the repository's existing Product 3 registry

The existing `vehicle_composition_registry.yaml` remains the repository's
canonical catalog for the checked-in TAORYX vehicle families, fidelity tiers,
mission templates, and source-owned execution factories. This provider API is
the portable plug-in shape a host can use at that boundary. A future adapter
can project a validated composition-registry entry into
`ProductThreeProviderMetadata`; it must preserve the existing fail-closed
status, topology, execution-mode, and claim-boundary rules. The analytical
example intentionally does not pretend to be such an adapter.
