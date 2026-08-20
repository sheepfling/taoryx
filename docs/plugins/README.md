# Plug-in documentation directory

The package README is the canonical landing page for every separately
installable TAORYX distribution. Package-specific architecture, source-data,
and witness notes live under that package's `docs/` directory when needed.
Shared contracts are intentionally not repeated here; see the
[API reference](../api/README.md).

| Distribution | Package-owned documentation | Scope |
| --- | --- | --- |
| `taoryx-a320` | [README](../../packages/taoryx-a320/README.md) | A320/OpenAP family |
| `taoryx-cadac` | [README](../../packages/taoryx-cadac/README.md), [package docs](../../packages/taoryx-cadac/docs/README.md) | Source-bound CADAC catalog and adapters |
| `taoryx-daveml` | [README](../../packages/taoryx-daveml/README.md) | DAVE-ML integration |
| `taoryx-debug-models` | [README](../../packages/taoryx-debug-models/README.md) | Development/test providers |
| `taoryx-dual-launch` | [README](../../packages/taoryx-dual-launch/README.md) | Dual-launch topology |
| `taoryx-f16` | [README](../../packages/taoryx-f16/README.md) | F-16 family |
| `taoryx-hl20` | [README](../../packages/taoryx-hl20/README.md) | HL-20 family |
| `taoryx-hummingbird` | [README](../../packages/taoryx-hummingbird/README.md) | Hummingbird multirotor family |
| `taoryx-nesc` | [README](../../packages/taoryx-nesc/README.md) | NESC source model integration |
| `taoryx-parametric-interceptors` | [README](../../packages/taoryx-parametric-interceptors/README.md), [package docs](../../packages/taoryx-parametric-interceptors/docs/README.md) | Evidence-aware interceptor surrogates |
| `taoryx-passive-bodies` | [README](../../packages/taoryx-passive-bodies/README.md) | Passive/tumbling-body models |
| `taoryx-reachability` | [README](../../packages/taoryx-reachability/README.md) | Reachability overlays and analysis |
| `taoryx-reference-models` | [README](../../packages/taoryx-reference-models/README.md) | Compatibility/reference catalog |
| `taoryx-simple-aero` | [README](../../packages/taoryx-simple-aero/README.md) | Simple Aero workflow models |
| `taoryx-source-table-fixed-wing` | [README](../../packages/taoryx-source-table-fixed-wing/README.md) | X8/B747 source-table fixed-wing models |
| `taoryx-x15` | [README](../../packages/taoryx-x15/README.md) | X-15 family |
| `taoryx-trajectory-contracts` | [README](../../packages/taoryx-trajectory-contracts/README.md) | Standalone external provider/host contract |

## Contributor rule

When adding a distribution, add its README row here and keep model-specific
documentation with the package. Its focused verification route belongs in
`python tools/dev.py plugin-focus <wheel-selector>` and must be covered by the
tooling-entrypoint tests.
