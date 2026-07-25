# Vehicle family packages

Each directory is a versioned, source-traceable vehicle-family package. Keep
identity, controls, observations, bindings, source locks, and qualification
records here; keep generated telemetry and plots under ignored `artifacts/`.

New families should follow the structure already used by the reference-anchor
packages:

```text
<family-id>/
  family.yaml
  bindings/
  plant/
  actuators/
  qualification/
```

The family package declares capabilities. Shared equations, sessions,
controllers, tables, and evidence evaluation remain in `src/taoryx/` and must
not be forked into per-vehicle runners. Candidate or not-yet-ingested source
packages belong in `verification/*_intake_v*.yaml` until they have a checked-in
family binding and maturity promotion.
