# Claim registry

```yaml
claims:
  - id: D
    name: documentary_fidelity
    scope: reconstructed 1995 manual edition
    excludes: executable behavior not printed by the source
  - id: S
    name: semantic_coherence
    scope: adopted equations, constants, defaults, restrictions
    excludes: unresolved Class D ambiguities in normal execution
  - id: P
    name: parser_conformance
    scope: explicitly evidenced .tbl/.prb constructs
    excludes: undocumented syntax and incomplete manual excerpts treated as full files
  - id: N
    name: numerical_correctness
    scope: individually benchmarked mathematical kernels
    excludes: historical iteration sequence or vehicle-table completeness
  - id: H
    name: historical_compatibility
    scope: TAOS 96.0 behavior
    evidence_required: historical executable, source, or trusted output corpus
    status: not_established

The numerical-verification work in this repository is scoped to TAOS 1995.
TAOS 96.0 compatibility remains a separate claim and must not be implied by
manual fidelity or by passing unit tests alone.
```

The status of a claim is determined per release and per scope. A passing unit
test may support a requirement; it does not promote the whole claim to verified.

Current powered fixed-wing evidence boundary:

- B747: a source-anchor 120-second powered trim hold, 60-second
  approach/go-around sequence, a generated 120-second native descent/recovery
  maneuver, source-deck reduction, and bounded controller probe are evidenced.
  This is a local public research-surrogate claim; heading capture, full
  altitude-step tracking, and the long route controller remain incomplete.
- Skywalker X8: source-differential static/collective/differential/thrust
  tables, a source-composed 30-second powered trim hold with all three body
  moments closed at the initial state, a source-neighborhood 120-second level-settling corridor, a
  metadata-generated 10-second powered controller recovery, a 60-second
  powered validation, and a native single-waypoint capture are evidenced. The
  four-leg rectangle is still only a bounded directional route: its fixed-
  corner boundary errors remain outside the declared capture gate. The current
  fixture uses a 30-second generic corner-blend window; the strict capture
  result remains provisional until a fresh packet records all four boundary
  errors.
- Neither vehicle has a global flight-envelope, flight-qualified, or
  historical TAOS compatibility claim.
- The X8 surface-authority route is not yet a controller-success claim: its
  current pitch tuning exits the source alpha envelope, and the strict
  aerodynamic boundary records that failure before numerical overflow.

Current four-family fidelity boundary:

- X-15: the static source grid and the symmetric-stabilator,
  differential-stabilator, and rudder control slices reproduce their checked-in
  source CSV grids to approximately `1e-11` absolute coefficient error. The
  source-anchored rigid-body plant, 30-second source-envelope rudder-trim hold,
  and a five-second native ProNav segment following that trim are evidenced;
  the short native ProNav case remains blocked when the trajectory leaves the
  declared beta envelope.
- Hummingbird: the direct-wrench source grid, open-loop hover, waypoint route,
  return-home, landing-order, and common-controller-manifest cases are
  evidenced as bounded research-model behavior. This is not a hardware or
  flight-qualification claim; the generic external attitude/actuator contract
  remains separate and blocked.
- The common fidelity ladder proves the point-mass, kinematic bridge, and
  rigid-body problem-file paths execute for all four families. It does not make
  the kinematic bridge a substitute for independent rigid-body controller
  evidence.

####
