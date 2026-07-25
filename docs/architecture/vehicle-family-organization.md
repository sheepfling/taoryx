# Vehicle-family organization and maturity policy

This repository supports many vehicle archetypes, but they must share one
runtime and one evidence path. Vehicle data, family bindings, reusable physics,
segments, and verification artifacts have different ownership and should not
be mixed in one large module or one bespoke problem-file runner.

## Repository layout

```text
families/<family-id>/
  family.yaml                  # identity, fidelities, sources, maturity
  parameters.yaml              # resolved parameter declarations
  controls.yaml                # requested and achieved control schema
  observations.yaml            # truth, sensed, resource, event channels
  segments.yaml                # reusable phase and transition contracts
  qualification/
    integration-record.yaml
    scenarios.yaml

src/taoryx/
  contracts/                   # neutral units, frames, state/control schemas
  catalog/                     # family/provider discovery and resolution
  trajectory/                  # neutral sessions, fidelity, authority, adapters
  simulation/                  # canonical transition and run/step lifecycle
  runtime/                     # Taoryx execution, tables, events, artifacts
  equations/                   # source-linked equation implementations
  physics/                     # shared atmosphere, gravity, aero, propulsion,
                               # rigid-body, rotorcraft, and resource services
  families/                    # family bindings only; no second simulator
    fixed_wing/
    rotorcraft/
    tiltrotor/
    spacecraft/
    surrogates/

tests/families/<family-id>/
  test_contract.py              # schema and onboarding checks
  test_conventions.py           # units, frames, signs, limits
  test_plant.py                 # source differential, trim, closure
  test_missions.py              # family-appropriate trajectories
  fixtures/                     # small checked-in inputs only

verification/
  *_intake_v*.yaml               # candidate source and evidence records
  vehicle_maturity_registry.yaml # current maturity/status ledger
  alpha*_post_release_backlog.yaml

artifacts/                      # ignored generated evidence only
```

The current top-level modules are retained during the migration. New family
implementations should use the target package boundaries; moving existing
modules is a separate refactor and must preserve imports and generated
artifacts.

## Family package rule

Every new vehicle enters through the same sequence:

```text
source intake
  -> family.yaml and integration record
  -> catalog registration
  -> convention firewall
  -> fidelity provider binding
  -> trim and source differential checks
  -> common scenario/evaluator
  -> reusable segments and controller presets
  -> maturity promotion
```

A `.prb` file is a generated or authored scenario representation. It is not a
vehicle definition database and must not become the place where mass, inertia,
control signs, or source provenance are re-hard-coded.

## Fidelity organization

Fidelity is a per-family capability, not a universal promise:

- `point_mass_3dof`: translation and resource state with declared attitude or
  aerodynamic assumptions.
- `pseudo_6dof`: explicit attitude/rate or scheduled response states without
  claiming a complete moment/inertia model.
- `rigid_body_6dof`: quaternion, body rates, mass properties, moments,
  actuators, allocation, and physical control effectors.

Each provider declares supported controls and observations. A missing fidelity
must be rejected at resolution, not approximated silently.

## Maturity labels

The maturity registry distinguishes software readiness from scientific or
historical truth:

| Level | Meaning |
| --- | --- |
| M0 | Cataloged intake; identity and evidence boundary exist. |
| M1 | Executable canonical case reaches declared finality. |
| M2 | Units, frames, states, data coverage, and numerical coherence pass. |
| M3 | Controls, stepping, telemetry, events, and batch/step parity work. |
| M4 | Composition, segments, objectives, and transitions are configurable. |
| M5 | All applicable qualification dimensions pass and no model-code edits are needed for normal use. |
| M6 | Broad envelope, correlation, uncertainty, and maintenance evidence exist. |

Evidence strength is recorded separately (`source`, `identified`, `derived`,
`estimated`, `synthetic`, or `unavailable`). A surrogate may reach M4 for
controls research while remaining scientifically unvalidated.

## What belongs where

| Concern | Owner |
| --- | --- |
| Units, frames, state/control schemas | `contracts/` |
| Family identity and source hashes | `families/` + `verification/*intake*` |
| Shared EOM and force services | `equations/`, `physics/`, or existing shared runtime modules |
| Family-specific coefficient mapping and allocation | `src/taoryx/families/<kind>/` |
| Mission composition and mode transitions | family `segments.yaml` plus `trajectory/` |
| Metrics, gates, and score rollups | common scenario/evaluation substrate |
| Raw/source-derived tables | checked-in fixture/data packages with manifests |
| Generated plots, telemetry, and reports | ignored `artifacts/` |

This keeps the backlog extensible without allowing a new family to create a
parallel control contract, simulator loop, or evidence vocabulary.

Before a family is implemented, its intake should pass the
[future-family interface stress test](../plan/future-family-interface-stress-test.md).
This makes future requirements visible early while keeping Alpha 2's runtime
scope bounded.
