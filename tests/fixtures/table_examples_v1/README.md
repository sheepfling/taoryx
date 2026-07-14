# Table Examples Workspace

This directory is a staging area for example `.tbl` data files.

It is intentionally separate from the reviewed TAOS application fixtures so we
can accumulate new table families, synthetic decks, and source-derived examples
without changing the existing regression corpus.

## Table families currently in scope

TAORYX currently recognizes these legacy TAOS table types:

- aerodynamic coefficients: `ca`, `cn`, `cl`, `cd`, `cs`, `cx`, `cy`, `cz`
- propulsion: `thrust`, `tvec1`, `tvec2`, `mdot`
- mass properties: `cg`
- wind: `windv`, `windh`, `winde`, `windn`, `windd`
- outputs: `output`

The examples in this workspace should stay explicit about provenance:

- `synthetic` for software-development decks
- `source-faithful` for transcribed historical tables
- `generated` for tables derived from a documented external model

## Seeded examples

- `propulsion/constant-thrust.tbl` and `propulsion/burn-mdot.tbl` are the
  minimal rocket propulsion pair.
- `propulsion/air_breathing/*.tbl` demonstrate Mach/altitude/throttle engine
  decks for air-breathing propulsion.
- `aerodynamics/*.tbl` demonstrate control-surface coefficient increments.
- `mass/*.tbl` demonstrate staging-related mass and CG histories.
- `output/*.tbl` provides a generic user-defined output example.

## Problem harnesses

The `harnesses/` subdirectory contains small `.prb` files that reference the
example tables directly. They are used by the ingest tests to verify that the
table families remain valid when consumed as problem inputs.

## Working rule

Add new example tables here first. If an example later becomes part of the
canonical regression suite, promote it deliberately and leave the original
here as research provenance unless the fixture is intentionally moved.
