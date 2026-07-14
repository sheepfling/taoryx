# Table Example Runtime Harnesses

This directory contains execution-oriented `.prb` harnesses for the example
tables under `tests/fixtures/table_examples_v1/`.

These harnesses are meant to do more than parse cleanly. They are used to
exercise runtime lowering and execution paths for the table families that are
already represented in the fixture workspace.

The current set includes simple drag, mass, rocket burn, wind, air-breathing,
and control-surface decks so that both scalar lookups and multidimensional
runtime interpolation stay covered by the test suite.
