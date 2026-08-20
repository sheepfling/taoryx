# Parametric interceptor package documentation

This directory contains the design notes that belong specifically to
`taoryx-parametric-interceptors`. Start with the [quickstart](quickstart.md)
for the shortest profile-to-provider path, then use the [package README](../README.md)
for the complete profile-first authoring workflow and CLI.

- [Quickstart](quickstart.md) — app-local profile, maintained-package model,
  focused verification, and standard-default handoff.
- [Model architecture](model-architecture.md) — evidence-aware resolution,
  assumptions, source-native channels, and the standard ECEF projection.

For common discovery, batch/step, control, and output semantics, use the
repository [API reference](../../../docs/api/README.md). For package-local
commands, run `python tools/dev.py plugin-focus parametric-interceptors`.
