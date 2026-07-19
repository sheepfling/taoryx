# Reusable problem-file fragments

Fragments are external composition inputs, not new TAOS grammar.

They use the `.prbfrag` suffix and contain ordinary problem-file blocks. A
template may reference them with `{{FRAGMENT:name}}`; scalar values use
`{{NAME}}`. Compose a complete, inspectable `.prb` with:

```bash
python tools/compose_problem.py template.prb mission.prb \
  --fragment terminal=fragments/guidance/terminal-propnav.prbfrag \
  --set TARGET_LAT=21.31
```

The output must be parsed and validated like any hand-written problem file.
Fragments must not introduce `*include`, function definitions, or other new
in-language syntax. Keep TAOS96-compatible fragments free of TAORYX-only
blocks.

## LQR controller alternatives

The `controllers/` fragments are TAORYX-extension alternatives for existing
3-DOF tracking, 6-DOF attitude, and quadrotor hover scenarios:

- `attitude-lqr-6dof.prbfrag`: full body-attitude/rate loop;
- `point-mass-track-lqr.prbfrag`: local point-mass tracking loop;
- `quadrotor-hover-lqr-6dof.prbfrag`: cascaded position and attitude loops.

They declare state/control contracts and matrix-source names for the LQR
adapter. The `attitude-lqr-6dof` declaration is wired by the native rigid-body
lowering when composed into a routed 6-DOF problem. The point-mass and
quadrotor fragments remain declaration-only until their model-specific
linearization and actuator adapters are connected. No fragment silently
replaces pursuit, thermal, or rotor-allocation logic.
