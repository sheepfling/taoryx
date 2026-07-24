# Taoryx language extension reference

This is the focused reference for successor `.prb` and `.tbl` syntax. It is
separate from the reconstructed TAOS 96 language: select the `taoryx` grammar
profile when using any construct listed here.

```python
from taoryx.language import GrammarProfile, parse_problem_file

document = parse_problem_file("mission.prb", profile=GrammarProfile.TAORYX)
```

The parser produces a lossless document, typed semantic AST nodes, and
source-located diagnostics. Grammar acceptance is not by itself a claim that a
feature is implemented by every runtime adapter.

## Extension surface

| Surface and scope | AST representation | Runtime status |
| --- | --- | --- |
| `*mode ...` (problem) | `ModeBlock` | point-mass and rigid-body modes supported; kinematic mode is adapter-dependent |
| `*3dof`, `*6dof`, `*sixdof` (problem) | `DofDirectiveBlock` | aliases for point-mass or rigid-body 6-DOF |
| `*method ...` (problem) | `ExtensionBlock` | method/options are runtime-dependent |
| `*mass ...`, `*ptmass` (segment) | `ExtensionBlock` | variable-mass and point-mass transitions |
| `*deployed ...` (trajectory) | `ExtensionBlock` | deployment initialization is supported |
| `*cases` (problem) | `ExtensionBlock` | rows retained for batch adapters |
| `surface_*` helpers (define/expressions) | typed helper or expression AST | supported helper path |
| `maxfit`, `minfit` (summaries) | summary AST function | supported summary path |
| `*atmos`, `*wind` (problem) | `AtmosBlock`, `WindBlock` | adapter, table-data, or file dependent |
| `aero_force`, `aero_moment`, `inertia` (tables) | typed table document | successor table families |
| `*runtime ...` (problem) | `RuntimeBlock` | lowers to shared runtime contract where supported |

The explicit aliases are intentionally normalized in the AST: `*3dof` maps to
`point-mass`, while `*6dof` and `*sixdof` map to `rigid-body-6dof`. Original
source spelling remains available through the lossless source records.

## Representative forms

```text
(successor-example)
*sixdof
*method rk4-fixed variable-mass
*atmos rcc ktf-annual
*wind geodetic file=wind.dat units=ft/sec

*trajectory 1 vehicle start on 1
  *initial ecic x=20925646 y=0 z=0 xdt=0 ydt=300 zdt=0 time=0 mass=100
  *segment 1 powered
    *mass xcg=0 ixx=10 iyy=11 izz=12
    *when time>1 stop

*trajectory 2 payload start on 1
  *deployed from trajectory 1, segment 1, wt=25
  *segment 1 coast
    *ptmass
    *when alt<0 stop
*end
```

```text
*define initial variables
  surface_ref(launch_lat,launch_long);
  launch_azimuth = surface_azm(target_lat,target_long);
  target_distance = surface_dist(target_lat,target_long);
*end

*cases
  case launch_lat launch_long target_lat target_long
  1 22.0 -159.8 39.5 -175.5
```

## AST and validation contract

The extension block models are defined in
[`src/taoryx/language/models.py`](../../src/taoryx/language/models.py),
dispatch and source validation are in
[`problem_parser.py`](../../src/taoryx/language/problem_parser.py), and the
reviewed keyword/table contract is in
[`grammar_contracts.py`](../../src/taoryx/language/grammar_contracts.py).
The documentary EBNF is
[`grammars/taoryx_extensions.ebnf`](../../grammars/taoryx_extensions.ebnf).

For focused syntax and negative-diagnostic coverage, run:

```bash
taoryx-validate --profile taoryx \
  examples/taoryx/syntax_fragments/*.prb \
  examples/taoryx/syntax_fragments/*.tbl
python tools/dev.py test-grammar
```

The focused fixtures are listed in
[`examples/taoryx/syntax_fragments/README.md`](../../examples/taoryx/syntax_fragments/README.md).
Larger combined cases are in
[`examples/taoryx/full_examples/`](../../examples/taoryx/full_examples/).

## End-to-end artifacts

To regenerate parser reports, normalized artifacts, telemetry, and plots for
the extension corpus:

```bash
PYTHONPATH=src .venv/bin/python examples/run_corpus.py \
  --family taoryx --execute --output artifacts/examples/taoryx
```

The shared output contract is documented in
[`examples/taoryx/output_contract.yaml`](../../examples/taoryx/output_contract.yaml).
A short run can return exit code `1` when its step budget is exhausted after
writing artifacts; parser errors and runtime exceptions are reported separately.

## Claim boundary

`taos96` remains the default profile and is the historical compatibility claim.
`taoryx` is the successor profile. The local runtime can execute the supported
subset, but this repository does not contain the historical TAOS executable and
complete table library, so these extensions must not be described as TAOS 96.0
behavioral compatibility.
