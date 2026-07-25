# TAOS 96.0 evidence-bounded compatibility profile

## Release position

Taoryx can plant a clear flag without possessing the historical TAOS 96.0
executable:

> Taoryx provides an evidence-bounded TAOS 96.0 language and specification
> profile, together with a separately labeled successor runtime.

This is a stronger and more accurate claim than either “fully TAOS compatible”
or “nothing can be claimed without the original binary.”

The historical reference is SAND95-1652, printed in December 1995, whose title
identifies the software as TAOS Version 96.0. The manual is authoritative for
what it prints. It is not an oracle for every behavior of the unavailable
executable and complete historical table library.

## Claims supported by this profile

### Documentary fidelity

The reconstructed manual, equation records, source-page references, editorial
notes, and generated provenance artifacts describe what the printed source
contains. The source scan remains authoritative over OCR, extracted text, and
reconstructed LaTeX.

### Bounded parser conformance

The `.tbl` and `.prb` parser accepts documented constructs represented by the
verified corpus and reports source-located diagnostics for malformed,
unsupported, or ambiguous forms. Corpus coverage is evidence-bounded; it is
not a claim that every undocumented historical extension has been recovered.

### Semantic traceability

Documented equations, constants, frames, defaults, restrictions, and
ambiguities are represented in source-linked registries and connected to tests
where their behavior is implemented. An equation transcription or parser
success alone does not promote an entire runtime feature to verified status.

### Independently checked numerical kernels

The repository can make numerical claims about individually benchmarked
equations, coordinate transforms, atmosphere/gravity models, table operations,
and successor runtime cases. These are Taoryx numerical results, even when the
equations originate in the manual.

## Explicit nonclaim

Taoryx does **not** claim historical runtime equivalence. That would require
comparison against at least one of:

- the TAOS 96.0 executable;
- original source code plus the complete vehicle-table library;
- a trusted corpus of historical intermediate and final outputs sufficient to
  distinguish defaults, interpolation, event ordering, convergence, and
  failure behavior.

The unavailable oracle leaves these questions unresolved:

- undocumented defaults and precedence;
- table interpolation and extrapolation details;
- event and segment ordering;
- guidance, search, and optimization iteration behavior;
- numerical tolerances and rounding;
- exact failure behavior;
- historical vehicle-table contents;
- intermediate-state and output-format equivalence.

No amount of modern unit testing can silently turn those unknowns into a
historical-equivalence claim.

## Profile separation

The `taos96` profile means the evidence-bounded historical language and
specification surface. The `taoryx` profile is the successor surface. Taoryx
extensions include pseudo-6DOF, rigid-body 6DOF, interactive stepping,
composable segments, and modern controller or optimizer adapters. They must be
marked as successor features rather than described as historical TAOS syntax or
behavior.

The machine-readable source of truth is
[`verification/taos96_compatibility_profile.yaml`](../../verification/taos96_compatibility_profile.yaml).

## Plantable release wording

Use this wording in release notes and evidence packets:

> Taoryx implements an evidence-bounded, lossless TAOS 96.0 language and
> specification profile based on SAND95-1652, with source-located parser,
> equation, convention, and diagnostic evidence. It also provides a clearly
> labeled successor runtime. Historical executable behavior and complete
> vehicle-library equivalence are not claimed because the original oracle is
> unavailable.

Do not shorten this to “TAOS 96 compatible.”
