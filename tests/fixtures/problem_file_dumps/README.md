# Problem File Dumps

This directory is a staging area for problem-file dumps that are too broad or
too early to promote directly into the main regression corpus.

Use it for grouped source material that still needs curation, slicing, or
deduplication before it becomes a test fixture.

## Current leaf areas

- `spectre_segments/` - initial Spectre problem-file segment dump area
- `spectre_trajectories/` - full Spectre-like trajectory working set

The Spectre leaf is being organized around the observed simple-aero family
set from `INBOX/` and can later be split into phase-specific subfixtures such
as boost, coast, maneuver, and final pronav. That phase split is an inferred
organization aid, not a rendered TAOS runtime model yet.

The working rule is to keep each dump self-describing, provenance-aware, and
small enough to be split into reusable fixtures later.
