# Simple Aero Segment Problem-File Dump

This is the initial Simple Aero leaf under the problem-file dump workspace.

The checked-in translated fixtures are generated from
[`spec.yaml`](spec.yaml). The source seed corpus is in
[`../../simple_aero_v1/`](../../simple_aero_v1/), which holds a
compact set of maneuver-family examples with a shared scaffold.

The higher-level phase index lives in [`catalog.yaml`](catalog.yaml). It groups
the families into inferred `boost`, `coast`, `maneuver`, and `final pronav`
bins so the dump can be assembled programmatically from a shared phase map.

The staged Simple Aero context adds the segment and solution notes, the
canonical `example_simple_aero_config.yaml`, and the simple-aero
dispatcher wiring. Taken together, those sources point to an observed family
set of ballistic, CBCR, crossrange, MARV, phugoid, range extension, skip,
slalom, and weave examples.

For organization, we treat the Simple Aero leaf as a staged translation area with
inferred phase buckets:

- `boost` for thrust-driven launch/burn segments
- `coast` for ballistic or passive flight segments
- `maneuver` for the family-specific guidance and steering examples
- `final pronav` for future proportional-navigation-style intercept segments

Those buckets are for fixture curation and generation planning. They are not
yet encoded as a TAOS runtime grammar feature.

Use this area for future Simple Aero problem-file material that needs to stay
grouped by segment family before it is split into individual TAORYX fixtures.
