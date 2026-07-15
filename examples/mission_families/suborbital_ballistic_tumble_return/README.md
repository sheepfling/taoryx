# Suborbital ballistic/tumbling atmospheric return

Status: scaffolded and executable from the independent `analysis/tumbling`
studies.

The analysis already supplies full-angle drag-area models and generated table
inputs. The current fixture connects one provenance-tracked generated dataset
to a suborbital launch and return mission and asserts that the ballistic return
remains finite and reaches the atmosphere/ground event.

The source-to-artifact hashes are recorded in `provenance.yaml`; changing the
analysis model or generated table requires regenerating and reviewing that
manifest.

The default model is an averaged 3-DOF tumble representation. A six-degree-
of-freedom moment model is a separate future claim and must not be implied by
this fixture.
